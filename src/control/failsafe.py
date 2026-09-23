"""Управляемое удержание FC после CH7 или верхнего положения CH6.

RPI передаёт в Betaflight только RC/setpoint-кадры. PWM моторов, PID и
внутренняя стабилизация остаются внутри FC.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from src.protocols.betaflight_msp_link import SensorSample
from src.receiver.crsf import rebuild_rc_frame, unpack_channels


class FailsafeState(str, Enum):
    """Состояния RPI после потери связи с пилотом."""

    LIVE = "LIVE"
    LEVELING = "LEVELING"
    TURNING = "TURNING"
    HOLDING = "HOLDING"
    FAULT = "FAULT"


@dataclass(frozen=True)
class FailsafeConfig:
    """Параметры удержания высоты, горизонта и разворота."""

    roll_channel: int
    pitch_channel: int
    throttle_channel: int
    yaw_channel: int
    rc_center: int
    rc_min: int
    rc_max: int
    target_roll_deg: float
    target_pitch_deg: float
    correction_per_degree: float
    max_correction: int
    sensor_max_age_s: float
    fault_action: str
    disarm_channel: int
    disarm_value: int
    stabilization_channel: int | None
    stabilization_value: int
    level_roll_tolerance_deg: float = 5.0
    level_pitch_tolerance_deg: float = 5.0
    level_hold_s: float = 1.0
    level_throttle: int = 992
    altitude_hold_gain: float = 80.0
    altitude_hold_integral_gain: float = 12.0
    altitude_hold_vario_gain: float = 35.0
    max_altitude_correction: int = 120
    max_altitude_integral_correction: int = 80
    turn_enabled: bool = True
    turn_degrees: float = 180.0
    turn_yaw_command: int = 1155
    takeover_throttle_step_per_s: float = 50.0
    climb_guard_altitude_error_m: float = 0.20
    climb_guard_vario_m_s: float = 0.50
    climb_guard_max_throttle: int = 850
    target_yaw_deadband_deg: float = 2.0
    target_yaw_correction_per_degree: float = 18.0
    target_yaw_max_correction: int = 250


@dataclass(frozen=True)
class FailsafeResult:
    """Результат формирования очередного RC-кадра."""

    output_frame: bytes
    state: FailsafeState
    events: tuple[str, ...]
    roll_command: int
    pitch_command: int
    throttle_command: int
    yaw_command: int
    disarm_requested: bool = False


class FailsafeController:
    """Удерживает высоту/горизонт, а в TARGET_CONTROL дополнительно ведёт yaw."""

    def __init__(self, config: FailsafeConfig) -> None:
        """Создаёт контроллер в обычном режиме LIVE."""
        self.config = config
        self.state = FailsafeState.LIVE
        self.last_sensor: SensorSample | None = None
        self.fault_frame: bytes | None = None
        self.hold_altitude_m: float | None = None
        self.altitude_zero_m: float | None = None
        self.armed_latched = False
        self.altitude_integral_m_s = 0.0
        self.last_hold_time: float | None = None
        self.takeover_entry_throttle: int | None = None
        self.last_takeover_throttle_time: float | None = None
        self.level_stable_since: float | None = None
        self.level_was_ok: bool | None = None
        self.turn_started_at: float | None = None
        self.turn_last_yaw_deg: float | None = None
        self.turn_progress_deg = 0.0
        self.turn_completed = False
        self.climb_guard_active = False

    def process(
        self,
        frame: bytes,
        channels: tuple[int, ...],
        takeover_state: str,
        sensor: SensorSample | None,
        now: float,
        armed: bool = True,
        target_mode: bool = False,
        target_yaw_error_deg: float | None = None,
    ) -> FailsafeResult:
        """Удерживает аппарат после CH7 или возвращает исходный кадр в LIVE."""
        cfg = self.config
        output_channels = list(unpack_channels(frame[3:-1]))
        events: list[str] = []

        if not armed:
            self._reset_runtime(clear_arm_reference=True)
            return self._result(frame, output_channels, events, FailsafeState.LIVE)

        if sensor is not None and sensor.is_fresh(now, cfg.sensor_max_age_s):
            self.last_sensor = sensor
            # Нулевая точка фиксируется только при переходе DISARM -> ARM.
            # Во время LIVE и TAKEOVER она больше не пересчитывается.
            if not self.armed_latched and sensor.altitude_m is not None:
                self.altitude_zero_m = sensor.altitude_m
                self.armed_latched = True
                events.append(f"ALTITUDE_ZERO={self.altitude_zero_m:.2f}m")

        if takeover_state == "LIVE":
            if self.state is not FailsafeState.LIVE:
                events.append(f"{self.state.value} -> LIVE")
            # Возврат связи не является DISARM: опорная высота текущего ARM
            # должна сохраниться для следующего CH7-перехвата.
            self._reset_runtime(clear_arm_reference=False)
            return self._result(frame, output_channels, events, FailsafeState.LIVE)

        if self.state is FailsafeState.LIVE:
            self.state = FailsafeState.LEVELING
            self.takeover_entry_throttle = output_channels[cfg.throttle_channel]
            self.last_takeover_throttle_time = now
            events.append("LINK_LOST -> LEVELING")

        current_sensor = self.last_sensor
        if current_sensor is None or not current_sensor.is_fresh(now, cfg.sensor_max_age_s):
            if self.state is not FailsafeState.FAULT:
                events.append("FAULT: MSP DATA STALE OR INCOMPLETE")
                self.fault_frame = frame
            self.state = FailsafeState.FAULT
            if cfg.fault_action == "DISARM":
                disarm_channels = list(unpack_channels(frame[3:-1]))
                disarm_channels[cfg.disarm_channel] = cfg.disarm_value
                disarm_frame = rebuild_rc_frame(frame, disarm_channels)
                events.append("FAULT ACTION: DISARM; MSP data unsafe")
                return self._result(disarm_frame, disarm_channels, events, self.state, True)
            held_frame = self.fault_frame if self.fault_frame is not None else frame
            return self._result(held_frame, list(unpack_channels(held_frame[3:-1])), events, self.state)

        if self.state is FailsafeState.FAULT:
            self.state = FailsafeState.LEVELING
            self.fault_frame = None
            events.append("FAULT -> LEVELING: MSP data restored")

        if self.hold_altitude_m is None and current_sensor.altitude_m is not None:
            self.hold_altitude_m = current_sensor.altitude_m
            self.last_hold_time = now
            self.altitude_integral_m_s = 0.0
            events.append(f"ALTITUDE_HOLD={self.hold_altitude_m:.2f}m")

        output_channels[cfg.roll_channel] = self._correction(current_sensor.roll_deg, cfg.target_roll_deg)
        output_channels[cfg.pitch_channel] = self._correction(current_sensor.pitch_deg, cfg.target_pitch_deg)
        if cfg.stabilization_channel is not None:
            output_channels[cfg.stabilization_channel] = cfg.stabilization_value

        if self.state is FailsafeState.TURNING:
            output_channels[cfg.yaw_channel] = cfg.turn_yaw_command
            self._update_turn_progress(current_sensor.yaw_deg)
            # Разворот завершается по фактическому углу yaw, а не по времени.
            if self.turn_progress_deg >= abs(cfg.turn_degrees):
                self.state = FailsafeState.HOLDING
                self.turn_completed = True
                output_channels[cfg.yaw_channel] = cfg.rc_center
                events.append(
                    f"TURNING -> HOLDING: yaw_target={abs(cfg.turn_degrees):.1f}deg "
                    f"progress={self.turn_progress_deg:.1f}deg"
                )
            else:
                output_channels[cfg.yaw_channel] = cfg.turn_yaw_command

        if self.state is FailsafeState.LEVELING:
            level_ok = self._is_level(current_sensor)
            if level_ok != self.level_was_ok:
                events.append("LEVELING: horizontal position confirmed" if level_ok else "LEVELING: correcting attitude")
                self.level_was_ok = level_ok
            if level_ok:
                if self.level_stable_since is None:
                    self.level_stable_since = now
            else:
                self.level_stable_since = None
            stable = self.level_stable_since is not None and now - self.level_stable_since >= cfg.level_hold_s
            if stable:
                if target_mode:
                    self.state = FailsafeState.HOLDING
                    events.append("LEVELING -> HOLDING: цель ведёт yaw")
                elif cfg.turn_enabled and not self.turn_completed:
                    if current_sensor.yaw_deg is None:
                        self.state = FailsafeState.FAULT
                        self.fault_frame = frame
                        events.append("FAULT: yaw data unavailable; turn not started")
                    else:
                        self.state = FailsafeState.TURNING
                        self.turn_started_at = now
                        self.turn_last_yaw_deg = current_sensor.yaw_deg
                        self.turn_progress_deg = 0.0
                        output_channels[cfg.yaw_channel] = cfg.turn_yaw_command
                        events.append(
                            f"LEVELING -> TURNING: yaw target={abs(cfg.turn_degrees):.0f}deg "
                            "по датчику yaw"
                        )
                else:
                    self.state = FailsafeState.HOLDING
                    events.append("LEVELING -> HOLDING")

        if self.state in (FailsafeState.LEVELING, FailsafeState.TURNING, FailsafeState.HOLDING):
            self._set_hold_throttle(output_channels, current_sensor, now, events)
            # Сначала выравниваем аппарат; наведение носа начинается только
            # после подтверждённого горизонта.
            if target_mode and self.state is FailsafeState.HOLDING:
                output_channels[cfg.yaw_channel] = self._target_yaw_command(target_yaw_error_deg)
        return self._result(rebuild_rc_frame(frame, output_channels), output_channels, events, self.state)

    def _reset_runtime(self, clear_arm_reference: bool) -> None:
        """Сбрасывает автоматические состояния после CH7=LINK_OK или DISARM."""
        self.state = FailsafeState.LIVE
        self.hold_altitude_m = None
        if clear_arm_reference:
            self.altitude_zero_m = None
            self.armed_latched = False
        self.altitude_integral_m_s = 0.0
        self.last_hold_time = None
        self.takeover_entry_throttle = None
        self.last_takeover_throttle_time = None
        self.level_stable_since = None
        self.level_was_ok = None
        self.fault_frame = None
        self.turn_started_at = None
        self.turn_last_yaw_deg = None
        self.turn_progress_deg = 0.0
        self.turn_completed = False
        self.climb_guard_active = False

    def _correction(self, measured: float | None, target: float) -> int:
        """Преобразует ошибку наклона в ограниченную RC-команду."""
        if measured is None:
            return self.config.rc_center
        delta = round((target - measured) * self.config.correction_per_degree)
        delta = max(-self.config.max_correction, min(self.config.max_correction, delta))
        return max(self.config.rc_min, min(self.config.rc_max, self.config.rc_center + delta))

    def _hold_throttle(self, sensor: SensorSample, now: float) -> int:
        """Непрерывно удерживает высоту по барометру, интегратору и вариометру."""
        cfg = self.config
        entry = cfg.level_throttle if self.takeover_entry_throttle is None else self.takeover_entry_throttle
        previous_time = self.last_takeover_throttle_time
        dt_takeover = 0.0 if previous_time is None else max(0.0, min(0.25, now - previous_time))
        self.last_takeover_throttle_time = now
        step = max(0.0, cfg.takeover_throttle_step_per_s) * dt_takeover
        command = min(float(cfg.level_throttle), entry + step) if entry < cfg.level_throttle else max(float(cfg.level_throttle), entry - step)
        self.climb_guard_active = False
        if self.hold_altitude_m is not None and sensor.altitude_m is not None:
            error = self.hold_altitude_m - sensor.altitude_m
            dt = 0.0 if self.last_hold_time is None else max(0.0, min(0.25, now - self.last_hold_time))
            self.last_hold_time = now
            self.altitude_integral_m_s += error * dt
            integral_gain = abs(cfg.altitude_hold_integral_gain)
            if integral_gain > 0.0:
                limit = cfg.max_altitude_integral_correction / integral_gain
                self.altitude_integral_m_s = max(-limit, min(limit, self.altitude_integral_m_s))
            correction = error * cfg.altitude_hold_gain
            correction += self.altitude_integral_m_s * cfg.altitude_hold_integral_gain
            correction -= (sensor.vario_m_s or 0.0) * cfg.altitude_hold_vario_gain
            command += max(-cfg.max_altitude_correction, min(cfg.max_altitude_correction, round(correction)))
            if (
                sensor.altitude_m - self.hold_altitude_m >= cfg.climb_guard_altitude_error_m
                or (sensor.vario_m_s or 0.0) >= cfg.climb_guard_vario_m_s
            ):
                command = min(command, float(cfg.climb_guard_max_throttle))
                self.climb_guard_active = True
        return max(cfg.rc_min, min(cfg.rc_max, round(command)))

    def _set_hold_throttle(self, channels: list[int], sensor: SensorSample, now: float, events: list[str]) -> None:
        """Записывает газ удержания и сообщает о защите от набора."""
        was_active = self.climb_guard_active
        channels[self.config.throttle_channel] = self._hold_throttle(sensor, now)
        if self.climb_guard_active and not was_active:
            events.append("CLIMB GUARD: gas limited; altitude/vario above hold target")

    def _update_turn_progress(self, yaw_deg: float | None) -> None:
        """Накопительно считает пройденный yaw с учётом перехода через 0/360."""
        if yaw_deg is None or self.turn_last_yaw_deg is None:
            return
        delta = (yaw_deg - self.turn_last_yaw_deg + 180.0) % 360.0 - 180.0
        direction = 1.0 if self.config.turn_yaw_command >= self.config.rc_center else -1.0
        self.turn_progress_deg += delta * direction
        self.turn_last_yaw_deg = yaw_deg

    def _is_level(self, sensor: SensorSample) -> bool:
        """Проверяет, находится ли аппарат в заданном горизонтальном диапазоне."""
        return (
            sensor.roll_deg is not None
            and sensor.pitch_deg is not None
            and abs(sensor.roll_deg - self.config.target_roll_deg) <= self.config.level_roll_tolerance_deg
            and abs(sensor.pitch_deg - self.config.target_pitch_deg) <= self.config.level_pitch_tolerance_deg
        )

    def _target_yaw_command(self, error_deg: float | None) -> int:
        """Преобразует горизонтальную ошибку цели в ограниченную команду yaw."""
        if error_deg is None or abs(error_deg) <= self.config.target_yaw_deadband_deg:
            return self.config.rc_center
        correction = round(error_deg * self.config.target_yaw_correction_per_degree)
        correction = max(-self.config.target_yaw_max_correction, min(self.config.target_yaw_max_correction, correction))
        return max(self.config.rc_min, min(self.config.rc_max, self.config.rc_center + correction))

    def relative_altitude(self, sensor: SensorSample | None) -> float | None:
        """Возвращает высоту относительно точки первого ARM для журнала."""
        if sensor is None or sensor.altitude_m is None or self.altitude_zero_m is None:
            return None
        return sensor.altitude_m - self.altitude_zero_m

    def _result(self, frame: bytes, channels: list[int], events: list[str], state: FailsafeState, disarm_requested: bool = False) -> FailsafeResult:
        """Создаёт унифицированный результат для журнала и моста."""
        return FailsafeResult(
            output_frame=frame,
            state=state,
            events=tuple(events),
            roll_command=channels[self.config.roll_channel],
            pitch_command=channels[self.config.pitch_channel],
            throttle_command=channels[self.config.throttle_channel],
            yaw_command=channels[self.config.yaw_channel],
            disarm_requested=disarm_requested,
        )
