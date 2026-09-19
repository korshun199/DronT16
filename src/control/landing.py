"""Посадочный контроллер по данным Betaflight.

Контроллер формирует только изменённый RC-кадр для Betaflight. Он не работает
с PWM и не может обходить арминг, PID или защиту моторов полётника.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from src.protocols.betaflight_msp_link import SensorSample
from src.receiver.crsf import rebuild_rc_frame, unpack_channels


class LandingState(str, Enum):
    """Состояния посадочного автомата."""

    LIVE = "LIVE"
    TAKEOVER = "TAKEOVER"
    LEVELING = "LEVELING"
    DESCENT = "DESCENT"
    LANDED = "LANDED"
    DISARM_RELEASE = "DISARM_RELEASE"
    FAULT = "FAULT"


@dataclass(frozen=True)
class LandingConfig:
    """Параметры каналов и ограничений посадочного контроллера."""

    roll_channel: int
    pitch_channel: int
    throttle_channel: int
    rc_center: int
    rc_min: int
    rc_max: int
    target_roll_deg: float
    target_pitch_deg: float
    correction_per_degree: float
    max_correction: int
    descent_start_after_s: float
    throttle_step_per_s: float
    throttle_min: int
    sensor_max_age_s: float
    landed_altitude_m: float
    landed_vario_abs_m_s: float
    landed_hold_s: float
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
    max_altitude_correction: int = 120
    landing_throttle_barrier: int = 600


@dataclass(frozen=True)
class LandingResult:
    """Результат обработки кадра и текущего образца датчиков."""

    output_frame: bytes
    state: LandingState
    events: tuple[str, ...]
    roll_command: int
    pitch_command: int
    throttle_command: int
    disarm_requested: bool = False


class LandingController:
    """Удерживает горизонт и снижает газ после перехода CRSF в takeover."""

    def __init__(self, config: LandingConfig) -> None:
        """Создаёт автомат в безопасном состоянии LIVE."""
        self.config = config
        self.state = LandingState.LIVE
        self.takeover_started_at: float | None = None
        self.landed_started_at: float | None = None
        self.last_sensor: SensorSample | None = None
        self.last_throttle: int | None = None
        self.initial_throttle: int | None = None
        self.fault_frame: bytes | None = None
        self.altitude_zero_m: float | None = None
        self.was_armed = False
        self.level_stable_since: float | None = None
        self.level_was_ok: bool | None = None
        self.hold_altitude_m: float | None = None
        self.throttle_barrier_triggered = False

    def process(
        self,
        frame: bytes,
        channels: tuple[int, ...],
        takeover_state: str,
        sensor: SensorSample | None,
        now: float,
        armed: bool = True,
    ) -> LandingResult:
        """Формирует RC-кадр для посадки или возвращает исходный кадр."""
        cfg = self.config
        output_channels = list(unpack_channels(frame[3:-1]))
        events: list[str] = []

        if not armed:
            if self.was_armed:
                events.append("ARM -> DISARM: altitude reference cleared")
            self.altitude_zero_m = None
            self.was_armed = False
            self.level_stable_since = None
            self.level_was_ok = None
            self.hold_altitude_m = None
            self.throttle_barrier_triggered = False
        elif sensor is not None and sensor.is_fresh(now, cfg.sensor_max_age_s) and self.altitude_zero_m is None:
            self.altitude_zero_m = sensor.altitude_m
            self.was_armed = True
            events.append(f"ALTITUDE_ZERO={self.altitude_zero_m:.2f}m")
        else:
            self.was_armed = armed

        if takeover_state == "LIVE":
            if self.state is not LandingState.LIVE:
                events.append(f"{self.state.value} -> LIVE")
            self.state = LandingState.LIVE
            self.takeover_started_at = None
            self.landed_started_at = None
            self.last_throttle = None
            self.initial_throttle = None
            self.fault_frame = None
            self.level_stable_since = None
            self.hold_altitude_m = None
            self.throttle_barrier_triggered = False
            return self._result(frame, output_channels, events, self.state)

        if self.state is LandingState.LIVE:
            self.state = LandingState.TAKEOVER
            self.takeover_started_at = now
            events.append("TAKEOVER -> LEVELING")

        if sensor is not None:
            self.last_sensor = sensor
        current_sensor = self.last_sensor
        if current_sensor is None or not current_sensor.is_fresh(now, cfg.sensor_max_age_s):
            if self.state is not LandingState.FAULT:
                events.append("FAULT: MSP DATA STALE OR INCOMPLETE")
                self.fault_frame = frame
            self.state = LandingState.FAULT
            if cfg.fault_action == "DISARM":
                disarm_channels = unpack_channels(frame[3:-1])
                disarm_channels = list(disarm_channels)
                disarm_channels[cfg.disarm_channel] = cfg.disarm_value
                disarm_frame = rebuild_rc_frame(frame, disarm_channels)
                events.append("FAULT ACTION: DISARM; MSP данные небезопасны")
                return self._result(
                    disarm_frame,
                    disarm_channels,
                    events,
                    self.state,
                    disarm_requested=True,
                )
            held_frame = self.fault_frame if self.fault_frame is not None else frame
            return self._result(held_frame, unpack_channels(held_frame[3:-1]), events, self.state)

        if self.state is LandingState.FAULT:
            self.state = LandingState.LEVELING
            self.fault_frame = None
            events.append("FAULT -> LEVELING: MSP данные восстановлены")

        # При потере связи фиксируем высоту, на которой был пилотский контроль.
        # До начала снижения Raspberry удерживает эту высоту, а не последний
        # случайный газ из RC-кадра.
        if self.hold_altitude_m is None and current_sensor.altitude_m is not None:
            self.hold_altitude_m = current_sensor.altitude_m
            events.append(f"ALTITUDE_HOLD={self.hold_altitude_m:.2f}m")

        roll = self._correction(current_sensor.roll_deg, cfg.target_roll_deg)
        pitch = self._correction(current_sensor.pitch_deg, cfg.target_pitch_deg)
        output_channels[cfg.roll_channel] = roll
        output_channels[cfg.pitch_channel] = pitch
        if cfg.stabilization_channel is not None:
            output_channels[cfg.stabilization_channel] = cfg.stabilization_value
        if self.state is LandingState.TAKEOVER:
            self.state = LandingState.LEVELING
            events.append("TAKEOVER -> LEVELING")

        level_ok = self._is_level(current_sensor)
        if level_ok != self.level_was_ok:
            events.append(
                "LEVELING: within attitude limits"
                if level_ok
                else "LEVELING: attitude outside limits; descent paused"
            )
            self.level_was_ok = level_ok
        if level_ok:
            if self.level_stable_since is None:
                self.level_stable_since = now
        else:
            self.level_stable_since = None

        start_time = self.takeover_started_at if self.takeover_started_at is not None else now
        elapsed = now - start_time
        level_hold = self.level_stable_since is not None and now - self.level_stable_since >= cfg.level_hold_s
        if elapsed >= cfg.descent_start_after_s and level_hold and self.state is LandingState.LEVELING:
            self.state = LandingState.DESCENT
            events.append("LEVELING -> DESCENT")

        if self.state is LandingState.LEVELING:
            output_channels[cfg.throttle_channel] = self._hold_throttle(current_sensor)
        if self.state in (LandingState.DESCENT, LandingState.LANDED):
            output_channels[cfg.throttle_channel] = self._throttle_command(output_channels[cfg.throttle_channel], elapsed)
            if self.throttle_barrier_triggered and self.state is LandingState.DESCENT:
                self.state = LandingState.LANDED
                output_channels[cfg.throttle_channel] = cfg.throttle_min
                events.append(
                    f"DESCENT -> LANDED: throttle barrier {cfg.landing_throttle_barrier}; "
                    "газ переведен в минимум"
                )
        if self.state is LandingState.DESCENT and self._is_landed(current_sensor, now):
            self.state = LandingState.LANDED
            output_channels[cfg.throttle_channel] = cfg.throttle_min
            events.append("DESCENT -> LANDED: baro threshold")
        return self._result(rebuild_rc_frame(frame, output_channels), output_channels, events, self.state)

    def _correction(self, measured: float | None, target: float) -> int:
        """Преобразует ошибку наклона в ограниченную RC-команду."""
        if measured is None:
            return self.config.rc_center
        delta = round((target - measured) * self.config.correction_per_degree)
        delta = max(-self.config.max_correction, min(self.config.max_correction, delta))
        return max(self.config.rc_min, min(self.config.rc_max, self.config.rc_center + delta))

    def _throttle_command(self, current: int, elapsed: float) -> int:
        """Плавно уменьшает газ с ограничением скорости изменения."""
        if self.last_throttle is None:
            self.last_throttle = current
            self.initial_throttle = current
        target = self.config.throttle_min
        step = max(0.0, self.config.throttle_step_per_s) * max(0.0, elapsed)
        initial = self.initial_throttle if self.initial_throttle is not None else current
        command = round(max(target, initial - step))
        # Резко завершаем снижение при пересечении настроенного барьера сверху.
        barrier = self.config.landing_throttle_barrier
        if self.last_throttle is not None and self.last_throttle > barrier >= command:
            command = target
            self.throttle_barrier_triggered = True
        self.last_throttle = command
        return command

    def _hold_throttle(self, sensor: SensorSample) -> int:
        """Поддерживает высоту около точки потери связи ограниченной поправкой."""
        command = self.config.level_throttle
        if self.hold_altitude_m is not None and sensor.altitude_m is not None:
            correction = round((self.hold_altitude_m - sensor.altitude_m) * self.config.altitude_hold_gain)
            correction = max(-self.config.max_altitude_correction, min(self.config.max_altitude_correction, correction))
            command += correction
        return max(self.config.rc_min, min(self.config.rc_max, command))

    def _is_landed(self, sensor: SensorSample, now: float) -> bool:
        """Определяет тестовое касание по барометрической высоте и вариометру."""
        if sensor.altitude_m is None or sensor.vario_m_s is None:
            self.landed_started_at = None
            return False
        if self.altitude_zero_m is None:
            return False
        relative_altitude = sensor.altitude_m - self.altitude_zero_m
        near_ground = abs(relative_altitude) <= self.config.landed_altitude_m
        nearly_still = abs(sensor.vario_m_s) <= self.config.landed_vario_abs_m_s
        if near_ground and nearly_still:
            if self.landed_started_at is None:
                self.landed_started_at = now
            return now - self.landed_started_at >= self.config.landed_hold_s
        self.landed_started_at = None
        return False

    def _is_level(self, sensor: SensorSample) -> bool:
        """Проверяет, находится ли корпус в допустимом горизонтальном положении."""
        if sensor.roll_deg is None or sensor.pitch_deg is None:
            return False
        return (
            abs(sensor.roll_deg - self.config.target_roll_deg) <= self.config.level_roll_tolerance_deg
            and abs(sensor.pitch_deg - self.config.target_pitch_deg) <= self.config.level_pitch_tolerance_deg
        )

    def relative_altitude(self, sensor: SensorSample | None) -> float | None:
        """Возвращает высоту относительно точки ARM для журнала и диагностики."""
        if sensor is None or sensor.altitude_m is None or self.altitude_zero_m is None:
            return None
        return sensor.altitude_m - self.altitude_zero_m

    def _result(
        self,
        frame: bytes,
        channels: list[int],
        events: list[str],
        state: LandingState = LandingState.LIVE,
        disarm_requested: bool = False,
    ) -> LandingResult:
        """Создаёт унифицированный результат для журнала и моста."""
        return LandingResult(
            output_frame=frame,
            state=state,
            events=tuple(events),
            roll_command=channels[self.config.roll_channel],
            pitch_command=channels[self.config.pitch_channel],
            throttle_command=channels[self.config.throttle_channel],
            disarm_requested=disarm_requested,
        )
