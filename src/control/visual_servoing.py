"""Внешний контур сопровождения ранее захваченной пилотом цели.

Модуль не выбирает цель и не управляет PWM моторов. Он рассчитывает один
полный CRSF RC-кадр, а быстрый PID и смешивание моторов остаются в Betaflight.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Mapping

from src.control.target_control import TargetGuidance
from src.protocols.betaflight_msp_link import SensorSample
from src.receiver.crsf import rebuild_rc_frame


class VisualServoState(str, Enum):
    """Состояния внешнего контура визуального сопровождения."""

    DIRECT = "DIRECT"
    CAPTURE = "CAPTURE"
    YAW_ALIGN = "YAW_ALIGN"
    YAW_HOLD = "YAW_HOLD"
    ALTITUDE_HOLD = "ALTITUDE_HOLD"
    FOLLOW = "FOLLOW"
    TARGET_LOST = "TARGET_LOST"
    FAULT = "FAULT"


@dataclass(frozen=True)
class VisualServoConfig:
    """Все проверяемые ограничения visual_servoing."""

    enabled: bool
    output_mode: str
    roll_channel: int
    pitch_channel: int
    throttle_channel: int
    yaw_channel: int
    rc_center: int
    rc_min: int
    rc_max: int
    target_max_age_s: float
    sensor_max_age_s: float
    sensor_fault_confirmations: int
    control_period_s: float
    min_scale_percent: float
    yaw_deadband: float
    yaw_kp: float
    yaw_kd: float
    yaw_max_correction: int
    yaw_direction: float
    derivative_alpha: float
    yaw_stable_frames: int
    yaw_hold_s: float
    level_roll_tolerance_deg: float
    level_pitch_tolerance_deg: float
    max_abs_tilt_deg: float
    attitude_kp: float
    attitude_max_correction: int
    roll_direction: float
    pitch_rc_direction: float
    altitude_hold_s: float
    altitude_tolerance_m: float
    vario_tolerance_m_s: float
    altitude_kp: float
    altitude_ki: float
    altitude_vario_gain: float
    altitude_integral_limit: float
    throttle_max_correction: int
    pitch_deadband: float
    pitch_kp: float
    pitch_kd: float
    pitch_max_angle_deg: float
    pitch_direction: float
    hover_learning_alpha: float
    hover_learning_max_vario_m_s: float
    hover_learning_max_tilt_deg: float
    hover_learning_min_throttle: int
    hover_learning_max_throttle: int

    def validate(self) -> None:
        """Отклоняет параметры, способные сделать контур непредсказуемым."""
        if self.output_mode not in {"dry-run", "real"}:
            raise ValueError("visual_servoing.output_mode должен быть dry-run или real")
        channels = (self.roll_channel, self.pitch_channel, self.throttle_channel, self.yaw_channel)
        if len(set(channels)) != 4 or any(not 0 <= channel < 16 for channel in channels):
            raise ValueError("Каналы visual_servoing должны быть разными и находиться в диапазоне 1..16")
        if not 0 <= self.rc_min < self.rc_center < self.rc_max <= 2047:
            raise ValueError("Неверные границы CRSF visual_servoing")
        if self.target_max_age_s <= 0 or self.sensor_max_age_s <= 0 or self.control_period_s <= 0:
            raise ValueError("Возраст координат и датчиков должен быть положительным")
        if self.sensor_fault_confirmations < 1:
            raise ValueError("Число подтверждений устаревших MSP-данных должно быть положительным")
        if self.min_scale_percent <= 0 or self.yaw_stable_frames < 1:
            raise ValueError("Неверный минимальный масштаб или число стабильных кадров")
        if not 0 < self.derivative_alpha <= 1 or not 0 < self.hover_learning_alpha <= 1:
            raise ValueError("Коэффициенты сглаживания должны быть в диапазоне (0, 1]")
        if self.yaw_hold_s < 0 or self.altitude_hold_s < 0:
            raise ValueError("Время подтверждения состояния не может быть отрицательным")
        if self.altitude_integral_limit < 0:
            raise ValueError("Ограничение интегратора не может быть отрицательным")
        if self.attitude_kp <= 0 or self.attitude_max_correction <= 0 or self.pitch_max_angle_deg <= 0:
            raise ValueError("Параметры контура горизонта должны быть положительными")
        if not self.rc_min <= self.hover_learning_min_throttle < self.hover_learning_max_throttle <= self.rc_max:
            raise ValueError("Неверный диапазон обучения газа висения")


def build_visual_servo_config(
    section: Mapping[str, object],
    *,
    target_max_age_s: float,
    sensor_max_age_s: float,
) -> VisualServoConfig:
    """Преобразует единую TOML-секцию в проверенную конфигурацию контроллера."""
    config = VisualServoConfig(
        enabled=bool(section["enabled"]),
        output_mode=str(section["output_mode"]),
        roll_channel=int(section["roll_channel"]) - 1,
        pitch_channel=int(section["pitch_channel"]) - 1,
        throttle_channel=int(section["throttle_channel"]) - 1,
        yaw_channel=int(section["yaw_channel"]) - 1,
        rc_center=int(section["rc_center"]),
        rc_min=int(section["rc_min"]),
        rc_max=int(section["rc_max"]),
        target_max_age_s=target_max_age_s,
        sensor_max_age_s=sensor_max_age_s,
        sensor_fault_confirmations=int(section["sensor_fault_confirmations"]),
        control_period_s=int(section["control_period_ms"]) / 1000.0,
        min_scale_percent=float(section["min_scale_percent"]),
        yaw_deadband=float(section["yaw_deadband"]),
        yaw_kp=float(section["yaw_kp"]),
        yaw_kd=float(section["yaw_kd"]),
        yaw_max_correction=int(section["yaw_max_correction"]),
        yaw_direction=float(section["yaw_direction"]),
        derivative_alpha=float(section["derivative_alpha"]),
        yaw_stable_frames=int(section["yaw_stable_frames"]),
        yaw_hold_s=float(section["yaw_hold_s"]),
        level_roll_tolerance_deg=float(section["level_roll_tolerance_deg"]),
        level_pitch_tolerance_deg=float(section["level_pitch_tolerance_deg"]),
        max_abs_tilt_deg=float(section["max_abs_tilt_deg"]),
        attitude_kp=float(section["attitude_kp"]),
        attitude_max_correction=int(section["attitude_max_correction"]),
        roll_direction=float(section["roll_direction"]),
        pitch_rc_direction=float(section["pitch_rc_direction"]),
        altitude_hold_s=float(section["altitude_hold_s"]),
        altitude_tolerance_m=float(section["altitude_tolerance_m"]),
        vario_tolerance_m_s=float(section["vario_tolerance_m_s"]),
        altitude_kp=float(section["altitude_kp"]),
        altitude_ki=float(section["altitude_ki"]),
        altitude_vario_gain=float(section["altitude_vario_gain"]),
        altitude_integral_limit=float(section["altitude_integral_limit"]),
        throttle_max_correction=int(section["throttle_max_correction"]),
        pitch_deadband=float(section["pitch_deadband"]),
        pitch_kp=float(section["pitch_kp"]),
        pitch_kd=float(section["pitch_kd"]),
        pitch_max_angle_deg=float(section["pitch_max_angle_deg"]),
        pitch_direction=float(section["pitch_direction"]),
        hover_learning_alpha=float(section["hover_learning_alpha"]),
        hover_learning_max_vario_m_s=float(section["hover_learning_max_vario_m_s"]),
        hover_learning_max_tilt_deg=float(section["hover_learning_max_tilt_deg"]),
        hover_learning_min_throttle=int(section["hover_learning_min_throttle"]),
        hover_learning_max_throttle=int(section["hover_learning_max_throttle"]),
    )
    config.validate()
    return config


@dataclass(frozen=True)
class VisualServoResult:
    """Результат одного шага расчёта и готовый кадр для выбранного транспорта."""

    output_frame: bytes
    state: VisualServoState
    events: tuple[str, ...]
    computed_channels: tuple[int, ...]
    output_applied: bool
    target_lost: bool = False
    fault: bool = False


class VisualServoController:
    """Наводит нос, удерживает высоту и затем регулирует дистанцию по масштабу."""

    def __init__(self, config: VisualServoConfig) -> None:
        """Создаёт безопасный контроллер без активной цели."""
        config.validate()
        self.config = config
        self.state = VisualServoState.DIRECT
        self.hover_reference: float | None = None
        self.hold_altitude_m: float | None = None
        self.scale_reference: float | None = None
        self.yaw_reference_deg: float | None = None
        self.altitude_integral = 0.0
        self.last_time: float | None = None
        self.last_yaw_error = 0.0
        self.last_scale_error = 0.0
        self.filtered_yaw_derivative = 0.0
        self.filtered_scale_derivative = 0.0
        self.yaw_stable_count = 0
        self.state_started_at: float | None = None
        self.last_calculation_at: float | None = None
        self.last_control_values: tuple[int, int, int, int] | None = None
        self._fault_latched = False
        self._target_lost_latched = False
        self._sensor_stale_count = 0
        # Нельзя считать несколько кадров очереди за одну ошибку MSP: новое
        # подтверждение устаревания разрешено только раз за период контура.
        self._last_sensor_stale_check_at: float | None = None

    def process(
        self,
        frame: bytes,
        channels: tuple[int, ...],
        receiver_mode: str,
        target: TargetGuidance | None,
        sensor: SensorSample | None,
        now: float,
        *,
        armed: bool,
        takeover_allowed: bool = True,
    ) -> VisualServoResult:
        """Обрабатывает снимок цели и возвращает исходный либо изменённый RC-кадр."""
        cfg = self.config
        live_channels = tuple(channels)
        mode = receiver_mode.upper()

        if not armed:
            self.hover_reference = None
            events = self._reset(VisualServoState.DIRECT)
            return VisualServoResult(frame, VisualServoState.DIRECT, events, live_channels, False)

        if not takeover_allowed or mode != "FOLLOW" or not cfg.enabled:
            passive_state = VisualServoState.CAPTURE if mode == "CAPTURE" and armed else VisualServoState.DIRECT
            if takeover_allowed:
                self._learn_hover_reference(live_channels, sensor, now, armed)
            events = self._reset(passive_state)
            return VisualServoResult(frame, passive_state, events, live_channels, False)

        if self._target_lost_latched:
            return VisualServoResult(
                frame, VisualServoState.TARGET_LOST, (), live_channels, False, target_lost=True
            )
        if self._fault_latched:
            return VisualServoResult(frame, VisualServoState.FAULT, (), live_channels, False, fault=True)

        if target is None or not target.is_complete(now, cfg.target_max_age_s, cfg.min_scale_percent):
            self.state = VisualServoState.TARGET_LOST
            self._target_lost_latched = True
            return VisualServoResult(
                frame,
                self.state,
                ("TARGET_LOST: координаты цели отсутствуют или устарели",),
                live_channels,
                False,
                target_lost=True,
            )
        sensor_age = self._sensor_age(sensor, now)
        if sensor is None or not sensor.is_fresh(now, cfg.sensor_max_age_s):
            confirmation_due = (
                self._last_sensor_stale_check_at is None
                or now - self._last_sensor_stale_check_at >= cfg.control_period_s
            )
            if confirmation_due:
                self._sensor_stale_count += 1
                self._last_sensor_stale_check_at = now
            age_text = "нет" if math.isinf(sensor_age) else f"{sensor_age:.3f}s"
            if self._sensor_stale_count < cfg.sensor_fault_confirmations:
                events = ()
                if confirmation_due:
                    events = (
                        f"MSP DATA DELAY: age={age_text} "
                        f"confirmation={self._sensor_stale_count}/{cfg.sensor_fault_confirmations}; "
                        "сохраняется последняя команда",
                    )
                if self.last_control_values is None:
                    return VisualServoResult(frame, self.state, events, live_channels, False)
                repeated_channels = list(live_channels)
                for channel, value in zip(
                    (cfg.roll_channel, cfg.pitch_channel, cfg.throttle_channel, cfg.yaw_channel),
                    self.last_control_values,
                ):
                    repeated_channels[channel] = value
                computed_channels = tuple(repeated_channels)
                output_applied = cfg.output_mode == "real"
                output_frame = rebuild_rc_frame(frame, computed_channels) if output_applied else frame
                return VisualServoResult(
                    output_frame,
                    self.state,
                    events,
                    computed_channels,
                    output_applied,
                )
            self.state = VisualServoState.FAULT
            self._fault_latched = True
            return VisualServoResult(
                frame,
                self.state,
                (
                    f"FAULT: обязательные MSP-датчики устарели; age={age_text}; "
                    f"подтверждений={self._sensor_stale_count}/{cfg.sensor_fault_confirmations}",
                ),
                live_channels,
                False,
                fault=True,
            )
        restored_events: list[str] = []
        if self._sensor_stale_count:
            restored_events.append(
                f"MSP DATA RESTORED: age={sensor_age:.3f}s после "
                f"{self._sensor_stale_count} пропусков; FOLLOW продолжен"
            )
        self._sensor_stale_count = 0
        self._last_sensor_stale_check_at = None
        if sensor.roll_deg is None or sensor.pitch_deg is None or sensor.altitude_m is None:
            self.state = VisualServoState.FAULT
            self._fault_latched = True
            return VisualServoResult(
                frame, self.state, ("FAULT: неполный MSP-снимок",), live_channels, False, fault=True
            )
        if abs(sensor.roll_deg) > cfg.max_abs_tilt_deg or abs(sensor.pitch_deg) > cfg.max_abs_tilt_deg:
            self.state = VisualServoState.FAULT
            self._fault_latched = True
            return VisualServoResult(
                frame,
                self.state,
                ("FAULT: наклон превышает разрешённый предел visual_servoing",),
                live_channels,
                False,
                fault=True,
            )

        events: list[str] = restored_events
        if self.state in {VisualServoState.DIRECT, VisualServoState.CAPTURE}:
            self._start_follow(live_channels, target, sensor, now)
            events.append(
                f"FOLLOW -> YAW_ALIGN: altitude={self.hold_altitude_m:.2f}m "
                f"scale={self.scale_reference:.3f}% hover={self.hover_reference:.0f}"
            )

        if (
            self.last_calculation_at is not None
            and now - self.last_calculation_at < cfg.control_period_s
            and self.last_control_values is not None
        ):
            repeated_channels = list(live_channels)
            for channel, value in zip(
                (cfg.roll_channel, cfg.pitch_channel, cfg.throttle_channel, cfg.yaw_channel),
                self.last_control_values,
            ):
                repeated_channels[channel] = value
            computed_channels = tuple(repeated_channels)
            output_applied = cfg.output_mode == "real"
            output_frame = rebuild_rc_frame(frame, computed_channels) if output_applied else frame
            return VisualServoResult(
                output_frame, self.state, tuple(events), computed_channels, output_applied
            )
        self.last_calculation_at = now

        dt = self._step_time(now)
        ex = float(target.normalized_x or 0.0)
        scale = float(target.scale_percent or cfg.min_scale_percent)
        yaw_command = self._yaw_command(ex, dt)
        throttle_command = self._altitude_command(sensor, dt)
        output_channels = list(live_channels)
        # Внешний контур задаёт требуемые углы, а Betaflight выполняет быстрый
        # внутренний PID и распределяет тягу между моторами.
        output_channels[cfg.roll_channel] = self._attitude_command(
            sensor.roll_deg, 0.0, cfg.roll_direction
        )
        output_channels[cfg.pitch_channel] = self._attitude_command(
            sensor.pitch_deg, 0.0, cfg.pitch_rc_direction
        )
        output_channels[cfg.throttle_channel] = throttle_command
        output_channels[cfg.yaw_channel] = yaw_command

        level_ok = (
            abs(sensor.roll_deg) <= cfg.level_roll_tolerance_deg
            and abs(sensor.pitch_deg) <= cfg.level_pitch_tolerance_deg
        )
        yaw_ok = abs(ex) <= cfg.yaw_deadband

        if self.state is VisualServoState.YAW_ALIGN:
            self.yaw_stable_count = self.yaw_stable_count + 1 if yaw_ok and level_ok else 0
            if self.yaw_stable_count >= cfg.yaw_stable_frames:
                self.state = VisualServoState.YAW_HOLD
                self.state_started_at = now
                self.yaw_reference_deg = sensor.yaw_deg
                events.append("YAW_ALIGN -> YAW_HOLD: цель и горизонт подтверждены")

        elif self.state is VisualServoState.YAW_HOLD:
            if not yaw_ok or not level_ok:
                self.state = VisualServoState.YAW_ALIGN
                self.state_started_at = now
                self.yaw_stable_count = 0
                events.append("YAW_HOLD -> YAW_ALIGN: цель вышла из допуска")
            elif self._state_age(now) >= cfg.yaw_hold_s:
                self.state = VisualServoState.ALTITUDE_HOLD
                self.state_started_at = now
                events.append("YAW_HOLD -> ALTITUDE_HOLD")

        elif self.state is VisualServoState.ALTITUDE_HOLD:
            altitude_error = abs((self.hold_altitude_m or sensor.altitude_m) - sensor.altitude_m)
            vario = abs(sensor.vario_m_s or 0.0)
            if not yaw_ok or not level_ok:
                self.state = VisualServoState.YAW_ALIGN
                self.state_started_at = now
                self.yaw_stable_count = 0
                events.append("ALTITUDE_HOLD -> YAW_ALIGN: потеряно наведение или горизонт")
            elif (
                altitude_error <= cfg.altitude_tolerance_m
                and vario <= cfg.vario_tolerance_m_s
                and self._state_age(now) >= cfg.altitude_hold_s
            ):
                self.state = VisualServoState.FOLLOW
                self.state_started_at = now
                events.append("ALTITUDE_HOLD -> FOLLOW: разрешён ограниченный pitch")

        elif self.state is VisualServoState.FOLLOW:
            target_pitch_deg = self._pitch_target_deg(scale, dt)
            output_channels[cfg.pitch_channel] = self._attitude_command(
                sensor.pitch_deg, target_pitch_deg, cfg.pitch_rc_direction
            )
            if abs(sensor.roll_deg) > cfg.level_roll_tolerance_deg:
                # При боковом наклоне движение вперёд/назад прекращается,
                # а pitch-контур сначала возвращает корпус к нулевому углу.
                output_channels[cfg.pitch_channel] = self._attitude_command(
                    sensor.pitch_deg, 0.0, cfg.pitch_rc_direction
                )
                events.append("FOLLOW: движение заблокировано до восстановления roll")

        computed_channels = tuple(self._clamp_rc(value) for value in output_channels)
        self.last_control_values = (
            computed_channels[cfg.roll_channel],
            computed_channels[cfg.pitch_channel],
            computed_channels[cfg.throttle_channel],
            computed_channels[cfg.yaw_channel],
        )
        output_applied = cfg.output_mode == "real"
        output_frame = rebuild_rc_frame(frame, computed_channels) if output_applied else frame
        return VisualServoResult(output_frame, self.state, tuple(events), computed_channels, output_applied)

    def _start_follow(
        self,
        channels: tuple[int, ...],
        target: TargetGuidance,
        sensor: SensorSample,
        now: float,
    ) -> None:
        """Фиксирует опорные значения только при подтверждённом входе в FOLLOW."""
        cfg = self.config
        self.state = VisualServoState.YAW_ALIGN
        self.hold_altitude_m = float(sensor.altitude_m if sensor.altitude_m is not None else 0.0)
        self.scale_reference = float(target.scale_percent or cfg.min_scale_percent)
        self.yaw_reference_deg = sensor.yaw_deg
        if self.hover_reference is None:
            self.hover_reference = float(channels[cfg.throttle_channel])
        self.hover_reference = float(self._clamp_rc(round(self.hover_reference)))
        self.altitude_integral = 0.0
        self.last_time = now
        self.last_yaw_error = float(target.normalized_x or 0.0)
        self.last_scale_error = 0.0
        self.filtered_yaw_derivative = 0.0
        self.filtered_scale_derivative = 0.0
        self.yaw_stable_count = 0
        self.state_started_at = now
        self.last_calculation_at = None
        self.last_control_values = None

    def _reset(self, state: VisualServoState) -> tuple[str, ...]:
        """Сбрасывает автономные опоры после выхода из FOLLOW или DISARM."""
        previous = self.state
        self.state = state
        self.hold_altitude_m = None
        self.scale_reference = None
        self.yaw_reference_deg = None
        self.altitude_integral = 0.0
        self.last_time = None
        self.filtered_yaw_derivative = 0.0
        self.filtered_scale_derivative = 0.0
        self.yaw_stable_count = 0
        self.state_started_at = None
        self.last_calculation_at = None
        self.last_control_values = None
        self._fault_latched = False
        self._target_lost_latched = False
        self._sensor_stale_count = 0
        self._last_sensor_stale_check_at = None
        if previous != state:
            return (f"{previous.value} -> {state.value}: управление возвращено пилоту",)
        return ()

    @staticmethod
    def _sensor_age(sensor: SensorSample | None, now: float) -> float:
        """Возвращает возраст самого старого обязательного MSP-показания."""
        if sensor is None or sensor.altitude_received_at is None or sensor.attitude_received_at is None:
            return math.inf
        return max(now - sensor.altitude_received_at, now - sensor.attitude_received_at)

    def _learn_hover_reference(
        self,
        channels: tuple[int, ...],
        sensor: SensorSample | None,
        now: float,
        armed: bool,
    ) -> None:
        """Медленно оценивает газ висения по устойчивому ручному полёту."""
        cfg = self.config
        if not armed or sensor is None or not sensor.is_fresh(now, cfg.sensor_max_age_s):
            return
        if sensor.roll_deg is None or sensor.pitch_deg is None:
            return
        throttle = channels[cfg.throttle_channel]
        stable = (
            abs(sensor.roll_deg) <= cfg.hover_learning_max_tilt_deg
            and abs(sensor.pitch_deg) <= cfg.hover_learning_max_tilt_deg
            and abs(sensor.vario_m_s or 0.0) <= cfg.hover_learning_max_vario_m_s
            and cfg.hover_learning_min_throttle <= throttle <= cfg.hover_learning_max_throttle
        )
        if not stable:
            return
        if self.hover_reference is None:
            self.hover_reference = float(throttle)
        else:
            alpha = cfg.hover_learning_alpha
            self.hover_reference = alpha * throttle + (1.0 - alpha) * self.hover_reference

    def _step_time(self, now: float) -> float:
        """Возвращает ограниченный dt, устойчивый к паузам процесса."""
        previous = self.last_time
        self.last_time = now
        if previous is None:
            return 0.0
        return max(0.0, min(0.2, now - previous))

    def _yaw_command(self, error: float, dt: float) -> int:
        """Рассчитывает ограниченную PD-команду yaw по горизонтальной ошибке."""
        cfg = self.config
        derivative = 0.0 if dt <= 0 else (error - self.last_yaw_error) / dt
        self.last_yaw_error = error
        alpha = cfg.derivative_alpha
        self.filtered_yaw_derivative = (
            alpha * derivative + (1.0 - alpha) * self.filtered_yaw_derivative
        )
        if abs(error) <= cfg.yaw_deadband:
            return cfg.rc_center
        correction = cfg.yaw_direction * (
            cfg.yaw_kp * error + cfg.yaw_kd * self.filtered_yaw_derivative
        )
        correction = max(-cfg.yaw_max_correction, min(cfg.yaw_max_correction, correction))
        return self._clamp_rc(round(cfg.rc_center + correction))

    def _pitch_target_deg(self, scale: float, dt: float) -> float:
        """Рассчитывает требуемый угол pitch по относительному масштабу цели."""
        cfg = self.config
        reference = max(cfg.min_scale_percent, float(self.scale_reference or scale))
        error = -math.log(max(cfg.min_scale_percent, scale) / reference)
        derivative = 0.0 if dt <= 0 else (error - self.last_scale_error) / dt
        self.last_scale_error = error
        alpha = cfg.derivative_alpha
        self.filtered_scale_derivative = (
            alpha * derivative + (1.0 - alpha) * self.filtered_scale_derivative
        )
        if abs(error) <= cfg.pitch_deadband:
            return 0.0
        target_angle = cfg.pitch_direction * (
            cfg.pitch_kp * error + cfg.pitch_kd * self.filtered_scale_derivative
        )
        return max(-cfg.pitch_max_angle_deg, min(cfg.pitch_max_angle_deg, target_angle))

    def _attitude_command(self, measured_deg: float, target_deg: float, direction: float) -> int:
        """Преобразует ошибку требуемого угла в ограниченное отклонение RC."""
        cfg = self.config
        correction = direction * (target_deg - measured_deg) * cfg.attitude_kp
        correction = max(
            -cfg.attitude_max_correction,
            min(cfg.attitude_max_correction, correction),
        )
        return self._clamp_rc(round(cfg.rc_center + correction))

    def _altitude_command(self, sensor: SensorSample, dt: float) -> int:
        """Удерживает высоту PI-контуром с демпфированием по вариометру."""
        cfg = self.config
        reference = float(
            self.hold_altitude_m
            if self.hold_altitude_m is not None
            else sensor.altitude_m if sensor.altitude_m is not None else 0.0
        )
        current = float(sensor.altitude_m if sensor.altitude_m is not None else reference)
        error = reference - current
        self.altitude_integral += error * dt
        self.altitude_integral = max(
            -cfg.altitude_integral_limit,
            min(cfg.altitude_integral_limit, self.altitude_integral),
        )
        correction = (
            cfg.altitude_kp * error
            + cfg.altitude_ki * self.altitude_integral
            - cfg.altitude_vario_gain * float(sensor.vario_m_s or 0.0)
        )
        correction = max(
            -cfg.throttle_max_correction,
            min(cfg.throttle_max_correction, correction),
        )
        hover = float(self.hover_reference or cfg.rc_center)
        return self._clamp_rc(round(hover + correction))

    def _state_age(self, now: float) -> float:
        """Возвращает время нахождения в текущем состоянии."""
        return 0.0 if self.state_started_at is None else max(0.0, now - self.state_started_at)

    def _clamp_rc(self, value: int) -> int:
        """Ограничивает RC-команду границами конфигурации."""
        return max(self.config.rc_min, min(self.config.rc_max, int(value)))
