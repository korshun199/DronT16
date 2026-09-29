"""Внешний контур сопровождения ранее захваченной пилотом цели.

Модуль не выбирает цель и не управляет PWM моторов. Он рассчитывает один
полный CRSF RC-кадр, а быстрый PID и смешивание моторов остаются в Betaflight.
"""

from __future__ import annotations

import math
import statistics
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
    derivative_alpha: float
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
    altitude_reference_window_s: float
    altitude_reference_max_vario_m_s: float
    altitude_reference_max_tilt_deg: float
    altitude_guard_error_m: float
    altitude_guard_vario_m_s: float
    altitude_authority_hard_error_m: float
    altitude_authority_hard_vario_m_s: float
    altitude_heading_min_authority: float
    altitude_kp: float
    altitude_ki: float
    altitude_vario_gain: float
    altitude_integral_limit: float
    altitude_integral_activation_s: float
    throttle_max_correction: int
    throttle_slew_per_s: float
    pitch_deadband: float
    pitch_kp: float
    pitch_kd: float
    pitch_max_angle_deg: float
    pitch_direction: float
    roll_follow_enabled: bool
    roll_follow_deadband: float
    roll_follow_kp: float
    roll_follow_kd: float
    roll_follow_max_angle_deg: float
    roll_follow_direction: float
    roll_angle_to_rate_kp: float
    roll_rate_max_correction: int
    roll_rate_slew_per_s: float
    coordinated_yaw_enabled: bool
    coordinated_yaw_kp: float
    coordinated_yaw_max_correction: int
    coordinated_yaw_direction: float
    yaw_error_alpha: float
    yaw_slew_per_s: float
    roll_thrust_compensation_enabled: bool
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
        if self.min_scale_percent <= 0:
            raise ValueError("Неверный минимальный масштаб цели")
        if not 0 < self.derivative_alpha <= 1 or not 0 < self.hover_learning_alpha <= 1:
            raise ValueError("Коэффициенты сглаживания должны быть в диапазоне (0, 1]")
        if self.altitude_hold_s < 0:
            raise ValueError("Время подтверждения состояния не может быть отрицательным")
        if self.altitude_reference_window_s < 0:
            raise ValueError("Окно усреднения исходной высоты не может быть отрицательным")
        if self.altitude_reference_max_vario_m_s < 0 or self.altitude_reference_max_tilt_deg < 0:
            raise ValueError("Пределы спокойного полёта не могут быть отрицательными")
        if self.altitude_guard_error_m <= 0 or self.altitude_guard_vario_m_s <= 0:
            raise ValueError("Пределы защиты высоты должны быть положительными")
        if (
            self.altitude_authority_hard_error_m <= self.altitude_guard_error_m
            or self.altitude_authority_hard_vario_m_s <= self.altitude_guard_vario_m_s
        ):
            raise ValueError("Жёсткие пределы приоритета высоты должны быть больше мягких")
        if not 0 <= self.altitude_heading_min_authority <= 1:
            raise ValueError("Минимальная власть курса должна быть в диапазоне 0..1")
        if self.altitude_integral_limit < 0:
            raise ValueError("Ограничение интегратора не может быть отрицательным")
        if self.altitude_integral_activation_s < 0 or self.throttle_slew_per_s <= 0:
            raise ValueError("Параметры плавного газа должны быть положительными")
        if self.attitude_kp <= 0 or self.attitude_max_correction <= 0:
            raise ValueError("Параметры контура горизонта должны быть положительными")
        if self.pitch_max_angle_deg <= 0 or self.roll_follow_max_angle_deg <= 0:
            raise ValueError("Максимальные углы pitch и roll должны быть положительными")
        if self.roll_follow_deadband < 0 or self.roll_follow_kp < 0 or self.roll_follow_kd < 0:
            raise ValueError("Параметры бокового контура не могут быть отрицательными")
        if self.roll_angle_to_rate_kp <= 0 or self.roll_rate_max_correction <= 0:
            raise ValueError("Параметры ACRO-контура roll должны быть положительными")
        if self.roll_rate_slew_per_s <= 0:
            raise ValueError("Скорость изменения команды roll должна быть положительной")
        if self.coordinated_yaw_kp < 0 or self.coordinated_yaw_max_correction < 0:
            raise ValueError("Параметры координированного yaw не могут быть отрицательными")
        if not 0 < self.yaw_error_alpha <= 1 or self.yaw_slew_per_s <= 0:
            raise ValueError("Сглаживание и скорость yaw должны быть положительными")
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
        derivative_alpha=float(section["derivative_alpha"]),
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
        altitude_reference_window_s=float(section["altitude_reference_window_s"]),
        altitude_reference_max_vario_m_s=float(section["altitude_reference_max_vario_m_s"]),
        altitude_reference_max_tilt_deg=float(section["altitude_reference_max_tilt_deg"]),
        altitude_guard_error_m=float(section["altitude_guard_error_m"]),
        altitude_guard_vario_m_s=float(section["altitude_guard_vario_m_s"]),
        altitude_authority_hard_error_m=float(section["altitude_authority_hard_error_m"]),
        altitude_authority_hard_vario_m_s=float(section["altitude_authority_hard_vario_m_s"]),
        altitude_heading_min_authority=float(section["altitude_heading_min_authority"]),
        altitude_kp=float(section["altitude_kp"]),
        altitude_ki=float(section["altitude_ki"]),
        altitude_vario_gain=float(section["altitude_vario_gain"]),
        altitude_integral_limit=float(section["altitude_integral_limit"]),
        altitude_integral_activation_s=float(section["altitude_integral_activation_s"]),
        throttle_max_correction=int(section["throttle_max_correction"]),
        throttle_slew_per_s=float(section["throttle_slew_per_s"]),
        pitch_deadband=float(section["pitch_deadband"]),
        pitch_kp=float(section["pitch_kp"]),
        pitch_kd=float(section["pitch_kd"]),
        pitch_max_angle_deg=float(section["pitch_max_angle_deg"]),
        pitch_direction=float(section["pitch_direction"]),
        roll_follow_enabled=bool(section["roll_follow_enabled"]),
        roll_follow_deadband=float(section["roll_follow_deadband"]),
        roll_follow_kp=float(section["roll_follow_kp"]),
        roll_follow_kd=float(section["roll_follow_kd"]),
        roll_follow_max_angle_deg=float(section["roll_follow_max_angle_deg"]),
        roll_follow_direction=float(section["roll_follow_direction"]),
        roll_angle_to_rate_kp=float(section["roll_angle_to_rate_kp"]),
        roll_rate_max_correction=int(section["roll_rate_max_correction"]),
        roll_rate_slew_per_s=float(section["roll_rate_slew_per_s"]),
        coordinated_yaw_enabled=bool(section["coordinated_yaw_enabled"]),
        coordinated_yaw_kp=float(section["coordinated_yaw_kp"]),
        coordinated_yaw_max_correction=int(section["coordinated_yaw_max_correction"]),
        coordinated_yaw_direction=float(section["coordinated_yaw_direction"]),
        yaw_error_alpha=float(section["yaw_error_alpha"]),
        yaw_slew_per_s=float(section["yaw_slew_per_s"]),
        roll_thrust_compensation_enabled=bool(section["roll_thrust_compensation_enabled"]),
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
    """Удерживает высоту и ведёт дрон к цели по координированной дуге."""

    def __init__(self, config: VisualServoConfig) -> None:
        """Создаёт безопасный контроллер без активной цели."""
        config.validate()
        self.config = config
        self.state = VisualServoState.DIRECT
        self.hover_reference: float | None = None
        self.hold_altitude_m: float | None = None
        self._reference_started_at: float | None = None
        self._reference_altitudes: list[float] = []
        self._reference_throttles: list[float] = []
        self.scale_reference: float | None = None
        self.altitude_integral = 0.0
        self.last_time: float | None = None
        self.last_roll_error = 0.0
        self.last_roll_rate_correction = 0.0
        self.last_yaw_correction = 0.0
        self.last_scale_error = 0.0
        self.filtered_roll_derivative = 0.0
        self.filtered_yaw_error = 0.0
        self.filtered_scale_derivative = 0.0
        self.state_started_at: float | None = None
        self.last_calculation_at: float | None = None
        self.last_control_values: tuple[int, int, int, int] | None = None
        self.last_throttle_command: float | None = None
        self._last_altitude_log_at: float | None = None
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
        starting_follow = self.state in {VisualServoState.DIRECT, VisualServoState.CAPTURE}
        if starting_follow:
            self._start_follow(live_channels, target, sensor, now)

        # Не фиксируем высоту и газ одним кадром в момент перевода CH6.
        # Пока пилот ещё набирает/снижает высоту, его RC-кадр проходит без
        # изменений: RPI ждёт спокойный участок и только затем перехватывает
        # CH1–CH4 для FOLLOW.
        if self.hold_altitude_m is None:
            if self._collect_altitude_reference(live_channels, sensor, now):
                events.append(
                    f"FOLLOW -> ALTITUDE_HOLD: altitude={self.hold_altitude_m:.2f}m "
                    f"scale={self.scale_reference:.3f}% hover={self.hover_reference:.0f}"
                )
            else:
                if starting_follow:
                    events.append("FOLLOW: ожидание спокойной высоты и газа пилота")
                return VisualServoResult(frame, self.state, tuple(events), live_channels, False)

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
        lateral_authority, heading_authority = self._follow_authority(sensor)
        desired_roll_target = 0.0
        if self.state is VisualServoState.FOLLOW and cfg.roll_follow_enabled:
            # Высота не выключает сопровождение скачком. Чем сильнее отклонение
            # высоты/вариометра, тем меньше крен и pitch; CH3 в это время
            # продолжает возвращать дрон к H_ref по реальным данным MSP.
            desired_roll_target = self._roll_target_deg(ex, dt)
        follow_roll_target = desired_roll_target * lateral_authority
        # До FOLLOW система только собирает и удерживает спокойную опору
        # высоты. Нос остаётся по центру: управление траекторией начинается
        # строго после подтверждённого перехода ALTITUDE_HOLD -> FOLLOW.
        yaw_command = cfg.rc_center
        if self.state is VisualServoState.FOLLOW:
            # Yaw и крен получают одну ошибку изображения: дрон продолжает
            # идти вперёд, смещается креном и поворачивает нос к цели.
            yaw_command = self._coordinated_yaw_command(ex, heading_authority, dt)
        else:
            self.last_yaw_correction = 0.0
            self.filtered_yaw_error = 0.0
        integral_allowed = self._altitude_is_stable(sensor) and (
            self._state_age(now) >= cfg.altitude_integral_activation_s
        )
        raw_throttle_command = self._altitude_command(sensor, dt, integrate=integral_allowed)
        throttle_command = self._slew_throttle(raw_throttle_command, dt)
        output_channels = list(live_channels)
        # В ACRO CH1 задаёт угловую скорость. Raspberry замыкает внешний
        # контур по фактическому roll из MSP, а Betaflight стабилизирует rate.
        output_channels[cfg.roll_channel] = self._roll_rate_command(
            sensor.roll_deg, follow_roll_target, dt
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
        if self.state is VisualServoState.ALTITUDE_HOLD:
            altitude_error = abs((self.hold_altitude_m or sensor.altitude_m) - sensor.altitude_m)
            vario = abs(sensor.vario_m_s or 0.0)
            if not level_ok:
                self.state_started_at = now
                events.append("ALTITUDE_HOLD: ожидание горизонтального положения")
            elif (
                altitude_error <= cfg.altitude_tolerance_m
                and vario <= cfg.vario_tolerance_m_s
                and self._state_age(now) >= cfg.altitude_hold_s
            ):
                self.state = VisualServoState.FOLLOW
                self.state_started_at = now
                events.append("ALTITUDE_HOLD -> FOLLOW: разрешена координированная дуга")

        elif self.state is VisualServoState.FOLLOW:
            if cfg.roll_follow_enabled:
                events.append(
                    f"FOLLOW: roll target={follow_roll_target:.2f}deg "
                    f"error_x={ex:+.3f} authority={lateral_authority:.2f}"
                )
            target_pitch_deg = self._pitch_target_deg(scale, dt) * lateral_authority
            output_channels[cfg.pitch_channel] = self._attitude_command(
                sensor.pitch_deg, target_pitch_deg, cfg.pitch_rc_direction
            )
            # Продольное движение не блокируется ожиданием выхода на roll.
            # В сопровождении крен, yaw и pitch должны работать одновременно:
            # дрон корректирует боковую траекторию и продолжает идти к цели.
            if lateral_authority < 1.0:
                events.append(
                    "ALTITUDE_PRIORITY: "
                    f"height_error={self.hold_altitude_m - sensor.altitude_m:+.2f}m "
                    f"vario={float(sensor.vario_m_s or 0.0):+.2f}m/s "
                    f"lateral={lateral_authority:.2f} heading={heading_authority:.2f}"
                )

        if (
            self._last_altitude_log_at is None
            or now - self._last_altitude_log_at >= 0.25
        ):
            self._last_altitude_log_at = now
            events.append(
                "ALTITUDE: "
                f"target={self.hold_altitude_m:.2f}m current={sensor.altitude_m:.2f}m "
                f"error={self.hold_altitude_m - sensor.altitude_m:+.2f}m "
                f"vario={float(sensor.vario_m_s or 0.0):+.2f}m/s "
                f"hover={self.hover_reference:.0f} raw={raw_throttle_command} CH3={throttle_command}"
            )

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
        """Начинает сбор спокойной опоры высоты перед автоматическим FOLLOW."""
        cfg = self.config
        self.state = VisualServoState.ALTITUDE_HOLD
        self.hold_altitude_m = None
        self._reference_started_at = now
        self._reference_altitudes = []
        self._reference_throttles = []
        self.scale_reference = float(target.scale_percent or cfg.min_scale_percent)
        self.altitude_integral = 0.0
        self.last_time = now
        self.last_roll_error = float(target.normalized_x or 0.0)
        self.last_roll_rate_correction = 0.0
        self.last_yaw_correction = 0.0
        self.last_scale_error = 0.0
        self.filtered_roll_derivative = 0.0
        self.filtered_yaw_error = 0.0
        self.filtered_scale_derivative = 0.0
        self.state_started_at = now
        self.last_calculation_at = None
        self.last_control_values = None
        self.last_throttle_command = None
        self._last_altitude_log_at = None

    def _reset(self, state: VisualServoState) -> tuple[str, ...]:
        """Сбрасывает автономные опоры после выхода из FOLLOW или DISARM."""
        previous = self.state
        self.state = state
        self.hold_altitude_m = None
        self._reference_started_at = None
        self._reference_altitudes = []
        self._reference_throttles = []
        self.scale_reference = None
        self.altitude_integral = 0.0
        self.last_time = None
        self.last_roll_error = 0.0
        self.last_roll_rate_correction = 0.0
        self.last_yaw_correction = 0.0
        self.filtered_roll_derivative = 0.0
        self.filtered_yaw_error = 0.0
        self.filtered_scale_derivative = 0.0
        self.state_started_at = None
        self.last_calculation_at = None
        self.last_control_values = None
        self.last_throttle_command = None
        self._last_altitude_log_at = None
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

    def _collect_altitude_reference(
        self,
        channels: tuple[int, ...],
        sensor: SensorSample,
        now: float,
    ) -> bool:
        """Усредняет высоту и газ только на спокойном участке ручного полёта."""
        cfg = self.config
        stable = (
            abs(float(sensor.vario_m_s or 0.0)) <= cfg.altitude_reference_max_vario_m_s
            and abs(sensor.roll_deg or 0.0) <= cfg.altitude_reference_max_tilt_deg
            and abs(sensor.pitch_deg or 0.0) <= cfg.altitude_reference_max_tilt_deg
            and cfg.hover_learning_min_throttle
            <= channels[cfg.throttle_channel]
            <= cfg.hover_learning_max_throttle
        )
        if not stable:
            self._reference_started_at = now
            self._reference_altitudes = []
            self._reference_throttles = []
            return False
        self._reference_altitudes.append(float(sensor.altitude_m or 0.0))
        self._reference_throttles.append(float(channels[cfg.throttle_channel]))
        started_at = self._reference_started_at if self._reference_started_at is not None else now
        if now - started_at < cfg.altitude_reference_window_s:
            return False
        self.hold_altitude_m = statistics.median(self._reference_altitudes)
        self.hover_reference = float(self._clamp_rc(round(statistics.median(self._reference_throttles))))
        self.last_throttle_command = self.hover_reference
        self._reference_started_at = None
        self._reference_altitudes = []
        self._reference_throttles = []
        self.altitude_integral = 0.0
        self.state_started_at = now
        return True

    def _altitude_is_stable(self, sensor: SensorSample) -> bool:
        """Проверяет, можно ли считать вертикальное движение спокойным."""
        cfg = self.config
        reference = float(self.hold_altitude_m if self.hold_altitude_m is not None else sensor.altitude_m)
        return (
            abs(reference - float(sensor.altitude_m)) <= cfg.altitude_tolerance_m
            and abs(float(sensor.vario_m_s or 0.0)) <= cfg.vario_tolerance_m_s
        )

    def _follow_authority(self, sensor: SensorSample) -> tuple[float, float]:
        """Возвращает доли власти боковой дуги и курса по фактической вертикали.

        Мягкие пределы означают начало плавного ослабления горизонтального
        движения, жёсткие — его полную остановку. Состояние остаётся FOLLOW:
        при восстановлении высоты траектория возобновляется без нового CH6.
        """
        cfg = self.config
        reference = float(self.hold_altitude_m if self.hold_altitude_m is not None else sensor.altitude_m)
        altitude_authority = self._linear_authority(
            abs(reference - float(sensor.altitude_m)),
            cfg.altitude_guard_error_m,
            cfg.altitude_authority_hard_error_m,
        )
        vario_authority = self._linear_authority(
            abs(float(sensor.vario_m_s or 0.0)),
            cfg.altitude_guard_vario_m_s,
            cfg.altitude_authority_hard_vario_m_s,
        )
        lateral_authority = min(altitude_authority, vario_authority)
        heading_authority = max(cfg.altitude_heading_min_authority, lateral_authority)
        return lateral_authority, heading_authority

    @staticmethod
    def _linear_authority(value: float, soft_limit: float, hard_limit: float) -> float:
        """Плавно переводит измеренную ошибку в долю допустимой команды 0..1."""
        if value <= soft_limit:
            return 1.0
        if value >= hard_limit:
            return 0.0
        return (hard_limit - value) / (hard_limit - soft_limit)

    def _step_time(self, now: float) -> float:
        """Возвращает ограниченный dt, устойчивый к паузам процесса."""
        previous = self.last_time
        self.last_time = now
        if previous is None:
            return 0.0
        return max(0.0, min(0.2, now - previous))

    def _coordinated_yaw_command(
        self, horizontal_error: float, authority: float, dt: float
    ) -> int:
        """Наводит yaw прямо по горизонтальной ошибке цели.

        Раньше yaw косвенно зависел от заданного roll. Это давало слишком
        маленькую команду и не гарантировало, что нос начнёт смотреть к цели.
        Теперь yaw и roll получают одну и ту же ошибку изображения, а
        ``authority`` снижает боковую активность при приоритете высоты.
        """
        cfg = self.config
        alpha = cfg.yaw_error_alpha
        self.filtered_yaw_error = (
            alpha * horizontal_error + (1.0 - alpha) * self.filtered_yaw_error
        )
        requested = 0.0
        if (
            cfg.coordinated_yaw_enabled
            and abs(self.filtered_yaw_error) > cfg.roll_follow_deadband
        ):
            requested = cfg.coordinated_yaw_direction * (
                cfg.coordinated_yaw_kp
                * self.filtered_yaw_error
                * max(0.0, min(1.0, authority))
            )
        requested = max(
            -cfg.coordinated_yaw_max_correction,
            min(cfg.coordinated_yaw_max_correction, requested),
        )
        interval = cfg.control_period_s if dt <= 0.0 else dt
        maximum_step = cfg.yaw_slew_per_s * interval
        correction = max(
            self.last_yaw_correction - maximum_step,
            min(self.last_yaw_correction + maximum_step, requested),
        )
        self.last_yaw_correction = correction
        return self._clamp_rc(round(cfg.rc_center + correction))

    def _roll_target_deg(self, error: float, dt: float) -> float:
        """Рассчитывает требуемый боковой крен по горизонтальной ошибке цели."""
        cfg = self.config
        derivative = 0.0 if dt <= 0 else (error - self.last_roll_error) / dt
        self.last_roll_error = error
        alpha = cfg.derivative_alpha
        self.filtered_roll_derivative = (
            alpha * derivative + (1.0 - alpha) * self.filtered_roll_derivative
        )
        if abs(error) <= cfg.roll_follow_deadband:
            return 0.0
        target_angle = cfg.roll_follow_direction * (
            cfg.roll_follow_kp * error + cfg.roll_follow_kd * self.filtered_roll_derivative
        )
        return max(
            -cfg.roll_follow_max_angle_deg,
            min(cfg.roll_follow_max_angle_deg, target_angle),
        )

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

    def _roll_rate_command(self, measured_deg: float, target_deg: float, dt: float) -> int:
        """Преобразует ошибку roll в плавную ограниченную RC-rate команду ACRO."""
        cfg = self.config
        requested = cfg.roll_direction * (target_deg - measured_deg) * cfg.roll_angle_to_rate_kp
        requested = max(
            -cfg.roll_rate_max_correction,
            min(cfg.roll_rate_max_correction, requested),
        )
        # На первом такте берём заданный период контура: это не даёт первой
        # команде перескочить ограничение при нулевом dt.
        interval = cfg.control_period_s if dt <= 0.0 else dt
        maximum_step = cfg.roll_rate_slew_per_s * interval
        correction = max(
            self.last_roll_rate_correction - maximum_step,
            min(self.last_roll_rate_correction + maximum_step, requested),
        )
        self.last_roll_rate_correction = correction
        return self._clamp_rc(round(cfg.rc_center + correction))

    def _attitude_command(self, measured_deg: float, target_deg: float, direction: float) -> int:
        """Преобразует ошибку угла pitch в ограниченную RC-rate команду ACRO."""
        cfg = self.config
        correction = direction * (target_deg - measured_deg) * cfg.attitude_kp
        correction = max(
            -cfg.attitude_max_correction,
            min(cfg.attitude_max_correction, correction),
        )
        return self._clamp_rc(round(cfg.rc_center + correction))

    def _altitude_command(self, sensor: SensorSample, dt: float, *, integrate: bool) -> int:
        """Рассчитывает газ: P и vario всегда, I только после стабилизации."""
        cfg = self.config
        reference = float(
            self.hold_altitude_m
            if self.hold_altitude_m is not None
            else sensor.altitude_m if sensor.altitude_m is not None else 0.0
        )
        current = float(sensor.altitude_m if sensor.altitude_m is not None else reference)
        error = reference - current
        if integrate:
            self.altitude_integral += error * dt
        else:
            # Не храним старую ошибку, когда барометр ещё догоняет аппарат.
            self.altitude_integral *= 0.92
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
        command = hover + correction
        if cfg.roll_thrust_compensation_enabled:
            # Компенсация работает по фактическому наклону FC, а не по одному
            # намерению внешнего контура: учитываются и roll, и pitch.
            vertical_projection = math.cos(math.radians(sensor.roll_deg or 0.0)) * math.cos(
                math.radians(sensor.pitch_deg or 0.0)
            )
            command /= max(0.90, vertical_projection)
        return self._clamp_rc(round(command))

    def _slew_throttle(self, requested: int, dt: float) -> int:
        """Не даёт CH3 менять тягу резким скачком между двумя расчётами."""
        cfg = self.config
        previous = self.last_throttle_command
        if previous is None:
            previous = float(self.hover_reference or requested)
        interval = cfg.control_period_s if dt <= 0.0 else dt
        maximum_step = cfg.throttle_slew_per_s * interval
        command = max(previous - maximum_step, min(previous + maximum_step, float(requested)))
        command = float(self._clamp_rc(round(command)))
        self.last_throttle_command = command
        return int(command)

    def _state_age(self, now: float) -> float:
        """Возвращает время нахождения в текущем состоянии."""
        return 0.0 if self.state_started_at is None else max(0.0, now - self.state_started_at)

    def _clamp_rc(self, value: int) -> int:
        """Ограничивает RC-команду границами конфигурации."""
        return max(self.config.rc_min, min(self.config.rc_max, int(value)))
