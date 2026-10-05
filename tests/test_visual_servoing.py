"""Проверки внешнего контура сопровождения без подключения оборудования."""

from __future__ import annotations

import dataclasses
import unittest

from src.control.target_control import TargetGuidance
from src.control.visual_servoing import (
    VisualServoConfig,
    VisualServoController,
    VisualServoState,
    build_visual_servo_config,
)
from src.configuration import load_config_section
from src.protocols.betaflight_msp_link import SensorSample
from src.receiver.crsf import (
    CRSF_RC_CHANNELS_PACKED,
    crc8_dvb_s2,
    extract_raw_frames,
    pack_channels,
    unpack_channels,
)


def config(**overrides: object) -> VisualServoConfig:
    """Возвращает быструю тестовую конфигурацию visual_servoing."""
    values: dict[str, object] = {
        "enabled": True,
        "output_mode": "dry-run",
        "roll_channel": 0,
        "pitch_channel": 1,
        "throttle_channel": 2,
        "yaw_channel": 3,
        "rc_center": 992,
        "rc_min": 191,
        "rc_max": 1792,
        "target_max_age_s": 0.3,
        "sensor_max_age_s": 0.3,
        "sensor_fault_confirmations": 3,
        "control_period_s": 0.05,
        "min_scale_percent": 0.05,
        "derivative_alpha": 0.25,
        "level_roll_tolerance_deg": 6.0,
        "level_pitch_tolerance_deg": 6.0,
        "max_abs_tilt_deg": 45.0,
        "attitude_kp": 18.0,
        "attitude_max_correction": 250,
        "roll_direction": 1.0,
        "pitch_rc_direction": 1.0,
        "altitude_hold_s": 0.1,
        "altitude_tolerance_m": 0.2,
        "vario_tolerance_m_s": 0.3,
        "altitude_reference_window_s": 0.0,
        "altitude_reference_max_vario_m_s": 0.2,
        "altitude_reference_max_tilt_deg": 6.0,
        "altitude_guard_error_m": 0.2,
        "altitude_guard_vario_m_s": 0.3,
        "altitude_guard_confirmation_s": 0.0,
        "altitude_authority_hard_error_m": 0.6,
        "altitude_authority_hard_vario_m_s": 0.9,
        "altitude_heading_min_authority": 0.25,
        "altitude_kp": 80.0,
        "altitude_ki": 12.0,
        "altitude_vario_gain": 35.0,
        "altitude_integral_limit": 2.0,
        "altitude_integral_activation_s": 0.1,
        "throttle_max_correction": 120,
        "throttle_slew_per_s": 200.0,
        "pitch_deadband": 0.05,
        "pitch_kp": 6.0,
        "pitch_kd": 0.0,
        "pitch_max_angle_deg": 8.0,
        "pitch_direction": 1.0,
        "forward_pitch_reference_enabled": True,
        "forward_pitch_reference_window_s": 0.35,
        "throttle_reference_window_s": 0.35,
        "forward_pitch_reference_max_deg": 12.0,
        "follow_entry_blend_s": 0.0,
        "pitch_max_correction_deg": 6.0,
        "pitch_rate_slew_per_s": 240.0,
        "roll_follow_enabled": False,
        "roll_follow_deadband": 0.04,
        "roll_follow_kp": 8.0,
        "roll_follow_kd": 0.4,
        "roll_follow_max_angle_deg": 8.0,
        "roll_follow_direction": 1.0,
        "roll_brake_lookahead_s": 0.25,
        "roll_brake_derivative_deadband": 0.03,
        "roll_angle_to_rate_kp": 18.0,
        "roll_rate_max_correction": 160,
        "roll_rate_slew_per_s": 480.0,
        "coordinated_yaw_enabled": True,
        "coordinated_yaw_kp": 6.0,
        "coordinated_yaw_max_correction": 60,
        "coordinated_yaw_direction": 1.0,
        "coordinated_yaw_brake_kd": 0.0,
        "yaw_error_alpha": 1.0,
        "yaw_slew_per_s": 240.0,
        "roll_thrust_compensation_enabled": True,
        "hover_learning_alpha": 0.1,
        "hover_learning_max_vario_m_s": 0.2,
        "hover_learning_max_tilt_deg": 8.0,
        "hover_learning_min_throttle": 600,
        "hover_learning_max_throttle": 1500,
        "immediate_follow_takeover": True,
    }
    values.update(overrides)
    return VisualServoConfig(**values)  # type: ignore[arg-type]


def rc_frame(throttle: int = 1100, pitch: int = 992, ch5: int = 1792,
             ch6: int = 1792, ch7: int = 191) -> tuple[bytes, tuple[int, ...]]:
    """Формирует полный проверенный CRSF-кадр для теста."""
    channels = [992] * 16
    channels[1] = pitch
    channels[2] = throttle
    channels[4] = ch5
    channels[5] = ch6
    channels[6] = ch7
    packed = pack_channels(tuple(channels))
    body = bytes((0xC8, len(packed) + 2, CRSF_RC_CHANNELS_PACKED)) + packed
    return body + bytes((crc8_dvb_s2(body[2:]),)), tuple(channels)


def target(now: float, x: float = 0.0, scale: float = 4.0) -> TargetGuidance:
    """Создаёт свежий снимок ранее выбранной цели."""
    return TargetGuidance(True, x * 45.0, 0.0, now, "Следить", x, 0.0, scale)


def sensor(now: float, altitude: float = 2.0, vario: float = 0.0,
           roll: float = 0.0, pitch: float = 0.0) -> SensorSample:
    """Создаёт полный свежий снимок обязательных MSP-датчиков."""
    return SensorSample(
        altitude, vario, roll, pitch, 90.0, now, True, True, now, now
    )


class VisualServoTests(unittest.TestCase):
    """Проверяет состояния, ограничения и реальную сборку RC-кадра."""

    def test_direct_and_capture_never_change_live_frame(self) -> None:
        """Нижнее и среднее положения CH6 всегда оставляют управление пилоту."""
        controller = VisualServoController(config(output_mode="real"))
        frame, channels = rc_frame()
        direct = controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)
        capture = controller.process(frame, channels, "CAPTURE", None, sensor(0.1), 0.1, armed=True)
        self.assertEqual(direct.output_frame, frame)
        self.assertEqual(capture.output_frame, frame)
        self.assertEqual(capture.state, VisualServoState.CAPTURE)

    def test_follow_takes_over_immediately_and_starts_arc_without_yaw_alignment(self) -> None:
        """FOLLOW сразу применяет удержание высоты и дугу без разворота на месте."""
        controller = VisualServoController(config())
        frame, channels = rc_frame(throttle=1100)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)

        first = controller.process(frame, channels, "FOLLOW", target(0.1), sensor(0.1), 0.1, armed=True)
        self.assertEqual(first.state, VisualServoState.FOLLOW)
        self.assertEqual(first.computed_channels[2], channels[2])
        follow = controller.process(frame, channels, "FOLLOW", target(0.27), sensor(0.27), 0.27, armed=True)
        moved = controller.process(
            frame, channels, "FOLLOW", target(0.38, scale=2.0), sensor(0.38), 0.38, armed=True
        )
        self.assertGreater(moved.computed_channels[1], 992)

    def test_follow_takes_over_even_when_throttle_is_below_hover_range(self) -> None:
        """FOLLOW не блокируется низким CH3 и начинает коррекцию высоты сразу."""
        controller = VisualServoController(
            config(output_mode="real", altitude_reference_window_s=0.10)
        )
        frame, channels = rc_frame(throttle=191)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)

        climbing = controller.process(
            frame, channels, "FOLLOW", target(0.1), sensor(0.1, vario=0.5), 0.1, armed=True
        )
        self.assertEqual(climbing.state, VisualServoState.FOLLOW)
        self.assertTrue(climbing.output_applied)
        self.assertEqual(climbing.computed_channels[0], channels[0])
        self.assertEqual(climbing.computed_channels[2], channels[2])

    def test_altitude_priority_weakens_but_does_not_cancel_follow(self) -> None:
        """Ошибка высоты ослабляет дугу, сохраняя FOLLOW и направление к цели."""
        controller = VisualServoController(
            config(output_mode="real", roll_follow_enabled=True)
        )
        frame, channels = rc_frame(throttle=1100)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.1), sensor(0.1), 0.1, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.16), sensor(0.16), 0.16, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.27), sensor(0.27), 0.27, armed=True)

        guarded = controller.process(
            frame,
            channels,
            "FOLLOW",
            target(0.38, x=1.0),
            sensor(0.38, altitude=2.5, vario=0.4),
            0.38,
            armed=True,
        )
        self.assertEqual(guarded.state, VisualServoState.FOLLOW)
        self.assertGreater(guarded.computed_channels[0], 992)
        self.assertGreater(guarded.computed_channels[3], 992)
        self.assertTrue(any(event.startswith("ALTITUDE_PRIORITY") for event in guarded.events))

    def test_hard_altitude_priority_centers_lateral_motion_but_keeps_heading(self) -> None:
        """Большая вертикальная ошибка оставляет FOLLOW и малую поправку курса."""
        controller = VisualServoController(
            config(output_mode="real", roll_follow_enabled=True)
        )
        frame, channels = rc_frame(throttle=1100)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.1), sensor(0.1), 0.1, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.16), sensor(0.16), 0.16, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.27), sensor(0.27), 0.27, armed=True)

        guarded = controller.process(
            frame,
            channels,
            "FOLLOW",
            target(0.38, x=1.0),
            sensor(0.38, altitude=2.8, vario=1.0),
            0.38,
            armed=True,
        )
        self.assertEqual(guarded.state, VisualServoState.FOLLOW)
        self.assertEqual(guarded.computed_channels[0], 992)
        self.assertGreater(guarded.computed_channels[3], 992)

    def test_soft_altitude_guard_requires_continuous_confirmation(self) -> None:
        """Краткий барометрический выброс не отменяет крен до подтверждения."""
        controller = VisualServoController(
            config(
                output_mode="real",
                roll_follow_enabled=True,
                altitude_guard_confirmation_s=0.30,
                roll_rate_slew_per_s=10000.0,
            )
        )
        frame, channels = rc_frame(throttle=1100)
        controller.process(frame, channels, "FOLLOW", target(0.0), sensor(0.0), 0.0, armed=True)

        pending = controller.process(
            frame,
            channels,
            "FOLLOW",
            target(0.10, x=1.0),
            sensor(0.10, altitude=1.65),
            0.10,
            armed=True,
        )
        self.assertGreater(pending.computed_channels[0], 992)
        self.assertFalse(any(event.startswith("ALTITUDE_PRIORITY") for event in pending.events))

        confirmed = controller.process(
            frame,
            channels,
            "FOLLOW",
            target(0.45, x=1.0),
            sensor(0.45, altitude=1.65),
            0.45,
            armed=True,
        )
        self.assertGreater(confirmed.computed_channels[0], 992)
        self.assertTrue(any(event.startswith("ALTITUDE_PRIORITY") for event in confirmed.events))

    def test_hard_altitude_guard_is_immediate_even_with_confirmation(self) -> None:
        """Опасная ошибка высоты сразу центрирует крен, не ожидая тайм-аут."""
        controller = VisualServoController(
            config(
                output_mode="real",
                roll_follow_enabled=True,
                altitude_guard_confirmation_s=1.0,
                roll_rate_slew_per_s=10000.0,
            )
        )
        frame, channels = rc_frame(throttle=1100)
        controller.process(frame, channels, "FOLLOW", target(0.0), sensor(0.0), 0.0, armed=True)
        guarded = controller.process(
            frame,
            channels,
            "FOLLOW",
            target(0.10, x=1.0),
            sensor(0.10, altitude=0.8),
            0.10,
            armed=True,
        )
        self.assertEqual(guarded.computed_channels[0], 992)
        self.assertTrue(any(event.startswith("ALTITUDE_PRIORITY") for event in guarded.events))

    def test_throttle_command_is_slewed(self) -> None:
        """Даже большая ошибка высоты не создаёт резкий скачок CH3."""
        controller = VisualServoController(
            config(output_mode="real", throttle_slew_per_s=100.0)
        )
        frame, channels = rc_frame(throttle=1000)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.1), sensor(0.1), 0.1, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.16), sensor(0.16), 0.16, armed=True)
        result = controller.process(
            frame, channels, "FOLLOW", target(0.22), sensor(0.22, altitude=0.0), 0.22, armed=True
        )
        self.assertLessEqual(result.computed_channels[2], 1012)

    def test_coordinated_yaw_and_altitude_commands_are_limited(self) -> None:
        """Прямой yaw по ошибке изображения остаётся ограниченным."""
        controller = VisualServoController(
            config(
                output_mode="real",
                roll_follow_enabled=True,
                roll_thrust_compensation_enabled=False,
            )
        )
        frame, channels = rc_frame(throttle=1100)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)
        controller.process(
            frame, channels, "FOLLOW", target(0.1, x=1.0), sensor(0.1, altitude=2.0), 0.1, armed=True
        )
        controller.process(
            frame, channels, "FOLLOW", target(0.2, x=1.0), sensor(0.2, altitude=2.0), 0.2, armed=True
        )
        controller.process(
            frame, channels, "FOLLOW", target(0.3, x=1.0), sensor(0.3, altitude=2.0), 0.3, armed=True
        )
        changed = controller.process(
            frame, channels, "FOLLOW", target(0.4, x=1.0), sensor(0.4, altitude=2.0), 0.4, armed=True
        )
        self.assertGreater(changed.computed_channels[0], 992)
        self.assertGreater(changed.computed_channels[3], 992)
        self.assertLessEqual(changed.computed_channels[3], 992 + 60)
        self.assertLessEqual(changed.computed_channels[2], 1220)
        self.assertEqual(extract_raw_frames(bytearray(changed.output_frame)), [changed.output_frame])

    def test_follow_combines_roll_yaw_and_pitch(self) -> None:
        """При боковой ошибке крен, yaw и движение к цели выдаются вместе."""
        controller = VisualServoController(
            config(
                output_mode="real",
                roll_follow_enabled=True,
                coordinated_yaw_kp=140.0,
                coordinated_yaw_max_correction=120,
                roll_thrust_compensation_enabled=False,
            )
        )
        frame, channels = rc_frame(throttle=1100)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.1), sensor(0.1), 0.1, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.2), sensor(0.2), 0.2, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.3), sensor(0.3), 0.3, armed=True)
        result = controller.process(
            frame,
            channels,
            "FOLLOW",
            target(0.4, x=0.6, scale=2.0),
            sensor(0.4),
            0.4,
            armed=True,
        )
        self.assertEqual(result.state, VisualServoState.FOLLOW)
        self.assertGreater(result.computed_channels[0], 992)  # крен вправо
        self.assertGreater(result.computed_channels[3], 992)  # yaw вправо
        self.assertGreater(result.computed_channels[1], 992)  # движение к цели

    def test_altitude_hold_keeps_yaw_centered_for_off_center_target(self) -> None:
        """До FOLLOW объект сбоку не имеет права повернуть нос дрона."""
        controller = VisualServoController(
            config(output_mode="real", altitude_hold_s=1.0, coordinated_yaw_kp=140.0)
        )
        frame, channels = rc_frame(throttle=1100)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)
        held = controller.process(
            frame, channels, "FOLLOW", target(0.1, x=1.0), sensor(0.1), 0.1, armed=True
        )
        self.assertEqual(held.state, VisualServoState.FOLLOW)
        self.assertGreater(held.computed_channels[3], 992)

    def test_follow_yaw_is_smoothed_and_slewed(self) -> None:
        """Yaw не перескакивает из центра к пределу за один видеокадр."""
        controller = VisualServoController(
            config(
                output_mode="real",
                roll_follow_enabled=True,
                altitude_hold_s=0.0,
                coordinated_yaw_kp=140.0,
                coordinated_yaw_max_correction=120,
                yaw_error_alpha=1.0,
                yaw_slew_per_s=100.0,
            )
        )
        frame, channels = rc_frame(throttle=1100)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.1), sensor(0.1), 0.1, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.2), sensor(0.2), 0.2, armed=True)
        moved = controller.process(
            frame, channels, "FOLLOW", target(0.3, x=1.0), sensor(0.3), 0.3, armed=True
        )
        self.assertEqual(moved.state, VisualServoState.FOLLOW)
        self.assertEqual(moved.computed_channels[3], 1002)
        centered = controller.process(
            frame, channels, "FOLLOW", target(0.4, x=0.0), sensor(0.4), 0.4, armed=True
        )
        self.assertEqual(centered.computed_channels[3], 992)

    def test_follow_roll_tracks_horizontal_target_with_limits(self) -> None:
        """В FOLLOW CH1 получает ограниченный крен по смещению цели."""
        controller = VisualServoController(
            config(roll_follow_enabled=True, roll_follow_kp=8.0, roll_follow_max_angle_deg=6.0)
        )
        frame, channels = rc_frame(throttle=1100)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.1), sensor(0.1), 0.1, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.16), sensor(0.16), 0.16, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.27), sensor(0.27), 0.27, armed=True)
        moved = controller.process(
            frame, channels, "FOLLOW", target(0.38, x=1.0), sensor(0.38), 0.38, armed=True
        )
        self.assertGreater(moved.computed_channels[0], 992)
        self.assertLessEqual(moved.computed_channels[0], 992 + 6 * 18)
        self.assertGreaterEqual(moved.computed_channels[2], 1100)

    def test_follow_roll_deadband_returns_to_level(self) -> None:
        """При цели в центре боковой крен возвращается к нулевому углу."""
        controller = VisualServoController(config(roll_follow_enabled=True))
        frame, channels = rc_frame(throttle=1100)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.1), sensor(0.1), 0.1, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.16), sensor(0.16), 0.16, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.27), sensor(0.27), 0.27, armed=True)
        centered = controller.process(
            frame, channels, "FOLLOW", target(0.38, x=0.01), sensor(0.38), 0.38, armed=True
        )
        self.assertEqual(centered.computed_channels[0], 992)
        self.assertEqual(centered.computed_channels[3], 992)

    def test_follow_roll_brakes_before_target_crosses_center(self) -> None:
        """Быстрое возвращение цели к центру создаёт ограниченный обратный крен."""
        controller = VisualServoController(
            config(
                roll_follow_enabled=True,
                roll_follow_kp=8.0,
                roll_follow_kd=0.0,
                roll_brake_lookahead_s=1.0,
                roll_brake_derivative_deadband=0.0,
                roll_angle_to_rate_kp=30.0,
                roll_rate_max_correction=200,
                roll_rate_slew_per_s=10000.0,
            )
        )
        frame, channels = rc_frame()
        controller.process(frame, channels, "FOLLOW", target(0.0), sensor(0.0), 0.0, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.11), sensor(0.11), 0.11, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.22), sensor(0.22), 0.22, armed=True)
        moving_right = controller.process(
            frame, channels, "FOLLOW", target(0.32, x=0.8), sensor(0.32), 0.32, armed=True
        )
        braking = controller.process(
            frame, channels, "FOLLOW", target(0.42, x=0.05), sensor(0.42), 0.42, armed=True
        )
        self.assertGreater(moving_right.computed_channels[0], 992)
        self.assertLess(braking.computed_channels[0], 992)

    def test_follow_preserves_reference_pitch_when_scale_is_stable(self) -> None:
        """Спокойный pitch пилота сохраняется как опора движения вперёд."""
        controller = VisualServoController(
            config(
                altitude_hold_s=0.0,
                forward_pitch_reference_enabled=True,
                forward_pitch_reference_max_deg=6.0,
                pitch_rate_slew_per_s=10000.0,
            )
        )
        frame, channels = rc_frame(pitch=1100)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0, pitch=4.0), 0.0, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.1), sensor(0.1, pitch=4.0), 0.1, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.2), sensor(0.2, pitch=4.0), 0.2, armed=True)
        result = controller.process(
            frame, channels, "FOLLOW", target(0.3, scale=4.0), sensor(0.3, pitch=0.0), 0.3, armed=True
        )
        self.assertEqual(result.state, VisualServoState.FOLLOW)
        self.assertGreater(result.computed_channels[1], 992)

    def test_follow_never_slows_when_object_grows(self) -> None:
        """Рост объекта сохраняет скорость, а уменьшение может её увеличить."""
        controller = VisualServoController(
            config(
                pitch_rate_slew_per_s=10000.0,
                pitch_deadband=0.0,
                pitch_kd=0.5,
            )
        )
        frame, channels = rc_frame(pitch=1100)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)

        controller.process(
            frame, channels, "FOLLOW", target(0.1, scale=4.0), sensor(0.1), 0.1, armed=True
        )
        stable = controller.process(
            frame, channels, "FOLLOW", target(0.2, scale=4.0), sensor(0.2), 0.2, armed=True
        )
        larger = controller.process(
            frame, channels, "FOLLOW", target(0.3, scale=8.0), sensor(0.3), 0.3, armed=True
        )
        smaller = controller.process(
            frame, channels, "FOLLOW", target(0.4, scale=2.0), sensor(0.4), 0.4, armed=True
        )

        self.assertGreaterEqual(larger.computed_channels[1], stable.computed_channels[1])
        self.assertGreater(smaller.computed_channels[1], stable.computed_channels[1])

    def test_follow_uses_median_actual_pitch_before_switch(self) -> None:
        """FOLLOW сохраняет угол корпуса, даже если CH2 уже вернулся в центр."""
        controller = VisualServoController(
            config(
                output_mode="real",
                forward_pitch_reference_window_s=0.35,
                pitch_rate_slew_per_s=10000.0,
            )
        )
        for now, pitch_deg in ((0.00, 6.8), (0.10, 7.3), (0.20, 7.1)):
            frame, channels = rc_frame(pitch=992)
            controller.process(
                frame,
                channels,
                "CAPTURE",
                None,
                sensor(now, pitch=pitch_deg),
                now,
                armed=True,
                takeover_allowed=False,
                reference_learning_allowed=True,
            )

        # В кадре FOLLOW CH2 в центре, но корпус продолжает лететь с +7.3°.
        frame, switched_channels = rc_frame(pitch=992)
        result = controller.process(
            frame,
            switched_channels,
            "FOLLOW",
            target(0.25, scale=4.0),
            sensor(0.25, pitch=7.3),
            0.25,
            armed=True,
        )
        self.assertEqual(result.state, VisualServoState.FOLLOW)
        # Разница между медианой +7.1° и текущими +7.3° даёт лишь малую
        # корректировку возврата к опоре, а не команду выравнивания к нулю.
        self.assertGreaterEqual(result.computed_channels[1], 987)
        self.assertLessEqual(result.computed_channels[1], 997)
        self.assertIn("pitch_ref=+7.10deg", result.events[0])
        self.assertIn("pitch_source=msp_median", result.events[0])

    def test_follow_uses_median_pilot_throttle_before_switch(self) -> None:
        """FOLLOW не теряет CH3 пилота из-за кадра переключения CH6."""
        controller = VisualServoController(
            config(output_mode="real", throttle_reference_window_s=0.35)
        )
        for now, throttle in ((0.00, 1050), (0.10, 1100), (0.20, 1150)):
            frame, channels = rc_frame(throttle=throttle)
            controller.process(frame, channels, "DIRECT", None, sensor(now), now, armed=True)

        # В кадре FOLLOW CH3 ошибочно попал в минимум: должна остаться
        # медиана ручного газа, то есть 1100.
        frame, switched_channels = rc_frame(throttle=191)
        result = controller.process(
            frame,
            switched_channels,
            "FOLLOW",
            target(0.25, scale=4.0),
            sensor(0.25),
            0.25,
            armed=True,
        )
        self.assertEqual(result.state, VisualServoState.FOLLOW)
        self.assertEqual(result.computed_channels[2], 1100)
        self.assertIn("CH3_ref=1100 throttle_source=pilot_median", result.events[0])

    def test_follow_first_frame_keeps_pilot_throttle_during_entry_blend(self) -> None:
        """Первый кадр FOLLOW не меняет CH3 даже с наклонённым корпусом."""
        controller = VisualServoController(
            config(output_mode="real", follow_entry_blend_s=0.35)
        )
        for now in (0.00, 0.10, 0.20):
            frame, channels = rc_frame(throttle=1080)
            controller.process(
                frame,
                channels,
                "CAPTURE",
                None,
                sensor(now, pitch=7.0),
                now,
                armed=True,
                takeover_allowed=False,
                reference_learning_allowed=True,
            )
        frame, channels = rc_frame(throttle=191)
        result = controller.process(
            frame,
            channels,
            "FOLLOW",
            target(0.25),
            sensor(0.25, pitch=7.0),
            0.25,
            armed=True,
        )
        self.assertEqual(result.computed_channels[2], 1080)
        self.assertIn("throttle_source=pilot_median", result.events[0])

    def test_altitude_priority_preserves_base_pitch_and_only_removes_visual_addition(self) -> None:
        """Защита высоты не должна обнулять сохранённый наклон пилота скачком."""
        controller = VisualServoController(
            config(
                output_mode="real",
                pitch_rate_slew_per_s=10000.0,
                altitude_guard_error_m=0.10,
                altitude_authority_hard_error_m=0.20,
                altitude_guard_confirmation_s=0.0,
            )
        )
        for now in (0.00, 0.10, 0.20):
            frame, channels = rc_frame()
            controller.process(
                frame,
                channels,
                "CAPTURE",
                None,
                sensor(now, altitude=2.0, pitch=4.0),
                now,
                armed=True,
                takeover_allowed=False,
                reference_learning_allowed=True,
            )
        frame, channels = rc_frame()
        result = controller.process(
            frame,
            channels,
            "FOLLOW",
            target(0.25),
            sensor(0.25, altitude=3.0, pitch=0.0),
            0.25,
            armed=True,
        )
        self.assertEqual(result.state, VisualServoState.FOLLOW)
        self.assertGreater(result.computed_channels[1], 992)

    def test_follow_keeps_low_throttle_without_creating_thrust(self) -> None:
        """Низкий CH3 пилота остаётся низким: FOLLOW сам не создаёт тягу."""
        controller = VisualServoController(config(output_mode="real"))
        frame, channels = rc_frame(throttle=191)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)
        result = controller.process(
            frame, channels, "FOLLOW", target(0.1), sensor(0.1), 0.1, armed=True
        )
        self.assertEqual(result.computed_channels[2], 191)
        self.assertIn("CH3_ref=191 throttle_source=pilot_median", result.events[0])

    def test_follow_ignores_stale_actual_pitch_before_switch(self) -> None:
        """Старая MSP-опора не переживает заданное окно перед FOLLOW."""
        controller = VisualServoController(
            config(output_mode="real", forward_pitch_reference_window_s=0.20)
        )
        old_frame, old_channels = rc_frame(pitch=992)
        controller.process(
            old_frame,
            old_channels,
            "DIRECT",
            None,
            sensor(0.0, pitch=7.0),
            0.0,
            armed=True,
        )
        frame, channels = rc_frame(pitch=992)
        result = controller.process(
            frame,
            channels,
            "FOLLOW",
            target(1.0, scale=4.0),
            sensor(1.0, pitch=0.0),
            1.0,
            armed=True,
        )
        self.assertIn("pitch_ref=+0.00deg", result.events[0])
        self.assertIn("pitch_source=msp_live_fallback", result.events[0])

    def test_target_loss_reports_reason_from_video_module(self) -> None:
        """Визуальный контур сохраняет точную причину, присланную камерой."""
        controller = VisualServoController(config(output_mode="real"))
        frame, channels = rc_frame()
        invalid = TargetGuidance(
            False, None, None, 0.1, "Цель потеряна", loss_reason="VERIFIER_REJECTED",
            loss_detail="capture_similarity=0.120 adaptive_similarity=0.140 bad_frames=16/16",
        )
        result = controller.process(frame, channels, "FOLLOW", invalid, sensor(0.1), 0.1, armed=True)
        self.assertEqual(result.state, VisualServoState.TARGET_LOST)
        self.assertIn("reason=VERIFIER_REJECTED", result.events[0])
        self.assertIn("bad_frames=16/16", result.events[0])

    def test_acro_roll_rate_command_is_limited_and_slewed(self) -> None:
        """В ACRO команда CH1 меняется ступенями и не превышает предел."""
        controller = VisualServoController(
            config(
                roll_follow_enabled=True,
                roll_angle_to_rate_kp=50.0,
                roll_rate_max_correction=40,
                roll_rate_slew_per_s=100.0,
            )
        )
        frame, channels = rc_frame()
        controller.process(frame, channels, "FOLLOW", target(0.0), sensor(0.0), 0.0, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.11), sensor(0.11), 0.11, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.22), sensor(0.22), 0.22, armed=True)
        first = controller.process(
            frame, channels, "FOLLOW", target(0.32, x=1.0), sensor(0.32), 0.32, armed=True
        )
        second = controller.process(
            frame, channels, "FOLLOW", target(0.42, x=1.0), sensor(0.42), 0.42, armed=True
        )
        self.assertEqual(first.state, VisualServoState.FOLLOW)
        self.assertEqual(first.computed_channels[0], 1002)
        self.assertEqual(second.computed_channels[0], 1012)
        self.assertLessEqual(second.computed_channels[0], 992 + 40)

    def test_alignment_corrects_measured_roll_and_pitch(self) -> None:
        """До движения к цели корпус выравнивается по фактическим углам MSP."""
        controller = VisualServoController(
            config(
                altitude_reference_max_tilt_deg=20.0,
                forward_pitch_reference_enabled=False,
            )
        )
        frame, channels = rc_frame()
        result = controller.process(
            frame,
            channels,
            "FOLLOW",
            target(0.0),
            sensor(0.0, roll=10.0, pitch=-5.0),
            0.0,
            armed=True,
        )
        self.assertLess(result.computed_channels[0], 992)
        self.assertGreater(result.computed_channels[1], 992)
        self.assertEqual(result.state, VisualServoState.FOLLOW)

    def test_dry_run_calculates_but_does_not_change_frame(self) -> None:
        """Ноутбучный режим показывает расчёт, не выдавая его в транспорт."""
        controller = VisualServoController(config(output_mode="dry-run", roll_follow_enabled=True))
        frame, channels = rc_frame()
        controller.process(frame, channels, "FOLLOW", target(0.0), sensor(0.0), 0.0, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.11), sensor(0.11), 0.11, armed=True)
        result = controller.process(
            frame, channels, "FOLLOW", target(0.22, x=0.5), sensor(0.22), 0.22, armed=True
        )
        self.assertEqual(result.output_frame, frame)
        self.assertFalse(result.output_applied)
        self.assertNotEqual(result.computed_channels[0], channels[0])
        self.assertNotEqual(result.computed_channels[3], channels[3])

    def test_repeated_setpoint_preserves_fresh_aux_channels(self) -> None:
        """Между расчётами меняются только CH1–CH4, а новые AUX проходят в FC."""
        controller = VisualServoController(config(output_mode="real"))
        first_frame, first_channels = rc_frame(ch7=191)
        controller.process(
            first_frame,
            first_channels,
            "FOLLOW",
            target(0.0, x=0.5),
            sensor(0.0),
            0.0,
            armed=True,
        )
        next_frame, next_channels = rc_frame(ch7=1792)
        repeated = controller.process(
            next_frame,
            next_channels,
            "FOLLOW",
            target(0.01, x=0.5),
            sensor(0.01),
            0.01,
            armed=True,
        )
        transmitted = unpack_channels(repeated.output_frame[3:-1])
        self.assertEqual(transmitted[6], 1792)
        self.assertEqual(transmitted[4], next_channels[4])

    def test_target_loss_is_latched_until_pilot_leaves_follow(self) -> None:
        """Потерянная цель не подменяется новой без команды пилота."""
        controller = VisualServoController(config(output_mode="real"))
        frame, channels = rc_frame()
        controller.process(frame, channels, "FOLLOW", target(0.0), sensor(0.0), 0.0, armed=True)
        lost = controller.process(frame, channels, "FOLLOW", None, sensor(0.1), 0.1, armed=True)
        self.assertEqual(lost.state, VisualServoState.TARGET_LOST)
        self.assertEqual(lost.output_frame, frame)
        still_lost = controller.process(
            frame, channels, "FOLLOW", target(0.2), sensor(0.2), 0.2, armed=True
        )
        self.assertEqual(still_lost.state, VisualServoState.TARGET_LOST)
        controller.process(frame, channels, "DIRECT", None, sensor(0.3), 0.3, armed=True)
        restarted = controller.process(
            frame, channels, "FOLLOW", target(0.4), sensor(0.4), 0.4, armed=True
        )
        self.assertEqual(restarted.state, VisualServoState.FOLLOW)

    def test_transient_stale_sensor_recovers_without_fault(self) -> None:
        """Краткий пропуск MSP повторяет последнюю команду и восстанавливается."""
        controller = VisualServoController(config(output_mode="real"))
        frame, channels = rc_frame()
        controller.process(frame, channels, "FOLLOW", target(0.0), sensor(0.0), 0.0, armed=True)
        delayed = controller.process(
            frame, channels, "FOLLOW", target(0.4), sensor(0.0), 0.4, armed=True
        )
        self.assertNotEqual(delayed.state, VisualServoState.FAULT)
        self.assertTrue(delayed.output_applied)
        self.assertIn("MSP DATA DELAY", delayed.events[0])
        restored = controller.process(
            frame, channels, "FOLLOW", target(0.45), sensor(0.45), 0.45, armed=True
        )
        self.assertNotEqual(restored.state, VisualServoState.FAULT)
        self.assertIn("MSP DATA RESTORED", restored.events[0])

    def test_persistent_stale_sensor_latches_fault_after_confirmations(self) -> None:
        """Несколько последовательных пропусков MSP переводят контур в FAULT."""
        controller = VisualServoController(config(output_mode="real"))
        frame, channels = rc_frame()
        controller.process(frame, channels, "FOLLOW", target(0.0), sensor(0.0), 0.0, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.4), sensor(0.0), 0.4, armed=True)
        controller.process(frame, channels, "FOLLOW", target(0.8), sensor(0.0), 0.8, armed=True)
        result = controller.process(
            frame, channels, "FOLLOW", target(1.2), sensor(0.0), 1.2, armed=True
        )
        self.assertEqual(result.state, VisualServoState.FAULT)
        self.assertTrue(result.fault)
        self.assertEqual(result.output_frame, frame)
        self.assertIn("age=1.200s", result.events[0])

    def test_many_queued_frames_do_not_confirm_one_msp_gap(self) -> None:
        """Несколько кадров за один момент не превращают один MSP GAP в FAULT."""
        controller = VisualServoController(config(output_mode="real"))
        frame, channels = rc_frame()
        controller.process(frame, channels, "FOLLOW", target(0.0), sensor(0.0), 0.0, armed=True)
        first = controller.process(frame, channels, "FOLLOW", target(0.4), sensor(0.0), 0.4, armed=True)
        second = controller.process(frame, channels, "FOLLOW", target(0.401), sensor(0.0), 0.401, armed=True)
        third = controller.process(frame, channels, "FOLLOW", target(0.402), sensor(0.0), 0.402, armed=True)
        self.assertNotEqual(first.state, VisualServoState.FAULT)
        self.assertNotEqual(second.state, VisualServoState.FAULT)
        self.assertNotEqual(third.state, VisualServoState.FAULT)
        self.assertEqual(second.events, ())
        self.assertEqual(third.events, ())

    def test_disarm_resets_controller_without_modifying_disarm_frame(self) -> None:
        """DISARM имеет приоритет над каждым состоянием visual_servoing."""
        controller = VisualServoController(config(output_mode="real"))
        frame, channels = rc_frame()
        controller.process(frame, channels, "FOLLOW", target(0.0), sensor(0.0), 0.0, armed=True)
        disarm_frame, disarm_channels = rc_frame(ch5=191)
        result = controller.process(
            disarm_frame, disarm_channels, "FOLLOW", target(0.1), sensor(0.1), 0.1, armed=False
        )
        self.assertEqual(result.state, VisualServoState.DIRECT)
        self.assertEqual(result.output_frame, disarm_frame)

    def test_configuration_rejects_overlapping_channels(self) -> None:
        """Один физический канал нельзя назначить двум органам управления."""
        invalid = dataclasses.replace(config(), pitch_channel=0)
        with self.assertRaises(ValueError):
            VisualServoController(invalid)

    def test_project_toml_builds_working_real_controller(self) -> None:
        """Единый TOML содержит полный стендовый real-контур."""
        visual_section = load_config_section("config/dront16.toml", "visual_servoing")
        target_section = load_config_section("config/dront16.toml", "follow", "control")
        msp_section = load_config_section("config/dront16.toml", "msp")
        loaded = build_visual_servo_config(
            visual_section,
            target_max_age_s=int(target_section["target_max_age_ms"]) / 1000.0,
            sensor_max_age_s=int(msp_section["sensor_max_age_ms"]) / 1000.0,
        )
        self.assertTrue(loaded.enabled)
        self.assertEqual(loaded.output_mode, "real")
        self.assertAlmostEqual(loaded.control_period_s, 0.05)
        self.assertTrue(loaded.roll_follow_enabled)
        self.assertTrue(loaded.coordinated_yaw_enabled)
        self.assertTrue(loaded.roll_thrust_compensation_enabled)
        self.assertAlmostEqual(loaded.roll_follow_deadband, 0.02)
        self.assertAlmostEqual(loaded.roll_follow_max_angle_deg, 14.0)
        self.assertEqual(loaded.roll_rate_max_correction, 160)
        self.assertAlmostEqual(loaded.altitude_guard_confirmation_s, 0.35)


if __name__ == "__main__":
    unittest.main()
