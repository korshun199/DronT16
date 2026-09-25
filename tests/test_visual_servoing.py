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
        "yaw_deadband": 0.04,
        "yaw_kp": 200.0,
        "yaw_kd": 0.0,
        "yaw_max_correction": 180,
        "yaw_direction": 1.0,
        "derivative_alpha": 0.25,
        "yaw_stable_frames": 2,
        "yaw_hold_s": 0.1,
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
        "altitude_kp": 80.0,
        "altitude_ki": 12.0,
        "altitude_vario_gain": 35.0,
        "altitude_integral_limit": 2.0,
        "throttle_max_correction": 120,
        "pitch_deadband": 0.05,
        "pitch_kp": 6.0,
        "pitch_kd": 0.0,
        "pitch_max_angle_deg": 8.0,
        "pitch_direction": 1.0,
        "hover_learning_alpha": 0.1,
        "hover_learning_max_vario_m_s": 0.2,
        "hover_learning_max_tilt_deg": 8.0,
        "hover_learning_min_throttle": 600,
        "hover_learning_max_throttle": 1500,
    }
    values.update(overrides)
    return VisualServoConfig(**values)  # type: ignore[arg-type]


def rc_frame(throttle: int = 1100, ch5: int = 1792, ch6: int = 1792,
             ch7: int = 191) -> tuple[bytes, tuple[int, ...]]:
    """Формирует полный проверенный CRSF-кадр для теста."""
    channels = [992] * 16
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

    def test_follow_passes_all_approved_states_before_pitch(self) -> None:
        """Pitch разрешается только после yaw, горизонта и удержания высоты."""
        controller = VisualServoController(config())
        frame, channels = rc_frame(throttle=1100)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)

        first = controller.process(frame, channels, "FOLLOW", target(0.1), sensor(0.1), 0.1, armed=True)
        self.assertEqual(first.state, VisualServoState.YAW_ALIGN)
        self.assertEqual(first.computed_channels[1], 992)
        yaw_hold = controller.process(frame, channels, "FOLLOW", target(0.16), sensor(0.16), 0.16, armed=True)
        self.assertEqual(yaw_hold.state, VisualServoState.YAW_HOLD)
        altitude = controller.process(frame, channels, "FOLLOW", target(0.27), sensor(0.27), 0.27, armed=True)
        self.assertEqual(altitude.state, VisualServoState.ALTITUDE_HOLD)
        follow = controller.process(frame, channels, "FOLLOW", target(0.38), sensor(0.38), 0.38, armed=True)
        self.assertEqual(follow.state, VisualServoState.FOLLOW)
        moved = controller.process(
            frame, channels, "FOLLOW", target(0.44, scale=2.0), sensor(0.44), 0.44, armed=True
        )
        self.assertGreater(moved.computed_channels[1], 992)

    def test_yaw_and_altitude_commands_are_limited(self) -> None:
        """Большая ошибка изображения и высоты не выходит за заданные пределы."""
        controller = VisualServoController(config(output_mode="real"))
        frame, channels = rc_frame(throttle=1100)
        controller.process(frame, channels, "DIRECT", None, sensor(0.0), 0.0, armed=True)
        started = controller.process(
            frame, channels, "FOLLOW", target(0.1, x=1.0), sensor(0.1, altitude=2.0), 0.1, armed=True
        )
        changed = controller.process(
            frame, channels, "FOLLOW", target(0.2, x=1.0), sensor(0.2, altitude=0.0), 0.2, armed=True
        )
        self.assertEqual(started.computed_channels[3], 1172)
        self.assertLessEqual(changed.computed_channels[2], 1220)
        self.assertEqual(extract_raw_frames(bytearray(changed.output_frame)), [changed.output_frame])

    def test_alignment_corrects_measured_roll_and_pitch(self) -> None:
        """До движения к цели корпус выравнивается по фактическим углам MSP."""
        controller = VisualServoController(config())
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
        self.assertEqual(result.state, VisualServoState.YAW_ALIGN)

    def test_dry_run_calculates_but_does_not_change_frame(self) -> None:
        """Ноутбучный режим показывает расчёт, не выдавая его в транспорт."""
        controller = VisualServoController(config(output_mode="dry-run"))
        frame, channels = rc_frame()
        result = controller.process(
            frame, channels, "FOLLOW", target(0.0, x=0.5), sensor(0.0), 0.0, armed=True
        )
        self.assertEqual(result.output_frame, frame)
        self.assertFalse(result.output_applied)
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
        self.assertEqual(restarted.state, VisualServoState.YAW_ALIGN)

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


if __name__ == "__main__":
    unittest.main()
