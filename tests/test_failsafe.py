"""Проверки MSP и удержания после потери связи без режима посадки."""

import struct
import unittest

from src.control.failsafe import FailsafeConfig, FailsafeController, FailsafeState
from src.protocols.betaflight_msp_link import (
    MSP_ALTITUDE,
    MSP_ATTITUDE,
    MspParser,
    SensorSample,
    msp_checksum,
)
from src.receiver.crsf import CRSF_RC_CHANNELS_PACKED, crc8_dvb_s2, pack_channels, unpack_channels


def response(command: int, payload: bytes) -> bytes:
    """Формирует синтетический MSPv1-ответ с корректной контрольной суммой."""
    return b"$M>" + bytes((len(payload), command)) + payload + bytes((msp_checksum(len(payload), command, payload),))


def frame(channels: tuple[int, ...]) -> bytes:
    """Формирует проверенный CRSF-кадр каналов."""
    payload = pack_channels(channels)
    body = bytes((0xC8, len(payload) + 2, CRSF_RC_CHANNELS_PACKED)) + payload
    return body + bytes((crc8_dvb_s2(body[2:]),))


def sensor(altitude: float = 2.0, roll: float = 0.0, pitch: float = 0.0, yaw: float = 10.0, received_at: float = 0.0) -> SensorSample:
    """Создаёт полный образец высоты и положения для теста."""
    return SensorSample(altitude, 0.0, roll, pitch, yaw, received_at, True, True, received_at, received_at)


def config(**overrides: object) -> FailsafeConfig:
    """Создаёт компактную тестовую конфигурацию failsafe."""
    values: dict[str, object] = {
        "roll_channel": 0,
        "pitch_channel": 1,
        "throttle_channel": 2,
        "yaw_channel": 3,
        "rc_center": 992,
        "rc_min": 191,
        "rc_max": 1792,
        "target_roll_deg": 0.0,
        "target_pitch_deg": 0.0,
        "correction_per_degree": 18.0,
        "max_correction": 250,
        "sensor_max_age_s": 0.25,
        "fault_action": "HOLD",
        "disarm_channel": 4,
        "disarm_value": 191,
        "stabilization_channel": None,
        "stabilization_value": 1500,
        "level_hold_s": 1.0,
        "level_throttle": 992,
        "turn_yaw_command": 1155,
    }
    values.update(overrides)
    return FailsafeConfig(**values)  # type: ignore[arg-type]


class FailsafeTests(unittest.TestCase):
    """Проверяет удержание высоты, выравнивание, разворот и безопасный fault."""

    def test_msp_parser_reads_attitude_and_altitude(self) -> None:
        """Парсер объединяет ответы MSP в полный образец датчиков."""
        parser = MspParser()
        data = response(MSP_ATTITUDE, struct.pack("<hhh", 50, -20, 123))
        data += response(MSP_ALTITUDE, struct.pack("<ih", 235, -12))
        samples = parser.feed(data, received_at=10.0)
        self.assertEqual(len(samples), 2)
        self.assertTrue(samples[-1].complete)
        self.assertAlmostEqual(samples[-1].altitude_m or 0.0, 2.35)
        self.assertAlmostEqual(samples[-1].roll_deg or 0.0, 5.0)

    def test_loss_turns_until_yaw_target_then_holds(self) -> None:
        """После CH7 контроллер крутит до фактического угла yaw и затем держит высоту."""
        controller = FailsafeController(config())
        channels = tuple([992] * 16)
        current = frame(channels)

        live = controller.process(current, channels, "LIVE", sensor(received_at=0.0), 0.0)
        self.assertEqual(live.state, FailsafeState.LIVE)
        leveling = controller.process(current, channels, "TAKEOVER", sensor(received_at=0.0), 0.0)
        self.assertEqual(leveling.state, FailsafeState.LEVELING)

        turning = controller.process(current, channels, "TAKEOVER", sensor(received_at=1.1), 1.1)
        self.assertEqual(turning.state, FailsafeState.TURNING)
        self.assertEqual(turning.yaw_command, 1155)

        still_turning = controller.process(current, channels, "TAKEOVER", sensor(yaw=20.0, received_at=10.0), 10.0)
        self.assertEqual(still_turning.state, FailsafeState.TURNING)
        self.assertEqual(still_turning.yaw_command, 1155)
        holding = controller.process(current, channels, "TAKEOVER", sensor(yaw=190.0, received_at=10.2), 10.2)
        self.assertEqual(holding.state, FailsafeState.HOLDING)
        self.assertTrue(any("TURNING -> HOLDING" in event for event in holding.events))
        self.assertTrue(any("yaw_target=180.0deg" in event for event in holding.events))
        self.assertEqual(holding.yaw_command, 992)
        self.assertNotIn("FAULT", " ".join(holding.events))

        restored = controller.process(current, channels, "LIVE", sensor(received_at=5.0), 5.0)
        self.assertEqual(restored.state, FailsafeState.LIVE)

    def test_hold_altitude_reacts_to_baro(self) -> None:
        """Ошибка барометра и вариометр меняют газ удержания, а не запускают снижение."""
        controller = FailsafeController(config())
        channels = tuple([992] * 16)
        current = frame(channels)
        controller.process(current, channels, "LIVE", sensor(altitude=10.0), 0.0)
        baseline = controller.process(current, channels, "TAKEOVER", sensor(altitude=10.0, received_at=0.0), 0.0)
        lower = controller.process(baseline.output_frame, channels, "TAKEOVER", sensor(altitude=9.5, received_at=0.1), 0.1)
        higher = controller.process(lower.output_frame, channels, "TAKEOVER", sensor(altitude=10.5, received_at=0.2), 0.2)
        self.assertGreater(lower.throttle_command, 992)
        self.assertLess(higher.throttle_command, lower.throttle_command)

    def test_altitude_zero_is_latched_until_disarm(self) -> None:
        """Нулевая высота фиксируется при ARM и не пересчитывается в LIVE/TAKEOVER."""
        controller = FailsafeController(config())
        channels = tuple([992] * 16)
        current = frame(channels)

        armed = controller.process(
            current, channels, "LIVE", sensor(altitude=1.0, received_at=0.0), 0.0, armed=True
        )
        self.assertIn("ALTITUDE_ZERO=1.00m", armed.events)

        live_again = controller.process(
            current, channels, "LIVE", sensor(altitude=2.0, received_at=0.1), 0.1, armed=True
        )
        self.assertNotIn("ALTITUDE_ZERO=2.00m", live_again.events)
        self.assertAlmostEqual(controller.relative_altitude(sensor(altitude=2.0)), 1.0)

        takeover = controller.process(
            current, channels, "TAKEOVER", sensor(altitude=2.0, received_at=0.2), 0.2, armed=True
        )
        self.assertNotIn("ALTITUDE_ZERO=2.00m", takeover.events)
        self.assertAlmostEqual(controller.altitude_zero_m or 0.0, 1.0)

        disarmed = controller.process(
            current, channels, "LIVE", sensor(altitude=2.0, received_at=0.3), 0.3, armed=False
        )
        self.assertEqual(disarmed.state, FailsafeState.LIVE)
        self.assertIsNone(controller.altitude_zero_m)

        rearmed = controller.process(
            current, channels, "LIVE", sensor(altitude=3.0, received_at=0.4), 0.4, armed=True
        )
        self.assertIn("ALTITUDE_ZERO=3.00m", rearmed.events)

    def test_stale_sensor_enters_fault_and_can_disarm(self) -> None:
        """Устаревшие MSP-данные переводят систему в FAULT с выбранным действием."""
        controller = FailsafeController(config(fault_action="DISARM"))
        channels = tuple([992] * 16)
        result = controller.process(frame(channels), channels, "TAKEOVER", None, 1.0)
        self.assertEqual(result.state, FailsafeState.FAULT)
        self.assertTrue(result.disarm_requested)
        self.assertEqual(unpack_channels(result.output_frame[3:-1])[4], 191)


if __name__ == "__main__":
    unittest.main()
