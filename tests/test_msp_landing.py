"""Проверки MSP-разбора и посадочного автомата без подключения железа."""

import struct
import unittest

from src.control.landing import LandingConfig, LandingController, LandingState
from src.protocols.betaflight_msp_link import MSP_ALTITUDE, MSP_ATTITUDE, MspParser, SensorSample, msp_checksum
from src.receiver.crsf import CRSF_RC_CHANNELS_PACKED, crc8_dvb_s2, extract_raw_frames, pack_channels, unpack_channels


def response(command: int, payload: bytes) -> bytes:
    """Формирует синтетический ответ MSP v1 с корректной контрольной суммой."""
    return b"$M>" + bytes((len(payload), command)) + payload + bytes((msp_checksum(len(payload), command, payload),))


def frame(channels: tuple[int, ...]) -> bytes:
    """Формирует проверенный кадр CRSF для посадочного теста."""
    payload = pack_channels(channels)
    body = bytes((0xC8, len(payload) + 2, CRSF_RC_CHANNELS_PACKED)) + payload
    return body + bytes((crc8_dvb_s2(body[2:]),))


class MspLandingTests(unittest.TestCase):
    """Проверяет чтение датчиков и ограниченные RC-команды посадки."""

    def test_msp_parser_reads_attitude_and_altitude(self) -> None:
        """Парсер объединяет два MSP-ответа в полный образец датчиков."""
        parser = MspParser()
        data = response(MSP_ATTITUDE, struct.pack("<hhh", 50, -20, 123))
        data += response(MSP_ALTITUDE, struct.pack("<ih", 235, -12))
        samples = parser.feed(data, received_at=10.0)
        self.assertEqual(len(samples), 2)
        sample = samples[-1]
        self.assertTrue(sample.complete)
        self.assertAlmostEqual(sample.roll_deg or 0, 5.0)
        self.assertAlmostEqual(sample.pitch_deg or 0, -2.0)
        self.assertAlmostEqual(sample.altitude_m or 0, 2.35)
        self.assertAlmostEqual(sample.vario_m_s or 0, -0.12)
        self.assertAlmostEqual(sample.altitude_received_at or 0, 10.0)
        self.assertAlmostEqual(sample.attitude_received_at or 0, 10.0)

    def test_msp_parser_rejects_bad_crc_and_supports_fragmented_input(self) -> None:
        """Парсер отбрасывает битый пакет и собирает разделённый пакет."""
        parser = MspParser()
        attitude = response(MSP_ATTITUDE, struct.pack("<hhh", 50, -20, 123))
        broken = attitude[:-1] + bytes((attitude[-1] ^ 0xFF,))
        self.assertEqual(parser.feed(broken, received_at=1.0), [])
        self.assertEqual(parser.feed(attitude[:4], received_at=2.0), [])
        samples = parser.feed(attitude[4:], received_at=2.0)
        self.assertEqual(len(samples), 1)
        self.assertFalse(samples[-1].complete)

    def test_msp_freshness_is_checked_per_sensor_type(self) -> None:
        """Свежий крен не маскирует устаревшую высоту."""
        parser = MspParser()
        altitude = response(MSP_ALTITUDE, struct.pack("<ih", 235, -12))
        attitude = response(MSP_ATTITUDE, struct.pack("<hhh", 50, -20, 123))
        parser.feed(altitude, received_at=1.0)
        sample = parser.feed(attitude, received_at=2.0)[0]
        self.assertTrue(sample.complete)
        self.assertFalse(sample.is_fresh(2.5, 1.0))
        self.assertTrue(sample.is_fresh(2.0, 1.0))

    def test_landing_levels_and_ramps_throttle(self) -> None:
        """Посадка корректирует наклон и медленно снижает газ с заданной скоростью."""
        config = LandingConfig(0, 1, 2, 992, 191, 1792, 0.0, 0.0, 18.0, 250, 2.0, 40.0, 191, 0.25, 0.30, 0.15, 1.5, "HOLD", 4, 191, None, 1500)
        controller = LandingController(config)
        channels = [992] * 16
        channels[0] = 1200
        channels[1] = 900
        channels[2] = 1500
        current = frame(tuple(channels))
        sample = SensorSample(2.5, 0.0, 5.0, -2.0, 0.0, 0.0, True, True, 0.0, 0.0)
        first = controller.process(current, tuple(channels), "TAKEOVER", sample, 0.0)
        self.assertEqual(first.state, LandingState.LEVELING)
        self.assertEqual(unpack_channels(first.output_frame[3:-1])[0], 902)
        second_sample = SensorSample(2.5, 0.0, 5.0, -2.0, 0.0, 2.1, True, True, 2.1, 2.1)
        second = controller.process(current, tuple(channels), "TAKEOVER", second_sample, 2.1)
        self.assertEqual(second.state, LandingState.DESCENT)
        self.assertEqual(second.throttle_command, 1416)

    def test_stale_sensor_enters_fault_without_new_descent(self) -> None:
        """Устаревший MSP-образец переводит автомат в FAULT."""
        config = LandingConfig(0, 1, 2, 992, 191, 1792, 0.0, 0.0, 10.0, 100, 1.0, 50.0, 191, 0.25, 0.30, 0.15, 1.5, "HOLD", 4, 191, None, 1500)
        controller = LandingController(config)
        channels = tuple([992] * 16)
        stale = SensorSample(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, True, True, 0.0, 0.0)
        current = frame(channels)
        result = controller.process(current, channels, "TAKEOVER", stale, 1.0)
        self.assertEqual(result.state, LandingState.FAULT)
        held = controller.process(current, channels, "TAKEOVER", stale, 2.0)
        self.assertEqual(held.output_frame, result.output_frame)

    def test_fault_can_request_disarm(self) -> None:
        """При fault_action DISARM формируется корректный кадр остановки моторов."""
        config = LandingConfig(0, 1, 2, 992, 191, 1792, 0.0, 0.0, 10.0, 100, 1.0, 50.0, 191, 0.25, 0.30, 0.15, 1.5, "DISARM", 4, 191, None, 1500)
        controller = LandingController(config)
        channels = tuple([992] * 16)
        current = frame(channels)
        result = controller.process(current, channels, "TAKEOVER", None, 1.0)
        self.assertTrue(result.disarm_requested)
        self.assertEqual(unpack_channels(result.output_frame[3:-1])[4], 191)
        self.assertEqual(extract_raw_frames(bytearray(result.output_frame)), [result.output_frame])

    def test_stabilization_aux_is_configurable(self) -> None:
        """Посадка может выставить заранее настроенный AUX режима стабилизации."""
        config = LandingConfig(0, 1, 2, 992, 191, 1792, 0.0, 0.0, 10.0, 100, 1.0, 50.0, 191, 0.25, 0.30, 0.15, 1.5, "HOLD", 4, 191, 7, 1792)
        controller = LandingController(config)
        channels = tuple([992] * 16)
        sample = SensorSample(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, True, True, 0.0, 0.0)
        result = controller.process(frame(channels), channels, "TAKEOVER", sample, 0.0)
        self.assertEqual(unpack_channels(result.output_frame[3:-1])[7], 1792)

    def test_arm_captures_altitude_zero_and_tilt_pauses_descent(self) -> None:
        """ARM фиксирует ноль, а наклон не позволяет начать снижение."""
        config = LandingConfig(0, 1, 2, 992, 191, 1792, 0.0, 0.0, 10.0, 100, 0.1, 50.0, 191, 0.25, 0.30, 0.15, 1.5, "HOLD", 4, 191, None, 1500, 2.0, 2.0, 1.0)
        controller = LandingController(config)
        channels = [992] * 16
        channels[2] = 1500
        current = frame(tuple(channels))
        armed_level = SensorSample(10.0, 0.0, 0.0, 0.0, 0.0, 0.0, True, True, 0.0, 0.0)
        controller.process(current, tuple(channels), "LIVE", armed_level, 0.0, armed=True)
        tilted = SensorSample(11.0, 0.0, 8.0, 0.0, 0.0, 2.0, True, True, 2.0, 2.0)
        result = controller.process(current, tuple(channels), "TAKEOVER", tilted, 2.0, armed=True)
        self.assertEqual(result.state, LandingState.LEVELING)
        self.assertTrue(any("descent paused" in event for event in result.events))
        self.assertAlmostEqual(controller.relative_altitude(tilted) or 0, 1.0)


if __name__ == "__main__":
    unittest.main()
