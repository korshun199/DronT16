"""Проверки MSP-разбора и посадочного автомата без подключения железа."""

import struct
import unittest

from src.control.landing import LandingConfig, LandingController, LandingState
from src.protocols.betaflight_msp_link import MSP_ALTITUDE, MSP_ATTITUDE, MspParser, SensorSample, msp_checksum
from src.receiver.crsf import CRSF_RC_CHANNELS_PACKED, crc8_dvb_s2, pack_channels, unpack_channels


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

    def test_landing_levels_and_ramps_throttle(self) -> None:
        """Посадка корректирует наклон и снижает газ с заданной скоростью."""
        config = LandingConfig(0, 1, 2, 992, 191, 1792, 0.0, 0.0, 18.0, 250, 2.0, 80.0, 191, 0.25, 0.30, 0.15, 1.5, "HOLD")
        controller = LandingController(config)
        channels = [992] * 16
        channels[0] = 1200
        channels[1] = 900
        channels[2] = 1500
        current = frame(tuple(channels))
        sample = SensorSample(2.5, 0.0, 5.0, -2.0, 0.0, 0.0, True, True)
        first = controller.process(current, tuple(channels), "TAKEOVER", sample, 0.0)
        self.assertEqual(first.state, LandingState.LEVELING)
        self.assertEqual(unpack_channels(first.output_frame[3:-1])[0], 902)
        second_sample = SensorSample(2.5, 0.0, 5.0, -2.0, 0.0, 2.1, True, True)
        second = controller.process(current, tuple(channels), "TAKEOVER", second_sample, 2.1)
        self.assertEqual(second.state, LandingState.DESCENT)
        self.assertEqual(second.throttle_command, 1332)

    def test_stale_sensor_enters_fault_without_new_descent(self) -> None:
        """Устаревший MSP-образец переводит автомат в FAULT."""
        config = LandingConfig(0, 1, 2, 992, 191, 1792, 0.0, 0.0, 10.0, 100, 1.0, 50.0, 191, 0.25, 0.30, 0.15, 1.5, "HOLD")
        controller = LandingController(config)
        channels = tuple([992] * 16)
        stale = SensorSample(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, True, True)
        current = frame(channels)
        result = controller.process(current, channels, "TAKEOVER", stale, 1.0)
        self.assertEqual(result.state, LandingState.FAULT)
        held = controller.process(current, channels, "TAKEOVER", stale, 2.0)
        self.assertEqual(held.output_frame, result.output_frame)


if __name__ == "__main__":
    unittest.main()
