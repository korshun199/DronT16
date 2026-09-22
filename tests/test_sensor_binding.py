"""Проверки GPS и магнитных полей в общем образце MSP-датчиков."""

import struct
import unittest

from src.protocols.betaflight_msp_link import (
    MSP_RAW_GPS,
    MSP_RAW_IMU,
    MspParser,
    msp_checksum,
)


def response(command: int, payload: bytes) -> bytes:
    """Формирует MSPv1-ответ для тестового потока."""
    return b"$M>" + bytes((len(payload), command)) + payload + bytes((msp_checksum(len(payload), command, payload),))


class SensorBindingTests(unittest.TestCase):
    """Проверяет единицы GPS, диагностический курс и свежесть данных."""

    def test_parser_reads_gps_and_magnetometer_from_split_stream(self) -> None:
        """GPS и магнитометр собираются даже при дроблении UART-потока."""
        parser = MspParser()
        raw_imu = struct.pack("<hhhhhhhhh", 0, 0, 1000, 0, 0, 0, 0, 1000, -1200)
        raw_gps = struct.pack("<BBiihHHH", 3, 10, 557_522_000, 376_156_000, 180, 125, 900, 120)
        stream = response(MSP_RAW_IMU, raw_imu) + response(MSP_RAW_GPS, raw_gps)
        samples = []
        for offset in range(0, len(stream), 3):
            samples.extend(parser.feed(stream[offset : offset + 3], received_at=4.0))

        sample = samples[-1]
        self.assertTrue(sample.gps_valid)
        self.assertTrue(sample.gps_is_fresh(4.1, 0.25))
        self.assertAlmostEqual(sample.gps_latitude_deg or 0.0, 55.7522)
        self.assertAlmostEqual(sample.gps_longitude_deg or 0.0, 37.6156)
        self.assertAlmostEqual(sample.gps_speed_m_s or 0.0, 1.25)
        self.assertAlmostEqual(sample.gps_course_deg or 0.0, 90.0)
        self.assertAlmostEqual(sample.magnetic_heading_deg or 0.0, 90.0)
        self.assertFalse(sample.gps_is_fresh(4.3, 0.25))

    def test_gps_without_fix_is_not_valid(self) -> None:
        """Координаты без 3D-fix не становятся источником управления."""
        parser = MspParser()
        payload = struct.pack("<BBiihHH", 0, 3, 557_522_000, 376_156_000, 180, 0, 0)
        sample = parser.feed(response(MSP_RAW_GPS, payload), received_at=1.0)[0]
        self.assertFalse(sample.gps_valid)


if __name__ == "__main__":
    unittest.main()
