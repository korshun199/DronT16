"""Проверки безопасного применения команды зрения в быстром CRSF-тракте."""

import time
import unittest

from src.receiver.crsf import CRSF_RC_CHANNELS_PACKED, crc8_dvb_s2, pack_channels, unpack_channels
from src.receiver.realtime_bridge import RealtimeCrsfBridge


def frame(ch5: int = 1792, ch6: int = 1792) -> bytes:
    """Создаёт CRC-проверенный RC-кадр с заданными ARM и режимом FOLLOW."""
    channels = [992] * 16
    channels[2] = 174
    channels[4] = ch5
    channels[5] = ch6
    payload = pack_channels(channels)
    body = bytes((0xC8, len(payload) + 2, CRSF_RC_CHANNELS_PACKED)) + payload
    return body + bytes((crc8_dvb_s2(body[2:]),))


class RealtimeBridgeTests(unittest.TestCase):
    """Проверяет, что быстрый тракт не задерживает и не обходит пилота."""

    def setUp(self) -> None:
        """Создаёт тракт без запуска реального UART-потока."""
        self.bridge = RealtimeCrsfBridge(
            "/dev/null", 420000,
            mode_channel=6,
            follow_min=1700,
            disarm_channel=5,
            disarm_max=700,
            override_max_age_s=0.25,
        )

    def test_follow_replaces_only_primary_controls(self) -> None:
        """В FOLLOW заменяются только CH1–CH4, служебные каналы остаются от пилота."""
        self.bridge.set_visual_override((1100, 1200, 1300, 1400))
        channels = unpack_channels(self.bridge._output_frame(frame(), time.monotonic())[3:-1])
        self.assertEqual(channels[:4], (1100, 1200, 1300, 1400))
        self.assertEqual(channels[4], 1792)
        self.assertEqual(channels[5], 1792)

    def test_direct_bypasses_override_immediately(self) -> None:
        """Нижнее положение CH6 сразу возвращает реальные команды пилота."""
        self.bridge.set_visual_override((1100, 1200, 1300, 1400))
        self.assertEqual(self.bridge._output_frame(frame(ch6=191), time.monotonic()), frame(ch6=191))

    def test_disarm_bypasses_override_immediately(self) -> None:
        """DISARM нельзя переписать даже свежей командой зрения."""
        self.bridge.set_visual_override((1100, 1200, 1300, 1400))
        self.assertEqual(self.bridge._output_frame(frame(ch5=191), time.monotonic()), frame(ch5=191))

    def test_stale_override_returns_pilot_frame(self) -> None:
        """Просроченная команда Raspberry не удерживает старое управление."""
        self.bridge.set_visual_override((1100, 1200, 1300, 1400))
        self.assertEqual(self.bridge._output_frame(frame(), time.monotonic() + 1.0), frame())


if __name__ == "__main__":
    unittest.main()
