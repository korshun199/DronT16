"""Тесты безопасной state machine управляемого CRSF-перехвата."""

import unittest

from src.receiver.crsf import (
    CRSF_RC_CHANNELS_PACKED,
    crc8_dvb_s2,
    extract_raw_frames,
    parse_link_statistics,
    pack_channels,
    unpack_channels,
)
from src.receiver.takeover import TakeoverConfig, TakeoverController, TakeoverState


def make_frame(channels: tuple[int, ...]) -> bytes:
    """Создаёт синтетический CRC-проверенный RC-кадр."""
    payload = pack_channels(channels)
    body = bytes((0xC8, len(payload) + 2, CRSF_RC_CHANNELS_PACKED)) + payload
    return body + bytes((crc8_dvb_s2(body[2:]),))


def make_channels(ch5: int = 1792, ch7: int = 191, throttle: int = 1500) -> tuple[int, ...]:
    """Возвращает стендовый набор 16 каналов CRSF."""
    channels = [992] * 16
    channels[2] = throttle
    channels[4] = ch5
    channels[6] = ch7
    return tuple(channels)


class TakeoverTests(unittest.TestCase):
    """Проверяет переходы, приоритет DISARM и CRC."""

    def setUp(self) -> None:
        """Создаёт контроллер с настройками текущего передатчика."""
        self.controller = TakeoverController(
            TakeoverConfig(6, 1700, 700, 4, 700, 1700, 2, 191, 5.0)
        )

    def process(self, frame: bytes, now: float = 0.0):
        """Передаёт кадр в state machine после его разбора."""
        return self.controller.process(frame, unpack_channels(frame[3:-1]), now)

    def test_live_takeover_frozen_and_restore(self) -> None:
        """Новые кадры подавляются, а после CH7 возвращается живой поток."""
        first = make_frame(make_channels(throttle=1500))
        self.assertEqual(self.process(first).output_kind, "LIVE")
        loss = make_frame(make_channels(ch7=1792, throttle=1700))
        takeover = self.process(loss)
        self.assertEqual(takeover.state, TakeoverState.TAKEOVER)
        self.assertEqual(takeover.output_kind, "THROTTLE_RAMP")
        self.assertEqual(takeover.output_frame, first)
        changed = make_frame(make_channels(ch7=1792, throttle=1800))
        ramped = self.process(changed, now=2.5)
        self.assertEqual(ramped.output_kind, "THROTTLE_RAMP")
        self.assertEqual(unpack_channels(ramped.output_frame[3:-1])[2], 846)
        restored = make_frame(make_channels(ch7=191, throttle=1600))
        result = self.process(restored)
        self.assertEqual(result.state, TakeoverState.LIVE)
        self.assertEqual(result.output_frame, restored)

    def test_disarm_has_priority_and_rebuilds_crc(self) -> None:
        """DISARM во время takeover возвращает живой канал и сохраняет CRC."""
        self.process(make_frame(make_channels()))
        self.process(make_frame(make_channels(ch7=1792)), now=1.0)
        disarm = make_frame(make_channels(ch5=191, ch7=1792))
        result = self.process(disarm, now=2.0)
        self.assertEqual(result.output_kind, "DISARM")
        self.assertEqual(result.state, TakeoverState.DISARM_RELEASE)
        self.assertEqual(unpack_channels(result.output_frame[3:-1])[4], 700)
        self.assertEqual(extract_raw_frames(bytearray(result.output_frame)), [result.output_frame])
        resumed = self.process(make_frame(make_channels(ch5=1792, ch7=1792)), now=3.0)
        self.assertEqual(resumed.state, TakeoverState.LIVE)

    def test_takeover_requires_arm(self) -> None:
        """CH7 без ARM не должен включать реальный takeover."""
        disarmed = make_frame(make_channels(ch5=191, ch7=1792))
        result = self.process(disarmed)
        self.assertEqual(result.state, TakeoverState.LIVE)
        self.assertEqual(result.output_kind, "LIVE")
        self.assertTrue(any("BLOCKED" in event for event in result.events))

    def test_takeover_repeats_when_receiver_frame_is_absent(self) -> None:
        """Во время takeover последний кадр повторяется без нового входного кадра."""
        first = make_frame(make_channels(throttle=1500))
        self.process(first)
        self.process(make_frame(make_channels(ch7=1792)), now=1.0)
        repeated = self.controller.repeat_without_receiver(1.5)
        self.assertIsNotNone(repeated)
        self.assertEqual(repeated.output_kind, "THROTTLE_RAMP_TIMEOUT")
        self.assertEqual(unpack_channels(repeated.output_frame[3:-1])[2], 1369)
        self.assertEqual(extract_raw_frames(bytearray(repeated.output_frame)), [repeated.output_frame])

    def test_takeover_holds_throttle_when_landing_controller_is_active(self) -> None:
        """При посадочном модуле мост не снижает газ самостоятельно."""
        controller = TakeoverController(
            TakeoverConfig(6, 1700, 700, 4, 700, 1700, 2, 191, 5.0, False)
        )
        first = make_frame(make_channels(throttle=1500))
        controller.process(first, unpack_channels(first[3:-1]), 0.0)
        takeover = make_frame(make_channels(ch7=1792, throttle=1700))
        result = controller.process(takeover, unpack_channels(takeover[3:-1]), 5.0)
        self.assertEqual(unpack_channels(result.output_frame[3:-1])[2], 1500)
        self.assertEqual(result.output_kind, "TAKEOVER_HOLD")

    def test_link_statistics_field_order(self) -> None:
        """LQ и SNR читаются из третьего и четвёртого байтов, а не из RSSI2."""
        stats = parse_link_statistics(bytes((44, 33, 98, 7, 1, 2, 50, 41, 97, 250)))
        self.assertEqual(stats.uplink_rssi_1, 44)
        self.assertEqual(stats.uplink_rssi_2, 33)
        self.assertEqual(stats.uplink_link_quality, 98)
        self.assertEqual(stats.uplink_snr, 7)
        self.assertEqual(stats.downlink_link_quality, 97)
        self.assertEqual(stats.downlink_snr, -6)


if __name__ == "__main__":
    unittest.main()
