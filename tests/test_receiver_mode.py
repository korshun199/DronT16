"""Проверки положений CH6/AUX2 для видеорежимов DronT16."""

import unittest

from src.receiver.mode import ModeThresholds, ReceiverMode, ReceiverModeDecoder


class ReceiverModeTests(unittest.TestCase):
    """Проверяет, что нижнее CH6 всегда возвращает DIRECT/LIVE."""

    def test_low_ch6_returns_direct(self) -> None:
        """После верхнего и среднего положений нижнее подтверждается как DIRECT."""
        decoder = ReceiverModeDecoder(ModeThresholds(700, 1700, 2))
        decoder.update(1800)
        decoder.update(1800)
        self.assertEqual(decoder.current_mode, ReceiverMode.FOLLOW)
        decoder.update(1000)
        mode, changed = decoder.update(1000)
        self.assertTrue(changed)
        self.assertEqual(mode, ReceiverMode.CAPTURE)
        decoder.update(500)
        mode, changed = decoder.update(500)
        self.assertTrue(changed)
        self.assertEqual(mode, ReceiverMode.DIRECT)


if __name__ == "__main__":
    unittest.main()
