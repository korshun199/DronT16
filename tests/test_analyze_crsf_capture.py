"""Проверки потокового разбора больших сессий PulseView."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path


ANALYZER_PATH = Path(__file__).parents[1] / "scripts" / "analyze_crsf_capture.py"
SPEC = importlib.util.spec_from_file_location("analyze_crsf_capture", ANALYZER_PATH)
assert SPEC is not None and SPEC.loader is not None
ANALYZER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = ANALYZER
SPEC.loader.exec_module(ANALYZER)


def uart_samples(values: bytes, sample_rate: int, baudrate: int, channel: int) -> bytes:
    """Создаёт 8N1-сигнал выбранного канала для независимой проверки декодера."""

    samples_per_bit = sample_rate // baudrate
    levels = [1] * samples_per_bit
    for value in values:
        levels.extend([0] * samples_per_bit)
        for bit in range(8):
            levels.extend([((value >> bit) & 1)] * samples_per_bit)
        levels.extend([1] * samples_per_bit)
    levels.extend([1] * samples_per_bit)
    return bytes(level << channel for level in levels)


class StreamUartTests(unittest.TestCase):
    """Подтверждает, что границы logic-1-N не теряют UART-байты."""

    def test_reads_uart_across_archive_chunks(self) -> None:
        """Байт, разрезанный между частями .sr, декодируется полностью."""

        expected = bytes((0x24, 0x4D, 0x3E, 0x6C))
        with tempfile.TemporaryDirectory() as directory:
            archive_path = Path(directory) / "capture.sr"
            values = uart_samples(expected, 1_000_000, 100_000, channel=3)
            split = len(values) // 2 + 7
            with zipfile.ZipFile(archive_path, "w") as archive:
                archive.writestr("metadata", "[device 1]\nsamplerate=1 MHz\n")
                archive.writestr("logic-1-0", values[:split])
                archive.writestr("logic-1-1", values[split:])

            index = ANALYZER.read_capture_index(archive_path)
            self.assertEqual(ANALYZER.decode_uart_stream(index, 100_000, 3), expected)


if __name__ == "__main__":
    unittest.main()
