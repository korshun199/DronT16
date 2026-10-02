"""Проверки однократного сохранения модели выбранной цели."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from src.core.state_machine import TargetBox
from src.target.snapshot import save_capture_snapshot


class CaptureSnapshotTests(unittest.TestCase):
    """Проверяет кадр модели без запуска камеры и трекера."""

    def test_saves_exact_capture_crop_once_per_call(self) -> None:
        """Каждый явный захват создаёт отдельный PNG точного размера рамки."""
        frame = np.zeros((80, 100, 3), dtype=np.uint8)
        frame[20:50, 30:70] = (17, 93, 201)
        target = TargetBox(30, 20, 40, 30)
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            first_path = save_capture_snapshot(frame, target, directory)
            second_path = save_capture_snapshot(frame, target, directory)
            self.assertNotEqual(first_path, second_path)
            self.assertTrue(first_path.is_file())
            self.assertTrue(second_path.is_file())
            saved = cv2.imread(str(first_path))
            self.assertIsNotNone(saved)
            assert saved is not None
            self.assertEqual(saved.shape, (30, 40, 3))
            np.testing.assert_array_equal(saved, frame[20:50, 30:70])

    def test_rejects_capture_box_outside_frame(self) -> None:
        """Полностью внешняя рамка не создаёт ошибочную пустую картинку."""
        frame = np.zeros((20, 20, 3), dtype=np.uint8)
        with tempfile.TemporaryDirectory() as temporary_directory:
            with self.assertRaises(ValueError):
                save_capture_snapshot(frame, TargetBox(40, 40, 5, 5), temporary_directory)


if __name__ == "__main__":
    unittest.main()
