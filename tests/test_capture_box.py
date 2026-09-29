"""Проверяет совпадение области захвата с видимой рамкой OSD."""

from __future__ import annotations

import unittest

import numpy as np

from src.app import center_target
from src.core.state_machine import TargetBox
from src.interface.osd_config import OsdConfig


class CaptureBoxTests(unittest.TestCase):
    """Не допускает расхождения рамки J7 и сохранённой модели цели."""

    def test_capture_target_uses_osd_center_and_offsets(self) -> None:
        """Рамка захвата учитывает те же смещения, что и отрисовка OSD."""
        config = OsdConfig(
            capture_box_size=20,
            crosshair_arm=8,
            line_thickness=2,
            target_point_radius=4,
            mode_font_scale=0.8,
            mode_font_thickness=2,
            message_font_scale=0.55,
            message_font_thickness=2,
            center_x_percent=50.0,
            center_y_percent=50.0,
            center_offset_x=-10,
            center_offset_y=-10,
            capture_box_offset_x=-5,
            capture_box_offset_y=-10,
        )
        frame = np.zeros((100, 200, 3), dtype=np.uint8)
        target = center_target(frame, config)
        self.assertIsNotNone(target)
        assert target is not None
        self.assertEqual(target, TargetBox(75.0, 20.0, 20.0, 20.0))


if __name__ == "__main__":
    unittest.main()
