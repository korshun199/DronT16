"""Проверки горизонтального положения и относительной дальности цели."""

import unittest

import cv2
import numpy as np

from src.control.target_range import RangeEstimatorConfig, TargetRangeEstimator
from src.core.state_machine import TargetBox
from src.target.verifier import TargetVerifier


class TargetRangeTests(unittest.TestCase):
    """Проверяет только измерение, без команд полётнику."""

    def setUp(self) -> None:
        """Создаёт измеритель с небольшими тестовыми порогами."""
        self.estimator = TargetRangeEstimator(RangeEstimatorConfig(5.0, 1.0, 1.0, 50.0))

    def test_horizontal_position_and_approach(self) -> None:
        """Объект справа и увеличивается — выдаются оба соответствующих состояния."""
        self.estimator.update(TargetBox(550, 200, 40, 40), 800, 600)
        result = self.estimator.update(TargetBox(560, 200, 80, 80), 800, 600)
        self.assertEqual(result.horizontal_position, "ОБЪЕКТ СМЕЩАЕТСЯ ВПРАВО")
        self.assertEqual(result.distance_trend, "ОБЪЕКТ ПРИБЛИЖАЕТСЯ")
        self.assertAlmostEqual(result.distance_to_object_percent, 0.0, places=4)
        self.assertAlmostEqual(result.capture_size_percent, 400.0, places=4)
        self.assertAlmostEqual(result.relative_size_change_percent, 300.0, places=4)
        self.assertAlmostEqual(result.control_gap_percent, 48.666666, places=4)

    def test_receding_and_center(self) -> None:
        """Уменьшение рамки в центре выдаёт удаление и центр."""
        self.estimator.update(TargetBox(380, 200, 100, 100), 800, 600)
        result = self.estimator.update(TargetBox(390, 200, 50, 50), 800, 600)
        self.assertEqual(result.horizontal_position, "ЦЕНТР")
        self.assertEqual(result.distance_trend, "ОБЪЕКТ УДАЛЯЕТСЯ")
        self.assertAlmostEqual(result.distance_to_object_percent, 75.0, places=4)
        self.assertAlmostEqual(result.relative_size_change_percent, -75.0, places=4)
        self.assertAlmostEqual(result.control_gap_percent, 49.479166, places=4)

    def test_capture_is_zero_distance_and_hundred_percent(self) -> None:
        """В момент захвата исходный размер равен 100%, дистанция равна 0%."""
        result = self.estimator.update(TargetBox(100, 100, 100, 100), 800, 600)
        self.assertAlmostEqual(result.capture_size_percent, 100.0)
        self.assertAlmostEqual(result.distance_to_object_percent, 0.0)

    def test_eighty_percent_area_marks_object_under_control(self) -> None:
        """Объект площадью не менее 50 процентов кадра даёт контрольный статус."""
        result = self.estimator.update(TargetBox(0, 0, 10, 10), 800, 600, 50.0)
        self.assertTrue(result.under_control)

    def test_control_area_does_not_change_signed_object_size(self) -> None:
        """Площадь контроля не меняет знак размера цели относительно захвата."""
        self.estimator.update(TargetBox(0, 0, 100, 100), 800, 600, 10.0, 10.0)
        result = self.estimator.update(TargetBox(0, 0, 50, 50), 800, 600, 5.0, 90.0)
        self.assertLess(result.relative_size_change_percent, 0.0)
        self.assertTrue(result.under_control)

    def test_full_frame_change_is_measured_outside_fixed_box(self) -> None:
        """Закрытие всего кадра не ограничивается размером рамки трекера."""
        reference = np.zeros((100, 100, 3), dtype=np.uint8)
        covered = np.full((100, 100, 3), 255, dtype=np.uint8)
        verifier = TargetVerifier(method="adaptive")
        verifier.start(reference, TargetBox(45, 45, 10, 10))
        area = verifier.frame_fill_percent(covered, TargetBox(45, 45, 10, 10))
        self.assertIsNotNone(area)
        assert area is not None
        self.assertGreaterEqual(area, 80.0)


if __name__ == "__main__":
    unittest.main()
