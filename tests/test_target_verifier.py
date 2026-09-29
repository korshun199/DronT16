"""Проверки неизменяемой модели пилотского захвата."""

from __future__ import annotations

import unittest

import cv2
import numpy as np

from src.core.state_machine import TargetBox
from src.target.verifier import TargetVerifier


class TargetVerifierTests(unittest.TestCase):
    """Проверяет, что адаптация не переписывает исходную цель пилота."""

    def test_adaptive_model_keeps_immutable_capture_template(self) -> None:
        """После хороших кадров исходная модель захвата остаётся неизменной."""
        frame = np.zeros((80, 80, 3), dtype=np.uint8)
        frame[20:60, 20:60] = (20, 180, 230)
        target = TargetBox(20, 20, 40, 40)
        verifier = TargetVerifier(method="adaptive", min_similarity=0.3, adaptation_rate=0.5)
        verifier.start(frame, target)

        captured_template = verifier._capture_template.copy()
        captured_histogram = verifier._capture_histogram.copy()
        self.assertTrue(verifier.verify(frame, target))
        self.assertTrue(np.array_equal(verifier._capture_template, captured_template))
        self.assertTrue(np.array_equal(verifier._capture_histogram, captured_histogram))

    def test_capture_model_rejects_area_accepted_only_by_drifted_working_model(self) -> None:
        """Рабочая модель не может сама разрешить переход на другой фон."""
        captured = np.zeros((80, 80, 3), dtype=np.uint8)
        captured[20:60, 20:60] = (20, 180, 230)
        foreign = np.zeros((80, 80, 3), dtype=np.uint8)
        foreign[20:60, 20:60] = (230, 40, 20)
        target = TargetBox(20, 20, 40, 40)
        verifier = TargetVerifier(method="adaptive", min_similarity=0.3, max_bad_frames=1)
        verifier.start(captured, target)

        # Имитируем уже дрейфовавшую рабочую модель: сама по себе она считает
        # новый фон хорошим, но исходная пилотская модель обязана его отклонить.
        crop = foreign[20:60, 20:60]
        verifier._template = cv2.resize(crop, (64, 64), interpolation=cv2.INTER_AREA)
        verifier._histogram = cv2.normalize(
            cv2.calcHist([cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)], [0, 1], None, [16, 16], [0, 180, 0, 256]),
            None,
            0,
            1,
            cv2.NORM_MINMAX,
        )
        self.assertFalse(verifier.verify(foreign, target))


if __name__ == "__main__":
    unittest.main()
