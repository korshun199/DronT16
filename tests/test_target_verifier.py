"""Проверки неизменяемой модели пилотского захвата."""

from __future__ import annotations

import unittest

import cv2
import numpy as np
from unittest.mock import patch

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
        self.assertEqual(verifier.last_failure_reason, "VERIFIER_REJECTED")
        self.assertIn("bad_frames=1/1", verifier.diagnostic_detail())

    def test_continuity_model_keeps_fast_motion_only_with_capture_evidence(self) -> None:
        """Быстро изменившаяся цель принимается лишь при следе исходного захвата."""
        frame = np.full((80, 80, 3), (20, 180, 230), dtype=np.uint8)
        target = TargetBox(20, 20, 40, 40)
        verifier = TargetVerifier(
            method="adaptive",
            min_similarity=0.42,
            continuity_capture_floor_ratio=0.70,
        )
        verifier.start(frame, target)

        # Первый результат — похожесть на исходный захват, второй — на
        # подтверждённый недавний вид. 0.31 ниже строгих 0.42, но выше
        # безопасного следа 0.42 * 0.70 = 0.294.
        with patch.object(verifier, "_similarity", side_effect=(0.31, 0.80)):
            self.assertTrue(verifier.verify(frame, target))

    def test_uncertain_frame_reports_grace_candidate(self) -> None:
        """Сомнительный кадр фиксируется до решения трекера о паузе FOLLOW."""
        frame = np.full((80, 80, 3), (20, 180, 230), dtype=np.uint8)
        target = TargetBox(20, 20, 40, 40)
        verifier = TargetVerifier(method="adaptive", min_similarity=0.42, max_bad_frames=48)
        verifier.start(frame, target)

        with patch.object(verifier, "_similarity", side_effect=(0.20, 0.95)):
            self.assertFalse(verifier.verify(frame, target))
        self.assertEqual(verifier.last_failure_reason, "VERIFIER_UNCERTAIN")
        self.assertIn("bad_frames=1/48", verifier.diagnostic_detail())


if __name__ == "__main__":
    unittest.main()
