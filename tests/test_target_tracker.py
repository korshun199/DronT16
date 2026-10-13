"""Проверки безопасной границы между CSRT и командами FOLLOW."""

from __future__ import annotations

import unittest
from unittest.mock import patch

import numpy as np

from src.core.state_machine import TargetBox
from src.target.tracker import TargetTracker
from src.target.verifier import TargetVerifier


class _StaticOpenCvTracker:
    """Минимальный двойник CSRT, возвращающий заранее заданную область."""

    def init(self, frame: object, box: tuple[int, int, int, int]) -> bool:
        """Подтверждает инициализацию тестового трекера."""
        return True

    def update(self, frame: object) -> tuple[bool, tuple[int, int, int, int]]:
        """Возвращает допустимую, но ещё не подтверждённую проверкой область."""
        return True, (20, 20, 40, 40)


class TargetTrackerTests(unittest.TestCase):
    """Проверяет безопасную короткую паузу проверки идентичности цели."""

    def test_uncertain_verifier_frame_holds_last_confirmed_target(self) -> None:
        """Сомнительный CSRT-кадр не меняет последнюю подтверждённую рамку."""
        frame = np.full((100, 100, 3), (20, 180, 230), dtype=np.uint8)
        target = TargetBox(20, 20, 40, 40)
        verifier = TargetVerifier(method="adaptive", max_bad_frames=8)
        tracker = TargetTracker(verifier)

        with patch("src.target.tracker._make_tracker", return_value=_StaticOpenCvTracker()):
            tracker.start(frame, target)
            with patch.object(verifier, "verify", return_value=False):
                verifier.last_failure_reason = "VERIFIER_UNCERTAIN"
                self.assertEqual(tracker.update(frame), target)

        self.assertEqual(tracker.last_failure_reason, "VERIFIER_GRACE")
        self.assertTrue(tracker.holding_last_confirmed_target)
        self.assertIn("action=hold_last_confirmed", tracker.last_diagnostic_detail)

    def test_eighth_uncertain_frame_stops_tracking(self) -> None:
        """После восьми сомнительных кадров FOLLOW обязан вернуть пилота."""
        frame = np.full((100, 100, 3), (20, 180, 230), dtype=np.uint8)
        target = TargetBox(20, 20, 40, 40)
        verifier = TargetVerifier(method="adaptive", max_bad_frames=8)
        tracker = TargetTracker(verifier)

        with patch("src.target.tracker._make_tracker", return_value=_StaticOpenCvTracker()):
            tracker.start(frame, target)
            # Два сравнения на каждый кадр: с исходной и рабочей моделями.
            # Исходная модель отвергает область, рабочая ещё узнаёт её.
            with patch.object(verifier, "_similarity", side_effect=[0.20, 0.95] * 8):
                for _ in range(7):
                    self.assertEqual(tracker.update(frame), target)
                self.assertIsNone(tracker.update(frame))

        self.assertEqual(tracker.last_failure_reason, "VERIFIER_REJECTED")
        self.assertFalse(tracker.holding_last_confirmed_target)


if __name__ == "__main__":
    unittest.main()
