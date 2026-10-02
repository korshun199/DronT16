"""Проверки безопасной передачи направления цели между процессами."""

import tempfile
import unittest
from pathlib import Path

from src.control.target_control import read_target_guidance, write_target_guidance


class TargetControlTests(unittest.TestCase):
    """Проверяет атомарный формат снимка захваченной цели."""

    def test_guidance_round_trip_and_freshness(self) -> None:
        """Направление читается и имеет проверяемый срок годности."""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "target.json"
            write_target_guidance(
                path,
                valid=True,
                yaw_error_deg=-8.5,
                pitch_error_deg=1.0,
                mode="Следить",
                normalized_x=-0.2,
                normalized_y=0.1,
                scale_percent=4.5,
            )
            guidance = read_target_guidance(path)
            self.assertIsNotNone(guidance)
            assert guidance is not None
            self.assertTrue(guidance.is_fresh(guidance.updated_at + 0.1, 0.3))
            self.assertFalse(guidance.is_fresh(guidance.updated_at + 0.4, 0.3))
            self.assertEqual(guidance.yaw_error_deg, -8.5)
            self.assertTrue(guidance.is_complete(guidance.updated_at + 0.1, 0.3, 0.05))
            self.assertEqual(guidance.scale_percent, 4.5)

    def test_lost_snapshot_preserves_diagnostic_reason(self) -> None:
        """Причина срыва трекера доходит до отдельного процесса моста."""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "target.json"
            write_target_guidance(
                path,
                valid=False,
                yaw_error_deg=None,
                pitch_error_deg=None,
                mode="Цель потеряна",
                loss_reason="VERIFIER_REJECTED",
                loss_detail="bad_frames=16/16",
            )
            guidance = read_target_guidance(path)
            self.assertIsNotNone(guidance)
            assert guidance is not None
            self.assertEqual(guidance.loss_reason, "VERIFIER_REJECTED")
            self.assertEqual(guidance.loss_detail, "bad_frames=16/16")

    def test_invalid_snapshot_is_not_usable(self) -> None:
        """Повреждённый или отсутствующий файл не даёт старой команды."""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "target.json"
            path.write_text("not-json", encoding="utf-8")
            self.assertIsNone(read_target_guidance(path))


if __name__ == "__main__":
    unittest.main()
