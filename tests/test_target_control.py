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
            )
            guidance = read_target_guidance(path)
            self.assertIsNotNone(guidance)
            assert guidance is not None
            self.assertTrue(guidance.is_fresh(guidance.updated_at + 0.1, 0.3))
            self.assertFalse(guidance.is_fresh(guidance.updated_at + 0.4, 0.3))
            self.assertEqual(guidance.yaw_error_deg, -8.5)

    def test_invalid_snapshot_is_not_usable(self) -> None:
        """Повреждённый или отсутствующий файл не даёт старой команды."""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "target.json"
            path.write_text("not-json", encoding="utf-8")
            self.assertIsNone(read_target_guidance(path))


if __name__ == "__main__":
    unittest.main()
