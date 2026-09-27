"""Проверки трёхрежимной индикации OSD."""

from __future__ import annotations

from pathlib import Path
import unittest

from src.core.state_machine import Mode
from src.interface.osd_config import load_osd_config
from src.interface.overlay import MODE_COLORS


class OverlayModeTest(unittest.TestCase):
    """На экране J7 остаются только состояния пилота, захвата и FOLLOW."""

    def test_project_config_has_exactly_three_visible_modes(self) -> None:
        """Конфиг содержит белый DIRECT, жёлтый CAPTURE и красный FOLLOW."""
        config_path = Path(__file__).resolve().parents[1] / "config" / "dront16.toml"
        colors = load_osd_config(config_path).mode_colors
        self.assertEqual(set(colors or {}), {Mode.IDLE, Mode.CAPTURE, Mode.TRACKING})
        self.assertEqual(colors[Mode.IDLE], (255, 255, 255))
        self.assertEqual(colors[Mode.CAPTURE], (0, 221, 255))
        self.assertEqual(colors[Mode.TRACKING], (0, 0, 255))

    def test_control_is_not_a_visible_osd_mode(self) -> None:
        """Расстояние до цели не создаёт отдельный синий или иной режим."""
        self.assertNotIn("CONTROL", Mode.__members__)
        self.assertEqual(MODE_COLORS[Mode.TRACKING], (0, 0, 255))


if __name__ == "__main__":
    unittest.main()
