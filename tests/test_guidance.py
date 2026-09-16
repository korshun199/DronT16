"""Проверки расчёта положения цели и безопасного MSP dry-run."""

import unittest
from pathlib import Path

from src.control.follow_config import load_follow_config
from src.control.guidance import calculate_guidance
from src.core.state_machine import TargetBox
from src.protocols.betaflight_msp import BetaflightMsp


class GuidanceTests(unittest.TestCase):
    """Проверяет координаты цели и ограничение направления."""

    def setUp(self) -> None:
        """Загружает штатную конфигурацию сопровождения."""
        self.config = load_follow_config(Path("config/follow.toml"))

    def test_center_target_has_zero_command(self) -> None:
        """Цель в центре кадра не требует поворота."""
        result = calculate_guidance(TargetBox(270, 190, 100, 100), 640, 480, self.config)
        self.assertAlmostEqual(result.yaw_error_deg, 0.0)
        self.assertEqual(result.yaw_rate_deg_s, 0.0)
        self.assertEqual(result.pitch_rate_deg_s, 0.0)

    def test_command_is_limited_and_dry_run(self) -> None:
        """Большая ошибка ограничивается, а MSP остаётся диагностическим."""
        result = calculate_guidance(TargetBox(600, 0, 40, 40), 640, 480, self.config)
        self.assertEqual(result.yaw_rate_deg_s, self.config.guidance.max_yaw_rate_deg_s)
        text = BetaflightMsp(self.config.msp).format_guidance(result)
        self.assertIn("MSP DRY-RUN", text)


if __name__ == "__main__":
    unittest.main()
