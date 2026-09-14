"""Проверки команд и запрета самостоятельного выбора новой цели."""

import unittest

from src.core.state_machine import Command, Mode, TargetBox, TargetStateMachine


class StateMachineTests(unittest.TestCase):
    """Проверяет безопасные переходы ядра."""

    def setUp(self) -> None:
        """Создаёт свежую машину перед каждым тестом."""
        self.machine = TargetStateMachine()
        self.target = TargetBox(10, 20, 100, 80)

    def test_capture_requires_valid_target(self) -> None:
        """Пустая область не должна создавать цель."""
        result = self.machine.handle(Command.CAPTURE, TargetBox(0, 0, 0, 20))
        self.assertFalse(result.accepted)
        self.assertEqual(self.machine.mode, Mode.IDLE)

    def test_capture_then_follow(self) -> None:
        """Захват пилотом включает экранное сопровождение."""
        self.assertTrue(self.machine.handle(Command.CAPTURE, self.target).accepted)
        result = self.machine.handle(Command.FOLLOW)
        self.assertTrue(result.accepted)
        self.assertEqual(result.mode, Mode.TRACKING)

    def test_autopilot_is_disabled(self) -> None:
        """Команда автопилота не должна проходить до выбора системы управления."""
        self.machine.handle(Command.CAPTURE, self.target)
        result = self.machine.handle(Command.AUTOPILOT)
        self.assertFalse(result.accepted)
        self.assertEqual(result.mode, Mode.TRACKING)

    def test_lost_target_needs_new_capture(self) -> None:
        """После потери нельзя продолжить без нового захвата."""
        self.machine.handle(Command.CAPTURE, self.target)
        self.machine.update_target(None)
        self.assertEqual(self.machine.mode, Mode.LOST)
        result = self.machine.handle(Command.FOLLOW)
        self.assertFalse(result.accepted)
        self.assertIsNone(self.machine.target)

    def test_abort_clears_target(self) -> None:
        """Отбой очищает цель и возвращает ожидание."""
        self.machine.handle(Command.CAPTURE, self.target)
        result = self.machine.handle(Command.ABORT)
        self.assertTrue(result.accepted)
        self.assertEqual(self.machine.mode, Mode.IDLE)
        self.assertIsNone(self.machine.target)


if __name__ == "__main__":
    unittest.main()
