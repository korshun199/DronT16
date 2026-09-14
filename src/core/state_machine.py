"""Безопасная машина состояний сопровождения выбранной пилотом цели."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class Mode(str, Enum):
    """Состояния прототипа без рабочего автопилота."""

    IDLE = "Ожидание"
    TRACKING = "Следить"
    LOST = "Цель потеряна"


class Command(str, Enum):
    """Команды, которые позднее могут поступать от стика или пульта."""

    CAPTURE = "Захватить"
    FOLLOW = "Следить"
    AUTOPILOT = "Автопилот"
    ABORT = "Отбой"


@dataclass(frozen=True)
class TargetBox:
    """Прямоугольник выбранной цели в координатах кадра."""

    x: float
    y: float
    width: float
    height: float

    def is_valid(self) -> bool:
        """Проверяет, что область цели имеет положительный размер."""
        return self.width > 0 and self.height > 0

    def center(self) -> tuple[float, float]:
        """Возвращает координаты центра цели."""
        return self.x + self.width / 2, self.y + self.height / 2


@dataclass(frozen=True)
class CommandResult:
    """Результат обработки команды для интерфейса и журнала."""

    accepted: bool
    mode: Mode
    message: str


class TargetStateMachine:
    """Управляет целями без права автоматического выбора новой цели."""

    def __init__(self) -> None:
        """Создаёт машину в безопасном состоянии ожидания."""
        self.mode = Mode.IDLE
        self.target: Optional[TargetBox] = None

    def handle(self, command: Command, target: Optional[TargetBox] = None) -> CommandResult:
        """Обрабатывает одну команду и возвращает проверяемый результат."""
        if command is Command.CAPTURE:
            if target is None or not target.is_valid():
                return CommandResult(False, self.mode, "CAPTURE REJECTED: invalid target area")
            self.target = target
            self.mode = Mode.TRACKING
            return CommandResult(True, self.mode, "TARGET CAPTURED")

        if command is Command.ABORT:
            self.target = None
            self.mode = Mode.IDLE
            return CommandResult(True, self.mode, "TRACKING ABORTED")

        if command is Command.FOLLOW:
            if self.target is None or self.mode is Mode.LOST:
                return CommandResult(False, self.mode, "FOLLOW REJECTED: CAPTURE TARGET FIRST")
            self.mode = Mode.TRACKING
            return CommandResult(True, self.mode, "FOLLOWING TARGET")

        if command is Command.AUTOPILOT:
            return CommandResult(False, self.mode, "AUTOPILOT DISABLED: CONTROL SYSTEM UNKNOWN")

        return CommandResult(False, self.mode, "UNKNOWN COMMAND")

    def update_target(self, target: Optional[TargetBox]) -> None:
        """Обновляет координаты только ранее захваченной цели."""
        if self.target is None:
            return
        if target is None or not target.is_valid():
            self.target = None
            self.mode = Mode.LOST
            return
        self.target = target
