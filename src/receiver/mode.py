"""Преобразование положения AUX-канала в безопасный режим DronT16."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ReceiverMode(str, Enum):
    """Режимы, которые разрешены первым модулем без управления полётником."""

    DIRECT = "DIRECT"
    CAPTURE = "CAPTURE"
    CANCEL = "CANCEL"


@dataclass(frozen=True)
class ModeThresholds:
    """Пороговые значения CRSF и число кадров для подтверждения переключения."""

    low_max: int
    high_min: int
    debounce_frames: int


class ReceiverModeDecoder:
    """Устойчиво определяет режим по одному AUX-каналу с защитой от дребезга."""

    def __init__(self, thresholds: ModeThresholds) -> None:
        """Создаёт декодер и проверяет границы его настроек."""
        if not 0 <= thresholds.low_max < thresholds.high_min <= 2047:
            raise ValueError("Пороги режима должны находиться в диапазоне 0..2047")
        if thresholds.debounce_frames < 1:
            raise ValueError("debounce_frames должен быть не меньше 1")
        self.thresholds = thresholds
        self.current_mode = ReceiverMode.DIRECT
        self._candidate_mode = self.current_mode
        self._candidate_count = 0

    def classify(self, channel_value: int) -> ReceiverMode:
        """Определяет необработанное положение AUX по значению CRSF."""
        if channel_value < self.thresholds.low_max:
            return ReceiverMode.DIRECT
        if channel_value >= self.thresholds.high_min:
            return ReceiverMode.CANCEL
        return ReceiverMode.CAPTURE

    def update(self, channel_value: int) -> tuple[ReceiverMode, bool]:
        """Возвращает устойчивый режим и признак подтверждённого перехода."""
        candidate_mode = self.classify(channel_value)
        if candidate_mode == self.current_mode:
            self._candidate_mode = candidate_mode
            self._candidate_count = 0
            return self.current_mode, False
        if candidate_mode != self._candidate_mode:
            self._candidate_mode = candidate_mode
            self._candidate_count = 1
        else:
            self._candidate_count += 1
        if self._candidate_count >= self.thresholds.debounce_frames:
            self.current_mode = candidate_mode
            self._candidate_count = 0
            return self.current_mode, True
        return self.current_mode, False


def mode_label(mode: ReceiverMode) -> str:
    """Возвращает понятное русское описание режима для консоли."""
    return {
        ReceiverMode.DIRECT: "Свободный",
        ReceiverMode.CAPTURE: "Захватить",
        ReceiverMode.CANCEL: "Отбой",
    }[mode]
