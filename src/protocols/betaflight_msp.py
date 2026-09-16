"""Безопасный адаптер Betaflight MSP в режиме подготовки команд."""

from __future__ import annotations

from src.control.follow_config import MspConfig
from src.control.guidance import GuidanceResult


class BetaflightMsp:
    """Формирует человекочитаемую MSP-команду без отправки в UART."""

    def __init__(self, config: MspConfig) -> None:
        """Создаёт адаптер и принудительно оставляет его в dry-run."""
        if config.enabled or config.transport != "dry-run":
            raise ValueError("Адаптер Betaflight пока разрешён только в dry-run")
        self.config = config

    def format_guidance(self, guidance: GuidanceResult) -> str:
        """Возвращает красную диагностическую строку будущей команды MSP."""
        return (
            "MSP DRY-RUN | SET_RAW_RC yaw_rate="
            f"{guidance.yaw_rate_deg_s:+.1f} pitch_rate={guidance.pitch_rate_deg_s:+.1f}"
        )
