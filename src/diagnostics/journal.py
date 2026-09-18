"""Общий цветной журнал для терминала и append-only файла."""

from __future__ import annotations

import time
from pathlib import Path


class Color:
    """ANSI-цвета терминального вывода."""

    RESET = "\033[0m"
    CYAN = "\033[36m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    RED = "\033[31m"
    MAGENTA = "\033[35m"
    BLUE = "\033[34m"
    WHITE = "\033[37m"


class EventJournal:
    """Пишет события одновременно в терминал и append-only журнал."""

    def __init__(self, path: Path, source: str, start_time: float | None = None) -> None:
        """Открывает журнал в режиме добавления, не удаляя историю испытаний."""
        self.path = path
        self.source = source
        self.start_time = time.monotonic() if start_time is None else start_time
        self.sequence = 0
        self.file = path.open("a", encoding="utf-8", buffering=1)

    def write(self, category: str, message: str, color: str = Color.WHITE) -> None:
        """Печатает событие с минутами, секундами и миллисекундами."""
        elapsed = time.monotonic() - self.start_time
        minutes = int(elapsed // 60)
        seconds = elapsed % 60
        self.sequence += 1
        stamp = f"[{minutes:02d}:{seconds:06.3f}] [{self.source}] [{self.sequence:06d}] [{category}]"
        plain = f"{stamp} {message}"
        print(f"{color}{plain}{Color.RESET}", flush=True)
        self.file.write(plain + "\n")
        self.file.flush()

    def close(self) -> None:
        """Закрывает файл общего журнала."""
        self.file.close()
