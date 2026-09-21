"""Общий цветной журнал для терминала и append-only файла."""

from __future__ import annotations

import time
from datetime import datetime
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
        # Файл испытания активируется только после ARM и закрывается после DISARM.
        self.session_active = False
        self.file = path.open("a", encoding="utf-8", buffering=1)

    def begin_session(self) -> None:
        """Начинает запись нового ARM-цикла, сохраняя предыдущие циклы."""
        self.session_active = True
        self.start_time = time.monotonic()
        self.sequence = 0

    def end_session(self) -> None:
        """Завершает запись ARM-цикла, не удаляя его из файла."""
        self.session_active = False

    def write(
        self,
        category: str,
        message: str,
        color: str = Color.WHITE,
        console: bool = True,
    ) -> None:
        """Пишет событие в файл и при необходимости показывает его в терминале."""
        elapsed = time.monotonic() - self.start_time
        minutes = int(elapsed // 60)
        seconds = elapsed % 60
        self.sequence += 1
        # Фиксируем местные реальные часы и отдельное время от старта журнала.
        wall_time = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        elapsed_time = f"{minutes:02d}:{seconds:06.3f}"
        stamp = f"[{wall_time}] [{elapsed_time}] [{self.source}] [{self.sequence:06d}] [{category}]"
        plain = f"{stamp} {message}"
        if console:
            print(f"{color}{plain}{Color.RESET}", flush=True)
        # До ARM сообщения видны в терминале, но в файл испытания не попадают.
        if self.session_active:
            self.file.write(plain + "\n")
            self.file.flush()

    def close(self) -> None:
        """Закрывает файл общего журнала."""
        self.file.close()
