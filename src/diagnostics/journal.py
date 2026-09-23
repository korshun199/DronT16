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

    def __init__(
        self,
        path: Path,
        source: str,
        start_time: float | None = None,
        console_enabled: bool = True,
        file_enabled: bool = True,
        console_categories: set[str] | None = None,
        file_categories: set[str] | None = None,
    ) -> None:
        """Открывает файл и настраивает независимые каналы вывода."""
        self.path = path
        self.source = source
        self.start_time = time.monotonic() if start_time is None else start_time
        self.sequence = 0
        self.console_enabled = console_enabled
        self.file_enabled = file_enabled
        self.console_categories = console_categories
        self.file_categories = file_categories
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
        console: bool | None = None,
        file: bool | None = None,
    ) -> None:
        """Маршрутизирует событие в консоль и файл независимо друг от друга."""
        elapsed = time.monotonic() - self.start_time
        minutes = int(elapsed // 60)
        seconds = elapsed % 60
        self.sequence += 1
        # Фиксируем местные реальные часы и отдельное время от старта журнала.
        wall_time = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        elapsed_time = f"{minutes:02d}:{seconds:06.3f}"
        stamp = f"[{wall_time}] [{elapsed_time}] [{self.source}] [{self.sequence:06d}] [{category}]"
        plain = f"{stamp} {message}"
        category_name = category.upper()
        show_console = (
            self.console_enabled
            and (console is not False)
            and (self.console_categories is None or category_name in self.console_categories)
        )
        write_file = (
            self.file_enabled
            and self.session_active
            and (file is not False)
            and (self.file_categories is None or category_name in self.file_categories)
        )
        if show_console:
            print(f"{color}{plain}{Color.RESET}", flush=True)
        # До ARM сообщения не попадают в файл испытания, но могут быть видны в консоли.
        if write_file:
            self.file.write(plain + "\n")
            self.file.flush()

    def close(self) -> None:
        """Закрывает файл общего журнала."""
        self.file.close()
