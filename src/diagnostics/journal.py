"""Общий цветной журнал для терминала и append-only файла."""

from __future__ import annotations

import time
import os
from datetime import datetime
from pathlib import Path
from typing import TextIO


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
        # Базовый путь используется как шаблон; отдельный файл создаётся при ARM.
        self.session_active = False
        self.file: TextIO | None = None
        self.session_path: Path | None = None
        self.last_sync = 0.0

    def begin_session(self) -> None:
        """Создаёт отдельный файл нового ARM-цикла с датой и временем."""
        if self.session_active:
            self.end_session()
        now = datetime.now().astimezone()
        suffix = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
        candidate = self.path.with_name(f"{self.path.stem}_{suffix}{self.path.suffix}")
        counter = 1
        while candidate.exists():
            candidate = self.path.with_name(
                f"{self.path.stem}_{suffix}_{counter:02d}{self.path.suffix}"
            )
            counter += 1
        candidate.parent.mkdir(parents=True, exist_ok=True)
        self.session_path = candidate
        self.file = candidate.open("x", encoding="utf-8", buffering=1)
        self.file.write(
            f"# ARM session started: {now.astimezone().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}\n"
        )
        self._sync_file(force=True)
        self.session_active = True
        self.start_time = time.monotonic()
        self.sequence = 0
        self.last_sync = self.start_time

    def end_session(self) -> None:
        """Завершает и закрывает текущий файл ARM-цикла."""
        self.session_active = False
        if self.file is not None:
            self._sync_file(force=True)
            self.file.close()
            self.file = None

    def _sync_file(self, *, force: bool = False) -> None:
        """Синхронизирует журнал с файловой системой не реже двух раз в секунду."""
        if self.file is None:
            return
        now = time.monotonic()
        if not force and now - self.last_sync < 0.5:
            return
        self.file.flush()
        os.fsync(self.file.fileno())
        self.last_sync = now

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
        if write_file and self.file is not None:
            self.file.write(plain + "\n")
            self._sync_file()

    def close(self) -> None:
        """Закрывает файл общего журнала."""
        self.end_session()
