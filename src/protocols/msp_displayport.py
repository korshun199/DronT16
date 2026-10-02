"""Приём и хранение экранной сетки Betaflight MSP DisplayPort.

Полётный контроллер передаёт не картинку, а простые команды DisplayPort:
очистить сетку, записать строку, показать готовый кадр.  Этот модуль хранит
последний подтверждённый кадр 30x13 и безопасно передаёт его видеопроцессу
Raspberry через JSON-файл.  Он не отправляет команд в полётный контроллер.
"""

from __future__ import annotations

import json
import os
import select
import termios
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from collections.abc import Callable

from src.protocols.betaflight_msp_link import MSP_DISPLAYPORT, MspParser


# Подкоманды MSP_DISPLAYPORT из Betaflight DisplayPort API.
MSP_DP_HEARTBEAT = 0
MSP_DP_RELEASE = 1
MSP_DP_CLEAR_SCREEN = 2
MSP_DP_WRITE_STRING = 3
MSP_DP_DRAW_SCREEN = 4


@dataclass(frozen=True)
class DisplayCell:
    """Один символ Betaflight OSD вместе с атрибутами выбранного шрифта."""

    code: int = 32
    attribute: int = 0


class DisplayPortCanvas:
    """Сетка внешнего OSD, обновляемая пакетами MSP_DISPLAYPORT."""

    def __init__(self, columns: int = 30, rows: int = 13) -> None:
        """Создаёт пустую ASCII-совместимую сетку заданного размера."""
        if not 1 <= columns <= 80 or not 1 <= rows <= 40:
            raise ValueError("Размер сетки DisplayPort должен быть в разумных пределах")
        self.columns = columns
        self.rows = rows
        self.frame_counter = 0
        self.active = False
        self._cells = self._new_cells()

    def _new_cells(self) -> list[list[DisplayCell]]:
        """Возвращает чистый экран, заполненный пробелами."""
        return [[DisplayCell() for _ in range(self.columns)] for _ in range(self.rows)]

    def clear(self) -> None:
        """Очищает видимый экран, не меняя назначенный размер сетки."""
        self._cells = self._new_cells()

    def apply(self, payload: bytes) -> bool:
        """Применяет MSP_DISPLAYPORT payload и сообщает о готовом новом кадре.

        Возврат ``True`` означает, что Betaflight прислал DRAW_SCREEN и
        изображение уже можно отдавать видеомодулю. Остальные команды меняют
        внутренний буфер, но не инициируют запись JSON на каждый символ.
        """
        if not payload:
            return False
        command = payload[0]
        if command == MSP_DP_HEARTBEAT:
            self.active = True
            return False
        if command == MSP_DP_RELEASE:
            self.active = False
            self.clear()
            return True
        if command == MSP_DP_CLEAR_SCREEN:
            self.active = True
            self.clear()
            return False
        if command == MSP_DP_WRITE_STRING:
            self.active = True
            self._write_string(payload)
            return False
        if command == MSP_DP_DRAW_SCREEN:
            self.active = True
            self.frame_counter += 1
            return True
        # Неизвестная команда игнорируется: она не должна ломать текущее OSD.
        return False

    def _write_string(self, payload: bytes) -> None:
        """Записывает строку в границах сетки без выхода за её пределы."""
        if len(payload) < 5:
            return
        row, column, attribute = payload[1], payload[2], payload[3]
        if row >= self.rows or column >= self.columns:
            return
        # В актуальном Betaflight строка может идти без завершающего NUL.
        text = payload[4:].split(b"\0", 1)[0]
        for offset, code in enumerate(text):
            target_column = column + offset
            if target_column >= self.columns:
                break
            self._cells[row][target_column] = DisplayCell(code, attribute)

    def snapshot(self) -> dict[str, Any]:
        """Строит переносимый снимок OSD для независимого процесса видеовывода."""
        return {
            "version": 1,
            "columns": self.columns,
            "rows": self.rows,
            "frame_counter": self.frame_counter,
            "active": self.active,
            "cells": [
                [{"code": cell.code, "attribute": cell.attribute} for cell in row]
                for row in self._cells
            ],
        }

    def save(self, path: str | Path) -> None:
        """Атомарно сохраняет последний готовый кадр для процесса src.app."""
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.with_suffix(target.suffix + ".tmp")
        temporary.write_text(json.dumps(self.snapshot(), ensure_ascii=True), encoding="ascii")
        os.replace(temporary, target)


def load_canvas_snapshot(path: str | Path, columns: int, rows: int) -> dict[str, Any] | None:
    """Читает снимок только при корректной версии и ожидаемом размере сетки."""
    try:
        raw = json.loads(Path(path).read_text(encoding="ascii"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None
    if not isinstance(raw, dict) or raw.get("version") != 1:
        return None
    if raw.get("columns") != columns or raw.get("rows") != rows:
        return None
    cells = raw.get("cells")
    if not isinstance(cells, list) or len(cells) != rows:
        return None
    if any(not isinstance(row, list) or len(row) != columns for row in cells):
        return None
    return raw


class MspDisplayPortLink:
    """Принимает штатное OSD на выделенном UART без запросов к полётнику."""

    def __init__(self, serial_port: str, baudrate: int, canvas: DisplayPortCanvas, state_file: str | Path) -> None:
        """Открывает выделенный UART и настраивает его на приём DisplayPort."""
        baud_constant = getattr(termios, f"B{baudrate}", None)
        if baud_constant is None:
            raise ValueError(f"Скорость DisplayPort не поддерживается termios: {baudrate}")
        self.serial_port = serial_port
        self.canvas = canvas
        self.state_file = Path(state_file)
        self._fd = os.open(serial_port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        settings = termios.tcgetattr(self._fd)
        settings[0] = 0
        settings[1] = 0
        settings[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        settings[3] = 0
        settings[4] = baud_constant
        settings[5] = baud_constant
        settings[6][termios.VMIN] = 0
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(self._fd, termios.TCSANOW, settings)
        self.parser = MspParser()
        # Счётчики нужны для стендовой диагностики линии FC UART6 → Raspberry.
        self.bytes_received = 0
        self.valid_packets = 0
        self.displayport_packets = 0

    def poll(self) -> int:
        """Сохраняет завершённые OSD-кадры и возвращает их количество."""
        ready, _, _ = select.select([self._fd], [], [], 0)
        if not ready:
            return 0
        try:
            data = os.read(self._fd, 4096)
        except BlockingIOError:
            return 0
        self.bytes_received += len(data)
        self.parser.feed(data)
        completed_frames = 0
        packets = self.parser.drain_packets()
        self.valid_packets += len(packets)
        for packet in packets:
            if packet.command == MSP_DISPLAYPORT and self.canvas.apply(packet.payload):
                self.canvas.save(self.state_file)
                completed_frames += 1
            if packet.command == MSP_DISPLAYPORT:
                self.displayport_packets += 1
        return completed_frames

    def status_text(self) -> str:
        """Возвращает краткое состояние входящей линии без чтения новых байтов."""
        return (
            f"bytes={self.bytes_received} valid_msp={self.valid_packets} "
            f"displayport={self.displayport_packets} frames={self.canvas.frame_counter}"
        )

    def close(self) -> None:
        """Закрывает только UART DisplayPort, не затрагивая UART датчиков."""
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None


class MspDisplayPortWorker:
    """Обслуживает DisplayPort в отдельном потоке от критичного CRSF-цикла."""

    def __init__(
        self,
        link: MspDisplayPortLink,
        report_period_s: float,
        status_callback: Callable[[str], None] | None = None,
        poll_period_s: float = 0.002,
    ) -> None:
        """Сохраняет параметры фонового приёма OSD без запуска потока."""
        if report_period_s <= 0 or poll_period_s <= 0:
            raise ValueError("Периоды DisplayPort должны быть положительными")
        self.link = link
        self.report_period_s = report_period_s
        self.status_callback = status_callback
        self.poll_period_s = poll_period_s
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        """Запускает независимый от RC цикл чтения и сохранения OSD."""
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run,
            name="dront16-displayport",
            daemon=True,
        )
        self._thread.start()

    def _run(self) -> None:
        """Читает UART DisplayPort и не задерживает обработку CRSF."""
        last_report = 0.0
        while not self._stop_event.is_set():
            self.link.poll()
            now = time.monotonic()
            if self.status_callback is not None and now - last_report >= self.report_period_s:
                self.status_callback(self.link.status_text())
                last_report = now
            self._stop_event.wait(self.poll_period_s)

    def stop(self) -> None:
        """Останавливает поток и закрывает только UART DisplayPort."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self.link.close()
