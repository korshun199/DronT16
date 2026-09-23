"""Отрисовка штатного Betaflight OSD, полученного по MSP DisplayPort."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from src.interface.betaflight_font import glyph_mask
from src.interface.osd_config import parse_color
from src.protocols.msp_displayport import load_canvas_snapshot


@dataclass(frozen=True)
class BetaflightOsdConfig:
    """Настройки слоя, который повторяет OSD полётного контроллера."""

    enabled: bool
    state_file: Path
    columns: int
    rows: int
    color: tuple[int, int, int]
    opacity: float


def load_betaflight_osd_config(raw: dict[str, object]) -> BetaflightOsdConfig:
    """Проверяет секцию ``[osd.betaflight]`` единого TOML-файла."""
    data = raw.get("betaflight", {})
    if not isinstance(data, dict):
        raise ValueError("osd.betaflight должна быть TOML-секцией")
    result = BetaflightOsdConfig(
        enabled=bool(data.get("enabled", False)),
        state_file=Path(str(data.get("state_file", "/tmp/dront16_betaflight_osd.json"))),
        columns=int(data.get("columns", 30)),
        rows=int(data.get("rows", 13)),
        color=parse_color(data.get("color", "#ffffff")),
        opacity=float(data.get("opacity", 1.0)),
    )
    if not 1 <= result.columns <= 80 or not 1 <= result.rows <= 40:
        raise ValueError("Размер OSD Betaflight должен быть в пределах 1..80 x 1..40")
    if not 0.0 < result.opacity <= 1.0:
        raise ValueError("Прозрачность OSD Betaflight должна быть от 0 до 1")
    return result


def _draw_betaflight_glyph(image: Any, code: int, x: int, y: int, cell_width: float,
                           cell_height: float, color: tuple[int, int, int]) -> None:
    """Рисует глиф реального 256-символьного шрифта Betaflight без Unicode."""
    right = min(image.shape[1], int((x + cell_width) + 0.5))
    bottom = min(image.shape[0], int((y + cell_height) + 0.5))
    if right <= x or bottom <= y:
        return
    resized = cv2.resize(glyph_mask(code), (right - x, bottom - y), interpolation=cv2.INTER_NEAREST)
    mask = resized > 0
    region = image[y:bottom, x:right]
    region[mask] = color


def draw_betaflight_osd(frame: Any, config: BetaflightOsdConfig, now: float | None = None) -> Any:
    """Рисует последний кадр DisplayPort перед рамкой и линией DronT16.

    Каждый код из DisplayPort отрисовывается штатным 256-символьным шрифтом
    Betaflight: буквенные метки и пиктограммы не заменяются квадратами.
    """
    if not config.enabled:
        return frame
    snapshot = load_canvas_snapshot(config.state_file, config.columns, config.rows)
    if snapshot is None or not bool(snapshot.get("active", False)):
        return frame
    target = frame if config.opacity >= 1.0 else frame.copy()
    height, width = target.shape[:2]
    cell_width = width / config.columns
    cell_height = height / config.rows
    current_time = time.monotonic() if now is None else now
    blink_visible = int(current_time * 2.0) % 2 == 0
    for row_index, row in enumerate(snapshot["cells"]):
        for column_index, cell in enumerate(row):
            if not isinstance(cell, dict):
                continue
            code = cell.get("code")
            attribute = cell.get("attribute", 0)
            if not isinstance(code, int) or not isinstance(attribute, int) or code == 32:
                continue
            if attribute & 0x40 and not blink_visible:
                continue
            x = int(column_index * cell_width)
            y = int(row_index * cell_height)
            # Латиница и цифры остаются ровными и хорошо читаемыми на J7.
            # Пиксельный шрифт MAX7456 используем лишь для собственных значков
            # Betaflight: батареи, RSSI, стрелок и обозначений режима.
            if 33 <= code <= 126:
                font_scale = max(0.25, min(cell_width / 24.0, cell_height / 30.0))
                baseline_offset = int(cell_height * 0.78)
                cv2.putText(
                    target, chr(code), (x, y + baseline_offset), cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, config.color, 1, cv2.LINE_AA,
                )
            else:
                _draw_betaflight_glyph(target, code, x, y, cell_width, cell_height, config.color)
    if target is not frame:
        cv2.addWeighted(target, config.opacity, frame, 1.0 - config.opacity, 0.0, dst=frame)
    return frame
