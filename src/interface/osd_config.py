"""Загрузка параметров экранного OSD DronT16."""

from __future__ import annotations

import tomllib
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class OsdConfig:
    """Размеры и толщины элементов экранного OSD."""

    capture_box_size: int
    crosshair_arm: int
    line_thickness: int
    target_point_radius: int
    mode_font_scale: float
    mode_font_thickness: int
    message_font_scale: float
    message_font_thickness: int


def load_osd_config(path: str | Path) -> OsdConfig:
    """Загружает TOML OSD и отклоняет нулевые или отрицательные размеры."""
    config_path = Path(path)
    try:
        with config_path.open("rb") as config_file:
            raw = tomllib.load(config_file)["osd"]
        result = OsdConfig(
            int(raw["capture_box_size"]), int(raw["crosshair_arm"]),
            int(raw["line_thickness"]), int(raw["target_point_radius"]),
            float(raw["mode_font_scale"]), int(raw["mode_font_thickness"]),
            float(raw["message_font_scale"]), int(raw["message_font_thickness"]),
        )
    except (OSError, KeyError, TypeError, ValueError, tomllib.TOMLDecodeError) as error:
        raise ValueError(f"Не удалось прочитать конфигурацию OSD {config_path}: {error}") from error
    if min(result.capture_box_size, result.crosshair_arm, result.line_thickness,
           result.target_point_radius, result.mode_font_thickness,
           result.message_font_thickness) <= 0:
        raise ValueError("Размеры OSD должны быть положительными")
    if result.mode_font_scale <= 0 or result.message_font_scale <= 0:
        raise ValueError("Масштабы шрифта OSD должны быть положительными")
    return result
