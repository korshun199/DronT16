"""Загрузка параметров экранного OSD DronT16."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

from src.configuration import load_config_section
from src.core.state_machine import Mode


def parse_color(value: object) -> tuple[int, int, int]:
    """Преобразует цвет #RRGGBB из TOML в формат BGR OpenCV."""
    if not isinstance(value, str) or len(value) != 7 or not value.startswith("#"):
        raise ValueError("Цвет должен быть строкой формата #RRGGBB")
    try:
        red = int(value[1:3], 16)
        green = int(value[3:5], 16)
        blue = int(value[5:7], 16)
    except ValueError as error:
        raise ValueError(f"Некорректный HEX-цвет: {value}") from error
    return blue, green, red


@dataclass(frozen=True)
class OsdConfig:
    """Размеры, положения, цвета и толщины элементов экранного OSD."""

    capture_box_size: int
    crosshair_arm: int
    line_thickness: int
    target_point_radius: int
    mode_font_scale: float
    mode_font_thickness: int
    message_font_scale: float
    message_font_thickness: int
    # Положение подписи режима в пикселях от левого верхнего угла.
    mode_x: int = 20
    mode_y: int = 32
    # Процентное положение центра рамки и перекрестия.
    center_x_percent: float = 50.0
    center_y_percent: float = 50.0
    # Ручное смещение центра в пикселях после процентного расчёта.
    center_offset_x: int = 0
    center_offset_y: int = 0
    # Отдельное смещение центра квадрата захвата относительно перекрестия.
    capture_box_offset_x: int = 0
    capture_box_offset_y: int = 0
    # Цвета режимов в формате BGR OpenCV.
    mode_colors: Mapping[Mode, tuple[int, int, int]] | None = None
    # Режим заполнения и ручная геометрия полного изображения на J7.
    video_standard: str = "NTSC"
    output_fit: str = "stretch"
    output_scale_x: float = 1.0
    output_scale_y: float = 1.0
    output_offset_x: int = 0
    output_offset_y: int = 0


def load_osd_config(path: str | Path) -> OsdConfig:
    """Загружает TOML OSD и отклоняет нулевые или отрицательные размеры."""
    config_path = Path(path)
    try:
        raw = load_config_section(config_path, "osd")
        colors = raw.get("colors", {})
        mode_colors = {
            mode: parse_color(colors[name])
            for mode, name in {
                Mode.IDLE: "idle", Mode.CAPTURE: "capture",
                Mode.TRACKING: "tracking", Mode.LOST: "lost",
                Mode.DISABLED: "disabled", Mode.RETURN: "return",
            }.items()
        }
        output = raw.get("output", {})
        result = OsdConfig(
            capture_box_size=int(raw["capture_box_size"]),
            crosshair_arm=int(raw["crosshair_arm"]),
            line_thickness=int(raw["line_thickness"]),
            target_point_radius=int(raw["target_point_radius"]),
            mode_font_scale=float(raw["mode_font_scale"]),
            mode_font_thickness=int(raw["mode_font_thickness"]),
            message_font_scale=float(raw.get("message_font_scale", 0.55)),
            message_font_thickness=int(raw.get("message_font_thickness", 2)),
            mode_x=int(raw["mode_x"]), mode_y=int(raw["mode_y"]),
            center_x_percent=float(raw["center_x_percent"]),
            center_y_percent=float(raw["center_y_percent"]),
            center_offset_x=int(raw["center_offset_x"]),
            center_offset_y=int(raw["center_offset_y"]),
            capture_box_offset_x=int(raw.get("capture_box_offset_x", 0)),
            capture_box_offset_y=int(raw.get("capture_box_offset_y", 0)),
            mode_colors=mode_colors,
            video_standard=str(output.get("video_standard", "NTSC")).upper(),
            output_fit=str(output.get("fit", "stretch")),
            output_scale_x=float(output.get("scale_x", 1.0)),
            output_scale_y=float(output.get("scale_y", 1.0)),
            output_offset_x=int(output.get("offset_x", 0)),
            output_offset_y=int(output.get("offset_y", 0)),
        )
    except (OSError, KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Не удалось прочитать конфигурацию OSD {config_path}: {error}") from error
    if min(result.capture_box_size, result.crosshair_arm, result.line_thickness,
           result.target_point_radius, result.mode_font_thickness,
           result.message_font_thickness) <= 0:
        raise ValueError("Размеры OSD должны быть положительными")
    if result.mode_font_scale <= 0 or result.message_font_scale <= 0:
        raise ValueError("Масштабы шрифта OSD должны быть положительными")
    if not 0 <= result.center_x_percent <= 100 or not 0 <= result.center_y_percent <= 100:
        raise ValueError("Процент положения центра OSD должен быть от 0 до 100")
    if result.mode_colors is None or set(result.mode_colors) != set(Mode):
        raise ValueError("Для каждого режима OSD должен быть задан цвет")
    for color in result.mode_colors.values():
        if len(color) != 3 or any(not 0 <= value <= 255 for value in color):
            raise ValueError("Каждый цвет OSD должен содержать три значения от 0 до 255")
    if result.output_fit not in {"stretch"}:
        raise ValueError("output_fit должен быть stretch")
    if result.video_standard not in {"NTSC", "PAL"}:
        raise ValueError("video_standard должен быть NTSC или PAL")
    if result.output_scale_x <= 0 or result.output_scale_y <= 0:
        raise ValueError("Масштаб полного изображения должен быть положительным")
    return result
