"""Рисует рамку цели, линию к ней и безопасное состояние прототипа."""

from __future__ import annotations

from typing import Any

import cv2

from src.core.state_machine import Mode, TargetBox
from src.interface.osd_config import OsdConfig


# ASCII-подписи режимов, которые безопасно отображаются на любом шрифте.
MODE_LABELS = {
    Mode.IDLE: "READY",
    Mode.CAPTURE: "CAPTURE",
    Mode.TRACKING: "FOLLOW",
    Mode.LOST: "LOST",
    Mode.DISABLED: "DISABLED",
    Mode.RETURN: "RETURN",
}

# Цвета режимов в формате BGR, который использует OpenCV.
MODE_COLORS = {
    Mode.IDLE: (255, 120, 0),
    Mode.CAPTURE: (0, 220, 255),
    Mode.TRACKING: (0, 220, 0),
    Mode.LOST: (0, 0, 255),
    Mode.DISABLED: (0, 0, 255),
    Mode.RETURN: (0, 140, 255),
}


def draw_overlay(frame: Any, mode: Mode, target: TargetBox | None, message: str,
                 config: OsdConfig | None = None) -> Any:
    """Добавляет на кадр ASCII-индикацию без изменения исходного управления."""
    # Значения по умолчанию сохраняют совместимость с тестами и старым вызовом.
    osd = config or OsdConfig(160, 12, 2, 4, 0.8, 2, 0.55, 2)
    height, width = frame.shape[:2]
    # Цвета из конфигурации хранятся в формате BGR OpenCV.
    colors = osd.mode_colors or MODE_COLORS
    color = colors[mode]
    # Текстовые строки отключены: в OSD остаются только графические элементы.
    if target is None:
        # Центральный квадрат показывает область, которая будет захвачена по 2.
        box_size = min(osd.capture_box_size, width, height)
        center_x = round(width * osd.center_x_percent / 100) + osd.center_offset_x
        center_y = round(height * osd.center_y_percent / 100) + osd.center_offset_y
        # Рамка имеет отдельное смещение относительно перекрестия.
        box_center_x = center_x + osd.capture_box_offset_x
        box_center_y = center_y + osd.capture_box_offset_y
        left = box_center_x - box_size // 2
        top = box_center_y - box_size // 2
        cv2.rectangle(frame, (left, top), (left + box_size, top + box_size),
                      color, osd.line_thickness)
        # Перекрестие привязано к центру рамки и не отделяется от неё.
        cv2.line(frame, (box_center_x - osd.crosshair_arm, box_center_y),
                 (box_center_x + osd.crosshair_arm, box_center_y), color, 1)
        cv2.line(frame, (box_center_x, box_center_y - osd.crosshair_arm),
                 (box_center_x, box_center_y + osd.crosshair_arm), color, 1)
        return frame
    left_top = (int(target.x), int(target.y))
    right_bottom = (int(target.x + target.width), int(target.y + target.height))
    target_center = tuple(int(value) for value in target.center())
    frame_center = (width // 2, height // 2)
    cv2.rectangle(frame, left_top, right_bottom, color, osd.line_thickness)
    cv2.line(frame, frame_center, target_center, color, osd.line_thickness)
    cv2.circle(frame, target_center, osd.target_point_radius, color, -1)
    return frame
