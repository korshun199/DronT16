"""Рисует рамку цели, линию к ней и безопасное состояние прототипа."""

from __future__ import annotations

from typing import Any

import cv2

from src.core.state_machine import Mode, TargetBox


MODE_LABELS = {
    Mode.IDLE: "READY",
    Mode.CAPTURE: "CAPTURE",
    Mode.TRACKING: "FOLLOW",
    Mode.LOST: "LOST",
    Mode.DISABLED: "DISABLED",
}

MODE_COLORS = {
    Mode.IDLE: (255, 120, 0),
    Mode.CAPTURE: (0, 220, 255),
    Mode.TRACKING: (0, 220, 0),
    Mode.LOST: (0, 0, 255),
    Mode.DISABLED: (0, 0, 255),
}


def draw_overlay(frame: Any, mode: Mode, target: TargetBox | None, message: str) -> Any:
    """Добавляет на кадр ASCII-индикацию без изменения исходного управления."""
    height, width = frame.shape[:2]
    color = MODE_COLORS[mode]
    cv2.putText(frame, MODE_LABELS[mode], (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    cv2.putText(frame, message[:90], (20, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    if target is None:
        # Центральный квадрат показывает область, которая будет захвачена по 1.
        box_size = min(160, max(40, min(width, height) // 3))
        left = width // 2 - box_size // 2
        top = height // 2 - box_size // 2
        cv2.rectangle(frame, (left, top), (left + box_size, top + box_size), color, 2)
        cv2.line(frame, (width // 2 - 12, height // 2), (width // 2 + 12, height // 2), color, 1)
        cv2.line(frame, (width // 2, height // 2 - 12), (width // 2, height // 2 + 12), color, 1)
        return frame
    left_top = (int(target.x), int(target.y))
    right_bottom = (int(target.x + target.width), int(target.y + target.height))
    target_center = tuple(int(value) for value in target.center())
    frame_center = (width // 2, height // 2)
    cv2.rectangle(frame, left_top, right_bottom, color, 2)
    cv2.line(frame, frame_center, target_center, color, 2)
    cv2.circle(frame, target_center, 4, color, -1)
    return frame
