"""Рисует рамку цели, линию к ней и безопасное состояние прототипа."""

from __future__ import annotations

from typing import Any

import cv2

from src.core.state_machine import Mode, TargetBox


MODE_LABELS = {
    Mode.IDLE: "IDLE",
    Mode.TRACKING: "FOLLOW",
    Mode.LOST: "TARGET LOST",
}


def draw_overlay(frame: Any, mode: Mode, target: TargetBox | None, message: str) -> Any:
    """Добавляет на кадр ASCII-индикацию без изменения исходного управления."""
    height, width = frame.shape[:2]
    color = (0, 220, 0) if target is not None and mode is Mode.TRACKING else (0, 180, 255)
    cv2.putText(frame, f"DronT16: {MODE_LABELS[mode]}", (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    cv2.putText(frame, message[:90], (20, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    if target is None:
        return frame
    left_top = (int(target.x), int(target.y))
    right_bottom = (int(target.x + target.width), int(target.y + target.height))
    target_center = tuple(int(value) for value in target.center())
    frame_center = (width // 2, height // 2)
    cv2.rectangle(frame, left_top, right_bottom, color, 2)
    cv2.line(frame, frame_center, target_center, color, 2)
    cv2.circle(frame, target_center, 4, color, -1)
    return frame
