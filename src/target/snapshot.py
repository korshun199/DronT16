"""Однократное сохранение модели цели, выбранной пилотом."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Any

import cv2

from src.core.state_machine import TargetBox


def save_capture_snapshot(frame: Any, target: TargetBox, directory: str | Path) -> Path:
    """Сохраняет PNG ровно из рамки успешного захвата и возвращает его путь."""
    if frame is None or not hasattr(frame, "shape") or len(frame.shape) < 2:
        raise ValueError("Кадр захвата имеет неверный формат")
    if not target.is_valid():
        raise ValueError("Нельзя сохранить пустую область цели")

    frame_height, frame_width = frame.shape[:2]
    left = max(0, min(frame_width, int(target.x)))
    top = max(0, min(frame_height, int(target.y)))
    right = max(left, min(frame_width, int(target.x + target.width)))
    bottom = max(top, min(frame_height, int(target.y + target.height)))
    crop = frame[top:bottom, left:right].copy()
    if crop.size == 0:
        raise ValueError("Рамка цели находится вне изображения")

    output_directory = Path(directory)
    output_directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    output_path = output_directory / f"target_{timestamp}.png"
    suffix = 1
    while output_path.exists():
        output_path = output_directory / f"target_{timestamp}_{suffix:02d}.png"
        suffix += 1
    if not cv2.imwrite(str(output_path), crop):
        raise OSError(f"OpenCV не записал снимок цели: {output_path}")
    return output_path
