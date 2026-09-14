"""Безопасная обёртка OpenCV для камеры или записанного видео."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2


def _camera_candidates() -> list[str]:
    """Возвращает USB-видеоустройства раньше прочих камер."""
    by_id = sorted(Path("/dev/v4l/by-id").glob("*"))
    usb_candidates = [str(path) for path in by_id if "usb" in path.name.lower()]
    if usb_candidates:
        return usb_candidates
    return [str(path) for path in sorted(Path("/dev").glob("video*"))]


class VideoSource:
    """Открывает камеру или видеофайл и проверяет ошибки чтения."""

    def __init__(self, source: str) -> None:
        """Открывает путь к видеофайлу или числовой индекс камеры."""
        self.source = source
        sources = _camera_candidates() if source == "auto" else [source]
        self.capture = None
        selected_source = source
        for candidate in sources:
            camera_index = int(candidate) if candidate.isdigit() else None
            capture = cv2.VideoCapture(camera_index if camera_index is not None else candidate)
            if capture.isOpened():
                self.capture = capture
                selected_source = candidate
                break
            capture.release()
        if self.capture is None:
            raise RuntimeError(f"Не удалось открыть видеопоток: {source}")
        self.source = selected_source

    def read(self) -> Any:
        """Возвращает очередной кадр или сообщает об ошибке чтения."""
        success, frame = self.capture.read()
        if not success or frame is None:
            raise EOFError("Видеопоток завершён или кадр не прочитан")
        return frame

    def close(self) -> None:
        """Освобождает камеру или файл."""
        self.capture.release()
