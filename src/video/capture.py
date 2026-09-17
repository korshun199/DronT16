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
            # Для V4L2 явно выбираем Linux-драйвер, чтобы EasyCap не открывался
            # через неподходящий универсальный backend OpenCV.
            capture_target = camera_index if camera_index is not None else candidate
            capture = cv2.VideoCapture(capture_target, cv2.CAP_V4L2)
            if capture.isOpened():
                self._configure_v4l2(capture)
                self.capture = capture
                selected_source = candidate
                break
            capture.release()
        if self.capture is None:
            raise RuntimeError(f"Не удалось открыть видеопоток: {source}")
        self.source = selected_source

    @staticmethod
    def _configure_v4l2(capture: cv2.VideoCapture) -> None:
        """Настраивает EasyCap на MJPG и очередь из одного актуального кадра."""
        # MJPG является рабочим форматом аналогового USB-захвата EasyCap.
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        # Полный режим MacroSilicon для NTSC: 720x480 при 25 кадрах в секунду.
        # Явная фиксация не даёт OpenCV самопроизвольно перейти к 480x320.
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        capture.set(cv2.CAP_PROP_FPS, 25)
        # Не накапливаем задержку из старых кадров в очереди захвата.
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def read(self) -> Any:
        """Возвращает очередной кадр или сообщает об ошибке чтения."""
        success, frame = self.capture.read()
        if not success or frame is None:
            raise EOFError("Видеопоток завершён или кадр не прочитан")
        return frame

    def close(self) -> None:
        """Освобождает камеру или файл."""
        self.capture.release()
