"""Адаптер OpenCV-трекера без распознавания и поиска новых объектов."""

from __future__ import annotations

from typing import Any, Optional

import cv2

from src.core.state_machine import TargetBox
from src.target.verifier import TargetVerifier


def _make_tracker(algorithm: str = "csrt") -> Any:
    """Создаёт выбранный локальный OpenCV-трекер."""
    names = {
        "csrt": "TrackerCSRT_create",
        "kcf": "TrackerKCF_create",
        "mil": "TrackerMIL_create",
    }
    selected_name = names.get(algorithm)
    if selected_name is None:
        raise ValueError(f"Неизвестный алгоритм трекера: {algorithm}")
    constructors = []
    legacy = getattr(cv2, "legacy", None)
    if legacy is not None:
        constructors.append(getattr(legacy, selected_name, None))
    constructors.append(getattr(cv2, selected_name, None))
    for constructor in constructors:
        if constructor is not None:
            return constructor()
    raise RuntimeError(f"В OpenCV нет трекера {algorithm}")


class TargetTracker:
    """Сопровождает только область, явно переданную пилотом."""

    def __init__(self, verifier: TargetVerifier | None = None, algorithm: str = "csrt") -> None:
        """Создаёт пустой трекер без активной цели."""
        self._tracker: Optional[Any] = None
        self._verifier = verifier
        self._algorithm = algorithm

    def start(self, frame: Any, target: TargetBox) -> None:
        """Запускает сопровождение на первом кадре захваченной области."""
        if not target.is_valid():
            raise ValueError("Нельзя начать сопровождение пустой областью")
        self._tracker = _make_tracker(self._algorithm)
        initialized = self._tracker.init(
            frame,
            (int(target.x), int(target.y), int(target.width), int(target.height)),
        )
        if initialized is False:
            self._tracker = None
            raise RuntimeError("OpenCV не смог инициализировать сопровождение")
        if self._verifier is not None:
            self._verifier.start(frame, target)

    def update(self, frame: Any) -> Optional[TargetBox]:
        """Возвращает новую область прежней цели или None при потере."""
        if self._tracker is None:
            return None
        success, box = self._tracker.update(frame)
        if not success:
            self._tracker = None
            return None
        x, y, width, height = box
        target = TargetBox(float(x), float(y), float(width), float(height))
        if self._verifier is not None and not self._verifier.verify(frame, target):
            self._tracker = None
            return None
        return target

    def reset(self) -> None:
        """Останавливает текущее сопровождение без выбора новой цели."""
        self._tracker = None
        if self._verifier is not None:
            self._verifier.reset()
