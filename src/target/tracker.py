"""Адаптер OpenCV-трекера без распознавания и поиска новых объектов."""

from __future__ import annotations

from typing import Any, Optional

import cv2

from src.core.state_machine import TargetBox
from src.target.verifier import TargetVerifier


def _make_tracker() -> Any:
    """Создаёт доступный локальный OpenCV-трекер по приоритету качества."""
    constructors = []
    legacy = getattr(cv2, "legacy", None)
    if legacy is not None:
        constructors.extend(
            getattr(legacy, name, None)
            for name in ("TrackerCSRT_create", "TrackerKCF_create", "TrackerMIL_create")
        )
    constructors.extend(
        getattr(cv2, name, None)
        for name in ("TrackerCSRT_create", "TrackerKCF_create", "TrackerMIL_create")
    )
    for constructor in constructors:
        if constructor is not None:
            return constructor()
    raise RuntimeError("В установленном OpenCV нет подходящего трекера")


class TargetTracker:
    """Сопровождает только область, явно переданную пилотом."""

    def __init__(self, verifier: TargetVerifier | None = None) -> None:
        """Создаёт пустой трекер без активной цели."""
        self._tracker: Optional[Any] = None
        self._verifier = verifier

    def start(self, frame: Any, target: TargetBox) -> None:
        """Запускает сопровождение на первом кадре захваченной области."""
        if not target.is_valid():
            raise ValueError("Нельзя начать сопровождение пустой областью")
        self._tracker = _make_tracker()
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
