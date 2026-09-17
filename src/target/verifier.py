"""Проверка сохранения выбранной цели без распознавания её класса."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from src.core.state_machine import TargetBox


class TargetVerifier:
    """Сравнивает цветовую структуру и изображение выбранной области."""

    def __init__(self, min_similarity: float = 0.35, max_bad_frames: int = 5) -> None:
        """Создаёт проверяющий модуль с порогом и запасом кадров."""
        self.min_similarity = min_similarity
        self.max_bad_frames = max_bad_frames
        self._template: Any = None
        self._histogram: Any = None
        self._bad_frames = 0

    @staticmethod
    def _crop(frame: Any, target: TargetBox) -> Any | None:
        """Возвращает ограниченную границами кадра область цели."""
        height, width = frame.shape[:2]
        left = max(0, int(target.x))
        top = max(0, int(target.y))
        right = min(width, int(target.x + target.width))
        bottom = min(height, int(target.y + target.height))
        if right <= left or bottom <= top:
            return None
        return frame[top:bottom, left:right]

    def start(self, frame: Any, target: TargetBox) -> None:
        """Сохраняет образец области, выбранной пилотом."""
        crop = self._crop(frame, target)
        if crop is None or crop.size == 0:
            raise ValueError("Нельзя сохранить пустой образец цели")
        self._template = cv2.resize(crop, (64, 64), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        self._histogram = cv2.normalize(
            cv2.calcHist([hsv], [0, 1], None, [16, 16], [0, 180, 0, 256]),
            None,
            0,
            1,
            cv2.NORM_MINMAX,
        )
        self._bad_frames = 0

    def verify(self, frame: Any, target: TargetBox) -> bool:
        """Проверяет цветовую и визуальную близость найденной области."""
        crop = self._crop(frame, target)
        if crop is None or self._template is None or self._histogram is None:
            return False
        resized = cv2.resize(crop, (64, 64), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        histogram = cv2.normalize(
            cv2.calcHist([hsv], [0, 1], None, [16, 16], [0, 180, 0, 256]),
            None,
            0,
            1,
            cv2.NORM_MINMAX,
        )
        histogram_score = max(0.0, float(cv2.compareHist(self._histogram, histogram, cv2.HISTCMP_CORREL)))
        template_gray = cv2.cvtColor(self._template, cv2.COLOR_BGR2GRAY)
        resized_gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        if float(template_gray.std()) < 2.0 or float(resized_gray.std()) < 2.0:
            similarity = histogram_score
        else:
            template_score = float(cv2.matchTemplate(
                resized_gray, template_gray, cv2.TM_CCOEFF_NORMED
            )[0][0])
            similarity = 0.5 * histogram_score + 0.5 * max(0.0, template_score)
        if similarity >= self.min_similarity:
            self._bad_frames = 0
            return True
        self._bad_frames += 1
        return self._bad_frames < self.max_bad_frames

    def reset(self) -> None:
        """Удаляет образец и состояние проверки цели."""
        self._template = None
        self._histogram = None
        self._bad_frames = 0
