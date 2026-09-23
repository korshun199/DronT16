"""Проверка сохранения выбранной цели без распознавания её класса."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from src.core.state_machine import TargetBox


class TargetVerifier:
    """Сравнивает цветовую структуру и изображение выбранной области."""

    def __init__(self, min_similarity: float = 0.35, max_bad_frames: int = 5,
                 method: str = "appearance", adaptation_rate: float = 0.05,
                 foreground_margin_percent: float = 15.0) -> None:
        """Создаёт проверяющий модуль с порогом и запасом кадров."""
        self.min_similarity = min_similarity
        self.max_bad_frames = max_bad_frames
        self.method = method
        self.adaptation_rate = adaptation_rate
        self.foreground_margin_percent = foreground_margin_percent
        self._template: Any = None
        self._histogram: Any = None
        self._bad_frames = 0
        # Полный кадр в момент захвата нужен для оценки заполнения вне рамки.
        self._reference_frame: Any = None

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
        self._reference_frame = frame.copy()
        sample = self._foreground_crop(crop)
        self._template = cv2.resize(sample, (64, 64), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(sample, cv2.COLOR_BGR2HSV)
        self._histogram = cv2.normalize(
            cv2.calcHist([hsv], [0, 1], None, [16, 16], [0, 180, 0, 256]),
            None,
            0,
            1,
            cv2.NORM_MINMAX,
        )
        self._bad_frames = 0

    def _foreground_crop(self, crop: Any) -> Any:
        """Выделяет вероятную переднюю область и подавляет внешний фон."""
        if self.method != "foreground":
            return crop
        foreground = self._foreground_mask(crop)
        if foreground is None:
            return crop
        result = crop.copy()
        result[~foreground] = 0
        return result

    def _foreground_mask(self, crop: Any) -> Any | None:
        """Возвращает маску объекта в рамке или None при неудаче выделения."""
        height, width = crop.shape[:2]
        margin_x = int(width * self.foreground_margin_percent / 100.0)
        margin_y = int(height * self.foreground_margin_percent / 100.0)
        mask = np.full((height, width), cv2.GC_BGD, dtype=np.uint8)
        inner_top = margin_y
        inner_bottom = max(inner_top + 1, height - margin_y)
        inner_left = margin_x
        inner_right = max(inner_left + 1, width - margin_x)
        mask[inner_top:inner_bottom, inner_left:inner_right] = cv2.GC_PR_FGD
        # Центральная часть области считается наиболее вероятным объектом.
        center_margin_x = max(1, (inner_right - inner_left) // 5)
        center_margin_y = max(1, (inner_bottom - inner_top) // 5)
        mask[inner_top + center_margin_y:inner_bottom - center_margin_y,
             inner_left + center_margin_x:inner_right - center_margin_x] = cv2.GC_FGD
        try:
            background_model = np.zeros((1, 65), dtype=np.float64)
            foreground_model = np.zeros((1, 65), dtype=np.float64)
            cv2.grabCut(crop, mask, None, background_model, foreground_model,
                        2, cv2.GC_INIT_WITH_MASK)
            foreground = (mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD)
            if int(foreground.sum()) >= max(16, crop.shape[0] * crop.shape[1] // 20):
                return foreground
        except cv2.error:
            return None
        return None

    def object_area_percent(self, frame: Any, target: TargetBox) -> float | None:
        """Считает площадь маски объекта относительно всего кадра."""
        crop = self._crop(frame, target)
        if crop is None or crop.size == 0:
            return None
        foreground = self._foreground_mask(crop)
        if foreground is None:
            return None
        frame_area = float(frame.shape[0] * frame.shape[1])
        return float(foreground.sum()) / frame_area * 100.0

    def frame_fill_percent(self, frame: Any, target: TargetBox) -> float | None:
        """Оценивает заполнение кадра объектом и изменившейся областью."""
        object_percent = self.object_area_percent(frame, target)
        changed_percent = self._changed_area_percent(frame)
        if object_percent is None and self._reference_frame is None:
            return None
        # Изменение полного кадра используется только для отдельного порога
        # контроля, но не участвует в знаковом размере объекта.
        return max(object_percent or 0.0, changed_percent)

    def _changed_area_percent(self, frame: Any) -> float:
        """Оценивает долю кадра, изменившуюся после захвата цели."""
        if self._reference_frame is None or self._reference_frame.shape != frame.shape:
            return 0.0
        reference_gray = cv2.cvtColor(self._reference_frame, cv2.COLOR_BGR2GRAY)
        current_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        difference = cv2.absdiff(current_gray, reference_gray)
        # 25 уровней подавляют шум сенсора и небольшие изменения яркости.
        changed = (difference >= 25).astype("uint8") * 255
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        changed = cv2.morphologyEx(changed, cv2.MORPH_OPEN, kernel)
        changed = cv2.morphologyEx(changed, cv2.MORPH_CLOSE, kernel)
        return float(cv2.countNonZero(changed)) / (frame.shape[0] * frame.shape[1]) * 100.0

    def verify(self, frame: Any, target: TargetBox) -> bool:
        """Проверяет цветовую и визуальную близость найденной области."""
        crop = self._crop(frame, target)
        if crop is None or self._template is None or self._histogram is None:
            return False
        sample = self._foreground_crop(crop)
        resized = cv2.resize(sample, (64, 64), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(sample, cv2.COLOR_BGR2HSV)
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
            if self.method == "adaptive":
                # Медленно обновляем образец только после уверенного совпадения.
                rate = self.adaptation_rate
                self._template = cv2.addWeighted(self._template, 1.0 - rate, resized, rate, 0)
                self._histogram = cv2.addWeighted(self._histogram, 1.0 - rate,
                                                   histogram, rate, 0)
            return True
        self._bad_frames += 1
        return self._bad_frames < self.max_bad_frames

    def reset(self) -> None:
        """Удаляет образец и состояние проверки цели."""
        self._template = None
        self._histogram = None
        self._bad_frames = 0
        self._reference_frame = None
