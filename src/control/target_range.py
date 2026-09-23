"""Оценка горизонтального смещения и относительной дальности цели.

Модуль не знает расстояние в метрах. Он использует положение центра рамки и
изменение её площади: увеличение объекта считается приближением, уменьшение —
удалением. Команды полётнику из этого модуля не формируются.
"""

from __future__ import annotations

from dataclasses import dataclass

from src.core.state_machine import TargetBox


@dataclass(frozen=True)
class RangeEstimatorConfig:
    """Пороговые параметры относительного измерения цели."""

    horizontal_deadband_percent: float
    size_change_deadband_percent: float
    smoothing_alpha: float
    control_threshold_percent: float


@dataclass(frozen=True)
class TargetMeasurement:
    """Один результат измерения для терминала и последующего анализа."""

    horizontal_position: str
    distance_trend: str
    horizontal_offset_percent: float
    object_size: float
    under_control: bool
    object_area_percent: float
    capture_size_percent: float
    relative_size_change_percent: float
    distance_to_object_percent: float
    control_gap_percent: float


class TargetRangeEstimator:
    """Сравнивает текущую рамку только с предыдущей рамкой той же цели."""

    def __init__(self, config: RangeEstimatorConfig) -> None:
        """Создаёт измеритель с пустой историей объекта."""
        if config.horizontal_deadband_percent < 0:
            raise ValueError("horizontal_deadband_percent не может быть отрицательным")
        if config.size_change_deadband_percent < 0:
            raise ValueError("size_change_deadband_percent не может быть отрицательным")
        if not 0 < config.smoothing_alpha <= 1:
            raise ValueError("smoothing_alpha должен быть больше 0 и не больше 1")
        if not 0 < config.control_threshold_percent <= 100:
            raise ValueError("control_threshold_percent должен быть от 0 до 100")
        self.config = config
        self._previous_size: float | None = None
        self._smoothed_size: float | None = None
        # Размер рамки в момент захвата считается исходным размером 100%.
        self._capture_size: float | None = None

    def reset(self) -> None:
        """Очищает историю при новом захвате или отбое."""
        self._previous_size = None
        self._smoothed_size = None
        self._capture_size = None

    def update(self, target: TargetBox, frame_width: int, frame_height: int,
               measured_area_percent: float | None = None,
               control_area_percent: float | None = None) -> TargetMeasurement:
        """Возвращает положение цели и размер выделенного объекта."""
        if frame_width <= 0 or frame_height <= 0 or not target.is_valid():
            raise ValueError("Для измерения нужны положительные размеры кадра и рамка цели")
        center_x, _ = target.center()
        offset_percent = (center_x - frame_width / 2.0) / (frame_width / 2.0) * 100.0
        deadband = self.config.horizontal_deadband_percent
        if offset_percent < -deadband:
            horizontal = "ОБЪЕКТ СМЕЩАЕТСЯ ВЛЕВО"
        elif offset_percent > deadband:
            horizontal = "ОБЪЕКТ СМЕЩАЕТСЯ ВПРАВО"
        else:
            horizontal = "ЦЕНТР"

        frame_area = float(frame_width * frame_height)
        if measured_area_percent is None:
            # Запасной путь для тестов без изображения; на рабочем пути
            # используется площадь объекта, выделенная проверяющим модулем.
            raw_size = max(1.0, target.width * target.height)
            object_area_percent = raw_size / frame_area * 100.0
        else:
            object_area_percent = max(0.0, min(100.0, measured_area_percent))
            raw_size = max(0.01, object_area_percent)
        if self._smoothed_size is None:
            self._smoothed_size = raw_size
        else:
            alpha = self.config.smoothing_alpha
            self._smoothed_size = alpha * raw_size + (1.0 - alpha) * self._smoothed_size
        if self._capture_size is None:
            # Захват задаёт нулевую дистанцию и исходный размер 100%.
            self._capture_size = self._smoothed_size
        previous_size = self._previous_size
        self._previous_size = self._smoothed_size
        if previous_size is None:
            trend = "ЦЕНТР"
        else:
            change_percent = (self._smoothed_size - previous_size) / previous_size * 100.0
            threshold = self.config.size_change_deadband_percent
            if change_percent > threshold:
                trend = "ОБЪЕКТ ПРИБЛИЖАЕТСЯ"
            elif change_percent < -threshold:
                trend = "ОБЪЕКТ УДАЛЯЕТСЯ"
            else:
                trend = "ЦЕНТР"
        # При отсутствии маски нельзя объявлять объект под контролем по одной
        # лишь рамке: её размер не является размером фактического объекта.
        control_area = object_area_percent if control_area_percent is None else max(
            0.0, min(100.0, control_area_percent)
        )
        control_available = control_area_percent is not None or measured_area_percent is not None
        under_control = control_available and control_area >= self.config.control_threshold_percent
        control_gap_percent = max(0.0, self.config.control_threshold_percent - control_area)
        capture_size_percent = self._smoothed_size / self._capture_size * 100.0
        relative_size_change_percent = capture_size_percent - 100.0
        # Это относительный индикатор, а не метры: при захвате всегда 0%.
        # При уменьшении объекта показатель растёт, при приближении стремится к 0%.
        distance_to_object_percent = max(0.0, 100.0 - capture_size_percent)
        return TargetMeasurement(
            horizontal,
            trend,
            offset_percent,
            self._smoothed_size,
            under_control,
            control_area,
            capture_size_percent,
            relative_size_change_percent,
            distance_to_object_percent,
            control_gap_percent,
        )
