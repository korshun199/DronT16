"""Расчёт координат цели и углового направления без управления дроном."""

from __future__ import annotations

from dataclasses import dataclass

from src.control.follow_config import FollowConfig
from src.core.state_machine import TargetBox


@dataclass(frozen=True)
class GuidanceResult:
    """Измерение цели в кадре и безопасная команда для диагностики."""

    target_x: float
    target_y: float
    normalized_x: float
    normalized_y: float
    yaw_error_deg: float
    pitch_error_deg: float
    yaw_rate_deg_s: float
    pitch_rate_deg_s: float


def _clamp(value: float, limit: float) -> float:
    """Ограничивает команду симметричным пределом."""
    return max(-limit, min(limit, value))


def calculate_guidance(target: TargetBox, frame_width: int, frame_height: int,
                      config: FollowConfig) -> GuidanceResult:
    """Переводит положение цели в пикселях в угловую ошибку камеры."""
    if frame_width <= 0 or frame_height <= 0:
        raise ValueError("Размер кадра должен быть положительным")
    target_x, target_y = target.center()
    center_x = frame_width * config.camera.center_x_percent / 100.0
    center_y = frame_height * config.camera.center_y_percent / 100.0
    normalized_x = (target_x - center_x) / (frame_width / 2.0)
    normalized_y = (target_y - center_y) / (frame_height / 2.0)
    yaw_error = normalized_x * config.camera.horizontal_fov_deg / 2.0
    pitch_error = -normalized_y * config.camera.vertical_fov_deg / 2.0
    if abs(yaw_error) <= config.guidance.yaw_deadband_deg:
        yaw_rate = 0.0
    else:
        yaw_rate = _clamp(yaw_error, config.guidance.max_yaw_rate_deg_s)
    if abs(pitch_error) <= config.guidance.pitch_deadband_deg:
        pitch_rate = 0.0
    else:
        pitch_rate = _clamp(pitch_error, config.guidance.max_pitch_rate_deg_s)
    return GuidanceResult(target_x, target_y, normalized_x, normalized_y,
                          yaw_error, pitch_error, yaw_rate, pitch_rate)
