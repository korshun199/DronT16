"""Загрузка и проверка параметров модуля сопровождения."""

from __future__ import annotations

import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class CameraConfig:
    """Параметры проекции камеры."""

    horizontal_fov_deg: float
    vertical_fov_deg: float
    center_x_percent: float
    center_y_percent: float


@dataclass(frozen=True)
class GuidanceConfig:
    """Ограничения расчёта направления и частота диагностики."""

    yaw_deadband_deg: float
    pitch_deadband_deg: float
    max_yaw_rate_deg_s: float
    max_pitch_rate_deg_s: float
    report_period_ms: int


@dataclass(frozen=True)
class MspConfig:
    """Настройки будущего транспорта MSP."""

    enabled: bool
    transport: str
    serial_port: str
    baudrate: int


@dataclass(frozen=True)
class FollowConfig:
    """Полная конфигурация сопровождения."""

    camera: CameraConfig
    guidance: GuidanceConfig
    msp: MspConfig


def _number(section: dict[str, Any], name: str) -> float:
    """Возвращает числовой параметр или сообщает о повреждённой конфигурации."""
    value = section.get(name)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ValueError(f"Параметр {name} должен быть числом")
    return float(value)


def load_follow_config(path: str | Path) -> FollowConfig:
    """Загружает TOML и проверяет параметры до запуска сопровождения."""
    config_path = Path(path)
    try:
        with config_path.open("rb") as config_file:
            raw = tomllib.load(config_file)
    except (OSError, tomllib.TOMLDecodeError) as error:
        raise ValueError(f"Не удалось прочитать конфигурацию {config_path}: {error}") from error
    try:
        camera = raw["camera"]
        guidance = raw["guidance"]
        msp = raw["msp"]
        result = FollowConfig(
            CameraConfig(
                _number(camera, "horizontal_fov_deg"),
                _number(camera, "vertical_fov_deg"),
                _number(camera, "center_x_percent"),
                _number(camera, "center_y_percent"),
            ),
            GuidanceConfig(
                _number(guidance, "yaw_deadband_deg"),
                _number(guidance, "pitch_deadband_deg"),
                _number(guidance, "max_yaw_rate_deg_s"),
                _number(guidance, "max_pitch_rate_deg_s"),
                int(_number(guidance, "report_period_ms")),
            ),
            MspConfig(
                bool(msp["enabled"]),
                str(msp["transport"]),
                str(msp["serial_port"]),
                int(msp["baudrate"]),
            ),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Ошибка параметров конфигурации {config_path}: {error}") from error
    if not 0 < result.camera.horizontal_fov_deg < 180 or not 0 < result.camera.vertical_fov_deg < 180:
        raise ValueError("Угол обзора камеры должен быть от 0 до 180 градусов")
    if result.guidance.report_period_ms <= 0:
        raise ValueError("report_period_ms должен быть положительным")
    if result.msp.transport != "dry-run" or result.msp.enabled:
        raise ValueError("До отдельного разрешения MSP должен оставаться в режиме dry-run")
    return result
