"""Единый загрузчик конфигурации DronT16."""

from __future__ import annotations

import tomllib
from pathlib import Path
from typing import Any


DEFAULT_CONFIG_PATH = Path("config/dront16.toml")


def load_project_config(path: str | Path = DEFAULT_CONFIG_PATH) -> dict[str, Any]:
    """Читает единый TOML-файл и возвращает все секции проекта."""
    config_path = Path(path)
    try:
        with config_path.open("rb") as config_file:
            raw = tomllib.load(config_file)
    except (OSError, tomllib.TOMLDecodeError) as error:
        raise ValueError(f"Не удалось прочитать конфигурацию {config_path}: {error}") from error
    if not isinstance(raw, dict):
        raise ValueError(f"Конфигурация {config_path} должна содержать TOML-таблицы")
    return raw


def load_config_section(path: str | Path, *section_names: str) -> dict[str, Any]:
    """Возвращает отдельную секцию единого конфига с проверкой пути."""
    section: Any = load_project_config(path)
    for section_name in section_names:
        if not isinstance(section, dict) or section_name not in section:
            raise ValueError(f"В конфигурации нет секции {'/'.join(section_names)}")
        section = section[section_name]
    if not isinstance(section, dict):
        raise ValueError(f"Секция {'/'.join(section_names)} должна быть таблицей")
    return section
