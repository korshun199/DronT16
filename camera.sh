#!/usr/bin/env bash
# Ручной запуск веб-настройки цифровой камеры DronT16.

set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python3"

if [[ ! -x "${PYTHON_BIN}" ]]; then
    printf '[DronT16] ОШИБКА: нет .venv. Сначала один раз запусти ./run.sh.\n' >&2
    exit 2
fi
if ! "${PYTHON_BIN}" -c 'import flask' >/dev/null 2>&1; then
    printf '[DronT16] ОШИБКА: в .venv нет Flask. Нужна обычная подготовка через ./run.sh.\n' >&2
    exit 2
fi

cd "${PROJECT_DIR}"
exec "${PYTHON_BIN}" -m src.interface.camera_web --config config/dront16.toml
