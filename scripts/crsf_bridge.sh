#!/usr/bin/env bash
# Запускает прозрачный CRSF-мост Raspberry -> Betaflight.

set -Eeuo pipefail

# Определяем корень проекта относительно расположения скрипта.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Используем локальное окружение проекта без установки библиотек.
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python3"

if [[ ! -x "${PYTHON_BIN}" ]]; then
    printf '[CRSF BRIDGE] Ошибка: не найдено окружение %s\n' "${PYTHON_BIN}" >&2
    exit 2
fi

cd "${PROJECT_DIR}"
exec "${PYTHON_BIN}" scripts/crsf_bridge.py
