#!/usr/bin/env bash
# Запускает модуль №1 проверки CRSF-приёмника без управления полётником.

set -Eeuo pipefail

# Корень проекта вычисляется относительно расположения скрипта.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Используем Python проекта без установки дополнительных библиотек.
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python3"

if [[ ! -x "${PYTHON_BIN}" ]]; then
    printf '[RX TEST] Ошибка: не найдено окружение %s\n' "${PYTHON_BIN}" >&2
    exit 2
fi

cd "${PROJECT_DIR}"
exec "${PYTHON_BIN}" scripts/receiver_link_test.py
