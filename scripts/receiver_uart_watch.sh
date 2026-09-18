#!/usr/bin/env bash
# Запускает фоновый диагностический слушатель UART приёмника.

set -Eeuo pipefail

# Определяем корень проекта относительно расположения этого скрипта.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Используем локальное окружение проекта без установки новых библиотек.
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python3"

if [[ ! -x "${PYTHON_BIN}" ]]; then
    printf '[RX WATCH] Ошибка: не найдено окружение %s\n' "${PYTHON_BIN}" >&2
    exit 2
fi

cd "${PROJECT_DIR}"
exec "${PYTHON_BIN}" scripts/receiver_uart_watch.py
