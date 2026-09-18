#!/usr/bin/env bash
# Запускает локальную проверку UART Raspberry перемычкой TXD0 -> RXD0.

set -Eeuo pipefail

# Определяем корень проекта относительно расположения скрипта.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Используем локальное окружение проекта без установки библиотек.
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python3"

if [[ ! -x "${PYTHON_BIN}" ]]; then
    printf '[UART LOOPBACK] Ошибка: не найдено окружение %s\n' "${PYTHON_BIN}" >&2
    exit 2
fi

cd "${PROJECT_DIR}"
exec "${PYTHON_BIN}" scripts/uart_loopback_test.py "$@"
