#!/usr/bin/env bash
# Запускает цветную диагностику CRSF без передачи в полётник.

set -Eeuo pipefail

# Определяем корень проекта относительно расположения скрипта.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Используем локальное окружение проекта без установки библиотек.
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python3"

if [[ ! -x "${PYTHON_BIN}" ]]; then
    printf '[BRIDGE DIAG] Ошибка: не найдено окружение %s\n' "${PYTHON_BIN}" >&2
    exit 2
fi

cd "${PROJECT_DIR}"
exec "${PYTHON_BIN}" scripts/crsf_bridge_diagnostic.py
