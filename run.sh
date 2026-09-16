#!/usr/bin/env bash
# Единая команда запуска DronT16 на ноутбуке или Raspberry Pi.

set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python3"
REQUIREMENTS_FILE="${PROJECT_DIR}/requirements-laptop.txt"

# Настройки проекта. Владелец запускает только ./run.sh, параметры меняются здесь.
VIDEO_SOURCE="auto"
DISPLAY_MODE="web"
WEB_HOST="127.0.0.1"
WEB_PORT="8080"
HDMI_FULLSCREEN="1"
CAPTURE_BOX_SIZE="160"
FOLLOW_CONFIG="config/follow.json"

log() {
    # Печатает понятное сообщение текущего шага запуска.
    printf '[DronT16] %s\n' "$1"
}

ensure_environment() {
    # Создаёт окружение и устанавливает зависимости только при необходимости.
    if [[ ! -x "${PYTHON_BIN}" ]]; then
        log "Создаю локальное окружение .venv"
        python3 -m venv "${PROJECT_DIR}/.venv"
    fi

    if ! "${PYTHON_BIN}" -c 'import cv2, flask, numpy' >/dev/null 2>&1; then
        log "Устанавливаю зависимости DronT16"
        "${PYTHON_BIN}" -m pip install -r "${REQUIREMENTS_FILE}"
    fi
}

main() {
    # Запускает приложение только с настройками из этого файла.
    ensure_environment
    cd "${PROJECT_DIR}"
    if [[ "$#" -ne 0 ]]; then
        log "Параметры не нужны: все настройки находятся внутри run.sh"
        return 2
    fi
    local -a app_args=(--source "${VIDEO_SOURCE}" --display "${DISPLAY_MODE}" --capture-size "${CAPTURE_BOX_SIZE}" --follow-config "${FOLLOW_CONFIG}")
    if [[ "${DISPLAY_MODE}" == "web" || "${DISPLAY_MODE}" == "both" ]]; then
        app_args+=(--web-host "${WEB_HOST}" --web-port "${WEB_PORT}")
    fi
    if [[ "${DISPLAY_MODE}" == "hdmi" || "${DISPLAY_MODE}" == "both" ]]; then
        local hdmi_x="0"
        local hdmi_y="0"
        # Берём координаты внешнего подключённого экрана из раскладки Ubuntu.
        if command -v xrandr >/dev/null 2>&1; then
            read -r hdmi_x hdmi_y < <(
                xrandr --query 2>/dev/null \
                    | sed -nE '/^(HDMI|DP|DVI|VGA)[^ ]* connected /s/.* ([0-9]+x[0-9]+\+(-?[0-9]+)\+(-?[0-9]+).*)/\2 \3/p' \
                    | head -n 1
            ) || true
        fi
        app_args+=(--hdmi-x "${hdmi_x:-0}" --hdmi-y "${hdmi_y:-0}")
        if [[ "${HDMI_FULLSCREEN}" == "1" ]]; then
            app_args+=(--fullscreen)
        fi
    fi
    log "Запуск DronT16: камера ${VIDEO_SOURCE}, вывод ${DISPLAY_MODE}"
    exec "${PYTHON_BIN}" -m src.app "${app_args[@]}"
}

main "$@"
