#!/usr/bin/env bash
# Единая команда запуска DronT16 на ноутбуке или Raspberry Pi.

set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python3"
REQUIREMENTS_FILE="${PROJECT_DIR}/requirements-laptop.txt"

# Настройки проекта. Владелец запускает только ./run.sh, параметры меняются здесь.
# Источник видео: auto выбирает подключённую USB-камеру.
VIDEO_SOURCE="auto"
# Вывод: auto выбирает J7 на Raspberry Pi и веб-морду на ноутбуке.
DISPLAY_MODE="auto"
# Адрес локальной веб-морды для просмотра.
WEB_HOST="127.0.0.1"
# Порт локальной веб-морды.
WEB_PORT="8080"
# Признак полноэкранного HDMI-вывода: 1 — включить.
HDMI_FULLSCREEN="1"
# Конфигурация размеров и оформления OSD.
OSD_CONFIG="config/osd.toml"
# Конфигурация расчёта сопровождения и безопасного MSP dry-run.
FOLLOW_CONFIG="config/follow.toml"
# DRM-устройство композитного выхода J7 Raspberry Pi 5.
J7_DEVICE="/dev/dri/by-path/platform-1f00144000.vec-card"
# Файл команд временного SSH-пульта с ноутбука.
CONTROL_FILE="/tmp/dront16_command"
# Запуск CRSF-моста вместе с видеомодулем на Raspberry Pi.
START_RECEIVER_BRIDGE="1"
# Симулятор failsafe отключён: текущая проверка использует реальные датчики FC.
START_FILESAFE_SIMULATOR="0"
# Конфигурация и журнал симулятора находятся рядом с проектом.
SIMULATOR_SCRIPT="scripts/simulator_filesafe.py"
# Рабочий журнал моста: очищается перед новым запуском DronT16.
RUNTIME_LOG="simulator_filesafe.log"

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
    # Не открываем вторую копию камеры, если проект уже запущен systemd или
    # другим экземпляром run.sh.
    if pgrep -af "${PROJECT_DIR}/.venv/bin/python3 -m src.app" >/dev/null 2>&1; then
        log "DronT16 уже запущен; второй экземпляр камеры не запускаю"
        return 0
    fi
    # Не очищаем журнал, если старый CRSF-мост ещё работает и может дописывать его.
    if pgrep -af "${PROJECT_DIR}/scripts/crsf_bridge.py" >/dev/null 2>&1; then
        log "Старый CRSF-мост уже запущен; журнал не очищаю"
    else
        # Новый запуск всегда начинает рабочий журнал с чистого файла.
        : > "${PROJECT_DIR}/${RUNTIME_LOG}"
        log "Рабочий журнал очищен: ${RUNTIME_LOG}"
    fi
    local output_mode="${DISPLAY_MODE}"
    # На Raspberry Pi рабочим выходом проекта является J7, даже если
    # подключённый монитор временно не сообщает DRM о соединении.
    if [[ "${output_mode}" == "auto" ]]; then
        if [[ -e /proc/device-tree/model ]] && grep -qi "raspberry pi" /proc/device-tree/model; then
            output_mode="j7"
        else
            output_mode="web"
        fi
    fi
    local -a app_args=(--source "${VIDEO_SOURCE}" --display "${output_mode}" --osd-config "${OSD_CONFIG}" --follow-config "${FOLLOW_CONFIG}" --j7-device "${J7_DEVICE}" --control-file "${CONTROL_FILE}")
    if [[ "${output_mode}" == "web" || "${output_mode}" == "both" ]]; then
        app_args+=(--web-host "${WEB_HOST}" --web-port "${WEB_PORT}")
    fi
    if [[ "${output_mode}" == "hdmi" || "${output_mode}" == "both" ]]; then
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
    local bridge_pid=""
    local simulator_pid=""
    if [[ "${START_RECEIVER_BRIDGE}" == "1" && "${output_mode}" == "j7" ]]; then
        if pgrep -af "scripts/crsf_bridge.py" >/dev/null 2>&1; then
            log "CRSF-мост уже запущен"
        else
            log "Запуск CRSF-моста и чтения MSP; датчики FC будут в этой консоли"
            "${PROJECT_DIR}/scripts/crsf_bridge.sh" &
            bridge_pid=$!
            sleep 0.3
        fi
    fi
    if [[ "${START_FILESAFE_SIMULATOR}" == "1" ]]; then
        log "Запуск simulator_filesafe; команды FC только SIMULATED"
        python3 "${SIMULATOR_SCRIPT}" &
        simulator_pid=$!
    fi
    cleanup() {
        # Останавливаем только мост, запущенный этим экземпляром run.sh.
        if [[ -n "${bridge_pid:-}" ]] && kill -0 "${bridge_pid}" 2>/dev/null; then
            kill "${bridge_pid}" 2>/dev/null || true
        fi
        # Останавливаем только симулятор, запущенный этим экземпляром run.sh.
        if [[ -n "${simulator_pid:-}" ]] && kill -0 "${simulator_pid}" 2>/dev/null; then
            kill "${simulator_pid}" 2>/dev/null || true
        fi
    }
    trap cleanup EXIT INT TERM
    log "Запуск DronT16: камера ${VIDEO_SOURCE}, вывод ${output_mode}"
    "${PYTHON_BIN}" -m src.app "${app_args[@]}"
}

main "$@"
