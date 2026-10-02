#!/usr/bin/env bash
# Единая команда запуска DronT16 на ноутбуке или Raspberry Pi.

set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python3"
REQUIREMENTS_FILE="${PROJECT_DIR}/docs/requirements-laptop.txt"

# Единая рабочая конфигурация всех модулей DronT16.
CONFIG_FILE="config/dront16.toml"

log() {
    # Печатает понятное сообщение текущего шага запуска.
    printf '[DronT16] %s\n' "$1"
}

is_raspberry_pi() {
    # Определяет Raspberry по модели платы, не по наличию произвольного Linux-файла.
    [[ -r /proc/device-tree/model ]] && grep -qi "raspberry pi" /proc/device-tree/model
}

ensure_environment() {
    # Создаёт окружение и устанавливает зависимости только при необходимости.
    if [[ ! -x "${PYTHON_BIN}" ]]; then
        log "Создаю локальное окружение .venv"
        if is_raspberry_pi; then
            # Picamera2/libcamera устанавливаются Raspberry Pi OS как системные пакеты.
            python3 -m venv --system-site-packages "${PROJECT_DIR}/.venv"
        else
            python3 -m venv "${PROJECT_DIR}/.venv"
        fi
    fi

    if ! "${PYTHON_BIN}" -c 'import cv2, flask, numpy' >/dev/null 2>&1; then
        log "Устанавливаю зависимости DronT16"
        "${PYTHON_BIN}" -m pip install -r "${REQUIREMENTS_FILE}"
    fi
    # Читаем только операционные флаги из единого TOML, без копий параметров в shell.
    START_RECEIVER_BRIDGE="$(${PYTHON_BIN} -c 'import tomllib, sys; print("1" if tomllib.load(open(sys.argv[1], "rb"))["runtime"]["start_receiver_bridge"] else "0")' "${PROJECT_DIR}/${CONFIG_FILE}")"
    VIDEO_SOURCE="$(${PYTHON_BIN} -c 'import tomllib, sys; print(tomllib.load(open(sys.argv[1], "rb"))["camera"]["source"])' "${PROJECT_DIR}/${CONFIG_FILE}")"
    if is_raspberry_pi && [[ "${VIDEO_SOURCE}" == "auto" || "${VIDEO_SOURCE}" == "csi" || "${VIDEO_SOURCE}" == "picamera2" ]]; then
        if ! "${PYTHON_BIN}" -c 'import picamera2' >/dev/null 2>&1; then
            log "ОШИБКА: Picamera2 не видна из .venv"
            log "Нужен пакет python3-picamera2 и окружение .venv с --system-site-packages"
            return 2
        fi
    fi
}

main() {
    # Запускает приложение только с настройками из этого файла.
    ensure_environment
    cd "${PROJECT_DIR}"
    # Не открываем вторую копию камеры, если проект уже запущен systemd или
    # другим экземпляром run.sh.
    if [[ "$#" -eq 1 && "$1" == "sensor-test" ]]; then
        log "Запуск локального теста привязки датчиков; реальные порты отключены"
        exec "${PYTHON_BIN}" "${PROJECT_DIR}/scripts/sensor_binding_test.py"
    fi
    if [[ "$#" -ne 0 ]]; then
        log "Допустим только режим sensor-test; рабочий запуск параметров не требует"
        return 2
    fi
    if pgrep -af "${PROJECT_DIR}/.venv/bin/python3 -m src.app" >/dev/null 2>&1; then
        log "DronT16 уже запущен; второй экземпляр камеры не запускаю"
        return 0
    fi
    # Журнал не очищаем: он нужен для последующего разбора испытания.
    if pgrep -af "${PROJECT_DIR}/scripts/crsf_bridge.py" >/dev/null 2>&1; then
        log "Старый CRSF-мост уже запущен; журнал сохраняется"
    else
        log "Рабочий журнал определяется секцией logging в ${CONFIG_FILE}"
    fi
    local output_mode="j7"
    # Автоматический выбор платформы повторяет значение video.display=auto.
    if ! is_raspberry_pi; then
        output_mode="web"
    fi
    local -a app_args=(--config "${CONFIG_FILE}")
    if [[ "${output_mode}" == "hdmi" || "${output_mode}" == "both" || "${output_mode}" == "web" ]]; then
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
    fi
    local bridge_pid=""
    if [[ "${START_RECEIVER_BRIDGE}" == "1" && "${output_mode}" == "j7" ]]; then
        if pgrep -af "scripts/crsf_bridge.py" >/dev/null 2>&1; then
            log "CRSF-мост уже запущен"
        else
            log "Запуск CRSF-моста и чтения MSP; датчики FC будут в этой консоли"
            "${PYTHON_BIN}" "${PROJECT_DIR}/scripts/crsf_bridge.py" &
            bridge_pid=$!
            sleep 0.3
        fi
    fi
    cleanup() {
        # Останавливаем только мост, запущенный этим экземпляром run.sh.
        if [[ -n "${bridge_pid:-}" ]] && kill -0 "${bridge_pid}" 2>/dev/null; then
            kill "${bridge_pid}" 2>/dev/null || true
        fi
    }
    trap cleanup EXIT INT TERM
    log "Запуск DronT16: конфигурация ${CONFIG_FILE}, вывод ${output_mode}"
    "${PYTHON_BIN}" -m src.app "${app_args[@]}"
}

main "$@"
