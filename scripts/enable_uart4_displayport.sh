#!/usr/bin/env bash
# Включает выделенный UART4 Raspberry Pi 5 для Betaflight MSP DisplayPort.
# Скрипт не перезагружает Raspberry сам: после проверки это решает владелец.

set -Eeuo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "[DronT16] Запусти с sudo: sudo ./scripts/enable_uart4_displayport.sh" >&2
    exit 2
fi

CONFIG_FILE="/boot/firmware/config.txt"
if [[ ! -f "${CONFIG_FILE}" ]]; then
    echo "[DronT16] ОШИБКА: не найден ${CONFIG_FILE}" >&2
    exit 1
fi

BACKUP_DIR="/home/oleg/DronT16_backups"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP_FILE="${BACKUP_DIR}/config.txt.before-uart4-displayport.${TIMESTAMP}.bak"
mkdir -p "${BACKUP_DIR}"
cp -p "${CONFIG_FILE}" "${BACKUP_FILE}"

if grep -qE '^dtoverlay=uart4-pi5([,[:space:]]|$)' "${CONFIG_FILE}"; then
    echo "[DronT16] UART4 уже включён; резервная копия: ${BACKUP_FILE}"
else
    printf '\n# DronT16: UART4 DisplayPort, GPIO12 pin32 TX / GPIO13 pin33 RX\ndtoverlay=uart4-pi5\n' >> "${CONFIG_FILE}"
    echo "[DronT16] Добавлен dtoverlay=uart4-pi5"
    echo "[DronT16] Резервная копия: ${BACKUP_FILE}"
fi

echo "[DronT16] Теперь выполни отдельно: sudo reboot"
