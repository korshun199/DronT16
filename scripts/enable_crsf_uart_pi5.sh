#!/usr/bin/env bash
# Включает UART0 Raspberry Pi 5 на GPIO14/15 для входа CRSF-приёмника.

set -Eeuo pipefail

# Файлы загрузочной конфигурации Raspberry Pi.
CONFIG_FILE="/boot/firmware/config.txt"
CMDLINE_FILE="/boot/firmware/cmdline.txt"
# Метка резервной копии с точностью до секунды.
BACKUP_TAG="$(date +%Y%m%d-%H%M%S)"

if [[ "${EUID}" -ne 0 ]]; then
    echo "Ошибка: запусти скрипт через sudo." >&2
    exit 2
fi

if [[ ! -f "${CONFIG_FILE}" || ! -f "${CMDLINE_FILE}" ]]; then
    echo "Ошибка: не найдены загрузочные файлы Raspberry Pi." >&2
    exit 1
fi

# Сохраняем исходные файлы до любых изменений.
cp -a "${CONFIG_FILE}" "${CONFIG_FILE}.bak-${BACKUP_TAG}"
cp -a "${CMDLINE_FILE}" "${CMDLINE_FILE}.bak-${BACKUP_TAG}"
echo "Резервные копии сохранены с меткой ${BACKUP_TAG}."

# Включаем UART0 Pi 5 на физических GPIO14/15.
if ! grep -Eq '^[[:space:]]*dtoverlay=uart0-pi5([,[:space:]]|$)' "${CONFIG_FILE}"; then
    printf '\n# UART0 Raspberry Pi 5 на GPIO14/15 для CRSF DronT16\ndtoverlay=uart0-pi5\n' >> "${CONFIG_FILE}"
    echo "Добавлен dtoverlay=uart0-pi5."
else
    echo "dtoverlay=uart0-pi5 уже присутствует."
fi

# Убираем консоль ядра с UART, оставляя консоль HDMI/терминала tty1.
temporary_cmdline="${CMDLINE_FILE}.tmp"
sed -E 's/(^| )console=ttyAMA10,[^ ]+//g; s/  +/ /g; s/^ //; s/ $//' \
    "${CMDLINE_FILE}" > "${temporary_cmdline}"
chmod --reference="${CMDLINE_FILE}" "${temporary_cmdline}"
chown --reference="${CMDLINE_FILE}" "${temporary_cmdline}"
mv -f "${temporary_cmdline}" "${CMDLINE_FILE}"
echo "Консоль ttyAMA10 удалена из cmdline.txt."

# Отключаем getty, если он был создан для старого UART-консольного режима.
systemctl disable --now serial-getty@ttyAMA10.service 2>/dev/null || true
echo "Настройка завершена. Требуется перезагрузка Raspberry Pi."
echo "После перезагрузки проверь: pinctrl get 14,15"
