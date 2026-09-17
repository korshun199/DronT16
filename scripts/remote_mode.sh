#!/usr/bin/env bash

set -Eeuo pipefail

# Адрес Raspberry Pi; при переносе меняется только эта строка.
REMOTE_HOST="oleg@192.168.20.125"
# Рабочий каталог DronT16 на Raspberry Pi.
REMOTE_PROJECT="/home/oleg/DronT16"
# Файл, через который Raspberry получает одну команду от SSH-пульта.
CONTROL_FILE="/tmp/dront16_command"

# Цвета терминала для понятного отображения состояния пульта.
GREEN="\033[32m"
RED="\033[31m"
YELLOW="\033[33m"
RESET="\033[0m"

send_command() {
    # Передаёт только проверенную цифру атомарной заменой командного файла.
    local command="$1"
    local temporary_file="${CONTROL_FILE}.tmp"
    if ssh -o ConnectTimeout=5 "$REMOTE_HOST" \
        "printf '%s\\n' '$command' > '$temporary_file' && mv -f '$temporary_file' '$CONTROL_FILE'"; then
        echo -e "${GREEN}Команда ${command} передана Raspberry${RESET}"
    else
        echo -e "${RED}Ошибка передачи команды ${command}${RESET}" >&2
    fi
}

echo -e "${YELLOW}DronT16 SSH-пульт: 1=Прямой, 2=Захватить, 3=Следить, 4=Отбой, q=Выход${RESET}"
echo "Вводи одну цифру и нажимай Enter."

while IFS= read -r command; do
    case "$command" in
        1|2|3|4)
            send_command "$command"
            ;;
        q|Q)
            echo "SSH-пульт завершён."
            break
            ;;
        "")
            ;;
        *)
            echo -e "${RED}Допустимы только 1, 2, 3, 4 или q${RESET}"
            ;;
    esac
done
