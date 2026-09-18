#!/usr/bin/env bash
# Генерирует медленные импульсы на физическом pin 8 или pin 10 для осциллографа.

set -Eeuo pipefail

# Выбираем физический контакт по аргументу пользователя.
case "${1:-}" in
    pin8)
        GPIO=14
        PIN_NAME="pin 8 / GPIO14 / TXD0"
        ;;
    pin10)
        GPIO=15
        PIN_NAME="pin 10 / GPIO15 / RXD0"
        ;;
    *)
        printf 'Использование: %s pin8|pin10\n' "$0" >&2
        exit 2
        ;;
esac

# После остановки возвращаем контакт в альтернативную функцию UART.
restore_uart() {
    pinctrl set "${GPIO}" a4 >/dev/null 2>&1 || true
    printf '\n[OSC] %s возвращён в функцию UART\n' "${PIN_NAME}"
}
trap restore_uart EXIT INT TERM

printf '[OSC] Импульсы на %s\n' "${PIN_NAME}"
printf '[OSC] Частота около 1 Гц, уровень 3,3 В\n'
printf '[OSC] Ctrl+C — остановить и вернуть UART\n'

# Переводим выбранный контакт в цифровой выход и начинаем прямоугольный сигнал.
pinctrl set "${GPIO}" op dl
while :; do
    pinctrl set "${GPIO}" dh
    sleep 0.5
    pinctrl set "${GPIO}" dl
    sleep 0.5
done
