#!/usr/bin/env bash
# Слушает фронты на физических pin 8 и pin 10 через GPIO-периферию.

set -Eeuo pipefail

# Номера GPIO соответствуют физическим контактам Raspberry Pi 5.
GPIO_TX=14
GPIO_RX=15

# После остановки возвращаем оба контакта в функцию UART0.
restore_uart() {
    pinctrl set "${GPIO_TX}" a4 >/dev/null 2>&1 || true
    pinctrl set "${GPIO_RX}" a4 >/dev/null 2>&1 || true
    printf '\n[GPIO WATCH] pin 8 и pin 10 возвращены в UART0\n'
}
trap restore_uart EXIT INT TERM

printf '[GPIO WATCH] Слушаю pin 8/GPIO14 и pin 10/GPIO15\n'
printf '[GPIO WATCH] Режим: rising + falling, pull-up, Ctrl+C — выход\n'
printf '[GPIO WATCH] UART временно отключён на этих двух пинах\n'

# Переводим контакты в цифровые входы с подтяжкой вверх.
pinctrl set "${GPIO_TX}" ip pu
pinctrl set "${GPIO_RX}" ip pu

# gpiomon печатает каждое изменение уровня и номер линии.
gpiomon --chip gpiochip0 --edges both --format '[GPIO] line=%o edge=%E time=%S' 14 15
