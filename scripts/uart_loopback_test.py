#!/usr/bin/env python3
"""Безопасная проверка UART Raspberry перемычкой TXD0 -> RXD0."""

from __future__ import annotations

import fcntl
import argparse
import os
import select
import struct
import sys
import termios
import time
import tomllib
from pathlib import Path

# Добавляем корень проекта для запуска скрипта из каталога scripts.
PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))


def configure_uart(file_descriptor: int, baudrate: int) -> None:
    """Настраивает UART в режиме 8N1 на нестандартную скорость CRSF."""
    # Переводим дескриптор в обычный сырой режим без эха и управляющих кодов.
    settings = termios.tcgetattr(file_descriptor)
    settings[0] = 0
    settings[1] = 0
    settings[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    settings[3] = 0
    settings[6][termios.VMIN] = 0
    settings[6][termios.VTIME] = 0
    termios.tcsetattr(file_descriptor, termios.TCSANOW, settings)

    # Linux termios2 позволяет выставить 420000 бод через BOTHER.
    tcgets2 = 0x802C542A
    tcsets2 = 0x402C542B
    bother = 0x1000
    cbaud = 0x100F
    termios2 = bytearray(44)
    fcntl.ioctl(file_descriptor, tcgets2, termios2, True)
    flags = struct.unpack_from("I", termios2, 8)[0]
    flags = (flags & ~cbaud) | bother
    struct.pack_into("I", termios2, 8, flags)
    struct.pack_into("I", termios2, 36, baudrate)
    struct.pack_into("I", termios2, 40, baudrate)
    fcntl.ioctl(file_descriptor, tcsets2, termios2)


def main() -> int:
    """Передаёт тестовый пакет и проверяет его возврат по loopback."""
    # Загружаем UART из общего конфигурационного файла проекта.
    with (PROJECT_DIR / "config/receiver.toml").open("rb") as config_file:
        settings = tomllib.load(config_file)["receiver"]
    serial_port = str(settings["serial_port"])
    # Позволяем отдельно проверить UART на стандартной скорости 115200 бод.
    arguments = argparse.ArgumentParser(description="Проверка UART TXD0 -> RXD0")
    arguments.add_argument("--port", default=serial_port)
    arguments.add_argument("--baudrate", type=int, default=int(settings["baudrate"]))
    parsed_arguments = arguments.parse_args()
    serial_port = str(parsed_arguments.port)
    baudrate = parsed_arguments.baudrate

    # Посылка содержит различимые байты и повторяется, чтобы контакт можно
    # было уверенно удерживать несколько секунд; это не команда CRSF.
    test_payload = bytes.fromhex("55 A3 5A C3 96 69 3C F0 0F")
    test_stream = test_payload * 128
    received = bytearray()

    print(f"[UART LOOPBACK] Порт: {serial_port}, скорость: {baudrate}")
    print("[UART LOOPBACK] Ожидается перемычка pin 8 TXD0 -> pin 10 RXD0")
    print("[UART LOOPBACK] Приёмник к pin 10 временно не подключён")

    try:
        file_descriptor = os.open(serial_port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as error:
        print(f"[UART LOOPBACK] Ошибка открытия UART: {error}", file=sys.stderr)
        return 1

    try:
        configure_uart(file_descriptor, baudrate)
        termios.tcflush(file_descriptor, termios.TCIOFLUSH)
        written = 0
        while written < len(test_stream):
            written += os.write(file_descriptor, test_stream[written:])
        # Ждём фактической отправки байтов из TX FIFO в линию перед чтением RX.
        termios.tcdrain(file_descriptor)
        time.sleep(0.02)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and len(received) < len(test_stream):
            ready, _, _ = select.select([file_descriptor], [], [], 0.05)
            if ready:
                received.extend(os.read(file_descriptor, 4096))
    finally:
        os.close(file_descriptor)

    print(f"[UART LOOPBACK] Отправлено байт: {len(test_stream)}")
    print(f"[UART LOOPBACK] Получено байт:  {len(received)}")
    if bytes(received) == test_stream:
        print("[UART LOOPBACK] OK: UART и распиновка pin 8/pin 10 исправны")
        return 0
    print("[UART LOOPBACK] FAIL: тестовая последовательность не вернулась полностью")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
