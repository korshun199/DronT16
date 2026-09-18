#!/usr/bin/env python3
"""Фоновый диагностический слушатель сырого UART CRSF."""

from __future__ import annotations

import sys
import time
import tomllib
from pathlib import Path

# Добавляем корень проекта для запуска скрипта непосредственно из каталога scripts.
PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from src.receiver.crsf import CrsfReceiver


def main() -> int:
    """Слушает UART и каждую секунду показывает сырые байты и кадры CRSF."""
    # Загружаем те же параметры, что и основной тест приёмника.
    with (PROJECT_DIR / "config/receiver.toml").open("rb") as config_file:
        settings = tomllib.load(config_file)["receiver"]

    serial_port = str(settings["serial_port"])
    baudrate = int(settings["baudrate"])
    buffer = bytearray()
    frames = 0
    last_report = time.monotonic()

    print(f"[RX WATCH] Слушаю {serial_port}, CRSF {baudrate} бод", flush=True)
    print("[RX WATCH] Только чтение; данные в полётник не передаются", flush=True)

    try:
        with CrsfReceiver(serial_port, baudrate) as receiver:
            while True:
                parsed_frames = receiver.read_frames(buffer)
                frames += len(parsed_frames)
                now = time.monotonic()
                if now - last_report >= 1.0:
                    print(
                        f"[RX WATCH] bytes={receiver.bytes_received} "
                        f"frames={frames} buffer={len(buffer)}",
                        flush=True,
                    )
                    last_report = now
    except KeyboardInterrupt:
        print("[RX WATCH] Остановлен", flush=True)
        return 0
    except (OSError, ValueError, KeyError, tomllib.TOMLDecodeError) as error:
        print(f"[RX WATCH] Ошибка: {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
