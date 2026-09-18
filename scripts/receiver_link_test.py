#!/usr/bin/env python3
"""Модуль №1: проверка связи Raspberry с CRSF-приёмником."""

from __future__ import annotations

import sys
import time
import tomllib
from pathlib import Path

# Добавляем корень проекта, чтобы запуск из каталога scripts находил пакет src.
PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from src.receiver.crsf import CrsfReceiver


def load_config() -> dict[str, int | str]:
    """Загружает параметры UART и канала из receiver.toml."""
    config_path = PROJECT_DIR / "config/receiver.toml"
    with config_path.open("rb") as config_file:
        return tomllib.load(config_file)["receiver"]


def main() -> int:
    """Показывает состояние CRSF и значение выбранного AUX-канала."""
    config = load_config()
    port = str(config["serial_port"])
    baudrate = int(config["baudrate"])
    timeout_s = int(config["link_timeout_ms"]) / 1000.0
    channel_index = int(config["mode_channel"]) - 1
    report_period_s = int(config["report_period_ms"]) / 1000.0
    if not 0 <= channel_index < 16:
        raise ValueError("mode_channel должен быть от 1 до 16")

    print(f"[RX TEST] UART: {port}, CRSF {baudrate} бод, канал CH{channel_index + 1}", flush=True)
    print("[RX TEST] Только чтение. Передача в полётник отключена. Ctrl+C — выход", flush=True)
    buffer = bytearray()
    last_frame_time = 0.0
    last_report = 0.0
    frame_count = 0
    try:
        with CrsfReceiver(port, baudrate) as receiver:
            while True:
                for frame in receiver.read_frames(buffer):
                    last_frame_time = frame.received_at
                    frame_count += 1
                    channel_value = frame.channels[channel_index]
                    if channel_value < 700:
                        position = "LOW"
                    elif channel_value > 1400:
                        position = "HIGH" if channel_value > 1700 else "MID"
                    else:
                        position = "MID"
                    print(f"[RX] LINK OK | CH{channel_index + 1}={channel_value} | {position}", flush=True)
                now = time.monotonic()
                if now - last_report >= report_period_s:
                    link = "OK" if last_frame_time and now - last_frame_time <= timeout_s else "LOST"
                    print(f"[RX] {link} | frames={frame_count} | bytes={receiver.bytes_received}", flush=True)
                    last_report = now
    except KeyboardInterrupt:
        print("[RX TEST] Остановлен", flush=True)
        return 0
    except (OSError, ValueError, KeyError, tomllib.TOMLDecodeError) as error:
        print(f"[RX TEST] Ошибка: {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
