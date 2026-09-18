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
from src.receiver.mode import ModeThresholds, ReceiverModeDecoder, mode_label


def load_config() -> dict[str, dict[str, int | str]]:
    """Загружает параметры UART, канала и декодера режима."""
    config_path = PROJECT_DIR / "config/receiver.toml"
    with config_path.open("rb") as config_file:
        return tomllib.load(config_file)


def main() -> int:
    """Показывает состояние CRSF и значение выбранного AUX-канала."""
    config = load_config()
    receiver_config = config["receiver"]
    mode_config = config["mode"]
    port = str(receiver_config["serial_port"])
    baudrate = int(receiver_config["baudrate"])
    timeout_s = int(receiver_config["link_timeout_ms"]) / 1000.0
    channel_index = int(receiver_config["mode_channel"]) - 1
    report_period_s = int(receiver_config["report_period_ms"]) / 1000.0
    decoder = ReceiverModeDecoder(
        ModeThresholds(
            low_max=int(mode_config["low_max"]),
            high_min=int(mode_config["high_min"]),
            debounce_frames=int(mode_config["debounce_frames"]),
        )
    )
    if not 0 <= channel_index < 16:
        raise ValueError("mode_channel должен быть от 1 до 16")

    print(f"[RX TEST] UART: {port}, CRSF {baudrate} бод, канал CH{channel_index + 1}", flush=True)
    print("[RX TEST] Только чтение. Передача в полётник отключена. Ctrl+C — выход", flush=True)
    buffer = bytearray()
    last_frame_time = 0.0
    last_report = 0.0
    frame_count = 0
    last_channel_value: int | None = None
    last_mode = decoder.current_mode
    try:
        with CrsfReceiver(port, baudrate) as receiver:
            while True:
                for frame in receiver.read_frames(buffer):
                    last_frame_time = frame.received_at
                    frame_count += 1
                    channel_value = frame.channels[channel_index]
                    last_channel_value = channel_value
                    mode, changed = decoder.update(channel_value)
                    last_mode = mode
                    if changed:
                        print(
                            f"[MODE] CH{channel_index + 1}={channel_value} "
                            f"→ {mode.value} ({mode_label(mode)})",
                            flush=True,
                        )
                now = time.monotonic()
                if now - last_report >= report_period_s:
                    link = "OK" if last_frame_time and now - last_frame_time <= timeout_s else "LOST"
                    if last_channel_value is None:
                        channel_info = "CH без кадра"
                    else:
                        channel_info = (
                            f"CH{channel_index + 1}={last_channel_value} "
                            f"{last_mode.value} ({mode_label(last_mode)})"
                        )
                    print(
                        f"[RX] {link} | {channel_info} | frames={frame_count} "
                        f"| bytes={receiver.bytes_received}",
                        flush=True,
                    )
                    last_report = now
    except KeyboardInterrupt:
        print("[RX TEST] Остановлен", flush=True)
        return 0
    except (OSError, ValueError, KeyError, tomllib.TOMLDecodeError) as error:
        print(f"[RX TEST] Ошибка: {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
