#!/usr/bin/env python3
"""Цветная диагностика кадров, подготовленных для передачи в Betaflight."""

from __future__ import annotations

import sys
import time
import tomllib
from pathlib import Path

# Добавляем корень проекта, чтобы скрипт запускался из каталога scripts.
PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from src.receiver.crsf import CRSF_RC_CHANNELS_PACKED, CrsfReceiver, unpack_channels
from src.receiver.mode import ModeThresholds, ReceiverModeDecoder, mode_label


# ANSI-цвета для читаемого терминального отчёта.
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
CYAN = "\033[36m"
RESET = "\033[0m"


def color_mode(mode_name: str, label: str) -> str:
    """Окрашивает режим по смыслу команды."""
    color = {
        "DIRECT": GREEN,
        "CAPTURE": YELLOW,
        "CANCEL": RED,
    }.get(mode_name, CYAN)
    return f"{color}{mode_name} ({label}){RESET}"


def load_config() -> tuple[dict[str, int | str], dict[str, int | str]]:
    """Загружает настройки UART и порогов режима."""
    with (PROJECT_DIR / "config/receiver.toml").open("rb") as config_file:
        config = tomllib.load(config_file)
    return config["receiver"], config["mode"]


def main() -> int:
    """Показывает кадры без передачи данных в полётник."""
    receiver_config, mode_config = load_config()
    serial_port = str(receiver_config["serial_port"])
    baudrate = int(receiver_config["baudrate"])
    channel_index = int(receiver_config["mode_channel"]) - 1
    report_period_s = int(receiver_config["report_period_ms"]) / 1000.0
    decoder = ReceiverModeDecoder(
        ModeThresholds(
            low_max=int(mode_config["low_max"]),
            high_min=int(mode_config["high_min"]),
            debounce_frames=int(mode_config["debounce_frames"]),
        )
    )
    buffer = bytearray()
    frame_count = 0
    forwarded_count = 0
    last_report = 0.0
    last_channels: tuple[int, ...] | None = None
    last_mode = decoder.current_mode

    print(f"{CYAN}[BRIDGE DIAG] UART RX: {serial_port}, CRSF {baudrate} бод{RESET}")
    print(f"{GREEN}[BRIDGE DIAG] Сухой прогон: в FC ничего не передаётся{RESET}")
    print(f"{CYAN}[BRIDGE DIAG] Ctrl+C — выход{RESET}")

    try:
        # write_enabled=False гарантирует отсутствие записи в полётник.
        with CrsfReceiver(serial_port, baudrate) as receiver:
            while True:
                for raw_frame in receiver.read_raw_frames(buffer):
                    frame_count += 1
                    if len(raw_frame) < 4 or raw_frame[2] != CRSF_RC_CHANNELS_PACKED:
                        continue
                    channels = unpack_channels(raw_frame[3:-1])
                    last_channels = channels
                    forwarded_count += 1
                    last_mode, mode_changed = decoder.update(channels[channel_index])
                    if mode_changed:
                        print(
                            f"{CYAN}[MODE] CH{channel_index + 1}="
                            f"{channels[channel_index]} → "
                            f"{color_mode(last_mode.value, mode_label(last_mode))}{RESET}",
                            flush=True,
                        )
                now = time.monotonic()
                if now - last_report >= report_period_s:
                    if last_channels is None:
                        print(f"{RED}[FC OUT] Нет корректных RC-кадров{RESET}", flush=True)
                    else:
                        channel_text = " ".join(
                            f"CH{index + 1}={value}"
                            for index, value in enumerate(last_channels)
                        )
                        print(
                            f"{CYAN}[FC OUT DRY-RUN] frame={frame_count} "
                            f"len=26 | {color_mode(last_mode.value, mode_label(last_mode))} "
                            f"| {channel_text} | would_forward={forwarded_count}{RESET}",
                            flush=True,
                        )
                    last_report = now
    except KeyboardInterrupt:
        print(f"\n{CYAN}[BRIDGE DIAG] Остановлен, FC не получал данных{RESET}")
        return 0
    except (OSError, ValueError, KeyError, tomllib.TOMLDecodeError) as error:
        print(f"{RED}[BRIDGE DIAG] Ошибка: {error}{RESET}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
