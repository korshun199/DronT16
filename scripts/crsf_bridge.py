#!/usr/bin/env python3
"""Безопасный прозрачный мост RC-кадров CRSF на UART полётника."""

from __future__ import annotations

import sys
import time
import tomllib
from pathlib import Path

# Добавляем корень проекта для запуска скрипта из каталога scripts.
PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from src.receiver.crsf import CRSF_RC_CHANNELS_PACKED, CrsfReceiver, unpack_channels
from src.receiver.mode import ModeThresholds, ReceiverModeDecoder


def load_config() -> dict[str, int | str]:
    """Загружает параметры моста из bridge.toml."""
    config_path = PROJECT_DIR / "config/bridge.toml"
    with config_path.open("rb") as config_file:
        return tomllib.load(config_file)["bridge"]


def main() -> int:
    """Передаёт только CRC-проверенные RC-кадры без изменения каналов."""
    config = load_config()
    serial_port = str(config["serial_port"])
    baudrate = int(config["baudrate"])
    frame_type = int(config["forward_frame_type"])
    timeout_s = int(config["link_timeout_ms"]) / 1000.0
    report_period_s = int(config["report_period_ms"]) / 1000.0
    command_file = Path(str(config.get("control_file", "/tmp/dront16_command")))
    simulator_command_file = Path(str(config.get("simulator_control_file", "/tmp/simulator_filesafe_command")))
    receiver_config_path = PROJECT_DIR / "config/receiver.toml"
    with receiver_config_path.open("rb") as config_file:
        receiver_config = tomllib.load(config_file)
    mode_config = receiver_config["mode"]
    mode_channel = int(receiver_config["receiver"]["mode_channel"]) - 1
    mode_decoder = ReceiverModeDecoder(
        ModeThresholds(
            int(mode_config["low_max"]),
            int(mode_config["high_min"]),
            int(mode_config["debounce_frames"]),
        )
    )
    if not 0 <= mode_channel < 16:
        raise ValueError("mode_channel должен быть от 1 до 16")
    simulator_config_path = PROJECT_DIR / "config/simulator_filesafe.toml"
    with simulator_config_path.open("rb") as config_file:
        simulator_config = tomllib.load(config_file)["control"]
    loss_channel = int(simulator_config["loss_channel"]) - 1
    link_ok_max = int(simulator_config["link_ok_max"])
    link_lost_min = int(simulator_config["link_lost_min"])
    disarm_channel = int(simulator_config["disarm_channel"]) - 1
    disarm_active_max = int(simulator_config["disarm_active_max"])
    if (
        not 0 <= loss_channel < 16
        or not 0 <= disarm_channel < 16
        or not 0 <= link_ok_max < link_lost_min <= 2047
        or not 0 <= disarm_active_max < 2047
    ):
        raise ValueError("Неверные параметры тумблера simulator_filesafe")
    last_link_lost: bool | None = None
    last_disarmed: bool | None = None
    buffer = bytearray()
    last_rc_time = 0.0
    last_report = 0.0
    received_frames = 0
    forwarded_frames = 0

    print(f"[CRSF BRIDGE] UART: {serial_port}, скорость: {baudrate}", flush=True)
    print("[CRSF BRIDGE] RX приёмника -> TX полётника, кадры не изменяются", flush=True)
    print("[CRSF BRIDGE] Ctrl+C — остановка передачи", flush=True)

    try:
        with CrsfReceiver(serial_port, baudrate, write_enabled=True) as bridge_uart:
            while True:
                for frame in bridge_uart.read_raw_frames(buffer):
                    received_frames += 1
                    if frame[2] != frame_type or frame_type != CRSF_RC_CHANNELS_PACKED:
                        continue
                    # Сначала безусловно передаём исходный RC-кадр в полётник.
                    # Обработка CH6 не должна влиять на прозрачный мост.
                    bridge_uart.write_frame(frame)
                    forwarded_frames += 1
                    last_rc_time = time.monotonic()
                    # Передаём режим видеомодулю через тот же файл, что и SSH-пульт.
                    channels = unpack_channels(frame[3:-1])
                    selected_mode, changed = mode_decoder.update(channels[mode_channel])
                    if changed:
                        mode_command = {"DIRECT": "1", "CAPTURE": "2", "FOLLOW": "3"}[selected_mode.value]
                        command_file.write_text(mode_command, encoding="ascii")
                        print(
                            f"[RX MODE] CH{mode_channel + 1}={channels[mode_channel]} "
                            f"-> {mode_command} {selected_mode.value}",
                            flush=True,
                        )
                    # Второй AUX-тумблер только моделирует состояние связи.
                    loss_value = channels[loss_channel]
                    link_lost = loss_value >= link_lost_min
                    if last_link_lost is None or link_lost != last_link_lost:
                        simulator_command_file.write_text(
                            "LINK_LOST" if link_lost else "LINK_OK", encoding="ascii"
                        )
                        print(
                            f"[SIM LINK] CH{loss_channel + 1}={loss_value} -> "
                            f"{'LINK_LOST' if link_lost else 'LINK_OK'}",
                            flush=True,
                        )
                        last_link_lost = link_lost
                    # DISARM имеет приоритет над failsafe и защёлкивается до перезапуска.
                    disarm_value = channels[disarm_channel]
                    disarmed = disarm_value <= disarm_active_max
                    if disarmed != last_disarmed:
                        print(
                            f"[SIM ARM] CH{disarm_channel + 1}={disarm_value} -> "
                            f"{'DISARM' if disarmed else 'ARM'}",
                            flush=True,
                        )
                    if disarmed and not last_disarmed:
                        simulator_command_file.write_text("DISARM", encoding="ascii")
                        print(
                            f"[SIM DISARM] CH{disarm_channel + 1}={disarm_value} -> DISARM; "
                            "виртуальные моторы OFF",
                            flush=True,
                        )
                    last_disarmed = disarmed
                now = time.monotonic()
                if now - last_report >= report_period_s:
                    link = "OK" if last_rc_time and now - last_rc_time <= timeout_s else "LOST"
                    print(
                        f"[CRSF BRIDGE] {link} | received={received_frames} "
                        f"forwarded={forwarded_frames} | bytes={bridge_uart.bytes_received}",
                        flush=True,
                    )
                    last_report = now
    except KeyboardInterrupt:
        print("[CRSF BRIDGE] Остановлен, передача прекращена", flush=True)
        return 0
    except (OSError, ValueError, KeyError, tomllib.TOMLDecodeError) as error:
        print(f"[CRSF BRIDGE] Ошибка: {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
