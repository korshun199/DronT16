#!/usr/bin/env python3
"""Текстовый симулятор failsafe без доступа к реальным портам и полётнику."""

from __future__ import annotations

import sys
import time
import tomllib
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

# Добавляем корень проекта для общего журнала и единых модулей.
if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.diagnostics.journal import Color, EventJournal

PROJECT_DIR = Path(__file__).resolve().parents[1]


class State(str, Enum):
    """Состояния виртуального failsafe-сценария."""

    LINK_OK = "LINK_OK"
    LINK_LOST = "LINK_LOST"
    TURN_180 = "TURN_180"
    RETURN = "RETURN"
    LINK_RESTORED = "LINK_RESTORED"
    DISARMED = "DISARMED"


@dataclass
class Packet:
    """Последний пакет, полученный от виртуального полётника."""

    number: int
    heading_deg: float
    altitude_m: float
    speed_m_s: float


def load_config() -> dict[str, object]:
    """Загружает и возвращает TOML-конфигурацию симулятора."""
    path = PROJECT_DIR / "config/simulator_filesafe.toml"
    with path.open("rb") as file:
        return tomllib.load(file)


def read_control(path: Path) -> str | None:
    """Читает одно событие тумблера и удаляет его после обработки."""
    try:
        command = path.read_text(encoding="ascii").strip()
        path.unlink()
        return command
    except FileNotFoundError:
        return None


def main() -> int:
    """Запускает сценарий потери и восстановления связи."""
    config = load_config()
    simulator = config["simulator"]
    fc = config["flight_controller"]
    obj = config["object"]
    failsafe = config["failsafe"]
    control = config["control"]
    start = time.monotonic()
    journal = EventJournal(PROJECT_DIR / str(simulator["log_file"]), "SIM", start)
    duration = float(simulator["duration_s"])
    step = float(simulator["step_ms"]) / 1000.0
    packet_period = float(simulator["packet_period_ms"]) / 1000.0
    control_file = PROJECT_DIR / str(control["command_file"])
    turn_total = float(failsafe["turn_degrees"])
    turn_rate = float(failsafe["turn_rate_deg_s"])
    state = State.LINK_OK
    packet_number = 0
    last_packet = Packet(0, float(fc["heading_deg"]), float(fc["altitude_m"]), float(fc["speed_m_s"]))
    last_packet_log = -packet_period
    last_control = None
    motors_armed = True
    turn_done = 0.0
    journal.write("RPI", "START: симулятор запущен; реальные порты и FC отключены", Color.CYAN)
    journal.write(
        "RPI",
        f"duration={duration:.1f}s link_control=CH{int(control['loss_channel'])}/AUX3 "
        f"disarm=CH{int(control['disarm_channel'])} SWITCH_ONLY",
        Color.CYAN,
    )
    journal.write("RPI", f"PORT: CRSF={fc['crsf_port']} MSP={fc['msp_port']} baud={fc['baudrate']} SIMULATED", Color.BLUE)
    journal.write("RPI", f"MEMORY: object={obj['object_id']} box=({obj['x']:.0f},{obj['y']:.0f},{obj['width']:.0f},{obj['height']:.0f}) confidence={obj['confidence']:.2f}", Color.MAGENTA)
    try:
        while True:
            elapsed = time.monotonic() - start
            if duration > 0 and elapsed >= duration:
                break
            control_command = read_control(control_file)
            if control_command is not None:
                last_control = control_command
                journal.write("PILOT", f"SWITCH: получено событие тумблера: {control_command}", Color.CYAN)
            if control_command == "DISARM" and motors_armed:
                motors_armed = False
                state = State.DISARMED
                journal.write("RPI", "DECISION: получен DISARM; немедленно отключаю моторы", Color.RED)
                journal.write("RPI", "COMMAND TO FC SIMULATED: DISARM; MOTORS=OFF", Color.RED)
                journal.write("RPI", "STATE: ANY_STATE -> DISARMED (защёлкнуто до перезапуска)", Color.RED)
            if state is State.DISARMED:
                time.sleep(step)
                continue
            loss_requested = last_control == "LINK_LOST"
            if state is State.LINK_OK and loss_requested:
                state = State.LINK_LOST
                journal.write("RPI", "DECISION: таймаут CRSF; связь потеряна", Color.RED)
                journal.write("RPI", f"MEMORY: сохраняю последний подтверждённый пакет FC #{last_packet.number}", Color.MAGENTA)
                journal.write("RPI", "STATE: LINK_LOST -> TURN_180", Color.RED)
                state = State.TURN_180
            elif state is not State.LINK_OK and last_control == "LINK_OK":
                journal.write("PILOT", "SWITCH: тумблер вернул связь; отменяю failsafe", Color.GREEN)
                journal.write("RPI", f"STATE: {state.value} -> LINK_RESTORED -> LINK_OK", Color.GREEN)
                state = State.LINK_OK
                turn_done = 0.0
                last_control = None
            if state is State.TURN_180:
                turn_done = min(turn_total, turn_done + turn_rate * step)
                journal.write("RPI", f"COMMAND TO FC SIMULATED: YAW +{turn_rate * step:.1f} deg; progress={turn_done:.1f}/{turn_total:.1f}", Color.RED)
                if turn_done >= turn_total:
                    state = State.RETURN
                    journal.write("RPI", "DECISION: разворот завершён; начинаю возврат по последним данным", Color.YELLOW)
                    journal.write("RPI", "STATE: TURN_180 -> RETURN", Color.YELLOW)
            elif state is State.RETURN:
                journal.write("RPI", f"COMMAND TO FC SIMULATED: RETURN heading={last_packet.heading_deg:.1f} altitude={last_packet.altitude_m:.1f}", Color.YELLOW)
            if state is State.LINK_OK and elapsed - last_packet_log >= packet_period:
                packet_number += 1
                last_packet = Packet(packet_number, last_packet.heading_deg, last_packet.altitude_m, last_packet.speed_m_s)
                journal.write("FC", f"RX: packet=#{packet_number} heading={last_packet.heading_deg:.1f} altitude={last_packet.altitude_m:.1f} speed={last_packet.speed_m_s:.1f} CRC=OK", Color.GREEN)
                journal.write("RPI", "PORT: CRSF RX=OK MSP TX=READY", Color.BLUE)
                journal.write("RPI", f"MEMORY: TARGET state=TRACKING box=({obj['x']:.0f},{obj['y']:.0f},{obj['width']:.0f},{obj['height']:.0f}) confidence={obj['confidence']:.2f}", Color.MAGENTA)
                last_packet_log = elapsed
            time.sleep(step)
    except KeyboardInterrupt:
        journal.write("RPI", "STOP: симулятор остановлен оператором", Color.YELLOW)
        return 0
    finally:
        journal.write("RPI", "STOP: сценарий завершён; реальные команды не отправлялись", Color.CYAN)
        journal.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
