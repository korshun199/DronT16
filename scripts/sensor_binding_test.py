#!/usr/bin/env python3
"""Локальная проверка привязки датчиков к модели DronT16.

Скрипт формирует только синтетические ответы MSPv1 и прогоняет их через
реальный парсер проекта. UART, камера, CRSF-мост и команды полётнику не
открываются. Это позволяет проверить контракт датчиков на ноутбуке до
подключения Raspberry Pi.
"""

from __future__ import annotations

import struct
import sys
import time
import tomllib
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from src.diagnostics.journal import Color, EventJournal
from src.protocols.betaflight_msp_link import (
    MSP_ALTITUDE,
    MSP_ATTITUDE,
    MSP_RAW_GPS,
    MSP_RAW_IMU,
    MspParser,
    msp_checksum,
)


def msp_response(command: int, payload: bytes) -> bytes:
    """Формирует синтетический ответ MSPv1 с корректной XOR-суммой."""
    size = len(payload)
    return b"$M>" + bytes((size, command)) + payload + bytes((msp_checksum(size, command, payload),))


def sensor_batch(cycle: int) -> bytes:
    """Возвращает один набор показаний барометра, IMU, магнитометра и GPS."""
    altitude_cm = 100 + cycle * 2
    vario_cm_s = 0
    attitude = struct.pack("<hhh", 0, 0, 900 + cycle * 2)
    altitude = struct.pack("<ih", altitude_cm, vario_cm_s)
    # ACC, GYRO, MAG: магнитное поле поворачивается в тестовом наборе.
    angle = cycle * 45
    if angle == 0:
        mag_x, mag_y = 1000, 0
    elif angle == 45:
        mag_x, mag_y = 707, 707
    else:
        mag_x, mag_y = 0, 1000
    raw_imu = struct.pack("<hhhhhhhhh", 0, 0, 1000, 0, 0, 0, mag_x, mag_y, -1200)
    gps = struct.pack(
        "<BBiihHHH",
        3,
        10,
        557_522_000,
        376_156_000,
        180,
        125,
        900,
        120,
    )
    return (
        msp_response(MSP_ATTITUDE, attitude)
        + msp_response(MSP_ALTITUDE, altitude)
        + msp_response(MSP_RAW_IMU, raw_imu)
        + msp_response(MSP_RAW_GPS, gps)
    )


def load_test_config() -> tuple[dict[str, object], dict[str, object]]:
    """Читает безопасные настройки теста и карту источников датчиков."""
    path = PROJECT_DIR / "config/dront16.toml"
    try:
        with path.open("rb") as config_file:
            config = tomllib.load(config_file)
    except (OSError, tomllib.TOMLDecodeError) as error:
        raise RuntimeError(f"Не удалось прочитать конфигурацию {path}: {error}") from error
    test_config = config.get("sensor_test")
    sensor_config = config.get("sensors")
    if not isinstance(test_config, dict) or not isinstance(sensor_config, dict):
        raise RuntimeError("В config/dront16.toml отсутствуют [sensor_test] или [sensors]")
    return test_config, sensor_config


def main() -> int:
    """Запускает локальную проверку MSP-парсера и журнала датчиков."""
    try:
        test_config, sensor_config = load_test_config()
        if not bool(test_config.get("enabled", False)):
            raise RuntimeError("sensor_test.enabled=false; локальный тест отключён")
        cycles = int(test_config.get("cycles", 3))
        period_s = int(test_config.get("period_ms", 100)) / 1000.0
        verbose = bool(test_config.get("verbose", False))
        log_path = PROJECT_DIR / str(test_config.get("log_file", "runs/sensor_binding_test.log"))
        if cycles <= 0 or period_s < 0:
            raise RuntimeError("sensor_test: cycles должно быть > 0, period_ms не может быть отрицательным")
        required_sources = {"barometer", "imu", "magnetometer", "compass", "gps", "front_camera"}
        missing_sources = sorted(required_sources.difference(sensor_config))
        if missing_sources:
            raise RuntimeError(f"В [sensors] отсутствуют источники: {', '.join(missing_sources)}")
    except (RuntimeError, ValueError, TypeError) as error:
        print(f"[SENSOR TEST] ERROR: {error}", file=sys.stderr)
        return 1

    log_path.parent.mkdir(parents=True, exist_ok=True)
    journal = EventJournal(log_path, "SENSOR_TEST")
    parser = MspParser()
    now = 0.0
    journal.begin_session()
    try:
        journal.write("RPI", "START: локальный тест привязки датчиков; реальные порты отключены", Color.CYAN)
        journal.write("RPI", "SAFETY: FC/CRSF/UART/камера не открываются, команды не отправляются", Color.GREEN)
        journal.write("RPI", "BINDING: " + "; ".join(f"{name}={source}" for name, source in sensor_config.items()), Color.BLUE)
        latest = None
        for cycle in range(cycles):
            now = cycle * period_s
            packet_stream = sensor_batch(cycle)
            # Имитируем дробление UART-потока: парсер должен собрать ответы из частей.
            split = max(1, len(packet_stream) // 3)
            for offset in range(0, len(packet_stream), split):
                samples = parser.feed(packet_stream[offset : offset + split], received_at=now)
                if samples:
                    latest = samples[-1]
            if latest is None or not latest.complete:
                raise RuntimeError(f"цикл {cycle + 1}: не собраны altitude и attitude")
            if not latest.gps_valid or not latest.gps_is_fresh(now, max_age_s=max(period_s, 0.001) + 0.01):
                raise RuntimeError(f"цикл {cycle + 1}: GPS невалиден или устарел")
            if latest.magnetic_heading_deg is None:
                raise RuntimeError(f"цикл {cycle + 1}: магнитометр не дал диагностический курс")
            text = (
                f"cycle={cycle + 1} altitude={latest.altitude_m:.2f}m "
                f"roll={latest.roll_deg:.1f}deg pitch={latest.pitch_deg:.1f}deg "
                f"mag=({latest.mag_x},{latest.mag_y},{latest.mag_z}) "
                f"mag_heading_raw={latest.magnetic_heading_deg:.1f}deg "
                f"gps=FIX{latest.gps_fix}/{latest.gps_satellites} "
                f"lat={latest.gps_latitude_deg:.7f} lon={latest.gps_longitude_deg:.7f} "
                f"speed={latest.gps_speed_m_s:.2f}m/s course={latest.gps_course_deg:.1f}deg"
            )
            journal.write("FC", f"SENSOR: {text}", Color.MAGENTA, console=verbose)
            if period_s:
                time.sleep(period_s)
        journal.write("RPI", f"PASS: {cycles} циклов MSP собраны, CRC и свежесть проверены", Color.GREEN)
        print(f"[SENSOR TEST] PASS: датчики привязаны, циклов={cycles}, лог={log_path}", flush=True)
        return 0
    except (RuntimeError, ValueError, struct.error) as error:
        journal.write("RPI", f"ERROR: {error}", Color.RED)
        print(f"[SENSOR TEST] ERROR: {error}", file=sys.stderr)
        return 1
    finally:
        journal.end_session()
        journal.close()


if __name__ == "__main__":
    raise SystemExit(main())
