"""Двунаправленный read-only MSP-канал Betaflight.

Модуль пока только читает состояние полётника. Команды MSP не отправляются:
управление остаётся в проверенном CRSF-мосте, а команды RC передаются через
Betaflight, который сохраняет PID и защиту моторов.
"""

from __future__ import annotations

import os
import select
import struct
import termios
import time
from dataclasses import dataclass


# Команды MSP v1, поддерживаемые Betaflight для первичного чтения датчиков.
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_RAW_IMU = 102


def msp_checksum(size: int, command: int, payload: bytes = b"") -> int:
    """Считает XOR-контрольную сумму MSP v1."""
    checksum = size ^ command
    for byte in payload:
        checksum ^= byte
    return checksum


def build_msp_request(command: int) -> bytes:
    """Формирует пустой запрос MSP v1 к полётному контроллеру."""
    if not 0 <= command <= 255:
        raise ValueError("Команда MSP должна находиться в диапазоне 0..255")
    return b"$M<" + bytes((0, command, command))


@dataclass(frozen=True)
class SensorSample:
    """Последний согласованный набор данных полётника."""

    altitude_m: float | None
    vario_m_s: float | None
    roll_deg: float | None
    pitch_deg: float | None
    yaw_deg: float | None
    received_at: float
    altitude_valid: bool
    attitude_valid: bool
    altitude_received_at: float | None = None
    attitude_received_at: float | None = None
    # Сырые оси магнитометра из MSP_RAW_IMU для диагностики вращения.
    mag_x: int | None = None
    mag_y: int | None = None
    mag_z: int | None = None
    mag_received_at: float | None = None

    @property
    def complete(self) -> bool:
        """Возвращает True, если высота и наклон доступны одновременно."""
        return self.altitude_valid and self.attitude_valid

    def is_fresh(self, now: float, max_age_s: float) -> bool:
        """Проверяет свежесть каждого обязательного типа датчиков отдельно."""
        if max_age_s < 0:
            return False
        if not self.complete or self.altitude_received_at is None or self.attitude_received_at is None:
            return False
        return (
            now - self.altitude_received_at <= max_age_s
            and now - self.attitude_received_at <= max_age_s
        )


class MspParser:
    """Разбирает ответы MSP v1 с проверкой заголовка и checksum."""

    def __init__(self) -> None:
        """Создаёт чистый буфер и пустое состояние датчиков."""
        self.buffer = bytearray()
        self.altitude_m: float | None = None
        self.vario_m_s: float | None = None
        self.roll_deg: float | None = None
        self.pitch_deg: float | None = None
        self.yaw_deg: float | None = None
        self.last_altitude_at: float | None = None
        self.last_attitude_at: float | None = None
        self.mag_x: int | None = None
        self.mag_y: int | None = None
        self.mag_z: int | None = None
        self.last_mag_at: float | None = None

    def feed(self, data: bytes, received_at: float | None = None) -> list[SensorSample]:
        """Принимает байты и возвращает обновлённые согласованные образцы."""
        self.buffer.extend(data)
        now = time.monotonic() if received_at is None else received_at
        samples: list[SensorSample] = []
        while True:
            if len(self.buffer) < 6:
                break
            marker = self.buffer.find(b"$M>")
            if marker < 0:
                self.buffer.clear()
                break
            if marker:
                del self.buffer[:marker]
            payload_size = self.buffer[3]
            total_size = 6 + payload_size
            if payload_size > 64:
                del self.buffer[:3]
                continue
            if len(self.buffer) < total_size:
                break
            packet = bytes(self.buffer[:total_size])
            del self.buffer[:total_size]
            payload = packet[5:-1]
            if msp_checksum(payload_size, packet[4], payload) != packet[-1]:
                continue
            sample = self._apply(packet[4], payload, now)
            if sample is not None:
                samples.append(sample)
        return samples

    def _apply(self, command: int, payload: bytes, now: float) -> SensorSample | None:
        """Применяет один проверенный ответ MSP к состоянию датчиков."""
        if command == MSP_ATTITUDE and len(payload) >= 6:
            self.roll_deg, self.pitch_deg, self.yaw_deg = struct.unpack_from("<hhh", payload)
            self.roll_deg /= 10.0
            self.pitch_deg /= 10.0
            self.last_attitude_at = now
        elif command == MSP_ALTITUDE and len(payload) >= 6:
            altitude_cm, vario_cm_s = struct.unpack_from("<ih", payload)
            self.altitude_m = altitude_cm / 100.0
            self.vario_m_s = vario_cm_s / 100.0
            self.last_altitude_at = now
        elif command == MSP_RAW_IMU and len(payload) >= 18:
            # MSP_RAW_IMU: ACC X/Y/Z, GYRO X/Y/Z, MAG X/Y/Z, по int16.
            _, _, _, _, _, _, self.mag_x, self.mag_y, self.mag_z = struct.unpack_from("<hhhhhhhhh", payload)
            self.last_mag_at = now
        else:
            return None
        return SensorSample(
            altitude_m=self.altitude_m,
            vario_m_s=self.vario_m_s,
            roll_deg=self.roll_deg,
            pitch_deg=self.pitch_deg,
            yaw_deg=self.yaw_deg,
            received_at=now,
            altitude_valid=self.altitude_m is not None and self.last_altitude_at is not None,
            attitude_valid=self.roll_deg is not None and self.last_attitude_at is not None,
            altitude_received_at=self.last_altitude_at,
            attitude_received_at=self.last_attitude_at,
            mag_x=self.mag_x,
            mag_y=self.mag_y,
            mag_z=self.mag_z,
            mag_received_at=self.last_mag_at,
        )


class BetaflightMspLink:
    """Открывает отдельный UART и циклически запрашивает датчики Betaflight."""

    def __init__(self, serial_port: str, baudrate: int, request_period_s: float = 0.05) -> None:
        """Открывает MSP UART в режиме чтения и записи запросов."""
        if request_period_s <= 0:
            raise ValueError("Период MSP должен быть положительным")
        self.serial_port = serial_port
        self.request_period_s = request_period_s
        self._fd = os.open(serial_port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self._configure_uart(self._fd, baudrate)
        settings = termios.tcgetattr(self._fd)
        settings[0] = 0
        settings[1] = 0
        settings[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        settings[3] = 0
        settings[6][termios.VMIN] = 0
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(self._fd, termios.TCSANOW, settings)
        self.parser = MspParser()
        self._next_request_at = 0.0

    @staticmethod
    def _configure_uart(fd: int, baudrate: int) -> None:
        """Настраивает стандартную скорость UART через termios."""
        baud_constant = getattr(termios, f"B{baudrate}", None)
        if baud_constant is None:
            raise ValueError(f"Скорость MSP не поддерживается termios: {baudrate}")
        settings = termios.tcgetattr(fd)
        settings[4] = baud_constant
        settings[5] = baud_constant
        termios.tcsetattr(fd, termios.TCSANOW, settings)

    def poll(self, now: float | None = None) -> list[SensorSample]:
        """Отправляет запросы по расписанию и читает доступные ответы."""
        current_time = time.monotonic() if now is None else now
        if current_time >= self._next_request_at:
            os.write(self._fd, build_msp_request(MSP_ATTITUDE))
            os.write(self._fd, build_msp_request(MSP_ALTITUDE))
            os.write(self._fd, build_msp_request(MSP_RAW_IMU))
            self._next_request_at = current_time + self.request_period_s
        ready, _, _ = select.select([self._fd], [], [], 0)
        if not ready:
            return []
        try:
            return self.parser.feed(os.read(self._fd, 4096), current_time)
        except BlockingIOError:
            return []

    def close(self) -> None:
        """Закрывает MSP UART."""
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def __enter__(self) -> "BetaflightMspLink":
        """Возвращает канал для контекстного менеджера."""
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        """Закрывает канал при выходе из контекстного менеджера."""
        self.close()
