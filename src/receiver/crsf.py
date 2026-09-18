"""Только чтение CRSF-пакетов от ELRS-приёмника через UART."""

from __future__ import annotations

import os
import select
import fcntl
import struct
import termios
import time
from dataclasses import dataclass


# Тип CRSF-пакета с 16 каналами по 11 бит.
CRSF_RC_CHANNELS_PACKED = 0x16
# Максимальный размер кадра CRSF по спецификации.
CRSF_MAX_FRAME_LENGTH = 64


@dataclass(frozen=True)
class ReceiverFrame:
    """Результат разбора одного кадра CRSF."""

    channels: tuple[int, ...]
    received_at: float


def crc8_dvb_s2(data: bytes) -> int:
    """Вычисляет CRC8-DVB-S2, используемый кадрами CRSF."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def unpack_channels(payload: bytes) -> tuple[int, ...]:
    """Распаковывает 16 каналов CRSF из 22 байт по 11 бит."""
    if len(payload) != 22:
        raise ValueError("CRSF RC_CHANNELS_PACKED должен содержать 22 байта")
    packed = int.from_bytes(payload, "little")
    return tuple((packed >> (11 * index)) & 0x07FF for index in range(16))


def parse_frames(buffer: bytearray, now: float | None = None) -> list[ReceiverFrame]:
    """Извлекает проверенные RC-кадры из накопленного буфера UART."""
    frames: list[ReceiverFrame] = []
    received_at = time.monotonic() if now is None else now
    while len(buffer) >= 2:
        length = buffer[1]
        if length < 2 or length > CRSF_MAX_FRAME_LENGTH:
            del buffer[0]
            continue
        total_length = length + 2
        if len(buffer) < total_length:
            break
        frame = bytes(buffer[:total_length])
        del buffer[:total_length]
        frame_type = frame[2]
        payload = frame[3:-1]
        if crc8_dvb_s2(frame[2:-1]) != frame[-1]:
            continue
        if frame_type == CRSF_RC_CHANNELS_PACKED:
            try:
                frames.append(ReceiverFrame(unpack_channels(payload), received_at))
            except ValueError:
                continue
    return frames


class CrsfReceiver:
    """Открывает UART только на чтение и принимает RC-кадры CRSF."""

    def __init__(self, serial_port: str, baudrate: int) -> None:
        """Открывает UART с параметрами 8N1 без передачи данных."""
        self.serial_port = serial_port
        # Диагностические счётчики входного потока без передачи данных.
        self.bytes_received = 0
        self._fd = os.open(serial_port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        settings = termios.tcgetattr(self._fd)
        settings[0] = 0
        settings[1] = 0
        settings[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        settings[3] = 0
        settings[6][termios.VMIN] = 0
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(self._fd, termios.TCSANOW, settings)
        self._configure_uart(self._fd, baudrate)

    @staticmethod
    def _configure_uart(fd: int, baudrate: int) -> None:
        """Настраивает нестандартную скорость UART через Linux termios2."""
        # TCSETS2/BOTHER позволяют использовать CRSF 420000 бод, которого
        # нет среди обычных Python-констант termios.
        tcgets2 = 0x802C542A
        tcsets2 = 0x402C542B
        bother = 0x1000
        cbaud = 0x100F
        termios2 = bytearray(44)
        fcntl.ioctl(fd, tcgets2, termios2, True)
        flags = struct.unpack_from("I", termios2, 8)[0]
        flags = (flags & ~cbaud) | bother
        struct.pack_into("I", termios2, 8, flags)
        struct.pack_into("I", termios2, 36, baudrate)
        struct.pack_into("I", termios2, 40, baudrate)
        fcntl.ioctl(fd, tcsets2, termios2)

    def read_frames(self, buffer: bytearray) -> list[ReceiverFrame]:
        """Читает доступные байты UART и возвращает корректные RC-кадры."""
        ready, _, _ = select.select([self._fd], [], [], 0.05)
        if ready:
            try:
                chunk = os.read(self._fd, 4096)
                buffer.extend(chunk)
                self.bytes_received += len(chunk)
            except BlockingIOError:
                pass
        return parse_frames(buffer)

    def close(self) -> None:
        """Закрывает UART без отправки байтов в приёмник."""
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def __enter__(self) -> "CrsfReceiver":
        """Возвращает открытый приёмник для контекстного менеджера."""
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        """Закрывает UART при выходе из контекстного менеджера."""
        self.close()
