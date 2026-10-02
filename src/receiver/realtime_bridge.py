"""Независимый быстрый тракт CRSF между приёмником и полётным контроллером.

Поток владеет UART0 один: читает входящие CRSF-кадры и сразу передаёт их на
FC. MSP, OSD, камера и журнал получают лишь копию кадров через очередь и не
могут остановить физический выход TXD0.
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass

from src.receiver.crsf import CRSF_RC_CHANNELS_PACKED, CrsfReceiver, rebuild_rc_frame, unpack_channels


@dataclass(frozen=True)
class RealtimeFrame:
    """Один кадр, уже физически переданный на выход Raspberry."""

    input_frame: bytes
    output_frame: bytes
    received_at: float
    transmitted_at: float


@dataclass(frozen=True)
class RealtimeBridgeStats:
    """Снимок счётчиков быстрого тракта для журнала и диагностики."""

    received_frames: int
    transmitted_frames: int
    received_bytes: int
    control_dropped_frames: int
    last_received_at: float | None
    last_rx_interval_ms: float | None
    rx_gap_events: int
    last_rx_gap_ms: float | None


class RealtimeCrsfBridge:
    """Передаёт CRSF в отдельном потоке без ожидания диагностики и камеры."""

    def __init__(
        self,
        serial_port: str,
        baudrate: int,
        *,
        mode_channel: int,
        follow_min: int,
        disarm_channel: int,
        disarm_max: int,
        override_max_age_s: float = 0.25,
        frame_queue_size: int = 128,
        rx_gap_threshold_s: float = 0.1,
    ) -> None:
        """Создаёт тракт; номера каналов передаются в привычной нумерации CH1=1."""
        if override_max_age_s <= 0:
            raise ValueError("Время жизни команды сопровождения должно быть положительным")
        if rx_gap_threshold_s <= 0:
            raise ValueError("Порог CRSF GAP должен быть положительным")
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.mode_channel = mode_channel - 1
        self.follow_min = follow_min
        self.disarm_channel = disarm_channel - 1
        self.disarm_max = disarm_max
        self.override_max_age_s = override_max_age_s
        self.rx_gap_threshold_s = rx_gap_threshold_s
        self._frames: queue.Queue[RealtimeFrame] = queue.Queue(maxsize=frame_queue_size)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._override_channels: tuple[int, int, int, int] | None = None
        self._override_at = 0.0
        self._received_frames = 0
        self._transmitted_frames = 0
        self._received_bytes = 0
        self._control_dropped_frames = 0
        self._last_received_at: float | None = None
        self._last_rx_interval_ms: float | None = None
        self._rx_gap_events = 0
        self._last_rx_gap_ms: float | None = None
        self._error: Exception | None = None

    def start(self) -> None:
        """Запускает единственный поток, владеющий UART приёмника и FC."""
        if self._thread is not None:
            raise RuntimeError("Быстрый CRSF-мост уже запущен")
        self._thread = threading.Thread(target=self._run, name="dront16-crsf-io", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Останавливает поток и ждёт освобождения UART."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def check_error(self) -> None:
        """Поднимает ошибку потока в основном процессе до следующего решения."""
        if self._error is not None:
            raise RuntimeError(f"Быстрый CRSF-мост остановлен: {self._error}") from self._error

    def set_visual_override(self, channels: tuple[int, ...] | list[int] | None) -> None:
        """Публикует только CH1–CH4 команды сопровождения для следующих RC-кадров.

        DISARM и выход CH6 из FOLLOW всегда обходят эту команду прямо в
        быстром потоке. Просроченная команда автоматически не используется.
        """
        override: tuple[int, int, int, int] | None = None
        if channels is not None:
            if len(channels) < 4:
                raise ValueError("Для сопровождения нужны значения CH1–CH4")
            override = tuple(int(value) for value in channels[:4])
            if not all(0 <= value <= 0x7FF for value in override):
                raise ValueError("Команда сопровождения содержит канал вне CRSF диапазона")
        with self._lock:
            self._override_channels = override
            self._override_at = time.monotonic() if override is not None else 0.0

    def drain_frames(self, maximum: int = 8) -> list[RealtimeFrame]:
        """Возвращает последние кадры для медленного контура, сбрасывая старые.

        Медленный контур не должен догонять историю: ему достаточно свежего
        состояния пульта. Физическая передача при этом уже выполнена.
        """
        if maximum < 1:
            raise ValueError("Максимум кадров должен быть не меньше одного")
        drained: list[RealtimeFrame] = []
        while True:
            try:
                drained.append(self._frames.get_nowait())
            except queue.Empty:
                break
        if len(drained) > maximum:
            with self._lock:
                self._control_dropped_frames += len(drained) - maximum
            return drained[-maximum:]
        return drained

    def stats(self) -> RealtimeBridgeStats:
        """Возвращает согласованный снимок счётчиков без ожидания UART."""
        with self._lock:
            return RealtimeBridgeStats(
                self._received_frames,
                self._transmitted_frames,
                self._received_bytes,
                self._control_dropped_frames,
                self._last_received_at,
                self._last_rx_interval_ms,
                self._rx_gap_events,
                self._last_rx_gap_ms,
            )

    def _output_frame(self, frame: bytes, now: float) -> bytes:
        """Применяет свежую команду зрения либо оставляет кадр пилота без изменений."""
        if len(frame) < 4 or frame[2] != CRSF_RC_CHANNELS_PACKED:
            return frame
        channels = list(unpack_channels(frame[3:-1]))
        # Физический DISARM имеет высший приоритет и никогда не переписывается.
        if channels[self.disarm_channel] <= self.disarm_max:
            return frame
        with self._lock:
            override = self._override_channels
            override_age_s = now - self._override_at
        # Нижнее и среднее положения CH6 мгновенно возвращают управление пилоту.
        if (
            override is None
            or override_age_s > self.override_max_age_s
            or channels[self.mode_channel] < self.follow_min
        ):
            return frame
        channels[:4] = override
        return rebuild_rc_frame(frame, channels)

    def _run(self) -> None:
        """Читает и выдаёт кадры; здесь намеренно нет MSP, файла и печати."""
        buffer = bytearray()
        try:
            with CrsfReceiver(self.serial_port, self.baudrate, write_enabled=True) as uart:
                while not self._stop.is_set():
                    for frame in uart.read_raw_frames(buffer, timeout_s=0.005):
                        received_at = time.monotonic()
                        output = self._output_frame(frame, received_at)
                        uart.write_frame(output)
                        transmitted_at = time.monotonic()
                        with self._lock:
                            if self._last_received_at is not None:
                                self._last_rx_interval_ms = (received_at - self._last_received_at) * 1000.0
                                if self._last_rx_interval_ms >= self.rx_gap_threshold_s * 1000.0:
                                    self._rx_gap_events += 1
                                    self._last_rx_gap_ms = self._last_rx_interval_ms
                            self._last_received_at = received_at
                            self._received_frames += 1
                            self._transmitted_frames += 1
                            self._received_bytes = uart.bytes_received
                        packet = RealtimeFrame(frame, output, received_at, transmitted_at)
                        try:
                            self._frames.put_nowait(packet)
                        except queue.Full:
                            with self._lock:
                                self._control_dropped_frames += 1
        except (OSError, ValueError, RuntimeError) as error:
            self._error = error

    def __enter__(self) -> "RealtimeCrsfBridge":
        """Запускает тракт для контекстного менеджера."""
        self.start()
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        """Всегда останавливает поток при завершении основного моста."""
        self.stop()
