"""Посадочный контроллер по данным Betaflight.

Контроллер формирует только изменённый RC-кадр для Betaflight. Он не работает
с PWM и не может обходить арминг, PID или защиту моторов полётника.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from src.protocols.betaflight_msp_link import SensorSample
from src.receiver.crsf import rebuild_rc_frame, unpack_channels


class LandingState(str, Enum):
    """Состояния посадочного автомата."""

    LIVE = "LIVE"
    TAKEOVER = "TAKEOVER"
    LEVELING = "LEVELING"
    DESCENT = "DESCENT"
    LANDED = "LANDED"
    DISARM_RELEASE = "DISARM_RELEASE"
    FAULT = "FAULT"


@dataclass(frozen=True)
class LandingConfig:
    """Параметры каналов и ограничений посадочного контроллера."""

    roll_channel: int
    pitch_channel: int
    throttle_channel: int
    rc_center: int
    rc_min: int
    rc_max: int
    target_roll_deg: float
    target_pitch_deg: float
    correction_per_degree: float
    max_correction: int
    descent_start_after_s: float
    throttle_step_per_s: float
    throttle_min: int
    sensor_max_age_s: float
    landed_altitude_m: float
    landed_vario_abs_m_s: float
    landed_hold_s: float
    fault_action: str


@dataclass(frozen=True)
class LandingResult:
    """Результат обработки кадра и текущего образца датчиков."""

    output_frame: bytes
    state: LandingState
    events: tuple[str, ...]
    roll_command: int
    pitch_command: int
    throttle_command: int


class LandingController:
    """Удерживает горизонт и снижает газ после перехода CRSF в takeover."""

    def __init__(self, config: LandingConfig) -> None:
        """Создаёт автомат в безопасном состоянии LIVE."""
        self.config = config
        self.state = LandingState.LIVE
        self.takeover_started_at: float | None = None
        self.landed_started_at: float | None = None
        self.last_sensor: SensorSample | None = None
        self.last_throttle: int | None = None
        self.initial_throttle: int | None = None
        self.fault_frame: bytes | None = None

    def process(
        self,
        frame: bytes,
        channels: tuple[int, ...],
        takeover_state: str,
        sensor: SensorSample | None,
        now: float,
    ) -> LandingResult:
        """Формирует RC-кадр для посадки или возвращает исходный кадр."""
        cfg = self.config
        output_channels = list(unpack_channels(frame[3:-1]))
        events: list[str] = []

        if takeover_state == "LIVE":
            if self.state is not LandingState.LIVE:
                events.append(f"{self.state.value} -> LIVE")
            self.state = LandingState.LIVE
            self.takeover_started_at = None
            self.landed_started_at = None
            self.last_throttle = None
            self.initial_throttle = None
            self.fault_frame = None
            return self._result(frame, output_channels, events, self.state)

        if self.state is LandingState.LIVE:
            self.state = LandingState.TAKEOVER
            self.takeover_started_at = now
            events.append("TAKEOVER -> LEVELING")

        if sensor is not None:
            self.last_sensor = sensor
        current_sensor = self.last_sensor
        if current_sensor is None or not current_sensor.complete or now - current_sensor.received_at > cfg.sensor_max_age_s:
            if self.state is not LandingState.FAULT:
                events.append("FAULT: MSP DATA STALE OR INCOMPLETE")
                self.fault_frame = frame
            self.state = LandingState.FAULT
            if cfg.fault_action == "DISARM":
                events.append("FAULT ACTION: DISARM должен обработать CRSF-мост")
            held_frame = self.fault_frame if self.fault_frame is not None else frame
            return self._result(held_frame, unpack_channels(held_frame[3:-1]), events, self.state)

        if self.state is LandingState.FAULT:
            self.state = LandingState.LEVELING
            self.fault_frame = None
            events.append("FAULT -> LEVELING: MSP данные восстановлены")

        roll = self._correction(current_sensor.roll_deg, cfg.target_roll_deg)
        pitch = self._correction(current_sensor.pitch_deg, cfg.target_pitch_deg)
        output_channels[cfg.roll_channel] = roll
        output_channels[cfg.pitch_channel] = pitch
        if self.state is LandingState.TAKEOVER:
            self.state = LandingState.LEVELING
            events.append("TAKEOVER -> LEVELING")

        start_time = self.takeover_started_at if self.takeover_started_at is not None else now
        elapsed = now - start_time
        if elapsed >= cfg.descent_start_after_s and self.state is LandingState.LEVELING:
            self.state = LandingState.DESCENT
            events.append("LEVELING -> DESCENT")

        if self.state in (LandingState.LEVELING, LandingState.DESCENT, LandingState.LANDED):
            output_channels[cfg.throttle_channel] = self._throttle_command(output_channels[cfg.throttle_channel], elapsed)
        if self.state is LandingState.DESCENT and self._is_landed(current_sensor, now):
            self.state = LandingState.LANDED
            output_channels[cfg.throttle_channel] = cfg.throttle_min
            events.append("DESCENT -> LANDED: baro threshold")
        return self._result(rebuild_rc_frame(frame, output_channels), output_channels, events, self.state)

    def _correction(self, measured: float | None, target: float) -> int:
        """Преобразует ошибку наклона в ограниченную RC-команду."""
        if measured is None:
            return self.config.rc_center
        delta = round((target - measured) * self.config.correction_per_degree)
        delta = max(-self.config.max_correction, min(self.config.max_correction, delta))
        return max(self.config.rc_min, min(self.config.rc_max, self.config.rc_center + delta))

    def _throttle_command(self, current: int, elapsed: float) -> int:
        """Плавно уменьшает газ с ограничением скорости изменения."""
        if self.last_throttle is None:
            self.last_throttle = current
            self.initial_throttle = current
        target = self.config.throttle_min
        step = max(0.0, self.config.throttle_step_per_s) * max(0.0, elapsed)
        initial = self.initial_throttle if self.initial_throttle is not None else current
        command = round(max(target, initial - step))
        self.last_throttle = command
        return command

    def _is_landed(self, sensor: SensorSample, now: float) -> bool:
        """Определяет тестовое касание по барометрической высоте и вариометру."""
        if sensor.altitude_m is None or sensor.vario_m_s is None:
            self.landed_started_at = None
            return False
        near_ground = abs(sensor.altitude_m) <= self.config.landed_altitude_m
        nearly_still = abs(sensor.vario_m_s) <= self.config.landed_vario_abs_m_s
        if near_ground and nearly_still:
            if self.landed_started_at is None:
                self.landed_started_at = now
            return now - self.landed_started_at >= self.config.landed_hold_s
        self.landed_started_at = None
        return False

    @staticmethod
    def _result(frame: bytes, channels: list[int], events: list[str], state: LandingState = LandingState.LIVE) -> LandingResult:
        """Создаёт унифицированный результат для журнала и моста."""
        return LandingResult(
            output_frame=frame,
            state=state,
            events=tuple(events),
            roll_command=channels[0],
            pitch_command=channels[1],
            throttle_command=channels[2],
        )
