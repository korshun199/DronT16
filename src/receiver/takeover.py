"""Состояния и решения управляемого CRSF-перехвата."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from src.receiver.crsf import rebuild_rc_frame, unpack_channels


class TakeoverState(str, Enum):
    """Режим пересылки RC-кадров на полётный контроллер."""

    LIVE = "LIVE"
    TAKEOVER = "TAKEOVER"
    DISARM_RELEASE = "DISARM_RELEASE"


@dataclass(frozen=True)
class TakeoverConfig:
    """Пороги каналов и номера каналов для takeover."""

    loss_channel: int
    link_lost_min: int
    link_ok_max: int
    disarm_channel: int
    disarm_active_max: int
    arm_active_min: int
    throttle_channel: int
    throttle_zero: int
    throttle_ramp_s: float


@dataclass(frozen=True)
class TakeoverResult:
    """Результат обработки одного входящего RC-кадра."""

    output_frame: bytes
    output_kind: str
    state: TakeoverState
    link_lost: bool
    disarmed: bool
    events: tuple[str, ...]


class TakeoverController:
    """Переключает живой RC-поток на повтор последнего полного кадра."""

    def __init__(self, config: TakeoverConfig) -> None:
        """Создаёт контроллер с безопасным состоянием LIVE."""
        self.config = config
        self.state = TakeoverState.LIVE
        self.last_live_frame: bytes | None = None
        self.frozen_frame: bytes | None = None
        self.frozen_channels: tuple[int, ...] | None = None
        self.takeover_started_at: float | None = None
        self.lockout_until_link_ok = False
        self.last_disarmed: bool | None = None

    def process(self, frame: bytes, channels: tuple[int, ...], now: float | None = None) -> TakeoverResult:
        """Обрабатывает один кадр и возвращает ровно один кадр для FC."""
        current_time = 0.0 if now is None else now
        cfg = self.config
        link_value = channels[cfg.loss_channel]
        disarm_value = channels[cfg.disarm_channel]
        link_lost = link_value >= cfg.link_lost_min
        disarmed = disarm_value <= cfg.disarm_active_max
        armed = disarm_value >= cfg.arm_active_min
        events: list[str] = []

        if self.state is TakeoverState.DISARM_RELEASE:
            self.state = TakeoverState.LIVE
            events.append("DISARM_RELEASE -> LIVE")

        if self.last_disarmed is None or disarmed != self.last_disarmed:
            events.append(f"CH{cfg.disarm_channel + 1}={disarm_value} -> {'DISARM' if disarmed else 'ARM'}")
        self.last_disarmed = disarmed

        if self.state is TakeoverState.LIVE:
            if self.lockout_until_link_ok and not link_lost:
                self.lockout_until_link_ok = False
                events.append("DISARM_RELEASE -> LIVE; CH7 normal")

            if link_lost and armed and not self.lockout_until_link_ok:
                if self.last_live_frame is None:
                    events.append("TAKEOVER BLOCKED: нет сохранённого RC-кадра")
                    self.last_live_frame = frame
                    return TakeoverResult(frame, "LIVE", self.state, link_lost, disarmed, tuple(events))
                self.frozen_frame = self.last_live_frame
                self.frozen_channels = tuple(unpack_channels(self.frozen_frame[3:-1]))
                self.takeover_started_at = current_time
                self.state = TakeoverState.TAKEOVER
                events.append("LIVE -> TAKEOVER")
                events.append("сохранен последний полный RC-кадр")
                return TakeoverResult(
                    self._ramped_frame(current_time),
                    "THROTTLE_RAMP",
                    self.state,
                    link_lost,
                    disarmed,
                    tuple(events),
                )

            if link_lost and not armed and not self.lockout_until_link_ok:
                events.append("TAKEOVER BLOCKED: CH5 не в ARM")
            self.last_live_frame = frame
            return TakeoverResult(frame, "LIVE", self.state, link_lost, disarmed, tuple(events))

        # В TAKEOVER DISARM всегда обрабатывается первым.
        if disarmed:
            disarm_channels = list(channels)
            disarm_channels[cfg.disarm_channel] = cfg.disarm_active_max
            disarm_frame = rebuild_rc_frame(frame, disarm_channels)
            self.state = TakeoverState.DISARM_RELEASE
            self.frozen_frame = None
            self.frozen_channels = None
            self.takeover_started_at = None
            self.lockout_until_link_ok = link_lost
            self.last_live_frame = disarm_frame
            events.append("DISARM: MOTORS OFF")
            events.append("TAKEOVER -> DISARM_RELEASE; живой канал возвращён")
            return TakeoverResult(disarm_frame, "DISARM", self.state, link_lost, disarmed, tuple(events))

        if not link_lost:
            self.state = TakeoverState.LIVE
            self.frozen_frame = None
            self.frozen_channels = None
            self.takeover_started_at = None
            self.lockout_until_link_ok = False
            self.last_live_frame = frame
            events.append("TAKEOVER -> LIVE")
            return TakeoverResult(frame, "LIVE", self.state, link_lost, disarmed, tuple(events))

        if self.frozen_frame is None:
            raise RuntimeError("TAKEOVER не имеет сохранённого RC-кадра")
        events.append("RC SUPPRESS: свежий кадр не передан")
        return TakeoverResult(
            self._ramped_frame(current_time),
            "THROTTLE_RAMP",
            self.state,
            link_lost,
            disarmed,
            tuple(events),
        )

    def _ramped_frame(self, current_time: float) -> bytes:
        """Формирует сохранённый кадр с плавно уменьшаемым газом."""
        if self.frozen_frame is None or self.frozen_channels is None:
            raise RuntimeError("Нет сохранённого кадра для рампы газа")
        if self.takeover_started_at is None:
            progress = 1.0
        elif self.config.throttle_ramp_s <= 0:
            progress = 1.0
        else:
            progress = min(1.0, max(0.0, (current_time - self.takeover_started_at) / self.config.throttle_ramp_s))
        channels = list(self.frozen_channels)
        initial_throttle = channels[self.config.throttle_channel]
        channels[self.config.throttle_channel] = round(
            initial_throttle + (self.config.throttle_zero - initial_throttle) * progress
        )
        return rebuild_rc_frame(self.frozen_frame, channels)
