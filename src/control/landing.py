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
    TURNING = "TURNING"
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
    disarm_channel: int
    disarm_value: int
    stabilization_channel: int | None
    stabilization_value: int
    level_roll_tolerance_deg: float = 5.0
    level_pitch_tolerance_deg: float = 5.0
    level_hold_s: float = 1.0
    level_throttle: int = 992
    # Пропорциональный коэффициент удержания высоты в единицах CRSF на метр.
    altitude_hold_gain: float = 80.0
    # Интегральный коэффициент, компенсирующий постоянную ошибку газа висения.
    altitude_hold_integral_gain: float = 12.0
    # Коэффициент демпфирования по вариометру.
    altitude_hold_vario_gain: float = 35.0
    max_altitude_correction: int = 120
    # Максимум накопленной интегральной поправки в единицах CRSF.
    max_altitude_integral_correction: int = 80
    landing_throttle_barrier: int = 600
    # Разворот перед посадкой выключен по умолчанию в API; конфигурация проекта включает его явно.
    turn_enabled: bool = False
    # Требуемый угол разворота по данным yaw полётного контроллера.
    turn_degrees: float = 180.0
    # Прямое значение CRSF канала yaw во время разворота; 1300 — направление по умолчанию.
    turn_yaw_command: int = 1300
    # Максимальное время разворота до безопасного FAULT.
    turn_timeout_s: float = 6.0
    # Если больше нуля, разворот завершается ровно по времени yaw-команды.
    turn_duration_s: float = 0.0
    # Допуск завершения разворота по углу.
    turn_heading_tolerance_deg: float = 8.0
    # Номер yaw-канала CRSF в нулевой индексации; CH4 = 3.
    yaw_channel: int = 3
    # Максимальная скорость перехода от газа пилота к газу висения.
    takeover_throttle_step_per_s: float = 50.0
    # Превышение высоты, после которого включается защита от набора.
    climb_guard_altitude_error_m: float = 0.20
    # Скорость набора, после которой включается защита от набора.
    climb_guard_vario_m_s: float = 0.50
    # Жёстний верхний предел газа при активной защите от набора.
    climb_guard_max_throttle: int = 850


@dataclass(frozen=True)
class LandingResult:
    """Результат обработки кадра и текущего образца датчиков."""

    output_frame: bytes
    state: LandingState
    events: tuple[str, ...]
    roll_command: int
    pitch_command: int
    throttle_command: int
    yaw_command: int
    disarm_requested: bool = False


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
        self.altitude_zero_m: float | None = None
        self.was_armed = False
        self.level_stable_since: float | None = None
        self.level_was_ok: bool | None = None
        self.hold_altitude_m: float | None = None
        self.altitude_integral_m_s = 0.0
        self.last_hold_time: float | None = None
        self.throttle_barrier_triggered = False
        self.turn_last_yaw_deg: float | None = None
        self.turn_progress_deg = 0.0
        self.turn_failed = False
        self.turn_completed = False
        self.turn_started_at: float | None = None
        self.takeover_entry_throttle: int | None = None
        self.last_takeover_throttle_time: float | None = None
        self.climb_guard_active = False

    def process(
        self,
        frame: bytes,
        channels: tuple[int, ...],
        takeover_state: str,
        sensor: SensorSample | None,
        now: float,
        armed: bool = True,
    ) -> LandingResult:
        """Формирует RC-кадр для посадки или возвращает исходный кадр."""
        cfg = self.config
        output_channels = list(unpack_channels(frame[3:-1]))
        events: list[str] = []

        if not armed:
            if self.was_armed:
                events.append("ARM -> DISARM: altitude reference cleared")
            self.altitude_zero_m = None
            self.was_armed = False
            self.level_stable_since = None
            self.level_was_ok = None
            self.hold_altitude_m = None
            self.altitude_integral_m_s = 0.0
            self.last_hold_time = None
            self.throttle_barrier_triggered = False
            self.turn_last_yaw_deg = None
            self.turn_progress_deg = 0.0
            self.turn_failed = False
            self.turn_completed = False
            self.turn_started_at = None
            self.takeover_entry_throttle = None
            self.last_takeover_throttle_time = None
            self.climb_guard_active = False
        elif sensor is not None and sensor.is_fresh(now, cfg.sensor_max_age_s) and self.altitude_zero_m is None:
            self.altitude_zero_m = sensor.altitude_m
            self.was_armed = True
            events.append(f"ALTITUDE_ZERO={self.altitude_zero_m:.2f}m")
        else:
            self.was_armed = armed

        if takeover_state == "LIVE":
            if self.state is not LandingState.LIVE:
                events.append(f"{self.state.value} -> LIVE")
            self.state = LandingState.LIVE
            self.takeover_started_at = None
            self.landed_started_at = None
            self.last_throttle = None
            self.initial_throttle = None
            self.fault_frame = None
            self.level_stable_since = None
            self.hold_altitude_m = None
            self.altitude_integral_m_s = 0.0
            self.last_hold_time = None
            self.throttle_barrier_triggered = False
            self.turn_last_yaw_deg = None
            self.turn_progress_deg = 0.0
            self.turn_failed = False
            self.turn_completed = False
            self.turn_started_at = None
            self.takeover_entry_throttle = None
            self.last_takeover_throttle_time = None
            self.climb_guard_active = False
            return self._result(frame, output_channels, events, self.state)

        if self.state is LandingState.LIVE:
            self.state = LandingState.TAKEOVER
            self.takeover_started_at = now
            # Запоминаем реальный газ из последнего кадра пилота. Переход к
            # заданному газу висения будет ограничен по скорости и не вызовет
            # резкого набора высоты при включении CH7.
            self.takeover_entry_throttle = output_channels[cfg.throttle_channel]
            self.last_takeover_throttle_time = now
            events.append("TAKEOVER -> LEVELING")

        if sensor is not None:
            self.last_sensor = sensor
        current_sensor = self.last_sensor
        if current_sensor is None or not current_sensor.is_fresh(now, cfg.sensor_max_age_s):
            if self.state is not LandingState.FAULT:
                events.append("FAULT: MSP DATA STALE OR INCOMPLETE")
                self.fault_frame = frame
            self.state = LandingState.FAULT
            if cfg.fault_action == "DISARM":
                disarm_channels = unpack_channels(frame[3:-1])
                disarm_channels = list(disarm_channels)
                disarm_channels[cfg.disarm_channel] = cfg.disarm_value
                disarm_frame = rebuild_rc_frame(frame, disarm_channels)
                events.append("FAULT ACTION: DISARM; MSP данные небезопасны")
                return self._result(
                    disarm_frame,
                    disarm_channels,
                    events,
                    self.state,
                    disarm_requested=True,
                )
            held_frame = self.fault_frame if self.fault_frame is not None else frame
            return self._result(held_frame, unpack_channels(held_frame[3:-1]), events, self.state)

        if self.state is LandingState.FAULT and self.turn_failed:
            # После тайм-аута разворота не разрешаем автоматически перейти к посадке.
            held_frame = self.fault_frame if self.fault_frame is not None else frame
            return self._result(held_frame, unpack_channels(held_frame[3:-1]), events, self.state)

        if self.state is LandingState.FAULT:
            self.state = LandingState.LEVELING
            self.fault_frame = None
            events.append("FAULT -> LEVELING: MSP данные восстановлены")

        # При потере связи фиксируем высоту, на которой был пилотский контроль.
        # До начала снижения Raspberry удерживает эту высоту, а не последний
        # случайный газ из RC-кадра.
        if self.hold_altitude_m is None and current_sensor.altitude_m is not None:
            self.hold_altitude_m = current_sensor.altitude_m
            self.altitude_integral_m_s = 0.0
            self.last_hold_time = now
            events.append(f"ALTITUDE_HOLD={self.hold_altitude_m:.2f}m")

        if self.state is LandingState.TAKEOVER:
            # Сначала выравниваем аппарат; разворот начинается только после
            # подтверждённого горизонтального положения.
            self.state = LandingState.LEVELING
            events.append("TAKEOVER -> LEVELING")

        roll = self._correction(current_sensor.roll_deg, cfg.target_roll_deg)
        pitch = self._correction(current_sensor.pitch_deg, cfg.target_pitch_deg)
        output_channels[cfg.roll_channel] = roll
        output_channels[cfg.pitch_channel] = pitch
        if cfg.stabilization_channel is not None:
            output_channels[cfg.stabilization_channel] = cfg.stabilization_value
        if self.state is LandingState.TURNING:
            output_channels[cfg.yaw_channel] = cfg.turn_yaw_command
            self._update_turn_progress(current_sensor.yaw_deg)
            elapsed_turn = now - (self.turn_started_at if self.turn_started_at is not None else now)
            if cfg.turn_duration_s > 0.0:
                turn_done = elapsed_turn >= cfg.turn_duration_s
            else:
                turn_done = self.turn_progress_deg >= max(0.0, cfg.turn_degrees - cfg.turn_heading_tolerance_deg)
            if turn_done:
                self.state = LandingState.LEVELING
                self.turn_completed = True
                output_channels[cfg.yaw_channel] = cfg.rc_center
                self.level_stable_since = None
                self.level_was_ok = None
                if cfg.turn_duration_s > 0.0:
                    events.append(
                        f"TURNING -> LEVELING: duration={elapsed_turn:.1f}s "
                        f"progress={self.turn_progress_deg:.1f}deg"
                    )
                else:
                    events.append(f"TURNING -> LEVELING: progress={self.turn_progress_deg:.1f}deg")
            elif cfg.turn_duration_s <= 0.0 and elapsed_turn >= cfg.turn_timeout_s:
                self.state = LandingState.FAULT
                self.turn_failed = True
                self.fault_frame = frame
                events.append(
                    f"TURN FAULT: timeout progress={self.turn_progress_deg:.1f}/{cfg.turn_degrees:.1f}deg"
                )

        if self.state is LandingState.TURNING:
            self._set_hold_throttle(output_channels, current_sensor, now, events)
            return self._result(rebuild_rc_frame(frame, output_channels), output_channels, events, self.state)

        level_ok = self._is_level(current_sensor)
        if self.state is not LandingState.LEVELING:
            level_ok = False
        if level_ok != self.level_was_ok:
            events.append(
                "LEVELING: within attitude limits"
                if level_ok
                else "LEVELING: attitude outside limits; descent paused"
            )
            self.level_was_ok = level_ok
        if level_ok:
            if self.level_stable_since is None:
                self.level_stable_since = now
        else:
            self.level_stable_since = None

        start_time = self.takeover_started_at if self.takeover_started_at is not None else now
        elapsed = now - start_time
        level_hold = self.level_stable_since is not None and now - self.level_stable_since >= cfg.level_hold_s
        if elapsed >= cfg.descent_start_after_s and level_hold and self.state is LandingState.LEVELING:
            if cfg.turn_enabled and not self.turn_completed:
                if current_sensor.yaw_deg is None:
                    self.state = LandingState.FAULT
                    self.turn_failed = True
                    self.fault_frame = frame
                    events.append("TURN FAULT: yaw данные отсутствуют")
                else:
                    self.state = LandingState.TURNING
                    self.turn_last_yaw_deg = current_sensor.yaw_deg
                    self.turn_progress_deg = 0.0
                    # Таймер начинается именно здесь, когда впервые подаём yaw,
                    # и не включает предыдущее выравнивание.
                    self.turn_started_at = now
                    events.append(f"LEVELING -> TURNING: yaw +{cfg.turn_degrees:.0f}deg")
                    output_channels[cfg.yaw_channel] = cfg.turn_yaw_command
                    self._set_hold_throttle(output_channels, current_sensor, now, events)
                    return self._result(rebuild_rc_frame(frame, output_channels), output_channels, events, self.state)
            else:
                self.state = LandingState.DESCENT
                events.append("LEVELING -> DESCENT")

        if self.state is LandingState.LEVELING:
            self._set_hold_throttle(output_channels, current_sensor, now, events)
        if self.state in (LandingState.DESCENT, LandingState.LANDED):
            output_channels[cfg.throttle_channel] = self._throttle_command(output_channels[cfg.throttle_channel], elapsed)
            if self.throttle_barrier_triggered and self.state is LandingState.DESCENT:
                self.state = LandingState.LANDED
                output_channels[cfg.throttle_channel] = cfg.throttle_min
                events.append(
                    f"DESCENT -> LANDED: throttle barrier {cfg.landing_throttle_barrier}; "
                    "газ переведен в минимум"
                )
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
        # Резко завершаем снижение при пересечении настроенного барьера сверху.
        barrier = self.config.landing_throttle_barrier
        if self.last_throttle is not None and self.last_throttle > barrier >= command:
            command = target
            self.throttle_barrier_triggered = True
        self.last_throttle = command
        return command

    def _hold_throttle(self, sensor: SensorSample, now: float) -> int:
        """Непрерывно удерживает высоту по ошибке, интегратору и вариометру."""
        cfg = self.config
        # Вход в takeover не должен менять газ ступенькой: это могло вызвать
        # набор высоты, если пилот в момент CH7 держал газ ниже висения.
        entry = self.takeover_entry_throttle
        if entry is None:
            entry = cfg.level_throttle
        previous_time = self.last_takeover_throttle_time
        dt_takeover = 0.0 if previous_time is None else max(0.0, min(0.25, now - previous_time))
        self.last_takeover_throttle_time = now
        step = max(0.0, cfg.takeover_throttle_step_per_s) * dt_takeover
        if entry < cfg.level_throttle:
            command = min(float(cfg.level_throttle), float(entry) + step)
        else:
            command = max(float(cfg.level_throttle), float(entry) - step)
        self.climb_guard_active = False
        if self.hold_altitude_m is not None and sensor.altitude_m is not None:
            # Положительная ошибка означает, что аппарат ниже заданной высоты.
            altitude_error = self.hold_altitude_m - sensor.altitude_m
            dt = 0.0 if self.last_hold_time is None else max(0.0, min(0.25, now - self.last_hold_time))
            self.last_hold_time = now
            self.altitude_integral_m_s += altitude_error * dt
            integral_gain = abs(cfg.altitude_hold_integral_gain)
            if integral_gain > 0.0:
                integral_limit = cfg.max_altitude_integral_correction / integral_gain
                self.altitude_integral_m_s = max(-integral_limit, min(integral_limit, self.altitude_integral_m_s))
            else:
                self.altitude_integral_m_s = 0.0
            proportional = altitude_error * cfg.altitude_hold_gain
            integral = self.altitude_integral_m_s * cfg.altitude_hold_integral_gain
            # Положительный варио означает набор высоты и уменьшает газ.
            vario = -(sensor.vario_m_s or 0.0) * cfg.altitude_hold_vario_gain
            correction = round(proportional + integral + vario)
            correction = max(-cfg.max_altitude_correction, min(cfg.max_altitude_correction, correction))
            command += correction
            # Отдельная защита от набора: при положительном варио или
            # превышении высоты команда не может оставаться на газе висения.
            # Это ограничитель, а не мгновенный DISARM.
            climb_guard = (
                sensor.altitude_m - self.hold_altitude_m >= cfg.climb_guard_altitude_error_m
                or (sensor.vario_m_s or 0.0) >= cfg.climb_guard_vario_m_s
            )
            if climb_guard:
                command = min(command, float(cfg.climb_guard_max_throttle))
                self.climb_guard_active = True
        return max(cfg.rc_min, min(cfg.rc_max, round(command)))

    def _set_hold_throttle(
        self,
        output_channels: list[int],
        sensor: SensorSample,
        now: float,
        events: list[str],
    ) -> None:
        """Записывает газ удержания и однократно сообщает о защите от набора."""
        was_active = self.climb_guard_active
        output_channels[self.config.throttle_channel] = self._hold_throttle(sensor, now)
        if self.climb_guard_active and not was_active:
            events.append(
                "CLIMB GUARD: gas limited; altitude/vario above takeover target"
            )

    def _update_turn_progress(self, yaw_deg: float | None) -> None:
        """Накопительно считает yaw с учётом перехода через 0/360 градусов."""
        if yaw_deg is None or self.turn_last_yaw_deg is None:
            return
        delta = (yaw_deg - self.turn_last_yaw_deg + 180.0) % 360.0 - 180.0
        direction = 1.0 if self.config.turn_yaw_command >= self.config.rc_center else -1.0
        self.turn_progress_deg += delta * direction
        self.turn_last_yaw_deg = yaw_deg

    def _is_landed(self, sensor: SensorSample, now: float) -> bool:
        """Определяет тестовое касание по барометрической высоте и вариометру."""
        if sensor.altitude_m is None or sensor.vario_m_s is None:
            self.landed_started_at = None
            return False
        if self.altitude_zero_m is None:
            return False
        relative_altitude = sensor.altitude_m - self.altitude_zero_m
        near_ground = abs(relative_altitude) <= self.config.landed_altitude_m
        nearly_still = abs(sensor.vario_m_s) <= self.config.landed_vario_abs_m_s
        if near_ground and nearly_still:
            if self.landed_started_at is None:
                self.landed_started_at = now
            return now - self.landed_started_at >= self.config.landed_hold_s
        self.landed_started_at = None
        return False

    def _is_level(self, sensor: SensorSample) -> bool:
        """Проверяет, находится ли корпус в допустимом горизонтальном положении."""
        if sensor.roll_deg is None or sensor.pitch_deg is None:
            return False
        return (
            abs(sensor.roll_deg - self.config.target_roll_deg) <= self.config.level_roll_tolerance_deg
            and abs(sensor.pitch_deg - self.config.target_pitch_deg) <= self.config.level_pitch_tolerance_deg
        )

    def relative_altitude(self, sensor: SensorSample | None) -> float | None:
        """Возвращает высоту относительно точки ARM для журнала и диагностики."""
        if sensor is None or sensor.altitude_m is None or self.altitude_zero_m is None:
            return None
        return sensor.altitude_m - self.altitude_zero_m

    def _result(
        self,
        frame: bytes,
        channels: list[int],
        events: list[str],
        state: LandingState = LandingState.LIVE,
        disarm_requested: bool = False,
    ) -> LandingResult:
        """Создаёт унифицированный результат для журнала и моста."""
        return LandingResult(
            output_frame=frame,
            state=state,
            events=tuple(events),
            roll_command=channels[self.config.roll_channel],
            pitch_command=channels[self.config.pitch_channel],
            throttle_command=channels[self.config.throttle_channel],
            yaw_command=channels[self.config.yaw_channel],
            disarm_requested=disarm_requested,
        )
