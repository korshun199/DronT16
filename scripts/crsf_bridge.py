#!/usr/bin/env python3
"""Управляемый CRSF-мост Raspberry Pi между приёмником и полётником."""

from __future__ import annotations

import sys
import signal
import time
from pathlib import Path

# Добавляем корень проекта для запуска скрипта из каталога scripts.
PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from src.diagnostics.journal import Color, EventJournal
from src.configuration import load_config_section
from src.control.failsafe import FailsafeConfig, FailsafeController
from src.control.target_control import TargetGuidance, read_target_guidance
from src.control.visual_servoing import (
    VisualServoController,
    VisualServoState,
    build_visual_servo_config,
)
from src.receiver.crsf import (
    CRSF_LINK_STATISTICS,
    CRSF_RC_CHANNELS_PACKED,
    CrsfReceiver,
    parse_link_statistics,
    unpack_channels,
)
from src.receiver.mode import ModeThresholds, ReceiverModeDecoder
from src.receiver.takeover import TakeoverConfig, TakeoverController, TakeoverState
from src.protocols.betaflight_msp_link import BetaflightMspLink, SensorSample, msp_command_name
from src.protocols.msp_displayport import DisplayPortCanvas, MspDisplayPortLink, MspDisplayPortWorker


def load_config() -> dict[str, int | str]:
    """Загружает параметры UART-моста."""
    return load_config_section(PROJECT_DIR / "config/dront16.toml", "bridge")


def load_takeover_config() -> dict[str, int | str]:
    """Загружает каналы и пороги управляемого перехвата."""
    return load_config_section(PROJECT_DIR / "config/dront16.toml", "simulation", "control")


def load_bridge_sections() -> tuple[dict[str, object], dict[str, object]]:
    """Загружает отдельные настройки MSP и удержания после потери связи."""
    config_path = PROJECT_DIR / "config/dront16.toml"
    return load_config_section(config_path, "msp"), load_config_section(config_path, "failsafe")


def load_target_control_config() -> dict[str, object]:
    """Загружает общий файл координат захваченной цели."""
    return load_config_section(PROJECT_DIR / "config/dront16.toml", "follow", "control")


def load_visual_servo_config() -> dict[str, object]:
    """Загружает параметры внешнего контура сопровождения цели."""
    return load_config_section(PROJECT_DIR / "config/dront16.toml", "visual_servoing")


def load_logging_config() -> dict[str, object]:
    """Загружает независимые настройки консольного и файлового каналов."""
    return load_config_section(PROJECT_DIR / "config/dront16.toml", "logging")


def load_betaflight_osd_config() -> dict[str, object]:
    """Загружает безопасно отключаемую секцию приёма MSP DisplayPort."""
    osd_config = load_config_section(PROJECT_DIR / "config/dront16.toml", "osd")
    raw = osd_config.get("betaflight", {})
    if not isinstance(raw, dict):
        raise ValueError("osd.betaflight должна быть TOML-секцией")
    return raw


def category_set(value: object) -> set[str] | None:
    """Проверяет список категорий журнала и приводит его к верхнему регистру."""
    if value is None:
        return None
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ValueError("Категории журнала должны быть TOML-массивом строк")
    return {item.upper() for item in value}


def main() -> int:
    """Передаёт RC, повторяет последний кадр при CH7 и даёт приоритет DISARM."""
    config = load_config()
    # systemd останавливает службу сигналом SIGTERM; переводим его в общий
    # путь завершения, чтобы текущий ARM-журнал успел fsync и закрылся.
    def stop_on_term(_signum: int, _frame: object) -> None:
        """Передаёт SIGTERM в общий обработчик корректной остановки моста."""
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop_on_term)
    control = load_takeover_config()
    msp_config, failsafe_config = load_bridge_sections()
    target_config = load_target_control_config()
    visual_config = load_visual_servo_config()
    logging_config = load_logging_config()
    betaflight_osd_config = load_betaflight_osd_config()
    receiver_config = load_config_section(PROJECT_DIR / "config/dront16.toml", "receiver")
    mode_config = load_config_section(PROJECT_DIR / "config/dront16.toml", "receiver", "mode")
    failsafe_mode = str(failsafe_config.get("mode", "CH7")).upper()
    if failsafe_mode not in {"CH7", "REAL"}:
        raise ValueError("failsafe.mode должен быть CH7 или real")
    # Когда модуль Raspberry выключен, потеря RC должна дойти до Betaflight
    # без заморозки кадра: FC тогда запускает свой резервный AUTO-LAND.
    rpi_failsafe_enabled = bool(failsafe_config.get("enabled", False))
    serial_port = str(receiver_config["serial_port"])
    baudrate = int(receiver_config["baudrate"])
    frame_type = int(config["forward_frame_type"])
    forward_link_statistics = bool(config.get("forward_link_statistics", True))
    timeout_s = int(receiver_config["link_timeout_ms"]) / 1000.0
    report_period_s = int(config["report_period_ms"]) / 1000.0
    verbose_frames = bool(config.get("verbose_frames", False))
    sensor_terminal = bool(config.get("sensor_terminal", False))
    # При полном диагностическом режиме каждый корректный MSP-ответ попадает в файл.
    sensor_log_every_packet = bool(config.get("sensor_log_every_packet", True))
    sensor_log_period_s = int(config.get("sensor_log_period_ms", 100)) / 1000.0
    pilot_log_period_s = int(config.get("pilot_log_period_ms", 500)) / 1000.0
    command_log_period_s = int(config.get("command_log_period_ms", 100)) / 1000.0
    rc_gap_log_ms = int(config.get("rc_gap_log_ms", 100))
    command_file = Path(str(config.get("control_file", "/tmp/dront16_command")))
    simulator_command_file = Path(str(config.get("simulator_control_file", "/tmp/simulator_filesafe_command")))
    target_state_file = Path(str(target_config.get("target_state_file", "/tmp/dront16_target.json")))
    target_max_age_s = int(target_config.get("target_max_age_ms", 300)) / 1000.0
    arm_state_file = Path(str(receiver_config.get("arm_state_file", "/tmp/dront16_arm_state")))
    log_file = PROJECT_DIR / str(config.get("log_file", "simulator_filesafe.log"))
    console_categories = category_set(logging_config.get("console_categories"))
    file_categories = category_set(logging_config.get("file_categories"))
    displayport_enabled = bool(betaflight_osd_config.get("enabled", False))
    displayport_state_file = Path(str(
        betaflight_osd_config.get("state_file", "/tmp/dront16_betaflight_osd.json")
    ))
    displayport_canvas = DisplayPortCanvas(
        int(betaflight_osd_config.get("columns", 30)),
        int(betaflight_osd_config.get("rows", 13)),
    )
    displayport_port = str(betaflight_osd_config.get("serial_port", "/dev/ttyAMA4"))
    displayport_baudrate = int(betaflight_osd_config.get("baudrate", 115200))
    displayport_report_period_s = int(betaflight_osd_config.get("diagnostic_report_ms", 2000)) / 1000.0

    mode_channel = int(receiver_config["mode_channel"]) - 1
    mode_decoder = ReceiverModeDecoder(
        ModeThresholds(
            int(mode_config["low_max"]),
            int(mode_config["high_min"]),
            int(mode_config["debounce_frames"]),
        )
    )

    loss_channel = int(control["loss_channel"]) - 1
    link_ok_max = int(control["link_ok_max"])
    link_lost_min = int(control["link_lost_min"])
    disarm_channel = int(control["disarm_channel"]) - 1
    disarm_active_max = int(control["disarm_active_max"])
    arm_active_min = int(control["arm_active_min"])
    throttle_channel = int(control["throttle_channel"]) - 1
    throttle_zero = int(control["throttle_zero"])
    throttle_ramp_s = float(control["throttle_ramp_s"])
    if (
        not 0 <= mode_channel < 16
        or not 0 <= loss_channel < 16
        or not 0 <= disarm_channel < 16
        or not 0 <= throttle_channel < 16
        or not 0 <= link_ok_max < link_lost_min <= 2047
        or not 0 <= disarm_active_max < arm_active_min <= 2047
        or not 0 <= throttle_zero <= 2047
        or throttle_ramp_s < 0
        or pilot_log_period_s <= 0
        or command_log_period_s <= 0
        or rc_gap_log_ms <= 0
        or sensor_log_period_s <= 0
        or target_max_age_s <= 0
    ):
        raise ValueError("Неверные параметры каналов simulator_filesafe")
    takeover = TakeoverController(
        TakeoverConfig(
            loss_channel=loss_channel,
            link_lost_min=link_lost_min,
            link_ok_max=link_ok_max,
            disarm_channel=disarm_channel,
            disarm_active_max=disarm_active_max,
            arm_active_min=arm_active_min,
            throttle_channel=throttle_channel,
            throttle_zero=throttle_zero,
            throttle_ramp_s=throttle_ramp_s,
            # При активном failsafe газ меняет только failsafe.py.
            ramp_throttle=not bool(failsafe_config.get("enabled", False)),
        )
    )
    last_link_lost: bool | None = None
    last_disarmed: bool | None = None
    buffer = bytearray()
    last_rc_time = 0.0
    last_report = 0.0
    received_frames = 0
    forwarded_frames = 0
    live_frames = 0
    frozen_frames = 0
    link_statistics_forwarded = 0
    last_link_statistics_log = 0.0
    journal = EventJournal(
        log_file,
        "BRIDGE",
        console_enabled=bool(logging_config.get("console_enabled", True)),
        file_enabled=bool(logging_config.get("file_enabled", True)),
        console_categories=console_categories,
        file_categories=file_categories,
        # Полная телеметрия идёт в очередь: запись на SD-карту не имеет права
        # задерживать выдачу очередного CRSF-кадра на UART0.
        asynchronous_file_write=True,
    )
    msp_link: BetaflightMspLink | None = None
    displayport_link: MspDisplayPortLink | None = None
    displayport_worker: MspDisplayPortWorker | None = None
    failsafe: FailsafeController | None = None
    visual_servo: VisualServoController | None = None
    latest_sensor: SensorSample | None = None
    last_sensor_log = 0.0
    last_repetitive_event_log = 0.0
    last_command_log = 0.0
    last_pilot_log = 0.0
    last_pilot_action_values: tuple[int, ...] | None = None
    last_rc_interval_ms: float | None = None
    rpi_timeout_reported = False

    # До первого свежего кадра считаем терминальный диагностический вывод неактивным.
    try:
        arm_state_file.write_text("DISARM", encoding="ascii")
    except OSError as error:
        journal.write("RPI", f"ARM STATE WARNING: {error}", Color.YELLOW, file=False)

    try:
        visual_servo = VisualServoController(
            build_visual_servo_config(
                visual_config,
                target_max_age_s=target_max_age_s,
                sensor_max_age_s=int(msp_config["sensor_max_age_ms"]) / 1000.0,
            )
        )
        if bool(msp_config.get("enabled", False)):
            msp_link = BetaflightMspLink(
                str(msp_config["serial_port"]),
                int(msp_config["baudrate"]),
                int(msp_config["request_period_ms"]) / 1000.0,
                request_gps=bool(msp_config.get("gps_enabled", False)),
            )
            if bool(failsafe_config.get("enabled", False)):
                failsafe = FailsafeController(
                    FailsafeConfig(
                        roll_channel=int(failsafe_config["roll_channel"]) - 1,
                        pitch_channel=int(failsafe_config["pitch_channel"]) - 1,
                        throttle_channel=int(failsafe_config["throttle_channel"]) - 1,
                        yaw_channel=int(failsafe_config["yaw_channel"]) - 1,
                        rc_center=int(failsafe_config["rc_center"]),
                        rc_min=int(failsafe_config["rc_min"]),
                        rc_max=int(failsafe_config["rc_max"]),
                        target_roll_deg=float(failsafe_config["target_roll_deg"]),
                        target_pitch_deg=float(failsafe_config["target_pitch_deg"]),
                        correction_per_degree=float(failsafe_config["correction_per_degree"]),
                        max_correction=int(failsafe_config["max_correction"]),
                        sensor_max_age_s=int(msp_config["sensor_max_age_ms"]) / 1000.0,
                        fault_action=str(failsafe_config["fault_action"]),
                        disarm_channel=int(failsafe_config["disarm_channel"]) - 1,
                        disarm_value=int(failsafe_config["disarm_value"]),
                        stabilization_channel=(
                            int(failsafe_config["stabilization_channel"]) - 1
                            if int(failsafe_config["stabilization_channel"]) > 0
                            else None
                        ),
                        stabilization_value=int(failsafe_config["stabilization_value"]),
                        level_roll_tolerance_deg=float(failsafe_config["level_roll_tolerance_deg"]),
                        level_pitch_tolerance_deg=float(failsafe_config["level_pitch_tolerance_deg"]),
                        level_hold_s=float(failsafe_config["level_hold_s"]),
                        level_throttle=int(failsafe_config["level_throttle"]),
                        altitude_hold_gain=float(failsafe_config["altitude_hold_gain"]),
                        altitude_hold_integral_gain=float(failsafe_config["altitude_hold_integral_gain"]),
                        altitude_hold_vario_gain=float(failsafe_config["altitude_hold_vario_gain"]),
                        max_altitude_correction=int(failsafe_config["max_altitude_correction"]),
                        max_altitude_integral_correction=int(failsafe_config["max_altitude_integral_correction"]),
                        turn_enabled=bool(failsafe_config["turn_enabled"]),
                        turn_degrees=float(failsafe_config["turn_degrees"]),
                        turn_yaw_command=int(failsafe_config["turn_yaw_command"]),
                        takeover_throttle_step_per_s=float(failsafe_config["takeover_throttle_step_per_s"]),
                        climb_guard_altitude_error_m=float(failsafe_config["climb_guard_altitude_error_m"]),
                        climb_guard_vario_m_s=float(failsafe_config["climb_guard_vario_m_s"]),
                        climb_guard_max_throttle=int(failsafe_config["climb_guard_max_throttle"]),
                    )
                )
        if displayport_enabled:
            displayport_link = MspDisplayPortLink(
                displayport_port, displayport_baudrate, displayport_canvas, displayport_state_file
            )
            displayport_worker = MspDisplayPortWorker(
                displayport_link,
                displayport_report_period_s,
                status_callback=lambda status: print(
                    f"[DronT16] OSD UART {displayport_port}: {status}", flush=True
                ),
            )
            displayport_worker.start()
            journal.write(
                "RPI",
                f"OSD DISPLAYPORT: UART={displayport_port} {displayport_baudrate} бод; "
                f"сетка={displayport_canvas.columns}x{displayport_canvas.rows}",
                Color.CYAN,
            )
    except (OSError, ValueError, KeyError, TypeError) as error:
        journal.write("RPI", f"ERROR: MSP/failsafe config: {error}", Color.RED)
        journal.close()
        return 1

    def send_simulator_event(event: str) -> None:
        """Передаёт текстовое событие симулятору failsafe."""
        simulator_command_file.write_text(event, encoding="ascii")

    def log_controller_events(events: tuple[str, ...], now: float) -> None:
        """Пишет решения автомата, ограничивая только повторяющийся шум."""
        nonlocal last_repetitive_event_log
        for event in events:
            # Повторяющиеся строки на каждый кадр не должны задерживать UART.
            # Переходы, DISARM, FAULT и ошибки всегда записываются сразу.
            repetitive = event.startswith(("RC SUPPRESS", "RC TIMEOUT"))
            if repetitive and now - last_repetitive_event_log < 0.5:
                continue
            color = Color.RED if "DISARM" in event else Color.YELLOW if "TAKEOVER" in event else Color.WHITE
            journal.write("RPI", f"DECISION: {event}", color)
            if repetitive:
                last_repetitive_event_log = now

    def format_channels(channels: tuple[int, ...] | list[int]) -> str:
        """Формирует единый полный снимок 16 RC-каналов для журнала."""
        return " ".join(f"CH{index + 1}={value}" for index, value in enumerate(channels))

    def format_sensor_sample(sample: SensorSample, relative_text: str) -> str:
        """Формирует полную строку всех полей, доступных из текущего MSP-парсера."""
        command_name = msp_command_name(sample.updated_command)
        altitude = "NA" if sample.altitude_m is None else f"{sample.altitude_m:.3f}m"
        vario = "NA" if sample.vario_m_s is None else f"{sample.vario_m_s:.3f}m/s"
        attitude = " ".join(
            f"{name}={'NA' if value is None else f'{value:.3f}deg'}"
            for name, value in (
                ("roll", sample.roll_deg),
                ("pitch", sample.pitch_deg),
                ("yaw", sample.yaw_deg),
            )
        )
        acc = " ".join(
            f"{name}={'NA' if value is None else value}"
            for name, value in (
                ("acc_x", sample.acc_x),
                ("acc_y", sample.acc_y),
                ("acc_z", sample.acc_z),
            )
        )
        gyro = " ".join(
            f"{name}={'NA' if value is None else value}"
            for name, value in (
                ("gyro_x", sample.gyro_x),
                ("gyro_y", sample.gyro_y),
                ("gyro_z", sample.gyro_z),
            )
        )
        mag = " ".join(
            f"{name}={'NA' if value is None else value}"
            for name, value in (
                ("mag_x", sample.mag_x),
                ("mag_y", sample.mag_y),
                ("mag_z", sample.mag_z),
            )
        )
        heading = "NA" if sample.magnetic_heading_deg is None else f"{sample.magnetic_heading_deg:.3f}deg"
        gps = (
            f"gps_fix={sample.gps_fix if sample.gps_fix is not None else 'NA'} "
            f"gps_sat={sample.gps_satellites if sample.gps_satellites is not None else 'NA'} "
            f"gps_lat={sample.gps_latitude_deg if sample.gps_latitude_deg is not None else 'NA'} "
            f"gps_lon={sample.gps_longitude_deg if sample.gps_longitude_deg is not None else 'NA'} "
            f"gps_alt={sample.gps_altitude_m if sample.gps_altitude_m is not None else 'NA'}m "
            f"gps_speed={sample.gps_speed_m_s if sample.gps_speed_m_s is not None else 'NA'}m/s "
            f"gps_course={sample.gps_course_deg if sample.gps_course_deg is not None else 'NA'}deg "
            f"gps_hdop={sample.gps_hdop if sample.gps_hdop is not None else 'NA'}"
        )
        return (
            f"MSP packet={command_name} complete={sample.complete} "
            f"altitude={altitude} vario={vario} {attitude} {acc} {gyro} {mag} "
            f"mag_heading_raw={heading} {gps}{relative_text}"
        )

    journal.write("RPI", f"START: UART={serial_port} baud={baudrate}; управляемый мост активен", Color.CYAN)
    journal.write(
        "RPI",
        f"CONFIG: CH{loss_channel + 1}=LINK_LOST CH{disarm_channel + 1}=DISARM "
        f"ARM>={arm_active_min} DISARM<={disarm_active_max} "
        f"FAILSAFE_MODE={failsafe_mode} RPI_FAILSAFE={'ON' if rpi_failsafe_enabled else 'OFF'}",
        Color.CYAN,
    )
    journal.write("RPI", "PORT: RX CRSF -> bridge -> FC UART1; реальные RC-кадры изменяются только для DISARM", Color.BLUE)
    if msp_link is not None:
        journal.write("RPI", f"PORT: MSP SENSOR={msp_config['serial_port']} baud={msp_config['baudrate']}", Color.BLUE)
        journal.write("RPI", f"CONFIG: FAILSAFE={'ON' if failsafe is not None else 'OFF'}; FC PID сохраняется", Color.CYAN)
    else:
        journal.write("RPI", "PORT: MSP SENSOR отключён; используется только CRSF-мост", Color.YELLOW)
    if visual_servo is not None:
        journal.write(
            "RPI",
            f"CONFIG: VISUAL_SERVO={'ON' if visual_servo.config.enabled else 'OFF'} "
            f"output={visual_servo.config.output_mode}",
            Color.CYAN,
        )
    print("[CRSF BRIDGE] Ctrl+C — остановка передачи", flush=True)

    try:
        with CrsfReceiver(serial_port, baudrate, write_enabled=True) as bridge_uart:
            while True:
                if msp_link is not None:
                    for sample in msp_link.poll():
                        latest_sensor = sample
                        now_sensor = time.monotonic()
                        if (
                            sensor_log_every_packet
                            or now_sensor - last_sensor_log >= sensor_log_period_s
                        ):
                            relative_text = ""
                            if failsafe is not None:
                                relative_altitude = failsafe.relative_altitude(sample)
                                if relative_altitude is not None:
                                    relative_text = f" relative_altitude={relative_altitude:.2f}m"
                            journal.write(
                                "FC",
                                format_sensor_sample(sample, relative_text),
                                Color.MAGENTA,
                                console=sensor_terminal,
                            )
                            last_sensor_log = now_sensor
                    # Этот UART выделен только под датчики; освобождаем
                    # внутренний список проверенных пакетов каждого цикла.
                    msp_link.drain_packets()
                rc_frame_seen = False
                for frame in bridge_uart.read_raw_frames(buffer):
                    received_frames += 1
                    if frame[2] != frame_type or frame_type != CRSF_RC_CHANNELS_PACKED:
                        # В LIVE служебный CRSF-кадр должен пройти к FC:
                        # Betaflight использует его для RSSI/LQ/SNR и OSD.
                        if (
                            forward_link_statistics
                            and frame[2] == CRSF_LINK_STATISTICS
                            and takeover.state is TakeoverState.LIVE
                        ):
                            bridge_uart.write_frame(frame)
                            forwarded_frames += 1
                            link_statistics_forwarded += 1
                            now = time.monotonic()
                            if now - last_link_statistics_log >= report_period_s:
                                payload = frame[3:-1]
                                if len(payload) == 10:
                                    link_stats = parse_link_statistics(payload)
                                    journal.write(
                                        "RPI",
                                        f"CRSF LINK_STATS forwarded: RSSI1={link_stats.uplink_rssi_1} "
                                        f"RSSI2={link_stats.uplink_rssi_2} "
                                        f"LQ={link_stats.uplink_link_quality}% "
                                        f"SNR={link_stats.uplink_snr}dB",
                                        Color.BLUE,
                                    )
                                last_link_statistics_log = now
                        continue
                    rc_frame_seen = True
                    channels = unpack_channels(frame[3:-1])
                    now = time.monotonic()
                    # DISARM закрывает журнал после записи последней команды FC.
                    close_journal_after_frame = False
                    if last_rc_time:
                        last_rc_interval_ms = (now - last_rc_time) * 1000.0
                        if last_rc_interval_ms >= rc_gap_log_ms:
                            journal.write(
                                "RPI",
                                f"CRSF RC GAP: interval={last_rc_interval_ms:.1f}ms "
                                f"threshold={rc_gap_log_ms}ms",
                                Color.RED,
                            )
                    # Пишем полный снимок каналов периодически, а важные тумблеры — сразу при изменении.
                    pilot_action_values = (
                        channels[mode_channel],
                        channels[loss_channel],
                        channels[disarm_channel],
                        channels[throttle_channel],
                    )
                    if (
                        last_pilot_action_values != pilot_action_values
                        or now - last_pilot_log >= pilot_log_period_s
                    ):
                        channel_text = format_channels(channels)
                        reason = "change" if last_pilot_action_values != pilot_action_values else "snapshot"
                        journal.write("PILOT", f"RC {reason}: {channel_text}", Color.GREEN)
                        last_pilot_action_values = pilot_action_values
                        last_pilot_log = now
                    previous_state = takeover.state
                    selected_mode, changed = mode_decoder.update(channels[mode_channel])
                    target_mode_active = (
                        bool(target_config.get("enabled", False))
                        and selected_mode.value == "FOLLOW"
                    )
                    target_guidance: TargetGuidance | None = (
                        read_target_guidance(target_state_file) if target_mode_active else None
                    )
                    if changed:
                        mode_command = {"DIRECT": "1", "CAPTURE": "2", "FOLLOW": "3"}[selected_mode.value]
                        command_file.write_text(mode_command, encoding="ascii")
                        journal.write(
                            "PILOT",
                            f"MODE: CH{mode_channel + 1}={channels[mode_channel]} -> {mode_command} {selected_mode.value}",
                            Color.CYAN,
                        )
                        if target_mode_active:
                            journal.write("RPI", "VISUAL_SERVO: запрошен режим FOLLOW", Color.YELLOW)

                    # В режиме real, а также при отключённом failsafe Raspberry,
                    # CH7 не может заморозить живой канал RC.
                    result = takeover.process(
                        frame,
                        channels,
                        now,
                        force_link_lost=(
                            False if not rpi_failsafe_enabled or failsafe_mode == "REAL" else None
                        ),
                    )
                    rpi_timeout_reported = False
                    log_controller_events(result.events, now)
                    if result.link_lost != last_link_lost:
                        journal.write(
                            "PILOT",
                            f"LINK SWITCH: CH{loss_channel + 1}={channels[loss_channel]} -> "
                            f"{'LINK_LOST' if result.link_lost else 'LINK_OK'}",
                            Color.YELLOW if result.link_lost else Color.GREEN,
                        )
                        last_link_lost = result.link_lost
                    if result.disarmed != last_disarmed:
                        if not result.disarmed:
                            # Новый файл испытания начинается с первого ARM.
                            journal.begin_session()
                            try:
                                arm_state_file.write_text("ARM", encoding="ascii")
                            except OSError as error:
                                journal.write("RPI", f"ARM STATE WARNING: {error}", Color.YELLOW, file=False)
                        journal.write(
                            "PILOT",
                            f"ARM SWITCH: CH{disarm_channel + 1}={channels[disarm_channel]} -> "
                            f"{'DISARM' if result.disarmed else 'ARM'}",
                            Color.RED if result.disarmed else Color.GREEN,
                        )
                        if result.disarmed:
                            close_journal_after_frame = True
                            try:
                                arm_state_file.write_text("DISARM", encoding="ascii")
                            except OSError as error:
                                journal.write("RPI", f"ARM STATE WARNING: {error}", Color.YELLOW, file=False)
                        last_disarmed = result.disarmed

                    if previous_state is TakeoverState.LIVE and result.state is TakeoverState.TAKEOVER:
                        send_simulator_event("LINK_LOST")
                    elif (
                        previous_state in (TakeoverState.TAKEOVER, TakeoverState.DISARM_RELEASE)
                        and result.state is TakeoverState.LIVE
                        and not result.link_lost
                    ):
                        send_simulator_event("LINK_OK")
                    if result.output_kind == "DISARM":
                        send_simulator_event("DISARM")

                    output_frame = result.output_frame
                    failsafe_result = None
                    if failsafe is not None and result.output_kind == "DISARM":
                        # DISARM завершает текущий ARM-цикл и очищает опорную высоту.
                        # Сам кадр DISARM уже сформирован мостом выше.
                        failsafe.process(
                            output_frame,
                            unpack_channels(output_frame[3:-1]),
                            "LIVE",
                            latest_sensor,
                            now,
                            armed=False,
                        )
                    if failsafe is not None and result.output_kind != "DISARM":
                        failsafe_result = failsafe.process(
                            output_frame,
                            unpack_channels(output_frame[3:-1]),
                            result.state.value,
                            latest_sensor,
                            now,
                            armed=not result.disarmed,
                        )
                        for event in failsafe_result.events:
                            event_kind = "SAFETY" if "FAULT" in event else "DECISION"
                            color = Color.RED if event_kind == "SAFETY" else Color.YELLOW
                            journal.write("RPI", f"{event_kind}: {event}", color)
                        if failsafe_result.state.value == "FAULT":
                            # Сбой закрывает текущую ARM-сессию; новые данные
                            # не смешиваются с этим испытательным циклом.
                            journal.write("RPI", "SESSION END: FAULT", Color.RED)
                            journal.end_session()
                        output_frame = failsafe_result.output_frame
                        if failsafe_result.state.value != "LIVE" and (
                            failsafe_result.events
                            or now - last_command_log >= command_log_period_s
                        ):
                            command_channels = unpack_channels(output_frame[3:-1])
                            journal.write(
                                "RPI",
                                f"COMMAND TO FC: kind={result.output_kind} state={failsafe_result.state.value} "
                                f"{format_channels(command_channels)}",
                                Color.RED,
                            )
                            last_command_log = now
                        if failsafe_result.disarm_requested:
                            journal.write("RPI", "COMMAND TO FC: RC CH5 DISARM; MOTORS OFF", Color.RED)
                    if visual_servo is not None:
                        visual_result = visual_servo.process(
                            output_frame,
                            tuple(unpack_channels(output_frame[3:-1])),
                            selected_mode.value,
                            target_guidance,
                            latest_sensor,
                            now,
                            armed=not result.disarmed,
                            takeover_allowed=(
                                result.state is TakeoverState.LIVE
                                and (failsafe_result is None or failsafe_result.state.value == "LIVE")
                            ),
                        )
                        output_frame = visual_result.output_frame
                        for event in visual_result.events:
                            color = Color.RED if visual_result.fault else Color.YELLOW
                            journal.write("RPI", f"VISUAL_SERVO: {event}", color)
                        if visual_result.target_lost and visual_result.events:
                            # Потерянная цель не переобнаруживается: видеомодуль
                            # возвращается в DIRECT и ждёт нового CAPTURE пилота.
                            command_file.write_text("1", encoding="ascii")
                        if visual_result.state not in {
                            VisualServoState.DIRECT,
                            VisualServoState.CAPTURE,
                        } and (visual_result.events or now - last_command_log >= command_log_period_s):
                            action = "COMMAND" if visual_result.output_applied else "DRY-RUN"
                            journal.write(
                                "RPI",
                                f"VISUAL_SERVO {action}: state={visual_result.state.value} "
                                f"{format_channels(visual_result.computed_channels)}",
                                Color.RED if visual_result.output_applied else Color.YELLOW,
                            )
                            last_command_log = now
                    bridge_uart.write_frame(output_frame)
                    forwarded_frames += 1
                    last_rc_time = now
                    if result.output_kind == "THROTTLE_RAMP":
                        frozen_frames += 1
                        output_channels = unpack_channels(result.output_frame[3:-1])
                        if verbose_frames:
                            journal.write(
                                "RPI",
                                f"frame={frozen_frames} CH{throttle_channel + 1}={output_channels[throttle_channel]} "
                                f"target={throttle_zero} state={result.state.value}",
                                Color.RED,
                            )
                    elif result.output_kind == "DISARM":
                        journal.write("RPI", "COMMAND TO FC: CH5 DISARM; MOTORS OFF; live channel restored", Color.RED)
                    else:
                        live_frames += 1
                    if verbose_frames:
                        journal.write(
                            "RPI",
                            f"kind={result.output_kind} received={received_frames} forwarded={forwarded_frames}",
                            Color.BLUE,
                        )
                    if close_journal_after_frame:
                        # Все действия DISARM уже зафиксированы, цикл завершён.
                        journal.end_session()
                # При TAKEOVER повторяем сохранённый кадр даже при временном
                # отсутствии новых байтов от приёмника.
                timeout_result = None
                if not rc_frame_seen:
                    timeout_now = time.monotonic()
                    receiver_timed_out = bool(last_rc_time and timeout_now - last_rc_time >= timeout_s)
                    if receiver_timed_out and not rpi_failsafe_enabled:
                        # Не посылаем синтетический кадр: Betaflight обязан увидеть
                        # настоящий RX LOSS и выполнить собственный AUTO-LAND.
                        if not rpi_timeout_reported:
                            journal.write(
                                "RPI",
                                "CRSF TIMEOUT: RPI failsafe OFF; передача остановлена, "
                                "FC выполняет собственный RX LOSS/AUTO-LAND",
                                Color.RED,
                            )
                            rpi_timeout_reported = True
                    elif failsafe_mode == "REAL" and receiver_timed_out:
                        timeout_result = takeover.process_receiver_timeout(timeout_now)
                        if timeout_result is not None and timeout_result.state is TakeoverState.TAKEOVER:
                            journal.write("PILOT", "CRSF TIMEOUT: реальная потеря связи; включён TAKEOVER", Color.YELLOW)
                    else:
                        timeout_result = takeover.repeat_without_receiver(timeout_now)
                if timeout_result is not None:
                    output_frame = timeout_result.output_frame
                    if visual_servo is not None:
                        visual_servo.process(
                            output_frame,
                            tuple(unpack_channels(output_frame[3:-1])),
                            selected_mode.value,
                            None,
                            latest_sensor,
                            time.monotonic(),
                            armed=True,
                            takeover_allowed=False,
                        )
                    if failsafe is not None:
                        timeout_failsafe = failsafe.process(
                            output_frame,
                            unpack_channels(output_frame[3:-1]),
                            timeout_result.state.value,
                            latest_sensor,
                            time.monotonic(),
                            armed=True,
                        )
                        for event in timeout_failsafe.events:
                            journal.write("RPI", f"{'SAFETY' if 'FAULT' in event else 'DECISION'}: {event}", Color.RED)
                        output_frame = timeout_failsafe.output_frame
                        if timeout_failsafe.disarm_requested:
                            journal.write("RPI", "COMMAND TO FC: RC CH5 DISARM; MOTORS OFF", Color.RED)
                    bridge_uart.write_frame(output_frame)
                    forwarded_frames += 1
                    frozen_frames += 1
                    timeout_now = time.monotonic()
                    if timeout_now - last_command_log >= command_log_period_s:
                        journal.write(
                            "RPI",
                            f"COMMAND TO FC: kind={timeout_result.output_kind} state={timeout_result.state.value} "
                            f"{format_channels(unpack_channels(output_frame[3:-1]))}",
                            Color.RED,
                        )
                        last_command_log = timeout_now
                    if frozen_frames % 10 == 0:
                        journal.write(
                            "RPI",
                            f"timeout повтор сохранённого кадра={frozen_frames}",
                            Color.RED,
                        )
                now = time.monotonic()
                if now - last_report >= report_period_s:
                    link = "OK" if last_rc_time and now - last_rc_time <= timeout_s else "LOST"
                    last_rc_interval_text = (
                        "n/a" if last_rc_interval_ms is None else f"{last_rc_interval_ms:.1f}"
                    )
                    journal.write(
                        "RPI",
                        f"STATUS: link={link} state={takeover.state.value} received={received_frames} "
                        f"forwarded={forwarded_frames} live={live_frames} frozen={frozen_frames} "
                        f"link_stats={link_statistics_forwarded} "
                        f"last_rc_interval_ms={last_rc_interval_text} "
                        f"bytes={bridge_uart.bytes_received}",
                        Color.GREEN if link == "OK" else Color.RED,
                    )
                    last_report = now
    except KeyboardInterrupt:
        journal.write("RPI", "STOP: мост остановлен оператором", Color.YELLOW)
        journal.end_session()
        return 0
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        journal.write("RPI", f"ERROR: {error}", Color.RED)
        journal.end_session()
        return 1
    finally:
        if msp_link is not None:
            msp_link.close()
        if displayport_worker is not None:
            displayport_worker.stop()
        elif displayport_link is not None:
            displayport_link.close()
        journal.close()


if __name__ == "__main__":
    raise SystemExit(main())
