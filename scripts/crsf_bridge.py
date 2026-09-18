#!/usr/bin/env python3
"""Управляемый CRSF-мост Raspberry Pi между приёмником и полётником."""

from __future__ import annotations

import sys
import time
import tomllib
from pathlib import Path

# Добавляем корень проекта для запуска скрипта из каталога scripts.
PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from src.diagnostics.journal import Color, EventJournal
from src.control.landing import LandingConfig, LandingController
from src.receiver.crsf import CRSF_RC_CHANNELS_PACKED, CrsfReceiver, unpack_channels
from src.receiver.mode import ModeThresholds, ReceiverModeDecoder
from src.receiver.takeover import TakeoverConfig, TakeoverController, TakeoverState
from src.protocols.betaflight_msp_link import BetaflightMspLink, SensorSample


def load_config() -> dict[str, int | str]:
    """Загружает параметры UART-моста."""
    config_path = PROJECT_DIR / "config/bridge.toml"
    with config_path.open("rb") as config_file:
        return tomllib.load(config_file)["bridge"]


def load_takeover_config() -> dict[str, int | str]:
    """Загружает каналы и пороги управляемого перехвата."""
    config_path = PROJECT_DIR / "config/simulator_filesafe.toml"
    with config_path.open("rb") as config_file:
        return tomllib.load(config_file)["control"]


def load_bridge_sections() -> tuple[dict[str, object], dict[str, object]]:
    """Загружает отдельные настройки MSP и посадочного автомата."""
    config_path = PROJECT_DIR / "config/bridge.toml"
    with config_path.open("rb") as config_file:
        config = tomllib.load(config_file)
    return config.get("msp", {}), config.get("landing", {})


def main() -> int:
    """Передаёт RC, повторяет последний кадр при CH7 и даёт приоритет DISARM."""
    config = load_config()
    control = load_takeover_config()
    msp_config, landing_config = load_bridge_sections()
    serial_port = str(config["serial_port"])
    baudrate = int(config["baudrate"])
    frame_type = int(config["forward_frame_type"])
    timeout_s = int(config["link_timeout_ms"]) / 1000.0
    report_period_s = int(config["report_period_ms"]) / 1000.0
    command_file = Path(str(config.get("control_file", "/tmp/dront16_command")))
    simulator_command_file = Path(str(config.get("simulator_control_file", "/tmp/simulator_filesafe_command")))
    log_file = PROJECT_DIR / str(config.get("log_file", "simulator_filesafe.log"))

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
    journal = EventJournal(log_file, "BRIDGE")
    msp_link: BetaflightMspLink | None = None
    landing: LandingController | None = None
    latest_sensor: SensorSample | None = None
    last_sensor_log = 0.0

    try:
        if bool(msp_config.get("enabled", False)):
            msp_link = BetaflightMspLink(
                str(msp_config["serial_port"]),
                int(msp_config["baudrate"]),
                int(msp_config["request_period_ms"]) / 1000.0,
            )
            if bool(landing_config.get("enabled", False)):
                landing = LandingController(
                    LandingConfig(
                        roll_channel=int(landing_config["roll_channel"]) - 1,
                        pitch_channel=int(landing_config["pitch_channel"]) - 1,
                        throttle_channel=int(landing_config["throttle_channel"]) - 1,
                        rc_center=int(landing_config["rc_center"]),
                        rc_min=int(landing_config["rc_min"]),
                        rc_max=int(landing_config["rc_max"]),
                        target_roll_deg=float(landing_config["target_roll_deg"]),
                        target_pitch_deg=float(landing_config["target_pitch_deg"]),
                        correction_per_degree=float(landing_config["correction_per_degree"]),
                        max_correction=int(landing_config["max_correction"]),
                        descent_start_after_s=float(landing_config["descent_start_after_s"]),
                        throttle_step_per_s=float(landing_config["throttle_step_per_s"]),
                        throttle_min=int(landing_config["throttle_min"]),
                        sensor_max_age_s=int(msp_config["sensor_max_age_ms"]) / 1000.0,
                        landed_altitude_m=float(landing_config["landed_altitude_m"]),
                        landed_vario_abs_m_s=float(landing_config["landed_vario_abs_m_s"]),
                        landed_hold_s=float(landing_config["landed_hold_s"]),
                        fault_action=str(landing_config["fault_action"]),
                    )
                )
    except (OSError, ValueError, KeyError, TypeError) as error:
        journal.write("ERROR", f"MSP/landing config: {error}", Color.RED)
        journal.close()
        return 1

    def send_simulator_event(event: str) -> None:
        """Передаёт текстовое событие симулятору failsafe."""
        simulator_command_file.write_text(event, encoding="ascii")

    def log_controller_events(events: tuple[str, ...]) -> None:
        """Пишет решения state machine в общий журнал."""
        for event in events:
            color = Color.RED if "DISARM" in event else Color.YELLOW if "TAKEOVER" in event else Color.WHITE
            journal.write("DECISION", event, color)

    journal.write("START", f"UART={serial_port} baud={baudrate}; управляемый мост активен", Color.CYAN)
    journal.write(
        "CONFIG",
        f"CH{loss_channel + 1}=LINK_LOST CH{disarm_channel + 1}=DISARM "
        f"ARM>={arm_active_min} DISARM<={disarm_active_max}",
        Color.CYAN,
    )
    journal.write("PORT", "RX CRSF -> bridge -> FC UART1; реальные RC-кадры изменяются только для DISARM", Color.BLUE)
    if msp_link is not None:
        journal.write("PORT", f"MSP SENSOR={msp_config['serial_port']} baud={msp_config['baudrate']}", Color.BLUE)
        journal.write("CONFIG", f"LANDING={'ON' if landing is not None else 'OFF'}; FC PID сохраняется", Color.CYAN)
    else:
        journal.write("PORT", "MSP SENSOR отключён; используется только CRSF-мост", Color.YELLOW)
    print("[CRSF BRIDGE] Ctrl+C — остановка передачи", flush=True)

    try:
        with CrsfReceiver(serial_port, baudrate, write_enabled=True) as bridge_uart:
            while True:
                if msp_link is not None:
                    for sample in msp_link.poll():
                        latest_sensor = sample
                        now_sensor = time.monotonic()
                        if now_sensor - last_sensor_log >= 0.1 and sample.complete:
                            journal.write(
                                "SENSOR",
                                f"altitude={sample.altitude_m:.2f}m vario={sample.vario_m_s:.2f}m/s "
                                f"roll={sample.roll_deg:.1f}deg pitch={sample.pitch_deg:.1f}deg yaw={sample.yaw_deg:.1f}deg",
                                Color.MAGENTA,
                            )
                            last_sensor_log = now_sensor
                for frame in bridge_uart.read_raw_frames(buffer):
                    received_frames += 1
                    if frame[2] != frame_type or frame_type != CRSF_RC_CHANNELS_PACKED:
                        continue
                    channels = unpack_channels(frame[3:-1])
                    now = time.monotonic()
                    previous_state = takeover.state
                    selected_mode, changed = mode_decoder.update(channels[mode_channel])
                    if changed:
                        mode_command = {"DIRECT": "1", "CAPTURE": "2", "FOLLOW": "3"}[selected_mode.value]
                        command_file.write_text(mode_command, encoding="ascii")
                        journal.write(
                            "MODE",
                            f"CH{mode_channel + 1}={channels[mode_channel]} -> {mode_command} {selected_mode.value}",
                            Color.CYAN,
                        )

                    result = takeover.process(frame, channels, now)
                    log_controller_events(result.events)
                    if result.link_lost != last_link_lost:
                        journal.write(
                            "INPUT",
                            f"CH{loss_channel + 1}={channels[loss_channel]} -> "
                            f"{'LINK_LOST' if result.link_lost else 'LINK_OK'}",
                            Color.YELLOW if result.link_lost else Color.GREEN,
                        )
                        last_link_lost = result.link_lost
                    if result.disarmed != last_disarmed:
                        journal.write(
                            "INPUT",
                            f"CH{disarm_channel + 1}={channels[disarm_channel]} -> "
                            f"{'DISARM' if result.disarmed else 'ARM'}",
                            Color.RED if result.disarmed else Color.GREEN,
                        )
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
                    landing_result = None
                    if landing is not None and result.output_kind != "DISARM":
                        landing_result = landing.process(
                            output_frame,
                            unpack_channels(output_frame[3:-1]),
                            result.state.value,
                            latest_sensor,
                            now,
                        )
                        for event in landing_result.events:
                            category = "SAFETY" if "FAULT" in event else "DECISION"
                            color = Color.RED if category == "SAFETY" else Color.YELLOW
                            journal.write(category, event, color)
                        output_frame = landing_result.output_frame
                        if landing_result.state.value != "LIVE":
                            journal.write(
                                "COMMAND",
                                f"FC RC roll={landing_result.roll_command} pitch={landing_result.pitch_command} "
                                f"throttle={landing_result.throttle_command} state={landing_result.state.value}",
                                Color.RED,
                            )
                    bridge_uart.write_frame(output_frame)
                    forwarded_frames += 1
                    last_rc_time = now
                    if result.output_kind == "THROTTLE_RAMP":
                        frozen_frames += 1
                        output_channels = unpack_channels(result.output_frame[3:-1])
                        journal.write(
                            "THROTTLE RAMP",
                            f"frame={frozen_frames} CH{throttle_channel + 1}={output_channels[throttle_channel]} "
                            f"target={throttle_zero} state={result.state.value}",
                            Color.RED,
                        )
                    elif result.output_kind == "DISARM":
                        journal.write("DISARM", "FC TX: CH5 DISARM; MOTORS OFF; live channel restored", Color.RED)
                    else:
                        live_frames += 1
                    journal.write(
                        "FORWARD",
                        f"kind={result.output_kind} received={received_frames} forwarded={forwarded_frames}",
                        Color.BLUE,
                    )
                now = time.monotonic()
                if now - last_report >= report_period_s:
                    link = "OK" if last_rc_time and now - last_rc_time <= timeout_s else "LOST"
                    journal.write(
                        "STATUS",
                        f"link={link} state={takeover.state.value} received={received_frames} "
                        f"forwarded={forwarded_frames} live={live_frames} frozen={frozen_frames} "
                        f"bytes={bridge_uart.bytes_received}",
                        Color.GREEN if link == "OK" else Color.RED,
                    )
                    last_report = now
    except KeyboardInterrupt:
        journal.write("STOP", "Мост остановлен оператором", Color.YELLOW)
        return 0
    except (OSError, ValueError, KeyError, tomllib.TOMLDecodeError, RuntimeError) as error:
        journal.write("ERROR", str(error), Color.RED)
        return 1
    finally:
        if msp_link is not None:
            msp_link.close()
        journal.close()


if __name__ == "__main__":
    raise SystemExit(main())
