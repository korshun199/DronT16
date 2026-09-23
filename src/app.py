"""Запускает локальный визуальный прототип DronT16 без автопилота."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2

from src.configuration import load_config_section
from src.control.follow_config import load_follow_config
from src.control.guidance import calculate_guidance
from src.control.target_control import write_target_guidance
from src.control.target_range import RangeEstimatorConfig, TargetRangeEstimator
from src.core.state_machine import Command, Mode, TargetBox, TargetStateMachine
from src.interface.overlay import draw_overlay
from src.interface.osd_config import load_osd_config
from src.interface.web import FrameHub, start_web_preview
from src.target.tracker import TargetTracker
from src.target.verifier import TargetVerifier
from src.video.capture import VideoSource


def parse_args() -> argparse.Namespace:
    """Читает параметры источника видео из командной строки."""
    parser = argparse.ArgumentParser(description="Визуальный прототип DronT16")
    parser.add_argument("--source", default=None, help="устаревшее переопределение источника видео")
    parser.add_argument("--display", choices=("hdmi", "web", "j7", "both"), default=None,
                        help="вывод: веб-морда, HDMI, J7 или оба тестовых экрана")
    parser.add_argument("--web-host", default=None, help="устаревшее переопределение адреса веб-просмотра")
    parser.add_argument("--web-port", type=int, default=None, help="устаревшее переопределение порта веб-просмотра")
    parser.add_argument("--hdmi-x", type=int, default=0, help="X внешнего HDMI-экрана")
    parser.add_argument("--hdmi-y", type=int, default=0, help="Y внешнего HDMI-экрана")
    parser.add_argument("--fullscreen", action="store_true", default=None, help="полноэкранный вывод HDMI")
    parser.add_argument("--capture-size", type=int, default=160, help="размер центральной области захвата")
    parser.add_argument("--config", default="config/dront16.toml",
                        help="единая конфигурация DronT16")
    parser.add_argument("--osd-config", default=None,
                        help="устаревший отдельный путь OSD; по умолчанию используется --config")
    parser.add_argument("--follow-config", default=None,
                        help="устаревший отдельный путь follow; по умолчанию используется --config")
    parser.add_argument("--j7-device", default=None, help="устаревшее переопределение DRM-устройства J7")
    parser.add_argument("--control-file", default=None, help="устаревшее переопределение файла команд")
    return parser.parse_args()


def center_target(frame: object, box_size: int) -> TargetBox | None:
    """Создаёт область захвата по центру кадра без ручного рисования мышью."""
    height, width = frame.shape[:2]
    actual_size = min(box_size, width, height)
    if actual_size <= 0:
        return None
    return TargetBox(
        float(width // 2 - actual_size // 2),
        float(height // 2 - actual_size // 2),
        float(actual_size),
        float(actual_size),
    )


def read_remote_command(control_file: str) -> str | None:
    """Читает одну команду SSH-пульта и удаляет её после чтения."""
    from pathlib import Path

    path = Path(control_file)
    try:
        command = path.read_text(encoding="ascii").strip()
        path.unlink()
    except FileNotFoundError:
        return None
    except (OSError, UnicodeError) as error:
        print(f"[DronT16] Ошибка чтения SSH-команды: {error}", file=sys.stderr, flush=True)
        return None
    if command not in {"1", "2", "3"}:
        print(f"[DronT16] Неизвестная SSH-команда: {command!r}", file=sys.stderr, flush=True)
        return None
    return command


def main() -> int:
    """Запускает цикл видео, обработки команд и экранного сопровождения."""
    args = parse_args()
    try:
        video_config = load_config_section(args.config, "video")
        follow_config = load_follow_config(args.follow_config or args.config)
        osd_config = load_osd_config(args.osd_config or args.config)
    except ValueError as error:
        print(f"[DronT16] Ошибка конфигурации сопровождения: {error}", file=sys.stderr)
        return 2
    source_name = str(args.source or video_config["source"])
    display_name = str(args.display or video_config["display"])
    web_host = str(args.web_host or video_config["web_host"])
    web_port = int(args.web_port or video_config["web_port"])
    j7_device = str(args.j7_device or video_config["j7_device"])
    control_file = str(args.control_file or video_config["control_file"])
    target_state_file = follow_config.control.state_file
    receiver_config = load_config_section(args.config, "receiver")
    arm_state_file = Path(str(receiver_config.get("arm_state_file", "/tmp/dront16_arm_state")))
    fullscreen = bool(video_config["hdmi_fullscreen"] if args.fullscreen is None else args.fullscreen)
    if display_name == "auto":
        display_name = "j7" if sys.platform.startswith("linux") and Path("/proc/device-tree/model").exists() else "web"
    if display_name not in {"hdmi", "web", "j7", "both"}:
        raise ValueError("video.display должен быть auto, hdmi, web, j7 или both")
    try:
        source = VideoSource(
            source_name,
            int(video_config["camera_width"]),
            int(video_config["camera_height"]),
            float(video_config["camera_fps"]),
            str(video_config["camera_pixel_format"]),
            int(video_config["camera_index"]),
            int(video_config["camera_buffer_count"]),
            bool(video_config["zoom"]["enabled"]),
            float(video_config["zoom"]["level"]),
            float(video_config["zoom"]["center_x"]),
            float(video_config["zoom"]["center_y"]),
            bool(video_config["lens"]["undistort"]),
            str(video_config["lens"]["calibration_file"]),
            bool(video_config["image"]["flip_horizontal"]),
            bool(video_config["image"]["flip_vertical"]),
            int(video_config["image"]["rotate_deg"]),
            float(video_config["image"]["contrast"]),
            float(video_config["image"]["brightness"]),
            float(video_config["image"]["sharpness"]),
        )
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        print(f"[DronT16] Ошибка видеовхода: {error}", file=sys.stderr, flush=True)
        return 2
    machine = TargetStateMachine()
    verifier = TargetVerifier(
        follow_config.verification.min_similarity,
        follow_config.verification.max_bad_frames,
        follow_config.verification.method,
        follow_config.verification.adaptation_rate,
        follow_config.verification.foreground_margin_percent,
    ) if follow_config.verification.enabled and follow_config.verification.method != "disabled" else None
    tracker = TargetTracker(verifier, follow_config.tracker.algorithm)
    range_estimator = TargetRangeEstimator(
        RangeEstimatorConfig(
            follow_config.range.horizontal_deadband_percent,
            follow_config.range.size_change_deadband_percent,
            follow_config.range.smoothing_alpha,
            follow_config.range.control_threshold_percent,
        )
    )
    hub = FrameHub()
    display_mode = Mode.IDLE
    if display_name in ("web", "both"):
        start_web_preview(hub, web_host, web_port)
    j7_output = None
    if display_name == "j7":
        from src.interface.j7_output import J7Output
        j7_output = J7Output(
            j7_device, osd_config.output_fit, osd_config.output_scale_x,
            osd_config.output_scale_y, osd_config.output_offset_x,
            osd_config.output_offset_y, osd_config.video_standard,
        )
        print(
            f"[DronT16] Видеотракт: камера={source.describe()} | "
            f"J7={j7_output.mode_name} ({osd_config.video_standard})",
            flush=True,
        )
        if source.width != j7_output.width or source.height != j7_output.height:
            print(
                "[DronT16] ИНФОРМАЦИЯ: размеры камеры и J7 различаются; "
                "кадр будет приведён к геометрии аналогового выхода.",
                file=sys.stderr, flush=True,
            )
    if display_name in ("hdmi", "both"):
        cv2.namedWindow("DronT16", cv2.WINDOW_NORMAL)
        cv2.moveWindow("DronT16", args.hdmi_x, args.hdmi_y)
        if fullscreen:
            cv2.setWindowProperty("DronT16", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    message = "LIVE"
    started_at = time.monotonic()
    last_range_report = 0.0
    last_reported_capture = False
    last_under_control = False
    last_terminal_report = 0.0

    try:
        while True:
            frame = source.read()
            if machine.mode in (Mode.CAPTURE, Mode.TRACKING) and machine.target is not None:
                updated_target = tracker.update(frame)
                machine.update_target(updated_target)
                if updated_target is None:
                    message = "TARGET LOST: SELECT AGAIN AND PRESS 1"
            if machine.mode is Mode.LOST:
                display_mode = Mode.LOST
            # На J7 оставляем только графику: рамку и линию к цели.
            # Пурпурный цвет включается после подтверждённого достижения порога.
            overlay_mode = Mode.CONTROL if last_under_control else display_mode
            overlay_message = "" if display_name == "j7" else message
            rendered = draw_overlay(frame, overlay_mode, machine.target, overlay_message, osd_config)
            hub.update(rendered, display_mode, machine.target, message)
            if j7_output is not None:
                j7_output.write(rendered)
            key = -1
            if display_name in ("hdmi", "both"):
                cv2.imshow("DronT16", rendered)
                key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            web_command = hub.next_command()
            remote_command = read_remote_command(control_file)
            # На временном SSH-пульте ноутбука используются три положения:
            # 1 — свободный режим, 2 — захват, 3 — сопровождение.
            key_command = {ord("1"): Command.ABORT, ord("2"): Command.CAPTURE,
                           ord("3"): Command.FOLLOW}.get(key)
            remote_key_command = ord(remote_command) if remote_command is not None else None
            remote_command_value = (
                {ord("1"): Command.ABORT, ord("2"): Command.CAPTURE,
                 ord("3"): Command.FOLLOW}.get(remote_key_command)
                if remote_key_command is not None else None
            )
            command = (web_command if web_command is not None else
                       remote_command_value if remote_command_value is not None else key_command)
            if command == Command.CAPTURE or command == 2:
                selected = center_target(frame, osd_config.capture_box_size)
                result = machine.handle(Command.CAPTURE, selected)
                if result.accepted and selected is not None:
                    try:
                        tracker.start(frame, selected)
                    except (RuntimeError, ValueError) as error:
                        machine.handle(Command.ABORT)
                        message = "TRACKER ERROR: TARGET RESET"
                    else:
                        message = result.message
                        range_estimator.reset()
                        message = "CAPTURE MODE"
                        print("\033[33mРЕЖИМ ЗАХВАТА\033[0m", flush=True)
                        last_reported_capture = True
                else:
                    message = result.message
                display_mode = machine.mode
            elif command == Command.FOLLOW:
                result = machine.handle(Command.FOLLOW)
                message = result.message
                display_mode = machine.mode
                if result.accepted and machine.target is not None:
                    range_estimator.reset()
            elif command == Command.ABORT:
                tracker.reset()
                range_estimator.reset()
                result = machine.handle(Command.ABORT)
                message = result.message
                display_mode = Mode.IDLE
            if machine.target is not None and machine.mode in (Mode.CAPTURE, Mode.TRACKING):
                measured_area_percent = tracker.object_area_percent(frame, machine.target)
                control_area_percent = tracker.frame_fill_percent(frame, machine.target)
                measurement = range_estimator.update(
                    machine.target, frame.shape[1], frame.shape[0], measured_area_percent,
                    control_area_percent,
                )
                now = time.monotonic()
                arm_active = False
                try:
                    arm_active = arm_state_file.read_text(encoding="ascii").strip() == "ARM"
                except (FileNotFoundError, OSError, UnicodeError):
                    arm_active = False
                if arm_active and now - last_terminal_report >= follow_config.range.report_period_ms / 1000.0:
                    if machine.mode is Mode.CAPTURE:
                        message = f"CAPTURE MODE | {measurement.horizontal_position}"
                        terminal_color = "\033[33m"
                    elif measurement.under_control:
                        message = "КОНТРОЛЬ!"
                        terminal_color = "\033[35m"
                    else:
                        message = f"FOLLOW MODE | {measurement.horizontal_position}"
                        terminal_color = "\033[31m"
                    elapsed = now - started_at
                    minutes = int(elapsed // 60)
                    seconds = elapsed % 60
                    # Пороговое событие выводится отдельной строкой один раз.
                    if not measurement.under_control or not last_under_control:
                        print(f"{terminal_color}[{minutes:02d}:{seconds:06.3f}] {message}\033[0m", flush=True)
                    last_terminal_report = now
                    last_range_report = now
                last_under_control = measurement.under_control
                guidance = calculate_guidance(
                    machine.target, frame.shape[1], frame.shape[0], follow_config
                )
                write_target_guidance(
                    target_state_file,
                    valid=True,
                    yaw_error_deg=guidance.yaw_error_deg,
                    pitch_error_deg=guidance.pitch_error_deg,
                    mode=machine.mode.value,
                )
            else:
                try:
                    arm_active = arm_state_file.read_text(encoding="ascii").strip() == "ARM"
                except (FileNotFoundError, OSError, UnicodeError):
                    arm_active = False
                now = time.monotonic()
                if arm_active and now - last_terminal_report >= follow_config.range.report_period_ms / 1000.0:
                    print(f"\033[37m[DIRECT/LIVE]\033[0m", flush=True)
                    last_terminal_report = now
                # После отбоя или потери цели мост не должен использовать старые координаты.
                write_target_guidance(
                    target_state_file,
                    valid=False,
                    yaw_error_deg=None,
                    pitch_error_deg=None,
                    mode=machine.mode.value,
                )
                last_reported_capture = False
                last_under_control = False
    finally:
        tracker.reset()
        source.close()
        if j7_output is not None:
            j7_output.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
