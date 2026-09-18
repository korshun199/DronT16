"""Запускает локальный визуальный прототип DronT16 без автопилота."""

from __future__ import annotations

import argparse
import sys
import time

import cv2

from src.control.follow_config import load_follow_config
from src.control.guidance import calculate_guidance
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
    parser.add_argument("--source", default="0", help="индекс камеры или путь к видеофайлу")
    parser.add_argument("--display", choices=("hdmi", "web", "j7", "both"), default="hdmi",
                        help="вывод: веб-морда, HDMI, J7 или оба тестовых экрана")
    parser.add_argument("--web-host", default="127.0.0.1", help="адрес веб-просмотра")
    parser.add_argument("--web-port", type=int, default=8080, help="порт веб-просмотра")
    parser.add_argument("--hdmi-x", type=int, default=0, help="X внешнего HDMI-экрана")
    parser.add_argument("--hdmi-y", type=int, default=0, help="Y внешнего HDMI-экрана")
    parser.add_argument("--fullscreen", action="store_true", help="полноэкранный вывод HDMI")
    parser.add_argument("--capture-size", type=int, default=160, help="размер центральной области захвата")
    parser.add_argument("--osd-config", default="config/osd.toml",
                        help="конфигурация размеров и оформления OSD")
    parser.add_argument("--follow-config", default="config/follow.toml",
                        help="конфигурация модуля сопровождения")
    parser.add_argument("--j7-device", default="/dev/dri/by-path/platform-1f00144000.vec-card",
                        help="DRM-устройство композитного J7")
    parser.add_argument("--control-file", default="/tmp/dront16_command",
                        help="файл команд временного SSH-пульта")
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
        follow_config = load_follow_config(args.follow_config)
        osd_config = load_osd_config(args.osd_config)
    except ValueError as error:
        print(f"[DronT16] Ошибка конфигурации сопровождения: {error}", file=sys.stderr)
        return 2
    source = VideoSource(args.source)
    machine = TargetStateMachine()
    verifier = TargetVerifier(
        follow_config.verification.min_similarity,
        follow_config.verification.max_bad_frames,
    ) if follow_config.verification.enabled else None
    tracker = TargetTracker(verifier)
    hub = FrameHub()
    display_mode = Mode.IDLE
    if args.display in ("web", "both"):
        start_web_preview(hub, args.web_host, args.web_port)
    j7_output = None
    if args.display == "j7":
        from src.interface.j7_output import J7Output
        j7_output = J7Output(
            args.j7_device, osd_config.output_fit, osd_config.output_scale_x,
            osd_config.output_scale_y, osd_config.output_offset_x,
            osd_config.output_offset_y,
        )
    if args.display in ("hdmi", "both"):
        cv2.namedWindow("DronT16", cv2.WINDOW_NORMAL)
        cv2.moveWindow("DronT16", args.hdmi_x, args.hdmi_y)
        if args.fullscreen:
            cv2.setWindowProperty("DronT16", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    message = "1: DIRECT | 2: CAPTURE | 3: FOLLOW | Q: EXIT"
    last_report = 0.0

    def report_guidance(frame: object, target: TargetBox, color: str, force: bool = False) -> None:
        """Печатает только координаты цели заданным цветом."""
        nonlocal last_report
        now = time.monotonic()
        if not force and now - last_report < follow_config.guidance.report_period_ms / 1000.0:
            return
        result = calculate_guidance(target, frame.shape[1], frame.shape[0], follow_config)
        reset = "\033[0m"
        print(
            f"{color}[TARGET] x={result.target_x:.1f}px y={result.target_y:.1f}px "
            f"norm=({result.normalized_x:+.3f},{result.normalized_y:+.3f}) "
            f"angle yaw={result.yaw_error_deg:+.1f}deg pitch={result.pitch_error_deg:+.1f}deg{reset}",
            flush=True,
        )
        last_report = now

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
            rendered = draw_overlay(frame, display_mode, machine.target, message, osd_config)
            hub.update(rendered, display_mode, machine.target, message)
            if j7_output is not None:
                j7_output.write(rendered)
            key = -1
            if args.display in ("hdmi", "both"):
                cv2.imshow("DronT16", rendered)
                key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            web_command = hub.next_command()
            remote_command = read_remote_command(args.control_file)
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
                        report_guidance(frame, selected, "\033[32m", force=True)
                else:
                    message = result.message
                display_mode = machine.mode
            elif command == Command.FOLLOW:
                result = machine.handle(Command.FOLLOW)
                message = result.message
                display_mode = machine.mode
                if result.accepted and machine.target is not None:
                    report_guidance(frame, machine.target, "\033[31m", force=True)
            elif command == Command.ABORT:
                tracker.reset()
                result = machine.handle(Command.ABORT)
                message = result.message
                display_mode = Mode.IDLE
            if machine.target is not None and machine.mode in (Mode.CAPTURE, Mode.TRACKING):
                color = "\033[32m" if machine.mode is Mode.CAPTURE else "\033[31m"
                report_guidance(frame, machine.target, color)
    finally:
        tracker.reset()
        source.close()
        if j7_output is not None:
            j7_output.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
