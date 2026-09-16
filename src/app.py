"""Запускает локальный визуальный прототип DronT16 без автопилота."""

from __future__ import annotations

import argparse

import cv2

from src.core.state_machine import Command, Mode, TargetBox, TargetStateMachine
from src.interface.overlay import draw_overlay
from src.interface.web import FrameHub, start_web_preview
from src.target.tracker import TargetTracker
from src.video.capture import VideoSource


def parse_args() -> argparse.Namespace:
    """Читает параметры источника видео из командной строки."""
    parser = argparse.ArgumentParser(description="Визуальный прототип DronT16")
    parser.add_argument("--source", default="0", help="индекс камеры или путь к видеофайлу")
    parser.add_argument("--display", choices=("hdmi", "web", "both"), default="hdmi",
                        help="вывод: веб-морда, HDMI или оба экрана")
    parser.add_argument("--web-host", default="127.0.0.1", help="адрес веб-просмотра")
    parser.add_argument("--web-port", type=int, default=8080, help="порт веб-просмотра")
    parser.add_argument("--hdmi-x", type=int, default=0, help="X внешнего HDMI-экрана")
    parser.add_argument("--hdmi-y", type=int, default=0, help="Y внешнего HDMI-экрана")
    parser.add_argument("--fullscreen", action="store_true", help="полноэкранный вывод HDMI")
    parser.add_argument("--capture-size", type=int, default=160, help="размер центральной области захвата")
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


def main() -> int:
    """Запускает цикл видео, обработки команд и экранного сопровождения."""
    args = parse_args()
    source = VideoSource(args.source)
    machine = TargetStateMachine()
    tracker = TargetTracker()
    hub = FrameHub()
    display_mode = Mode.IDLE
    if args.display in ("web", "both"):
        start_web_preview(hub, args.web_host, args.web_port)
    if args.display in ("hdmi", "both"):
        cv2.namedWindow("DronT16", cv2.WINDOW_NORMAL)
        cv2.moveWindow("DronT16", args.hdmi_x, args.hdmi_y)
        if args.fullscreen:
            cv2.setWindowProperty("DronT16", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    message = "1: CAPTURE | 2: FOLLOW | 3: AUTOPILOT | 4: ABORT | Q: EXIT"

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
            rendered = draw_overlay(frame, display_mode, machine.target, message)
            hub.update(rendered, display_mode, machine.target, message)
            key = -1
            if args.display in ("hdmi", "both"):
                cv2.imshow("DronT16", rendered)
                key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            web_command = hub.next_command()
            key_command = {ord("1"): Command.CAPTURE, ord("2"): Command.FOLLOW,
                           ord("3"): Command.AUTOPILOT, ord("4"): Command.ABORT}.get(key)
            command = web_command if web_command is not None else key_command
            if command == Command.CAPTURE or command == 1:
                selected = center_target(frame, args.capture_size)
                result = machine.handle(Command.CAPTURE, selected)
                if result.accepted and selected is not None:
                    try:
                        tracker.start(frame, selected)
                    except (RuntimeError, ValueError) as error:
                        machine.handle(Command.ABORT)
                        message = "TRACKER ERROR: TARGET RESET"
                    else:
                        message = result.message
                else:
                    message = result.message
                display_mode = machine.mode
            elif command == Command.FOLLOW or command == 2:
                result = machine.handle(Command.FOLLOW)
                message = result.message
                display_mode = machine.mode
            elif command == Command.AUTOPILOT or command == 3:
                result = machine.handle(Command.AUTOPILOT)
                message = result.message
                display_mode = Mode.DISABLED
            elif command == Command.ABORT or command == 4:
                tracker.reset()
                result = machine.handle(Command.ABORT)
                message = result.message
                display_mode = Mode.IDLE
    finally:
        tracker.reset()
        source.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
