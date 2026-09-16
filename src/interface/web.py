"""Локальная веб-морда просмотра кадра и состояния DronT16."""

from __future__ import annotations

import threading
import time
from queue import Empty, Queue
from typing import Any

import cv2
from flask import Flask, Response, jsonify, render_template_string

from src.core.state_machine import Mode, TargetBox


# Единая HTML-страница локального просмотра и тестовых команд.
_PAGE = """<!doctype html>
<html lang="ru"><head><meta charset="utf-8"><title>DronT16</title>
<style>body{background:#111;color:#eee;font:16px sans-serif;margin:20px}
img{max-width:100%;height:auto;border:2px solid #555}.status{margin:12px 0}
code{color:#8f8}</style></head><body><h1>DronT16</h1>
<div class="status" id="status">Загрузка состояния...</div>
<div><button onclick="cmd(1)">1 DIRECT</button>
<button onclick="cmd(2)">2 CAPTURE</button>
<button onclick="cmd(3)">3 FOLLOW</button>
<button onclick="cmd(4)">4 ABORT</button></div>
<img src="/video.mjpg" alt="Видеопоток DronT16">
<script>setInterval(async()=>{const s=await fetch('/api/status').then(r=>r.json());
document.getElementById('status').textContent=s.mode+': '+s.message},1000);
async function cmd(n){await fetch('/api/command/'+n,{method:'POST'})}
window.addEventListener('keydown', event => {
  if ('1234'.includes(event.key)) { event.preventDefault(); cmd(event.key); }
});</script>
</body></html>"""


class FrameHub:
    """Потокобезопасно хранит последний кадр и состояние для веб-клиентов."""

    def __init__(self) -> None:
        """Создаёт пустое хранилище кадра."""
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._mode = Mode.IDLE
        self._target: TargetBox | None = None
        self._message = "Ожидание видеокадра"
        self._commands: Queue[int] = Queue()

    def command(self, number: int) -> None:
        """Кладёт команду веб-кнопки в очередь обработки."""
        self._commands.put(number)

    def next_command(self) -> int | None:
        """Возвращает следующую веб-команду или None без ожидания."""
        try:
            return self._commands.get_nowait()
        except Empty:
            return None

    def update(self, frame: Any, mode: Mode, target: TargetBox | None, message: str) -> None:
        """Кодирует и сохраняет последний обработанный кадр."""
        success, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not success:
            return
        with self._lock:
            self._jpeg = encoded.tobytes()
            self._mode = mode
            self._target = target
            self._message = message

    def snapshot(self) -> tuple[bytes | None, dict[str, object]]:
        """Возвращает копию кадра и безопасное состояние без секретов."""
        with self._lock:
            target = self._target
            target_data = None if target is None else {
                "x": target.x, "y": target.y, "width": target.width, "height": target.height,
            }
            status = {"mode": self._mode.value, "message": self._message, "target": target_data}
            return self._jpeg, status


def create_app(hub: FrameHub) -> Flask:
    """Создаёт Flask-приложение только для просмотра локального состояния."""
    app = Flask(__name__)

    @app.get("/")
    def index() -> str:
        """Возвращает страницу просмотра."""
        return render_template_string(_PAGE)

    @app.get("/api/status")
    def status() -> Any:
        """Возвращает JSON-состояние веб-морды и текущего режима."""
        return jsonify(hub.snapshot()[1])

    @app.post("/api/command/<int:number>")
    def command(number: int) -> Any:
        """Принимает только команды 1..4 от локального тестового интерфейса."""
        if number not in (1, 2, 3, 4):
            return jsonify({"error": "unknown command"}), 400
        hub.command(number)
        return jsonify({"accepted": True})

    @app.get("/video.mjpg")
    def video() -> Response:
        """Потоково отдаёт последний кадр в формате MJPEG."""
        def frames() -> Any:
            """Генерирует JPEG-кадры для браузера."""
            while True:
                jpeg, _status = hub.snapshot()
                if jpeg is not None:
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
                time.sleep(0.05)
        return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

    return app


def start_web_preview(hub: FrameHub, host: str, port: int) -> threading.Thread:
    """Запускает веб-просмотр в отдельном daemon-потоке."""
    app = create_app(hub)
    thread = threading.Thread(
        target=lambda: app.run(host=host, port=port, debug=False, threaded=True, use_reloader=False),
        name="dron-t16-web",
        daemon=True,
    )
    thread.start()
    return thread
