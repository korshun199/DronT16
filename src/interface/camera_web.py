"""Ручной веб-интерфейс настройки цифровой камеры DronT16."""

from __future__ import annotations

import argparse
import json
import os
import re
import tempfile
from pathlib import Path
from typing import Any

from flask import Flask, Response, jsonify, request

from src.configuration import load_config_section


# Разрешены только заранее известные параметры камеры: веб-интерфейс не даёт
# произвольно редактировать TOML и не меняет настройки полётника или моста.
CAMERA_FIELDS: dict[str, tuple[tuple[str, ...], str, float, float]] = {
    "zoom_level": (("camera", "zoom"), "level", 1.0, 4.0),
    "zoom_center_x": (("camera", "zoom"), "center_x", 0.0, 1.0),
    "zoom_center_y": (("camera", "zoom"), "center_y", 0.0, 1.0),
    "contrast": (("camera", "image"), "contrast", 0.5, 2.0),
    "brightness": (("camera", "image"), "brightness", -100.0, 100.0),
    "sharpness": (("camera", "image"), "sharpness", 0.0, 3.0),
}
BOOL_FIELDS: dict[str, tuple[tuple[str, ...], str]] = {
    "zoom_enabled": (("camera", "zoom"), "enabled"),
    "flip_horizontal": (("camera", "image"), "flip_horizontal"),
    "flip_vertical": (("camera", "image"), "flip_vertical"),
    "undistort": (("camera", "lens"), "undistort"),
}
INT_FIELDS: dict[str, tuple[tuple[str, ...], str, int, int]] = {
    "camera_index": (("camera",), "index", 0, 3),
    "camera_buffer_count": (("camera",), "buffer_count", 2, 8),
}
ROTATIONS = {0, 90, 180, 270}
PIXEL_FORMATS = {"RGB888", "BGR888"}
CAMERA_MODES = {
    "640x480@60": (640, 480, 60.0),
    "640x480@90": (640, 480, 90.0),
    "1296x972@46.3": (1296, 972, 46.3),
    "1920x1080@30": (1920, 1080, 30.0),
}
DEFAULT_CAMERA_VALUES: dict[tuple[tuple[str, ...], str], object] = {
    (("camera",), "source"): "auto",
    (("camera",), "width"): 1296,
    (("camera",), "height"): 972,
    (("camera",), "fps"): 46.3,
    (("camera",), "pixel_format"): "RGB888",
    (("camera",), "index"): 0,
    (("camera",), "buffer_count"): 4,
    (("camera", "zoom"), "enabled"): False,
    (("camera", "zoom"), "level"): 1.0,
    (("camera", "zoom"), "center_x"): 0.5,
    (("camera", "zoom"), "center_y"): 0.5,
    (("camera", "lens"), "undistort"): False,
    (("camera", "image"), "flip_horizontal"): False,
    (("camera", "image"), "flip_vertical"): False,
    (("camera", "image"), "rotate_deg"): 0,
    (("camera", "image"), "contrast"): 1.0,
    (("camera", "image"), "brightness"): 0.0,
    (("camera", "image"), "sharpness"): 0.0,
}


def _toml_value(value: object) -> str:
    """Преобразует разрешённое простое значение в TOML-представление."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return json.dumps(value, ensure_ascii=False)
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return f"{value:.6f}".rstrip("0").rstrip(".")
    raise ValueError("неподдерживаемый тип TOML-значения")


def _replace_toml_values(config_path: Path, updates: dict[tuple[tuple[str, ...], str], object]) -> None:
    """Атомарно меняет известные строки TOML, сохраняя комментарии и порядок."""
    original = config_path.read_text(encoding="utf-8")
    lines = original.splitlines(keepends=True)
    active_section: tuple[str, ...] = ()
    remaining = set(updates)
    section_pattern = re.compile(r"^\s*\[([^]]+)\]\s*(?:#.*)?$")
    key_pattern = re.compile(r"^(\s*)([A-Za-z0-9_]+)\s*=")

    for index, line in enumerate(lines):
        section_match = section_pattern.match(line)
        if section_match:
            active_section = tuple(section_match.group(1).split("."))
            continue
        key_match = key_pattern.match(line)
        if not key_match:
            continue
        target = (active_section, key_match.group(2))
        if target not in updates:
            continue
        suffix = "\n" if line.endswith("\n") else ""
        lines[index] = f"{key_match.group(1)}{target[1]} = {_toml_value(updates[target])}{suffix}"
        remaining.discard(target)

    if remaining:
        missing = ", ".join(f"{'.'.join(section)}.{key}" for section, key in sorted(remaining))
        raise ValueError(f"в TOML не найдены параметры: {missing}")

    file_descriptor, temp_name = tempfile.mkstemp(prefix=".dront16-camera-", dir=config_path.parent)
    try:
        with os.fdopen(file_descriptor, "w", encoding="utf-8") as temp_file:
            temp_file.writelines(lines)
            temp_file.flush()
            os.fsync(temp_file.fileno())
        os.replace(temp_name, config_path)
    except Exception:
        Path(temp_name).unlink(missing_ok=True)
        raise


def _camera_payload(config_path: Path) -> dict[str, object]:
    """Возвращает камеру в удобной для страницы плоской форме."""
    camera = load_config_section(config_path, "camera")
    zoom = camera["zoom"]
    image = camera["image"]
    lens = camera["lens"]
    mode = f"{camera['width']}x{camera['height']}@{camera['fps']:g}"
    return {
        "source": camera["source"],
        "mode": mode,
        "pixel_format": camera["pixel_format"],
        "camera_index": camera["index"],
        "camera_buffer_count": camera["buffer_count"],
        "zoom_enabled": zoom["enabled"],
        "zoom_level": zoom["level"],
        "zoom_center_x": zoom["center_x"],
        "zoom_center_y": zoom["center_y"],
        "flip_horizontal": image["flip_horizontal"],
        "flip_vertical": image["flip_vertical"],
        "rotate_deg": image["rotate_deg"],
        "contrast": image["contrast"],
        "brightness": image["brightness"],
        "sharpness": image["sharpness"],
        "undistort": lens["undistort"],
    }


def _validated_updates(payload: dict[str, Any]) -> dict[tuple[tuple[str, ...], str], object]:
    """Проверяет вход веб-формы и формирует точечные изменения TOML."""
    updates: dict[tuple[tuple[str, ...], str], object] = {}
    for name, (section, key, lower, upper) in CAMERA_FIELDS.items():
        if name not in payload:
            continue
        try:
            value = float(payload[name])
        except (TypeError, ValueError) as error:
            raise ValueError(f"{name}: нужно число") from error
        if not lower <= value <= upper:
            raise ValueError(f"{name}: допустимо от {lower} до {upper}")
        updates[(section, key)] = value
    for name, (section, key) in BOOL_FIELDS.items():
        if name not in payload or not isinstance(payload[name], bool):
            continue
        updates[(section, key)] = payload[name]
    for name, (section, key, lower, upper) in INT_FIELDS.items():
        if name not in payload:
            continue
        try:
            value = int(payload[name])
        except (TypeError, ValueError) as error:
            raise ValueError(f"{name}: нужно целое число") from error
        if not lower <= value <= upper:
            raise ValueError(f"{name}: допустимо от {lower} до {upper}")
        updates[(section, key)] = value
    if "rotate_deg" in payload:
        try:
            rotation = int(payload["rotate_deg"])
        except (TypeError, ValueError) as error:
            raise ValueError("rotate_deg: нужно целое значение") from error
        if rotation not in ROTATIONS:
            raise ValueError("rotate_deg: допустимо 0, 90, 180 или 270")
        updates[(("camera", "image"), "rotate_deg")] = rotation
    if "source" in payload:
        source = str(payload["source"])
        if source not in {"auto", "csi", "picamera2", "0", "1"}:
            raise ValueError("source: разрешены auto, csi, picamera2, 0 или 1")
        updates[(("camera",), "source")] = source
    if "pixel_format" in payload:
        pixel_format = str(payload["pixel_format"])
        if pixel_format not in PIXEL_FORMATS:
            raise ValueError("pixel_format: разрешены RGB888 или BGR888")
        updates[(("camera",), "pixel_format")] = pixel_format
    if "mode" in payload:
        try:
            width, height, fps = CAMERA_MODES[str(payload["mode"])]
        except KeyError as error:
            raise ValueError("mode: неподдерживаемый режим OV5647") from error
        updates[(("camera",), "width")] = width
        updates[(("camera",), "height")] = height
        updates[(("camera",), "fps")] = fps
    unknown = (set(payload) - set(CAMERA_FIELDS) - set(BOOL_FIELDS) - set(INT_FIELDS)
               - {"rotate_deg", "source", "pixel_format", "mode"})
    if unknown:
        raise ValueError(f"неизвестные параметры: {', '.join(sorted(unknown))}")
    if not updates:
        raise ValueError("не переданы параметры камеры")
    return updates


def _page() -> str:
    """Возвращает автономную страницу без внешних библиотек и интернета."""
    return """<!doctype html><html lang=\"ru\"><meta charset=\"utf-8\"><title>DronT16: камера</title>
<style>body{font:17px sans-serif;max-width:720px;margin:24px auto;background:#1d1d1d;color:#eee}h1{color:#ffcc00}fieldset{margin:12px 0;border:1px solid #777}label{display:block;margin:12px 0}input[type=range]{width:100%}select,button{font-size:16px;padding:7px}output{color:#ffcc00;font-weight:bold}#status{padding:10px;background:#333}small{color:#bbb}</style>
<h1>DronT16 — настройка камеры</h1><p>Сохраняет параметры в <code>config/dront16.toml</code>, секция <code>[camera]</code>. Изменения применятся после перезапуска <code>run.sh</code> или <code>dront16.service</code>.</p>
<div id=status>Читаю конфигурацию…</div><form id=form>
<fieldset><legend>Режим OV5647</legend><label>Источник <select name=source><option value=auto>auto (CSI на Raspberry)</option><option value=csi>CSI</option><option value=picamera2>Picamera2</option><option value=0>OpenCV 0</option><option value=1>OpenCV 1</option></select></label><label>Разрешение и FPS <select name=mode><option value=640x480@60>640×480 @ 60</option><option value=640x480@90>640×480 @ 90</option><option value=1296x972@46.3>1296×972 @ 46.3</option><option value=1920x1080@30>1920×1080 @ 30</option></select></label><label>Формат пикселей <select name=pixel_format><option value=RGB888>RGB888</option><option value=BGR888>BGR888</option></select></label><label>Номер камеры <input type=range name=camera_index min=0 max=3 step=1><output></output></label><label>Буферы камеры <input type=range name=camera_buffer_count min=2 max=8 step=1><output></output></label></fieldset>
<fieldset><legend>Цифровое изображение</legend><label><input type=checkbox name=zoom_enabled> Включить цифровой zoom</label><label>Zoom <input type=range name=zoom_level min=1 max=4 step=.1><output></output></label><label>Центр zoom X <input type=range name=zoom_center_x min=0 max=1 step=.01><output></output></label><label>Центр zoom Y <input type=range name=zoom_center_y min=0 max=1 step=.01><output></output></label><label>Контраст <input type=range name=contrast min=.5 max=2 step=.05><output></output></label><label>Яркость <input type=range name=brightness min=-100 max=100 step=1><output></output></label><label>Резкость <input type=range name=sharpness min=0 max=3 step=.1><output></output></label><label>Поворот <select name=rotate_deg><option value=0>0°</option><option value=90>90°</option><option value=180>180°</option><option value=270>270°</option></select></label><label><input type=checkbox name=flip_horizontal> Отразить по горизонтали</label><label><input type=checkbox name=flip_vertical> Отразить по вертикали</label><label><input type=checkbox name=undistort> Коррекция дисторсии (требует NPZ-калибровку)</label></fieldset>
<button>Сохранить в dront16.toml</button> <button type=button id=defaults>Settings default</button></form><script>
const form=document.querySelector('#form'),status=document.querySelector('#status');
function fill(data){for(const [k,v] of Object.entries(data)){const e=form.elements[k];if(!e)continue;if(e.type==='checkbox')e.checked=v;else e.value=v;}outputs();}
function outputs(){for(const e of form.querySelectorAll('input[type=range]'))e.nextElementSibling.value=e.value;}
async function load(){const r=await fetch('/api/camera');const d=await r.json();if(!r.ok)throw Error(d.error);fill(d);status.textContent='Параметры загружены.';}
form.addEventListener('input',outputs);form.addEventListener('submit',async e=>{e.preventDefault();const d={};for(const e of form.elements){if(!e.name)continue;d[e.name]=e.type==='checkbox'?e.checked:e.value;}const r=await fetch('/api/camera',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});const answer=await r.json();status.textContent=r.ok?'Сохранено. Перезапусти run.sh, чтобы применить изменения.':'Ошибка: '+answer.error;});
document.querySelector('#defaults').addEventListener('click',async()=>{if(!confirm('Вернуть все параметры камеры по умолчанию?'))return;const r=await fetch('/api/camera',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({reset_defaults:true})});const answer=await r.json();if(r.ok){fill(answer);status.textContent='Settings default: сохранено. Перезапусти run.sh.';}else status.textContent='Ошибка: '+answer.error;});
load().catch(e=>status.textContent='Ошибка: '+e.message);</script></html>"""


def create_camera_app(config_path: str | Path) -> Flask:
    """Создаёт отдельное Flask-приложение ручной настройки камеры."""
    path = Path(config_path)
    app = Flask(__name__)

    @app.get("/camera")
    def camera_page() -> Response:
        """Отдаёт страницу с безопасными элементами управления камеры."""
        return Response(_page(), mimetype="text/html")

    @app.get("/api/camera")
    def camera_settings() -> Response:
        """Возвращает текущие значения секции camera."""
        try:
            return jsonify(_camera_payload(path))
        except ValueError as error:
            return jsonify({"error": str(error)}), 500

    @app.post("/api/camera")
    def save_camera_settings() -> Response:
        """Проверяет и атомарно сохраняет допустимые параметры камеры."""
        payload = request.get_json(silent=True)
        if not isinstance(payload, dict):
            return jsonify({"error": "ожидается JSON-объект параметров"}), 400
        try:
            if payload.get("reset_defaults") is True:
                updates = DEFAULT_CAMERA_VALUES
            else:
                updates = _validated_updates(payload)
            _replace_toml_values(path, updates)
            return jsonify(_camera_payload(path))
        except (OSError, ValueError) as error:
            return jsonify({"error": str(error)}), 400

    return app


def main() -> int:
    """Запускает ручной веб-сервис; systemd и автозагрузка не используются."""
    parser = argparse.ArgumentParser(description="Ручная настройка камеры DronT16")
    parser.add_argument("--config", default="config/dront16.toml", help="единый TOML-файл проекта")
    args = parser.parse_args()
    try:
        web = load_config_section(args.config, "camera", "web")
        host = str(web["host"])
        port = int(web["port"])
    except (KeyError, ValueError) as error:
        print(f"[DronT16] Ошибка конфигурации веб-камеры: {error}", flush=True)
        return 2
    print(f"[DronT16] Камера: http://{host}:{port}/camera", flush=True)
    print("[DronT16] Ручной сервис. Ctrl+C — остановить; автозагрузка не используется.", flush=True)
    create_camera_app(args.config).run(host=host, port=port, debug=False, use_reloader=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
