"""Проверки ручной веб-настройки камеры без реального сервера и камеры."""

from __future__ import annotations

import shutil
import tempfile
import tomllib
import unittest
from pathlib import Path

from src.interface.camera_web import create_camera_app


class CameraWebTests(unittest.TestCase):
    """Проверяет безопасное чтение и запись только секции camera."""

    def setUp(self) -> None:
        """Создаёт самостоятельную копию общего TOML для каждого теста."""
        self.temp_dir = tempfile.TemporaryDirectory()
        self.config_path = Path(self.temp_dir.name) / "dront16.toml"
        shutil.copy("config/dront16.toml", self.config_path)
        self.client = create_camera_app(self.config_path).test_client()

    def tearDown(self) -> None:
        """Удаляет временную конфигурацию после проверки."""
        self.temp_dir.cleanup()

    def _config(self) -> dict[str, object]:
        """Читает сохранённый TOML штатным парсером Python."""
        with self.config_path.open("rb") as config_file:
            return tomllib.load(config_file)

    def test_camera_page_is_available(self) -> None:
        """Страница открывается по требуемому пути /camera."""
        response = self.client.get("/camera")
        self.assertEqual(response.status_code, 200)
        self.assertIn("DronT16", response.get_data(as_text=True))
        self.assertIn("Сохранить в dront16.toml", response.get_data(as_text=True))

    def test_current_camera_settings_are_returned(self) -> None:
        """API выдаёт текущие параметры отдельной секции camera."""
        response = self.client.get("/api/camera")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json["mode"], "1296x972@46.3")
        self.assertEqual(response.json["zoom_level"], 1.0)

    def test_allowed_values_are_saved_only_to_camera(self) -> None:
        """Ползунки и режим OV5647 меняют TOML атомарно и предсказуемо."""
        response = self.client.post("/api/camera", json={
            "source": "csi", "mode": "1920x1080@30", "pixel_format": "BGR888",
            "camera_index": 1, "camera_buffer_count": 6, "zoom_enabled": True,
            "zoom_level": 2.4, "zoom_center_x": 0.4, "zoom_center_y": 0.6,
            "contrast": 1.3, "brightness": -12, "sharpness": 0.8,
            "flip_horizontal": True, "flip_vertical": False, "undistort": False,
            "rotate_deg": 180,
        })
        self.assertEqual(response.status_code, 200)
        saved = self._config()
        camera = saved["camera"]
        self.assertEqual(camera["source"], "csi")
        self.assertEqual((camera["width"], camera["height"], camera["fps"]), (1920, 1080, 30.0))
        self.assertEqual(camera["pixel_format"], "BGR888")
        self.assertEqual((camera["index"], camera["buffer_count"]), (1, 6))
        self.assertEqual(camera["zoom"]["level"], 2.4)
        self.assertTrue(camera["image"]["flip_horizontal"])
        self.assertEqual(camera["image"]["rotate_deg"], 180)
        self.assertEqual(saved["video"]["display"], "auto")

    def test_unknown_or_invalid_parameter_is_not_saved(self) -> None:
        """API отвергает неизвестные ключи и значения вне безопасного диапазона."""
        before = self.config_path.read_text(encoding="utf-8")
        response = self.client.post("/api/camera", json={"brightness": 101})
        self.assertEqual(response.status_code, 400)
        self.assertEqual(self.config_path.read_text(encoding="utf-8"), before)
        response = self.client.post("/api/camera", json={"dangerous": True})
        self.assertEqual(response.status_code, 400)
        self.assertEqual(self.config_path.read_text(encoding="utf-8"), before)

    def test_settings_default_restores_all_camera_parameters(self) -> None:
        """Кнопка Settings default возвращает только рабочую секцию камеры."""
        self.client.post("/api/camera", json={
            "mode": "1920x1080@30", "pixel_format": "BGR888",
            "zoom_enabled": True, "zoom_level": 3.0, "brightness": 40,
            "rotate_deg": 180, "camera_index": 2, "camera_buffer_count": 8,
        })
        response = self.client.post("/api/camera", json={"reset_defaults": True})
        self.assertEqual(response.status_code, 200)
        saved = self._config()
        camera = saved["camera"]
        self.assertEqual((camera["width"], camera["height"], camera["fps"]), (1296, 972, 46.3))
        self.assertEqual(camera["pixel_format"], "RGB888")
        self.assertEqual(camera["zoom"]["level"], 1.0)
        self.assertFalse(camera["zoom"]["enabled"])
        self.assertEqual(camera["image"]["brightness"], 0.0)
        self.assertEqual(camera["image"]["rotate_deg"], 0)
        self.assertEqual(saved["video"]["display"], "auto")


if __name__ == "__main__":
    unittest.main()
