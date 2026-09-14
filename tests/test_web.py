"""Проверки веб-просмотра без запуска сети и камеры."""

import unittest

import cv2
import numpy as np

from src.core.state_machine import Mode, TargetBox
from src.interface.web import FrameHub, create_app


class WebPreviewTests(unittest.TestCase):
    """Проверяет страницу, состояние и MJPEG-маршрут."""

    def test_status_and_video_routes(self) -> None:
        """Веб-морда возвращает состояние и закодированный кадр."""
        hub = FrameHub()
        frame = np.zeros((32, 48, 3), dtype=np.uint8)
        hub.update(frame, Mode.TRACKING, TargetBox(1, 2, 10, 12), "Тест")
        client = create_app(hub).test_client()
        status = client.get("/api/status")
        self.assertEqual(status.status_code, 200)
        self.assertEqual(status.json["mode"], Mode.TRACKING.value)
        self.assertEqual(client.get("/").status_code, 200)
        response = client.get("/video.mjpg", buffered=False)
        try:
            chunk = next(response.response)
        finally:
            response.close()
        self.assertIn(b"Content-Type: image/jpeg", chunk)


if __name__ == "__main__":
    unittest.main()
