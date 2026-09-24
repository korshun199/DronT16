"""Проверки выбора CSI/OpenCV и параметров видеовхода без реальной камеры."""

import unittest

import cv2
import numpy as np

from src.video.capture import VideoSource, select_video_backend


class FakePicamera2:
    """Имитирует минимальный интерфейс Picamera2 для локальных тестов."""

    instances: list["FakePicamera2"] = []

    def __init__(self, camera_index: int) -> None:
        """Запоминает номер камеры и создаёт тестовый кадр."""
        self.camera_index = camera_index
        self.camera_properties = {"Model": "ov5647"}
        self.configuration = None
        self.configured = None
        self.started = False
        self.stopped = False
        self.closed = False
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.__class__.instances.append(self)

    def create_video_configuration(self, **configuration: object) -> dict[str, object]:
        """Возвращает конфигурацию в том же виде для последующей проверки."""
        self.configuration = configuration
        return configuration

    def configure(self, configuration: object) -> None:
        """Запоминает применённую видеоконфигурацию."""
        self.configured = configuration

    def start(self) -> None:
        """Отмечает запуск камеры."""
        self.started = True

    def capture_array(self, stream_name: str) -> np.ndarray:
        """Возвращает тестовый OpenCV-совместимый кадр основного потока."""
        if stream_name != "main":
            raise AssertionError("ожидался основной поток Picamera2")
        return self.frame

    def stop(self) -> None:
        """Отмечает остановку камеры."""
        self.stopped = True

    def close(self) -> None:
        """Отмечает освобождение камеры."""
        self.closed = True


class FakeOpenCvCapture:
    """Имитирует открытую USB-камеру OpenCV."""

    def __init__(self, target: object, backend: object = None) -> None:
        """Запоминает источник и создаёт набор свойств OpenCV."""
        self.target = target
        self.backend = backend
        self.properties: dict[int, float] = {}
        self.released = False
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)

    def isOpened(self) -> bool:
        """Сообщает, что тестовая камера открыта."""
        return True

    def set(self, property_id: int, value: float) -> bool:
        """Сохраняет запрошенное свойство камеры."""
        self.properties[property_id] = value
        return True

    def get(self, property_id: int) -> float:
        """Возвращает сохранённое свойство камеры."""
        return self.properties.get(property_id, 0.0)

    def read(self) -> tuple[bool, np.ndarray]:
        """Возвращает тестовый кадр."""
        return True, self.frame

    def release(self) -> None:
        """Отмечает освобождение камеры."""
        self.released = True


class VideoCaptureTests(unittest.TestCase):
    """Проверяет платформенный выбор и общий интерфейс видеовхода."""

    def setUp(self) -> None:
        """Очищает экземпляры тестовой Picamera2 перед каждой проверкой."""
        FakePicamera2.instances.clear()

    def test_auto_uses_picamera2_only_on_raspberry(self) -> None:
        """Автовыбор не путает CSI Raspberry и USB-камеру ноутбука."""
        self.assertEqual(select_video_backend("auto", True), "picamera2")
        self.assertEqual(select_video_backend("auto", False), "opencv")

    def test_explicit_csi_uses_picamera2_on_any_platform(self) -> None:
        """Явный источник csi принудительно выбирает Picamera2."""
        self.assertEqual(select_video_backend("csi", False), "picamera2")

    def test_file_and_device_use_opencv(self) -> None:
        """Путь и числовой индекс сохраняют совместимость с OpenCV."""
        self.assertEqual(select_video_backend("test.mp4", True), "opencv")
        self.assertEqual(select_video_backend("0", True), "opencv")

    def test_picamera2_receives_requested_video_configuration(self) -> None:
        """CSI получает заданные размеры, RGB888 и фиксированную частоту."""
        source = VideoSource(
            "auto", 640, 480, 60.0, "RGB888", 0, 4,
            raspberry_detector=lambda: True,
            picamera_factory=FakePicamera2,
        )
        camera = FakePicamera2.instances[0]
        self.assertEqual(
            camera.configuration,
            {
                "main": {"size": (640, 480), "format": "RGB888"},
                "controls": {"FrameDurationLimits": (16667, 16667)},
                "buffer_count": 4,
                "queue": False,
            },
        )
        self.assertTrue(camera.started)
        self.assertEqual(source.read().shape, (480, 640, 3))
        self.assertIn("ov5647", source.describe())
        source.close()
        self.assertTrue(camera.stopped)
        self.assertTrue(camera.closed)

    def test_opencv_camera_receives_requested_geometry(self) -> None:
        """Ноутбучная USB-камера получает те же размеры и частоту."""
        captures: list[FakeOpenCvCapture] = []

        def capture_factory(target: object, backend: object = None) -> FakeOpenCvCapture:
            capture = FakeOpenCvCapture(target, backend)
            captures.append(capture)
            return capture

        source = VideoSource(
            "0", 640, 480, 60.0,
            raspberry_detector=lambda: False,
            opencv_factory=capture_factory,
        )
        capture = captures[0]
        self.assertEqual(capture.properties[cv2.CAP_PROP_FRAME_WIDTH], 640)
        self.assertEqual(capture.properties[cv2.CAP_PROP_FRAME_HEIGHT], 480)
        self.assertEqual(capture.properties[cv2.CAP_PROP_FPS], 60.0)
        self.assertEqual(capture.properties[cv2.CAP_PROP_BUFFERSIZE], 1)
        self.assertEqual(source.width, 640)
        self.assertEqual(source.height, 480)
        source.close()
        self.assertTrue(capture.released)

    def test_zoom_keeps_stream_geometry(self) -> None:
        """Цифровой zoom меняет поле зрения, но сохраняет размер кадра."""
        source = VideoSource(
            "auto", 640, 480, 60.0, "RGB888", 0, 4,
            zoom_enabled=True, zoom_level=2.0,
            raspberry_detector=lambda: True,
            picamera_factory=FakePicamera2,
        )
        frame = source.read()
        self.assertEqual(frame.shape, (480, 640, 3))
        source.close()

    def test_nested_camera_sections_are_available(self) -> None:
        """Общие настройки zoom, объектива и изображения читаются из TOML."""
        from src.configuration import load_config_section

        config = load_config_section("config/dront16.toml", "camera")
        self.assertEqual(config["zoom"]["level"], 1.0)
        self.assertFalse(config["lens"]["undistort"])
        self.assertEqual(config["image"]["rotate_deg"], 0)

    def test_invalid_camera_parameters_are_rejected(self) -> None:
        """Ошибочная геометрия не доходит до аппаратного backend."""
        with self.assertRaises(ValueError):
            VideoSource("auto", width=0, raspberry_detector=lambda: False)
        with self.assertRaises(ValueError):
            VideoSource("auto", buffer_count=1, raspberry_detector=lambda: False)
        with self.assertRaises(ValueError):
            VideoSource("auto", pixel_format="YUV420", raspberry_detector=lambda: False)


if __name__ == "__main__":
    unittest.main()
