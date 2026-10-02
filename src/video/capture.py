"""Единый видеовход DronT16 для CSI-камеры Raspberry и OpenCV на ноутбуке."""

from __future__ import annotations

import importlib
from pathlib import Path
from typing import Any, Callable

import cv2
import numpy as np


def is_raspberry_pi() -> bool:
    """Возвращает True только на Raspberry Pi по модели из device tree."""
    model_path = Path("/proc/device-tree/model")
    try:
        return "raspberry pi" in model_path.read_text(encoding="ascii").lower()
    except (FileNotFoundError, OSError, UnicodeError):
        return False


def select_video_backend(source: str, raspberry_pi: bool) -> str:
    """Выбирает Picamera2 для CSI на Raspberry и OpenCV для остальных входов."""
    normalized = source.strip().lower()
    if normalized in {"csi", "picamera2"}:
        return "picamera2"
    if normalized == "auto":
        return "picamera2" if raspberry_pi else "opencv"
    return "opencv"


def _camera_candidates() -> list[str]:
    """Возвращает USB-видеоустройства раньше прочих OpenCV-камер."""
    by_id = sorted(Path("/dev/v4l/by-id").glob("*"))
    usb_candidates = [str(path) for path in by_id if "usb" in path.name.lower()]
    if usb_candidates:
        return usb_candidates
    return [str(path) for path in sorted(Path("/dev").glob("video*"))]


class _FrameProcessor:
    """Одинаково применяет zoom, коррекцию объектива и геометрию кадра."""

    def __init__(
        self,
        width: int,
        height: int,
        zoom_enabled: bool,
        zoom_level: float,
        zoom_center_x: float,
        zoom_center_y: float,
        undistort: bool,
        calibration_file: str,
        flip_horizontal: bool,
        flip_vertical: bool,
        rotate_deg: int,
        contrast: float,
        brightness: float,
        sharpness: float,
    ) -> None:
        """Проверяет параметры и загружает калибровку, если она включена."""
        if zoom_level < 1.0:
            raise ValueError("video.zoom.level должен быть не меньше 1.0")
        if not 0.0 <= zoom_center_x <= 1.0 or not 0.0 <= zoom_center_y <= 1.0:
            raise ValueError("Центр zoom должен находиться в диапазоне 0.0..1.0")
        if rotate_deg not in {0, 90, 180, 270}:
            raise ValueError("video.image.rotate_deg должен быть 0, 90, 180 или 270")
        if contrast < 0.0 or sharpness < 0.0:
            raise ValueError("contrast и sharpness не могут быть отрицательными")
        self.width = width
        self.height = height
        self.zoom_enabled = zoom_enabled
        self.zoom_level = zoom_level
        self.zoom_center_x = zoom_center_x
        self.zoom_center_y = zoom_center_y
        self.undistort = undistort
        self.flip_horizontal = flip_horizontal
        self.flip_vertical = flip_vertical
        self.rotate_deg = rotate_deg
        self.contrast = contrast
        self.brightness = brightness
        self.sharpness = sharpness
        self._map_x: np.ndarray | None = None
        self._map_y: np.ndarray | None = None
        if undistort:
            self._load_calibration(calibration_file)

    def _load_calibration(self, calibration_file: str) -> None:
        """Загружает матрицу камеры и коэффициенты дисторсии из NPZ."""
        path = Path(calibration_file)
        if not path.is_file():
            raise FileNotFoundError(
                f"Калибровка объектива не найдена: {path}; "
                "сначала создай её или отключи video.lens.undistort"
            )
        with np.load(path) as calibration:
            if "camera_matrix" not in calibration or "dist_coeffs" not in calibration:
                raise ValueError("NPZ калибровки должен содержать camera_matrix и dist_coeffs")
            camera_matrix = np.asarray(calibration["camera_matrix"], dtype=np.float64)
            dist_coeffs = np.asarray(calibration["dist_coeffs"], dtype=np.float64)
        optimal_matrix, _ = cv2.getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs, (self.width, self.height), 1.0,
            (self.width, self.height),
        )
        self._map_x, self._map_y = cv2.initUndistortRectifyMap(
            camera_matrix, dist_coeffs, None, optimal_matrix,
            (self.width, self.height), cv2.CV_32FC1,
        )

    def process(self, frame: np.ndarray) -> np.ndarray:
        """Возвращает кадр после коррекции, zoom и настроек изображения."""
        if frame is None or frame.ndim != 3:
            raise ValueError("Видеокадр должен быть цветным массивом HxWx3")
        result = frame
        if self._map_x is not None and self._map_y is not None:
            result = cv2.remap(result, self._map_x, self._map_y, cv2.INTER_LINEAR)
        if self.zoom_enabled and self.zoom_level > 1.0:
            result = self._zoom(result)
        if self.flip_horizontal:
            result = cv2.flip(result, 1)
        if self.flip_vertical:
            result = cv2.flip(result, 0)
        if self.rotate_deg:
            rotations = {
                90: cv2.ROTATE_90_CLOCKWISE,
                180: cv2.ROTATE_180,
                270: cv2.ROTATE_90_COUNTERCLOCKWISE,
            }
            result = cv2.rotate(result, rotations[self.rotate_deg])
        if self.contrast != 1.0 or self.brightness != 0.0:
            result = cv2.convertScaleAbs(result, alpha=self.contrast, beta=self.brightness)
        if self.sharpness > 0.0:
            result = self._sharpen(result, self.sharpness)
        return result

    def _zoom(self, frame: np.ndarray) -> np.ndarray:
        """Обрезает центральную область и возвращает её к размеру потока."""
        height, width = frame.shape[:2]
        crop_width = max(2, round(width / self.zoom_level))
        crop_height = max(2, round(height / self.zoom_level))
        center_x = round(self.zoom_center_x * width)
        center_y = round(self.zoom_center_y * height)
        left = min(max(0, center_x - crop_width // 2), width - crop_width)
        top = min(max(0, center_y - crop_height // 2), height - crop_height)
        cropped = frame[top:top + crop_height, left:left + crop_width]
        return cv2.resize(cropped, (width, height), interpolation=cv2.INTER_LINEAR)

    @staticmethod
    def _sharpen(frame: np.ndarray, amount: float) -> np.ndarray:
        """Повышает резкость с заданной силой без изменения размера кадра."""
        blurred = cv2.GaussianBlur(frame, (0, 0), 1.2)
        return cv2.addWeighted(frame, 1.0 + amount, blurred, -amount, 0)


class _OpenCvSource:
    """Открывает USB-камеру, V4L2-устройство или видеозапись через OpenCV."""

    def __init__(
        self,
        source: str,
        width: int,
        height: int,
        fps: float,
        capture_factory: Callable[..., Any] = cv2.VideoCapture,
    ) -> None:
        """Открывает первый подходящий источник и запрашивает рабочую геометрию."""
        sources = _camera_candidates() if source == "auto" else [source]
        self.capture: Any | None = None
        self.source = source
        for candidate in sources:
            camera_index = int(candidate) if candidate.isdigit() else None
            capture_target: int | str = camera_index if camera_index is not None else candidate
            is_device = camera_index is not None or str(candidate).startswith("/dev/")
            capture = (
                capture_factory(capture_target, cv2.CAP_V4L2)
                if is_device else capture_factory(capture_target)
            )
            if capture.isOpened():
                if is_device:
                    self._configure_camera(capture, width, height, fps)
                self.capture = capture
                self.source = str(candidate)
                break
            capture.release()
        if self.capture is None:
            raise RuntimeError(f"Не удалось открыть OpenCV-видеопоток: {source}")

    @staticmethod
    def _configure_camera(capture: Any, width: int, height: int, fps: float) -> None:
        """Запрашивает рабочий формат USB-камеры без привязки к её модели."""
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_FPS, fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    @property
    def width(self) -> int:
        """Возвращает фактическую ширину OpenCV-потока."""
        return round(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))

    @property
    def height(self) -> int:
        """Возвращает фактическую высоту OpenCV-потока."""
        return round(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    @property
    def fps(self) -> float:
        """Возвращает фактическую частоту OpenCV-потока."""
        return float(self.capture.get(cv2.CAP_PROP_FPS))

    def describe(self) -> str:
        """Описывает фактический OpenCV-поток."""
        return f"OpenCV {self.source}, {self.width}x{self.height} @ {self.fps:.2f} fps"

    def read(self) -> Any:
        """Возвращает очередной кадр OpenCV."""
        success, frame = self.capture.read()
        if not success or frame is None:
            raise EOFError("Видеопоток OpenCV завершён или кадр не прочитан")
        return frame

    def close(self) -> None:
        """Освобождает OpenCV-камеру или файл."""
        self.capture.release()


class _Picamera2Source:
    """Получает кадры OV5647 по CSI через штатный стек Picamera2/libcamera."""

    def __init__(
        self,
        width: int,
        height: int,
        fps: float,
        pixel_format: str,
        camera_index: int,
        buffer_count: int,
        camera_factory: Callable[..., Any] | None = None,
    ) -> None:
        """Настраивает CSI-видеопоток с фиксированной частотой кадров."""
        if camera_factory is None:
            try:
                picamera2_module = importlib.import_module("picamera2")
            except ImportError as error:
                raise RuntimeError(
                    "Picamera2 недоступна. На Raspberry установите пакет "
                    "python3-picamera2 и используйте .venv с system-site-packages"
                ) from error
            camera_factory = picamera2_module.Picamera2

        self.width = width
        self.height = height
        self.fps = fps
        self.pixel_format = pixel_format
        self.camera_index = camera_index
        self.camera = camera_factory(camera_index)
        frame_duration_us = round(1_000_000 / fps)
        try:
            configuration = self.camera.create_video_configuration(
                main={"size": (width, height), "format": pixel_format},
                controls={"FrameDurationLimits": (frame_duration_us, frame_duration_us)},
                buffer_count=buffer_count,
                queue=False,
            )
            self.camera.configure(configuration)
            self.camera.start()
        except Exception:
            self.camera.close()
            raise
        properties = getattr(self.camera, "camera_properties", {})
        self.model = str(properties.get("Model", "CSI camera"))

    def describe(self) -> str:
        """Описывает запрошенный CSI-поток и найденный сенсор."""
        return (
            f"Picamera2 {self.model} camera={self.camera_index}, "
            f"{self.width}x{self.height} @ {self.fps:.2f} fps, {self.pixel_format}"
        )

    def read(self) -> Any:
        """Возвращает очередной кадр CSI в формате, совместимом с OpenCV."""
        frame = self.camera.capture_array("main")
        if frame is None:
            raise EOFError("Picamera2 не вернула кадр CSI-камеры")
        if self.pixel_format == "BGR888":
            # Picamera2 располагает BGR888 как R,G,B; OpenCV ожидает B,G,R.
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame

    def close(self) -> None:
        """Останавливает Picamera2 и освобождает CSI-камеру."""
        try:
            self.camera.stop()
        finally:
            self.camera.close()


class VideoSource:
    """Предоставляет одинаковый интерфейс CSI и OpenCV всему приложению."""

    def __init__(
        self,
        source: str,
        width: int = 1296,
        height: int = 972,
        fps: float = 46.3,
        pixel_format: str = "RGB888",
        camera_index: int = 0,
        buffer_count: int = 4,
        zoom_enabled: bool = False,
        zoom_level: float = 1.0,
        zoom_center_x: float = 0.5,
        zoom_center_y: float = 0.5,
        undistort: bool = False,
        calibration_file: str = "config/ov5647_calibration.npz",
        flip_horizontal: bool = False,
        flip_vertical: bool = False,
        rotate_deg: int = 0,
        contrast: float = 1.0,
        brightness: float = 0.0,
        sharpness: float = 0.0,
        raspberry_detector: Callable[[], bool] = is_raspberry_pi,
        picamera_factory: Callable[..., Any] | None = None,
        opencv_factory: Callable[..., Any] = cv2.VideoCapture,
    ) -> None:
        """Выбирает backend по платформе и проверяет параметры видеопотока."""
        if width <= 0 or height <= 0 or fps <= 0:
            raise ValueError("camera.width, camera.height и camera.fps должны быть больше нуля")
        if buffer_count < 2:
            raise ValueError("camera.buffer_count должен быть не меньше 2")
        if pixel_format not in {"RGB888", "BGR888"}:
            raise ValueError("camera.pixel_format должен быть RGB888 или BGR888")
        # Проверяем постпроцессор до открытия железа, чтобы ошибка конфигурации
        # не оставляла захватчик камеры занятым.
        self._processor = _FrameProcessor(
            width, height, zoom_enabled, zoom_level, zoom_center_x, zoom_center_y,
            undistort, calibration_file, flip_horizontal, flip_vertical, rotate_deg,
            contrast, brightness, sharpness,
        )
        self.backend = select_video_backend(source, raspberry_detector())
        if self.backend == "picamera2":
            self._implementation: Any = _Picamera2Source(
                width, height, fps, pixel_format, camera_index, buffer_count,
                picamera_factory,
            )
        else:
            self._implementation = _OpenCvSource(
                source, width, height, fps, opencv_factory,
            )

    @property
    def width(self) -> int:
        """Возвращает фактическую ширину активного видеопотока."""
        return int(self._implementation.width)

    @property
    def height(self) -> int:
        """Возвращает фактическую высоту активного видеопотока."""
        return int(self._implementation.height)

    @property
    def fps(self) -> float:
        """Возвращает частоту активного видеопотока."""
        return float(self._implementation.fps)

    def describe(self) -> str:
        """Возвращает диагностическое описание активного видеовхода."""
        return str(self._implementation.describe())

    def read(self) -> Any:
        """Возвращает очередной кадр выбранного backend."""
        return self._processor.process(self._implementation.read())

    def close(self) -> None:
        """Освобождает выбранную камеру или видеозапись."""
        self._implementation.close()
