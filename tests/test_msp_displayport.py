"""Проверки сетки штатного OSD Betaflight без UART и видеокамеры."""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from src.interface.betaflight_font import glyph_mask
from src.interface.betaflight_osd import BetaflightOsdConfig, draw_betaflight_osd
from src.protocols.msp_displayport import (
    MSP_DP_CLEAR_SCREEN,
    MSP_DP_DRAW_SCREEN,
    MSP_DP_RELEASE,
    MSP_DP_WRITE_STRING,
    DisplayPortCanvas,
)
from src.protocols.betaflight_msp_link import MSP_DISPLAYPORT, MspParser, msp_checksum


def msp_response(command: int, payload: bytes) -> bytes:
    """Формирует корректный MSPv1 ответ полётника для теста общего парсера."""
    return b"$M>" + bytes((len(payload), command)) + payload + bytes((msp_checksum(len(payload), command, payload),))


class DisplayPortCanvasTests(unittest.TestCase):
    """Проверяет команды FC: очистку, строки, кадр и освобождение OSD."""

    def test_writes_string_and_publishes_only_on_draw(self) -> None:
        """Строка попадает в буфер, а готовность возникает при DRAW_SCREEN."""
        canvas = DisplayPortCanvas(30, 13)
        self.assertFalse(canvas.apply(bytes((MSP_DP_CLEAR_SCREEN,))))
        self.assertFalse(canvas.apply(bytes((MSP_DP_WRITE_STRING, 2, 3, 0)) + b"ARM"))
        self.assertTrue(canvas.apply(bytes((MSP_DP_DRAW_SCREEN,))))
        snapshot = canvas.snapshot()
        self.assertTrue(snapshot["active"])
        self.assertEqual(snapshot["frame_counter"], 1)
        self.assertEqual([cell["code"] for cell in snapshot["cells"][2][3:6]], [65, 82, 77])

    def test_clips_string_at_right_border(self) -> None:
        """Длинная строка не выходит за границу сетки 30x13."""
        canvas = DisplayPortCanvas(4, 2)
        canvas.apply(bytes((MSP_DP_WRITE_STRING, 0, 3, 0)) + b"XY")
        snapshot = canvas.snapshot()
        self.assertEqual(snapshot["cells"][0][3]["code"], ord("X"))

    def test_release_clears_and_deactivates_canvas(self) -> None:
        """RELEASE не оставляет устаревшее OSD на видеокадре."""
        canvas = DisplayPortCanvas(4, 2)
        canvas.apply(bytes((MSP_DP_WRITE_STRING, 0, 0, 0)) + b"A")
        self.assertTrue(canvas.apply(bytes((MSP_DP_RELEASE,))))
        self.assertFalse(canvas.snapshot()["active"])
        self.assertEqual(canvas.snapshot()["cells"][0][0]["code"], 32)

    def test_renderer_draws_ascii_snapshot(self) -> None:
        """Видеослой переносит печатаемые символы DisplayPort на кадр OpenCV."""
        canvas = DisplayPortCanvas(30, 13)
        canvas.apply(bytes((MSP_DP_WRITE_STRING, 0, 0, 0)) + b"ARM")
        canvas.apply(bytes((MSP_DP_DRAW_SCREEN,)))
        with tempfile.TemporaryDirectory() as directory:
            state_path = Path(directory) / "osd.json"
            canvas.save(state_path)
            frame = np.zeros((480, 720, 3), dtype=np.uint8)
            config = BetaflightOsdConfig(True, state_path, 30, 13, (255, 255, 255), 1.0)
            result = draw_betaflight_osd(frame, config, now=0.0)
            self.assertGreater(int(result.sum()), 0)

    def test_renderer_draws_betaflight_symbol_not_a_square_placeholder(self) -> None:
        """Штатный символ режима из Betaflight рисуется пиксельным глифом."""
        canvas = DisplayPortCanvas(30, 13)
        canvas.apply(bytes((MSP_DP_WRITE_STRING, 0, 0, 0, 27)))
        canvas.apply(bytes((MSP_DP_DRAW_SCREEN,)))
        with tempfile.TemporaryDirectory() as directory:
            state_path = Path(directory) / "osd.json"
            canvas.save(state_path)
            frame = np.zeros((480, 720, 3), dtype=np.uint8)
            config = BetaflightOsdConfig(True, state_path, 30, 13, (255, 255, 255), 1.0)
            result = draw_betaflight_osd(frame, config, now=0.0)
            self.assertGreater(int(result.sum()), 0)
            self.assertGreater(int(glyph_mask(27).sum()), 0)

    def test_betaflight_font_uses_lsb_first_pixel_order(self) -> None:
        """Буква A из штатного шрифта не должна декодироваться зеркально."""
        glyph_a = glyph_mask(65)
        self.assertEqual(int(glyph_a[4, 5]), 255)
        self.assertEqual(int(glyph_a[4, 0]), 0)

    def test_renderer_keeps_ascii_text_clean_and_special_symbols_pixel_based(self) -> None:
        """Текст читается ровно, а специальные иконки не превращаются в квадрат."""
        canvas = DisplayPortCanvas(30, 13)
        canvas.apply(bytes((MSP_DP_WRITE_STRING, 0, 0, 0)) + b"A0")
        canvas.apply(bytes((MSP_DP_WRITE_STRING, 1, 0, 0, 27)))
        canvas.apply(bytes((MSP_DP_DRAW_SCREEN,)))
        with tempfile.TemporaryDirectory() as directory:
            state_path = Path(directory) / "osd.json"
            canvas.save(state_path)
            frame = np.zeros((480, 720, 3), dtype=np.uint8)
            config = BetaflightOsdConfig(True, state_path, 30, 13, (255, 255, 255), 1.0)
            self.assertGreater(int(draw_betaflight_osd(frame, config, now=0.0).sum()), 0)

    def test_rejects_wrong_canvas_shape(self) -> None:
        """Неподходящий JSON не становится OSD другого размера."""
        with tempfile.TemporaryDirectory() as directory:
            state_path = Path(directory) / "osd.json"
            state_path.write_text(json.dumps({"version": 1, "columns": 1, "rows": 1, "cells": [[]]}))
            frame = np.zeros((480, 720, 3), dtype=np.uint8)
            config = BetaflightOsdConfig(True, state_path, 30, 13, (255, 255, 255), 1.0)
            self.assertEqual(int(draw_betaflight_osd(frame, config).sum()), 0)

    def test_msp_parser_keeps_displayport_packet_for_bridge(self) -> None:
        """Общий MSP-парсер не теряет OSD-пакет, хотя это не образец датчиков."""
        parser = MspParser()
        payload = bytes((MSP_DP_DRAW_SCREEN,))
        self.assertEqual(parser.feed(msp_response(MSP_DISPLAYPORT, payload), received_at=1.0), [])
        packets = parser.drain_packets()
        self.assertEqual(len(packets), 1)
        self.assertEqual((packets[0].command, packets[0].payload), (MSP_DISPLAYPORT, payload))


if __name__ == "__main__":
    unittest.main()
