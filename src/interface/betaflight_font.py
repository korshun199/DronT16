"""Штатный 12x18 OSD-шрифт Betaflight для цифрового видеослоя.

Данные получены из открытого исходника Betaflight ``font_betaflight.c``
(GPL-3.0-or-later). Это стандартный шрифт 256 символов MAX7456: он нужен,
потому что DisplayPort передаёт номера глифов, а не Unicode/ASCII-текст.
"""

from __future__ import annotations

import base64
import zlib
from functools import lru_cache

import numpy as np


# Размер одного глифа MAX7456: 12 x 18 пикселей, по 2 бита на пиксель.
GLYPH_WIDTH = 12
GLYPH_HEIGHT = 18
GLYPH_BYTES = 54

# Сжатый исходный набор из 256 глифов. Храним в коде, чтобы Raspberry работала
# автономно, без сети и без необходимости устанавливать Betaflight Configurator.
_FONT_B64 = (
    "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"
)


@lru_cache(maxsize=256)
def glyph_mask(code: int) -> np.ndarray:
    """Возвращает маску белых пикселей глифа Betaflight с номером ``code``.

    Формат MAX7456 хранит три байта на строку: шесть 2-битных пикселей в
    каждом байте. Значение 2 — белый, остальные значения прозрачны/чёрные.
    """
    if not 0 <= code <= 255:
        return np.zeros((GLYPH_HEIGHT, GLYPH_WIDTH), dtype=np.uint8)
    raw = zlib.decompress(base64.b64decode(_FONT_B64))
    glyph = raw[code * GLYPH_BYTES:(code + 1) * GLYPH_BYTES]
    mask = np.zeros((GLYPH_HEIGHT, GLYPH_WIDTH), dtype=np.uint8)
    for row in range(GLYPH_HEIGHT):
        for byte_index in range(3):
            byte = glyph[row * 3 + byte_index]
            for pixel_in_byte in range(4):
                # MAX7456 хранит первую пару пикселей в младших битах байта.
                # Чтение от старших битов зеркалит/перемешивает буквы.
                value = (byte >> (pixel_in_byte * 2)) & 0x03
                if value == 2:
                    mask[row, byte_index * 4 + pixel_in_byte] = 255
    return mask
