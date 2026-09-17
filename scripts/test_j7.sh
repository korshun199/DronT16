#!/usr/bin/env bash

set -Eeuo pipefail

# Каталог проекта определяется относительно расположения этого скрипта.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Используем локальное виртуальное окружение проекта.
PYTHON_BIN="$PROJECT_DIR/.venv/bin/python3"
# Устройство DRM штатного аналогового выхода J7 Raspberry Pi.
J7_DEVICE="/dev/dri/by-path/platform-1f00144000.vec-card"
# Системный путь состояния композитного разъёма J7.
J7_STATUS="/sys/class/drm/card1-Composite-1/status"
# Системный путь включённости композитного выхода J7.
J7_ENABLED="/sys/class/drm/card1-Composite-1/enabled"

# Цвета терминала для понятного результата проверки.
GREEN="\033[32m"
RED="\033[31m"
YELLOW="\033[33m"
RESET="\033[0m"

cd "$PROJECT_DIR"

if [[ ! -x "$PYTHON_BIN" ]]; then
    echo -e "${RED}J7: СИГНАЛ НЕ ИДЁТ — не найден Python проекта: $PYTHON_BIN${RESET}"
    exit 1
fi

status="$(cat "$J7_STATUS" 2>/dev/null || true)"
enabled="$(cat "$J7_ENABLED" 2>/dev/null || true)"
echo "J7: статус DRM=$status, выход=$enabled"

if [[ "$status" != "connected" || "$enabled" != "enabled" ]]; then
    echo -e "${RED}J7: СИГНАЛ НЕ ИДЁТ — выход не подключён или выключен${RESET}"
    exit 2
fi

"$PYTHON_BIN" - "$J7_DEVICE" <<'PY'
"""Подаёт тестовые кадры и проверяет успешную запись в DRM-буфер J7."""

import sys
import time

import cv2
import numpy as np

from src.interface.j7_output import J7Output


device = sys.argv[1]
frame = np.zeros((480, 720, 3), dtype=np.uint8)
frame[:, :] = (40, 40, 180)
cv2.rectangle(frame, (10, 10), (709, 469), (255, 255, 255), 10)
cv2.putText(
    frame,
    "DRONT16 J7 TEST",
    (110, 250),
    cv2.FONT_HERSHEY_SIMPLEX,
    2,
    (0, 255, 0),
    5,
    cv2.LINE_AA,
)

output = None
try:
    output = J7Output(device)
    print(f"J7: DRM master получен, режим {output.width}x{output.height}")
    for _ in range(30):
        output.write(frame)
        time.sleep(0.1)
except (OSError, RuntimeError, ValueError) as error:
    print(f"J7: ошибка передачи кадра: {error}")
    raise SystemExit(3)
finally:
    if output is not None:
        output.close()

print("J7: TEST_FRAMES_WRITTEN=30")
PY

echo -e "${GREEN}J7: СИГНАЛ ИДЁТ — тестовые кадры приняты DRM-выходом${RESET}"
echo -e "${YELLOW}Примечание: это программная проверка передачи; кабель и вход монитора проверяются визуально или измерителем.${RESET}"
