"""Общий безопасный канал координат захваченной пилотом цели.

Видеомодуль публикует только положение уже выбранной цели. CRSF-мост читает
этот снимок и использует его для наведения по одной плоскости. Автоматический
поиск новой цели этим модулем не выполняется.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class TargetGuidance:
    """Последнее направление на захваченную цель в градусах камеры."""

    valid: bool
    yaw_error_deg: float | None
    pitch_error_deg: float | None
    updated_at: float
    mode: str

    def is_fresh(self, now: float, max_age_s: float) -> bool:
        """Проверяет, что снимок цели не устарел и содержит направление."""
        return (
            self.valid
            and self.yaw_error_deg is not None
            and max_age_s >= 0
            and now - self.updated_at <= max_age_s
        )


def write_target_guidance(
    path: str | Path,
    *,
    valid: bool,
    yaw_error_deg: float | None,
    pitch_error_deg: float | None,
    mode: str,
) -> None:
    """Атомарно публикует направление цели для отдельного процесса-моста."""
    target_path = Path(path)
    target_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "valid": bool(valid),
        "yaw_error_deg": yaw_error_deg,
        "pitch_error_deg": pitch_error_deg,
        "updated_at": time.monotonic(),
        "mode": str(mode),
    }
    temporary = target_path.with_name(f".{target_path.name}.tmp")
    temporary.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
    os.replace(temporary, target_path)


def read_target_guidance(path: str | Path) -> TargetGuidance | None:
    """Читает последний снимок цели; повреждённый снимок считается отсутствующим."""
    try:
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        return TargetGuidance(
            valid=bool(payload["valid"]),
            yaw_error_deg=(None if payload["yaw_error_deg"] is None else float(payload["yaw_error_deg"])),
            pitch_error_deg=(None if payload["pitch_error_deg"] is None else float(payload["pitch_error_deg"])),
            updated_at=float(payload["updated_at"]),
            mode=str(payload.get("mode", "UNKNOWN")),
        )
    except (FileNotFoundError, OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None
