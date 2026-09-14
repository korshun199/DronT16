"""Явно отключённая заглушка автопилота до выбора системы управления."""

from __future__ import annotations


class AutopilotDisabled:
    """Не отправляет никаких команд полётному контроллеру."""

    def follow_target(self, *_args: object, **_kwargs: object) -> bool:
        """Отклоняет попытку включить автопилот и возвращает False."""
        return False

    def stop(self) -> None:
        """Оставляет систему бездействующей; команд наружу не отправляет."""
        return None

