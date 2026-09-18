#!/usr/bin/env bash
# Запускает автономный симулятор failsafe без доступа к реальному оборудованию.
set -Eeuo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"
exec python3 scripts/simulator_filesafe.py
