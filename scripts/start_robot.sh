#!/usr/bin/env bash
# Installed Jetson profile: Route B + N10P + voice + chassis telemetry.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WITH_CHASSIS=true WITH_VOICE=true
export SERIAL_PORT="${SERIAL_PORT:-/dev/wheeltec_controller}"
export SERIAL_BAUD_RATE="${SERIAL_BAUD_RATE:-115200}"
export CAR_MODE="${CAR_MODE:-mini_akm}"
# The user explicitly authorized provisional RGB-D coordinates; motion stays disabled.
export MOTION_ENABLED=false
export DEPTH_REGISTERED=true
exec bash "$ROOT/scripts/start_project.sh" "$@"
