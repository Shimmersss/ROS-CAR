#!/usr/bin/env bash
# Installed Jetson profile: Route B + voice + chassis + guarded automatic follower.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WITH_CHASSIS=true WITH_VOICE=true WITH_FOLLOWER=true
export WITH_RADAR=false
export SERIAL_PORT="${SERIAL_PORT:-/dev/wheeltec_controller}"
export SERIAL_BAUD_RATE="${SERIAL_BAUD_RATE:-115200}"
export CAR_MODE="${CAR_MODE:-mini_akm}"
# The user explicitly authorized provisional RGB-D coordinates and guarded motion.
export MOTION_ENABLED=true
export DEPTH_REGISTERED=true
export FOLLOW_GEOMETRY_CONFIRMED=true
export FOLLOW_MOUNT_CALIBRATED=true
export FOLLOW_STOPPING_MODEL_CONFIRMED=true
exec bash "$ROOT/scripts/start_project.sh" "$@"
