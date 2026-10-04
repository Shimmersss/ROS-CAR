#!/usr/bin/env bash
# Installed Jetson profile: Route B + voice + chassis + guarded automatic follower.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WITH_CHASSIS=true WITH_VOICE=true WITH_FOLLOWER=true
export WITH_RADAR=false
export SERIAL_PORT="${SERIAL_PORT:-/dev/wheeltec_controller}"
export SERIAL_BAUD_RATE="${SERIAL_BAUD_RATE:-115200}"
export CAR_MODE="${CAR_MODE:-mini_akm}"
# Sensing starts normally; motion requires explicit configuration and runtime arm.
export MOTION_ENABLED="${MOTION_ENABLED:-false}"
export DEPTH_REGISTERED=true
export FOLLOW_GEOMETRY_CONFIRMED="${FOLLOW_GEOMETRY_CONFIRMED:-false}"
export FOLLOW_MOUNT_CALIBRATED="${FOLLOW_MOUNT_CALIBRATED:-false}"
export FOLLOW_STOPPING_MODEL_CONFIRMED="${FOLLOW_STOPPING_MODEL_CONFIRMED:-false}"
exec bash "$ROOT/scripts/start_project.sh" "$@"
