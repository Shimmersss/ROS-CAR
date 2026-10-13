#!/usr/bin/env bash
# Boot entry for the explicitly configured fixed-camera robot; one motion guard.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${GEMINI_SERIAL:?Set the actual Gemini serial in the service environment}"
: "${MODEL_PATH:?Set the target-built Pose engine}"
: "${YOLO_PYTHON:?Set the existing Jetson Pose Python}"
: "${POSE_CONFIG:?Set an explicitly confirmed camera-specific fall profile}"
export GEMINI_CALIBRATION="${GEMINI_CALIBRATION:-$ROOT/ros2_ws/src/perception_bringup/config/gemini_AY2755200PW.json}"
export WITH_CHASSIS=true WITH_VOICE=true WITH_FOLLOWER=true
export WITH_RADAR=false WITH_TONGUE_MODE=false WITH_CAPTURE_TASKS=false
export PERCEPTION_AUXILIARY=false VOICE_BACKEND=offline ASR_BACKEND=offline VOICE_TTS_ENABLED=true
export MOTION_ENABLED="${MOTION_ENABLED:-false}"
export FOLLOW_AUTO_ARM="${FOLLOW_AUTO_ARM:-false}"
export FOLLOW_TARGET_AUTO_RECOVER="${FOLLOW_TARGET_AUTO_RECOVER:-false}"
export FOLLOW_EMERGENCY_KWS_REQUIRED=true
export FOLLOW_TARGET_DISTANCE_M="${FOLLOW_TARGET_DISTANCE_M:-1.3}"
export FOLLOW_TARGET_GRACE_S="${FOLLOW_TARGET_GRACE_S:-0.6}"
export FOLLOW_GEOMETRY_CONFIRMED="${FOLLOW_GEOMETRY_CONFIRMED:-false}"
export FOLLOW_MOUNT_CALIBRATED="${FOLLOW_MOUNT_CALIBRATED:-false}"
export FOLLOW_STOPPING_MODEL_CONFIRMED="${FOLLOW_STOPPING_MODEL_CONFIRMED:-false}"
export SERIAL_PORT="${SERIAL_PORT:-/dev/wheeltec_controller}" CAR_MODE=mini_akm
export AUTO_LOCK_SINGLE="${AUTO_LOCK_SINGLE:-true}" OWNER_LOST_TIMEOUT_S=10.0 DEPTH_REGISTERED=true
# Current onboard trial: follow continuity without appearance verification.
export REID_ENABLED=false REID_LOCK_ENABLED=false REID_REACQUIRE_ENABLED=false
export SOT_ENABLED="${SOT_ENABLED:-true}"
export GEMINI_INPROCESS_REGISTRATION=true GEMINI_REGISTRATION_BACKEND="${GEMINI_REGISTRATION_BACKEND:-cuda}"
export POSE_CONFIG
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 OPENCV_FOR_THREADS_NUM=1
# Wait for udev names once at boot. A later failure stops the stack; no automatic re-arming restart.
for device in "$SERIAL_PORT" /dev/wheeltec_mic; do
  for attempt in {1..30}; do [[ -e "$device" ]] && break; sleep 1; done
  [[ -e "$device" ]] || { echo "Required device unavailable: $device" >&2; exit 1; }
done
mkdir -p "$ROOT/artifacts/project"
if [[ "${ROSCAR_CPU_LAYOUT:-false}" == true ]]; then
  python3 "$ROOT/scripts/apply_robot_cpu_layout.py" --parent "$$" >"$ROOT/artifacts/project/cpu-layout.log" 2>&1 &
  exec taskset -c 0-3 bash "$ROOT/scripts/start_c.sh"
fi
exec bash "$ROOT/scripts/start_c.sh"
