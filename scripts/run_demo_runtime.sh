#!/usr/bin/env bash
# Boot: voice + unique chassis/guard + lightweight dispatcher. No camera or recording.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/demo_runtime_env.sh"
mkdir -p "$ROOT/artifacts/demo-runtime"
exec 9>"$ROOT/artifacts/demo-runtime/core.lock"
flock -n 9 || { echo 'Demo runtime already running.' >&2; exit 1; }
for device in "$SERIAL_PORT" /dev/wheeltec_mic; do
  for attempt in {1..30}; do [[ -e "$device" ]] && break; sleep 1; done
  [[ -e "$device" ]] || { echo "Required device unavailable: $device" >&2; exit 1; }
done
PIDS=()
cleanup() {
  trap - EXIT INT TERM
  # Stop the module before core interfaces disappear. Entire unit is also BindsTo core.
  systemctl --user stop roscar-demo-module.service 2>/dev/null || true
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for _ in {1..30}; do
    alive=false
    for pid in "${PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && alive=true; done
    [[ "$alive" == false ]] && break
    sleep .1
  done
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
start() { local name="$1"; shift; setsid "$@" >"$ROOT/artifacts/demo-runtime/$name.log" 2>&1 & PIDS+=("$!"); }
start chassis bash "$ROOT/scripts/run_chassis_io.sh"
start guard ros2 launch motion_guard follow.launch.py \
  follow_request_topic:=/demo/follow_request motion_enabled:=true auto_arm:=false target_auto_recover:=true \
  expected_source:=yolo target_frame:=camera_color_optical_frame radar_required:=false \
  pause_on_fall:=true fall_resume_s:=3.0 target_grace_s:=0.6 emergency_kws_required:=false \
  geometry_confirmed:="${FOLLOW_GEOMETRY_CONFIRMED:-false}" mount_calibrated:="${FOLLOW_MOUNT_CALIBRATED:-false}" \
  stopping_model_confirmed:="${FOLLOW_STOPPING_MODEL_CONFIRMED:-false}" \
  target_distance_m:=1.3 max_linear_mps:=0.2 distance_gain:=1.0 bearing_gain:=1.6 ackermann:=true min_turn_radius_m:=0.4
start voice bash "$ROOT/scripts/run_voice_assistant.sh" with_motion_adapter:=false voice_estop_enabled:=false enable_speech_action:=true
start speech ros2 launch roscar_api speech_api.launch.py with_announcements:=true
start manager ros2 run demo_motion runtime_manager --ros-args -p project_root:="$ROOT"
wait -n "${PIDS[@]}"
exit 1
