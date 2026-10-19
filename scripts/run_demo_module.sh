#!/usr/bin/env bash
# One on-demand heavy mode, owned and reaped as a systemd cgroup.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/demo_runtime_env.sh"
case "${1:-}" in
  FOLLOW|PERCEPTION)
    export WITH_VOICE=false WITH_CHASSIS=false WITH_FOLLOWER=false WITH_DEMO_MOTION=false
    export WITH_CAPTURE_TASKS=false WITH_TONGUE_MODE=false WITH_RADAR=false
    export FOLLOW_AUTO_ARM=false FOLLOW_TARGET_AUTO_RECOVER=false MOTION_ENABLED=false
    exec bash "$ROOT/scripts/start_c.sh" ;;
  RELATIVE)
    exec ros2 run demo_motion relative_motion --ros-args -r /chassis/cmd_vel:=/demo/relative_request ;;
  FACE|FACE_UPLOAD|PCB|HELMET|TONGUE) kind="${1,,}" ;;
  *) echo 'Unknown module' >&2; exit 2 ;;
esac
# Capture uses native RGB only, without Pose/ReID/TensorRT/depth registration.
export GEMINI_DEPTH_ENABLED=false
bash "$ROOT/scripts/run_gemini_camera.sh" & camera_pid=$!
cleanup() { trap - EXIT INT TERM; kill -TERM "$camera_pid" "${capture_pid:-}" 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT INT TERM
pipeline=infer
[[ "$kind" == face ]] && pipeline=collect
[[ "$kind" == face_upload ]] && { kind=face; pipeline=upload; }
ros2 launch capture_task capture_task.launch.py pipeline_mode:="$pipeline" allowed_task_type:="$kind" \
  safety_profile:=robot with_bridge:=false with_ros_api:=false \
  inference_url:="$CAPTURE_INFERENCE_URL" inference_token_file:="$CAPTURE_INFERENCE_TOKEN_FILE" \
  cloud_url:="$CAPTURE_CLOUD_URL" token_file:="$CAPTURE_TOKEN_FILE" \
  camera_profile:=gemini_rgb_tongue_v1 storage_dir:="$ROOT/artifacts/capture-task" save_images:=false & capture_pid=$!
wait -n "$camera_pid" "$capture_pid"
exit 1
