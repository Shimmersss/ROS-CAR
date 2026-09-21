#!/usr/bin/env bash
# Foreground Route A supervisor. RGB-D comes from an independently configured driver.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
# Optional chassis overlay is built explicitly; its defaults never arm motion.
if [[ "${WITH_CHASSIS:-false}" == true ]]; then
  if [[ -z "${SERIAL_PORT:-}" || -z "${CAR_MODE:-}" ]]; then
    echo '开启底盘需要非空的 SERIAL_PORT 和 CAR_MODE。' >&2
    exit 2
  fi
  source "$ROOT/ros2_ws/chassis_install/setup.bash"
fi
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
export ROS_LOCALHOST_ONLY=0
mkdir -p "$ROOT/artifacts/route-a"
# All managed Route A variants share a lock; the legacy runner takes the same lock.
exec 9>"$ROOT/artifacts/route-a/stack.lock"
flock -n 9 || { echo '方案 A 已有实例运行。' >&2; exit 1; }
PIDS=()
# shellcheck disable=SC2329 # Invoked by EXIT/INT/TERM traps.
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for _ in {1..30}; do
    local alive=false
    for pid in "${PIDS[@]}"; do kill -0 -- "-$pid" 2>/dev/null && alive=true; done
    [[ "$alive" == false ]] && break
    sleep .1
  done
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
launch_args=(
  "performance_enabled:=${PERFORMANCE_ENABLED:-true}"
  "depth_registered:=${DEPTH_REGISTERED:-false}"
  "color_topic:=${COLOR_TOPIC:-/camera/color/image_rect}"
  "depth_topic:=${DEPTH_TOPIC:-/camera/aligned_depth_to_color/image_raw}"
  "camera_info_topic:=${CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
  "with_chassis:=${WITH_CHASSIS:-false}"
  "motion_enabled:=${MOTION_ENABLED:-false}"
  "target_frame:=${TARGET_FRAME:-camera_color_optical_frame}"
)
if [[ -n "${SAFETY_CONFIG:-}" ]]; then
  launch_args+=("safety_config:=$SAFETY_CONFIG")
fi
launch_args+=("target_distance_m:=${TARGET_DISTANCE_M:-1.0}")
if [[ "${WITH_CHASSIS:-false}" == true ]]; then
  if [[ -z "${SERIAL_PORT:-}" || -z "${CAR_MODE:-}" ]]; then
    echo '开启底盘需要非空 SERIAL_PORT 和 CAR_MODE。' >&2
    exit 2
  fi
  launch_args+=(
    "serial_port:=${SERIAL_PORT}"
    "car_mode:=${CAR_MODE}"
    "serial_baud_rate:=${SERIAL_BAUD_RATE:-115200}"
  )
fi
setsid ros2 launch perception_bringup route_a.launch.py "${launch_args[@]}" &
PIDS+=("$!")
setsid bash "$ROOT/scripts/run_foxglove.sh" &
PIDS+=("$!")
echo '方案 A 可视化已启动：红色物体感知进程（视频需外部相机彩色话题）'
echo '原始视频：/perception/color_image；画框视频：/perception/detections_image'
echo 'Foxglove 布局：foxglove/red-layout.json'
echo '运动使能目前由 Jetson 控制，下位机实体开关尚未接入。'
echo "串口=${WITH_CHASSIS:-false}，运动=${MOTION_ENABLED:-false}，配准确认=${DEPTH_REGISTERED:-false}"
wait -n "${PIDS[@]}" || true
echo '方案 A 子进程退出，清理整组。' >&2
exit 1
