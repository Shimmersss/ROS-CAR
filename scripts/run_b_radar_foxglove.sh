#!/usr/bin/env bash
# Foreground B + N10P supervisor. Camera runs independently; no chassis or motion nodes.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/radar_install/setup.bash"
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
export ROS_LOCALHOST_ONLY=0
MODEL_PATH="${MODEL_PATH:-$ROOT/models/weights/yolo26s.pt}"
YOLO_PYTHON="${YOLO_PYTHON:-$ROOT/.venv-yolo/bin/python3}"
[[ -f "$MODEL_PATH" ]] || { echo "缺少 B 模型：$MODEL_PATH" >&2; exit 2; }
[[ -x "$YOLO_PYTHON" ]] || { echo "缺少 B Python 环境：$YOLO_PYTHON" >&2; exit 2; }
mkdir -p "$ROOT/artifacts/route-b-radar"
exec 9>"$ROOT/artifacts/route-b-radar/stack.lock"
flock -n 9 || { echo 'B + N10P 已有实例运行。' >&2; exit 1; }
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
setsid ros2 launch perception_bringup perception.launch.py \
  route:=yolo with_radar:=true with_foxglove:=false \
  yolo_python:="$YOLO_PYTHON" model_path:="$MODEL_PATH" device:="${YOLO_DEVICE:-0}" \
  depth_registered:="${DEPTH_REGISTERED:-false}" \
  color_topic:="${COLOR_TOPIC:-/camera/color/image_raw}" \
  depth_topic:="${DEPTH_TOPIC:-/camera/depth/image_raw}" \
  camera_info_topic:="${CAMERA_INFO_TOPIC:-/camera/color/camera_info}" \
  visualization_fps:="${VISUALIZATION_FPS:-10}" \
  visualization_scale:="${VISUALIZATION_SCALE:-0.5}" &
PIDS+=("$!")
setsid bash "$ROOT/scripts/run_foxglove.sh" &
PIDS+=("$!")
echo 'B + N10P 可视化已启动；不含底盘或运动节点。'
echo 'Foxglove 布局：foxglove/b-radar-layout.json'
echo "模型=$MODEL_PATH，设备=${YOLO_DEVICE:-0}，配准确认=${DEPTH_REGISTERED:-false}"
echo "画框视频=${VISUALIZATION_FPS:-10} FPS，缩放=${VISUALIZATION_SCALE:-0.5}"
wait -n "${PIDS[@]}" || true
echo 'B + N10P 子进程退出，清理整组。' >&2
exit 1
