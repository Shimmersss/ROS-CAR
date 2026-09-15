#!/usr/bin/env bash
# Foreground, perception-only A-route launcher used manually or by systemd.
# It never starts the chassis or publishes cmd_vel.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR_WS="/home/wheeltec/wheeltec_ros2"
SDK_RUNTIME="$VENDOR_WS/src/wheeltec_bodyreader/bodyreader/lib"
LOG_DIR="$ROOT/artifacts/astra-foxglove"
# The vendor SDK currently stops publishing Bodylist when RGB and body streams
# are enabled together on the ASTRA S, so Route A defaults to body stream only.
RGB_STREAM="${RGB_STREAM:-false}"

if [[ ! -d "$SDK_RUNTIME" ]]; then
  echo "缺少 Astra SDK 运行目录：$SDK_RUNTIME" >&2
  exit 1
fi
if [[ ! -x "$ROOT/scripts/run_foxglove.sh" ]]; then
  echo '缺少 Foxglove Bridge 启动脚本。' >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$VENDOR_WS/install/setup.bash"
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
export ROS_LOCALHOST_ONLY=0
mkdir -p "$LOG_DIR"

# shellcheck disable=SC2329  # Invoked by the EXIT/INT/TERM trap below.
cleanup() {
  trap - EXIT INT TERM
  local process_id
  for process_id in "${BRIDGE_PID:-}" "${ADAPTER_PID:-}" "${BODY_PID:-}"; do
    [[ -n "$process_id" ]] && kill -TERM -- "-$process_id" 2>/dev/null || true
  done
  for _ in {1..30}; do
    local running=false
    for process_id in "${BRIDGE_PID:-}" "${ADAPTER_PID:-}" "${BODY_PID:-}"; do
      if [[ -n "$process_id" ]] && kill -0 -- "-$process_id" 2>/dev/null; then
        running=true
      fi
    done
    [[ "$running" == false ]] && break
    sleep 0.1
  done
  for process_id in "${BRIDGE_PID:-}" "${ADAPTER_PID:-}" "${BODY_PID:-}"; do
    [[ -n "$process_id" ]] && kill -KILL -- "-$process_id" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

( cd "$SDK_RUNTIME" && exec setsid ros2 run bodyreader main --ros-args \
    -p "rgb_stream:=$RGB_STREAM" -p body_stream:=true ) >"$LOG_DIR/bodyreader.log" 2>&1 &
BODY_PID=$!
setsid ros2 launch perception_bringup perception.launch.py route:=astra with_foxglove:=false \
  >"$LOG_DIR/adapter.log" 2>&1 &
ADAPTER_PID=$!
setsid bash "$ROOT/scripts/run_foxglove.sh" >"$LOG_DIR/foxglove-bridge.log" 2>&1 &
BRIDGE_PID=$!

echo '方案 A 可视化已启动：/bodylist、/perception/target_state、/perception/target_marker、/perception/detection_box、/perception/body_mask_image'
echo "RGB 流：$RGB_STREAM（需由 bodyreader 实际发布图像话题后 Foxglove 才能显示）"
echo 'Foxglove Bridge 监听 Jetson 0.0.0.0:8765；Mac 可连接 ws://192.168.1.240:8765。'
echo 'Ctrl-C 会停止全部感知进程。'
set +e
wait -n "$BODY_PID" "$ADAPTER_PID" "$BRIDGE_PID"
EXITED_STATUS=$?
set -e
echo "方案 A 子进程退出，状态=$EXITED_STATUS；正在清理整组进程。" >&2
exit 1
