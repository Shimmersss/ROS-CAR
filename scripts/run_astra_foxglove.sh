#!/usr/bin/env bash
# Manual, perception-only A-route launcher. No chassis, cmd_vel, or autostart.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR_WS="/home/wheeltec/wheeltec_ros2"
SDK_RUNTIME="$VENDOR_WS/src/wheeltec_bodyreader/bodyreader/lib"
LOG_DIR="$ROOT/artifacts/astra-foxglove"
RGB_STREAM=false

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

cleanup() {
  trap - EXIT INT TERM
  for process_id in "${BRIDGE_PID:-}" "${ADAPTER_PID:-}" "${BODY_PID:-}"; do
    [[ -n "$process_id" ]] && kill -TERM -- "-$process_id" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

( cd "$SDK_RUNTIME" && exec setsid ros2 run bodyreader main --ros-args \
    -p "rgb_stream:=$RGB_STREAM" -p body_stream:=true ) >"$LOG_DIR/bodyreader.log" 2>&1 &
BODY_PID=$!
setsid ros2 run astra_body_adapter bodylist_adapter --ros-args -r __ns:=/perception \
  >"$LOG_DIR/adapter.log" 2>&1 &
ADAPTER_PID=$!
setsid bash "$ROOT/scripts/run_foxglove.sh" >"$LOG_DIR/foxglove-bridge.log" 2>&1 &
BRIDGE_PID=$!

echo '方案 A 可视化已启动：/bodylist、/perception/target_state、/perception/target_marker'
echo "RGB 流：$RGB_STREAM（当前关闭；另行排查后再接入）"
echo 'Foxglove Bridge 仅监听 Jetson 127.0.0.1:8765；Ctrl-C 会停止全部感知进程。'
wait "$BODY_PID"
