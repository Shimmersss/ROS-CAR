#!/usr/bin/env bash
# Gemini raw driver + explicit software rectification/D2C. Never use Astra provisional K.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${GEMINI_SERIAL:?必须指定真实 Gemini 序列号}"
: "${GEMINI_CALIBRATION:?必须指定该设备该模式的有效标定 JSON}"
[[ -f "$GEMINI_CALIBRATION" ]] || { echo '缺少 Gemini 设备标定。' >&2; exit 1; }
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/gemini_driver/install/setup.bash"
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}" ROS_LOCALHOST_ONLY=0
python3 - "$GEMINI_CALIBRATION" "$GEMINI_SERIAL" <<'PY'
import json,sys
from yolo_person_tracker.registration import Registration
data=json.load(open(sys.argv[1]))
assert data['serial']==sys.argv[2], '相机标定序列号不匹配'
Registration(data)
assert (data['color_intrinsic']['width'],data['color_intrinsic']['height'])==(640,480)
assert (data['depth_intrinsic']['width'],data['depth_intrinsic']['height'])==(640,400)
PY
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
setsid ros2 launch astra_camera gemini.launch.xml camera_name:=camera \
  serial_number:="$GEMINI_SERIAL" depth_registration:=false \
  enable_color:=true enable_depth:=true enable_ir:=false enable_point_cloud:=false \
  color_width:=640 color_height:=480 color_fps:=30 \
  depth_width:=640 depth_height:=400 depth_fps:=30 &
PIDS+=("$!")
setsid ros2 run yolo_person_tracker gemini_registration --ros-args \
  -p calibration_file:="$GEMINI_CALIBRATION" -p expected_serial:="$GEMINI_SERIAL" &
PIDS+=("$!")
wait -n "${PIDS[@]}" || true
exit 1
