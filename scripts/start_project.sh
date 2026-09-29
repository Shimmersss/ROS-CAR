#!/usr/bin/env bash
# Jetson foreground supervisor for Route B, voice and chassis I/O.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
export ROS_LOCALHOST_ONLY=0
export ROSCAR_WS="$ROOT/ros2_ws"
export COLOR_TOPIC="${COLOR_TOPIC:-/camera/color/image_raw}"
export DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/depth/image_raw}"
export CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
export AUTO_LOCK_SINGLE="${AUTO_LOCK_SINGLE:-true}"
WITH_VOICE="${WITH_VOICE:-true}"
WITH_CHASSIS="${WITH_CHASSIS:-false}"
export WITH_RADAR="${WITH_RADAR:-false}"
WITH_FOLLOWER="${WITH_FOLLOWER:-false}"
MOTION_ENABLED="${MOTION_ENABLED:-false}"
MODEL_PATH="${MODEL_PATH:-$ROOT/models/weights/yolo26s-fp16.engine}"
YOLO_PYTHON="${YOLO_PYTHON:-$ROOT/.venv-yolo/bin/python3}"
case "${1:-}" in
  --help|-h)
    cat <<'HELP'
用法：bash scripts/start_project.sh
    启动 Astra RGB-D、B 人体检测/ByteTrack、Foxglove 和语音。
WITH_VOICE=false 关闭语音；VOICE_BACKEND=offline|online 选择离线语音或原在线语音。
WITH_CHASSIS=true 开启底盘串口收发。
底盘要求 SERIAL_PORT 和 CAR_MODE；WITH_FOLLOWER=true 时跟随器通过 motion_guard 受保护输出。
DEPTH_REGISTERED=true 使用驱动当前 RGB-D 配准；坐标精度仍须现场量距验证。
MODEL_PATH、YOLO_PYTHON、YOLO_DEVICE 可覆盖模型和推理环境。
Ctrl-C 或任一子模块退出时停止本入口启动的全部模块。
HELP
    exit 0;;
  '') ;;
  *) echo '未知参数，使用 --help 查看用法。' >&2; exit 2;;
esac
set -u
for value in "$WITH_VOICE" "$WITH_CHASSIS" "$WITH_RADAR" "$WITH_FOLLOWER" "$MOTION_ENABLED" "${DEPTH_REGISTERED:-false}" "$AUTO_LOCK_SINGLE"; do
  [[ "$value" == true || "$value" == false ]] || { echo '开关必须为 true/false' >&2; exit 2; }
done
if [[ "$WITH_FOLLOWER" == true && "$MOTION_ENABLED" != true ]]; then
  echo 'WITH_FOLLOWER=true 时必须显式 MOTION_ENABLED=true。' >&2
  exit 2
fi
[[ -f "$MODEL_PATH" && -x "$YOLO_PYTHON" ]] || { echo '缺少 B 模型或 Python 环境。' >&2; exit 1; }
for file in /opt/ros/humble/setup.bash /home/wheeltec/wheeltec_ros2/install/setup.bash \
  "$ROOT/install/setup.bash"; do
  [[ -f "$file" ]] || { echo "缺少环境：$file" >&2; exit 1; }
done
if [[ "$WITH_CHASSIS" == true ]]; then
  [[ -f "$ROOT/ros2_ws/chassis_install/setup.bash" && -n "${SERIAL_PORT:-}" && -n "${CAR_MODE:-}" ]] || {
    echo '底盘需要已构建 overlay 及明确的 SERIAL_PORT、CAR_MODE。' >&2; exit 1;
  }
fi
if [[ "$WITH_VOICE" == true ]]; then
  case "${VOICE_BACKEND:-offline}" in
    online)
      voice_file="${ROSCAR_VOICE_ENV:-${XDG_CONFIG_HOME:-$HOME/.config}/roscar/voice.env}"
      [[ -f "$voice_file" ]] || { echo "缺少语音私有配置：$voice_file" >&2; exit 1; }
      ;;
    offline)
      offline_root="${ROSCAR_OFFLINE_ROOT:-/home/wheeltec/ROSCAR-offline}"
      offline_models="${ROSCAR_OFFLINE_MODEL_ROOT:-$offline_root/models}"
      voice_files=(
        "$offline_root/venv/lib/python3.10/site-packages/sherpa_onnx/__init__.py"
        "$offline_models/vits-melo-tts-zh_en/model.onnx"
      )
      if [[ "${ASR_BACKEND:-offline}" == offline ]]; then
        qwen_model="$offline_models/sherpa-onnx-qwen3-asr-0.6B-int8-2026-03-25"
        voice_files+=("$qwen_model/conv_frontend.onnx" "$qwen_model/encoder.int8.onnx" \
          "$qwen_model/decoder.int8.onnx" "$qwen_model/tokenizer/vocab.json" \
          "$qwen_model/tokenizer/merges.txt")
      fi
      for file in "${voice_files[@]}"; do
        [[ -f "$file" ]] || { echo "缺少离线语音依赖：$file" >&2; exit 1; }
      done
      ;;
    *) echo 'VOICE_BACKEND 必须为 offline 或 online。' >&2; exit 2;;
  esac
fi
if systemctl is-active --quiet roscar-route-a.service 2>/dev/null; then
  echo '旧 A 服务仍占用相机，请先停止。' >&2; exit 1
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/wheeltec/wheeltec_ros2/install/setup.bash
if [[ "$WITH_RADAR" == true ]]; then
  [[ -f "$ROOT/ros2_ws/radar_install/setup.bash" ]] || { echo "缺少雷达环境" >&2; exit 1; }
  # shellcheck disable=SC1091
  source "$ROOT/ros2_ws/radar_install/setup.bash"
fi
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"
if [[ "$WITH_CHASSIS" == true ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/ros2_ws/chassis_install/setup.bash"
fi
set -u
packages=(astra_camera yolo_person_tracker perception_bringup)
[[ "$WITH_RADAR" == true ]] && packages+=(lslidar_driver)
for package in "${packages[@]}"; do
  ros2 pkg prefix "$package" >/dev/null || { echo "未构建 ROS 包：$package" >&2; exit 1; }
done
if [[ "$WITH_VOICE" == true && "${VOICE_BACKEND:-offline}" == offline ]]; then
  ros2 pkg prefix offline_voice >/dev/null || { echo '未构建 ROS 包：offline_voice' >&2; exit 1; }
fi
mkdir -p "$ROOT/artifacts/project"
exec 8>"$ROOT/artifacts/project/start.lock"
flock -n 8 || { echo '项目总入口已运行。' >&2; exit 1; }
PIDS=()
# shellcheck disable=SC2329
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for _ in {1..60}; do
    local alive=false
    for pid in "${PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && alive=true; done
    [[ "$alive" == false ]] && break
    sleep .1
  done
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
start_module() {
  local name="$1"; shift
  setsid "$@" >"$ROOT/artifacts/project/$name.log" 2>&1 &
  PIDS+=("$!")
  printf '启动 %-10s 日志：%s/artifacts/project/%s.log\n' "$name" "$ROOT" "$name"
}
start_module camera bash "$ROOT/scripts/run_astra_camera.sh"
start_module perception bash "$ROOT/scripts/run_b_radar_foxglove.sh"
if [[ "$WITH_FOLLOWER" == true ]]; then
  start_module follower bash -c "source /opt/ros/humble/setup.bash; source '$ROOT/install/setup.bash'; exec ros2 launch motion_guard follow.launch.py motion_enabled:=true expected_source:=yolo target_frame:=camera_color_optical_frame radar_required:=false auto_arm:=true geometry_confirmed:='${FOLLOW_GEOMETRY_CONFIRMED:-false}' mount_calibrated:='${FOLLOW_MOUNT_CALIBRATED:-false}' stopping_model_confirmed:='${FOLLOW_STOPPING_MODEL_CONFIRMED:-false}' target_distance_m:=1.0"
fi
if [[ "$WITH_CHASSIS" == true ]]; then
  start_module chassis bash "$ROOT/scripts/run_chassis_io.sh"
fi
if [[ "$WITH_VOICE" == true ]]; then
  start_module voice bash "$ROOT/scripts/run_voice_assistant.sh"
fi
printf 'B 感知已派发；N10P=%s，底盘串口=%s，直接跟随=%s，语音=%s，配准确认=%s\n' \
  "${WITH_RADAR:-false}" \
  "$WITH_CHASSIS" "$WITH_FOLLOWER" "$WITH_VOICE" "${DEPTH_REGISTERED:-false}"
printf 'Foxglove 网口：ws://192.168.100.2:8765；布局 foxglove/b-radar-layout.json\n'
set +e
wait -n "${PIDS[@]}"
result=$?
set -e
printf '项目模块退出（%s），正在清理整组；检查 artifacts/project/ 日志。\n' "$result" >&2
exit 1
