#!/usr/bin/env bash
# Jetson foreground supervisor for Route B/C, voice and chassis I/O.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
export ROS_LOCALHOST_ONLY=0
export ROSCAR_WS="$ROOT/ros2_ws"
export PERCEPTION_ROUTE="${PERCEPTION_ROUTE:-b}"
case "$PERCEPTION_ROUTE" in
  b) camera_script=run_astra_camera.sh; model_name=yolo26s; layout=b-radar-layout.json ;;
  c) camera_script=run_gemini_camera.sh; model_name=yolo26s-pose; layout=c-layout.json
     export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
     if [[ "${1:-}" != --help && "${1:-}" != -h ]]; then
       : "${GEMINI_SERIAL:?C 需要 GEMINI_SERIAL}"
       : "${GEMINI_CALIBRATION:?C 需要 GEMINI_CALIBRATION}"
     fi
     export GEMINI_INPROCESS_REGISTRATION="${GEMINI_INPROCESS_REGISTRATION:-false}"
     if [[ "$GEMINI_INPROCESS_REGISTRATION" == true ]]; then
       # Tracker subscribes native frames and publishes camera_info_rect itself.
       export COLOR_TOPIC="${COLOR_TOPIC:-/camera/color/image_raw}"
       export DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/depth/image_raw}"
     fi
     export COLOR_TOPIC="${COLOR_TOPIC:-/camera/color/image_rect}"
     export DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/aligned_depth_to_color/image_raw}"
     export CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/color/camera_info_rect}"
     export CAMERA_MOUNT_CONFIG="${CAMERA_MOUNT_CONFIG:-$ROOT/ros2_ws/src/perception_bringup/config/gemini_mount.yaml}"
     export PERCEPTION_INSTALL="${PERCEPTION_INSTALL:-$ROOT/install}"
     ;;
  *) echo "PERCEPTION_ROUTE 必须为 b 或 c" >&2; exit 2 ;;
esac
export COLOR_TOPIC="${COLOR_TOPIC:-/camera/color/image_raw}"
export DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/depth/image_raw}"
export CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
export AUTO_LOCK_SINGLE="${AUTO_LOCK_SINGLE:-true}"
WITH_VOICE="${WITH_VOICE:-true}"
WITH_CHASSIS="${WITH_CHASSIS:-false}"
export WITH_RADAR="${WITH_RADAR:-false}"
WITH_FOLLOWER="${WITH_FOLLOWER:-false}"
WITH_TONGUE_MODE="${WITH_TONGUE_MODE:-false}"
TONGUE_SAFETY_PROFILE="${TONGUE_SAFETY_PROFILE:-robot}"
TONGUE_CAMERA_TOPIC="${TONGUE_CAMERA_TOPIC:-/camera/color/image_raw}"
if [[ "$PERCEPTION_ROUTE" == c ]]; then
  TONGUE_CAMERA_PROFILE="${TONGUE_CAMERA_PROFILE:-gemini_rgb_tongue_v1}"
else
  TONGUE_CAMERA_PROFILE="${TONGUE_CAMERA_PROFILE:-astra_rgb_tongue_v1}"
fi
MOTION_ENABLED="${MOTION_ENABLED:-false}"
# Follow entry: auto-arm and target auto-recover default on only when motion is enabled.
FOLLOW_AUTO_ARM="${FOLLOW_AUTO_ARM:-$MOTION_ENABLED}"
FOLLOW_TARGET_AUTO_RECOVER="${FOLLOW_TARGET_AUTO_RECOVER:-$MOTION_ENABLED}"
FOLLOW_EMERGENCY_KWS_REQUIRED="${FOLLOW_EMERGENCY_KWS_REQUIRED:-true}"
export MODEL_PATH="${MODEL_PATH:-$ROOT/models/weights/$model_name-fp16.engine}"
YOLO_PYTHON="${YOLO_PYTHON:-$ROOT/.venv-yolo/bin/python3}"
case "${1:-}" in
  --help|-h)
    cat <<'HELP'
用法：bash scripts/start_project.sh
    默认启动 B；PERCEPTION_ROUTE=c 或 scripts/start_c.sh 启动 Gemini/Pose。
C 必须指定 GEMINI_SERIAL 和 GEMINI_CALIBRATION；相机/配准实测后才设 DEPTH_REGISTERED=true。
WITH_VOICE=false 关闭语音；VOICE_BACKEND=offline，ASR_BACKEND=xfyun 可回退云端识别。
启用语音及 TTS 时同时提供 /voice/say，复用唯一 TTS 队列。
WITH_CHASSIS=true 开启底盘串口收发。
WITH_TONGUE_MODE=true 将舌苔流程接入现有语音，要求 WITH_VOICE/WITH_FOLLOWER=true。
TONGUE_SAFETY_PROFILE=robot 要求底盘回传；bench 仅允许运动关闭的台架。
TONGUE_CAMERA_TOPIC 默认原始 RGB；C 的拍照元数据默认为 gemini_rgb_tongue_v1。
底盘要求 SERIAL_PORT 和 CAR_MODE；WITH_FOLLOWER=true 时跟随器通过 motion_guard 受保护输出。
DEPTH_REGISTERED=true 使用驱动当前 RGB-D 配准；坐标精度仍须现场量距验证。
FOLLOW_TARGET_DISTANCE_M 跟随距离（米，默认 1.0，按相机测得水平距离）；
FOLLOW_TARGET_GRACE_S 目标短时缺失宽限（默认 0.6）；FOLLOW_TARGET_AUTO_RECOVER=false
关闭"目标恢复后自动续跟"；FOLLOW_AUTO_ARM=false 改为显式 /control/authorize。
两者默认随 MOTION_ENABLED；STOP 回到 IDLE，自动授权不会撤销停车。
FOLLOW_EMERGENCY_KWS_REQUIRED（默认 true）要求语音急停心跳；无语音时须显式 false。
MODEL_PATH、YOLO_PYTHON、YOLO_DEVICE 可覆盖模型和推理环境。
C 可选 GEMINI_INPROCESS_REGISTRATION=true：配准并入 Pose 进程（默认 false，不能与 REID_ENABLED 同用）。
Ctrl-C 或任一子模块退出时停止本入口启动的全部模块。
HELP
    exit 0;;
  '') ;;
  *) echo '未知参数，使用 --help 查看用法。' >&2; exit 2;;
esac
set -u
for value in "$WITH_VOICE" "$WITH_CHASSIS" "$WITH_RADAR" "$WITH_FOLLOWER" "$WITH_TONGUE_MODE" "$MOTION_ENABLED" "${DEPTH_REGISTERED:-false}" "$AUTO_LOCK_SINGLE" "$FOLLOW_AUTO_ARM" "$FOLLOW_TARGET_AUTO_RECOVER" "$FOLLOW_EMERGENCY_KWS_REQUIRED" "${GEMINI_INPROCESS_REGISTRATION:-false}"; do
  [[ "$value" == true || "$value" == false ]] || { echo '开关必须为 true/false' >&2; exit 2; }
done
if [[ "${GEMINI_INPROCESS_REGISTRATION:-false}" == true && "${REID_ENABLED:-false}" == true ]]; then
  echo '进程内配准不发布校正彩色图，ReID 需要独立配准节点；二者不能同时开启。' >&2; exit 2
fi
if [[ "$WITH_TONGUE_MODE" == true ]]; then
  [[ "$WITH_VOICE" == true && "$WITH_FOLLOWER" == true && "${VOICE_TTS_ENABLED:-true}" == true ]] || {
    echo '舌苔模式需要现有语音、带完成确认的 TTS 和唯一 guard（WITH_FOLLOWER=true）。' >&2; exit 2;
  }
  case "$TONGUE_SAFETY_PROFILE" in
    robot) [[ "$WITH_CHASSIS" == true ]] || { echo 'robot 舌苔模式要求底盘速度回传。' >&2; exit 2; };;
    bench) [[ "$MOTION_ENABLED" == false ]] || { echo 'bench 舌苔模式要求 MOTION_ENABLED=false。' >&2; exit 2; };;
    *) echo 'TONGUE_SAFETY_PROFILE 必须为 robot 或 bench。' >&2; exit 2;;
  esac
fi
[[ "${FOLLOW_TARGET_DISTANCE_M:-1.0}" =~ ^[0-9]+(\.[0-9]+)?$ ]] || { echo 'FOLLOW_TARGET_DISTANCE_M 必须为非负米数' >&2; exit 2; }
if [[ "$WITH_FOLLOWER" == true && ( "$FOLLOW_AUTO_ARM" == true || "$FOLLOW_TARGET_AUTO_RECOVER" == true ) && "$MOTION_ENABLED" != true ]]; then
  echo '自动授权/自动续跟只能在 MOTION_ENABLED=true 时开启。' >&2; exit 2
fi
if [[ "$WITH_FOLLOWER" == true && "$FOLLOW_EMERGENCY_KWS_REQUIRED" == true && "$WITH_VOICE" != true ]]; then
  echo 'guard 默认要求语音急停心跳：请 WITH_VOICE=true，或显式 FOLLOW_EMERGENCY_KWS_REQUIRED=false 并由现场人员手动急停。' >&2; exit 2
fi
[[ -f "$MODEL_PATH" && -x "$YOLO_PYTHON" ]] || { echo "缺少 $PERCEPTION_ROUTE 模型或 Python 环境。" >&2; exit 1; }
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
      echo "统一安全链路要求 VOICE_BACKEND=offline，云端识别可用 ASR_BACKEND=xfyun。" >&2; exit 2
      ;;
    offline)
      offline_root="${ROSCAR_OFFLINE_ROOT:-/home/wheeltec/ROSCAR-offline}"
      offline_models="${ROSCAR_OFFLINE_MODEL_ROOT:-$offline_root/models}"
      voice_files=(
        "$offline_root/venv/lib/python3.10/site-packages/sherpa_onnx/__init__.py"
        "$offline_models/vits-melo-tts-zh_en/model.onnx"
        "$offline_models/silero_vad.onnx"
        "$offline_models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/encoder-epoch-13-avg-2-chunk-8-left-64.int8.onnx"
        "$offline_models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/decoder-epoch-13-avg-2-chunk-8-left-64.onnx"
        "$offline_models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/joiner-epoch-13-avg-2-chunk-8-left-64.int8.onnx"
        "$offline_models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/tokens.txt"
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
    *) echo 'VOICE_BACKEND 必须为 offline。' >&2; exit 2;;
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
if [[ "$PERCEPTION_ROUTE" == c ]]; then
  # Colcon setup scripts read unset variables (COLCON_TRACE); source before set -u.
  # shellcheck disable=SC1091
  source "$ROOT/ros2_ws/gemini_driver/install/setup.bash"
fi
set -u
packages=(astra_camera yolo_person_tracker perception_bringup)
[[ "$WITH_TONGUE_MODE" == true ]] && packages+=(execution_mode_interfaces execution_mode_manager tongue_mode)
[[ "$WITH_RADAR" == true ]] && packages+=(lslidar_driver)
for package in "${packages[@]}"; do
  ros2 pkg prefix "$package" >/dev/null || { echo "未构建 ROS 包：$package" >&2; exit 1; }
done
if [[ "$WITH_VOICE" == true && "${VOICE_BACKEND:-offline}" == offline ]]; then
  ros2 pkg prefix offline_voice >/dev/null || { echo '未构建 ROS 包：offline_voice' >&2; exit 1; }
  if [[ "${VOICE_TTS_ENABLED:-true}" == true ]]; then
    ros2 pkg prefix roscar_api >/dev/null || { echo '未构建 ROS 包：roscar_api' >&2; exit 1; }
  fi
fi
if [[ "$WITH_TONGUE_MODE" == true ]]; then
  tongue_catalog="${TONGUE_CATALOG:-$ROOT/install/execution_mode_manager/share/execution_mode_manager/config/modes.yaml}"
  [[ -f "$tongue_catalog" ]] || { echo "缺少执行模式词表：$tongue_catalog" >&2; exit 1; }
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
start_module camera bash "$ROOT/scripts/$camera_script"
start_module perception bash "$ROOT/scripts/run_b_radar_foxglove.sh"
if [[ "$WITH_FOLLOWER" == true ]]; then
  start_module follower bash -c "source /opt/ros/humble/setup.bash; source '$ROOT/install/setup.bash'; exec ros2 launch motion_guard follow.launch.py motion_enabled:='$MOTION_ENABLED' expected_source:=yolo target_frame:=camera_color_optical_frame radar_required:=false auto_arm:='$FOLLOW_AUTO_ARM' target_grace_s:='${FOLLOW_TARGET_GRACE_S:-0.6}' target_auto_recover:='$FOLLOW_TARGET_AUTO_RECOVER' emergency_kws_required:='$FOLLOW_EMERGENCY_KWS_REQUIRED' geometry_confirmed:='${FOLLOW_GEOMETRY_CONFIRMED:-false}' mount_calibrated:='${FOLLOW_MOUNT_CALIBRATED:-false}' stopping_model_confirmed:='${FOLLOW_STOPPING_MODEL_CONFIRMED:-false}' target_distance_m:='${FOLLOW_TARGET_DISTANCE_M:-1.0}'"
fi
if [[ "$WITH_CHASSIS" == true ]]; then
  start_module chassis bash "$ROOT/scripts/run_chassis_io.sh"
fi
if [[ "$WITH_VOICE" == true ]]; then
  voice_args=()
  if [[ "${VOICE_TTS_ENABLED:-true}" == true ]]; then
    voice_args+=(enable_speech_action:=true)
  fi
  if [[ "$WITH_TONGUE_MODE" == true ]]; then
    start_module tongue bash "$ROOT/scripts/run_tongue_mode.sh" \
      catalog:="$tongue_catalog" safety_profile:="$TONGUE_SAFETY_PROFILE" \
      camera_topic:="$TONGUE_CAMERA_TOPIC" camera_profile:="$TONGUE_CAMERA_PROFILE" \
      server_url:="${TONGUE_SERVER_URL:-http://172.19.128.171:8096}"
    voice_args+=(execution_modes_config:="$tongue_catalog")
  fi
  start_module voice bash "$ROOT/scripts/run_voice_assistant.sh" "${voice_args[@]}"
  if [[ "${VOICE_TTS_ENABLED:-true}" == true ]]; then
    start_module speech_api bash -c "source /opt/ros/humble/setup.bash; source '$ROOT/install/setup.bash'; exec ros2 launch roscar_api speech_api.launch.py"
  fi
fi
printf '%s 感知已派发；N10P=%s，底盘串口=%s，直接跟随=%s，语音=%s，配准确认=%s\n' \
  "$PERCEPTION_ROUTE" "${WITH_RADAR:-false}" \
  "$WITH_CHASSIS" "$WITH_FOLLOWER" "$WITH_VOICE" "${DEPTH_REGISTERED:-false}"
printf 'Foxglove 网口：ws://192.168.100.2:8765；布局 foxglove/%s\n' "$layout"
set +e
wait -n "${PIDS[@]}"
result=$?
set -e
printf '项目模块退出（%s），正在清理整组；检查 artifacts/project/ 日志。\n' "$result" >&2
exit 1
