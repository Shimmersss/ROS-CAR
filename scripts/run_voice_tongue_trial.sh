#!/usr/bin/env bash
# Run under the accompanying user systemd unit so all descendants are cleaned up.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TRIAL_CAMERA_BACKEND="${TRIAL_CAMERA_BACKEND:-gemini}"
TONGUE_CAMERA_TOPIC="${TONGUE_CAMERA_TOPIC:-/camera/color/image_raw}"
case "$TRIAL_CAMERA_BACKEND" in
  gemini)
    : "${GEMINI_SERIAL:?Gemini 试运行需要 GEMINI_SERIAL}"
    : "${GEMINI_CALIBRATION:?Gemini 试运行需要 GEMINI_CALIBRATION}"
    [[ -f "$GEMINI_CALIBRATION" ]] || { echo 'Gemini 标定文件不存在。' >&2; exit 2; }
    TONGUE_CAMERA_PROFILE="${TONGUE_CAMERA_PROFILE:-gemini_rgb_tongue_v1}"
    ;;
  astra) TONGUE_CAMERA_PROFILE="${TONGUE_CAMERA_PROFILE:-astra_rgb_tongue_v1}" ;;
  *) echo 'TRIAL_CAMERA_BACKEND 必须为 gemini 或 astra。' >&2; exit 2 ;;
esac
[[ -n "${INVOCATION_ID:-}" ]] || { echo '请通过 roscar-voice-tongue-trial 用户服务启动。' >&2; exit 2; }
if systemctl is-active --quiet roscar-robot.service; then
  echo '请先停止旧整车服务，避免争抢相机、录音和底盘。' >&2; exit 2
fi
CHASSIS_SETUP="${CHASSIS_SETUP:-/home/wheeltec/ROSCAR-current/ros2_ws/chassis_install/setup.bash}"
[[ -f "$ROOT/install/setup.bash" && -f "$CHASSIS_SETUP" ]] || { echo '缺少构建产物。' >&2; exit 2; }
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
set -u
export ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=0
export ROSCAR_WS="$ROOT/ros2_ws" VOICE_BACKEND=offline ASR_BACKEND=offline VOICE_TTS_ENABLED=true
mkdir -p "$ROOT/artifacts/trial"
exec 9>"${XDG_RUNTIME_DIR:?}/roscar-voice-tongue-trial.lock"
flock -n 9 || { echo '已有试运行实例。' >&2; exit 2; }
pids=()
start() {
  local name="$1"; shift
  setsid "$@" >"$ROOT/artifacts/trial/$name.log" 2>&1 &
  pids+=("$!")
}
cleanup() {
  trap - EXIT INT TERM
  timeout 3 ros2 service call /control/stop std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  # systemd KillMode=control-group also catches nested voice launch sessions.
}
trap cleanup EXIT
trap 'exit 0' INT TERM
start guard ros2 run motion_guard guard --ros-args \
  -p motion_enabled:=false -p command_mode:=IDLE -p auto_arm:=false -p radar_required:=false
if [[ "$TRIAL_CAMERA_BACKEND" == gemini ]]; then
  start camera bash "$ROOT/scripts/run_gemini_camera.sh"
else
  # Explicit compatibility profile from the earlier RGB-only trial.
  start camera env ASTRA_LAUNCH=astra_pro.launch.xml ASTRA_UVC_PRODUCT_ID=0x0511 ASTRA_ENABLE_DEPTH=false \
    bash "$ROOT/scripts/run_astra_camera.sh"
fi
start chassis bash -c 'source /opt/ros/humble/setup.bash; source "$1"; exec ros2 run turn_on_wheeltec_robot wheeltec_robot_node --ros-args -p usart_port_name:=/dev/wheeltec_controller -p serial_baud_rate:=115200 -p car_mode:=mini_akm -p command_timeout_s:=0.5 -p feedback_timeout_s:=0.5 -p max_linear_mps:=0.2 -p max_angular_rps:=0.5' _ "$CHASSIS_SETUP"
start voice bash "$ROOT/scripts/run_voice_assistant.sh" \
  "execution_modes_config:=$ROOT/install/execution_mode_manager/share/execution_mode_manager/config/modes.yaml" \
  enable_speech_action:=true
start tongue bash "$ROOT/scripts/run_tongue_mode.sh" safety_profile:=robot \
  camera_topic:="$TONGUE_CAMERA_TOPIC" camera_profile:="$TONGUE_CAMERA_PROFILE" \
  server_url:="${TONGUE_SERVER_URL:-http://172.19.128.171:8096}"
start speech_api ros2 launch roscar_api speech_api.launch.py
echo '试运行已启动：运动关闭，原始 RGB、真实里程计、共享录音、KWS、ASR、TTS、舌苔模式。'
wait -n "${pids[@]}"
# A child exiting cleanly is still unexpected while the supervisor is running.
exit 1
