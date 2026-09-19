#!/usr/bin/env bash
set -eo pipefail

workspace_dir="${ROSCAR_WS:-/home/wheeltec/ROSCAR/ros2_ws}"
voice_env_file="${ROSCAR_VOICE_ENV:-${XDG_CONFIG_HOME:-${HOME}/.config}/roscar/voice.env}"

if [[ ! -f "${voice_env_file}" ]]; then
  echo "缺少私有凭据文件: ${voice_env_file}" >&2
  exit 1
fi

set -a
# shellcheck disable=SC1090
source "${voice_env_file}"
set +a

missing=()
for variable_name in XFYUN_APP_ID XFYUN_API_KEY XFYUN_API_SECRET DEEPSEEK_API_KEY; do
  if [[ -z "${!variable_name:-}" ]]; then
    missing+=("${variable_name}")
  fi
done
if (( ${#missing[@]} > 0 )); then
  echo "以下凭据尚未填写: ${missing[*]}" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
vendor_setup="/home/wheeltec/wheeltec_ros2/install/setup.bash"
if [[ -f "${vendor_setup}" ]]; then
  # Only the microphone serial wake executable is used from this overlay.
  # shellcheck disable=SC1090
  source "${vendor_setup}"
fi
source "${workspace_dir}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
voice_tts_enabled="${VOICE_TTS_ENABLED:-true}"
if [[ "${voice_tts_enabled}" != true && "${voice_tts_enabled}" != false ]]; then
  echo 'VOICE_TTS_ENABLED 必须为 true 或 false。' >&2
  exit 2
fi

# Optional voice overrides; empty keeps xfyun_speech/config/voice_assistant.yaml.
tts_arguments=()
voice_overrides=()
if [[ -n "${VOICE_NAME:-}" ]]; then
  voice_overrides+=("voice_name:=${VOICE_NAME}")
fi
if [[ -n "${VOICE_SPEED:-}" ]]; then
  voice_overrides+=("speed:=${VOICE_SPEED}")
fi
if [[ -n "${VOICE_PITCH:-}" ]]; then
  voice_overrides+=("pitch:=${VOICE_PITCH}")
fi
if [[ -n "${VOICE_VOLUME:-}" ]]; then
  voice_overrides+=("volume:=${VOICE_VOLUME}")
fi
if (( ${#voice_overrides[@]} > 0 )); then
  tts_arguments=("${voice_overrides[@]}")
  echo "TTS 音色覆盖: ${tts_arguments[*]}"
fi

exec ros2 launch xfyun_speech voice_assistant.launch.py \
  enable_wake_driver:=true enable_tts:="${voice_tts_enabled}" enable_buzzer:=false \
  "${tts_arguments[@]}" "$@"
