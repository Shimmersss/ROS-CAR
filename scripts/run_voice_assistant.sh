#!/usr/bin/env bash
set -eo pipefail

workspace_dir="${ROSCAR_WS:-/home/wheeltec/ROSCAR/ros2_ws}"
project_dir="$(cd "${workspace_dir}/.." && pwd)"
voice_env_file="${ROSCAR_VOICE_ENV:-${XDG_CONFIG_HOME:-${HOME}/.config}/roscar/voice.env}"
voice_backend="${VOICE_BACKEND:-offline}"
asr_backend="${ASR_BACKEND:-offline}"
continuous_asr="${CONTINUOUS_ASR:-true}"
if [[ "${asr_backend}" != xfyun && "${asr_backend}" != offline ]]; then
  echo "ASR_BACKEND 必须为 xfyun 或 offline。" >&2; exit 2
fi
if [[ "${continuous_asr}" != true && "${continuous_asr}" != false ]]; then
  echo 'CONTINUOUS_ASR 必须为 true 或 false。' >&2; exit 2
fi
offline_root="${ROSCAR_OFFLINE_ROOT:-/home/wheeltec/ROSCAR-offline}"

if [[ "${voice_backend}" == online || "${asr_backend}" == xfyun ]]; then
  if [[ ! -f "${voice_env_file}" ]]; then
    echo "缺少私有凭据文件: ${voice_env_file}" >&2
    exit 1
  fi
  set -a
  # shellcheck disable=SC1090
  source "${voice_env_file}"
  set +a
  missing=()
  required=(XFYUN_APP_ID XFYUN_API_KEY XFYUN_API_SECRET)
  if [[ "${voice_backend}" == online ]]; then required+=(DEEPSEEK_API_KEY); fi
  for variable_name in "${required[@]}"; do
    if [[ -z "${!variable_name:-}" ]]; then missing+=("${variable_name}"); fi
  done
  if (( ${#missing[@]} > 0 )); then
    echo "以下凭据尚未填写: ${missing[*]}" >&2
    exit 1
  fi
fi

if [[ "${voice_backend}" == offline ]]; then
  export ROSCAR_OFFLINE_MODEL_ROOT="${ROSCAR_OFFLINE_MODEL_ROOT:-${offline_root}/models}"
  model_files=("${ROSCAR_OFFLINE_MODEL_ROOT}/vits-melo-tts-zh_en/model.onnx")
  if [[ "${asr_backend}" == offline ]]; then
    qwen_model="${ROSCAR_OFFLINE_MODEL_ROOT}/sherpa-onnx-qwen3-asr-0.6B-int8-2026-03-25"
    model_files+=("${qwen_model}/conv_frontend.onnx" "${qwen_model}/encoder.int8.onnx" \
      "${qwen_model}/decoder.int8.onnx" "${qwen_model}/tokenizer/vocab.json" \
      "${qwen_model}/tokenizer/merges.txt")
    [[ "${continuous_asr}" == true ]] && model_files+=("${ROSCAR_OFFLINE_MODEL_ROOT}/silero_vad.onnx")
  fi
  for model_file in "${model_files[@]}"; do
    [[ -f "${model_file}" ]] || { echo "缺少离线模型: ${model_file}" >&2; exit 1; }
  done
  [[ -d "${offline_root}/venv/lib/python3.10/site-packages/sherpa_onnx" ]] || {
    echo "缺少隔离安装的 sherpa-onnx: ${offline_root}/venv" >&2; exit 1;
  }
elif [[ "${voice_backend}" != online ]]; then
  echo 'VOICE_BACKEND 必须为 offline 或 online。' >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
vendor_setup="/home/wheeltec/wheeltec_ros2/install/setup.bash"
if [[ -f "${vendor_setup}" ]]; then
  # Only the microphone serial wake executable is used from this overlay.
  # shellcheck disable=SC1090
  source "${vendor_setup}"
fi
source "${project_dir}/install/setup.bash"
if [[ "${voice_backend}" == offline ]]; then
  export PYTHONPATH="${offline_root}/venv/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
fi
# ament's generated Python interface path can be shadowed by an older overlay
# on Jetson; keep the current VoiceCommand messages first for launch children.
interface_python="${project_dir}/install/roscar_interfaces/local/lib/python3.10/dist-packages"
if [[ -d "${interface_python}" ]]; then
  export PYTHONPATH="${interface_python}${PYTHONPATH:+:${PYTHONPATH}}"
fi
interface_lib="${project_dir}/install/roscar_interfaces/lib"
if [[ -d "${interface_lib}" ]]; then
  export LD_LIBRARY_PATH="${interface_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
voice_tts_enabled="${VOICE_TTS_ENABLED:-true}"
if [[ "${voice_tts_enabled}" != true && "${voice_tts_enabled}" != false ]]; then
  echo 'VOICE_TTS_ENABLED 必须为 true 或 false。' >&2
  exit 2
fi

if [[ "${voice_backend}" == online ]]; then
  exec ros2 launch xfyun_speech voice_assistant.launch.py \
    enable_wake_driver:=true enable_tts:="${voice_tts_enabled}" "$@"
fi

launch_pid=''
cleanup_offline() {
  trap - EXIT INT TERM
  if [[ -n "${launch_pid}" ]]; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup_offline EXIT INT TERM
use_xfyun_asr=false
[[ "${asr_backend}" == xfyun ]] && use_xfyun_asr=true
legacy_listen_polling=false
[[ "${use_xfyun_asr}" == true || "${continuous_asr}" == false ]] && legacy_listen_polling=true
echo "语音后端: ASR=${asr_backend}, 控制=固定规则, TTS=本地"
setsid ros2 launch offline_voice offline_voice.launch.py \
  use_xfyun_asr:="${use_xfyun_asr}" continuous_asr:="${continuous_asr}" \
  legacy_listen_polling:="${legacy_listen_polling}" \
  enable_wake_driver:=true enable_tts:="${voice_tts_enabled}" "$@" &
launch_pid=$!
wait "${launch_pid}"
exec ros2 launch xfyun_speech voice_assistant.launch.py \
  enable_wake_driver:=true enable_tts:="${voice_tts_enabled}" "$@"
