#!/usr/bin/env bash
set -eo pipefail

workspace_dir="${ROSCAR_WS:-/home/wheeltec/ROSCAR/ros2_ws}"
project_dir="$(cd "${workspace_dir}/.." && pwd)"
voice_env_file="${ROSCAR_VOICE_ENV:-${XDG_CONFIG_HOME:-${HOME}/.config}/roscar/voice.env}"
voice_backend="${VOICE_BACKEND:-offline}"
offline_root="${ROSCAR_OFFLINE_ROOT:-/home/wheeltec/ROSCAR-offline}"

if [[ "${voice_backend}" == online ]]; then
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
    if [[ -z "${!variable_name:-}" ]]; then missing+=("${variable_name}"); fi
  done
  if (( ${#missing[@]} > 0 )); then
    echo "以下凭据尚未填写: ${missing[*]}" >&2
    exit 1
  fi
elif [[ "${voice_backend}" == offline ]]; then
  export ROSCAR_OFFLINE_MODEL_ROOT="${ROSCAR_OFFLINE_MODEL_ROOT:-${offline_root}/models}"
  for model_file in \
    "${ROSCAR_OFFLINE_MODEL_ROOT}/sherpa-onnx-streaming-paraformer-bilingual-zh-en/encoder.int8.onnx" \
    "${ROSCAR_OFFLINE_MODEL_ROOT}/sherpa-onnx-streaming-paraformer-bilingual-zh-en/decoder.int8.onnx" \
    "${ROSCAR_OFFLINE_MODEL_ROOT}/vits-melo-tts-zh_en/model.onnx"; do
    [[ -f "${model_file}" ]] || { echo "缺少离线模型: ${model_file}" >&2; exit 1; }
  done
  [[ -x "${offline_root}/ollama/bin/ollama" ]] || {
    echo "缺少隔离安装的 Ollama: ${offline_root}/ollama/bin/ollama" >&2; exit 1;
  }
  [[ -d "${offline_root}/venv/lib/python3.10/site-packages/sherpa_onnx" ]] || {
    echo "缺少隔离安装的 sherpa-onnx: ${offline_root}/venv" >&2; exit 1;
  }
else
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

ollama_pid=''
launch_pid=''
cleanup_offline() {
  trap - EXIT INT TERM
  if [[ -n "${launch_pid}" ]]; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
  if [[ -n "${ollama_pid}" ]]; then
    kill "${ollama_pid}" 2>/dev/null || true
    wait "${ollama_pid}" 2>/dev/null || true
  fi
}
trap cleanup_offline EXIT INT TERM
if ! curl --noproxy '*' --max-time 2 -fsS http://127.0.0.1:11435/api/tags >/dev/null 2>&1; then
  mkdir -p "${offline_root}/models/ollama"
  OLLAMA_HOST=127.0.0.1:11435 \
    OLLAMA_MODELS="${offline_root}/models/ollama" \
    "${offline_root}/ollama/bin/ollama" serve &
  ollama_pid=$!
  ready=false
  for _ in {1..30}; do
    if curl --noproxy '*' --max-time 2 -fsS http://127.0.0.1:11435/api/tags >/dev/null 2>&1; then
      ready=true
      break
    fi
    sleep 1
  done
  [[ "${ready}" == true ]] || { echo '本机离线 Ollama 启动失败。' >&2; exit 1; }
fi
if ! curl --noproxy '*' --max-time 3 -fsS http://127.0.0.1:11435/api/tags | \
    python3 -c 'import json,sys; sys.exit("qwen3:1.7b" not in {m.get("name") for m in json.load(sys.stdin).get("models", [])})'; then
  echo '本机离线 Ollama 尚未安装 qwen3:1.7b。' >&2
  exit 1
fi
setsid ros2 launch offline_voice offline_voice.launch.py \
  enable_wake_driver:=true enable_tts:="${voice_tts_enabled}" "$@" &
launch_pid=$!
wait "${launch_pid}"
