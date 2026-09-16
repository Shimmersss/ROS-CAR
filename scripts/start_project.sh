#!/usr/bin/env bash
# Jetson foreground project supervisor; Ctrl-C stops everything it started.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WITH_VOICE="${WITH_VOICE:-true}"
export WITH_CHASSIS="${WITH_CHASSIS:-false}"
export MOTION_ENABLED="${MOTION_ENABLED:-false}"
export DEPTH_REGISTERED="${DEPTH_REGISTERED:-false}"
export COLOR_TOPIC="${COLOR_TOPIC:-/camera/color/image_raw}"
export DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/depth/image_raw}"
export CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}"
export ROS_LOCALHOST_ONLY=0
export ROSCAR_WS="$ROOT/ros2_ws"
case "${1:-}" in
  --help|-h)
    cat <<'HELP'
用法：bash scripts/start_project.sh
默认启动：Astra 彩色/深度相机、红色检测、Foxglove、语音助手。
Ctrl-C 停止本次启动的所有模块；单个模块退出也会清理整组。
WITH_VOICE=false                不启动语音
WITH_CHASSIS=true               启动底盘收发（要求 SERIAL_PORT、CAR_MODE）
MOTION_ENABLED=true             允许跟随（要求开启底盘并确认配准）
DEPTH_REGISTERED=true           仅现场验证彩色/深度配准后设置
COLOR_TOPIC / DEPTH_TOPIC / CAMERA_INFO_TOPIC 可覆盖相机输入
本脚本在 Jetson 上运行，不执行 SSH、编译或自动部署。
HELP
    exit 0;;
  '') ;;
  *) echo '未知参数，使用 --help 查看用法。' >&2; exit 2;;
esac
set -u
for value in "$WITH_VOICE" "$WITH_CHASSIS" "$MOTION_ENABLED" "$DEPTH_REGISTERED"; do
  [[ "$value" == true || "$value" == false ]] || { echo '开关必须为 true/false' >&2; exit 2; }
done
if [[ "$MOTION_ENABLED" == true && ( "$WITH_CHASSIS" != true || "$DEPTH_REGISTERED" != true ) ]]; then
  echo '开启运动需要 WITH_CHASSIS=true 和已实测的 DEPTH_REGISTERED=true。' >&2; exit 2
fi
if [[ "$WITH_CHASSIS" == true && ( -z "${SERIAL_PORT:-}" || -z "${CAR_MODE:-}" ) ]]; then
  echo '底盘要求显式 SERIAL_PORT 和已核验 CAR_MODE。' >&2; exit 2
fi
for file in /opt/ros/humble/setup.bash /home/wheeltec/wheeltec_ros2/install/setup.bash "$ROOT/ros2_ws/install/setup.bash"; do
  [[ -f "$file" ]] || { echo "缺少环境：$file；请先安装依赖并构建项目。" >&2; exit 1; }
done
if [[ "$WITH_CHASSIS" == true && ! -f "$ROOT/ros2_ws/chassis_install/setup.bash" ]]; then
  echo '请先运行 bash scripts/build_chassis.sh 构建底盘。' >&2; exit 1
fi
if [[ "$WITH_VOICE" == true ]]; then
  voice_file="${ROSCAR_VOICE_ENV:-${XDG_CONFIG_HOME:-$HOME/.config}/roscar/voice.env}"
  [[ -f "$voice_file" ]] || { echo "缺少语音私有配置：$voice_file；不需要语音可设置 WITH_VOICE=false。" >&2; exit 1; }
fi
# Refuse camera contention with the deployed skeleton service.
if systemctl is-active --quiet roscar-route-a.service 2>/dev/null; then
  echo '旧/独立 A 服务正在运行。先执行 sudo systemctl stop roscar-route-a.service，再运行总入口。' >&2; exit 1
fi
set +u
source /opt/ros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
for package in astra_camera red_object_tracker perception_bringup; do
  ros2 pkg prefix "$package" >/dev/null || { echo "未构建/安装 ROS 包：$package" >&2; exit 1; }
done
mkdir -p "$ROOT/artifacts/project"
exec 8>"$ROOT/artifacts/project/start.lock"
flock -n 8 || { echo '项目总入口已运行。' >&2; exit 1; }
PIDS=()
# shellcheck disable=SC2329
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  # The red supervisor owns nested process groups and performs its own cleanup.
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
start_module perception bash "$ROOT/scripts/run_red_foxglove.sh"
if [[ "$WITH_VOICE" == true ]]; then
  start_module voice bash "$ROOT/scripts/run_voice_assistant.sh"
fi
printf '\n模块已派发，等待相机数据；这不代表硬件验收通过。\n'
printf '串口=%s 运动=%s 配准确认=%s 语音=%s\n' "$WITH_CHASSIS" "$MOTION_ENABLED" "$DEPTH_REGISTERED" "$WITH_VOICE"
printf 'Foxglove：ws://192.168.1.240:8765；导入 foxglove/red-layout.json\n'
printf '原图 /perception/color_image；框图 /perception/detections_image\n'
printf 'Ctrl-C 停止整个项目。下位机实体运动开关尚未接入。\n'
set +e
wait -n "${PIDS[@]}"
result=$?
set -e
printf '项目模块退出（%s），正在停止所有模块，请检查 artifacts/project/ 日志。\n' "$result" >&2
exit 1
