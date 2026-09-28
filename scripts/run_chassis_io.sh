#!/usr/bin/env bash
# Chassis telemetry only; this entry launches no follower or cmd_vel publisher.
set -eo pipefail
[[ -n "${SERIAL_PORT:-}" && -n "${CAR_MODE:-}" ]] || { echo '缺少 SERIAL_PORT 或 CAR_MODE' >&2; exit 2; }
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/chassis_install/setup.bash"
set -u
exec ros2 run turn_on_wheeltec_robot wheeltec_robot_node --ros-args \
  -p usart_port_name:="$SERIAL_PORT" \
  -p serial_baud_rate:="${SERIAL_BAUD_RATE:-115200}" \
  -p car_mode:="$CAR_MODE" \
  -p command_timeout_s:=0.5 -p feedback_timeout_s:=0.5 \
  -p max_linear_mps:=0.15 -p max_angular_rps:=0.5
