#!/usr/bin/env bash
# Current robot profile; reuse the project supervisor for ownership and cleanup.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-}" in
  --help|-h)
    cat <<'HELP'
在小车项目根目录运行：bash scripts/start_robot.sh
启动：Astra RGB-D、红色检测、底盘串口收发、语音助手、Foxglove。
Ctrl-C 停止整组；任一模块退出也会停止其余模块。

默认使用上次在线启动配置：
  SERIAL_PORT=/dev/wheeltec_controller
  SERIAL_BAUD_RATE=115200
  CAR_MODE=mini_akm（沿用在线配置，实物车型仍需核验）
当前小车入口默认开启运动，并沿用现场配准配置。仅感知/收发测试：
  MOTION_ENABLED=false bash scripts/start_robot.sh
当前跟随默认距离 0.30 m，最高前进速度 0.15 m/s。
所有配置可通过同名环境变量覆盖；语音凭据默认 ~/.config/roscar/voice.env。
此入口固定启用检测、底盘和语音；按需启动模块请使用 start_project.sh。
日志：artifacts/project/{camera,perception,voice}.log
Foxglove：ws://192.168.1.240:8765，布局 foxglove/red-layout.json
HELP
    exit 0;;
  '') ;;
  *) echo '未知参数，使用 --help 查看用法。' >&2; exit 2;;
esac
export WITH_CHASSIS=true WITH_VOICE=true
export SERIAL_PORT="${SERIAL_PORT:-/dev/wheeltec_controller}"
export SERIAL_BAUD_RATE="${SERIAL_BAUD_RATE:-115200}"
export CAR_MODE="${CAR_MODE:-mini_akm}"
export DEPTH_REGISTERED="${DEPTH_REGISTERED:-true}"
export MOTION_ENABLED="${MOTION_ENABLED:-true}"
printf '启动检测、底盘、语音；串口=%s，车型配置=%s，运动=%s\n' \
  "$SERIAL_PORT" "$CAR_MODE" "$MOTION_ENABLED"
exec bash "$ROOT/scripts/start_project.sh"
