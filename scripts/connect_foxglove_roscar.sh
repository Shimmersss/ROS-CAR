#!/usr/bin/env bash
set -euo pipefail
ROSCAR_HOST="${1:-192.168.1.240}"
ROSCAR_PORT="${2:-8765}"
if ! nc -z -w 3 "$ROSCAR_HOST" "$ROSCAR_PORT" >/dev/null 2>&1; then
  echo "无法连接小车 Foxglove Bridge：${ROSCAR_HOST}:${ROSCAR_PORT}" >&2
  echo '请确认小车感知链路正在运行且 Mac 与小车位于同一网络。' >&2
  exit 1
fi
echo "Foxglove 数据源：Foxglove WebSocket"
echo "地址：ws://${ROSCAR_HOST}:${ROSCAR_PORT}"
echo '无需 SSH 隧道。'
