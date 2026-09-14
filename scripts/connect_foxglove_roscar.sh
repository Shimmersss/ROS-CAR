#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCAL_PORT="${1:-8766}"
if nc -z 127.0.0.1 "$LOCAL_PORT" >/dev/null 2>&1; then
  echo "复用已存在的本地隧道 127.0.0.1:${LOCAL_PORT}"
else
  "$ROOT/scripts/open_foxglove_tunnel.sh" "$LOCAL_PORT"
fi
echo "Foxglove 数据源：Foxglove WebSocket"
echo "地址：ws://localhost:${LOCAL_PORT}"
echo "SSH：roscar-wifi -> 127.0.0.1:8765"
echo "随后在 Foxglove 选择打开连接，或导入 foxglove/ssh-ros-datasource.json"
