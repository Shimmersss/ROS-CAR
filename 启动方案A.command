#!/usr/bin/env bash
# Double-click this file on macOS to start Route A on the Jetson.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT" || exit 1

if bash scripts/route_a_remote.sh start; then
  printf '\n方案 A 已拉起。Foxglove 连接：ws://192.168.1.240:8765\n'
  open -a Foxglove >/dev/null 2>&1 || true
else
  printf '\n启动失败，请根据上面的提示检查。\n' >&2
fi

printf '\n按回车键关闭窗口。\n'
read -r
