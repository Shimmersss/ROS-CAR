#!/usr/bin/env bash
# Double-click this file on macOS to start Route A on the Jetson.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT" || exit 1

printf '启动方案 A：红色物体检测与跟踪\n'
printf '当前为上位机运动开关；下位机实体开关尚未接入。\n'
if bash scripts/route_a_remote.sh start; then
  printf '\n红色方案 A 进程已启动。Foxglove 连接：ws://192.168.1.240:8765\n'
  printf '请在 Foxglove 导入布局：%s/foxglove/red-layout.json\n' "$ROOT"
  printf '原始视频：/perception/color_image\n画框视频：/perception/detections_image\n'
  printf '若无视频，请检查相机是否在发布彩色话题。\n'
  open -a Foxglove >/dev/null 2>&1 || true
else
  printf '\n启动失败，请根据上面的提示检查。\n' >&2
fi

printf '\n按回车键关闭窗口。\n'
read -r
