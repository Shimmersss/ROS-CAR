#!/usr/bin/env bash
# Isolated foreground local capture stack; synthetic data only with --synthetic.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID=190 ROS_LOCALHOST_ONLY=1
synthetic=false
camera_device="${CAPTURE_VIDEO_DEVICE:-/dev/video0}"
case "${1:-}" in
  --synthetic) synthetic=true ;;
  --help|-h) echo '用法：bash scripts/run_capture_tasks_local.sh [--synthetic]'; echo '默认真实 UVC 1920x1080 + 假播报 + loopback mock；--synthetic 仅合成图像软件联调。'; exit 0 ;;
  '') ;;
  *) echo '未知参数' >&2; exit 2 ;;
esac
source /opt/ros/humble/setup.bash
[[ -f "$ROOT/install/setup.bash" ]] || { echo '请先按实现与验收文档构建。' >&2; exit 1; }
source "$ROOT/install/setup.bash"
if [[ "$synthetic" == false ]]; then
  overlay="$ROOT/artifacts/capture-usb-cam"
  if [[ -f "$overlay/install/setup.bash" && -f "$overlay/timestamp-fix.txt" ]]; then
    source "$overlay/install/setup.bash"
  elif rg -q 'epoch_time.tv_usec / 1000.0' /opt/ros/humble/include/usb_cam/utils.hpp 2>/dev/null; then
    echo '已安装 usb_cam 0.8.1 存在时间戳单位问题；请先 bash scripts/build_capture_usb_cam.sh。' >&2
    exit 1
  fi
fi
cd "$ROOT"
# Inspect isolated graph before introducing fake resources.
if ros2 action list 2>/dev/null | rg -q '^/voice/speak$'; then
  echo '隔离域已存在 SpeakText；本地入口拒绝启动第二个播报所有者。' >&2; exit 1
fi
mkdir -p artifacts/capture-local
exec 9>artifacts/capture-local/start.lock
flock -n 9 || { echo '本地拍照入口已运行。' >&2; exit 1; }
pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for _ in {1..60}; do
    alive=false
    for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null && alive=true; done
    [[ "$alive" == false ]] && break
    sleep .1
  done
  for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
start() { local name="$1"; shift; setsid "$@" >"artifacts/capture-local/$name.log" 2>&1 & pids+=("$!"); }
start mock python3 scripts/mock_capture_cloud.py serve --store artifacts/capture-mock
if [[ "$synthetic" == false ]]; then
  [[ -r "$camera_device" ]] || { echo '摄像头缺失或无权限；可显式 --synthetic 做软件联调。' >&2; exit 1; }
  start camera ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="$camera_device" \
    -p image_width:=1920 -p image_height:=1080 -p pixel_format:=mjpeg2rgb -p framerate:=15.0 \
    -r image_raw:=/camera/color/image_raw
fi
start tasks ros2 launch capture_task capture_task.launch.py fake_tts:=true synthetic_camera:="$synthetic" \
  cloud_url:=http://127.0.0.1:8090 storage_dir:="$ROOT/artifacts/capture-task" camera_profile:="local_$([[ "$synthetic" == true ]] && echo synthetic || echo uvc)_1080p"
echo '隔离域190/localhost；假播报无扬声器声音。另开终端：python3 scripts/mock_capture_cloud.py send pcb'
echo '日志 artifacts/capture-local；图片仅 mock 存于 artifacts/capture-mock；Ctrl-C 清理本入口。'
wait -n "${pids[@]}"
echo '子模块退出，请检查日志。' >&2
exit 1
