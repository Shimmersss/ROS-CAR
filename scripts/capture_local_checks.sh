#!/usr/bin/env bash
# Pure shell preflight checks, shared by the local launcher and non-ROS tests.
capture_check_usb_cam_timestamp() {
  local header="$1" overlay="$2" code
  if [[ -f "$overlay/install/setup.bash" && -f "$overlay/timestamp-fix.txt" ]]; then
    return 0
  fi
  [[ -r "$header" ]] || { echo '无法检查 usb_cam 时间戳头文件，拒绝启动。' >&2; return 1; }
  if grep -Fq 'epoch_time.tv_usec / 1000.0' "$header"; then
    echo '已安装 usb_cam 0.8.1 存在时间戳单位问题；请先 bash scripts/build_capture_usb_cam.sh。' >&2
    return 1
  else
    code=$?
    [[ "$code" == 1 ]] || { echo 'usb_cam 时间戳检查失败。' >&2; return 1; }
  fi
}

capture_check_speech_owner() {
  local actions
  if ! actions="$(ros2 action list 2>/dev/null)"; then
    echo '无法查询 ROS action 列表，拒绝启动；请检查 ROS 环境。' >&2
    return 1
  fi
  if grep -Fxq /voice/speak <<< "$actions"; then
    echo '隔离域已存在 SpeakText；本地入口拒绝启动第二个播报所有者。' >&2
    return 1
  fi
}
