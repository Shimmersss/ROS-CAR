#!/usr/bin/env bash
# Manage Route A; red perception is default, chassis and motion are opt-in.
set -euo pipefail

ROOT="${ROSCAR_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
RUNNER="${ROUTE_A_RUNNER:-$ROOT/scripts/run_red_foxglove.sh}"
STATE_DIR="$ROOT/artifacts/route-a"
PID_FILE="$STATE_DIR/supervisor.pid"
LOG_FILE="$STATE_DIR/supervisor.log"
COMMAND="${1:-start}"
ROS_SETUP="${ROSCAR_ROS_SETUP:-/opt/ros/humble/setup.bash}"
PROJECT_SETUP="${ROSCAR_PROJECT_SETUP:-$ROOT/ros2_ws/install/setup.bash}"
SYSTEMD_UNIT="${ROSCAR_ROUTE_A_UNIT:-roscar-route-a.service}"

mkdir -p "$STATE_DIR"
# Serialize lifecycle changes; the runner must not inherit this lock descriptor.
case "$COMMAND" in
  start|stop|restart)
    exec 9>"$STATE_DIR/lifecycle.lock"
    flock -x 9
    ;;
esac

read_pid() {
  local value=''
  if [[ -f "$PID_FILE" ]]; then
    IFS= read -r value < "$PID_FILE" || true
  fi
  [[ "$value" =~ ^[0-9]+$ ]] && printf '%s\n' "$value"
}

running_pid() {
  local process_id
  process_id="$(read_pid || true)"
  if [[ -n "$process_id" ]] && kill -0 "$process_id" 2>/dev/null && \
      ps -p "$process_id" -o command= 2>/dev/null | grep -Fq "$RUNNER"; then
    printf '%s\n' "$process_id"
    return 0
  fi
  return 1
}

show_connection() {
  local host_address="${ROSCAR_ADDRESS:-192.168.1.240}"
  printf 'Foxglove：ws://%s:8765\n' "$host_address"
  printf '原始视频：/perception/color_image\n'
  printf '画框视频：/perception/detections_image\n'
  printf '布局：foxglove/red-layout.json\n'
  printf '目标状态：/perception/target_state\n'
  printf '红色掩码：/perception/red_mask_image\n'
  printf '3D 目标：/perception/target_marker\n'
}

systemd_active() {
  local state
  command -v systemctl >/dev/null 2>&1 || return 1
  state="$(systemctl show --property=ActiveState --value "$SYSTEMD_UNIT" 2>/dev/null)" || return 1
  case "$state" in
    active|activating|reloading|deactivating) return 0 ;;
    *) return 1 ;;
  esac
}

check_service_route() {
  local entry
  entry="$(systemctl show "$SYSTEMD_UNIT" --property=ExecStart --value)"
  if [[ "$entry" != *run_red_foxglove.sh* ]]; then
    printf '当前服务仍是旧方案 A，尚未切换红色入口：%s\n' "$entry" >&2
    printf '请先部署新代码并更新 systemd 服务，再启动红色方案。\n' >&2
    return 1
  fi
}

preflight() {
  local required
  for required in \
    "$ROS_SETUP" \
    "$PROJECT_SETUP" \
    "$RUNNER" \
    "$ROOT/scripts/run_foxglove.sh"; do
    if [[ ! -e "$required" ]]; then
      printf '缺少方案 A 运行文件：%s\n' "$required" >&2
      return 1
    fi
  done
}

start_stack() {
  local process_id
  if systemd_active; then
    check_service_route || return 1
    printf '方案 A 已由 systemd 开机自启服务管理：%s\n' "$SYSTEMD_UNIT"
    show_connection
    return 0
  fi
  if process_id="$(running_pid)"; then
    printf '方案 A 已经在运行，PID=%s\n' "$process_id"
    show_connection
    return 0
  fi

  if command -v systemctl >/dev/null 2>&1 &&
      [[ "$(systemctl show --property=LoadState --value "$SYSTEMD_UNIT" 2>/dev/null || true)" == loaded ]]; then
    printf '方案 A 由已安装服务管理；请运行 sudo systemctl start %s\n' "$SYSTEMD_UNIT" >&2
    return 1
  fi
  preflight
  : > "$LOG_FILE"
  nohup env \
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}" \
    bash "$RUNNER" 9>&- >> "$LOG_FILE" 2>&1 < /dev/null &
  process_id=$!
  printf '%s\n' "$process_id" > "$PID_FILE"

  for _ in {1..50}; do
    if ! kill -0 "$process_id" 2>/dev/null; then
      printf '方案 A 启动失败，日志如下：\n' >&2
      tail -n 40 "$LOG_FILE" >&2 || true
      return 1
    fi
    if grep -q '方案 A 可视化已启动' "$LOG_FILE"; then
      printf '方案 A 已启动，PID=%s，ROS_DOMAIN_ID=%s\n' \
        "$process_id" "${ROS_DOMAIN_ID:-182}"
      show_connection
      tail -n 8 "$LOG_FILE"
      return 0
    fi
    sleep 0.1
  done

  printf '方案 A 进程已启动，但 5 秒内没有看到就绪日志。\n' >&2
  printf '请运行：bash scripts/route_a.sh logs\n' >&2
  return 1
}

stop_stack() {
  local process_id
  if systemd_active; then
    check_service_route || return 1
    printf '方案 A 正由 systemd 管理，请运行：sudo systemctl stop %s\n' "$SYSTEMD_UNIT" >&2
    return 1
  fi
  if ! process_id="$(running_pid)"; then
    : > "$PID_FILE"
    printf '方案 A 当前未运行。\n'
    return 0
  fi

  kill -TERM "$process_id" 2>/dev/null || true
  for _ in {1..80}; do
    if ! kill -0 "$process_id" 2>/dev/null; then
      : > "$PID_FILE"
      printf '方案 A 已停止。\n'
      return 0
    fi
    sleep 0.1
  done

  printf '方案 A 监督进程未在 8 秒内退出，未强制终止未知残留进程。\n' >&2
  printf '请检查：bash scripts/route_a.sh status\n' >&2
  return 1
}

show_status() {
  local process_id
  if systemd_active; then
    check_service_route || return 1
    printf '方案 A 正由 systemd 运行：%s\n' "$SYSTEMD_UNIT"
    show_connection
    return 0
  fi
  if process_id="$(running_pid)"; then
    printf '方案 A 正在运行，PID=%s\n' "$process_id"
    show_connection
    if command -v ss >/dev/null 2>&1; then
      if ss -ltn 2>/dev/null | grep -q ':8765 '; then
        printf 'Foxglove Bridge 端口 8765：监听中\n'
      else
        printf 'Foxglove Bridge 端口 8765：尚未监听，请查看日志\n'
      fi
    fi
    return 0
  fi
  printf '方案 A 当前未运行。\n'
  return 1
}

case "$COMMAND" in
  start)
    start_stack
    ;;
  stop)
    stop_stack
    ;;
  restart)
    stop_stack
    start_stack
    ;;
  status)
    show_status
    ;;
  logs)
    if systemd_active; then
      journalctl -u "$SYSTEMD_UNIT" -n "${LOG_LINES:-100}" --no-pager
    elif [[ -f "$LOG_FILE" ]]; then
      tail -n "${LOG_LINES:-100}" "$LOG_FILE"
    else
      printf '还没有方案 A 启动日志。\n'
    fi
    ;;
  *)
    printf '用法：bash scripts/route_a.sh {start|stop|restart|status|logs}\n' >&2
    exit 2
    ;;
esac
