#!/usr/bin/env bash
# Install or manage the Jetson systemd service for Route A.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNIT_NAME="${ROSCAR_ROUTE_A_UNIT:-roscar-route-a.service}"
UNIT_SOURCE="$ROOT/deploy/systemd/$UNIT_NAME"
SYSTEMD_DIR="${ROSCAR_SYSTEMD_DIR:-/etc/systemd/system}"
SYSTEMCTL_BIN="${ROSCAR_SYSTEMCTL_BIN:-systemctl}"
COMMAND="${1:-install}"
TARGET="$SYSTEMD_DIR/$UNIT_NAME"
SUDO_COMMAND=''

if [[ "$SYSTEMD_DIR" == /etc/systemd/system ]] && (( EUID != 0 )); then
  SUDO_COMMAND='sudo'
fi

run_as_root() {
  if [[ -n "$SUDO_COMMAND" ]]; then
    "$SUDO_COMMAND" "$@"
  else
    "$@"
  fi
}

run_systemctl() {
  run_as_root "$SYSTEMCTL_BIN" "$@"
}

case "$COMMAND" in
  install)
    if [[ ! -f "$UNIT_SOURCE" ]]; then
      printf '缺少 systemd 服务文件：%s\n' "$UNIT_SOURCE" >&2
      exit 1
    fi
    if ! "$SYSTEMCTL_BIN" is-active --quiet "$UNIT_NAME" 2>/dev/null; then
      bash "$ROOT/scripts/route_a.sh" stop
    fi
    run_as_root mkdir -p "$SYSTEMD_DIR"
    run_as_root install -m 0644 "$UNIT_SOURCE" "$TARGET"
    run_systemctl daemon-reload
    run_systemctl enable --now "$UNIT_NAME"
    printf '方案 A 开机自启已启用。\n'
    printf 'Foxglove：ws://192.168.1.240:8765\n'
    run_systemctl --no-pager --full status "$UNIT_NAME" || true
    ;;
  remove)
    run_systemctl disable --now "$UNIT_NAME" || true
    if [[ -e "$TARGET" ]]; then
      run_as_root rm -- "$TARGET"
    fi
    run_systemctl daemon-reload
    printf '方案 A 开机自启已移除。\n'
    ;;
  status)
    run_systemctl --no-pager --full status "$UNIT_NAME"
    ;;
  logs)
    journalctl -u "$UNIT_NAME" -n "${LOG_LINES:-100}" --no-pager
    ;;
  *)
    printf '用法：bash scripts/install_route_a_autostart.sh {install|remove|status|logs}\n' >&2
    exit 2
    ;;
esac
