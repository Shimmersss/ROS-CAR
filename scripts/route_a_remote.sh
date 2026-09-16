#!/usr/bin/env bash
# Start or manage Route A on the Jetson from the Mac.
set -euo pipefail

SSH_HOST="${ROSCAR_SSH_HOST:-roscar-wifi}"
REMOTE_ROOT="${ROSCAR_REMOTE_ROOT:-/home/wheeltec/ROSCAR}"
COMMAND="${1:-start}"

if [[ ! "$REMOTE_ROOT" =~ ^/[A-Za-z0-9_./-]+$ ]] || [[ "$REMOTE_ROOT" =~ (^|/)\.\.(/|$) ]]; then
  printf 'ROSCAR_REMOTE_ROOT 必须是不含空格或 .. 的绝对路径。\n' >&2
  exit 2
fi

case "$COMMAND" in
  start|stop|restart|status|logs) ;;
  *)
    printf '用法：bash scripts/route_a_remote.sh {start|stop|restart|status|logs}\n' >&2
    exit 2
    ;;
esac

# Fail explicitly on an old deployment instead of reporting skeleton startup as red.
printf -v RUN_COMMAND \
  'if [ ! -x %q ] || ! grep -q run_red_foxglove.sh %q; then printf "远端尚未部署新版红色方案 A，请先同步并构建。\\n" >&2; exit 1; fi; cd %q && bash scripts/route_a.sh %q' \
  "$REMOTE_ROOT/scripts/run_red_foxglove.sh" "$REMOTE_ROOT/scripts/route_a.sh" "$REMOTE_ROOT" "$COMMAND"

# Values expanded here are constrained above; one SSH call also means one password prompt.
# shellcheck disable=SC2029
if ! ssh -o ConnectTimeout=5 "$SSH_HOST" "$RUN_COMMAND"; then
  printf '远程操作失败。请确认小车已开机、SSH 可达且代码已同步到 %s。\n' "$REMOTE_ROOT" >&2
  exit 1
fi
