#!/usr/bin/env bash
# Run on the Mac. Keeps one local tunnel open for the manually started bridge.
set -euo pipefail
LOCAL_PORT="${1:-8766}"
if [[ ! "$LOCAL_PORT" =~ ^[0-9]+$ ]] || (( LOCAL_PORT < 1024 || LOCAL_PORT > 65535 )); then
  echo '本地端口必须在 1024–65535 范围内。' >&2
  exit 1
fi
exec ssh -fNT -o ExitOnForwardFailure=yes -o ConnectTimeout=10 \
  -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
  -L "127.0.0.1:${LOCAL_PORT}:127.0.0.1:8765" roscar-wifi
