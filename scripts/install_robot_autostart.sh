#!/usr/bin/env bash
# Install the verified robot deployment profile (run on Jetson).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$ROOT" != /home/wheeltec/ROSCAR-red ]]; then
  echo '此服务固定部署到 /home/wheeltec/ROSCAR-red，请在该目录运行。' >&2
  exit 1
fi
sudo systemctl disable --now roscar-route-a.service
sudo install -m 644 "$ROOT/deploy/systemd/roscar-robot.service" /etc/systemd/system/roscar-robot.service
sudo systemctl daemon-reload
sudo systemctl enable --now roscar-robot.service
sudo systemctl status --no-pager roscar-robot.service
