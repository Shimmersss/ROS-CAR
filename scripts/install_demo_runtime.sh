#!/usr/bin/env bash
# Explicit onboard installer; current symlink must already select this demo deployment.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
[[ "$(readlink -f /home/wheeltec/ROSCAR-current)" == "$ROOT" ]] || { echo 'current must point to this deployment'; exit 1; }
[[ -x "$ROOT/install/demo_motion/lib/demo_motion/runtime_manager" ]] || { echo 'Build demo_motion first'; exit 1; }
config_dir="$HOME/.config/systemd/user"
backup_dir="$HOME/.config/roscar/backups/demo-runtime-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$backup_dir" "$config_dir"
chmod 700 "$backup_dir"
for item in "$config_dir/roscar-robot.service" "$config_dir/roscar-robot.service.d" \
  "/run/user/$(id -u)/systemd/user/roscar-robot.service.d"; do
  if [[ -e "$item" ]]; then
    case "$item" in /run/*) cp -a "$item" "$backup_dir/runtime-dropins";;
      *) cp -a "$item" "$backup_dir/";; esac
  fi
done
systemctl --user stop roscar-capture-inference.service c518-capture-adapter.service roscar-robot.service
systemctl --user disable roscar-capture-inference.service c518-capture-adapter.service 2>/dev/null || true
# Remove conflicting old ExecStart overrides only after saving exact originals.
[[ ! -d "$config_dir/roscar-robot.service.d" ]] || mv "$config_dir/roscar-robot.service.d" "$backup_dir/removed-persistent-dropins"
runtime_dir="/run/user/$(id -u)/systemd/user/roscar-robot.service.d"
[[ ! -d "$runtime_dir" ]] || mv "$runtime_dir" "$backup_dir/removed-runtime-dropins"
cp "$ROOT/deploy/systemd/roscar-demo-runtime.service" "$config_dir/roscar-robot.service"
systemctl --user daemon-reload
systemctl --user enable --now roscar-robot.service
printf 'Installed idle demo runtime. Backup: %s\n' "$backup_dir"
