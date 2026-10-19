#!/usr/bin/env bash
# Demo boot is on demand; voice and chassis remain resident, motion starts on ROS call.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$ROOT/scripts/run_demo_runtime.sh" "$@"
