#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PERCEPTION_ROUTE=c
export PERCEPTION_INSTALL="${PERCEPTION_INSTALL:-$ROOT/install}"
exec bash "$ROOT/scripts/start_project.sh" "$@"
