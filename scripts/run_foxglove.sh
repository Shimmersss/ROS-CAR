#!/usr/bin/env bash
# Manual, localhost-only bridge; does not launch camera or chassis nodes.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  source "$ROOT/ros2_ws/install/setup.bash"
fi
BRIDGE_PREFIX="$ROOT/tools/foxglove-root/opt/ros/humble"
if [[ ! -x "$BRIDGE_PREFIX/lib/foxglove_bridge/foxglove_bridge" ]]; then
  echo '缺少已核验的 Foxglove 用户目录运行环境，见 deploy/README.md。' >&2
  exit 1
fi
export AMENT_PREFIX_PATH="$BRIDGE_PREFIX:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$BRIDGE_PREFIX/lib:${LD_LIBRARY_PATH:-}"
exec "$BRIDGE_PREFIX/lib/foxglove_bridge/foxglove_bridge" --ros-args \
  -p address:=127.0.0.1 -p port:=8765 "$@"
