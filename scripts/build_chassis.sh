#!/usr/bin/env bash
# Build a disposable copy: preserve COLCON_IGNORE in the default source workspace.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
set -u
STAGE="$ROOT/ros2_ws/chassis_src"
mkdir -p "$STAGE"
for package in serial wheeltec_robot_msg turn_on_wheeltec_robot; do
  rm -rf "${STAGE:?}/$package"
  cp -R "$ROOT/ros2_ws/src/chassis_vendor/$package" "$STAGE/$package"
  rm -f "$STAGE/$package/COLCON_IGNORE"
done
cd "$ROOT/ros2_ws"
colcon --log-base chassis_log build --base-paths "$STAGE" \
  --build-base chassis_build --install-base chassis_install \
  --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=OFF
