#!/usr/bin/env bash
# Build radar packages in an overlay; never start devices or remove source ignores.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
STAGE="$ROOT/ros2_ws/radar_src"
mkdir -p "$STAGE"
rm -rf "${STAGE:?}/lslidar_ros2"
cp -R "$ROOT/ros2_ws/src/radar_vendor/lslidar_ros2" "$STAGE/lslidar_ros2"
find "$STAGE/lslidar_ros2" -name COLCON_IGNORE -delete
cd "$ROOT/ros2_ws"
CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}" \
colcon --log-base radar_log build --base-paths "$STAGE" \
  --packages-up-to lslidar_driver --executor sequential --build-base radar_build --install-base radar_install \
  --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=OFF
