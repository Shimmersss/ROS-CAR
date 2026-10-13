#!/usr/bin/env bash
# Local-only reproducible usb_cam 0.8.1 timestamp correction. No system/Jetson changes.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="$ROOT/artifacts/capture-usb-cam"
SHA=cb2ae6bc0a312de629f84fef933fb3e07dac1ef8
mkdir -p "$DIR/src"
if [[ ! -d "$DIR/src/usb_cam/.git" ]]; then
  timeout 60 git clone --depth 1 --branch 0.8.1 https://github.com/ros-drivers/usb_cam.git "$DIR/src/usb_cam"
fi
[[ "$(git -C "$DIR/src/usb_cam" rev-parse HEAD)" == "$SHA" ]] || { echo 'usb_cam 源码版本不符，拒绝修改。' >&2; exit 1; }
python3 - "$DIR/src/usb_cam/include/usb_cam/utils.hpp" <<'PY'
from pathlib import Path
import sys
path=Path(sys.argv[1]);text=path.read_text()
bad='epoch_time.tv_sec * 1000000 + epoch_time.tv_usec / 1000.0;'
good='epoch_time.tv_sec * 1000000 + epoch_time.tv_usec;'
if text.count(bad)==1:
    path.write_text(text.replace(bad,good))
elif text.count(good)!=1:
    raise SystemExit('Timestamp implementation differs; manual review required')
PY
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=190 ROS_LOCALHOST_ONLY=1
cd "$DIR"
colcon build --base-paths src --build-base build --install-base install \
  --executor sequential --cmake-args -DBUILD_TESTING=OFF
printf '%s\n' "$SHA epoch_time.tv_usec: keep microsecond units" > "$DIR/timestamp-fix.txt"
echo '已构建本机独立 usb_cam overlay；本地联调入口自动加载。'
