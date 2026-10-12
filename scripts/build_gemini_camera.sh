#!/usr/bin/env bash
# Isolated, pinned official legacy Gemini ROS2 driver. No camera is started.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REV=f7e71d9ce806e788cb48d8580aac2c778fba4214
WS="$ROOT/ros2_ws/gemini_driver"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
mkdir -p "$WS/src"
if [[ ! -d "$WS/src/ros2_astra_camera/.git" ]]; then
  git clone https://github.com/orbbec/ros2_astra_camera.git "$WS/src/ros2_astra_camera"
fi
[[ -z "$(git -C "$WS/src/ros2_astra_camera" status --porcelain)" ]] || { echo '相机驱动有本地修改，保留并退出。' >&2; exit 1; }
git -C "$WS/src/ros2_astra_camera" checkout --detach "$REV"
sudo apt-get update
# The upstream package.xml omits these CMake dependencies.
sudo apt-get install -y pkg-config libuvc-dev libgoogle-glog-dev libgflags-dev libeigen3-dev nlohmann-json3-dev \
  ros-humble-tf2-eigen ros-humble-tf2-sensor-msgs
rosdep install --from-paths "$WS/src" --ignore-src --rosdistro humble -y
sudo install -m 0644 "$ROOT/deploy/udev/60-roscar-gemini.rules" /etc/udev/rules.d/60-roscar-gemini.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
case " $(id -nG) " in
  *' video '*) ;;
  *) sudo usermod -aG video "$(id -un)"; echo '已加入 video 组；重新登录后启动 Gemini。' ;;
esac
bash "$ROOT/scripts/prepare_gemini_pkgconfig.sh" "$WS/pkgconfig"
export PKG_CONFIG_PATH="$WS/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
cd "$WS"
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
