#!/usr/bin/env bash
# Start only the hardened chassis driver, verify STM32 feedback, and optionally
# send one short low-speed forward pulse. This deliberately avoids launching
# person_follower so /cmd_vel has exactly one publisher during the motion test.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERIAL_PORT="${SERIAL_PORT:-/dev/wheeltec_controller}"
SERIAL_BAUD_RATE="${SERIAL_BAUD_RATE:-115200}"
SMOKE_LINEAR_MPS="${SMOKE_LINEAR_MPS:-0.05}"
SMOKE_DURATION_S="${SMOKE_DURATION_S:-0.5}"
CAR_MODE=""
TEST_MODE="check"

usage() {
  cat <<'EOF'
用法：
  bash scripts/chassis_motion_smoke_test.sh --car-mode <已核验车型> [--check|--move]

选项：
  --car-mode NAME  必填；必须与 robot_model.yaml 中的车型键一致，不能猜默认值
  --check          只打开串口并验证 STM32 回传，不发送非零速度（默认）
  --move           以 0.05 m/s 前进 0.5 秒，随后连续发送零速度并关闭驱动

可选环境变量：
  SERIAL_PORT=/dev/wheeltec_controller
  SERIAL_BAUD_RATE=115200
  SMOKE_LINEAR_MPS=0.05     # 范围 (0, 0.08]
  SMOKE_DURATION_S=0.5      # 范围 (0, 1.0]
  SMOKE_ROS_DOMAIN_ID=183     # 与在线感知域隔离

实车测试前请架空车轮或清空车辆前方空间，并准备实体断电。
EOF
}

while (($#)); do
  case "$1" in
    --car-mode)
      (($# >= 2)) || { echo '--car-mode 缺少值。' >&2; exit 2; }
      CAR_MODE="$2"
      shift 2
      ;;
    --check)
      TEST_MODE="check"
      shift
      ;;
    --move)
      TEST_MODE="move"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数：$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -n "$CAR_MODE" ]] || { echo '必须通过 --car-mode 指定已核验车型。' >&2; exit 2; }
[[ "$CAR_MODE" =~ ^[A-Za-z0-9_]+$ ]] || { echo '车型只能包含字母、数字和下划线。' >&2; exit 2; }
[[ "$SERIAL_BAUD_RATE" =~ ^[1-9][0-9]*$ ]] || { echo 'SERIAL_BAUD_RATE 必须是正整数。' >&2; exit 2; }

MODEL_CONFIG="$ROOT/ros2_ws/src/chassis_vendor/turn_on_wheeltec_robot/config/robot_model.yaml"
grep -Eq "^[[:space:]]{2}${CAR_MODE}:([[:space:]]*)$" "$MODEL_CONFIG" || {
  echo "robot_model.yaml 中没有车型：$CAR_MODE" >&2
  echo '可用车型键：' >&2
  sed -n 's/^  \([A-Za-z0-9_]*\):$/  \1/p' "$MODEL_CONFIG" >&2
  exit 2
}

source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
[[ -f "$ROOT/ros2_ws/chassis_install/setup.bash" ]] || {
  echo '缺少底盘构建；请先运行 bash scripts/build_chassis.sh。' >&2
  exit 2
}
source "$ROOT/ros2_ws/chassis_install/setup.bash"
set -u

export ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-183}"
export ROS_LOCALHOST_ONLY=1

[[ -c "$SERIAL_PORT" ]] || { echo "串口不存在或不是字符设备：$SERIAL_PORT" >&2; exit 2; }
SERIAL_OWNER="$(fuser "$SERIAL_PORT" 2>/dev/null || true)"
[[ -z "$SERIAL_OWNER" ]] || {
  echo "串口已被进程占用（PID:${SERIAL_OWNER}）；拒绝重复打开。" >&2
  exit 3
}

# The driver itself also owns an OS file lock. These graph checks provide a
# clearer refusal before opening the port or creating a second command source.
if ros2 node list 2>/dev/null | grep -Eq '(^|/)wheeltec_robot$'; then
  echo '检测到已有 wheeltec_robot 节点；拒绝重复打开串口。' >&2
  exit 3
fi
CMD_INFO="$(ros2 topic info /cmd_vel 2>/dev/null || true)"
PUBLISHER_COUNT="$(sed -n 's/^Publisher count: //p' <<<"$CMD_INFO" | tail -n 1)"
PUBLISHER_COUNT="${PUBLISHER_COUNT:-0}"
[[ "$PUBLISHER_COUNT" == 0 ]] || {
  echo "检测到 /cmd_vel 已有 ${PUBLISHER_COUNT} 个发布者；拒绝叠加测试命令。" >&2
  exit 3
}

python3 - "$SMOKE_LINEAR_MPS" "$SMOKE_DURATION_S" <<'PY'
import math
import sys

speed = float(sys.argv[1])
duration = float(sys.argv[2])
if not math.isfinite(speed) or not 0.0 < speed <= 0.08:
    raise SystemExit("SMOKE_LINEAR_MPS 必须在 (0, 0.08] m/s")
if not math.isfinite(duration) or not 0.0 < duration <= 1.0:
    raise SystemExit("SMOKE_DURATION_S 必须在 (0, 1.0] 秒")
PY

LOG_DIR="$ROOT/artifacts/chassis-smoke"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/driver-$(date -u +%Y%m%dT%H%M%SZ).log"
DRIVER_PID=""

# shellcheck disable=SC2329 # Invoked by EXIT/INT/TERM traps.
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$DRIVER_PID" ]] && kill -0 "$DRIVER_PID" 2>/dev/null; then
    kill -TERM -- "-$DRIVER_PID" 2>/dev/null || kill -TERM "$DRIVER_PID" 2>/dev/null || true
    for _ in {1..30}; do
      kill -0 "$DRIVER_PID" 2>/dev/null || break
      sleep .1
    done
    kill -KILL -- "-$DRIVER_PID" 2>/dev/null || true
    wait "$DRIVER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "启动底盘驱动：串口=$SERIAL_PORT，波特率=$SERIAL_BAUD_RATE，车型=$CAR_MODE"
setsid ros2 run turn_on_wheeltec_robot wheeltec_robot_node --ros-args \
  -p "usart_port_name:=$SERIAL_PORT" \
  -p "serial_baud_rate:=$SERIAL_BAUD_RATE" \
  -p "car_mode:=$CAR_MODE" \
  -p command_timeout_s:=0.5 \
  -p feedback_timeout_s:=0.5 \
  -p max_linear_mps:=0.15 \
  -p max_angular_rps:=0.5 >"$LOG_FILE" 2>&1 &
DRIVER_PID="$!"

for _ in {1..50}; do
  kill -0 "$DRIVER_PID" 2>/dev/null || {
    echo "底盘驱动提前退出，日志：$LOG_FILE" >&2
    tail -n 30 "$LOG_FILE" >&2
    exit 4
  }
  ros2 node list 2>/dev/null | grep -Eq '(^|/)wheeltec_robot$' && break
  sleep .1
done
ros2 node list 2>/dev/null | grep -Eq '(^|/)wheeltec_robot$' || {
  echo "等待 wheeltec_robot 节点超时，日志：$LOG_FILE" >&2
  exit 4
}

echo '等待 STM32 有效回传（PowerVoltage）……'
if ! timeout 5 ros2 topic echo /PowerVoltage std_msgs/msg/Float32 --once >/dev/null 2>&1; then
  echo "5 秒内没有收到有效底盘回传；不会发送运动命令。日志：$LOG_FILE" >&2
  exit 5
fi
echo '串口已打开并收到有效底盘回传。'

if [[ "$TEST_MODE" == check ]]; then
  echo '只读检查完成：没有发送非零速度。'
  exit 0
fi

echo "开始短距测试：前进 ${SMOKE_LINEAR_MPS} m/s，持续 ${SMOKE_DURATION_S} 秒。"
python3 - "$SMOKE_LINEAR_MPS" "$SMOKE_DURATION_S" <<'PY'
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

speed = float(sys.argv[1])
duration = float(sys.argv[2])
rclpy.init()
node = rclpy.create_node("chassis_motion_smoke_publisher")
publisher = node.create_publisher(Twist, "/cmd_vel", 10)

deadline = time.monotonic() + 3.0
while publisher.get_subscription_count() != 1 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
if publisher.get_subscription_count() != 1:
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit("/cmd_vel 订阅者数量不是 1，拒绝发送")

moving = Twist()
moving.linear.x = speed
stopped = Twist()
period = 0.05
try:
    # DDS endpoint discovery can be briefly asymmetric: the publisher may see
    # the driver before the driver's count_publishers() sees this publisher.
    # Hold an explicit zero command first so the safety check is established
    # without moving the vehicle.
    ready_end = time.monotonic() + 1.0
    while time.monotonic() < ready_end:
        publisher.publish(stopped)
        rclpy.spin_once(node, timeout_sec=period)

    end = time.monotonic() + duration
    while time.monotonic() < end:
        publisher.publish(moving)
        rclpy.spin_once(node, timeout_sec=period)
finally:
    # Hold zero longer than the driver's 0.5 s command watchdog, then closing
    # the driver sends one more basic zero frame from its destructor.
    zero_end = time.monotonic() + 0.75
    while time.monotonic() < zero_end:
        publisher.publish(stopped)
        rclpy.spin_once(node, timeout_sec=period)
    node.destroy_node()
    rclpy.shutdown()
PY

echo '短距测试完成：已连续发送零速度，正在关闭底盘驱动。'
echo "驱动日志：$LOG_FILE"
