#!/usr/bin/env bash
# Shared, explicitly configured demo profile. Source only; never print private env.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set -a
source "${ROSCAR_ROBOT_ENV:-$HOME/.config/roscar/robot.env}"
set +a
export ROSCAR_WS="$ROOT/ros2_ws" ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}" ROS_LOCALHOST_ONLY=0
export POSE_CONFIG="$ROOT/ros2_ws/src/perception_bringup/config/pose_fixed_27cm_demo.yaml"
export GEMINI_CALIBRATION="${GEMINI_CALIBRATION:-$ROOT/ros2_ws/src/perception_bringup/config/gemini_AY2755200PW.json}"
export REID_ENABLED=true REID_REACQUIRE_ENABLED=true REID_LOCK_ENABLED=false SOT_ENABLED=true
export REID_CONFIG="$ROOT/ros2_ws/src/perception_bringup/config/reid_owner_trial.yaml"
export AUTO_LOCK_SINGLE=false OWNER_LOST_TIMEOUT_S=20.0 PERCEPTION_AUXILIARY=false
export GEMINI_INPROCESS_REGISTRATION=true GEMINI_REGISTRATION_BACKEND=cuda DEPTH_REGISTERED=true
export FOLLOW_PAUSE_ON_FALL=true FOLLOW_TARGET_GRACE_S=0.6 FOLLOW_ACKERMANN=true
export FOLLOW_TARGET_DISTANCE_M=1.3 FOLLOW_MAX_LINEAR_MPS=0.2 CHASSIS_MAX_LINEAR_MPS=0.2
export FOLLOW_DISTANCE_GAIN=1.0 FOLLOW_BEARING_GAIN=1.6 FOLLOW_MIN_TURN_RADIUS_M=0.4
export MOTION_ENABLED=true FOLLOW_AUTO_ARM=false FOLLOW_TARGET_AUTO_RECOVER=true
export VOICE_BACKEND=offline ASR_BACKEND=offline VOICE_TTS_ENABLED=true VOICE_ESTOP_ENABLED=false
export FOLLOW_EMERGENCY_KWS_REQUIRED=false SERIAL_PORT="${SERIAL_PORT:-/dev/wheeltec_controller}" CAR_MODE=mini_akm
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 OPENCV_FOR_THREADS_NUM=1
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
# Explicit known deployment endpoints; tokens remain in 0600 private files.
export CAPTURE_INFERENCE_URL="${CAPTURE_INFERENCE_URL:-http://192.168.31.149:8200}"
export CAPTURE_INFERENCE_TOKEN_FILE="${CAPTURE_INFERENCE_TOKEN_FILE:-$HOME/.config/roscar/gateway.token}"
export CAPTURE_CLOUD_URL="${CAPTURE_CLOUD_URL:-http://192.168.31.149:5011}"
export CAPTURE_TOKEN_FILE="${CAPTURE_TOKEN_FILE:-$HOME/.config/roscar/c518-capture.token}"
