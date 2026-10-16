#!/usr/bin/env bash
# Demo worktree only; each relative action ends in IDLE and needs its own authorization.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WITH_DEMO_MOTION=true
export FOLLOW_PAUSE_ON_FALL=true FOLLOW_TARGET_GRACE_S=0.0
export POSE_CONFIG="${POSE_CONFIG:-$ROOT/ros2_ws/src/perception_bringup/config/pose_fixed_27cm_demo.yaml}"
export MOTION_ENABLED="${MOTION_ENABLED:-false}"
export FOLLOW_AUTO_ARM=false FOLLOW_TARGET_AUTO_RECOVER=false
export FOLLOW_TARGET_DISTANCE_M="${FOLLOW_TARGET_DISTANCE_M:-1.3}"
export FOLLOW_MAX_LINEAR_MPS="${FOLLOW_MAX_LINEAR_MPS:-0.35}"
exec bash "$ROOT/scripts/run_robot_onboard.sh" "$@"
