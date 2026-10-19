#!/usr/bin/env bash
# Demo worktree only; each relative action ends in IDLE and needs its own authorization.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WITH_DEMO_MOTION=true
# Continuous following uses detection/VitTrack; ReID only re-acquires a lost owner.
export REID_ENABLED=true REID_REACQUIRE_ENABLED=true SOT_ENABLED=true
# First claim and every expired-owner claim are explicit; never auto-pick another person.
export AUTO_LOCK_SINGLE=false OWNER_LOST_TIMEOUT_S=20.0
export FOLLOW_PAUSE_ON_FALL=true FOLLOW_TARGET_GRACE_S=0.6
export FOLLOW_ACKERMANN=true FOLLOW_MIN_TURN_RADIUS_M="${FOLLOW_MIN_TURN_RADIUS_M:-0.4}"
export POSE_CONFIG="${POSE_CONFIG:-$ROOT/ros2_ws/src/perception_bringup/config/pose_fixed_27cm_demo.yaml}"
export MOTION_ENABLED="${MOTION_ENABLED:-false}"
export FOLLOW_AUTO_ARM=false FOLLOW_TARGET_AUTO_RECOVER="$MOTION_ENABLED"
export FOLLOW_TARGET_DISTANCE_M="${FOLLOW_TARGET_DISTANCE_M:-1.3}"
export FOLLOW_MAX_LINEAR_MPS="${FOLLOW_MAX_LINEAR_MPS:-0.35}"
exec bash "$ROOT/scripts/run_robot_onboard.sh" "$@"
