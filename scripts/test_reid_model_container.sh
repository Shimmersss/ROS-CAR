#!/usr/bin/env bash
# Actual model is explicit and required; no skipped model validation or automatic download.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ -f "$ROOT/models/weights/osnet_x0_25_msmt17.onnx" && -f "$ROOT/artifacts/reid-smoke/fixtures.json" ]] || {
  echo 'Run prepare_reid_model.py and smoke_reid_model.py first.' >&2; exit 1;
}
docker run --rm --platform linux/arm64 -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=200 \
  -v "$ROOT/ros2_ws/src:/workspace/ros2_ws/src:ro" -v "$ROOT/tests:/workspace/tests:ro" \
  -v "$ROOT/models/weights:/models:ro" -v "$ROOT/artifacts/reid-smoke:/evidence:ro" \
  "${ROSCAR_C_TEST_IMAGE:-roscar-humble-test}" bash -c '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --base-paths src --packages-up-to yolo_person_tracker --event-handlers console_cohesion+
    source install/setup.bash
    python3 /workspace/tests/test_reid_model_runtime.py --model /models/osnet_x0_25_msmt17.onnx --evidence /evidence
  ' 2>&1 | tee "$ROOT/artifacts/reid-arm64-model.log"
