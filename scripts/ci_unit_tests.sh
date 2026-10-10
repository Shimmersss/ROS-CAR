#!/usr/bin/env bash
# Pure-Python unit tests of the active ROS packages and the recording tools: no ROS runtime,
# hardware, model weights or recordings. Run by .github/workflows/unit-tests.yml; locally:
#   PYTHON=.venv/bin/python scripts/ci_unit_tests.sh
# Passing here is not a colcon build, a ROS runtime test or a hardware check.
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1
PY=${PYTHON:-python3}
# Packages are tested from their own directory: make a relative interpreter path absolute.
case $PY in */*) PY="$(cd "$(dirname "$PY")" && pwd)/$(basename "$PY")" ;; esac
# These import rclpy; they belong to the ROS runtime checks (Docker/Jetson), not this job.
NEEDS_ROS=(navigation_bringup/test/test_status_validation.py xfyun_speech/test/test_asr_timeout.py)
status=0
for dir in ros2_ws/src/*/test; do
  pkg=${dir%/test}
  name=$(basename "$pkg")
  [ -f "$pkg/COLCON_IGNORE" ] && continue
  ignore=()
  for f in "${NEEDS_ROS[@]}"; do [[ $f == "$name"/* ]] && ignore+=(--ignore "${f#"$name"/}"); done
  echo "== $name"
  (cd "$pkg" && PYTHONPATH=. "$PY" -m pytest -q -p no:cacheprovider test ${ignore[@]+"${ignore[@]}"})
  code=$?
  # 5: every test of the package needs ROS and was ignored.
  if [ $code -ne 0 ] && [ $code -ne 5 ]; then status=1; fi
done
echo "== recording tools"
"$PY" tests/test_recording_tools.py || status=1
exit $status
