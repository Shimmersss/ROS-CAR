#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
exec "${PYTHON_BIN:-/root/roscar-models/venv-yolo/bin/python}" -m services.inference_gateway.gateway --config "${1:?provide a private gateway configuration}"
