#!/usr/bin/env bash
# Explicit target-device build; never download a model or change the running robot.
set -euo pipefail
[[ $# == 2 ]] || { echo 'Usage: build_reid_engine.sh SOURCE.onnx OUTPUT.engine' >&2; exit 2; }
SOURCE_MODEL="$1"
OUTPUT_ENGINE="$2"
TRTEXEC_BIN="${TRTEXEC_BIN:-/usr/src/tensorrt/bin/trtexec}"
python3 - "$SOURCE_MODEL" "$OUTPUT_ENGINE" <<'PY'
import sys,json,hashlib
from pathlib import Path
source,destination=map(Path,sys.argv[1:])
if source.suffix!='.onnx' or destination.suffix!='.engine':raise SystemExit('Require .onnx source and .engine output')
metadata=json.loads(Path(str(source)+'.json').read_text())
if metadata.get('sha256')!=hashlib.sha256(source.read_bytes()).hexdigest():raise SystemExit('Source model hash mismatch')
if metadata.get('architecture') not in ('osnet_x0_25_msmt17','osnet_x1_0_msmt17') or metadata.get('preprocessing')!='rgb_256x128_imagenet':raise SystemExit('Unsupported OSNet source metadata')
if source.resolve().parent!=destination.resolve().parent:raise SystemExit('Engine and source must share a directory for runtime provenance checks')
PY
"$TRTEXEC_BIN" --onnx="$SOURCE_MODEL" --saveEngine="$OUTPUT_ENGINE" --noTF32 --memPoolSize=workspace:128 --skipInference
python3 - "$SOURCE_MODEL" "$OUTPUT_ENGINE" <<'PY'
import sys,json,hashlib
from pathlib import Path
source,engine=map(Path,sys.argv[1:])
metadata=json.loads(Path(str(source)+'.json').read_text())
metadata.update(sha256=hashlib.sha256(engine.read_bytes()).hexdigest(),source_onnx=source.name,
                source_onnx_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),precision='FP32-no-TF32')
Path(str(engine)+'.json').write_text(json.dumps(metadata,indent=2))
PY
