#!/usr/bin/env bash
# Explicit target-device FP32 build. Never starts or authorizes the robot.
set -euo pipefail
[[ $# == 2 ]] || { echo 'Usage: build_vittrack_engine.sh SOURCE.onnx OUTPUT.engine' >&2; exit 2; }
SOURCE_MODEL="$1"
OUTPUT_ENGINE="$2"
TRTEXEC_BIN="${TRTEXEC_BIN:-/usr/src/tensorrt/bin/trtexec}"
python3 - "$SOURCE_MODEL" "$OUTPUT_ENGINE" <<'PY'
import sys,json,hashlib
from pathlib import Path
source,destination=map(Path,sys.argv[1:])
if source.suffix!='.onnx' or destination.suffix!='.engine':raise SystemExit('Require .onnx source and .engine output')
if destination.exists() or Path(str(destination)+'.json').exists():raise SystemExit('Output already exists; choose a new destination')
metadata=json.loads(Path(str(source)+'.json').read_text())
if metadata.get('architecture')!='vittrack_2023sep' or metadata.get('sha256')!=hashlib.sha256(source.read_bytes()).hexdigest():raise SystemExit('VitTrack source metadata/hash mismatch')
if source.parent.resolve()!=destination.parent.resolve():raise SystemExit('Engine and source must share a directory for runtime provenance checks')
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
