#!/usr/bin/env python3
"""Plan/export YOLO26s FP16 NMS-free TensorRT on the deployment Jetson itself."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import tempfile

ROOT = Path(__file__).resolve().parents[1]
EXPORT_ARGS = dict(format='engine', imgsz=640, batch=1, dynamic=False,
                   quantize=16, nms=False, device=0, workspace=2,
                   simplify=False, opset=17)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Actually build on target Jetson; otherwise print plan')
    args = parser.parse_args()
    source = ROOT/'models/weights/yolo26s.pt'
    target = ROOT/'models/weights/yolo26s-fp16.engine'
    report_path = target.with_suffix('.engine.json')
    if not args.execute:
        print(json.dumps(dict(source=str(source), target=str(target), export_args=EXPORT_ARGS,
                              requires='Target Jetson, working CUDA torch, JetPack TensorRT 10.x, onnx; no remote actions'), indent=2))
        return
    if platform.system() != 'Linux' or platform.machine() != 'aarch64' or not Path('/etc/nv_tegra_release').is_file():
        raise SystemExit('Execute on the deployment Jetson; a Mac-built artifact is not a Jetson TensorRT engine.')
    if target.exists() or report_path.exists():
        raise SystemExit('Existing engine/report found; preserve or move it before rebuilding.')
    entry = next(item for item in json.loads((ROOT/'models/manifest.json').read_text())['models']
                 if item['name'] == 'yolo26s.pt')
    if not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest() != entry['sha256']:
        raise SystemExit('Run scripts/prepare_model.py first; the official checkpoint must match its manifest.')
    # Do not let Ultralytics install or replace the target CUDA/TensorRT environment.
    os.environ['YOLO_AUTOINSTALL'] = 'false'
    import torch
    import tensorrt
    import onnx
    import ultralytics
    if ultralytics.__version__ != '8.4.156':
        raise SystemExit('Use the pinned Ultralytics 8.4.156 environment.')
    if not torch.cuda.is_available() or tensorrt.__version__.split('.')[0] != '10':
        raise SystemExit('Require working CUDA torch and the JetPack-supplied TensorRT 10.x.')
    from ultralytics import YOLO
    import numpy as np
    # Stage in an isolated directory; publish the engine only after a load/inference smoke test.
    with tempfile.TemporaryDirectory(prefix='yolo26-export-', dir=target.parent) as folder:
        staged = Path(folder)/'yolo26s-fp16.pt'
        shutil.copyfile(source, staged)
        built = Path(YOLO(str(staged), task='detect').export(**EXPORT_ARGS))
        engine = YOLO(str(built), task='detect')
        engine.track(np.zeros((640,640,3), dtype=np.uint8), device=0, imgsz=640,
                     tracker='bytetrack.yaml', persist=True, classes=[0], conf=.1,
                     nms=False, rect=False, verbose=False)
        if (not engine.predictor.model.end2end
                or engine.predictor.model.metadata.get('args', {}).get('quantize') != 16):
            raise RuntimeError('Engine lacks FP16 export metadata or end-to-end output')
        report = dict(created_utc=datetime.now(timezone.utc).isoformat(),
                      source_sha256=entry['sha256'], engine_sha256=hashlib.sha256(built.read_bytes()).hexdigest(),
                      export_args=EXPORT_ARGS, input_binding_fp16=bool(engine.predictor.model.fp16),
                      precision_note='FP16 builder enabled; input binding and some layers may remain FP32',
                      gpu=torch.cuda.get_device_name(0),
                      gpu_capability=list(torch.cuda.get_device_capability(0)),
                      torch=torch.__version__, cuda=torch.version.cuda, tensorrt=tensorrt.__version__,
                      ultralytics=ultralytics.__version__, onnx=onnx.__version__,
                      l4t=Path('/etc/nv_tegra_release').read_text().strip(),
                      validation='Local engine load and blank-image tracking only; no person/camera acceptance')
        shutil.copyfile(built, target)
        report_path.write_text(json.dumps(report, indent=2)+'\n')
    print(f'Created {target}\nBuild record: {report_path}')


if __name__ == '__main__':
    main()
