#!/usr/bin/env python3
"""Explicit OSNet preparation; runtime never downloads. Uses the existing torch environment."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/yolo_person_tracker'))
DRIVE='https://drive.usercontent.google.com/download?id={}&export=download&confirm=t'
# Same-domain MSMT17 checkpoints from the pinned Torchreid MODEL_ZOO; hashes recorded on first download.
MODELS={
    'x0_25':dict(drive='1sSwXSUlj4_tHZequ_iZ8w_Jh0VaRQMqF',
                 sha='6f57607fed9f502b9efed546108132ee715df5a5b6e6932c6269bacb47f59f99'),
    'x1_0':dict(drive='112EMUfBPYeYg70w-syK6V6Mx8-Qb9Q1M',
                sha='b7d73dc67c016fd044e4027ff856019496392a7aca8fa0ed56d862a1632c1cf2'),
}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--download',action='store_true')
    parser.add_argument('--arch',choices=sorted(MODELS),default='x0_25')
    parser.add_argument('--checkpoint',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    name=f'osnet_{args.arch}_msmt17';CHECKPOINT_SHA=MODELS[args.arch]['sha'];URL=DRIVE.format(MODELS[args.arch]['drive'])
    args.checkpoint=args.checkpoint or ROOT/f'models/weights/{name}.pth'
    args.output=args.output or ROOT/f'models/weights/{name}.onnx'
    if args.download and not args.checkpoint.exists():
        args.checkpoint.parent.mkdir(parents=True,exist_ok=True)
        with urllib.request.urlopen(URL,timeout=60) as response:
            data=response.read(32*1024*1024)
        if hashlib.sha256(data).hexdigest()!=CHECKPOINT_SHA:
            raise ValueError('Downloaded checkpoint does not match pinned hash')
        args.checkpoint.write_bytes(data)
    if hashlib.sha256(args.checkpoint.read_bytes()).hexdigest()!=CHECKPOINT_SHA:
        raise ValueError('Not the pinned official MSMT17 ReID checkpoint')
    import torch
    import numpy as np
    from yolo_person_tracker.vendor import osnet
    from yolo_person_tracker.reid_backend import OSNetBackend
    state=torch.load(args.checkpoint,map_location='cpu',weights_only=True)
    model=getattr(osnet,f'osnet_{args.arch}')(num_classes=1041,pretrained=False).eval()
    model.load_state_dict(state,strict=True)
    torch.set_num_threads(2)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    sample=torch.zeros((1,3,256,128))
    torch.onnx.export(model,sample,str(args.output),opset_version=11,
                      input_names=['images'],output_names=['features'],dynamo=False)
    metadata=dict(architecture=name,preprocessing='rgb_256x128_imagenet',
                  sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
                  checkpoint_sha256=CHECKPOINT_SHA,checkpoint_url=URL,
                  source_commit='f8cd150fdf77e8d9e1ed143b7f308c2c609ded50')
    Path(str(args.output)+'.json').write_text(json.dumps(metadata,indent=2)+'\n')
    # Compare the actual runtime backend to original PyTorch, not merely graph export.
    rng=np.random.default_rng(10);crop=rng.integers(0,256,(256,128,3),dtype=np.uint8)
    backend=OSNetBackend(str(args.output))
    with torch.inference_mode():
        native=model(torch.from_numpy(backend.tensor(crop))).numpy().reshape(-1)
    native/=np.linalg.norm(native)
    converted=backend.extract([crop])[0]
    np.testing.assert_allclose(converted,native,atol=2e-5,rtol=1e-3)
    print(json.dumps(dict(**metadata,onnx_bytes=args.output.stat().st_size,
                         max_feature_error=float(np.max(np.abs(converted-native)))),indent=2))

if __name__=='__main__':main()
