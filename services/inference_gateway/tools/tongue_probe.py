#!/usr/bin/env python3
"""Probe the untouched tongue package in its own interpreter, GPU or CPU reference."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--package',type=Path,required=True)
    parser.add_argument('--device',choices=('cpu','cuda'),required=True)
    parser.add_argument('--image',type=Path,action='append',required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    import numpy as np
    import torch
    import torchvision
    from PIL import Image,ImageOps
    torch.set_num_threads(2)
    root=args.package.resolve()
    sys.path.insert(0,str(root/'code/deploy'))
    from cloud.server import TongueService,Response
    manifest=json.loads((root/'code/deploy/configs/cloud/pipeline_r50_384_cvAC10_passthrough_debug.json').read_text())
    expected=json.loads((root/'integration/expected_pipeline.json').read_text())
    assert manifest['pipeline_digest']==expected['pipeline_digest']
    start=time.monotonic()
    service=TongueService(manifest,root/'code',device=args.device,workers=1,max_queue=2,deadline_ms=120000)
    ready,checks=service.ready()
    assert ready,checks
    report=dict(device=args.device,torch=torch.__version__,torchvision=torchvision.__version__,
        pipeline_id=manifest['pipeline_id'],pipeline_digest=manifest['pipeline_digest'],
        samples=[],scope='Numerical comparison on named inputs only; not clinical accuracy validation.')
    for path in args.image:
        raw=path.read_bytes()
        with Image.open(path) as pil:rgb=np.asarray(ImageOps.exif_transpose(pil).convert('RGB'))
        began=time.monotonic()
        classification=service.pipeline.classifier.predict(Image.fromarray(rgb))
        pipeline=service.pipeline.analyse(rgb)
        response=Response('numerical-probe',time.monotonic(),time.monotonic()+120)
        response.started_at=time.monotonic()
        service.reply_for(pipeline,response,dict(capture_id='probe-capture',session_id='probe-session'))
        labels={k:dict(score=float(v['probability']),threshold=float(v['threshold']),
                       decision='positive' if v['probability']>=v['threshold'] else 'negative')
                for k,v in classification['labels'].items()}
        assert set(labels)==set(expected['labels'])
        report['samples'].append(dict(image_sha256=hashlib.sha256(raw).hexdigest(),
            labels=labels,status=response.payload['status'],
            classification_performed=response.payload['classification_performed'],
            pipeline_labels=response.payload['labels'],seconds=time.monotonic()-began))
    report['seconds']=time.monotonic()-start
    if args.device=='cuda':
        report['cuda_capability']=torch.cuda.get_device_capability()
        report['peak_allocated_bytes']=torch.cuda.max_memory_allocated()
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(status='complete',device=args.device,samples=len(report['samples']),seconds=report['seconds'])))


if __name__=='__main__':
    main()
