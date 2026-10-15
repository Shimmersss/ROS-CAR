#!/usr/bin/env python3
"""Compare frozen-model outputs without changing thresholds or source files."""
import argparse
import json
import math
from pathlib import Path


def compare(reference, candidate):
    if reference['pipeline_digest']!=candidate['pipeline_digest'] or reference['pipeline_id']!=candidate['pipeline_id']:
        raise ValueError('pipeline_mismatch')
    if len(reference['samples'])!=len(candidate['samples']) or not reference['samples']:
        raise ValueError('sample_count_mismatch')
    maximum=0.;problems=[]
    for index,(cpu,gpu) in enumerate(zip(reference['samples'],candidate['samples'])):
        if cpu['image_sha256']!=gpu['image_sha256']:
            raise ValueError('input_mismatch')
        for field in ('status','classification_performed'):
            if cpu[field]!=gpu[field]:problems.append(f'{index}:{field}')
        for section in ('labels','pipeline_labels'):
            if set(cpu[section])!=set(gpu[section]):
                problems.append(f'{index}:{section}:labels');continue
            for label,item in cpu[section].items():
                other=gpu[section][label]
                delta=abs(item['score']-other['score'])
                if not math.isfinite(delta):raise ValueError('nonfinite_delta')
                maximum=max(maximum,delta)
                if delta>1e-3 or item['decision']!=other['decision'] or item['threshold']!=other['threshold']:
                    problems.append(f'{index}:{section}:{label}')
    return dict(status='PASS' if not problems else 'FAIL',max_abs_score_delta=maximum,
                tolerance=1e-3,problems=problems,samples=len(reference['samples']),
                scope='Named nonclinical inputs only; human-material accuracy/retake validation not performed.')


def main():
    p=argparse.ArgumentParser();p.add_argument('reference',type=Path);p.add_argument('candidate',type=Path);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args();result=compare(json.loads(args.reference.read_text()),json.loads(args.candidate.read_text()))
    args.out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    raise SystemExit(0 if result['status']=='PASS' else 1)


if __name__=='__main__':
    main()
