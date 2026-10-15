#!/usr/bin/env python3
"""Bounded sequential gateway probe; stores only metrics unless --save-dir is explicit."""
import argparse
import base64
from datetime import datetime,timezone
import hashlib
import http.client
import json
import math
from pathlib import Path
import time
from urllib.parse import urlsplit
import uuid


def percentile(values,q):
    ordered=sorted(values)
    point=(len(ordered)-1)*q
    lo,hi=math.floor(point),math.ceil(point)
    return ordered[lo]+(ordered[hi]-ordered[lo])*(point-lo)


def main():
    p=argparse.ArgumentParser();p.add_argument('--url',default='http://127.0.0.1:8200')
    p.add_argument('--token-file',type=Path,required=True);p.add_argument('--image',type=Path,required=True)
    p.add_argument('--samples',type=int,default=20);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--save-dir',type=Path);args=p.parse_args()
    if not 1<=args.samples<=100:raise ValueError('samples_out_of_bounds')
    url=urlsplit(args.url)
    if url.scheme not in ('http','https') or not url.hostname or url.username or url.password or url.query or url.fragment or url.path not in ('','/'):
        raise ValueError('invalid_gateway_url')
    token=args.token_file.read_text().strip();image=args.image.read_bytes()
    if len(image)>12*1024*1024:raise ValueError('input_too_large')
    def request(method,path,raw=None,headers=None):
        cls=http.client.HTTPSConnection if url.scheme=='https' else http.client.HTTPConnection
        conn=cls(url.hostname,url.port,timeout=150.)
        try:
            conn.request(method,path,raw,dict(Authorization='Bearer '+token,**(headers or {})))
            response=conn.getresponse();body=response.read(8*1024*1024+1)
            if len(body)>8*1024*1024:raise ValueError('response_too_large')
            if response.status!=200:raise ValueError('http_'+str(response.status))
            return json.loads(body)
        finally:conn.close()
    report=dict(scope='Synthetic or archived input; transport and latency only, not detection accuracy.',
                image_sha256=hashlib.sha256(image).hexdigest(),before=request('GET','/health/ready'),backends={})
    for kind in ('face','tongue','helmet','pcb'):
        rows=[]
        for index in range(1 if kind=='face' else args.samples):
            request_id='bench-'+uuid.uuid4().hex;task_id='bench-'+kind;capture_id='cap-'+uuid.uuid4().hex
            headers={'Content-Type':'image/png' if image.startswith(b'\x89PNG') else 'image/jpeg','X-Request-Id':request_id,'X-Task-Id':task_id,
                'X-Capture-Id':capture_id,'X-Robot-Id':'benchmark','X-Capture-Time':datetime.now(timezone.utc).isoformat()}
            start=time.monotonic();value=request('POST','/v1/infer/'+kind,image,headers)
            elapsed=(time.monotonic()-start)*1000
            assert (value['request_id'],value['task_id'],value['capture_id'])==(request_id,task_id,capture_id)
            annotation=value.get('annotated_image')
            if annotation:
                jpeg=base64.b64decode(annotation['data_b64'],validate=True)
                assert jpeg.startswith(b'\xff\xd8') and hashlib.sha256(jpeg).hexdigest()==annotation['sha256']
                if index==0 and args.save_dir:
                    args.save_dir.mkdir(parents=True,exist_ok=True)
                    (args.save_dir/(kind+'-annotated.jpg')).write_bytes(jpeg)
            rows.append(dict(status=value['status'],total_ms=elapsed,server_timings_ms=value['timings_ms']))
            if index==0:
                clean=dict(value)
                if annotation:clean['annotated_image']={k:v for k,v in annotation.items() if k!='data_b64'}
                report['backends'][kind]=dict(example=clean)
        totals=[r['total_ms'] for r in rows]
        report['backends'][kind].update(samples=rows,p50_ms=percentile(totals,.5),p95_ms=percentile(totals,.95))
    report['after']=request('GET','/health/ready')
    args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({kind:dict(statuses=sorted({r['status'] for r in v['samples']}),p50_ms=v['p50_ms'],p95_ms=v['p95_ms']) for kind,v in report['backends'].items()}))


if __name__=='__main__':
    main()
