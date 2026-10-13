"""Actual Jetson TensorRT FP32 vs CPU OSNet; synthetic crops, not identity accuracy."""
from pathlib import Path
import time,json
import numpy as np
from yolo_person_tracker.reid_backend import OSNetBackend


def main():
    root=Path(__file__).resolve().parents[1]/'models/weights'
    cpu=OSNetBackend(str(root/'osnet_x1_0_msmt17.onnx'))
    gpu=OSNetBackend(str(root/'osnet_x1_0_msmt17-fp32.engine'))
    rng=np.random.default_rng(52)
    crops=[rng.integers(0,256,(height,width,3),dtype=np.uint8) for height,width in [(256,128),(180,90),(320,240),(50,100)]]
    crops.extend([np.zeros((128,64,3),np.uint8),np.full((128,64,3),255,np.uint8)])
    expected=cpu.extract(crops);actual=gpu.extract(crops)
    errors=[]
    for a,b in zip(expected,actual):
        np.testing.assert_allclose(a,b,atol=2e-5,rtol=2e-4)
        errors.append(float(np.max(np.abs(a-b))))
    timings={}
    for name,model in [('cpu',cpu),('tensorrt',gpu)]:
        samples=[]
        for _ in range(20):
            t=time.perf_counter();model.extract([crops[0]]);samples.append((time.perf_counter()-t)*1000)
        timings[name]={'mean_ms':float(np.mean(samples)),'p95_ms':float(np.percentile(samples,95))}
    print(json.dumps({'PASS':'6 crop descriptor equivalence and finite 512-D outputs','max_error':max(errors),'benchmark':timings}))
if __name__=='__main__':main()
