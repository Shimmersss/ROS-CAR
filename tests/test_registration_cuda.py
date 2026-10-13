"""Actual CUDA vs NumPy projection on synthetic geometry; no physical calibration claim."""
from pathlib import Path
import json
import time
import numpy as np
from yolo_person_tracker.registration import load_registration


def main():
    calibration=Path(__file__).resolve().parents[1]/'ros2_ws/src/perception_bringup/config/gemini_AY2755200PW.json'
    ref=load_registration(calibration,'AY2755200PW')
    gpu=load_registration(calibration,'AY2755200PW','cuda')
    rng=np.random.default_rng(25)
    image=rng.integers(0,256,(480,640,3),dtype=np.uint8)
    scenes=[np.full((400,640),z,np.float32) for z in (0.,.2,.201,1.5,8.,8.01)]
    random=rng.uniform(.1,8.1,(400,640)).astype(np.float32)
    random.flat[::17]=np.nan;random.flat[1::17]=np.inf;random.flat[2::17]=0.
    scenes.append(random)
    foreground=np.full((400,640),3.,np.float32);foreground[80:300,200:450]=1.;scenes.append(foreground)
    for depth in scenes:
        c,z=ref.apply(image,depth);cg,zg=gpu.apply(image,depth)
        np.testing.assert_array_equal(c,cg)
        np.testing.assert_array_equal(z,zg)
    for method in (ref,gpu):
        try:method.apply(image,np.zeros((3,4),np.float32))
        except ValueError:pass
        else:raise AssertionError('Bad dimensions were accepted')
    timings={}
    for name,method in [('numpy',ref),('cuda',gpu)]:
        samples=[]
        for _ in range(20):
            start=time.perf_counter();method.apply(image,foreground);samples.append((time.perf_counter()-start)*1000)
        timings[name]={'mean_ms':float(np.mean(samples)),'p95_ms':float(np.percentile(samples,95))}
    print(json.dumps({'PASS':'8 scenes exact pixels/depth incl invalid/range/occlusion; dimension rejection','benchmark':timings}))

if __name__=='__main__':main()
