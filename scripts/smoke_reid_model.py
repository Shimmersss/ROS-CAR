#!/usr/bin/env python3
"""Real OSNet + YOLO Pose CPU smoke on a local image; static image is NOT tracking accuracy."""
import argparse
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/yolo_person_tracker'))
from yolo_person_tracker.reid_backend import OSNetBackend,appearance_crop
from yolo_person_tracker.reid import IdentityManager,distance

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--image',type=Path,required=True)
p.add_argument('--pose-model',type=Path,default=ROOT/'models/weights/yolo26s-pose.pt')
p.add_argument('--reid-model',type=Path,default=ROOT/'models/weights/osnet_x0_25_msmt17.onnx')
p.add_argument('--output',type=Path,default=ROOT/'artifacts/reid-smoke')
a=p.parse_args()
if not a.pose_model.is_file():raise ValueError('Explicit local Pose model required')
from ultralytics import YOLO
image=cv2.imread(str(a.image))
if image is None:raise ValueError('Image cannot be decoded')
r=YOLO(str(a.pose_model)).predict(image,device='cpu',classes=[0],verbose=False)[0]
people=[]
for i,(box,kp) in enumerate(zip(r.boxes.xyxy.cpu().numpy(),r.keypoints.data.cpu().numpy())):
    people.append(NS(track_id=f'0:{i}',confidence=float(r.boxes.conf[i]),box=box,keypoints_2d=[NS(x=float(x),y=float(y)) for x,y,_ in kp],
                     keypoint_confidences=[float(c) for _,_,c in kp]))
backend=OSNetBackend(str(a.reid_model));features={};rows=[];crops={};fixtures=[]
a.output.mkdir(parents=True,exist_ok=True)
for person in people:
    crop,reason=appearance_crop(image,person,people)
    row={'track':person.track_id,'quality':reason}
    # Independently exercise actual descriptors on isolated person crops.
    # This diagnostic does NOT override full-scene overlap rejection.
    isolated,_=appearance_crop(image,person,[person])
    if isolated is not None:
        crop=isolated
        start=time.perf_counter();feature=backend.extract([crop])[0]
        row['cpu_ms']=(time.perf_counter()-start)*1000
        row['norm']=float(np.linalg.norm(feature));features[person.track_id]=feature
        row['descriptor_scope']='isolated-crop diagnostic; full-scene quality unchanged'
        filename='person_'+person.track_id.replace(':','_')+'.png'
        cv2.imwrite(str(a.output/filename),crop);crops[filename]=feature
        x1,y1=map(int,person.box[:2])
        fixtures.append(dict(file=filename,confidence=person.confidence,
            keypoints=[[float(p.x-x1),float(p.y-y1),float(c)]
                       for p,c in zip(person.keypoints_2d,person.keypoint_confidences)]))
    rows.append(row)
assert features,'No quality-approved real person crops'
np.savez(a.output/'features.npz',**crops)
(a.output/'fixtures.json').write_text(json.dumps(fixtures,indent=2)+'\n')
m=IdentityManager()
for i in range(3):registered=m.update(10.+.2*i,features)
m.update(10.6,{})
renamed={f'1:{k.split(":")[1]}':v for k,v in features.items()}
for i in range(3):recovered=m.update(10.8+.2*i,renamed)
report=dict(evidence='Real pretrained models on static photo; repeated observations and disappearance are scripted, not real occlusion/re-ID accuracy',
            image=str(a.image),people=len(people),crops=rows,
            association_scope='controlled isolated descriptors; not full-scene end-to-end acceptance',
            pairwise=[{'a':x,'b':y,'distance':distance(features[x],features[y])}
                      for x in features for y in features if x<y],
            registered=[vars(x) for x in registered],recovered=[vars(x) for x in recovered])
# NaN diagnostic distances are represented explicitly by null in JSON evidence.
for group in ('registered','recovered'):
    for item in report[group]:
        if not np.isfinite(item['distance']):item['distance']=None
(a.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
print(json.dumps(report,indent=2,allow_nan=False))
