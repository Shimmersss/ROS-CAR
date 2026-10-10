#!/usr/bin/env python3
"""Offline C replay. Host-receive pairing is diagnostic, not exposure synchronization."""
import argparse, csv, json, os, sys, time
from pathlib import Path
from dataclasses import fields
from types import SimpleNamespace as NS
os.environ['YOLO_AUTOINSTALL']='false'
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/yolo_person_tracker'))
import cv2
import numpy as np
import yaml
from yolo_person_tracker.backend import YoloBackend, Detection
from yolo_person_tracker.registration import Registration
from yolo_person_tracker.fusion import fuse_depth
from yolo_person_tracker.pose import PoseConfig, PostureTracker, LABELS, EDGES, points2d, plausible_torso3d
from yolo_person_tracker.reid_backend import OSNetBackend, appearance_crop
from yolo_person_tracker.reid import IdentityConfig, IdentityManager
sys.path.insert(0,str(ROOT/'scripts'))
from gemini_recording import load_depth_m, pairing_key


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('recording',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--stride',type=int,default=6);p.add_argument('--reid',action='store_true');p.add_argument('--cached-poses',type=Path,help='Prior JSONL from this same recording, including detection confidence');a=p.parse_args()
 if a.stride<1:raise ValueError('stride must be positive')
 a.output.mkdir(parents=True,exist_ok=True)
 colors=list(csv.DictReader((a.recording/'color_timestamps.csv').open()));depths=list(csv.DictReader((a.recording/'timestamps.csv').open()))
 key=pairing_key(colors,depths);times=np.array([int(x[key]) for x in depths],dtype=np.int64)
 reg=Registration(json.loads((a.recording/'device.json').read_text()));k=reg.color;intr=(k['fx'],k['fy'],k['cx'],k['cy'])
 params=yaml.safe_load((ROOT/'ros2_ws/src/perception_bringup/config/pose.yaml').read_text())['/**']['ros__parameters'];cfg=PoseConfig(**{f.name:params['pose_'+f.name] for f in fields(PoseConfig) if 'pose_'+f.name in params})
 cache={r['frame']:r for r in map(json.loads,a.cached_poses.read_text().splitlines())} if a.cached_poses else None
 tracker=PostureTracker(cfg);backend=None if cache is not None else YoloBackend(str(ROOT/'models/weights/yolo26s-pose.pt'),'cpu',640,task='pose')
 reid=OSNetBackend(str(ROOT/'models/weights/osnet_x0_25_msmt17.onnx')) if a.reid else None
 reid_params=yaml.safe_load((ROOT/'ros2_ws/src/perception_bringup/config/reid.yaml').read_text())
 reid_values=next(iter(reid_params.values()))['ros__parameters']
 manager=IdentityManager(IdentityConfig(**{f.name:reid_values[f.name] for f in fields(IdentityConfig) if f.name in reid_values}))
 cap=cv2.VideoCapture(str(a.recording/'color.avi'));video=cv2.VideoWriter(str(a.output/'annotated.avi'),cv2.VideoWriter_fourcc(*'MJPG'),30/a.stride,(640,480));assert cap.isOpened() and video.isOpened()
 start=int(colors[0]['host_monotonic_ns']);last=None;epoch=0
 try:
  with (a.output/'frames.jsonl').open('w') as out:
   for i,row in enumerate(colors):
    ok,image=cap.read()
    if not ok:raise RuntimeError(f'video shorter than timestamps at frame {i}')
    if i%a.stride:continue
    stamp=(int(row['host_monotonic_ns'])-start)/1e9+1.;ns=int(row[key]);j=int(np.argmin(np.abs(times-ns)));delta=float(times[j]-ns)/1e6
    matched=abs(delta)<=40
    if matched:
     z=load_depth_m(a.recording,depths[j])
    else:z=np.zeros((400,640),np.float32)
    image,aligned=reg.apply(image,z)
    if last is not None and stamp-last>cfg.max_gap_s:
     if backend is not None:backend.reset()
     tracker.reset();epoch+=1
    last=stamp;t=time.monotonic()
    if cache is None:
     detections=backend.infer(image);ms=(time.monotonic()-t)*1000
    else:
     cached=cache[i+1]
     if abs(cached['time_s']-(stamp-1))>1e-6:raise ValueError('Cached frame timestamp differs from recording')
     if any(q['id'] is not None and int(q['id'].split(':')[0])!=epoch for q in cached['people']):raise ValueError('Cached tracker epoch differs')
     detections=[Detection(int(q['id'].split(':')[1]) if q['id'] is not None else None,tuple(q['box']),q['confidence'],tuple(map(tuple,q['keypoints']))) for q in cached['people']];ms=cached['inference_ms']
    people=[]
    observations={};qualities={};identities=[]
    proxies=[NS(box=d.box,confidence=d.confidence,keypoints_2d=[NS(x=x,y=y) for x,y,c in d.keypoints],keypoint_confidences=[c for x,y,c in d.keypoints]) for d in detections]
    if reid is not None:
     for d,proxy in zip(detections,proxies):
      if d.track_id is None:continue
      ident=f'{epoch}:{d.track_id}'
      crop,quality=appearance_crop(image,proxy,[p for d,p in zip(detections,proxies) if d.track_id is not None],min_height=reid_values.get('min_crop_height',80),min_width=reid_values.get('min_crop_width',32),blur_min=reid_values.get('blur_min',20.))
      qualities[ident]=quality
      observations[ident]=reid.extract([crop])[0] if crop is not None else None
     identities=manager.update(stamp,observations)
    for index,d in enumerate(detections):
     ident=f'{epoch}:{d.track_id}' if d.track_id is not None else None
     state,phase,reason=tracker.update(ident,stamp,d.box,d.keypoints) if ident is not None else (0,0,'untracked')
     dep=fuse_depth(aligned,d.box,d.keypoints,intr,other_boxes=[x.box for n,x in enumerate(detections) if n!=index])
     pts,valid=points2d(d.keypoints);jvalid=np.isfinite(dep.joints).all(axis=1)
     person=dict(id=ident,confidence=d.confidence,box=d.box,keypoints=d.keypoints,state=LABELS[state],phase=phase,reason=reason,depth_source=dep.source,depth_reason=dep.reason,target=dep.target,valid_joints=int(jvalid.sum()),joints3d=[[float(v) if np.isfinite(v) else None for v in point] for point in dep.joints],complete_torso=bool(jvalid[[5,6,11,12]].all()),plausible_torso=bool(plausible_torso3d(dep.joints)),reid_quality=qualities.get(ident,'not evaluated'))
     people.append(person);x1,y1,x2,y2=map(int,d.box);cv2.rectangle(image,(x1,y1),(x2,y2),(30,220,50),2)
     for u,v in EDGES:
      if valid[u] and valid[v]:cv2.line(image,tuple(map(int,pts[u,:2])),tuple(map(int,pts[v,:2])),(0,230,255),2)
     cv2.putText(image,f'{ident} {LABELS[state]}',(max(0,x1),max(48,y1-6)),0,.45,(0,200,255),1)
    cv2.putText(image,f'{stamp-1:.1f}s | 2D | host depth pair={matched}',(8,22),0,.55,(0,0,255),2)
    identity_rows=[dict(track_id=x.track_id,person_id=x.person_id,state=x.state,distance=x.distance if np.isfinite(x.distance) else None,detail=x.detail) for x in identities]
    video.write(image);out.write(json.dumps(dict(frame=i+1,time_s=stamp-1,pair_delta_ms=delta,depth_matched=matched,depth_frame=depths[j]['frame'] if matched else None,inference_ms=ms,pose_cached=cache is not None,people=people,identities=identity_rows),allow_nan=False)+'\n');out.flush()
    if i%300==0:print(f'{i+1}/{len(colors)}',flush=True)
 finally:cap.release();video.release()

if __name__=='__main__':main()
