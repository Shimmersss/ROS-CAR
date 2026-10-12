#!/usr/bin/env python3
"""Summarize offline diagnostics without treating observations as ground truth."""
import argparse,collections,json
from pathlib import Path
import numpy as np


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('frames',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 rows=[json.loads(x) for x in a.frames.read_text().splitlines()];selected=[]
 for r in rows:
  if r['people']:
   q=max(r['people'],key=lambda q:(q['box'][2]-q['box'][0])*(q['box'][3]-q['box'][1]));selected.append((r,q))
 paired=[(r,q) for r,q in selected if r['depth_matched']]
 def finite_pair(q,indices):return all(q['joints3d'][i][2] is not None and .2<=q['joints3d'][i][2]<=3 for i in indices)
 def percentile(values):return dict(zip(['min','p50','p95','max'],map(float,np.percentile(values,[0,50,95,100])))) if values else None
 joint_counts={name:sum(finite_pair(q,inds) for r,q in paired) for name,inds in [('shoulders',[5,6]),('hips',[11,12]),('knees',[13,14]),('ankles',[15,16])]}
 steps=[];joint_steps=[];last={};jlast={}
 for r,q in selected:
  t=r['time_s'];ident=q['id']
  if ident is None:continue
  if q['target'] is not None:
   z=q['target'][2]
   if ident in last and 0<t-last[ident][0]<=.5:steps.append({'time_s':t,'id':ident,'dt':t-last[ident][0],'dz':abs(z-last[ident][1])})
   last[ident]=(t,z)
  if ident in jlast and 0<t-jlast[ident][0]<=.5:
   old=jlast[ident][1]
   for i in range(17):
    if old[i][2] is not None and q['joints3d'][i][2] is not None:joint_steps.append(float(np.linalg.norm(np.array(old[i])-q['joints3d'][i])))
  jlast[ident]=(t,q['joints3d'])
 static=[]
 for lo,hi in [(29,37),(40,50),(94,98)]:
  values=[q['target'][2] for r,q in selected if lo<=r['time_s']<hi and q['target'] is not None]
  static.append(dict(start=lo,end=hi,n=len(values),z_m=percentile(values),note='Person can move within this window; spread is not sensor absolute error'))
 id_counts=collections.defaultdict(collections.Counter);identity_counts=collections.defaultdict(collections.Counter);events=[]
 for r in rows:
  for x in r['identities']:
   id_counts[x['track_id']][x['state']]+=1
   if x['state'] in ('REGISTERED','REASSOCIATED','VERIFIED'):identity_counts[x['track_id']][x['person_id']]+=1
   if x['state'] in ('REGISTERED','REASSOCIATED'):events.append(dict(time_s=r['time_s'],**x))
 result=dict(frames=len(rows),paired_frames=sum(r['depth_matched'] for r in rows),largest_box_paired_samples=len(paired),body_depth_valid=sum(q['target'] is not None for r,q in paired),complete_torso=sum(q['complete_torso'] for r,q in paired),plausible_torso=sum(q['plausible_torso'] for r,q in paired),joint_pairs_within_3m=joint_counts,body_z_step_m=percentile([s['dz'] for s in steps]),steps_over_025m=[s for s in steps if s['dz']>.25],joint_step_m=percentile(joint_steps),window_depth=static,reid_states={k:dict(v) for k,v in id_counts.items()},track_identity_counts={k:dict(v) for k,v in identity_counts.items()},reid_events=events,reid_quality=dict(collections.Counter(q['reid_quality'] for r in rows for q in r['people'])),pair_abs_delta_ms=percentile([abs(r['pair_delta_ms']) for r in rows if r['depth_matched']]),limitations=['Largest box is a diagnostic subset, not persistent target selection','No measured distance truth or calibrated gravity/ground','5Hz offline replay, no actual worker latency or lock service acceptance','Host receive pairing does not prove exposure synchronization','No labeled identity ground truth; do not infer general ReID precision'])
 a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ('reid_events','steps_over_025m')},indent=2))

if __name__=='__main__':main()
