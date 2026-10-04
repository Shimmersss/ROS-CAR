"""Map fresh body-depth observations onto the foot reference using recent same-track pairs.

No identity transfer, stature prior, remembered position or invented 3D joint.
"""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np
from .ground import GroundEstimate, _Camera, _extract, _usable


@dataclass(frozen=True)
class BodyContactConfig:
    confirm_frames: int = 3
    max_age_s: float = .75
    max_offset_m: float = .6
    consistency_m: float = .15
    max_torso_angle_deg: float = 35.
    min_height_m: float = .35
    max_height_m: float = 1.8
    drift_std_mps: float = .3

    def __post_init__(self):
        if type(self.confirm_frames) is not int or not 2<=self.confirm_frames<=8:
            raise ValueError('body contact confirm_frames must be between 2 and 8')
        for name,value in vars(self).items():
            if not math.isfinite(value) or value<=0:raise ValueError(f'body contact {name} must be positive')
        if self.min_height_m>=self.max_height_m or self.max_torso_angle_deg>=90:
            raise ValueError('Invalid body contact geometry limits')


def body_anchor(person,model,transform,ground,unified,config):
    # Restrict learned relation to a standing body; sitting changes torso-foot offset.
    if person.posture!=1 or person.fall_stage:return None
    if not getattr(person,'body_depth_valid',False):return None
    z=float(person.body_depth_m)
    if not math.isfinite(z) or not .2<=z<=unified.depth_max_m:return None
    camera=_Camera(*model,transform)
    points,confidence=_extract(person)
    if points is None or not all(_usable(points,confidence,i,ground,camera) for i in (5,6,11,12)):
        return None
    delta=points[[11,12]].mean(axis=0)-points[[5,6]].mean(axis=0)
    if delta[1]<=0 or math.degrees(math.atan2(abs(delta[0]),delta[1]))>config.max_torso_angle_deg:
        return None
    # Move the original bbox-centre ray to a visible torso-centre ray at the same
    # current body depth before projecting into base XY. This removes bbox crop shifts.
    uv=points[[5,6,11,12]].mean(axis=0)
    optical=np.array([(uv[0]-camera.cx)*z/camera.fx,(uv[1]-camera.cy)*z/camera.fy,z])
    base=camera.rotation@optical+camera.origin
    if not config.min_height_m<=base[2]-ground.ground_z_m<=config.max_height_m:return None
    if math.hypot(*base[:2])>ground.max_range_m:return None
    return base


class BodyContact:
    def __init__(self,config=None):
        self.cfg=config or BodyContactConfig()
        self.samples=deque(maxlen=8)
        self.last_seen=None
        self.last_pair=None
        self.offset=None
        self.std=math.inf

    def reset(self):
        self.samples.clear();self.last_pair=None;self.offset=None;self.std=math.inf

    def observe(self,stamp,anchor,foot,base_std,ground_z,max_std,hard=False):
        c=self.cfg
        if not math.isfinite(stamp) or stamp<=0:
            self.reset();self.last_seen=None
            return None,'invalid body contact timestamp'
        if self.last_seen is not None and not 0<stamp-self.last_seen<=c.max_age_s:
            self.reset()
        self.last_seen=stamp
        if hard or anchor is None:
            self.reset()
            return None,'body/stance evidence unavailable; mapping reset'
        if self.last_pair is not None and stamp-self.last_pair>c.max_age_s:self.reset()
        # Use only calibration learned on EARLIER frames, never fit and validate
        # the current observation against itself.
        estimate=None
        if self.offset is not None:
            point=np.asarray(anchor[:2])+self.offset
            age=stamp-self.last_pair
            std=math.sqrt(base_std**2+self.std**2+(c.drift_std_mps*age)**2)
            if std<=max_std:
                estimate=GroundEstimate((float(point[0]),float(point[1]),float(ground_z)),
                    .5,std,'body_contact',f'fresh body depth; same-track torso/feet mapping age={age:.3f}s')
        if foot is not None:
            offset=np.asarray(foot.point[:2])-np.asarray(anchor[:2])
            if np.linalg.norm(offset)>c.max_offset_m:
                self.reset()
                return None,'body/feet offset exceeds limit'
            if self.offset is not None and np.linalg.norm(offset-self.offset)>c.consistency_m:
                self.reset()
                return None,'body/feet relation changed; mapping reset'
            if self.samples and np.linalg.norm(offset-self.samples[-1][0])>c.consistency_m:
                self.reset()
            self.samples.append((offset,float(foot.std_m)))
            self.last_pair=stamp
            if len(self.samples)>=c.confirm_frames:
                offsets=np.array([p for p,_ in self.samples])
                self.offset=np.mean(offsets,axis=0)
                spread=max(float(np.linalg.norm(p-self.offset)) for p in offsets)
                self.std=math.hypot(max(s for _,s in self.samples),spread)
        return estimate, ('body-to-feet mapping ready' if self.offset is not None
                          else f'body-to-feet pairing {len(self.samples)}/{c.confirm_frames}')
