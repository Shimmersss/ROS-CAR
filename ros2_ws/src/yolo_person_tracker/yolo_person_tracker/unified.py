"""Depth/monocular selection on a single ground-projected bilateral ankle midpoint.

Sources share pixels/calibration; covariance intersection avoids treating their
errors as independent. Only estimates of the SAME reference point are combined. No missing skeleton joint is synthesized.
"""
from dataclasses import dataclass, replace
import math
import numpy as np
from .ground import GroundEstimate, _Camera, _extract, _usable, estimate_ground_point


@dataclass(frozen=True)
class UnifiedConfig:
    confirm_frames: int = 3
    max_gap_s: float = .5
    depth_max_m: float = 3.
    depth_std_m: float = .06
    body_projection_std_m: float = .25
    mount_std_m: float = .10
    mount_angle_std_deg: float = 1.
    ankle_plane_tolerance_m: float = .25
    source_agreement_m: float = .6
    motion_base_m: float = .25
    max_speed_mps: float = 2.

    def __post_init__(self):
        if not isinstance(self.confirm_frames, int) or self.confirm_frames < 2:
            raise ValueError('unified confirm_frames must be an integer >=2')
        for k,v in vars(self).items():
            if not math.isfinite(v) or v <= 0:
                raise ValueError(f'unified {k} must be positive and finite')
        if self.mount_angle_std_deg>=45:
            raise ValueError('unified mount angular uncertainty must be <45 degrees')


def depth_contact(person, intrinsics, size, transform, ground, config):
    """Bilateral ankle midpoint from real ankle depth, or body-depth ray approximation."""
    if person.posture not in (0,1,2) or person.fall_stage:
        return None, 'lying/fall: ground contact undefined'
    camera = _Camera(intrinsics, size, transform)
    points, confidences = _extract(person)
    if points is None or not all(_usable(points, confidences, i, ground, camera,
                                        ground.edge_margin_px) for i in (15,16)):
        return None, 'two visible ankles required'
    xyz = np.array([(p.x,p.y,p.z) for p in person.keypoints_3d], float)
    flags = np.asarray(person.keypoints_3d_valid, bool)
    if xyz.shape != (17,3) or flags.shape != (17,):
        return None, 'no measured 3D skeleton'
    valid = flags & np.isfinite(xyz).all(axis=1) & (xyz[:,2] >= .2) & (xyz[:,2] <= config.depth_max_m)
    if valid[[15,16]].all():
        optical = xyz[[15,16]].copy()
        sigma, method = config.depth_std_m, 'depth_ankles'
    else:
        indices = [i for i in (5,6,11,12) if valid[i] and confidences[i] >= ground.min_keypoint_confidence]
        zs = xyz[indices,2]
        if len(indices) >= 2:
            if np.ptp(zs) > .35:
                return None, 'conflicting body depths'
            z = float(np.median(zs))
            spread_z = float(np.ptp(zs))
        elif (getattr(person,'body_depth_valid',False)
              and math.isfinite(person.body_depth_m) and .2<=person.body_depth_m<=config.depth_max_m):
            z = float(person.body_depth_m)
            spread_z = 0.
        else:
            return None, 'insufficient real depth; monocular fallback allowed'
        optical = np.array([((points[i,0]-camera.cx)*z/camera.fx,
                             (points[i,1]-camera.cy)*z/camera.fy,z) for i in (15,16)])
        sigma = math.hypot(config.body_projection_std_m, spread_z)
        method = 'depth_body_projection'
    feet = optical @ camera.rotation.T + camera.origin
    if np.max(np.abs(feet[:,2]-ground.ground_z_m-ground.ankle_height_m)) > config.ankle_plane_tolerance_m:
        return None, 'conflicting depth/contact plane'
    spread = float(np.linalg.norm(feet[0,:2]-feet[1,:2]))
    if spread > ground.consistency_max_m:
        return None, 'conflicting depth ankles'
    point = np.mean(feet,axis=0); point[2] = ground.ground_z_m
    if math.hypot(*point[:2]) > ground.max_range_m:
        return None, 'depth point out of range'
    std = math.sqrt(sigma*sigma+mount_std(point,config)**2+(spread/2)**2)
    if std > ground.max_std_m:
        return None, 'depth uncertainty exceeds limit'
    return GroundEstimate(tuple(map(float,point)),float(min(confidences[15:17])),std,method,
                          'bilateral ankle midpoint projected to ground; body projection is not a measured joint'), None


def conservative_fusion(estimates):
    """Equal-weight covariance intersection for isotropic XY uncertainty.

    Correlation is unknown (shared pixels/depth). Precision is AVERAGED, not
    added: agreeing copies cannot make the declared uncertainty shrink.
    Disagreement adds an extra spread term instead of being hidden by averaging.
    """
    if len(estimates)==1:return estimates[0]
    variances=np.array([max(e.std_m,.01)**2 for e in estimates])
    precision=1./variances
    weights=precision/precision.sum()
    points=np.array([e.point[:2] for e in estimates])
    xy=weights@points
    variance=1./float(np.mean(precision))
    spread=max(float(np.linalg.norm(p-xy)) for p in points)
    sources=','.join(sorted(e.method for e in estimates))
    return GroundEstimate((float(xy[0]),float(xy[1]),estimates[0].point[2]),
        min(e.confidence for e in estimates),math.sqrt(variance+spread**2),
        'fused_contact','sources='+sources+'; conservative correlated-observation fusion')


class UnifiedTrack:
    def __init__(self, config=None):
        self.cfg=config or UnifiedConfig()
        self.last_stamp=None;self.last=None;self.last_sources=set()
        self.pending=None;self.pending_stamp=None;self.count=0
        self.previous_stamp=None

    def update(self,stamp,depth,mono,hard_invalid=False,body=None,max_std=math.inf):
        c=self.cfg
        if not math.isfinite(stamp) or stamp<=0:
            self.__init__(c);return None,'invalid timestamp'
        if self.previous_stamp is not None and not 0<stamp-self.previous_stamp<=c.max_gap_s:
            self.__init__(c)
        self.previous_stamp=stamp
        if self.last_stamp is not None and stamp-self.last_stamp>c.max_gap_s:
            self.last=None;self.last_stamp=None;self.last_sources=set()
        observations=[v for v in (depth,mono,body) if v is not None]
        if hard_invalid:
            self.pending=None;self.count=0
            return None,'conflicting or undefined contact evidence'
        if not observations:
            self.pending=None;self.count=0
            return None,'no fresh ground-contact observation'
        for i,a in enumerate(observations):
            for b in observations[i+1:]:
                if np.linalg.norm(np.array(a.point[:2])-b.point[:2])>c.source_agreement_m:
                    self.pending=None;self.count=0
                    return None,'conflicting common-reference observations'
        chosen=conservative_fusion(observations)
        if chosen.std_m>max_std:
            self.pending=None;self.count=0
            return None,'fused uncertainty exceeds limit'
        sources={e.method for e in observations}
        if self.last is not None:
            distance=np.linalg.norm(np.array(chosen.point[:2])-self.last.point[:2])
            if distance>c.motion_base_m+c.max_speed_mps*(stamp-self.last_stamp):
                self.pending=None;self.count=0
                return None,'implausible position jump; awaiting reacquisition'
        # Shared current evidence preserves continuity. Only a new track or a
        # complete replacement of all source types requires fresh confirmation.
        if self.last is None or not sources.intersection(self.last_sources):
            compatible=(self.pending is not None and
                        np.linalg.norm(np.array(chosen.point[:2])-self.pending.point[:2])<=
                        c.motion_base_m+c.max_speed_mps*(stamp-self.pending_stamp))
            self.count=self.count+1 if compatible else 1
            self.pending=chosen;self.pending_stamp=stamp
            if self.count<c.confirm_frames:
                return None,f'confirming common point: {self.count}/{c.confirm_frames}'
        self.last,self.last_stamp,self.last_sources=chosen,stamp,sources
        self.pending=None;self.count=0
        return chosen,chosen.detail+'; accepted current observation'


def candidates(person, model, transform, ground, config):
    # Height prior refers to a different virtual body point; not a feet measurement.
    feet_config = replace(ground, enable_height_prior=False, height_prior_confirmed=False)
    depth, dr = depth_contact(person,*model,transform,feet_config,config)
    mono, mr = estimate_ground_point(person,*model,transform,feet_config)
    if mono:
        mono = replace(mono, std_m=math.hypot(mono.std_m,mount_std(mono.point,config)), method='mono_ankles')
        if mono.std_m > ground.max_std_m:
            mono=None; mr='monocular uncertainty exceeds limit'
    hard = (person.posture not in (0,1,2) or bool(person.fall_stage)
            or 'conflicting' in (dr or '') or 'conflicting' in (mr or ''))
    return depth, mono, hard, f'depth: {dr or "available"}; mono: {mr or "available"}'


def mount_std(point,config):
    return math.hypot(config.mount_std_m, math.hypot(*point[:2])*math.tan(math.radians(config.mount_angle_std_deg)))
