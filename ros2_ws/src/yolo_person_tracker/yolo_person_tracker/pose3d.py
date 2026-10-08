"""Measured 3D geometry augments the existing bounded fall state machine.

No monocular joint lifting or filling. Gravity/ground are explicit calibration.
With up_source='floor' or 'tf' the up vector and camera height arrive per frame
(gimbal/IMU or a fitted floor); the confirmation flags still gate their use.
"""
from dataclasses import dataclass, replace
import math
import numpy as np
from .pose import PostureTracker, PoseConfig, points2d, UNKNOWN, STANDING, SITTING_CROUCHING, LYING, FALLEN, plausible_torso3d, geometry2d
from .floor_plane import angle_deg, camera_angles

UP_SOURCES = ('static', 'floor', 'tf')


@dataclass(frozen=True)
class Pose3DConfig:
    enabled: bool = True
    gravity_confirmed: bool = False
    ground_confirmed: bool = False
    up_x: float = 0.
    up_y: float = -1.
    up_z: float = 0.
    camera_height_m: float = 1.
    depth_max_m: float = 3.
    metric_drop_m: float = .30
    low_center_m: float = .65
    torso_min_m: float = .20
    torso_max_m: float = .85
    width_min_m: float = .12
    width_max_m: float = .80
    thigh_min_m: float = .18
    thigh_max_m: float = .80
    thigh_sitting_deg: float = 45.
    depth_gap_s: float = .35
    # Dynamic gravity (gimbal): per-frame up vector; 'static' keeps the fixed up_* above.
    up_source: str = 'static'
    # 2D rules assume no image roll; larger roll makes that frame's 2D result unknown.
    upright_roll_deg: float = 3.
    # Camera rotation (change of up) beyond this resets the pixel-domain drop reference.
    camera_motion_deg: float = 2.

    def __post_init__(self):
        for key,value in vars(self).items():
            if key in ('enabled','gravity_confirmed','ground_confirmed'):
                if not isinstance(value,bool):raise ValueError(f'{key} must be bool')
            elif key=='up_source':
                if value not in UP_SOURCES:raise ValueError(f'up_source must be one of {UP_SOURCES}')
            elif not math.isfinite(value):raise ValueError(f'{key} must be finite')
            elif not key.startswith('up_') and value<=0:raise ValueError(f'{key} must be positive')
        if abs(np.linalg.norm([self.up_x,self.up_y,self.up_z])-1.)>.01:
            raise ValueError('pose3d up vector must be normalized')
        if self.ground_confirmed and not self.gravity_confirmed:
            raise ValueError('pose3d ground confirmation requires gravity confirmation')
        if not self.torso_min_m<self.torso_max_m or not self.width_min_m<self.width_max_m:
            raise ValueError('Invalid 3D anatomy bounds')
        if not self.thigh_min_m<self.thigh_max_m or not 0<self.thigh_sitting_deg<90:
            raise ValueError('Invalid 3D thigh thresholds')


def geometry3d(keypoints,joints,cfg,pose):
    points,visible=points2d(keypoints,pose.confidence)
    xyz=np.asarray(joints,float)
    if xyz.shape!=(17,3):return None,'missing measured joints',False
    valid=np.isfinite(xyz).all(axis=1)&visible&(xyz[:,2]>=.2)&(xyz[:,2]<=cfg.depth_max_m)
    if not valid[[5,6,11,12]].all():return None,'incomplete measured shoulders/hips',False
    shoulders=xyz[[5,6]].mean(axis=0);hips=xyz[[11,12]].mean(axis=0)
    torso=shoulders-hips;length=float(np.linalg.norm(torso))
    if not plausible_torso3d(xyz,cfg.torso_min_m,cfg.torso_max_m,cfg.width_min_m,cfg.width_max_m):
        return None,'inconsistent measured torso anatomy',True
    up=np.array([cfg.up_x,cfg.up_y,cfg.up_z],float)
    alignment=float(torso@up/length)
    if alignment<-.25:return None,'inverted measured torso',True
    angle=math.degrees(math.acos(np.clip(abs(alignment),0.,1.)))
    center=(shoulders+hips)*.5
    center_height=float(center@up+cfg.camera_height_m)
    if cfg.ground_confirmed and center_height < -.15:
        return None,'measured torso below ground',True
    low=cfg.ground_confirmed and center_height<=cfg.low_center_m
    lying=angle>=pose.horizontal_deg and low
    posture=LYING if lying else UNKNOWN
    if angle<pose.standing_deg and not valid[[13,14]].all():
        return None,'incomplete measured knees; 2D classification fallback',False
    if angle<pose.standing_deg and valid[[13,14]].all():
        thighs=xyz[[13,14]]-xyz[[11,12]]
        lengths=np.linalg.norm(thighs,axis=1)
        if np.any(lengths<cfg.thigh_min_m) or np.any(lengths>cfg.thigh_max_m):
            return None,'inconsistent measured thigh anatomy',True
        angles=np.degrees(np.arccos(np.clip(thighs@(-up)/lengths,-1.,1.)))
        if abs(float(angles[0]-angles[1]))>45:
            posture=UNKNOWN
        else:
            posture=SITTING_CROUCHING if float(np.mean(angles))>=cfg.thigh_sitting_deg else STANDING
    # The existing transition rule uses drop_fraction*height. Scale=1 metre here.
    result=(posture,float(-center@up),1.,lying)
    return result,f'angle={angle:.1f}deg; center_height={center_height:.3f}m; ground_confirmed={cfg.ground_confirmed}',False


class EnhancedPostureTracker:
    """Continuous 2D history, separate metric history, and one confirmed-event latch."""
    def __init__(self,pose=None,spatial=None):
        self.pose=pose or PoseConfig();self.spatial=spatial or Pose3DConfig()
        self.planar=PostureTracker(self.pose)
        self.metric=PostureTracker(replace(self.pose,drop_fraction=self.spatial.metric_drop_m,
            max_gap_s=max(self.pose.max_gap_s,self.spatial.depth_gap_s)))
        self.tracks={}

    def reset(self):
        self.planar.reset();self.metric.reset();self.tracks.clear()

    def _forget(self,identity):
        self.tracks.pop(identity,None)
        self.planar.tracks.pop(identity,None);self.metric.tracks.pop(identity,None)

    @staticmethod
    def _clear_event(tracker,identity):
        state=tracker.tracks.get(identity)
        if state:
            state['fallen']=False;state['pending']=None;state['recovery']=None

    @staticmethod
    def _reset_reference(tracker,identity):
        """Drop pixel baselines/pending only; a latched confirmed event is kept."""
        state=tracker.tracks.get(identity)
        if state:
            state['history'].clear();state['pending']=None;state['baseline']=None

    @staticmethod
    def _frame_gravity(gravity):
        """(unit up, camera height or None) from a per-frame source, or (None, None)."""
        if gravity is None:return None,None
        up,height=gravity
        up=np.asarray(up,float)
        if up.shape!=(3,) or not np.isfinite(up).all() or abs(np.linalg.norm(up)-1.)>.05:
            return None,None
        height=float(height) if height is not None and math.isfinite(height) and height>0 else None
        return up/np.linalg.norm(up),height

    def update(self,identity,stamp,box,keypoints,joints=None,gravity=None):
        c=self.spatial
        if not math.isfinite(stamp) or stamp<=0:
            self._forget(identity)
            return UNKNOWN,0,'Invalid observation timestamp'
        for key in list(self.tracks):
            if not 0<=stamp-self.tracks[key]['stamp']<=self.pose.max_gap_s:
                self._forget(key)
        state=self.tracks.get(identity)
        if state is not None and stamp<=state['stamp']:
            self._forget(identity);state=None
        if state is None:
            state=dict(stamp=stamp,metric_stamp=None,missing_since=None,
                       fallen=False,origin='',recovery=None)
            self.tracks[identity]=state
        state['stamp']=stamp
        if len(self.tracks)>128:
            oldest=min((k for k in self.tracks if k!=identity),key=lambda k:self.tracks[k]['stamp'])
            self._forget(oldest)
        # Always advance pixel-domain history, including frames with valid depth.
        g2,_=geometry2d(box,keypoints,self.pose)
        spatial=c;gravity_ok=True;note=''
        if c.up_source=='static':
            planar=self.planar.update(identity,stamp,box,keypoints,geometry=g2)
        else:
            # A moving camera shifts the person in the image: pixel drops are only
            # compared under (nearly) the same camera orientation and no roll.
            up,height=self._frame_gravity(gravity)
            if up is None:
                gravity_ok=False;state['up_anchor']=None
                self._reset_reference(self.planar,identity)
                planar=(UNKNOWN,0,'dynamic gravity unavailable; 2D drop reference reset')
                g2=None;note='; gravity=unavailable'
            else:
                _,roll=camera_angles(up)
                anchor=state.get('up_anchor')
                if anchor is None or angle_deg(anchor,up)>c.camera_motion_deg:
                    if anchor is not None:note='; camera rotated: 2D drop reference reset'
                    self._reset_reference(self.planar,identity);state['up_anchor']=up
                note=f'; roll={roll:.1f}deg'+note
                if abs(roll)>c.upright_roll_deg:
                    self._reset_reference(self.planar,identity)
                    planar=(UNKNOWN,0,f'camera roll {roll:.1f} deg exceeds 2D limit');g2=None
                else:
                    planar=self.planar.update(identity,stamp,box,keypoints,geometry=g2,
                                              fall_confirmed=self.pose.upright_confirmed)
                spatial=replace(c,up_x=float(up[0]),up_y=float(up[1]),up_z=float(up[2]),
                                camera_height_m=c.camera_height_m if height is None else height,
                                ground_confirmed=c.ground_confirmed and height is not None)
        geometry=None;conflict=False;metric=None
        reason='3D disabled or gravity unconfirmed'
        if c.enabled and c.gravity_confirmed:
            if gravity_ok:
                geometry,reason,conflict=geometry3d(keypoints,joints,spatial,self.pose)
            else:
                reason='dynamic gravity unavailable this frame'
        _, visible=points2d(keypoints,self.pose.confidence)
        if not visible[[5,6,11,12]].all():
            # Actual keypoint loss is not a depth-only dropout.
            self.metric.tracks.pop(identity,None)
            state['metric_stamp']=None;state['missing_since']=None
            geometry=None
        if geometry is not None:
            if state['missing_since'] is not None:
                elapsed=stamp-state['metric_stamp'] if state['metric_stamp'] is not None else math.inf
                metric_state=self.metric.tracks.get(identity)
                if elapsed>c.depth_gap_s:
                    self.metric.tracks.pop(identity,None)
                elif metric_state:
                    # Pause evidence-duration timers; never advance them on missing depth.
                    pause=stamp-state['missing_since']
                    if metric_state['pending']:
                        start,center,scale=metric_state['pending']
                        metric_state['pending']=(start+pause,center,scale)
                    if metric_state['recovery'] is not None:metric_state['recovery']+=pause
                state['missing_since']=None
            metric=self.metric.update(identity,stamp,box,keypoints,geometry,'3d',
                spatial.gravity_confirmed and spatial.ground_confirmed)
            state['metric_stamp']=stamp
        else:
            if state['missing_since'] is None:state['missing_since']=stamp
            expired=(state['metric_stamp'] is None or stamp-state['metric_stamp']>c.depth_gap_s)
            if conflict or expired:self.metric.tracks.pop(identity,None)
        basis='invalid3d' if conflict else '3d' if geometry is not None else '2d'
        instant=geometry[0] if geometry is not None else (g2[0] if g2 is not None else UNKNOWN)
        # Reliable height/upright evidence can veto a perspective-induced 2D fall.
        veto=bool(geometry is not None and spatial.ground_confirmed and
                  (geometry[0] in (STANDING,SITTING_CROUCHING) or
                   spatial.camera_height_m-geometry[1]>spatial.low_center_m))
        if veto:self._clear_event(self.planar,identity)
        if not state['fallen'] and not conflict:
            if metric is not None and metric[1]==2:
                state['fallen']=True;state['origin']='3d'
            elif planar[1]==2 and not veto:
                state['fallen']=True;state['origin']='2d'
        if state['fallen']:
            # A metric-confirmed fall needs metric upright evidence for recovery;
            # a prone person facing the camera can look upright in 2D.
            upright=(not conflict and instant==STANDING and
                     (state['origin']!='3d' or geometry is not None))
            if upright:
                if state['recovery'] is None:state['recovery']=stamp
                if stamp-state['recovery']>=self.pose.recovery_s:
                    state['fallen']=False;state['recovery']=None
                    for tracker in (self.planar,self.metric):
                        self._clear_event(tracker,identity)
                        if identity in tracker.tracks:
                            tracker.tracks[identity]['history'].clear();tracker.tracks[identity]['baseline']=None
                    return instant,0,f'basis={basis}; continuous upright recovery complete{note}'
            else:state['recovery']=None
            if state['fallen']:
                return FALLEN,2,f'basis={basis}; confirmed_by={state["origin"]}; awaiting continuous upright recovery{note}'
        if conflict:return UNKNOWN,0,'basis=invalid3d; '+reason+note
        metric_state=self.metric.tracks.get(identity)
        pending_metric=bool(metric_state and metric_state['pending'])
        phase=max(planar[1] if not veto else 0, metric[1] if metric is not None else int(pending_metric))
        if geometry is None and pending_metric and planar[1]==0:instant=UNKNOWN
        detail=metric[2] if metric is not None else planar[2]
        return instant,phase,f'basis={basis}; 2d_continuous; metric_pending={pending_metric}; {reason}; {detail}{note}'
