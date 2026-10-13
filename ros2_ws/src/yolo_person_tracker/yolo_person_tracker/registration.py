"""Explicit native-depth -> rectified-color projection with a nearest-surface z-buffer."""
import copy
import json
from pathlib import Path
import cv2
import numpy as np


def depth_millimetres(depth_m):
    """Quantize valid metres to uint16 mm; zero remains an invalid pixel."""
    millimetres = np.rint(np.asarray(depth_m, dtype=np.float64) * 1000.)
    valid = np.isfinite(millimetres) & (millimetres >= 1) & (millimetres <= 65535)
    result = np.zeros(millimetres.shape, dtype=np.uint16)
    result[valid] = millimetres[valid].astype(np.uint16)
    return result


def camera_matrix(intrinsic):
    matrix=np.array([[intrinsic['fx'],0,intrinsic['cx']], [0,intrinsic['fy'],intrinsic['cy']], [0,0,1]], dtype=float)
    if not np.isfinite(matrix).all() or matrix[0,0]<=0 or matrix[1,1]<=0:
        raise ValueError('Invalid device intrinsics')
    if intrinsic['width']<=0 or intrinsic['height']<=0:
        raise ValueError('Invalid calibration dimensions')
    return matrix


class Registration:
    def __init__(self, calibration):
        self.calibration=calibration
        self.color=calibration['color_intrinsic']; self.depth=calibration['depth_intrinsic']
        self.kc=camera_matrix(self.color); self.kd=camera_matrix(self.depth)
        self.dc=np.asarray(calibration['color_distortion'],dtype=float)
        self.dd=np.asarray(calibration['depth_distortion'],dtype=float)
        self.rotation=np.asarray(calibration['depth_to_color_rotation'],dtype=float).reshape(3,3)
        self.translation=np.asarray(calibration['depth_to_color_translation_mm'],dtype=float)/1000.
        if (self.dc.shape!=(8,) or self.dd.shape!=(8,) or self.translation.shape!=(3,)
                or not all(np.isfinite(v).all() for v in (self.dc,self.dd,self.rotation,self.translation))
                or not np.allclose(self.rotation.T@self.rotation,np.eye(3),atol=.01)
                or not np.isclose(np.linalg.det(self.rotation),1,atol=.01)
                or np.linalg.norm(self.translation)>1):
            raise ValueError('Invalid device distortion/extrinsics; do not use placeholder calibration')
        width,height=self.color['width'],self.color['height']
        self.maps=cv2.initUndistortRectifyMap(self.kc,self.dc,np.eye(3),self.kc,(width,height),cv2.CV_32FC1)
        u,v=np.meshgrid(np.arange(self.depth['width']),np.arange(self.depth['height']))
        pixels=np.stack((u,v),axis=-1).astype(np.float32).reshape(-1,1,2)
        normalized=cv2.undistortPoints(pixels,self.kd,self.dd).reshape(-1,2)
        self.rays=np.column_stack((normalized,np.ones(len(normalized))))
        # Depth only scales each ray, so (ray*z)@R.T == z*(ray@R.T): rotate the fixed rays once.
        self.rotated_rays=self.rays@self.rotation.T
        self.ray_ok=np.isfinite(self.rotated_rays).all(axis=1)

    def apply(self, color, depth_m):
        if color.shape[:2]!=(self.color['height'],self.color['width']) or depth_m.shape!=(self.depth['height'],self.depth['width']):
            raise ValueError('Native RGB-D dimensions differ from device calibration')
        corrected=cv2.remap(color,*self.maps,cv2.INTER_LINEAR)
        z=depth_m.reshape(-1)
        # Range comparisons also reject NaN/inf; elementwise float64 keeps the old per-pixel results.
        valid=np.flatnonzero(self.ray_ok&(z>=.2)&(z<=8.))
        points=self.rotated_rays[valid]*z[valid,None].astype(np.float64)+self.translation
        points=points[points[:,2]>.05]
        x,y,depth=points[:,0],points[:,1],points[:,2]
        kc=self.kc
        u=np.rint((kc[0,0]*x+kc[0,2]*depth)/depth).astype(np.int64)
        v=np.rint((kc[1,1]*y+kc[1,2]*depth)/depth).astype(np.int64)
        width,height=self.color['width'],self.color['height']
        inside=(u>=0)&(u<width)&(v>=0)&(v<height)
        index=v[inside]*width+u[inside]
        depth=depth[inside].astype(np.float32)
        # Nearest-surface z-buffer. Pixels hit once are written directly; only pixels hit by several
        # points (occlusion edges) go through np.minimum.at, which is slow on Jetson's NumPy 1.21.
        shared=np.bincount(index,minlength=width*height)[index]>1
        aligned=np.zeros(width*height,dtype=np.float32)
        aligned[index[~shared]]=depth[~shared]
        if shared.any():
            nearest=np.full(width*height,np.inf,dtype=np.float32)
            np.minimum.at(nearest,index[shared],depth[shared])
            hit=np.isfinite(nearest)
            aligned[hit]=nearest[hit]
        return corrected,aligned.reshape(height,width)


def load_registration(path, expected_serial):
    """Device calibration for the explicitly selected Gemini; a serial mismatch refuses to start."""
    calibration=json.loads(Path(path).read_text())
    if not expected_serial or calibration['serial']!=expected_serial:
        raise ValueError('Calibration serial must match explicitly selected Gemini')
    return Registration(calibration)


def rectified_camera_info(registration, header):
    """CameraInfo of the rectified color image (zero distortion, P=[K|0]) with the given header."""
    from sensor_msgs.msg import CameraInfo
    info=CameraInfo(); info.header=copy.deepcopy(header)
    info.width=registration.color['width']; info.height=registration.color['height']
    info.distortion_model='plumb_bob'; info.d=[0.]*5
    k=registration.kc; info.k=list(map(float,k.reshape(-1))); info.r=list(map(float,np.eye(3).reshape(-1)))
    info.p=list(map(float,np.column_stack((k,np.zeros(3))).reshape(-1)))
    return info


def validate_raw_pair(color, depth, now, max_age, sync_slop, color_frame, depth_frame, registration):
    """Checks for native Gemini RGB-D before in-process registration; returns rectified intrinsics."""
    from .input_contract import stamp_seconds
    for msg in (color, depth):
        stamp=stamp_seconds(msg.header.stamp)
        if stamp<=0 or not 0<=now-stamp<=max_age:
            raise ValueError('Expired, zero or future raw timestamp')
    if abs(stamp_seconds(color.header.stamp)-stamp_seconds(depth.header.stamp))>sync_slop:
        raise ValueError('RGB-D skew exceeds tolerance')
    if color.header.frame_id!=color_frame or depth.header.frame_id!=depth_frame:
        raise ValueError('Unexpected native optical frames')
    if ((color.width,color.height)!=(registration.color['width'],registration.color['height'])
            or (depth.width,depth.height)!=(registration.depth['width'],registration.depth['height'])):
        raise ValueError('Native RGB-D dimensions differ from device calibration')
    if color.encoding not in ('rgb8','bgr8','rgba8','bgra8'):
        raise ValueError('Expected color RGB/BGR image')
    if depth.encoding not in ('16UC1','32FC1'):
        raise ValueError('Unsupported depth unit/encoding')
    k=registration.kc
    return (k[0,0],k[1,1],k[0,2],k[1,2])
