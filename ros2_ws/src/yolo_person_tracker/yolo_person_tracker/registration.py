"""Explicit native-depth -> rectified-color projection with a nearest-surface z-buffer."""
import cv2
import numpy as np


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

    def apply(self, color, depth_m):
        if color.shape[:2]!=(self.color['height'],self.color['width']) or depth_m.shape!=(self.depth['height'],self.depth['width']):
            raise ValueError('Native RGB-D dimensions differ from device calibration')
        corrected=cv2.remap(color,*self.maps,cv2.INTER_LINEAR)
        z=depth_m.reshape(-1)
        valid=np.isfinite(z)&(z>=.2)&(z<=8.)
        points=(self.rays[valid]*z[valid,None])@self.rotation.T+self.translation
        points=points[np.isfinite(points).all(axis=1)&(points[:,2]>.05)]
        projected=points@self.kc.T
        uv=np.rint(projected[:,:2]/projected[:,2,None]).astype(np.int64)
        width,height=self.color['width'],self.color['height']
        inside=(uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
        aligned=np.full(width*height,np.inf,dtype=np.float32)
        np.minimum.at(aligned,uv[inside,1]*width+uv[inside,0],points[inside,2])
        aligned[~np.isfinite(aligned)]=0
        return corrected,aligned.reshape(height,width)
