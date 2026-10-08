"""Gimbal URDF from a measured mount file (no xacro). The camera driver keeps
camera_link -> optical frames; the vendor base_footprint -> camera_link TF must be off."""
import math

import yaml

SEGMENTS = ('base_to_pan', 'pan_to_tilt', 'tilt_to_camera', 'tilt_to_imu')


def load_mount(path):
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    return validate_mount(data.get('gimbal_mount', data))


def validate_mount(mount):
    if not isinstance(mount.get('confirmed'), bool):
        raise ValueError('gimbal_mount.confirmed must be bool')
    for name in ('parent_frame', 'camera_frame', 'imu_frame'):
        if not isinstance(mount.get(name), str) or not mount[name]:
            raise ValueError(f'gimbal_mount.{name} must be a non-empty frame name')
    for segment in SEGMENTS:
        values = mount.get(segment, {})
        for key in ('xyz', 'rpy'):
            vector = values.get(key)
            if (not isinstance(vector, list) or len(vector) != 3
                    or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vector)):
                raise ValueError(f'gimbal_mount.{segment}.{key} must be three finite numbers')
    limit, lower, upper = mount.get('pan_limit_rad'), mount.get('tilt_lower_rad'), mount.get('tilt_upper_rad')
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (limit, lower, upper)):
        raise ValueError('gimbal joint limits must be finite numbers')
    if not 0 < limit <= math.pi or not -math.pi/2 <= lower < upper <= math.pi/2:
        raise ValueError('invalid gimbal joint limits')
    return mount


def _origin(segment):
    xyz = ' '.join(f'{v:.6f}' for v in segment['xyz'])
    rpy = ' '.join(f'{v:.6f}' for v in segment['rpy'])
    return f'<origin xyz="{xyz}" rpy="{rpy}"/>'


def build_urdf(mount):
    m = validate_mount(mount)
    links = [m['parent_frame'], 'pan_link', 'tilt_link', m['camera_frame'], m['imu_frame']]
    body = ''.join(f'<link name="{name}"/>' for name in links)
    body += (f'<joint name="pan_joint" type="revolute"><parent link="{m["parent_frame"]}"/><child link="pan_link"/>'
             f'{_origin(m["base_to_pan"])}<axis xyz="0 0 1"/>'
             f'<limit lower="{-m["pan_limit_rad"]:.6f}" upper="{m["pan_limit_rad"]:.6f}" effort="1" velocity="3"/></joint>')
    # Positive tilt looks down: rotation about +y (left) by the right-hand rule.
    body += (f'<joint name="tilt_joint" type="revolute"><parent link="pan_link"/><child link="tilt_link"/>'
             f'{_origin(m["pan_to_tilt"])}<axis xyz="0 1 0"/>'
             f'<limit lower="{m["tilt_lower_rad"]:.6f}" upper="{m["tilt_upper_rad"]:.6f}" effort="1" velocity="3"/></joint>')
    body += (f'<joint name="camera_mount_joint" type="fixed"><parent link="tilt_link"/>'
             f'<child link="{m["camera_frame"]}"/>{_origin(m["tilt_to_camera"])}</joint>')
    body += (f'<joint name="imu_mount_joint" type="fixed"><parent link="tilt_link"/>'
             f'<child link="{m["imu_frame"]}"/>{_origin(m["tilt_to_imu"])}</joint>')
    return f'<?xml version="1.0"?><robot name="roscar_gimbal">{body}</robot>'
