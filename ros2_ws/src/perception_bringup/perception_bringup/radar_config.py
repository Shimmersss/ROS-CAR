"""Vendor driver mapping without vendor launch side effects or guessed hardware."""
import math

MODELS = ('n10p_uart', 'n10p_net')


def driver_spec(c):
    model = c['model']
    if model not in MODELS:
        raise ValueError('Supported project profiles: ' + ', '.join(MODELS))
    if not c['frame_id'] or not c['scan_topic'].startswith('/') or not c['pointcloud_topic'].startswith('/'):
        raise ValueError('Require a frame and absolute output topics')
    if model == 'n10p_uart' and not c['serial_port'].startswith('/dev/'):
        raise ValueError('Set an explicit /dev/ serial path')
    params = dict(lidar_type='X10', lidar_model='N10Plus',
                  serial_port=c['serial_port'] if model.endswith('_uart') else '',
                  device_ip=c['device_ip'], msop_port=int(c['msop_port']), difop_port=int(c['difop_port']),
                  frame_id=c['frame_id'], publish_scan=True, N10Plus_hz=10,
                  laserscan_topic=c['scan_topic'], pointcloud_topic=c['pointcloud_topic'],
                  min_range=.15, max_range=15., angle_disable_min=[0], angle_disable_max=[0],
                  is_pretreatment=False, is_MatrixTransformation=False)
    return 'lslidar_driver', 'lslidar_driver_node', params, []


def mount_args(c):
    if not c['publish_mount_tf']:
        return None
    if not c['mount_calibrated']:
        raise ValueError('Confirm measured radar mount before publishing TF')
    xyz, rpy = c['translation'], c['rotation_rpy']
    if (len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(v) for v in xyz+rpy)
            or not c['parent_frame'] or c['parent_frame'] == c['frame_id']):
        raise ValueError('Invalid radar mount')
    return ['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
            '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
            '--frame-id', c['parent_frame'], '--child-frame-id', c['frame_id']]
