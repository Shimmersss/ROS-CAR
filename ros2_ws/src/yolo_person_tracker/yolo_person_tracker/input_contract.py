"""Shared metadata checks; passing these never proves physical registration."""
import math


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def validate_pair(color, depth, info, now, max_age, sync_slop):
    """Return rectified intrinsics or raise ValueError with an actionable reason."""
    if info is None:
        raise ValueError('Waiting for color CameraInfo')
    for image in (color, depth):
        stamp = stamp_seconds(image.header.stamp)
        if stamp <= 0 or not 0 <= now-stamp <= max_age:
            raise ValueError('Zero, future or expired image timestamp')
    if abs(stamp_seconds(color.header.stamp)-stamp_seconds(depth.header.stamp)) > sync_slop:
        raise ValueError('RGB/depth timestamps exceed sync tolerance')
    if (not color.header.frame_id or color.header.frame_id != depth.header.frame_id
            or color.header.frame_id != info.header.frame_id):
        raise ValueError('Color, registered depth and CameraInfo must share optical frame')
    if (color.width <= 0 or color.height <= 0
            or (color.width, color.height) != (depth.width, depth.height)
            or (color.width, color.height) != (info.width, info.height)):
        raise ValueError('Color, registered depth and calibration dimensions differ or are empty')
    if (info.binning_x > 1 or info.binning_y > 1
            or info.roi.x_offset or info.roi.y_offset):
        raise ValueError('Binned/cropped calibration requires explicit normalization')
    intrinsics = (info.p[0], info.p[5], info.p[2], info.p[6])
    if (not all(math.isfinite(v) for v in info.p)
            or intrinsics[0] <= 0 or intrinsics[1] <= 0
            or info.p[1] != 0 or info.p[4] != 0
            or info.p[3] != 0 or info.p[7] != 0
            or tuple(info.p[8:12]) != (0., 0., 1., 0.)):
        raise ValueError('Invalid or unsupported rectified color projection matrix')
    if color.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
        raise ValueError('Expected color RGB/BGR image')
    if depth.encoding not in ('16UC1', '32FC1'):
        raise ValueError('Depth must be 16UC1 millimetres or 32FC1 metres')
    return intrinsics
