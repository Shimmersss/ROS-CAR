#!/usr/bin/env python3
"""Camera mounting extrinsic from a ChArUco board lying flat on the floor; no ROS or vehicle output.

Placement: board face up on the floor in front of the car. In the live view the red axis (board X)
must point straight forward, away from the car, and the green axis (board Y) to the car's right.
--board-x/--board-y are the tape-measured position of the axis origin corner in the base frame
(forward/left positive, metres). Floor height in the base frame plus board thickness is --board-z.

Keys: S/space add frame (keep car and board still), C compute, R reset frames, Q quit.
Offline: --images a.png b.png ... (all frames of the same placement).
Height, pitch and roll come from the board plane; x, y and yaw are only as good as the tape and
alignment. Factory color intrinsics are used (checked in WORKLOG 2026-10-08).
"""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from charuco_calib_mac import ROOT, detect, load_factory

MIN_FRAMES = 10
# Board X forward, board Y to the right, board Z into the floor.
R_BASE_BOARD = np.array([[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]])


def quaternion_xyzw(rotation):
    w = np.sqrt(max(0., 1.+np.trace(rotation)))/2
    if w > 1e-6:
        x = (rotation[2, 1]-rotation[1, 2])/(4*w)
        y = (rotation[0, 2]-rotation[2, 0])/(4*w)
        z = (rotation[1, 0]-rotation[0, 1])/(4*w)
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i+1) % 3, (i+2) % 3
        q = np.zeros(3)
        q[i] = np.sqrt(max(0., 1.+rotation[i, i]-rotation[j, j]-rotation[k, k]))/2
        q[j] = (rotation[j, i]+rotation[i, j])/(4*q[i])
        q[k] = (rotation[k, i]+rotation[i, k])/(4*q[i])
        w = (rotation[k, j]-rotation[j, k])/(4*q[i])
        x, y, z = q
    q = np.array([x, y, z, w])
    return (q/np.linalg.norm(q)*(1 if q[3] >= 0 else -1)).tolist()


def solve(frames, matrix, dist, board_origin):
    obj = np.vstack([f[0] for f in frames]).astype(np.float32)
    img = np.vstack([f[1] for f in frames]).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, img, matrix, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        raise RuntimeError('PnP failed')
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, matrix, dist, rvec, tvec)
    projected, _ = cv2.projectPoints(obj, rvec, tvec, matrix, dist)
    rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2)-img)**2, axis=1))))
    r_cam_board, _ = cv2.Rodrigues(rvec)
    # Per-frame spread shows whether the car/board stayed still.
    heights = []
    for o, i in frames:
        ok, rv, tv = cv2.solvePnP(o, i, matrix, dist, flags=cv2.SOLVEPNP_IPPE)
        r, _ = cv2.Rodrigues(rv)
        heights.append(float(-(r.T@tv.reshape(3))[2]))
    r_board_cam = r_cam_board.T
    t_board_cam = -r_board_cam@tvec.reshape(3)
    rotation = R_BASE_BOARD@r_board_cam
    translation = R_BASE_BOARD@t_board_cam+board_origin
    optical_z, optical_x = rotation[:, 2], rotation[:, 0]
    return dict(
        frames=len(frames),
        reprojection_rms_px=rms,
        base_to_camera_color_optical_frame=dict(xyz=translation.tolist(), quaternion_xyzw=quaternion_xyzw(rotation)),
        readable=dict(
            camera_height_above_board_m=float(-t_board_cam[2]),
            optical_axis_pitch_down_deg=float(np.degrees(np.arcsin(-optical_z[2]))),
            optical_axis_yaw_left_deg=float(np.degrees(np.arctan2(optical_z[1], optical_z[0]))),
            image_x_axis_tilt_deg=float(np.degrees(np.arcsin(-optical_x[2]))),
            per_frame_height_spread_m=float(np.ptp(heights)),
        ),
    )


def check(report):
    warnings = []
    readable = report['readable']
    if readable['camera_height_above_board_m'] <= 0:
        warnings.append('Camera is not above the board: board face down or axes misread')
    if abs(readable['optical_axis_yaw_left_deg']) > 45:
        warnings.append('Optical axis is not facing forward: check red axis points away from the car')
    if readable['per_frame_height_spread_m'] > .005:
        warnings.append('Height varies >5 mm between frames: car or board moved, recapture')
    if report['reprojection_rms_px'] > 1.:
        warnings.append('Reprojection >1 px: board not flat or wrong --square/--marker')
    report['warnings'] = warnings
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--calibration', default=str(ROOT/'ros2_ws/src/perception_bringup/config/gemini_AY2755200PW.json'))
    parser.add_argument('--cols', type=int, default=7)
    parser.add_argument('--rows', type=int, default=5)
    parser.add_argument('--square', type=float, required=True)
    parser.add_argument('--marker', type=float, required=True)
    parser.add_argument('--dictionary', default='DICT_5X5_50')
    parser.add_argument('--board-x', type=float, required=True, help='Origin corner forward of base origin, m')
    parser.add_argument('--board-y', type=float, required=True, help='Origin corner left of base origin, m')
    parser.add_argument('--board-z', type=float, default=0., help='Board top surface height in base frame, m')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--out', default=str(ROOT/f'artifacts/charuco-mount-{time.strftime("%Y%m%d-%H%M%S")}'))
    parser.add_argument('--images', nargs='*')
    args = parser.parse_args()
    if not 0 < args.marker < args.square:
        parser.error('--marker must be positive and smaller than --square')

    calibration, matrix, dist, size = load_factory(args.calibration)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dictionary))
    board = cv2.aruco.CharucoBoard((args.cols, args.rows), args.square, args.marker, dictionary)
    detector = cv2.aruco.CharucoDetector(board)
    origin = np.array([args.board_x, args.board_y, args.board_z])
    out = Path(args.out)

    def finish(frames):
        if len(frames) < MIN_FRAMES:
            print(f'Need at least {MIN_FRAMES} frames, have {len(frames)}')
            return
        report = check(solve(frames, matrix, dist, origin))
        report.update(base_frame=args.base_frame, child_frame='camera_color_optical_frame',
                      board_origin_in_base_m=origin.tolist(), serial=calibration['serial'],
                      square_m=args.square, marker_m=args.marker,
                      note='Tape-measured x/y/yaw; not yet a confirmed mount. camera_link needs the driver TF.')
        out.mkdir(parents=True, exist_ok=True)
        (out/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
        print(json.dumps(report, indent=2, ensure_ascii=False))
        print(f'Report written to {out/"report.json"}')

    if args.images:
        frames = []
        for name in args.images:
            gray = cv2.imread(name, cv2.IMREAD_GRAYSCALE)
            if gray is None or gray.shape[::-1] != size:
                raise RuntimeError(f'{name}: unreadable or not {size[0]}x{size[1]}')
            found = detect(detector, board, gray)
            if found is not None:
                frames.append(found[:2])
        finish(frames)
        return

    capture = cv2.VideoCapture(args.camera, cv2.CAP_AVFOUNDATION)
    if not capture.isOpened():
        raise RuntimeError('Cannot open selected camera')
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    frames = []
    title = 'ChArUco floor mount - S add, C compute, R reset, Q quit'
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 960, 720)
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError('Camera read failed')
            if frame.shape[1::-1] != size:
                raise RuntimeError(f'Camera gives {frame.shape[1]}x{frame.shape[0]}, expected {size[0]}x{size[1]}')
            found = detect(detector, board, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            shown = frame.copy()
            text = f'frames {len(frames)}/{MIN_FRAMES}  board not found'
            if found is not None:
                obj, img, corners, ids, _ = found
                cv2.aruco.drawDetectedCornersCharuco(shown, corners, ids, (0, 255, 0))
                ok, rvec, tvec = cv2.solvePnP(obj, img, matrix, dist, flags=cv2.SOLVEPNP_IPPE)
                if ok:
                    cv2.drawFrameAxes(shown, matrix, dist, rvec, tvec, args.square*3)
                    r, _ = cv2.Rodrigues(rvec)
                    text = (f'frames {len(frames)}/{MIN_FRAMES}  corners {len(ids)}'
                            f'  height {-(r.T@tvec.reshape(3))[2]:.3f} m')
            cv2.putText(shown, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
            cv2.imshow(title, shown)
            key = cv2.waitKey(1) & 255
            if key in (27, ord('q')):
                break
            if key in (ord('s'), ord(' ')) and found is not None:
                (out/'frames').mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(out/'frames'/f'floor_{len(frames):03d}.png'), frame)
                frames.append(found[:2])
            if key == ord('r'):
                frames.clear()
            if key == ord('c'):
                finish(frames)
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
