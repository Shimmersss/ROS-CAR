#!/usr/bin/env python3
"""ChArUco check of the Gemini factory color intrinsics on the Mac; no ROS, depth or vehicle output.

Live: shows board distance (PnP with factory intrinsics) and reprojection error.
Keys: S/space save view, C calibrate saved views and compare with factory, Q quit.
Offline: --images a.png b.png ... runs the same comparison on saved frames.
--color-mode full checks the 2592x1944-shrunk stream used by record_gemini_mac.py against the
intrinsics derived from scripts/config/gemini_color_mode_*.json (the "factory" fields of the report
then hold those derived values).
"""
import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from gemini_color import ColorCapture, load_mapping, mode_intrinsics  # noqa: E402
MIN_CORNERS = 12
MIN_VIEWS = 12


def load_factory(path):
    calibration = json.loads(Path(path).read_text())
    i = calibration['color_intrinsic']
    matrix = np.array([[i['fx'], 0, i['cx']], [0, i['fy'], i['cy']], [0, 0, 1]], dtype=float)
    dist = np.asarray(calibration['color_distortion'], dtype=float)
    if dist.shape != (8,) or not np.isfinite(matrix).all() or not np.isfinite(dist).all():
        raise ValueError('Invalid factory color calibration')
    return calibration, matrix, dist, (i['width'], i['height'])


def detect(detector, board, gray):
    corners, ids, markers, _ = detector.detectBoard(gray)
    if ids is None or len(ids) < MIN_CORNERS:
        return None
    obj, img = board.matchImagePoints(corners, ids)
    return obj.reshape(-1, 3), img.reshape(-1, 2), corners, ids, markers


def pnp(obj, img, matrix, dist):
    ok, rvec, tvec = cv2.solvePnP(obj, img, matrix, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, matrix, dist, rvec, tvec)
    projected, _ = cv2.projectPoints(obj, rvec, tvec, matrix, dist)
    rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2)-img)**2, axis=1))))
    return rvec, tvec, rms


def board_center_distance(board, rvec, tvec):
    cols, rows = board.getChessboardSize()
    square = board.getSquareLength()
    center = np.array([cols*square/2, rows*square/2, 0.])
    rotation, _ = cv2.Rodrigues(rvec)
    return float(np.linalg.norm(rotation@center+tvec.reshape(3)))


def compare(views, size, matrix, dist):
    if len(views) < MIN_VIEWS:
        raise RuntimeError(f'Need at least {MIN_VIEWS} saved views, have {len(views)}')
    objs = [v[0].astype(np.float32) for v in views]
    imgs = [v[1].astype(np.float32) for v in views]
    rms, k, d, _, _ = cv2.calibrateCamera(objs, imgs, size, None, None,
                                          flags=cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_ASPECT_RATIO)
    factory = [pnp(o, i, matrix, dist)[2] for o, i in zip(objs, imgs)]
    fitted = [pnp(o, i, k, d)[2] for o, i in zip(objs, imgs)]
    return dict(
        views=len(views),
        image_size=list(size),
        calibrated=dict(fx=k[0, 0], fy=k[1, 1], cx=k[0, 2], cy=k[1, 2], distortion=d.reshape(-1).tolist(),
                        overall_rms_px=rms),
        factory=dict(fx=matrix[0, 0], fy=matrix[1, 1], cx=matrix[0, 2], cy=matrix[1, 2],
                     distortion=dist.tolist()),
        delta=dict(fx_pct=100*(k[0, 0]/matrix[0, 0]-1), fy_pct=100*(k[1, 1]/matrix[1, 1]-1),
                   cx_px=k[0, 2]-matrix[0, 2], cy_px=k[1, 2]-matrix[1, 2]),
        per_view_pnp_rms_px=dict(factory_median=float(np.median(factory)), factory_max=float(np.max(factory)),
                                 calibrated_median=float(np.median(fitted)), calibrated_max=float(np.max(fitted))),
        note='Color intrinsics only; no depth, D2C registration or mounting extrinsics are checked.',
    )


def write_report(out, report):
    out.mkdir(parents=True, exist_ok=True)
    path = out/'report.json'
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False, default=float))
    print(json.dumps(report, indent=2, ensure_ascii=False, default=float))
    print(f'Report written to {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--calibration', default=str(ROOT/'ros2_ws/src/perception_bringup/config/gemini_AY2755200PW.json'))
    parser.add_argument('--cols', type=int, default=7)
    parser.add_argument('--rows', type=int, default=5)
    parser.add_argument('--square', type=float, required=True, help='Measured checker side in metres, e.g. 0.030')
    parser.add_argument('--marker', type=float, required=True, help='Measured marker side in metres, e.g. 0.022')
    parser.add_argument('--dictionary', default='DICT_5X5_50')
    parser.add_argument('--out', default=str(ROOT/f'artifacts/charuco-{time.strftime("%Y%m%d-%H%M%S")}'))
    parser.add_argument('--images', nargs='*', help='Offline frames instead of the live camera')
    parser.add_argument('--color-mode', choices=('native', 'full'), default='native',
                        help='native: factory 640x480 mode; full: 2592x1944 shrunk to 640x480 as recorded')
    args = parser.parse_args()
    if not 0 < args.marker < args.square:
        parser.error('--marker must be positive and smaller than --square')

    calibration, matrix, dist, size = load_factory(args.calibration)
    if args.color_mode == 'full':
        mapping = load_mapping()
        i = mode_intrinsics(calibration['color_intrinsic'], mapping)
        matrix = np.array([[i['fx'], 0, i['cx']], [0, i['fy'], i['cy']], [0, 0, 1]], dtype=float)
        print(f'Full mode: reference intrinsics from mapping {mapping.get("source")}: '
              f'fx {i["fx"]:.2f} cx {i["cx"]:.2f} cy {i["cy"]:.2f}')
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dictionary))
    board = cv2.aruco.CharucoBoard((args.cols, args.rows), args.square, args.marker, dictionary)
    detector = cv2.aruco.CharucoDetector(board)
    out = Path(args.out)
    print(f'Factory calibration: {calibration["serial"]} {size[0]}x{size[1]}')

    if args.images:
        views = []
        for name in args.images:
            gray = cv2.imread(name, cv2.IMREAD_GRAYSCALE)
            if gray is None or gray.shape[::-1] != size:
                raise RuntimeError(f'{name}: unreadable or not {size[0]}x{size[1]}')
            found = detect(detector, board, gray)
            print(f'{name}: {"skip" if found is None else len(found[0])} corners')
            if found is not None:
                views.append(found[:2])
        write_report(out, compare(views, size, matrix, dist))
        return

    capture = ColorCapture(args.camera, args.color_mode)
    views = []
    title = 'ChArUco factory-intrinsic check - S save, C compare, Q quit'
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 960, 720)
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError('Camera read failed')
            if frame.shape[1::-1] != size:
                raise RuntimeError(f'Camera gives {frame.shape[1]}x{frame.shape[0]}, factory calibration is '
                                   f'{size[0]}x{size[1]}; wrong camera index or mode')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found = detect(detector, board, gray)
            shown = frame.copy()
            text = f'views {len(views)}/{MIN_VIEWS}  board not found'
            if found is not None:
                obj, img, corners, ids, markers = found
                cv2.aruco.drawDetectedCornersCharuco(shown, corners, ids, (0, 255, 0))
                solved = pnp(obj, img, matrix, dist)
                if solved is not None:
                    rvec, tvec, rms = solved
                    cv2.drawFrameAxes(shown, matrix, dist, rvec, tvec, args.square*2)
                    text = (f'views {len(views)}/{MIN_VIEWS}  corners {len(ids)}  center {board_center_distance(board, rvec, tvec):.3f} m'
                            f'  reproj {rms:.2f} px')
            cv2.putText(shown, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
            cv2.imshow(title, shown)
            key = cv2.waitKey(1) & 255
            if key in (27, ord('q')):
                break
            if key in (ord('s'), ord(' ')) and found is not None:
                frames = out/'frames'
                frames.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(frames/f'view_{len(views):03d}.png'), frame)
                views.append(found[:2])
                print(f'Saved view {len(views)} ({len(found[3])} corners)')
            if key == ord('c'):
                try:
                    write_report(out, compare(views, size, matrix, dist))
                except RuntimeError as error:
                    print(error, file=sys.stderr)
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
