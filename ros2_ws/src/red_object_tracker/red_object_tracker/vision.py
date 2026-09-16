"""Red components, conservative temporal association and mask-only depth."""
from dataclasses import dataclass
import math
import cv2
import numpy as np


@dataclass
class Component:
    box: tuple
    area: int
    mask: np.ndarray


def detect(image, hue_low_max=10, hue_high_min=170, saturation_min=100,
           value_min=70, min_area_fraction=.001):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, saturation_min, value_min), (hue_low_max, 255, 255))
    mask |= cv2.inRange(hsv, (hue_high_min, saturation_min, value_min), (179, 255, 255))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    found = []
    for label in range(1, count):
        x, y, w, h, area = stats[label]
        if area >= max(12, image.shape[0]*image.shape[1]*min_area_fraction):
            found.append(Component((int(x), int(y), int(x+w), int(y+h)),
                                   int(area), (labels == label).astype(np.uint8)*255))
    return sorted(found, key=lambda item: (-item.area, item.box)), mask


def affinity(a, b):
    ax, ay, ar, ab = a.box; bx, by, br, bb = b.box
    inter = max(0, min(ar, br)-max(ax, bx))*max(0, min(ab, bb)-max(ay, by))
    iou = inter / max(1, (ar-ax)*(ab-ay)+(br-bx)*(bb-by)-inter)
    shift = math.hypot((ax+ar-bx-br)/2, (ay+ab-by-bb)/2)
    size = max(1, math.hypot(ar-ax, ab-ay))
    ratio = b.area / a.area
    if not .25 <= ratio <= 4 or not (iou >= .1 or shift <= .5*size):
        return None
    return iou - .25*shift/size


class Selection:
    def __init__(self, confirm_frames=3, lost_timeout_s=1.):
        self.confirm_frames = confirm_frames
        self.lost_timeout_s = lost_timeout_s
        self.serial = 0
        self.clear()

    def clear(self):
        self.target_id = ''
        self.previous = None
        self.selected = None
        self.hits = 0
        self.last_seen = None

    def update(self, components, now):
        self.selected = None
        if self.last_seen is not None and now-self.last_seen >= self.lost_timeout_s:
            self.clear()
        chosen = None
        if self.previous is not None:
            ranked = [(affinity(self.previous, c), c) for c in components]
            ranked = [(score, c) for score, c in ranked if score is not None]
            if ranked:
                chosen = max(ranked, key=lambda item: item[0])[1]
        else:
            chosen = components[0] if components else None
        if chosen is None:
            self.hits = 0
            if not self.target_id:
                self.previous = None
            return
        self.previous = chosen
        self.last_seen = now
        self.hits += 1
        if not self.target_id and self.hits >= self.confirm_frames:
            self.serial += 1
            self.target_id = f'red:{self.serial}'
        if self.target_id:
            self.selected = chosen


def measure(depth, mask, intrinsics):
    # Erode the component so object/background boundaries do not dominate depth.
    interior = cv2.erode(mask, np.ones((3, 3), np.uint8)) > 0
    valid = interior & np.isfinite(depth) & (depth >= .2) & (depth <= 8.)
    if valid.sum() < max(12, interior.sum()*.3):
        return None
    values = depth[valid]
    z = float(np.median(values))
    if np.percentile(values, 75)-np.percentile(values, 25) > max(.25, .2*z):
        return None
    fx, fy, cx, cy = intrinsics
    if not all(map(math.isfinite, intrinsics)) or fx <= 0 or fy <= 0:
        return None
    v, u = np.nonzero(valid & (np.abs(depth-z) <= max(.1, .1*z)))
    if not len(u):
        return None
    return ((float(np.median(u))-cx)*z/fx, (float(np.median(v))-cy)*z/fy, z)
