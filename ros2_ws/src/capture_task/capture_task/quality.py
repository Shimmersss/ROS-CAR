"""Diagnostic image metrics, not medical/object recognition accuracy."""
import cv2
import numpy as np


def measure(rgb):
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    means = rgb.mean(axis=(0,1))
    return dict(brightness=float(gray.mean()), sharpness=float(cv2.Laplacian(gray, cv2.CV_64F).var()),
                rb_ratio=float(means[0]/max(1.,means[2])), width=int(rgb.shape[1]), height=int(rgb.shape[0]))


def warnings(metrics, task):
    q = task['quality']
    problems = []
    if metrics['width'] < task['min_width']:
        problems.append('width_below_minimum')
    for field in ('brightness','rb_ratio'):
        if not q[field][0] <= metrics[field] <= q[field][1]:
            problems.append(field+'_out_of_range')
    if metrics['sharpness'] < q['min_sharpness']:
        problems.append('sharpness_below_minimum')
    return problems
