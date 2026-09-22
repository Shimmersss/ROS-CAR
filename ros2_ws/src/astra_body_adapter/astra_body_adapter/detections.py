"""Humble Detection2DArray: original image pixels, no fabricated 3D pose."""
import copy
import math
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


def detection_array(header, candidates):
    """Candidates: (xyxy, class_id, score, identity); NaN denotes unknown score."""
    output = Detection2DArray()
    output.header = copy.deepcopy(header)
    for box, class_id, score, identity in candidates:
        x1, y1, x2, y2 = map(float, box)
        if not all(map(math.isfinite, (x1, y1, x2, y2))) or x2 <= x1 or y2 <= y1:
            raise ValueError('Invalid detection box')
        detection = Detection2D()
        detection.header = copy.deepcopy(header)
        detection.id = identity
        detection.bbox.center.position.x = (x1+x2)/2
        detection.bbox.center.position.y = (y1+y2)/2
        detection.bbox.size_x = x2-x1
        detection.bbox.size_y = y2-y1
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = class_id
        hypothesis.hypothesis.score = float(score)
        # This interface contains 2D evidence only. Pose must never imply origin.
        hypothesis.pose.pose.position.x = math.nan
        hypothesis.pose.pose.position.y = math.nan
        hypothesis.pose.pose.position.z = math.nan
        detection.results = [hypothesis]
        output.detections.append(detection)
    return output
