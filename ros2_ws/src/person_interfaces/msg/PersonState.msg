uint8 UNKNOWN=0
uint8 STANDING=1
uint8 SITTING_CROUCHING=2
uint8 LYING=3
uint8 FALLEN=4
uint8 FALL_NONE=0
uint8 FALL_SUSPECTED=1
uint8 FALL_CONFIRMED=2
string track_id
float32[4] box
float32 confidence
# COCO17 order; optical pixels, zero confidence denotes unavailable 2D point.
geometry_msgs/Point[17] keypoints_2d
float32[17] keypoint_confidences
geometry_msgs/Point[17] keypoints_3d
bool[17] keypoints_3d_valid
uint8 posture
uint8 fall_stage
string detail

# Current body-depth observation for a virtual representative point, NOT a joint.
# Supports projecting the visible ankle rays when their own depth is missing.
# No held/predicted depth here. NaN when invalid; does not populate keypoints_3d.
bool body_depth_valid
float32 body_depth_m
string body_depth_source
