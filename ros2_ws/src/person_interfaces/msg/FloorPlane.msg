# Floor plane fitted by RANSAC from registered depth, in header.frame_id (optical frame).
# Evidence only: it never confirms the ground plane or gravity by itself.
std_msgs/Header header
bool valid                 # this frame's fit passed residual/inlier/tilt/height checks
bool stable                # consistent with the preceding confirm_frames fits (no gap)
geometry_msgs/Vector3 up   # unit normal from the floor towards the camera
float32 height_m           # optical centre above the plane
float32 pitch_up_deg       # optical axis elevation relative to the plane (positive = looking up)
float32 roll_deg           # image x-axis tilt relative to the plane
uint32 inliers
float32 inlier_fraction
float32 rms_m
string detail
