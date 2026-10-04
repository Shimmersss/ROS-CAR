string track_id
# True only for an accepted fresh contact estimate (legacy mono topic may predict briefly).
bool valid
# Person ground-contact point in the array frame (z = confirmed ground plane); NaN if invalid.
geometry_msgs/Point position
# Metres/radians in the base frame: hypot(X,Y), atan2(Y,X), positive left. NaN if invalid.
float32 horizontal_distance_m
float32 bearing_rad
# Conditional planar uncertainty from pixels, height priors and anchor spread.
# Does not certify contact, mount accuracy, or the correctness of pose assumptions.
float32 std_m
float32 confidence
# Legacy mono: feet_ankles, height_prior, predicted.
# Unified: depth_ankles, depth_body_projection, mono_ankles, body_contact, fused_contact.
# Fused source names and body-mapping age are in detail. Empty method when invalid.
string method
# Seconds since the last real measurement; prediction never resets it.
float32 measurement_age_s
string detail
