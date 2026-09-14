# One selected target, never a motor command. See docs/interfaces.md.
uint8 NOT_READY=0
uint8 SEARCHING=1
uint8 TRACKING=2
uint8 LOST=3
uint8 STALE=4

# State publication time; frame_id identifies the coordinate frame of position.
std_msgs/Header header
# Actual sensor observation time; zero means unavailable.
builtin_interfaces/Time observation_stamp
string source
bool is_simulated
uint8 status
string detail
string target_id
# true only for a fresh accepted observation (or explicit simulated observation).
bool position_valid
geometry_msgs/Point position
# Metres and radians, using optical X right, Y down, Z forward.
float32 horizontal_distance_m
float32 bearing_rad
# NaN if unavailable; never silently use zero as a valid distance.
float32 measurement_age_s
float32 confidence
