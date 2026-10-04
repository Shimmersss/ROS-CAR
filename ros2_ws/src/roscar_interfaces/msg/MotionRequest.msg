# Semantic intent. MOVE speed is m/s; TURN speed is rad/s.
# Style: NORMAL / MICRO / IN_PLACE. Chassis adapter validates capability.
# authority_epoch must match current ARMED_MANUAL authority.
builtin_interfaces/Time stamp
string request_id
uint64 authority_epoch
string action
string direction
string style
bool speed_specified
float64 speed
bool duration_specified
float64 duration_s
