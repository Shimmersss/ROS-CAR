# Exact source RGB acquisition header. Zero timestamp on invalid heartbeat.
std_msgs/Header header
bool enabled
bool valid
string detail
float32 processing_ms
float32 observation_age_s
PersonIdentity[] persons
