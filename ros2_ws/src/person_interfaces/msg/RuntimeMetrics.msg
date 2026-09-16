# One wall-clock window. Empty timing samples are NaN, rates are zero.
std_msgs/Header header
string source
float64 window_s
uint32 input_count
uint32 output_count
float64 input_fps
float64 output_fps
float64 processing_ms
float64 processing_p95_ms
float64 rgbd_ms
float64 rgbd_p95_ms
float64 observation_age_ms
float64 observation_age_p95_ms
float64 control_latency_ms
float64 control_latency_p95_ms
