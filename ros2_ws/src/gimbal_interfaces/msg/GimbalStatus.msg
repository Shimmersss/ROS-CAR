# Latest MCU status. header.stamp is the MCU sample time mapped to ROS time once
# time_synced is true; otherwise it is the host receive time.
uint8 SOURCE_NONE=0
uint8 SOURCE_ENCODER=1
uint8 SOURCE_GYRO_HOMED=2
std_msgs/Header header
bool connected
bool time_synced
uint8 pan_source
bool pan_valid
bool homed
bool magnet_ok
bool imu_ok
bool imu_calibrating
bool cmd_timeout
bool pan_limit
bool servos_enabled
uint8 fault
float32 pan_rad
float32 pan_rate_radps
float32 imu_pitch_rad        # looking down positive
float32 imu_roll_rad
uint32 ms_since_home         # 0xFFFFFFFF: never homed / encoder source
float32 clock_offset_s       # ROS time minus MCU time
float32 sync_rtt_s           # best recent round trip
uint32 clock_steps           # host clock steps/MCU restarts detected (offset re-anchored)
uint32 frames_ok
uint32 crc_errors
string detail
