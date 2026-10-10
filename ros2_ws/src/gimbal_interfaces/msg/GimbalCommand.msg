# Desired gimbal motion. gimbal_bridge forwards it to the MCU at 50 Hz while it is fresh,
# otherwise sends HOLD. See docs/云台跟随设计与串口协议.md §11.2.
uint8 DISABLE=0
uint8 HOLD=1
uint8 POSITION=2
uint8 RATE=3
uint8 HOME=4
std_msgs/Header header
uint8 mode
float32 pan_target_rad       # relative to the car body, 0 = straight ahead, left positive
float32 tilt_target_rad      # IMU pitch relative to gravity, looking down positive
float32 pan_rate_ff_radps    # POSITION: feed-forward; RATE: the pan rate command
float32 max_pan_rate_radps
float32 max_tilt_rate_radps
