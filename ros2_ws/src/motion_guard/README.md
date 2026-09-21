# 跟随速度保护

数据链路：`person_follower → /control/cmd_vel_request (TwistStamped) → motion_guard → /cmd_vel (Twist) → 底盘驱动`。跟随节点不再直接发布底盘速度。此包不启动底盘、传感器或自动授权。

```bash
ros2 launch motion_guard follow.launch.py
```

默认 PERCEPTION_ONLY；现有 A/B/red 纯感知入口保持不变。安全配置在 `config/safety.yaml`，均为启动后只读参数。填写实测外参、车体尺寸与停车模型后，确认 geometry_confirmed、mount_calibrated、stopping_model_confirmed，显式设置 motion_enabled:=true 才进入 STANDBY。即使开启此选项也不会自动运动。

```bash
ros2 service call /control/arm std_srvs/srv/Trigger '{}'
ros2 service call /control/stop std_srvs/srv/Trigger '{}'
ros2 service call /control/disarm std_srvs/srv/Trigger '{}'
```

授权要求唯一请求/目标/扫描/最终速度发布者、实时真实目标、带时戳请求、实时完整扫描和扫描时刻的 laser→base_link TF，并通过障碍检查。不能用最新 TF 替代缺失的历史 TF。当前只支持水平二维雷达，倾斜安装被拒绝。

授权后任何检查失败都会输出零速度并进入 FAULT。数据恢复、目标重新出现或节点重启不会自动恢复运动；需要再调 arm。stop/disarm 立即发零并回待命。节点使用 steady timer/monotonic 接收看门狗，ROS 时钟停止也会触发超时。A 的源观测时刻未知仍按已有契约检查发布与接收时效；B/red 要求观测本身新鲜。

`/control/state` 是 JSON String，含 mode、ready、reason、last_fault、motion_enabled。ready 只表示当前输入通过检查，不等于已授权。

停车包络用包围整车的圆再加反应距离、刹车距离、裕量和离散余量，检查各个方向，因此直行与原地转弯均受保护，也覆盖指令突然变化时的旧运动。它比精确矩形轨迹保守，窄空间可能拒绝运动。本版本只停车，不绕障、不识别可跨越物，也不是硬件急停。

尺寸应是以 base_link 为中心、能包住车体与突出部件的边界。reaction_s 必须涵盖传感器延迟、软件周期及执行延迟，代码至少要求 scan_timeout_s + 0.5 秒底盘命令超时 + 0.05 秒保护周期；deceleration_mps2 必须采用实测保守值。默认值是未确认占位，不是实测制动能力。雷达高度以下/以上的障碍、玻璃和不可见物体仍是二维感知的限制。

默认拒绝缺角度、NaN、越界和无回波；只有确认设备用 +inf 表示量程内无障碍后，才可设 allow_infinite_clear=true。不应为了消除故障提示而盲目放宽。未标定/未确认配置只阻止控制，不改变原感知图像、目标或雷达输出。

默认目标来源是 astra、光学 frame 是 astra_depth_optical_frame。用于 red_object/yolo 时须同时设置 expected_source 和实际 target_frame，不能将 target_state_base 直接接入旧光学偏角控制器。新的受保护启动入口跟随距离默认 1 m；旧 FollowConfig 未改变，显式启动参数可调整，但近障保护不会因此绕过。

记录/回放：`bash scripts/record_follow.sh [目录]` 记录目标、扫描、TF、请求、保护状态、最终速度和里程计；`bash scripts/replay_follow.sh BAG目录` 限定域 174/本机和允许的话题，并重映射到 /replay/*，不会把历史命令送入实际 /cmd_vel。回放用于诊断，不能当成实时传感器解锁保护。

进程被 SIGKILL/系统崩溃时软件节点不能发最后一帧零，仍依赖底盘驱动已有命令超时与反馈超时停车；整机掉线保护必须由 STM32 固件/硬件兜底，当前代码测试不能替代实车验证。

审查补充：速度请求的 `base_frame` 由 safety_config 传给直接跟随器；可通过 launch 的 `target_frame` 显式覆盖目标坐标系。红色入口默认使用 camera_color_optical_frame（可用 TARGET_FRAME 修改）。雷达 `range_min` 形成的盲区若超出已确认的车体矩形，拒绝授权。SIGTERM 会先发送零速度再销毁 ROS 节点。
