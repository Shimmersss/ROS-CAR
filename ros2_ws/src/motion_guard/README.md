# 跟随速度保护

当前跟随入口（`scripts/start_project.sh` / `start_robot.sh` / `start_c.sh` 的 `WITH_FOLLOWER=true`）走 `person_follower → /control/cmd_vel_request → motion_guard → /cmd_vel → 底盘驱动`，由 guard 唯一输出最终速度。（2026-09-28 曾短暂改为跟随器直发 `/cmd_vel`，语音 V5 起恢复 guard。）

```bash
ros2 launch motion_guard follow.launch.py
```

默认 PERCEPTION_ONLY；现有 A/B/red 纯感知入口保持不变。安全配置在 `config/safety.yaml`，均为启动后只读参数。填写实测外参、车体尺寸与停车模型后，确认 geometry_confirmed、mount_calibrated、stopping_model_confirmed，显式设置 motion_enabled:=true 才进入 STANDBY。即使开启此选项也不会自动运动。

本版本固定按 `radar_required:=false` 运行，EXTERNAL 和 FOLLOW 均不等待 `/scan`、雷达 TF 或雷达净空结果即可进入后续授权检查；雷达不构成任何控制门禁。此模式没有雷达障碍检查，必须由现场人员清空环境并保持人工急停准备。`true` 仅保留给未来单独启用雷达的版本，不属于本版本运行配置。

`auto_arm` 默认为 false。自启动跟随可显式启用 `auto_arm:=true`：只有新鲜有效目标和跟随请求同时满足时才自动 arm（只从 STANDBY 进入）；目标失联超出宽限进入 FAULT，默认不会自动续动，见下面的 `target_auto_recover`。

`target_grace_s`（只读参数，默认 0 即关闭，上限 1 秒；`follow.launch.py` 同名启动参数）：仅在 FOLLOW 模式、且失败原因是 `target_timeout`/`target_timestamp`/`target_invalid` 时，从最后一次有效周期起的这段时间内保持 ARMED，输出由最后一次有效指令线性衰减到零（开启雷达检查时直接输出零，因为此期间无法做净空检查），`reason` 为 `target_grace:...`；目标在宽限内恢复有效即继续跟随，超过宽限才进入 FAULT 并照旧锁存。其他任何检查失败（配置未确认、发布者异常、请求超时/无效、雷达、TF 等）都不享受宽限，立即 FAULT。`scripts/start_project.sh` 的跟随入口默认 `FOLLOW_TARGET_GRACE_S=0.6`：在当前最高 0.2 m/s 下，宽限期内按线性衰减最多多行驶约 6 cm。该值为用户 2026-10-12 确认，未经实车制动验证。

`target_auto_recover`（只读参数，默认 false；`follow.launch.py` 同名启动参数）：仅在 FOLLOW 模式、且锁存的故障原因是 `target_timeout`/`target_timestamp`/`target_invalid` 时，一旦本周期全部检查重新通过（目标新鲜有效、请求有效、发布者唯一、确认开关等），自动回到 ARMED 并记日志 `re-armed`，效果同方案 A 跟随器“目标回来就接着走”。请求超时/无效、发布者异常、配置未确认、雷达/TF 等其他故障仍锁存，需显式 arm；stop/disarm 回到 STANDBY 不是故障，不受此参数影响（是否从 STANDBY 自动 arm 仍只由 `auto_arm` 决定）。`start_project.sh` 跟随入口默认 `FOLLOW_TARGET_AUTO_RECOVER=true`（用户 2026-10-12 确认），设为 false 恢复锁存。语音会话发起的跟随中，路由器看到非 ARMED 会自行停止并回 IDLE，不受此参数影响。

自启动入口可单独传入 `geometry_confirmed`、`mount_calibrated`、`stopping_model_confirmed`；这只改变该入口的启动参数，不修改安全配置文件默认值。

```bash
ros2 service call /control/arm std_srvs/srv/Trigger '{}'
ros2 service call /control/stop std_srvs/srv/Trigger '{}'
ros2 service call /control/disarm std_srvs/srv/Trigger '{}'
```

授权要求唯一请求/目标/扫描/最终速度发布者、实时真实目标、带时戳请求、实时完整扫描和扫描时刻的 laser→base_link TF，并通过障碍检查。不能用最新 TF 替代缺失的历史 TF。当前只支持水平二维雷达，倾斜安装被拒绝。

授权后任何检查失败都会输出零速度并进入 FAULT。数据恢复或节点重启不会自动恢复运动，需要再调 arm；唯一例外是显式开启 `target_auto_recover` 时的目标类故障。stop/disarm 立即发零并回待命。节点使用 steady timer/monotonic 接收看门狗，ROS 时钟停止也会触发超时。A 的源观测时刻未知仍按已有契约检查发布与接收时效；B/red 要求观测本身新鲜。

`/control/state` 是 JSON String，含 mode、ready、reason、last_fault、motion_enabled。ready 只表示当前输入通过检查，不等于已授权。

停车包络用包围整车的圆再加反应距离、刹车距离、裕量和离散余量，检查各个方向，因此直行与原地转弯均受保护，也覆盖指令突然变化时的旧运动。它比精确矩形轨迹保守，窄空间可能拒绝运动。本版本只停车，不绕障、不识别可跨越物，也不是硬件急停。

尺寸应是以 base_link 为中心、能包住车体与突出部件的边界。reaction_s 必须涵盖传感器延迟、软件周期及执行延迟，代码至少要求 scan_timeout_s + 0.5 秒底盘命令超时 + 0.05 秒保护周期；deceleration_mps2 必须采用实测保守值。默认值是未确认占位，不是实测制动能力。雷达高度以下/以上的障碍、玻璃和不可见物体仍是二维感知的限制。

默认拒绝缺角度、NaN、越界和无回波；只有确认设备用 +inf 表示量程内无障碍后，才可设 allow_infinite_clear=true。不应为了消除故障提示而盲目放宽。未标定/未确认配置只阻止控制，不改变原感知图像、目标或雷达输出。

默认目标来源是 astra、光学 frame 是 astra_depth_optical_frame。用于 red_object/yolo 时须同时设置 expected_source 和实际 target_frame，不能将 target_state_base 直接接入旧光学偏角控制器。新的受保护启动入口跟随距离默认 1 m（`start_project.sh` 可用 `FOLLOW_TARGET_DISTANCE_M` 覆盖）；旧 FollowConfig 未改变，显式启动参数可调整，但近障保护不会因此绕过。

记录/回放：`bash scripts/record_follow.sh [目录]` 记录目标、扫描、TF、请求、保护状态、最终速度和里程计；`bash scripts/replay_follow.sh BAG目录` 限定域 174/本机和允许的话题，并重映射到 /replay/*，不会把历史命令送入实际 /cmd_vel。回放用于诊断，不能当成实时传感器解锁保护。

进程被 SIGKILL/系统崩溃时软件节点不能发最后一帧零，仍依赖底盘驱动已有命令超时与反馈超时停车；整机掉线保护必须由 STM32 固件/硬件兜底，当前代码测试不能替代实车验证。

审查补充：速度请求的 `base_frame` 由 safety_config 传给直接跟随器；可通过 launch 的 `target_frame` 显式覆盖目标坐标系。红色入口默认使用 camera_color_optical_frame（可用 TARGET_FRAME 修改）。雷达 `range_min` 形成的盲区若超出已确认的车体矩形，拒绝授权。手动 EXTERNAL 请求允许配置的正负线速度（默认 `-0.15..0.15 m/s`），FOLLOW 生产者仍可自行禁止倒车。SIGTERM 会先发送零速度再销毁 ROS 节点。
