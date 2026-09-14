# 方案 A 分段联调（2026-09-14）

本轮由主代理协调三个 Sol 子代理：相机入口核查、SDK 核查、人体列表适配实现。没有启动底盘，也没有使用厂商整车 launch。

## 实机结果

- 独立相机：ASTRA S（2bc5:0402）成功输出 640×480、16UC1 深度图；15 秒测试窗口内收到 345 帧深度和 344 条 CameraInfo（包含启动时间，不作为稳定帧率基准）。内参 fx=fy=570.3422、cx=319.5、cy=239.5。RGB、深度精度和标定未验证。
- SDK：main 能启动并发布话题，但明确报 `0x50000a19 Invalid Orbbec Body Tracking license`。帧编号/话题存在不代表骨架识别成功。用户尚不确定是否有授权，需要向厂商核对。
- SDK ARM64 动态库可解析；当前程序依赖厂商源码树的库路径，不能只复制可执行文件。后续建议从含 astra.toml 和 Plugins 的 SDK lib 目录运行。
- 新增 bodyreader_msg 消息包与 bodylist_adapter，五包已在 Jetson 编译通过。完成真人锁定验收后，正式 `route:=astra` 已切换到该适配器；默认 route 仍为尚未就绪的 yolo，不会隐式启动相机或底盘。

## 可单独复现的入口

先加载 ROS 和原厂工作区，使用隔离 ROS 域。两个相机入口分别运行，不可同时占用同一设备。

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
export ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=1
# 深度流，不启底盘
ros2 launch astra_camera astra.launch.xml product_id:=0x0402 enable_color:=false enable_ir:=false enable_point_cloud:=false enable_colored_point_cloud:=false depth_registration:=false
```

SDK 入口（当前仍被授权阻塞，先停止上面的相机节点）：

```bash
cd /home/wheeltec/wheeltec_ros2/src/wheeltec_bodyreader/bodyreader/lib
ros2 run bodyreader main --ros-args -p rgb_stream:=false -p body_stream:=true
```

正式 A route 入口（另一个终端加载相同 ROS 域）：

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/ROSCAR/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=1
ros2 launch perception_bringup perception.launch.py route:=astra with_foxglove:=false
```

## 适配行为与限制

直接读取 /bodylist，叉腰锁定 ID，未锁定 SEARCHING、ID 缺席 LOST、输入停止 STALE；不创建 cmd_vel 发布器。质心毫米转米，SDK Y 朝上由厂商姿态判断推断，默认取反以对应 optical Y 朝下，必须在获得授权后实测坐标方向。Bodylist 不含传感器时间戳和置信度，observation_stamp 保持零，measurement_age_s/confidence 为 NaN。当前没有 ReID、卡尔曼滤波或抗遮挡身份恢复；SDK ID 重用和手势误选仍需后续评估。

不要运行厂商 bodyfollow/bodyinteraction/final.launch.py：三者均包含底盘入口。bodydata_process 自身也会发布零速度，本轮未使用。

## 授权待办

向厂商提供错误码和相机型号，确认设备是否包含与该 SDK 兼容的合法人体骨架授权。当前源码传的是占位字符串。后续授权信息应通过权限受控的本机文件或运行时变量注入，禁止把密钥写入仓库或日志。该提示没有阻止本次真实骨架与锁定验收，但长期含义仍需厂商说明。

## 软件验证补充

5个纯逻辑单测、Jetson合成ROS适配测试（SEARCHING/TRACKING/LOST/STALE、单位、未知时间戳、无cmd_vel）通过。A/B/demo 回归通过。Foxglove 用户目录桥接环境、Wi-Fi 直连和客户端面板已验收。骨架错误摘录见 [错误日志](方案A骨架错误日志.txt)。

## 真人锁定与正式入口

15 秒复测中，人体 ID 41 有 404 帧，33 帧满足全部叉腰条件；约 1.93 秒进入 TRACKING。363 条状态全部位置有效，距离约 0.865–1.264 m、偏角约 -0.140–0.007 rad，目标球和检测体积框各更新 363 次。Foxglove 同步显示目标 ID、有效位置、掩码、曲线和 3D Marker。

基于上述结果，正式 `route:=astra` 已改为启动 bodylist_adapter。该 route 只消费已存在的 `/bodylist`；`scripts/run_astra_foxglove.sh` 负责组合启动厂商 bodyreader、正式 A route 和 Bridge，全程不启动底盘。

## 人体实测复测（2026-09-14 17:19）

重新单独运行 `bodyreader/main` 40 秒，收到 1004 条 `/bodylist`；其中 707 条检测到 1 人，稳定 ID 为 93，707 条有有效质心，706 条有非零关节坐标。质心 Z 大约为 0.99–1.18 m，单帧最多 19 个有效关节；叉腰判定出现 1 帧。测试期间仍打印 `Invalid Orbbec Body Tracking license`（本轮码 `0x50000719`），但它没有阻止真实人体骨架和质心输出。因此该错误应记录为厂商 SDK 的异常/授权提示，不能再视为当前骨架功能的硬阻塞；仍需让厂商解释其含义和后续影响。

本轮只启动骨架读取，不启动 `bodydata_process`、`follower` 或底盘。测试已正常停止。

## 用户在场骨架测试（2026-09-14 17:17）

用户确认已就位后，从SDK lib工作目录单独运行main，关闭RGB，隔离ROS域182。40秒测试窗口收到1005条Bodylist，positive_frames=0、max_count=0、IDs为空、有效质心和叉腰均0。启动仍报0x50000739 Invalid Orbbec Body Tracking license（与上一轮错误码不同，文字相同）。当前未识别到人体；不能单凭结果把原因确定为授权，仍需核对深度视野、SDK配置与厂商指定程序。退出阶段另有ROS publisher析构错误，应与识别失败分开看。测试已停止，底盘未启动。
