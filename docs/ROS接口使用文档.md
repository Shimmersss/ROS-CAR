# ROSCAR ROS 2 接口使用文档

本文对应本机软件接口；硬件标定、制动距离、RGB-D 配准和实车运动必须另行验收。本轮不访问或部署小车。所有名称以下述 launch 的默认绝对路径为准，可用 `ros2 topic info -v` 核对实际发布者、类型与 QoS。

## 1. 环境与启动

使用 Ubuntu 22.04 / ROS 2 Humble，Python 与 rclpy 的 ABI 必须兼容。Mac 上使用项目 ARM64 Humble 容器验证；Mac Foxglove WebSocket 连接并不提供原生 ROS Python 接口。

```bash
source /opt/ros/humble/setup.bash
# 在项目根目录；安装现有包及新增 vision_msgs 的系统依赖
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
bash scripts/build_ros.sh
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=182
export ROS_LOCALHOST_ONLY=1
ros2 interface show roscar_interfaces/srv/SetControlMode
ros2 launch roscar_api api.launch.py
```

默认仅启动一个 motion_guard，`command_mode=IDLE`、`motion_enabled=false`；发布零速度和状态，不启动相机、雷达、底盘、视觉或跟随器，也不自动 arm。不要与已有 route A、导航或其他包含 guard 的组合入口叠加运行。先停止旧组合入口，或者继续使用旧入口提供的相同公共接口。出现重复速度发布者时 guard 拒绝授权或进入 FAULT。

统一入口可选参数：

| 参数 | 默认 | 作用 |
|---|---|---|
| `with_perception` / `route` | false / yolo | 启用 yolo、red 或 astra 感知；不启动相机/骨架 SDK |
| `model_path` / `device` / `yolo_python` | 空 / cpu / 空 | 本地权重、推理设备、ABI 兼容解释器 |
| `depth_registered` | false | 现场确认配准后才能设 true，不能只按 frame 名称认定配准 |
| `color_topic` / `depth_topic` / `camera_info_topic` | 见下例 | 同步 RGB-D 输入 |
| `camera_mount_config` | perception_bringup/config/camera_mount.yaml | YOLO 车体坐标派生接口；未标定不影响原光学接口 |
| `with_follower` / `target_distance_m` | false / 1.0 | 启动内部跟随请求生产者，不自动切 FOLLOW/arm |
| `safety_config` | motion_guard/config/safety.yaml | 实测尺寸、雷达安装、停车模型、来源和 frame；占位确认开关默认 false |
| `motion_enabled` | false | 允许调用 arm；仍须满足全部保护条件 |
| `with_radar` / `radar_config` | false / perception_bringup/config/radar.yaml | 显式启用已有 N10P 驱动与状态节点 |
| `with_chassis` / `serial_port` / `car_mode` | false / 空 / 空 | 显式启用底盘；后两项必须填写实测值 |

仅启动视觉软件的例子（相机已提供配准数据；权重路径替换为实际绝对路径）：

```bash
ros2 launch roscar_api api.launch.py with_perception:=true route:=yolo \
  model_path:=/absolute/path/yolo26s.pt depth_registered:=true \
  color_topic:=/camera/color/image_rect \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info
```

底盘和雷达为可选独立覆盖工作区，使用前分别执行 `bash scripts/build_chassis.sh`、`bash scripts/build_radar.sh` 并 source `ros2_ws/chassis_install/setup.bash`、`ros2_ws/radar_install/setup.bash`。不取消厂商目录的 COLCON_IGNORE。不提供自动电机转速、电源或距离运动服务。

局域网原生 DDS：两端 source 相同接口版本，设置相同 `ROS_DOMAIN_ID`，两端 `ROS_LOCALHOST_ONLY=0`，允许 DDS 发现/数据通信，保持系统时钟同步（例如系统 NTP/chrony），使用一致的时间源。带时间戳的请求在接收时必须仍在有效期内，时钟偏差会触发拒绝。Foxglove 的 `ws://小车地址:8765` 用于展示，与 DDS 的 ROS 域和 Python 接入是两条不同链路。

## 2. 底盘指令与模式

### 2.1 外部速度请求

**功能**：外部程序 → `/chassis/cmd_vel`，类型 `geometry_msgs/msg/TwistStamped`。它是请求入口，最终 `/cmd_vel` 仅由 motion_guard 发布。不要绕过保护直接写 `/cmd_vel`。

| 字段 | 单位 / 约定 |
|---|---|
| `header.stamp` | 当前 ROS 时间；零、未来、切换模式前或超时的请求不能驱动 |
| `header.frame_id` | 必须 `base_link`；EXTERNAL 同时要求 guard 的 base_frame 为 base_link |
| `twist.linear.x` | m/s，前进，默认允许 [0, 0.15] |
| `twist.angular.z` | rad/s，左转正，默认允许 [-0.5, 0.5] |
| 其他 linear/angular 分量 | 必须为零；拒绝横移、倒车及非有限值 |

默认 QoS 为 reliable / volatile / keep_last(1)。推荐 20 Hz 持续发布；默认请求有效期 0.2 s，同时检查消息时间与单调接收时间。超出限速直接拒绝，不在 API 层悄悄裁剪。合法零速度也必须定期发送。

### 2.2 模式服务与授权

`/control/set_mode` 类型 `roscar_interfaces/srv/SetControlMode`：

```text
string mode
---
bool success
string message
```

| mode | 语义 |
|---|---|
| `IDLE` | 不接收运动来源，不能 arm |
| `EXTERNAL` | 仅 `/chassis/cmd_vel`；不要求视觉目标或视觉发布者 |
| `FOLLOW` | 仅已有 `/control/cmd_vel_request`；还要求新鲜有效的真实锁定目标 |

区分大小写。非法字符串返回 `success=false`，保留当前模式。合法切换（包括重复设置当前模式）立即发零、清空旧请求、解除授权；之后必须收到时间戳晚于切换时刻的新模式请求，再显式 arm。非选中来源不填充请求缓存。旧入口默认 FOLLOW，新 API 入口强制从 IDLE 开始。`command_mode` ROS 参数只表示启动值，运行期模式以 `/control/state` 为准。

两种运动模式均要求尺寸、安装与停车模型确认，雷达新鲜有效、扫描时刻 TF 可用、包络通行、发布者数量正确。EXTERNAL 不会因没有视觉目标而拒绝，但绝不绕过雷达保护。

| 服务 | 类型 | 成功 / 失败含义 |
|---|---|---|
| `/control/arm` | std_srvs/srv/Trigger | 全部检查通过才 success=true；失败原因在 message；不改变来源模式 |
| `/control/stop` | std_srvs/srv/Trigger | 立即发零并解除授权；需要再次显式 arm 才能恢复 |
| `/control/disarm` | std_srvs/srv/Trigger | 与 stop 相同 |

缺请求/雷达/TF、障碍、目标失效（FOLLOW）、重复发布者等会使已授权状态进入 FAULT 并发零；数据恢复或节点重启不会自动恢复运动。服务用 ROS 默认可靠服务 QoS，无固定频率。

```bash
ros2 service call /control/set_mode roscar_interfaces/srv/SetControlMode "{mode: EXTERNAL}"
# 持续零请求的可运行示例；在另一个终端明确 arm 后才前进，默认最多 2 秒
ros2 run roscar_api chassis --speed 0.05 --turn 0.0 --duration 2.0
# 示例自己也调用 set_mode，因此须在示例报告等待授权之后执行：
ros2 service call /control/arm std_srvs/srv/Trigger '{}'
ros2 service call /control/stop std_srvs/srv/Trigger '{}'
ros2 service call /control/disarm std_srvs/srv/Trigger '{}'
```

上述启动默认未开启运动，所以 arm 会失败。真实使用需使用经确认的安全配置并显式 `motion_enabled:=true`。示例不替调用者 arm；等待授权最多 60 秒，开始运动后失去授权、模式改变或状态超时即退出，退出发送零请求并调用 stop，打印服务失败结果。源码见 `ros2_ws/src/roscar_api/roscar_api/chassis.py`；其 Python 发布核心为：

```python
from geometry_msgs.msg import TwistStamped
publisher = node.create_publisher(TwistStamped, '/chassis/cmd_vel', 1)
msg = TwistStamped()
msg.header.stamp = node.get_clock().now().to_msg()
msg.header.frame_id = 'base_link'
msg.twist.linear.x = 0.0  # 未授权前持续发送零
publisher.publish(msg)
```

### 2.3 控制状态和底盘反馈

`/control/state` 是 `std_msgs/msg/String` 中的 JSON，20 Hz，reliable / volatile / keep_last(10)：

| JSON 字段 | 说明 |
|---|---|
| `mode` | PERCEPTION_ONLY / STANDBY / ARMED / FAULT，授权状态 |
| `command_mode` | IDLE / EXTERNAL / FOLLOW，来源选择 |
| `ready` | 当前检查是否通过；不等于已授权 |
| `reason` | 当前检查原因，例如 idle、request_timeout、invalid_velocity_request |
| `last_fault` | 最近锁存故障；可保留历史值 |
| `motion_enabled` | 启动期运动许可开关 |

`ros2 topic echo /control/state` 可观察状态。最终 `/cmd_vel` 为 `geometry_msgs/msg/Twist`，20 Hz、reliable / volatile / keep_last(1)，仅供底盘驱动订阅。

底盘驱动启动并收到有效下位机回传后才有以下反馈，频率取决于实际串口帧，不承诺固定值：

| 话题 / 类型 | 字段与单位 | 坐标与注意事项 |
|---|---|---|
| `/odom` / nav_msgs/msg/Odometry | pose.position.x/y：m；orientation：四元数；twist：m/s、rad/s | 默认 odom → base_footprint（以实际 header 为准）；厂商 pose.position.z 存航向 rad，**不是高度** |
| `/imu/data_raw` / sensor_msgs/msg/Imu | angular_velocity：rad/s；linear_acceleration：m/s²；orientation：四元数 | 读取实际 frame_id，不能假设与相机一致 |
| `/PowerVoltage` / std_msgs/msg/Float32 | data：V | 无 header；不得据接收值推断当前仍在线 |

标准消息的完整字段可用 `ros2 interface show nav_msgs/msg/Odometry`、`ros2 interface show sensor_msgs/msg/Imu` 查看；pose/twist covariance 与 IMU covariance 是相应量的协方差，未校验的厂商估计不能视为实测精度。

反馈采用驱动 reliable/volatile/keep_last：odom 与 IMU 为 2，电压为 1（可用 `ros2 topic info -v` 查看）；读取方可用 sensor_data QoS。没有反馈可能是未启动、串口故障或失联，不能把“未收到”当零速度/正常电压。驱动另保留命令及反馈超时停车。

## 3. RGB-D 视觉接口

### 3.1 全部二维检测

**功能**：视觉 → `/perception/detections`，`vision_msgs/msg/Detection2DArray`，reliable / volatile / keep_last(10)。YOLO/red 每次新鲜、成功的同步 RGB-D 检测完成后发布一次；频率随相机和推理耗时变化，不补帧。没有目标发空 `detections`；缺流、配准未确认、内参错误、推理失败或结果过期不发“正常空列表”。调用者必须自行检查时间戳和接收超时。

| 字段 | 格式 / 单位 / 含义 |
|---|---|
| array.header、每个 detection.header | 原彩色帧时间戳与光学 frame_id，不是处理完成时刻 |
| detection.bbox.center.position.x/y | 原图像素中心 u/v；不是缩小后的可视化图像坐标 |
| detection.bbox.center.theta | 0；轴对齐框 |
| detection.bbox.size_x/y | 原图像素宽/高 |
| detection.results[0].hypothesis.class_id | `person` 或 `red_object` |
| detection.results[0].hypothesis.score | YOLO 检测置信度（阈值 0.1）；红色为 NaN，表示没有概率估计 |
| detection.id | YOLO `epoch:track_id`；未分配轨迹时空字符串；红色仅已确认关联目标使用 `red:N`，其他候选为空 |
| detection.results[0].pose | 不提供三维；position 为 NaN，其他默认字段无测量意义 |

YOLO 每帧仅一次模型推理，再把原始检测交给 ByteTrack；完整框列表不会因尚未建立轨迹而丢失。只检测人体；流重置增加 epoch，防止复用 ID 误接旧目标。列表不是逐人体三维结果，也不是运动授权。红色输出全部符合 HSV/面积门槛的连通块。Astra 骨架路线不新增此二维列表。

```bash
ros2 topic echo /perception/detections --once
ros2 run roscar_api detections
```

可运行 Python 示例见 `ros2_ws/src/roscar_api/roscar_api/observe.py`：

```python
from vision_msgs.msg import Detection2DArray
from rclpy.qos import qos_profile_sensor_data
node.create_subscription(Detection2DArray, '/perception/detections',
    lambda msg: print([(d.id, d.bbox.size_x) for d in msg.detections]),
    qos_profile_sensor_data)
```

### 3.2 锁定目标与三维位置

`/perception/target_state` 类型 `person_interfaces/msg/TargetState` 不改变结构。Astra、YOLO、red 提供实际路线的锁定目标；状态发布通常 20 Hz，并不意味着每秒 20 次新观测。QoS reliable / volatile / keep_last(10)。

| 字段 | 含义 |
|---|---|
| header | 状态发布时刻；frame_id 为 position 坐标系 |
| observation_stamp | 真实观测时刻；Astra SDK 没提供时为零，不能伪造为相机时间 |
| source / is_simulated | astra、yolo、red_object；模拟目标不得授权真实运动 |
| status | 0 NOT_READY、1 SEARCHING、2 TRACKING、3 LOST、4 STALE |
| detail / target_id | 解释信息 / 当前锁定身份 |
| position_valid | 仅 true 时可用三维位置；TRACKING 单独不保证深度有效 |
| position.x/y/z | 米；光学坐标右/下/前 |
| horizontal_distance_m / bearing_rad | 光学下 hypot(X,Z) 与 atan2(X,Z)，偏角右正，单位 m / rad |
| measurement_age_s / confidence | 秒 / 置信度；未知为 NaN |

保留配准、内参、同步、深度有效性和时效检查。有效二维框可以同时对应无效深度，但不能因此生成有效三维坐标或允许跟随运动。

YOLO 锁定/释放服务为 `/perception/lock_target`、`/perception/release_target`，均 `std_srvs/srv/Trigger`。lock 选择当前新鲜、有跟踪 ID、最靠近原图中央的候选；无候选/过期返回失败。release 清除选择。失去目标不会悄悄换人；服务本身不 arm。红色路线自动确认/关联红块，Astra 由叉腰条件锁定，不提供这两个 YOLO 服务。

```bash
ros2 run roscar_api target lock
ros2 run roscar_api target release
ros2 service call /perception/lock_target std_srvs/srv/Trigger '{}'
ros2 topic echo /perception/target_state --once
```

YOLO 的 `/perception/target_state_base` 保持同类型、20 Hz；只在相机外参已确认且观测时刻 tf2 变换有效时输出有效车体坐标。base_link 为前/左/上，距离 hypot(X,Y)，偏角 atan2(Y,X) 左正。未标定仅使这个派生接口无效，原光学接口继续工作。禁止将它直接接到假设光学坐标右正的旧跟随算法。

### 3.3 展示话题

| 路线 | 话题 / 类型 | 行为 |
|---|---|---|
| YOLO | detections_image / sensor_msgs/Image | bgr8 画框，默认最高 10 Hz、缩放 0.5；header 保留彩色帧 |
| red | color_image、detections_image、red_mask_image / sensor_msgs/Image | 原始 RGB、bgr8 框、mono8 红色掩码；现有显示逻辑保留 |
| Astra | body_mask_image / sensor_msgs/Image | SDK 人体掩码，现有实测 320×240 mono8 |
| 各路线 | target_marker / visualization_msgs/Marker | 有效位置 ADD，无效 DELETE；仅展示 |
| YOLO | target_marker_base / visualization_msgs/Marker | 车体坐标派生球；标定/TF 失败则删除 |

上述话题均在 `/perception/` 下，图像 reliable/volatile（队列 2；Astra 以实际接口为准），Marker reliable/volatile（队列 10），频率依路线/输入而定。RGB-D 是本次新增检测接口的前提；没有增加 RGB-only 模式或独立二维推理流水线。红色路线原有视频展示功能保持原状。

## 4. 雷达接口

默认 N10P/N10Plus 配置目标 10 Hz，串口 460800；串口名、实际型号与安装外参必须核对。构建/组合方法见 [N10P雷达接入.md](N10P雷达接入.md)。

| 话题 / 类型 | 字段、坐标、单位 | QoS / 频率 / 失败语义 |
|---|---|---|
| `/scan` / sensor_msgs/msg/LaserScan | header：扫描时间/laser；angle_min/max/increment：rad；ranges：m；range_min/max：有效范围；intensities：厂商强度 | 驱动 reliable/volatile/keep_last(10)；目标 10 Hz。越界/非有限回波不是有效障碍距离，更不能默认视为空旷 |
| `/radar/points` / sensor_msgs/msg/PointCloud2 | 与 scan 同 frame，fields 描述 x/y/z（m）及强度；按消息 offsets/point_step 解码 | 驱动 reliable/volatile/keep_last(10)；随扫描输出，不能仅凭点少判断通行 |
| `/radar/status` / std_msgs/msg/String | JSON，见下表 | reliable/volatile/keep_last(10)，5 Hz；健康状态不是避障或运动许可 |

laser 平面 x 方向为零角、正角朝 +y；安装后与车体的关系由实测 TF 决定，不能从产品正面猜测。订阅使用 `qos_profile_sensor_data` 可兼容可靠/尽力而为传感器源。

| status JSON 字段 | 含义 |
|---|---|
| status | WAITING / STALE / INVALID / OK / NO_RETURNS |
| frame_id | 期望雷达 frame |
| hz | 最近到达频率，超时归零 |
| valid_points / nearest_m | 有效点数 / 最近有效距离，缺失为 null |
| observation_age_s / detail | 观测年龄 / 状态解释 |

```bash
ros2 topic echo /radar/status --once
ros2 topic info /scan -v
ros2 run roscar_api radar
```

## 5. 验证范围与定位问题

本机测试使用隔离 ROS 域和合成 RGB-D/雷达。真实 C++ 底盘驱动通过伪串口核对控制及停车帧；真实 YOLO26s 权重在 Mac CPU 上推理静态示例图片。以上不等于实车运动、相机配准或雷达覆盖验收。

- 看不到话题：先检查进程、域号、DDS 网络和接口安装，不使用 Foxglove 连接状态推断 DDS 是否可用。
- arm 失败：先读 message 与 `/control/state`；确认新鲜选中来源、启动许可及所有保护前提，不反复自动 arm。
- 列表停止：检查彩色/深度/CameraInfo、配准确认、时戳、推理错误；不得把停止更新当“当前无人”。
- 有框无坐标：检查 position_valid、detail 和配准深度；框只表示二维检测。
- 光学位置有效但 base 无效：检查外参确认及观测时刻 TF，禁止用占位外参冒充实测。

回归入口：`scripts/test_container.sh`、`scripts/test_navigation_container.sh`、`scripts/test_radar_container.sh`、`scripts/test_yolo_model.py`。验证日志放在 `artifacts/`；最终本轮结果见 WORKLOG。
