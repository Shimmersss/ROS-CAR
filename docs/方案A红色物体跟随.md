# 方案 A：红色物体跟随（分支 a）

## 状态与部署边界

本分支从 `909123d` 创建，包含远端合并的 `cb0b87a` 跟随控制及本地 B 实现。新 A 默认不运行人体骨架，使用 OpenCV 红色连通区域和配准深度；B 不变，原骨架可通过 `route:=astra` 或 `scripts/run_astra_foxglove.sh` 显式运行。

历史记录：合并提交记载 Jetson 上运行过厂商底盘驱动与低速跟随控制，且做过受限速度下发。随后本地记录记载控制进程已停止。**本轮仅本机实现；没有连接、比对、部署或重启小车，不能据此认定与在线文件和进程完全一致。** 彩色图与深度配准、实物车型及完整实车跟随仍须现场验收。

## 链路

```text
独立相机驱动：校正后的彩色图 + 配准深度 + 彩色 CameraInfo
  → red_object_tracker：红块检测 → 自动锁定/关联 → 掩码内测距
  → /perception/target_state
  → person_follower（仅显式开启底盘时启动；运动默认关闭）
  → /cmd_vel → wheeltec_robot_node → 11 字节速度帧 → STM32
                    ↑ 24 字节基础回传
                    └→ /odom、/imu/data_raw、/PowerVoltage
```

相机驱动独立提供输入。本轮不自动选择未经实测的 ASTRA S RGB 驱动，不启动 bodyreader，不把未知的 RGB/深度对应关系当作已配准。输入不完整时 NOT_READY；不根据红块面积猜测距离。

## 检测、锁定和深度

- HSV 使用 H=0–10 和 170–179，S≥100、V≥70；3×3 开运算及闭运算去噪，合格区域至少为图像面积的 0.1% 且不少于 12 像素。
- 首次选择最大连通区域，相同面积按边框位置排序。连续三帧关联成功才锁定。已锁定目标用边框重叠、中心位移和面积比关联，不随其他红块变大而切换。
- 关联门限：面积比 0.25–4；IoU≥0.1 或中心距离≤上一框对角线的一半。候选按 IoU 与归一化位移排序。颜色相同物体交叠或突然换位仍可能混淆；这不是物体身份识别或 ReID。
- 一帧丢失即 LOST、位置无效、停车；最后可见后满一秒重新搜索，三帧确认后使用新 ID。短时遮挡允许原 ID 恢复。
- 输入必须是校正后的彩色图和已对齐到相同光学坐标系、相同尺寸的深度。使用 CameraInfo.P；拒绝不支持的裁剪、binning、无效投影、零/未来/过期时间戳。默认同步容差 0.06 秒、最大年龄 0.5 秒。
- 16UC1 毫米转米，32FC1 按米解释。红色目标掩码腐蚀一圈后，只取内部 0.2–8 m 的有效深度；至少 12 个点且有效比例≥30%。使用中位数，深度四分位距超过 max(0.25 m, 20%距离) 则拒绝。无效位置 NaN、Marker DELETE。
- `/perception/target_state` 沿用 TargetState，`source=red_object`、真实节点 `is_simulated=false`。测试输入为合成图像的事实由测试记录说明，不代表实机相机验证。

## 运行入口

先构建主动工作区（不含底盘），然后显式构建可选底盘：

```bash
source /opt/ros/humble/setup.bash
cd /home/wheeltec/ROSCAR/ros2_ws
colcon build --base-paths src
source install/setup.bash
# 只有需要底盘时执行；先安装 manifests 中声明的依赖。
bash ../scripts/build_chassis.sh
source chassis_install/setup.bash
```

`build_chassis.sh` 在忽略的 `chassis_src` 中复制三个串口包并移除副本的 COLCON_IGNORE，编译到 `chassis_install`，不改变默认包发现边界。

默认 A（等待外部 RGB-D，没有底盘、没有 cmd_vel）：

```bash
ros2 launch perception_bringup route_a.launch.py
```

已完成相机配准核验后：

```bash
ros2 launch perception_bringup route_a.launch.py depth_registered:=true \
  color_topic:=/camera/color/image_rect \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info
```

底盘通过 `with_chassis:=true serial_port:=实际设备 car_mode:=核验后的车型` 单独开启，运动仍为关闭。确认有效目标与底盘后，使用 `motion_enabled:=true`，或对 `/person_follower` 设置 `enabled=true`。启动参数不允许在未开底盘时使能运动。不要直接复制厂商默认车型。

首次实车应先绕过跟随器做受限串口检查。`chassis_motion_smoke_test.sh` 只启动加固后的底盘驱动，默认只等待 STM32 电压回传；加 `--move` 后才创建唯一一个 `/cmd_vel` 发布者，以不超过 0.08 m/s 的速度前进不超过 1 秒，随后持续发送零速度并关闭串口。脚本要求车型键与 `robot_model.yaml` 精确匹配，检测到已有底盘节点或速度发布者会拒绝运行：

```bash
bash scripts/build_chassis.sh
bash scripts/chassis_motion_smoke_test.sh --car-mode 已核验车型 --check
# 架空车轮或清空前方空间并准备断电后：
bash scripts/chassis_motion_smoke_test.sh --car-mode 已核验车型 --move
```

管理入口 `scripts/route_a.sh` 和 `启动方案A.command` 默认调用 `run_red_foxglove.sh`。环境变量 `DEPTH_REGISTERED`、`COLOR_TOPIC`、`DEPTH_TOPIC`、`CAMERA_INFO_TOPIC`、`WITH_CHASSIS`、`MOTION_ENABLED`、`SERIAL_PORT`、`SERIAL_BAUD_RATE`、`CAR_MODE` 对应配置；默认配准、串口、运动均为 false。HSV 阈值、确认帧数、丢失超时和最小面积通过 ROS launch 参数调整。

仓库 systemd 单元已改为红色感知入口且固定串口/运动关闭；**本轮未安装该单元到小车，现有在线服务不会自动变更。** 切换部署时应先停旧服务和旧控制实例，核对摄像头占用，再安装并启动新入口。

## 控制与串口保护

- 默认 0.30 m 目标距离、0 m 距离死区、0.08 rad 偏角死区、0.15 m/s 前进上限、0.5 rad/s 转向上限；大偏角原地转向、不倒车，不包含避障。
- 控制器同时检查接收时间、消息发布时间、图像观测时间和测量年龄；持续重发旧目标也会停车。拒绝模拟数据及非预期 source。原骨架来源仅显式选择 Astra 时允许缺少传感器时间戳。
- 驱动使用 20 ms 串口读取超时；20 Hz 检查命令与回传。任一超过 0.5 秒，或 `/cmd_vel` 发布者数量不是一个时，清除缓存命令并发零。无底盘回传不能运动；控制端不代表 STM32 已具备断电/拔线保护。
- 驱动发送范围限幅，拒绝 NaN/Inf；11 字节基本帧标志为零。旧机械臂、回充、灯光和安全扩展命令不注册订阅，退出只发送基本停车帧。
- 每个实际串口路径使用进程文件锁，红色与骨架组合入口使用同一运行锁；避免重复管理实例。部署前仍须停止旧版或第三方串口进程，并确认 ROS 图中只有预期控制来源。
- 接收采用滑动帧头搜索、定长、帧尾和 BCC 校验，支持分片及坏帧重新同步；超时不读取未初始化字节，异常退出不恢复缓存速度。里程计积分步长最多 0.1 秒，避免长时间断流恢复后一次性积分。

## 验收

运行 `bash scripts/test_container.sh` 在本机 Linux ARM64 Humble 环境构建并执行红色检测、控制新鲜度、伪终端真实驱动与合成闭环测试，同时回归 A/B、demo 和语音逻辑。日志写入 `artifacts/humble-test.log`。

实机后续必须核验：彩色流可用、RGB-D 配准、真实串口与车型、真实回传及方向、停车行为、现场红色目标跟随。伪终端验证不替代这些项目，也不能证明 STM32 断线停车。

2026-09-16 首轮底盘冒烟中，实物照片的 OLED 显示 `Akm`，且可见转向舵机，故使用 `mini_akm`。串口收到约 11.337 V 的有效 24 字节回传；0.08 m/s、1 秒直行命令期间 `/odom` 的线速度从零上升至约 0.088 m/s，结束后回到零，驱动退出且串口释放。这证明指令已进入带编码器反馈的底盘链路；远程没有肉眼观察车身位移，不能替代用户现场确认或方向验收。

## 原始视频与检测框视频（2026-09-16）

Foxglove 导入 `foxglove/red-layout.json`（仓库根目录下）后，上方并排显示原始彩色视频 `/perception/color_image` 与画框视频 `/perception/detections_image`，下方保留掩码、状态和底盘信息。原始帧保持相机输入的像素、编码和 header；画框帧为 bgr8 并保留相同 header，黄色框标识红色候选，绿色框仅标识同帧被 RGB-D 跟踪接受且深度有效的目标。

视频仅依赖配置的 `color_topic`，无需深度或配准确认即可显示与检测；缺少深度时 TargetState 仍为 NOT_READY，不会因此允许运动。必须有真实相机发布彩色话题才能看到实时画面。本轮完成本机代码和布局，未修改小车或在线 Foxglove 配置。

## 性能统计

默认每秒发布 `/perception/performance` 与可选控制器的 `/control/performance`（person_interfaces/RuntimeMetrics）。重新导入 red-layout 后底部显示 FPS、处理耗时、观测/控制延迟。

- input_fps：节点实际收到的回调帧率，不等于相机硬件帧率，不能单独证明相机到 ROS 无丢帧。
- output_fps：成功输出画框图像的帧率；控制话题上则为速度指令输出频率。
- processing_ms / processing_p95_ms：彩色回调耗时，包含转换、红色检测、绘制和 publish 调用；不是纯 HSV 算法耗时，也不包含网络传输/客户端绘制。
- rgbd_ms / rgbd_p95_ms：同步 RGB-D 回调耗时，包含校验、检测、关联和测距；该路径与彩色视频检测目前各自计算，不能把单路耗时当作节点总 CPU 开销。
- observation_age_ms：采集时间戳到画框输出时的年龄。
- control_latency_ms：有效目标采集时间戳到本次速度指令 publish 完成的延迟；不是电机实际动作延迟。仅有效、使能的控制周期记录；Astra 缺采集时间戳时不记录。

每秒给出平均/P95，窗口没有样本时耗时为 NaN（不能当零延迟），无输入时 FPS=0。耗时用单调时钟计算；观测年龄依赖相机与 Jetson ROS 时间戳同一时基，负值或未知戳不统计。样本队列每类上限 4096，限制内存；无逐帧 ROS 指标发布或日志。

关闭统计做 A/B 开销对照：`PERFORMANCE_ENABLED=false bash scripts/start_project.sh`，或 launch 参数 `performance_enabled:=false`。重启生效。统计开销预计较小，但未在 Jetson 实测百分比；开启前后保持分辨率、场景、模块和 Foxglove 订阅一致，各测五分钟，对比 FPS、CPU 与延迟。关闭后不创建性能发布者/计时器。CPU/GPU/内存仍用 tegrastats 测量，当前性能话题不包含资源占用。
