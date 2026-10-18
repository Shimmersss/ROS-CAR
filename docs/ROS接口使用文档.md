# ROS-CAR 接口使用手册

更新日期：2026-10-16。本文按当前工作区的消息定义、服务实现和启动入口核对，覆盖感知、跌倒、主人认领、语音、控制、舌苔及拍照上传；从其他电脑远程调用见 [2.5 远程连接小车](#25-远程连接小车)。接口契约与现场验收范围分别说明；实际启用的接口以运行图为准。

## 1. 先选你需要的接口

| 你想做的事 | 应调用的接口 | 返回什么 |
|---|---|---|
| 拍面部、舌体、安全帽场景或电路板并上传 | `/capture/face`、`/capture/tongue`、`/capture/helmet`、`/capture/pcb` | 等任务结束，返回上传成功或失败及回执 |
| 拍照后在 5060 分析并取回标注图（显式 infer） | `/inspect/face`、`/inspect/tongue`、`/inspect/helmet`、`/inspect/pcb` | JSON 分析结果与 CompressedImage JPEG；面诊暂无模型 |
| 取消自己通过上述服务发起的拍照 | upload用`/capture/cancel`，infer用`/inspect/cancel` | 是否已请求取消；最终结果由原拍照调用返回 |
| 让小车读出一段文字 | `/voice/say` | 等合成与播放结束，返回完成或失败 |
| 订阅检测框、锁定目标的位置 | `/perception/detections`、`/perception/target_state` | 持续发布的 ROS 消息 |
| 读取跌倒告警和每人的骨架、姿态 | `/perception/fall_alarm`、`/perception/person_states` | 告警汇总 / 逐人状态；不是运动命令 |
| 认领主人、释放主人、查看丢失等待 | `/perception/claim_owner`、`/perception/release_target`、`/perception/owner_state` | 服务结果 / 生命周期状态 |
| 切换控制来源、明确授权、停车 | `/control/set_mode`、`/control/authorize`、`/control/stop` | 控制服务的执行结果 |
| 外部程序发送速度请求 | `/chassis/cmd_vel` | 请求话题；最终速度由唯一 `motion_guard` 决定 |
| （demo 分支）前进/后退 N 米、左转/右转 N 度 | `/demo/move_distance`、`/demo/turn_angle`、`/demo/move_relative` | 按 `/odom` 闭环执行，结束返回实测距离/角度；见第 13 节 |
| 拍照等任务期间保持停车 | `/control/task_hold` | 有期限的停车租约与续期 token；释放不恢复运动 |
| 读取里程计、IMU、电压或雷达 | `/odom`、`/imu/data_raw`、`/PowerVoltage`、`/scan` | 需要对应硬件驱动已启动 |

这里有两套通信方式：

- **ROS 接口**给本机或局域网内的 ROS 2 程序使用，本文说明它们的调用方法。
- **HTTP 接口**给云端服务使用。云端下发任务、小车上传 JPEG 的协议见[云端拍照任务接口协议](云端拍照任务接口协议.md)。服务器上提供的文档下载链接只是静态文件地址，不是拍照业务接口。

当前小车常驻启用的是 `/inspect/*`，运动关闭，语音已恢复；`/capture/*` 仅在明确切回upload时提供。模型与小车地址迁移见[云端推理部署与迁移](云端推理部署与迁移.md)，三类真实返回样例见[拍照推理接口返回值](拍照推理接口返回值.md)。

源码默认 `/capture/*` 接口只负责“播报、准备、拍照、上传”；显式 infer 使用独立 `/inspect/*`（见第 3 节增补），不会改变 `/capture/*` 的含义。`/capture/tongue` 不返回舌苔分类或诊断；语音口令“舌苔检测”走另一套 `tongue_mode` 流程，两者共享相机和播报资源并互斥。

## 2. 调用前先准备环境

### 2.1 构建与加载

在项目根目录执行。服务端和调用端都要有对应的 ROS 2 Humble 消息定义；Python 客户端使用与 `rclpy` 兼容的解释器。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
# 只构建本文的 API、拍照模块及其依赖；构建产物统一位于仓库根目录
bash scripts/build_ros.sh --packages-up-to roscar_api capture_task
source install/setup.bash
export ROS_DOMAIN_ID=182
export ROS_LOCALHOST_ONLY=1
```

`ROS_LOCALHOST_ONLY=1` 适合单机调试。电脑通过局域网调用小车时，两端使用相同域号和接口版本，并设为 `ROS_LOCALHOST_ONLY=0`；允许 DDS 通信，保持时钟同步。域号 182 是项目常用值，必须与正在运行的服务一致。

若运行入口设置了 `RMW_IMPLEMENTATION` 或 `FASTRTPS_DEFAULT_PROFILES_FILE`，调用终端也应加载该部署对应的通信配置；XML 路径必须在调用端真实存在，不能直接把小车文件路径用于另一台电脑。已有 Fast DDS 配置时，例如：

```bash
# 替换为本部署的实际配置；没有自定义 profile 时不必设置
export FASTRTPS_DEFAULT_PROFILES_FILE=/absolute/path/fastdds-profile.xml
```

Foxglove 的 WebSocket 连接只负责展示，不能替代 `rclpy` 或 `ros2 service call`。Windows 没有 ROS 环境时，可以通过 SSH 在已经安装 ROS 的小车终端调用。从另一台电脑远程调用小车接口的完整步骤见 [2.5 远程连接小车](#25-远程连接小车)。

### 2.2 服务不会因构建完成而自动启动

| 启动入口 | 会启动什么 | 前提与默认行为 |
|---|---|---|
| `ros2 run capture_task api_node` | 仅拍照 ROS 服务节点 | 复用已有 `/capture_task/run`；不新启执行器、相机或 TTS |
| `ros2 launch capture_task capture_task.launch.py with_ros_api:=true` | 拍照执行器、云端 bridge、ROS 服务节点 | 默认 `bench`；相机与唯一 TTS 需另行提供；可设 `with_bridge:=false` |
| `ros2 launch roscar_api speech_api.launch.py` | 仅 `/voice/say` 服务节点 | 默认复用已有 `/voice/speak` |
| `ros2 launch roscar_api api.launch.py` | 一个 `motion_guard` | 默认 IDLE、运动关闭，不自动启相机、底盘、语音、拍照或跟随器 |

按需要选择入口。已有整机入口包含的节点不能重复启动，尤其是 `motion_guard`、相机和 TTS。

**不能把“默认控制入口”和“bench 拍照入口”简单同时运行：**前者会发布 `/cmd_vel`，后者要求该话题的发布者为零。接有底盘或已有 guard 时，拍照必须选择 `robot` 配置，见下节。

B/C 统一构建与启动使用仓库根目录 install/，不再默认加载旧 ros2_ws/install/；需要另一套 overlay 时显式设置 PERCEPTION_INSTALL。厂商底盘、雷达和麦克风仍使用独立 overlay。

### 2.3 确认接口确实存在

```bash
ros2 run roscar_api interfaces --json --wait 3
ros2 service list -t
ros2 action list -t
ros2 interface show capture_task_interfaces/srv/RunCapture
ros2 interface show roscar_interfaces/srv/SayText
```

找不到服务时，先确认相应节点已启用、overlay 已加载、域号和 DDS 网络一致。不要先反复重发业务请求。

### 2.4 公共 API 的可选启动项


| 参数 | 默认 | 含义 |
|---|---|---|
| `with_perception`、`route` | false、yolo | B 用 yolo，C 用 yolo_pose；相机独立启动；astra/red 仅保留旧调用兼容 |
| `model_path`、`device`、`yolo_python` | 空、cpu、空 | 明确本地模型及兼容解释器；缺模型不静默回退 |
| `depth_registered` | false | 物理配准验收后显式确认 |
| `color_topic`、`depth_topic`、`camera_info_topic` | `/camera/color/image_rect`、`/camera/aligned_depth_to_color/image_raw`、`/camera/color/camera_info` | 同步 RGB-D 输入 |
| `pose_config`、`camera_mount_config`、`reid_config` | perception_bringup 对应 YAML | 骨架/跌倒、安装/地面、身份参数；确认开关不自动置真 |
| `reid_enabled`、`reid_lock_enabled`、`reid_reacquire_enabled`、`sot_enabled` | false | 身份观察、身份锁定、主人找回、单目标联动；仍遵守互斥和模型校验规则 |
| `reid_model_path`、`sot_model_path` | 空 | 本地模型和校验文件，显式启用时必须齐全 |
| `registration_calibration_file`、`registration_expected_serial` | 空 | C 可选进程内 Gemini 配准；发布同帧校正 RGB，可与 ReID 同开；不要再启动独立配准发布者 |
| `auto_lock_single`、`owner_lost_timeout_s` | false、10.0 | 单人连续三帧自动认领开关、主人丢失等待秒数 |
| `with_follower`、`target_distance_m` | false、1.0 | 跟随请求生产者；不自动切模式或授权 |
| `expected_source`、`target_frame`、`target_topic` | yolo、camera_color_optical_frame、`/perception/target_state` | 同时配置 guard 与跟随器的目标契约；来源/坐标必须与输出一致 |
| `motion_enabled`、`safety_config` | false、motion_guard/config/safety.yaml | 运动总开关及已验收安全参数；开总开关仍需逐次授权 |
| `with_radar`、`radar_required` | false、false | 启动雷达、让 guard 要求扫描/TF/包络保护；两个开关含义不同 |
| `with_chassis`、`serial_port`、`car_mode` | false、空、空 | 启动底盘必须明确串口与车型；本车使用 mini_akm |
| `with_voice`、`voice_config` | false、offline_voice/config/offline_voice.yaml | 启动共享录音、常开 KWS、固定规则、ASR、唯一 TTS，同时开放播报 API |
| `asr_enabled` | true | `with_voice=true` 时的识别启动初值；运行中用 `/voice/set_asr_enabled` 切换 |
| `enable_wake_driver`、`use_xfyun_asr` | true、false | 仅 with_voice 时生效；讯飞只替换识别，不引入 LLM 运动控制 |
| `with_speech`、`speech_with_tts`、`with_announcements` | false、false、true | 仅播报桥、额外启动 TTS、阶段播报；已有 TTS 必须保持 speech_with_tts=false；与 with_voice 重复所有者组合会拒绝启动 |
| `with_tongue`、`tongue_safety_profile` | false、robot | 模式管理器及舌苔；robot 要新鲜静止里程计，bench 必须运动关闭 |
| `tongue_config`、`modes_config`、`tongue_server_url`、`tongue_camera_topic`、`tongue_camera_profile` | 对应包 YAML、现有服务地址、`/camera/color/image_raw` | 检测契约、模式目录、云端服务和原始 RGB；profile 默认按 C/B 使用 Gemini/Astra，可显式覆盖；本 API 不重复启动 guard/router/TTS |
| `with_gimbal`、`gimbal_port`、`gimbal_mock`、`gimbal_mount_config` | false、空、false、gimbal_bridge/config/gimbal_mount.yaml | 云台串口或显式 PTY 模拟；未确认安装不发布云台 TF |

```bash
# 相机已独立运行时，启动 C 感知软件，保持运动关闭
ros2 launch roscar_api api.launch.py with_perception:=true route:=yolo_pose \
  model_path:=/绝对路径/yolo26s-pose-fp16.engine device:=0

# 已有 TTS 和 guard，仅增加文字播报及主人/跟随阶段播报
ros2 launch roscar_api speech_api.launch.py with_announcements:=true

# 没有其他整车入口时，显式启动语音和舌苔；其依赖、模型、录音设备须先就绪
ros2 launch roscar_api api.launch.py with_voice:=true with_tongue:=true
```

### 2.5 远程连接小车

本节给"在另一台电脑上调用小车 ROS 接口"的使用者。

| 项 | 值 |
|---|---|
| 小车地址 | `192.168.31.145`（当前局域网，地址变化以现场为准） |
| SSH 用户 | `wheeltec`，端口 22；密码向项目负责人单独索取，不写入仓库 |
| ROS 版本 | ROS 2 Humble（Ubuntu 22.04） |
| ROS 域号 | `ROS_DOMAIN_ID=182` |
| 本机限制 | 小车入口设置 `ROS_LOCALHOST_ONLY=0`，允许局域网发现 |
| DDS | Fast DDS（Humble 默认 RMW）；小车加载 `perception_bringup/config/fastdds_robot.xml`，启用共享内存 + UDPv4，不限制网卡或地址 |
| 运行目录 | `/home/wheeltec/ROSCAR-current`，由用户服务 `roscar-robot.service` 开机启动固定机位主链 |

接口能否远程访问，取决于小车上**实际启动了哪些节点**，不是代码里有就一定在运行。部分功能可能尚未部署到小车或默认关闭。连上后请先按下面的"第 0 步"查看实际运行的接口，再调用。

**2026-10-16 小车部署状态**（固定机位主链与独立infer适配；之后改启动参数会变化）：

| 类别 | 当前可用 |
|---|---|
| 控制服务 | `/control/set_mode`、`/control/authorize`、`/control/arm`、`/control/disarm`、`/control/stop`、`/control/estop`、`/control/reset_estop`、`/control/reset_fault`、`/control/task_hold` |
| 感知服务 | `/perception/claim_owner`、`/perception/lock_target`、`/perception/release_target` |
| 语音 | 服务 `/voice/say`、`/voice/set_asr_enabled`、`/voice/start_listening`、`/voice/cancel_listening`、`/voice/cancel_speech`；action `/voice/speak`；ASR 状态 `/voice/asr_enabled`、`/voice/asr_status` |
| 常用话题 | `/perception/target_state`、`/perception/detections`、`/perception/person_states`、`/perception/fall_alarm`、`/perception/owner_state`、`/control/state`、`/system/phase`、`/odom`、`/imu/data_raw`、`/PowerVoltage`、`/camera/color/image_raw`、`/perception/detections_preview/compressed` |
| 拍照推理 | `/inspect/face`、`/inspect/tongue`、`/inspect/helmet`、`/inspect/pcb`、`/inspect/cancel`及Action `/capture_task/run`；独立 `roscar-capture-inference.service` enabled/active，face无模型；旧upload适配已停用 |

当前主链运动总开关关闭（`motion_enabled=false`），状态IDLE/DISARMED。语音已解除静音并由用户确认正常。整车脚本仍固定 `WITH_CAPTURE_TASKS=false`，拍照由独立用户服务提供，不要再按3.1叠加执行器。远程调用控制类服务前请先读第5节。

#### 方式 A：SSH 登录小车，在小车终端调用（最简单，推荐先用）

不需要在自己电脑上装 ROS，Windows 也可以用。

```bash
ssh wheeltec@192.168.31.145
# 登录后在小车上：
source /opt/ros/humble/setup.bash
source ~/ROSCAR-current/install/setup.bash
# 调用当前/inspect与新版CaptureTask时必须再加载：
source ~/ROSCAR-capture-infer-2b27f36/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ROSCAR-current/ros2_ws/src/perception_bringup/config/fastdds_robot.xml
export ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=0
```

之后本文中的所有 `ros2 service call`、`ros2 topic echo` 示例都可以直接在这个终端运行。

#### 方式 B：自己的电脑装 ROS 2 Humble，通过局域网直接通信

适合要在自己电脑上运行 Python 客户端或长时间订阅数据的情况。

1. **网络**：电脑必须**直接**接入小车所在的 `192.168.31.x` 网段，双方能互相 ping 通。经过另一台路由器或 NAT（例如电脑在 `10.0.0.x` 子网、只能单向 ping 到小车）时 DDS 无法通信：小车回不到电脑。2026-10-14 实测从 NAT 后的电脑能 SSH 但发现不到任何节点，这种情况请用方式 A。DDS 依赖 UDP 组播，部分路由器或访客 Wi-Fi 会隔离客户端或拦截组播，也请改用方式 A 或网线直连。关闭电脑防火墙对 UDP 的拦截（Ubuntu 可先 `sudo ufw status` 检查）。
2. **系统**：Ubuntu 22.04 + ROS 2 Humble，与小车一致。Windows 下的 WSL2 默认 NAT 网络收不到组播；需要用 WSL 的 mirrored 网络模式，否则请用方式 A。
3. **消息定义**：电脑上必须有与小车**同一版本**的自定义接口包。克隆本仓库并切换到与小车部署一致的提交，然后只编译接口包：

   ```bash
   source /opt/ros/humble/setup.bash
   sudo apt install -y ros-humble-vision-msgs
   colcon build --base-paths ros2_ws/src --packages-select \
     roscar_interfaces capture_task_interfaces execution_mode_interfaces person_interfaces gimbal_interfaces
   source install/setup.bash
   ```

4. **环境变量**（每个终端都要设置）：

   ```bash
   export ROS_DOMAIN_ID=182
   export ROS_LOCALHOST_ONLY=0
   ```

   小车未设置 `RMW_IMPLEMENTATION`，即 Humble 默认的 Fast DDS；电脑端也保持默认即可，不需要复制小车的 XML。
   图像话题（如 `/camera/color/image_raw`）数据量大，经 Wi-Fi 订阅会占满带宽，远程查看画面优先订阅 `/perception/detections_preview/compressed`。

#### 第 0 步：确认能看到哪些接口

```bash
ros2 node list
ros2 service list -t
ros2 topic list -t
ros2 action list -t
```

- 方式 B 下 `ros2 node list` 为空：多半是网络/组播或域号问题，先排查网络，不要反复调用业务服务。
- 当前应查 `/inspect/*`；看不到 `/capture/*` 是infer配置的正常表现。其他接口缺失先核对运行服务与overlay，不要额外启动第二个拍照执行器。

#### 远程调用注意事项

- **同一时间只让一个人控制小车**。远程客户端也能发送 `/chassis/cmd_vel`、调用 `/control/authorize`，运动仍受 motion_guard 授权和运动开关约束，但多人同时操作容易互相干扰。随时可以调用 `/control/stop` 停车。
- 拍照服务会等到任务结束才返回：默认TTL60秒时门面预算约95秒，显式timeout_sec=180时约215秒；不要因暂未返回而重复调用，重复调用会得到 `busy`。
- 时间相关的新鲜度检查都在小车端完成，电脑与小车不需要精确对时；但长时间订阅时建议两端时间大致一致，便于看日志。
- 不要在命令行、脚本或日志中写入 SSH 密码和云端 token。


## 3. 拍照并上传：最常用的新增接口

### 3.1 启用方法

如果已有拍照执行器，只追加 ROS 服务节点：

```bash
ros2 run capture_task api_node
```

如果尚未启动拍照执行器，可选择组合入口。下面是**无底盘、无 `/cmd_vel` 发布者**环境的配置示例；相机原始 RGB 和 `/voice/speak` 须已存在，`CAPTURE_CLOUD_URL` 须先设为实际业务地址：

```bash
: "${CAPTURE_CLOUD_URL:?请先设置实际拍照云端地址}"
ros2 launch capture_task capture_task.launch.py \
  with_ros_api:=true with_bridge:=false safety_profile:=bench \
  cloud_url:="$CAPTURE_CLOUD_URL" camera_topic:=/camera/color/image_raw
```

`with_bridge:=false` 表示只接收 ROS 发起的任务，不轮询云端命令；图像依然上传云端。需要同时接收云端任务时保留默认 `with_bridge:=true`。

| 配置 | 使用场景 | 执行前要求 |
|---|---|---|
| `safety_profile:=bench` | 脱离底盘的本机联调 | `/cmd_vel` 发布者为零；不申请停车租约 |
| `safety_profile:=robot` | 接有底盘，或复用已有整机 guard | 申请并保持 TaskHold，检查新鲜控制状态和 `/odom` 静止回传 |

robot 下任务结束会释放自己的租约，但不会恢复运动授权或清除急停。ESTOP 下租约申请失败时返回 `failed / hold_failed`。

常用组合入口参数：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `with_ros_api` | false | 显式开启四个拍照服务和取消服务 |
| `with_bridge` | true | 是否同时启用云端任务轮询 |
| `cloud_url` | `CAPTURE_CLOUD_URL`，未设则 `http://127.0.0.1:8090` | 上传业务地址；本机默认地址需要 mock 服务在运行 |
| `robot_id` | `CAPTURE_ROBOT_ID`，未设则 `roscar-01` | 上传携带的机器人 ID |
| `token_file` | `CAPTURE_TOKEN_FILE`，未设则空 | 当前用户拥有、权限 0600 的私有凭据文件；也支持 `CAPTURE_CLOUD_TOKEN` 环境变量 |
| `camera_topic` | `/camera/color/image_raw` | 原始 RGB，不使用画框或文字叠加图 |
| `catalog` | 包内 `config/tasks.yaml` | 任务默认播报、准备时间和质检参数；门面与执行器应一致 |
| `save_images` | false | 默认不把图像保存在执行器本地 |

整机脚本的开关为 `WITH_CAPTURE_TASKS=true CAPTURE_ROS_API=true`，默认均为 false；后者不能脱离前者开启。接底盘还必须显式 `WITH_FOLLOWER=true` 以复用唯一 guard；该入口同时启动跟随器；拍照启用时强制 `FOLLOW_AUTO_ARM=false`、`FOLLOW_TARGET_AUTO_RECOVER=false`、`FOLLOW_EMERGENCY_KWS_REQUIRED=true`，仍须显式切换模式并绑定 epoch 授权，租约释放不自动续跟。整机配置见[实现与验收](云端拍照任务实现与验收.md)。

### 3.2 四个服务怎么选

四个服务都使用 `capture_task_interfaces/srv/RunCapture`，选择服务名就选择了任务类型，不必再传 `task_type`。

| 服务名 | 拍什么 | 默认准备时间 | 默认候选帧数 | 图像宽度质检门槛 |
|---|---|---:|---:|---:|
| `/capture/face` | 面部 | 3 秒 | 3 | 640 像素 |
| `/capture/tongue` | 舌体 | 2 秒 | 3 | 640 像素 |
| `/capture/helmet` | 安全帽场景 | 1 秒 | 1 | 640 像素 |
| `/capture/pcb` | 电路板 | 3 秒 | 5 | 640 像素 |

PCB 缺陷检测模型默认输入为 **640×480**，当前 Gemini 原始 RGB 正好匹配，最小宽度为 640；仍按原始尺寸上传，不为拍照切换相机分辨率或放大图像。

当前目录默认挑选最佳帧上传、JPEG 质量 95、任务时限 60 秒。宽度、亮度、清晰度等质量不足默认按 `warn` 上传并记录指标；目录改为 `quality_policy: fail` 时才因质检不足而失败。程序不放大图像冒充原始分辨率。

### 3.3 最简单的一次调用

```bash
ros2 service call /capture/pcb capture_task_interfaces/srv/RunCapture \
  "{task_id: '', announce: '', prepare_sec: -1.0, timeout_sec: 0.0}"
```

流程是：检查资源与互斥条件 → robot 下停车并保持租约 → 等准备播报完成 → 等准备时间 → 采集此后到达的新鲜 RGB → 选帧、质检、JPEG 上传 → 播报结果、清理资源 → 返回服务结果。

调用会等待任务结束。不要把“终端暂时没有返回”误认为请求没有送达，再发一次会得到 `busy`。

请求字段：

| 字段 / 类型 | 如何填写 |
|---|---|
| `task_id` / string | 空串自动生成 `ros-<32位十六进制>`；非空须匹配 `[A-Za-z0-9_-]{1,64}`，建议每次使用新的 `ros-` 前缀 ID |
| `announce` / string | 空串使用任务默认播报；自定义最多 300 字符，不得包含停车、急停等禁用词，详见 `catalog.py` 的 `RESERVED_STOP` |
| `prepare_sec` / float64 | **负数使用默认值**；0 表示不额外等待，其他值为 0–30 秒；必须为有限数 |
| `timeout_sec` / float64 | 0 使用默认 60 秒；其他值为大于 0、最多 600 秒，约束执行器任务 TTL |

尤其注意：ROS 字段省略时 `prepare_sec` 默认是 0，**不会自动使用目录默认准备时间**。希望采用默认值时，像示例一样明确传 `-1.0`。

### 3.4 返回值怎么读

成功返回的结构如下，ID 和回执仅为示例：

```yaml
success: true
status: uploaded
detail: ''
task_id: ros-example-001
capture_id: cap-example-001
upload_json: '[{"status":"received","task_id":"ros-example-001","capture_id":"cap-example-001","upload_id":"u-example-001","sha256":"<图像SHA256>"}]'
```

| 返回字段 | 含义 |
|---|---|
| `success` | 只有 `status=uploaded` 才为 true |
| `status` | 任务最终状态，见下表 |
| `detail` | 更具体的原因码；定位错误时同时看它 |
| `task_id` | 本次实际 ID，包括自动生成的 ID |
| `capture_id` | 执行器分配的采集 ID；尚未采集时可能为空 |
| `upload_json` | 已校验上传回执的 JSON **数组字符串**，不是图片、下载 URL 或分类结果；失败时可能含已经完成的部分上传回执 |

| status | 含义 / 下一步 |
|---|---|
| `uploaded` | 收到合法上传回执；不代表云端推理、诊断或缺陷检测完成 |
| `invalid` | 参数不合法，未发送执行器 goal；修正输入 |
| `busy` | 有任务占用；本接口不排队 |
| `unavailable` | 约 2 秒内找不到拍照执行器；检查 `/capture_task/run` |
| `rejected` | 执行器拒绝，例如 `voice_mode_active`；按 detail 处理 |
| `failed` | 执行中失败，例如相机、TTS、停车租约或 HTTP 上传错误 |
| `cancelled` | 取消结果；结合 detail 区分执行器结束结果与门面仅发出取消请求的情况 |
| `timeout` | 门面整体等待超时，已请求取消自身 goal；不代表已收到最终取消确认 |

两层时限不同：`timeout_sec` 控制执行器 TTL，门面总等待上限是 TTL + `speech_timeout_sec` + 5 秒，当前默认约 95 秒（从服务接收开始计）。执行器 TTL 先到通常返回 `failed / task_timeout` 或 `deadline_exceeded`，不会强行改成 `timeout`。

**正式云端的接入前提：**服务器须允许未经自己下发的 ROS 任务 ID 上传。ROS 上传携带 `X-Task-Source: ros_api`，不会经过 bridge 产生 command/ack/events。如果云端只认自己下发的 ID，可能返回 `failed / upload_http_404`；本机 mock 联调通过不代表正式云端已支持。

### 3.5 怎么取消

需要取消时，应在发起任务前自己生成唯一 ID。自动 ID 要到服务返回才知道，不适合另一个终端提前取消。

终端 A：

```bash
# 每次换一个新的 ID，两个终端使用同一 ID
ros2 service call /capture/face capture_task_interfaces/srv/RunCapture \
  "{task_id: 'ros-manual-20261013-001', announce: '', prepare_sec: 3.0, timeout_sec: 60.0}"
```

终端 B：

```bash
ros2 service call /capture/cancel capture_task_interfaces/srv/CancelCapture \
  "{task_id: 'ros-manual-20261013-001'}"
```

| 取消返回 detail | 含义 |
|---|---|
| `cancel_requested`，success=true | 已向本门面自己的匹配 goal 请求取消；再查看终端 A 的最终结果 |
| `not_owner`，success=false | ID 不匹配，或当前执行器任务来自云端/直接 Action；本服务不能取消它 |
| `not_active`，success=false | 门面没有在途任务，且没有新鲜的执行器 active 状态 |

取消不会撤回云端已经接收的图像。关闭调用端终端或客户端自行等待超时，不会自动取消服务端已经接收的任务。

### 3.6 Python 示例

```python
import json
import uuid
import rclpy
from rclpy.node import Node
from capture_task_interfaces.srv import RunCapture

rclpy.init()
node = Node('my_capture_client')
client = node.create_client(RunCapture, '/capture/pcb')
try:
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('拍照服务未启动或 ROS 域不一致')
    request = RunCapture.Request()
    request.task_id = 'ros-' + uuid.uuid4().hex
    request.announce = ''
    request.prepare_sec = -1.0
    request.timeout_sec = 60.0
    print('本次任务 ID：', request.task_id)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if result is None:
        raise RuntimeError('未收到服务响应')
    print(result.success, result.status, result.detail)
    if result.upload_json:
        print(json.loads(result.upload_json))
finally:
    node.destroy_node()
    rclpy.shutdown()
```

### 3.7 需要进度时用 Action

```bash
ros2 action send_goal /capture_task/run capture_task_interfaces/action/CaptureTask \
  "{task_id: 'ros-action-20261013-001', task_type: 'pcb', source: 'ros_api', params_json: '{}', ttl_sec: 60.0}" --feedback
ros2 topic echo /capture_task/state
ros2 topic echo /capture_task/result
```

Action 请求为 `task_id / task_type / source / params_json / ttl_sec`；反馈为 `phase / detail`；结果为 `status / detail / capture_id / upload_json`。`source` 只接受 `cloud / ros_api / local_test`。直接 Action 拒绝时没有 action result，原因另发到 `/capture_task/result`。

直接 Action 的发起方须保存 goal handle，以它取消自己的任务；`/capture/cancel` 不拥有该 goal。相机、TTS 和 robot 租约仍由同一个执行器管理，换用 Action 不会绕过互斥条件。

### 3.8 拍照推理闭环：/inspect/*（当前车端已启用）

当前车端常驻服务已经启用本节接口，不要再次运行下方启动示例叠加节点。ROS返回契约和三类实车记录逐字段见[拍照推理接口返回值](拍照推理接口返回值.md)。

这是新增接口版本，服务端与调用端都须重新编译 `capture_task_interfaces` 并加载同一 overlay。默认 upload 不启动这些服务。infer 时只注册 `/inspect/*`，不注册 `/capture/*`。两种模式的执行器都沿用原相机、TTS、停车租约与互斥条件。

```bash
# 相机和唯一 TTS 已存在；有底盘时选择 robot 并复用唯一 guard
ros2 launch capture_task capture_task.launch.py \
  pipeline_mode:=infer inference_url:=http://127.0.0.1:18200 \
  inference_token_file:=/absolute/private/gateway.token \
  with_ros_api:=true with_bridge:=false safety_profile:=bench
ros2 service call /inspect/pcb capture_task_interfaces/srv/RunInspection \
  "{task_id: 'pcb-001', announce: '', prepare_sec: -1.0, timeout_sec: 180.0}"
```

当前车端18200是到WSL网关8200的loopback SSH转发端口；其他部署可更改。地址、token和上传端口的更新步骤见[部署迁移说明](云端推理部署与迁移.md)。bench 要求 `/cmd_vel` 零发布者；不能与已有 guard 简单并开。

`RunInspection` 请求和 `RunCapture` 一致：空 task_id 自动生成；空 announce 用目录文本；prepare_sec < 0 用目录值；timeout_sec=0 用 60 秒默认，最多 600 秒。舌诊建议调用方显式给 180 秒，单次模型请求还受目录 inference.timeout_sec 的 120 秒限制。

| 响应字段 | 含义 |
|---|---|
| success | 只有 status=analyzed 为 true |
| status | analyzed / retake / failed / cancelled / rejected / busy / unavailable / invalid / timeout |
| detail | 固定原因码，例如 inference_model_unavailable、inference_busy、task_timeout |
| task_id / capture_id | 本次任务与选中帧 ID |
| result_json | 网关响应的 JSON 字符串，去掉 annotated_image.data_b64；含模型/结构化结果/摘要/耗时/图片哈希 |
| annotated_image | sensor_msgs/CompressedImage；analyzed时format=jpeg，data为标注JPEG；失败或重拍为空；header当前默认stamp=0、frame_id空 |

网关 ok/uncertain 均返回 analyzed；retake 表示需要重拍。PCB 保留原始 640×480 画面，模型 imgsz=640；标注尺寸不变，只有大图最长边超过 1280 才缩小。面诊无模型会失败，不能当成识别人脸已通过。

推荐用脚本保存返回值，避免在终端打印大段 JPEG 数组：

```bash
python3 scripts/inspect_client.py pcb --timeout 180 --output artifacts/inspection
ros2 service call /inspect/cancel capture_task_interfaces/srv/CancelCapture "{task_id: 'pcb-001'}"
```

取消只影响本门面拥有的匹配任务，不会取消云端或直接 Action 的任务。客户端调用超时不等于服务器任务已取消，应调用上述取消服务或使用脚本的超时清理。

`CaptureTask.action` 的原 Result 字段不变，末尾追加 `result_json` 与 `sensor_msgs/CompressedImage annotated_image`；upload 模式两者为空。`/capture_task/state` 增加 pipeline_mode。机器人总入口通过 `CAPTURE_PIPELINE_MODE`、`CAPTURE_INFERENCE_URL`、`CAPTURE_INFERENCE_TOKEN_FILE` 传入。infer 缺少 URL 会启动失败。

结果播报只用本地模板：PCB 疑似缺陷数量、安全帽未佩戴数量、舌象特征数量；不播放模型自由文本，不作医疗诊断。原有语音“舌苔检测”仍走独立 tongue_mode，不被本接口替换。部署证据与边界见 [闭环验收记录](拍照推理闭环实现与验收.md)。

## 4. 文字播报

已有唯一 TTS 和 `/voice/speak` 时，启动服务节点并调用：

```bash
ros2 launch roscar_api speech_api.launch.py
# 另一个终端调用
ros2 service call /voice/say roscar_interfaces/srv/SayText \
  "{text: '你好，现在开始拍照。'}"
```

请求只有 `text`，非空、最多 300 字符；服务返回 `success / status / utterance_id / message`。`success=true, status=completed` 表示软件已完成合成与 ALSA 播放，现场是否可听仍需确认。它无需唤醒词，不经过意图或运动解析。

默认播放等待上限 120 秒；忙时不排队；超时仅取消本次 goal。需要进度、直接取消、Python 示例或独立启动 TTS 时，见[文字播报 ROS 接口](文字播报ROS接口.md)。

语音系统运行时常开 KWS 仍会监听扬声器声音。播报内容包含停车/急停词可能触发保护，因此拍照准备播报禁用这些词；普通文字播报接口本身没有同样的禁用词过滤。

## 5. 底盘控制：模式、授权和速度是三件事

### 5.1 默认不会运动

```bash
ros2 launch roscar_api api.launch.py
ros2 topic echo /control/state
```

默认入口只启唯一 guard：`command_mode=IDLE`、运动关闭、不自动授权。相机不由公共 API 自动启动；底盘、感知、跟随器和雷达分别由 `with_perception / with_chassis / with_follower / with_radar` 显式开启，上述开关全部默认 false；语音、舌苔、独立播报和云台也可通过 with_voice / with_tongue / with_speech / with_gimbal 显式开启，详见 2.4。

`with_chassis:=true` 还须显式填写已核对的 `serial_port` 和 `car_mode`；本项目实物按 `mini_akm` 配置。厂商底盘/雷达工作区分别由 `scripts/build_chassis.sh`、`scripts/build_radar.sh` 构建，不取消 `COLCON_IGNORE`。

远端 C 跟随整机入口支持自动授权与目标恢复后自动续跟：`FOLLOW_AUTO_ARM`、`FOLLOW_TARGET_AUTO_RECOVER` 默认随 `MOTION_ENABLED`；STOP 仍回 IDLE，急停及其他故障保持保护。上述选项仅影响显式选择的跟随入口，不改变公共 API 默认行为。启用本次云端拍照组合时，两项强制关闭且紧急 KWS 心跳必须存在，以保持拍照和当前语音流程的逐次授权约定。

### 5.2 模式与服务

| 模式 | 允许的请求来源 |
|---|---|
| `IDLE` | 无运动来源，不能授权 |
| `EXTERNAL` | `/chassis/cmd_vel`；不要求视觉目标 |
| `FOLLOW` | `/control/cmd_vel_request`；另需新鲜有效、真实锁定目标 |

`/control/set_mode` 使用 `roscar_interfaces/srv/SetControlMode`：请求 `mode`；响应 `success / message / authority_epoch`。每次成功切换，包括重复设置同模式，都会停止并使旧授权失效。

| 服务 | 类型 | 行为 |
|---|---|---|
| `/control/authorize` | `roscar_interfaces/srv/AuthorizeMotion` | 请求当前 `authority_epoch` 和 `mode`；检查通过后授权 |
| `/control/stop`、`/control/disarm` | `std_srvs/srv/Trigger` | 发零、回 IDLE、撤销授权；不清 ESTOP/FAULT |
| `/control/estop` | 同上 | 锁存急停并发零，锁存跨重启保存 |
| `/control/reset_estop` | 同上 | 健康条件满足才解除急停；只回未授权状态 |
| `/control/reset_fault` | 同上 | 故障条件恢复后复位；只回未授权状态 |
| `/control/arm` | 同上 | 旧兼容服务，默认拒绝并提示改用 `/control/authorize` |
| `/control/task_hold` | `execution_mode_interfaces/srv/TaskHold` | 申请/续期/释放停车租约；见下文 |

正确顺序是：**选择模式并取得 epoch → 持续发送该模式的新鲜零请求 → 明确绑定本次 epoch 授权 → 确认状态已授权 → 才发送非零请求。**不能先缓存非零速度再授权。STOP、故障、模式切换和重启都会使旧 epoch 失效。

```bash
ros2 service call /control/set_mode roscar_interfaces/srv/SetControlMode "{mode: EXTERNAL}"
# 先由自己的客户端持续发送新鲜零请求；再把下面占位内容替换为上次成功响应的整数
ros2 service call /control/authorize roscar_interfaces/srv/AuthorizeMotion \
  "{authority_epoch: <刚返回的整数>, mode: EXTERNAL}"
# 停车调用可直接执行
ros2 service call /control/stop std_srvs/srv/Trigger '{}'
```

默认 `motion_enabled=false`，上述授权会失败。真正授权还要求已验收的几何/安装/停车模型、常开 KWS 健康心跳、正确且唯一的发布者及新鲜输入，FOLLOW 还要求有效目标；不能仅修改确认开关代替实测。TaskHold 活跃期间禁止授权。

`radar_required` 默认 false，此时不提供雷达避障保护；显式 true 时，guard 还检查新鲜扫描、扫描时刻 TF 和安全包络。它是配置项，不是“永久关闭雷达检查”。

**停车租约 `/control/task_hold`** 用于外部静止任务。请求字段为 `operation / owner / session_id / token / lease_sec`，响应为 `success / token / detail`。

| operation | 必填内容 | 执行结果 |
|---|---|---|
| `ACQUIRE` | 非空 owner、session_id；lease_sec 为 0.5–10 秒 | 无其他租约、无 ESTOP/FAULT 且 KWS 健康时取得 token，回 IDLE 并撤销运动授权 |
| `RENEW` | 本次返回的 token、有效 lease_sec | 从当前时刻延长租约；过期 token 拒绝 |
| `RELEASE` | 本次返回的 token | 释放本次租约，仍保持 IDLE/未授权 |

owner/session_id 各最多 80 字符。任务须在租约到期前续期；释放或到期不会续跟、不会清除故障或急停。TaskHold 不替代机器人实际静止检查，需同时检查新鲜 `/odom`。

```bash
ros2 service call /control/task_hold execution_mode_interfaces/srv/TaskHold \
  '{operation: ACQUIRE, owner: "external_capture", session_id: "session-001", token: "", lease_sec: 3.0}'
# 把 TOKEN_FROM_ACQUIRE 替换为 success=true 响应中的 token
ros2 service call /control/task_hold execution_mode_interfaces/srv/TaskHold \
  '{operation: RENEW, token: "TOKEN_FROM_ACQUIRE", lease_sec: 3.0}'
ros2 service call /control/task_hold execution_mode_interfaces/srv/TaskHold \
  '{operation: RELEASE, token: "TOKEN_FROM_ACQUIRE"}'
```

`/control/emergency_stop` 是 `std_msgs/msg/Bool` 急停输入，`data=true` 锁存急停，`false` 不能复位。外部应用通常调用有回执的 `/control/estop`，然后读 `/control/state` 确认结果。

### 5.3 外部速度格式

`/chassis/cmd_vel` 类型为 `geometry_msgs/msg/TwistStamped`，只是请求入口。最终 `/cmd_vel` 是 `geometry_msgs/msg/Twist`，只能由 guard 发布。

| 字段 | 约定 |
|---|---|
| `header.stamp` | 当前 ROS 时间，必须晚于模式切换且在请求时限内 |
| `header.frame_id` | `base_link`；EXTERNAL 要求 guard 的 base_frame 也为 base_link |
| `twist.linear.x` | m/s，正向前、负向后；范围由安全配置的 max_linear_mps/max_reverse_mps 决定 |
| `twist.angular.z` | rad/s，左转为正；范围由 max_angular_rps 决定 |
| 其余速度分量 | 全部为零；拒绝横移、NaN、Infinity 和超限请求 |

当前公共 `safety.yaml` 的占位限速为前后各 0.2 m/s、角速度 0.5 rad/s，**实际以启动加载的安全配置为准**。底盘为阿克曼车型，车辆能力和运动验收仍需独立确认，不能将角速度接口理解成已支持原地转向。

请求默认有效期 0.2 秒，建议 20 Hz 连续发布，零请求也要更新时戳。最小发布片段：

```python
from geometry_msgs.msg import TwistStamped
publisher = node.create_publisher(TwistStamped, '/chassis/cmd_vel', 1)
msg = TwistStamped()
msg.header.stamp = node.get_clock().now().to_msg()
msg.header.frame_id = 'base_link'
msg.twist.linear.x = 0.0
msg.twist.angular.z = 0.0
publisher.publish(msg)  # 放在周期回调中，每次生成新时戳
```

不要让语音路由器和另一个外部速度程序同时发布 `/chassis/cmd_vel`。guard 要求选中请求来源恰好一个发布者；超时或发布者异常会停车。

### 5.4 怎么读控制状态

`/control/state` 为 `std_msgs/msg/String` 中的 JSON，约 20 Hz：

| 字段 | 含义 |
|---|---|
| `command_mode` | IDLE / EXTERNAL / FOLLOW，当前请求来源 |
| `authority_state` | DISARMED / ARMED_MANUAL / ARMED_FOLLOW / ESTOP / FAULT |
| `authority_epoch` | 当前授权代次；不应复用历史值 |
| `mode` | 兼容字段 PERCEPTION_ONLY / STANDBY / ARMED / ESTOP / FAULT；精确授权状态看 authority_state |
| `ready / reason` | 当前输入检查是否通过，以及原因；ready=true 不等于已授权 |
| `motion_enabled` | 启动运动许可开关 |
| `emergency_healthy` | 紧急监听是否健康且新鲜 |
| `task_hold` | 拍照等任务是否持有停车租约及其 owner/session_id |
| `last_fault` | 最近故障原因，可能保留历史值 |

### 5.5 语音与这些接口的关系

源码语音入口使用共享录音、Qwen3-ASR、Silero VAD、固定语义规则、常开 KWS 和唯一 Matcha 中文女声（Vocos）播报器。2026-10-14 此次模型替换尚未部署，小车此前运行的仍是 Melo；模型准备见 [文字播报接口](文字播报ROS接口.md)。固定规则经 `MotionRequest` 和车辆能力适配后提交请求；`motion_guard` 是最终授权和速度出口。方向口令不会自动授权，不允许 LLM 生成底盘运动控制。

`/voice/command` 与 `/voice/command_result` 是结构化控制/结果接口，不是文字播报服务，也不是任意自然语言的聊天接口。日常语音操作见[语音控制说明书](语音控制使用说明书.md)，开发边界见[语音优化无车实施与验收](语音优化无车实施与验收.md)。

## 6. 人体感知与位置订阅

### 6.1 启动哪个路线

项目默认为 B（`route:=yolo`）；当前 C 主线为 `route:=yolo_pose`（Gemini Pro + YOLO26s-pose）。历史 Astra 骨架与红色目标路线已退役，不作为新接入建议。

感知软件入口 `perception_bringup/perception.launch.py` 不启动相机。相机须另行提供同步 RGB、已配准深度和 CameraInfo；Gemini 的序列号、标定、安装与地面确认见[方案 C 实现与验收](方案C实现与验收.md)。例如已有正确输入时，单独启动 B 软件：

```bash
ros2 launch perception_bringup perception.launch.py route:=yolo \
  model_path:=/absolute/path/yolo26s.pt depth_registered:=true
```

示例权重路径需替换；`depth_registered:=true` 只可在真实配准确认后使用。C 使用本地 pose 权重与 pose 配置，不会静默回退 detect。

### 6.2 常用接口

| 接口 | 类型 | 用途 |
|---|---|---|
| `/perception/detections` | `vision_msgs/msg/Detection2DArray` | 当帧全部人体二维框，包括尚未形成轨迹的框 |
| `/perception/target_state` | `person_interfaces/msg/TargetState` | 当前锁定目标，原光学坐标代表点 |
| `/perception/target_state_base` | 同上 | 通过已确认安装外参与观测时刻 TF 派生的车体坐标 |
| `/perception/lock_target`、`/perception/release_target` | `std_srvs/srv/Trigger` | 锁定/释放目标，不授权运动 |
| `/perception/person_states` | `person_interfaces/msg/PersonStateArray` | C 的骨架、姿态与跌倒状态 |
| `/perception/person_states_continued` | 同上 | 显式启用高度延续时的未检出人员；不混入真实检测列表，不用于认领 |
| `/perception/fall_alarm` | `person_interfaces/msg/FallAlarm` | 当前 RGB 帧的最高跌倒阶段，汇总检出与延续人员；见 6.3 |
| `/perception/person_identities` | `person_interfaces/msg/PersonIdentityArray` | 启用 ReID 时的外观身份假设；检查 enabled、valid、verified、visible |
| `/perception/owner_focus` | `person_interfaces/msg/PersonIdentity` | 启用主人 ReID 找回时的主人身份与当前轨迹提示 |
| `/perception/floor_plane` | `person_interfaces/msg/FloorPlane` | 显式启用地面拟合时的逐帧平面证据；不会自动确认地面/重力 |
| `/perception/performance` | `person_interfaces/msg/RuntimeMetrics` | C 启用性能统计时的实际推理频率、耗时与观测年龄 |
| `/perception/person_ground_states` | `person_interfaces/msg/PersonGroundArray` | C 的单目地面定位结果 |
| `/perception/person_positions` | 同上 | C 的统一脚下位置，参考点为双踝中点地面投影 |
| `/perception/target_state_ground`、`/perception/target_state_unified` | `person_interfaces/msg/TargetState` | C 的独立锁定目标地面/统一位置输出 |

未开启对应路线或功能时，不应期待每个话题都有有效数据。C 的地面定位、统一定位、三维姿态、ReID 与动态地面有各自确认条件，不能将“节点在运行”当作定位有效。跌倒状态目前不自动触发语音或运动。

二维框的中心、宽高为**原图像素**；class_id 为 person；YOLO ID 为 `epoch:track_id`，未跟踪框 ID 为空。没有目标时发布空列表；输入断流或推理失败不是“无人”，接收端必须检查时间戳与超时。列表只提供二维框，不在 `results.pose` 中提供有效三维测量。

`TargetState` 最重要的字段：

| 字段 | 含义 |
|---|---|
| `status` | 0 NOT_READY、1 SEARCHING、2 TRACKING、3 LOST、4 STALE |
| `position_valid` | true 才可使用位置；TRACKING 单独不足以证明深度有效 |
| `header` | 状态时间戳及位置坐标系；20 Hz 状态发布不代表 20 Hz 新测量 |
| `observation_stamp / measurement_age_s` | 真实观测时刻 / 测量年龄，预测不能刷新真实观测年龄 |
| `source / is_simulated` | 来源与模拟标记；模拟目标不得授权真实运动 |
| `target_id / detail` | 当前锁定目标身份与状态解释 |
| `position / horizontal_distance_m / bearing_rad` | 米 / 米 / 弧度；坐标与正方向见下表 |

| 输出坐标 | 轴方向 | 距离与偏角 |
|---|---|---|
| 原光学系 | 右 / 下 / 前 | hypot(X,Z)，atan2(X,Z)，右为正 |
| base_link 车体系 | 前 / 左 / 上 | hypot(X,Y)，atan2(Y,X)，左为正 |

原框中心射线上的代表点和双踝地面参考点含义不同，不能直接混合；地面/统一位置的完整契约见[统一定位与三维姿态](方案C统一定位与三维姿态.md)。

```bash
ros2 topic echo /perception/detections --once
ros2 service call /perception/lock_target std_srvs/srv/Trigger '{}'
ros2 topic echo /perception/target_state --once
ros2 service call /perception/release_target std_srvs/srv/Trigger '{}'
```

YOLO 的中央认领仅在主人未认领时接受；已有主人 TRACKING 或丢失等待 LOST 期间，所有认领方式均拒绝。丢失默认等待 10 秒，主人未返回才释放并允许重新认领；主动换人须先 release_target，再认领。主人等待不延长底盘运动宽限，详细调用见第 8 节。展示图 `/perception/detections_image` 与 Marker 只用于观察，不能替代原始采集图或有效位置契约。

### 6.3 跌倒检测：如何订阅、如何判断

跌倒检测在 C 的 Pose 节点内运行。普通 B 检测路线不提供相同的骨架/跌倒输出；识别结果无需先认领主人。应用只需要整体告警时订阅 `/perception/fall_alarm`，需要逐人骨架或同时处理多人时订阅 `/perception/person_states`。

```bash
ros2 interface show person_interfaces/msg/FallAlarm
ros2 topic echo /perception/fall_alarm person_interfaces/msg/FallAlarm \
  --once --qos-reliability best_effort
ros2 topic echo /perception/person_states person_interfaces/msg/PersonStateArray \
  --once --qos-reliability best_effort
```

`FallAlarm` 字段：

| 字段 | 类型 / 单位 | 含义 |
|---|---|---|
| `header` | std_msgs/Header | RGB 采集时间与光学坐标系；检查时间戳及接收超时 |
| `valid` | bool | 本帧汇总是否有效；false 表示未知，不能解释成没有跌倒 |
| `stage` | uint8 | 0 NONE、1 SUSPECTED、2 CONFIRMED，即无告警、疑似、确认 |
| `track_id` | string | 最高阶段对应的 epoch:track_id；stage=0 时为空 |
| `continued` | bool | 此告警是否来自未检出的延续人员，而非当前真实检测 |
| `box` | float32[4]，像素 | 原图 x1、y1、x2、y2；延续人员用最后检出框；无告警时为 NaN |
| `geometry_confirmed` | bool | 三维/高度几何的重力、地面是否均确认；不是二维阶段上限 |
| `suspected_count / confirmed_count` | uint32 | 本帧疑似 / 确认人数，含检出及延续人员 |
| `detail` | string | 原因、证据来源或失效说明 |

`valid=true, stage=0, detail="no fall"` 表示当前有效帧没有跌倒告警；`valid=false`、零/过期/未来时间戳或话题断流表示结果未知。不能把最后一帧永久显示为当前状态。`geometry_confirmed=false` 不表示检测没运行：三维证据不足时可使用二维时序规则；二维确认还受 `pose_upright_confirmed` 等配置约束，依据不足时最多疑似。

每帧可能重复发布相同阶段。应用应按人员/阶段变化处理，避免每条消息都触发一次提示；同一 RGB 时刻的检出与延续输出已由 `fall_alarm` 汇总，不要再相加造成重复告警。若要完整的多人事件，逐人处理数组而非只看全局最高阶段。

下面的 Python 客户端只观察，不发布语音或运动：

```python
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from person_interfaces.msg import FallAlarm

class FallReader(Node):
    def __init__(self):
        super().__init__('fall_alarm_reader')
        self.max_age_sec = 0.5  # 应用自己的读数期限，按实测处理延迟配置
        self.received_at = None
        self.observation_stamp = None
        self.previous = None
        self.unknown = False
        self.create_subscription(FallAlarm, '/perception/fall_alarm',
                                 self.on_alarm, qos_profile_sensor_data)
        self.create_timer(0.2, self.watchdog)

    def report_unknown(self):
        if not self.unknown:
            self.get_logger().warning('跌倒检测数据未知或已过期')
        self.unknown = True

    def on_alarm(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if not msg.valid or stamp <= 0 or not 0 <= age <= self.max_age_sec:
            self.report_unknown()
            return
        self.received_at = time.monotonic()
        self.observation_stamp = stamp
        was_unknown = self.unknown
        self.unknown = False
        key = (msg.track_id, msg.stage)
        if key != self.previous or was_unknown:
            self.get_logger().info(
                f'target={msg.track_id or "-"} stage={msg.stage} '
                f'suspected={msg.suspected_count} confirmed={msg.confirmed_count}')
            self.previous = key

    def watchdog(self):
        age = (None if self.observation_stamp is None else
               self.get_clock().now().nanoseconds * 1e-9 - self.observation_stamp)
        if (self.received_at is None or time.monotonic() - self.received_at > self.max_age_sec
                or age is None or not 0 <= age <= self.max_age_sec):
            self.report_unknown()

def main():
    rclpy.init()
    node = FallReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

`PersonStateArray.header` 同样是 RGB 采集时间；检查 `valid / is_simulated / detail`。每个 `PersonState` 包含 ID、检测框、置信度、17 个 COCO 关节、逐关节置信度/三维有效标记、姿态与跌倒阶段。`posture` 为 0 UNKNOWN、1 STANDING、2 SITTING_CROUCHING、3 LYING、4 FALLEN；`fall_stage` 为 0 NONE、1 SUSPECTED、2 CONFIRMED。二维点单位为像素，三维点为光学坐标米；缺失关节保持无效/NaN，不得使用 `body_depth_m` 补成三维关节。

当前跌倒输出只供感知与显示，没有自动接入语音报警或底盘动作；软件输出有效不代表真人跌倒或分类准确率已验收。完整判定逻辑见[跌倒检测整体逻辑](跌倒检测整体逻辑.md)。

### 6.4 相机、可视化与运行频率

| 话题 | 类型 | 内容与条件 |
|---|---|---|
| `/camera/color/image_raw` | sensor_msgs/msg/Image | 原始 RGB，拍照/舌苔使用此输入；不含检测框 |
| `/camera/depth/image_raw` | sensor_msgs/msg/Image | 原生深度；16UC1 为毫米，32FC1 为米，以实际 encoding 为准 |
| `/camera/color/camera_info`、`/camera/depth/camera_info` | sensor_msgs/msg/CameraInfo | 对应原生图像的内参 |
| `/camera/color/image_rect`、`/camera/color/camera_info_rect` | Image、CameraInfo | 校正彩色与内参；进程内配准按完成推理发布校正图 |
| `/camera/aligned_depth_to_color/image_raw` | sensor_msgs/msg/Image | 独立配准节点的彩色对齐深度；进程内配准不单独发布此话题 |
| `/perception/color_preview/compressed`、`/perception/detections_preview/compressed` | sensor_msgs/msg/CompressedImage | JPEG 预览，后者含框/标签；不作为业务采集原图 |
| `/perception/skeleton_markers` | visualization_msgs/msg/MarkerArray | 骨架显示，需要相应坐标/有效关节证据 |
| `/perception/target_marker` | visualization_msgs/msg/Marker | 当前有效目标代表点的显示 |

图像/相机信息建议用 `qos_profile_sensor_data`（best effort）订阅；即使发布者使用 reliable，也可由 best effort 订阅者接收。ROS 的命名空间或重映射会改变表中的默认话题名，接入时先发现实际运行图。

```bash
ros2 topic echo /perception/performance person_interfaces/msg/RuntimeMetrics --once
ros2 topic hz /perception/person_states
ros2 topic info /perception/fall_alarm --verbose
python3 scripts/measure_c_rates.py --duration 15
```

`RuntimeMetrics.input_fps / output_fps` 是实际窗口内推理/发布计数；`processing_ms` 是工作线程处理耗时，可能包含配准、深度和跟踪，不能当作 TensorRT 模型执行时间。`observation_age_ms` 是处理输出距采集的年龄。目标状态 20 Hz 心跳、相机帧率与实际人体推理帧率是不同指标。

负载选项属于 `perception_bringup/perception.launch.py` 或 B/C 脚本入口：`with_auxiliary:=false` / `PERCEPTION_AUXILIARY=false` 移出地面/统一定位和坐标转换，启用的 ReID 仍保留；`registration_backend:=cuda` / `GEMINI_REGISTRATION_BACKEND=cuda` 显式使用进程内 CUDA 配准，默认 numpy。公共 `roscar_api/api.launch.py` 当前未透传这两个选项，不能将其当作公共 API 参数。具体环境与互斥规则见[方案 C 实现与验收](方案C实现与验收.md)。

## 7. 底盘与雷达反馈

| 话题 | 类型 | 读取要点 |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | 平面位置 m、速度 m/s 和 rad/s；厂商 pose.position.z 存航向 rad，不是高度 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 角速度 rad/s、加速度 m/s²；坐标以实际 frame_id 为准 |
| `/PowerVoltage` | `std_msgs/msg/Float32` | V，无 header；旧接收值不能证明底盘仍在线 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 距离 m、角度 rad；无效回波不能当作空旷 |
| `/radar/points` | `sensor_msgs/msg/PointCloud2` | 与 scan 同 frame；按 fields/offsets 解码 |
| `/radar/status` | `std_msgs/msg/String` JSON | WAITING / STALE / INVALID / OK / NO_RETURNS；健康不等于允许通行 |

底盘反馈频率取决于串口实际回传。雷达配置目标 10 Hz、状态 5 Hz；串口 460800 及安装 TF 需现场核对，详见[N10P 雷达接入](N10P雷达接入.md)。

一般消息使用 reliable/volatile；传感器订阅可用 `qos_profile_sensor_data` 兼容可靠或尽力而为数据源。实际类型、发布者和 QoS 用以下命令核对：

```bash
ros2 topic info /odom -v
ros2 topic info /scan -v
ros2 topic echo /radar/status --once
```

## 8. 主人认领和十秒等待

| 接口 | 类型 | 行为 |
|---|---|---|
| `/perception/claim_owner` | person_interfaces/srv/ClaimOwner | 请求 `target_id`；空值选当前中央人物，非空精确选检测列表中的 epoch:track_id；返回 success/message |
| `/perception/lock_target` | std_srvs/srv/Trigger | 兼容中央人物认领 |
| `/perception/release_target` | std_srvs/srv/Trigger | 主动释放，抑制自动认领，等待显式再次认领 |
| `/perception/owner_state` | person_interfaces/msg/OwnerState | 20 Hz 生命周期状态；变化时 sequence 增加 |

```bash
ros2 run roscar_api target claim
ros2 run roscar_api target claim --id 0:7
ros2 service call /perception/claim_owner person_interfaces/srv/ClaimOwner '{target_id: "0:7"}'
ros2 run roscar_api target release
ros2 topic echo /perception/owner_state
```

认领要求 `OwnerState.can_claim=true`（UNCLAIMED）、新鲜推理与已确认配准；未分配轨迹的空 ID、过期 epoch、不可见 ID 均拒绝。TRACKING 或 LOST 预留期间，指定 ID、空参数中央认领及兼容 `lock_target` 全部拒绝，重复认领同一 ID 也不重置计时或触发播报。需要主动更换主人时，先调用 `release_target`，再认领；流重置或 SOT 冲突撤回轨迹同样不会自动解除主人预留。认领只选择人，不授权底盘。

`OwnerState` 包含 `header/state/target_id/person_id/sequence/event/lost_for_s/reclaim_after_s/can_claim`。state 为 UNCLAIMED、TRACKING、LOST；event 为 ready、claimed、lost、returned、expired、released。TRACKING 描述目标连续性，不表示三维测距或运动授权有效。

从目标首次不可跟踪开始按单调时钟计时：10 秒内同一主人恢复则清计时并播报返回；到 10 秒先释放旧目标、ReID 主人绑定与旧联动结果，再进入 UNCLAIMED 并播报“主人未返回，可重新认领”。断流同样计时，恰好达到期限时先执行释放。未超时不会因画面里只剩另一人而自动换主人。

播报单独过滤不足 0.5 秒的丢失/返回抖动，不延迟原始状态或 10 秒计时。普通 ByteTrack 的轨迹编号不是持久身份；离开后换号时，要确认原主人，须显式接入已有 ReID 找回联动并先完成主人外观绑定，不能把“画面里只有一个人”当作旧主人身份依据。

超时后可调用 claim_owner 认领任何新鲜可见人物。原先显式开启 `auto_lock_single` 的入口恢复单人连续三帧自动认领；多人仍须显式选择。新主人使用新的选择代次，旧异步联动结果不能接回；机器人是否运动仍由 guard 的模式、故障和授权决定。


## 9. 语音、阶段播报与舌苔

### 9.1 语音与阶段播报

| 接口 | 类型 | 调用含义 |
|---|---|---|
| `/voice/say` | roscar_interfaces/srv/SayText | 纯文字播报；请求 text，返回 success/status/utterance_id/message；完成后返回 |
| `/voice/speak` | execution_mode_interfaces/action/SpeakText | text + 唯一 utterance_id；反馈 phase；result.status=completed 表示完成；可取消自己的 goal |
| `/voice/tts_text` | std_msgs/msg/String | 兼容无需完成回执的文字播报输入 |
| `/voice/start_listening`、`/voice/cancel_listening` | std_srvs/srv/Trigger | 触发一次识别 / 取消正在监听的会话 |
| `/voice/set_asr_enabled` | std_srvs/srv/SetBool | `data=false` 退出识别进程并释放模型内存；`data=true` 重新加载，返回 success/message |
| `/voice/asr_enabled`、`/voice/asr_status` | std_msgs/msg/Bool、std_msgs/msg/String JSON | 当前识别可用状态 / 模型与进程状态；可靠、transient_local，并周期更新 |
| `/voice/cancel_speech` | std_srvs/srv/Trigger | 全局取消播报；普通调用超时应用自己的 action goal 取消，避免干扰检测 |
| `/voice/asr_text`、`/voice/asr_state` | std_msgs/msg/String | 识别结果 / 状态；固定规则入口使用完整结果，讯飞无最终包或截断不发布临时文本 |
| `/voice/speaking`、`/voice/tts_state` | std_msgs/msg/Bool、std_msgs/msg/String | 播放中 / 播放状态 |
| `/voice/audio` | roscar_interfaces/msg/AudioFrame | 唯一录音所有者的共享 PCM16、16 kHz 单声道帧，不增加第二录音器 |
| `/voice_words` | std_msgs/msg/String | 硬件内部唤醒事件“小车唤醒”；不是口头唤醒词“小微小微” |
| `/voice/command` | roscar_interfaces/msg/VoiceCommand | 程序化命令；新鲜 stamp 和 request_id；STOP/DISARM/ESTOP/ARM/SET_MODE/RESET_ESTOP/RESET_FAULT/QUERY_STATUS |
| `/voice/motion_request` | roscar_interfaces/msg/MotionRequest | 固定规则产生的语义动作，经能力校验/阿克曼适配；绑定当前 authority_epoch |
| `/voice/command_result` | roscar_interfaces/msg/VoiceCommandResult | request_id、action、success、message、模式回执 |
| `/voice/tool_result`、`/voice/assistant_text` | std_msgs/msg/String | 兼容 JSON 回执 / 助手文字 |
| `/voice/emergency_healthy`、`/voice/emergency_event` | std_msgs/msg/Bool、std_msgs/msg/String | 常开 KWS 健康及事件输出；包括 TTS 期间 |
| `/system/phase` | std_msgs/msg/String | 主人/跟随阶段 JSON：source、phase、text；表示播报请求，非播放成功回执 |

```bash
ros2 service call /voice/say roscar_interfaces/srv/SayText '{text: "开始跟随"}'
ros2 action send_goal /voice/speak execution_mode_interfaces/action/SpeakText \
  '{utterance_id: "app-001", text: "正在分析"}' --feedback
ros2 service call /voice/start_listening std_srvs/srv/Trigger '{}'
ros2 topic echo /voice/command_result
```

文字不进入意图解析。300 字上限，空白拒绝；服务忙/不可用/失败/超时必须检查 status，见 [文字播报接口](文字播报ROS接口.md)。底层始终复用唯一 TTS 队列，不靠新建播放器实现阶段播报。

B/C 整机入口开启语音及 TTS 时自动启动阶段观察器，通过 `/voice/say` 播报：等待认领、认领成功、开始跟随、主人丢失、主人返回、10 秒到期重新认领、主动释放。相同阶段心跳不重复播，目标故障恢复不重复“开始跟随”。播报忙时短暂重试并合并同来源未发送状态；当前阶段心跳持续时等待语音就绪，心跳中断超过 4 秒的过时内容丢弃，接口不可用不阻塞感知/控制。故障可查看播报节点日志和 `/system/phase`。

#### ASR 开关与内存释放（2026-10-14 已部署验证）

小车 `/home/wheeltec/ROSCAR-current` 已更新并原生构建新版语音入口，`roscar-robot.service` 正在提供此接口，ROS 域号仍为 182。其他部署需要先构建：

```bash
source /opt/ros/humble/setup.bash
bash scripts/build_ros.sh --packages-select offline_voice voice_command_router roscar_api
source install/setup.bash
```

随后使用原有完整语音/整机入口加载新版本。只启动旧 `asr_node` 不会提供新的开关，不能与 manager 管理的识别进程同时启动。

板端两轮真实 Qwen3/Silero 加载占用约 1.2 GiB RSS，关闭约 0.68 秒后进程消失；重载后的数字音频回放识别到“停止。”。真实 MeloTTS/ALSA 在关闭 ASR 时完成正在播放的内容，关闭期间的新 `/voice/say` 请求也完成。正式主链另外验证了麦克风继续采集、重载后监听/取消服务及 CPU 亲和性 0、1，且语音/速度发布者各只有一份。测试结束 ASR 已开启，小车保持 IDLE/DISARMED；这不代表真人识别准确率或运动/制动验收。

```bash
# 关闭识别，等待识别进程退出并释放 ASR/VAD 模型内存
ros2 service call /voice/set_asr_enabled std_srvs/srv/SetBool '{data: false}'

# 恢复识别：模型加载成功后才返回 success=true
ros2 service call /voice/set_asr_enabled std_srvs/srv/SetBool '{data: true}'

ros2 topic echo /voice/asr_status --once
```

关闭后的状态为 `enabled=false/state=DISABLED/models_loaded=false/worker_pid=0/worker_rss_kib=0`。开启后为 `ENABLED`，带新进程 PID 与 RSS（KiB）。其他阶段为 `LOADING`、`DISABLING`、`ERROR`；程序应检查服务 `success/message`，不能只看请求已发出。重复设置同一状态可安全调用，已经开启时不重复加载模型。

轻量 `asr_manager` 保持 ROS 开关和原 `/voice/start_listening`、`/voice/cancel_listening` 服务在线，模型在独立工作进程。关闭时先停止转发识别结果，再请求该进程退出；原生推理未响应时有界强制结束。只有进程确认退出才返回关闭成功，因此 Qwen3-ASR、Silero VAD、原生推理会话和该进程的中间缓冲一并释放，而不是仅暂停监听。Linux 可回收的文件页缓存可能仍保留，不代表模型仍驻留在识别进程。

开启默认最多等待 60 秒加载，失败/超时返回 `success=false`、状态 ERROR，并清理失败进程。正在加载时的关闭请求会中止加载；其他并发开启请求可能返回正在切换状态，按 message 稍后重试。识别进程意外退出后不会自动反复加载，需显式重新开启。初始状态参数 `asr_enabled` 只用于启动，运行时使用上述服务。

关闭会清理普通语音会话和待完成的语音授权，丢弃迟到识别结果；恢复后需要重新喊“小微小微”，不会恢复旧动作或旧会话。TTS、现有播报接口、共享录音以及独立 KWS 的原有配置保持不变；此开关不会取消正在播放的提示或检测任务，也不会关闭跟随主链或解除故障。已完成的驾驶授权仍由原保护逻辑管理，尚在等待的语音授权会按既有取消逻辑撤销。当前配置若原本关闭 KWS，此开关也不会自行启用它。

ASR 关闭/加载时，`/voice/start_listening` 返回 `success=false`；取消空会话可成功返回。需要开机先不加载识别模型，可设置 `VOICE_ASR_ENABLED=false` 启动现有语音脚本，或在 launch 中传 `asr_enabled:=false`；默认仍开启。ASR 开关运行状态不写入私有配置，重启按启动参数决定。

### 9.2 舌苔执行模式

```bash
ros2 run roscar_api mode ENTER --mode tongue
ros2 run roscar_api mode RETAKE --mode tongue
ros2 run roscar_api mode REPEAT --mode tongue
ros2 run roscar_api mode EXIT --mode tongue
```

旧 `python3 scripts/mode_command.py ...` 保留为兼容入口，需先 source 当前 install。命令行创建当前时间戳及唯一 request_id，等待订阅者后发送；“请求已发送”不是检测成功，后续读取 state/result。

| 接口 | 类型 | 契约 |
|---|---|---|
| `/execution_mode/request` | execution_mode_interfaces/msg/ModeCommand | 外部请求入口，header.stamp、request_id、mode_id、operation；ENTER/RETAKE/REPEAT/EXIT；session_id 由管理器填充 |
| `/execution_mode/state` | std_msgs/msg/String | active/busy/mode_id/session_id/phase/detail/result_json |
| `/modes/tongue/run` | execution_mode_interfaces/action/RunMode | 完整模式会话；goal 包含 mode_id/session_id/request_id；反馈 phase/detail/result_json；最终 status/detail/last_result_json |
| `/execution_mode/control`、`/execution_mode/heartbeat` | ModeCommand、std_msgs/msg/String | 管理器到模式的内部协作通道；外部应用使用 request 避免绕过串行清理 |
| `/tongue/state`、`/tongue/result` | std_msgs/msg/String | 阶段 JSON / 经版本、请求/采集 ID 和八项契约校验的结果 JSON |

完整消息类型可用 `ros2 interface show execution_mode_interfaces/action/RunMode` 查看。RunMode 在 READY 仍保持，不会一轮拍摄结束就返回；直接 action 客户端须承担管理器心跳/串行退出职责，普通集成应调用 mode/request。EXIT 等旧任务完成清理后才允许切换；不会恢复运动。

阶段播报通过 `/voice/speak` 等待逐条完成：检查服务 → 面向相机伸舌 → 准备拍摄 → 正在拍摄 → 正在分析 → 简短结果 → “可重拍或退出”。拍摄窗口在“正在拍摄”播放完成后建立，不能取播报前的缓存帧。READY_PROMPT 是最后提示播放阶段，完成后才进入 READY 并开放有界续聊；失败播报为“检测未完成，请重拍”。最后的续聊提示若播放失败，会记录 detail 并保留操作入口，不丢弃已完成的检测结果。阶段不随反馈心跳重复播报，晚到的过期语音 goal 只取消自身。

进入检测先获得停车保持并确认静止；异常模式、ESTOP、KWS 失效及心跳失效均退出。云端地址可配置，冻结模型版本和响应契约不能省略；图片及凭据不提交。完整流程见 [舌苔整合与验收](语音舌苔流程整合与验收.md)。


## 10. 云台与导航

| 接口 | 类型 | 调用 / 订阅 |
|---|---|---|
| `/scan` | sensor_msgs/msg/LaserScan | 距离米、角弧度；订阅使用 sensor_data / best_effort；按 header 时间查 TF |
| `/radar/status` | std_msgs/msg/String | 雷达状态 JSON |
| `/gimbal/command` | gimbal_interfaces/msg/GimbalCommand | 云台命令输入，见下文 |
| `/gimbal/status` | gimbal_interfaces/msg/GimbalStatus | connected/time_synced/pan_source/pan_valid/角度/故障等 |
| `/gimbal/joint_states`、`/gimbal/imu` | sensor_msgs/msg/JointState、sensor_msgs/msg/Imu | MCU 采样时间换算的关节和惯性反馈 |
| `/gimbal/set_pan_zero`、`/gimbal/calibrate_gyro`、`/gimbal/save_config` | std_srvs/srv/Trigger | 设零/校准/保存；success 表示 MCU ACK，不能替代硬件精度验收 |
| `/navigation/start_follow`、`/navigation/stop_follow` | std_srvs/srv/Trigger | 导航跟随目标规划开关，不授予运动权限 |
| `/navigation/state`、`/navigation/follow_goal` | std_msgs/msg/String、geometry_msgs/msg/PoseStamped | 导航状态 / 跟随目标 |
| `/navigation/cmd_vel_raw` | geometry_msgs/msg/Twist | Nav2 输出给速度适配器的内部通道 |
| `/navigate_to_pose` | nav2_msgs/action/NavigateToPose | Nav2 导航 action，仅在导航栈启动后可用 |

```bash
ros2 run roscar_api radar
ros2 topic echo /scan --qos-reliability best_effort
ros2 service call /gimbal/set_pan_zero std_srvs/srv/Trigger '{}'
ros2 interface show gimbal_interfaces/msg/GimbalCommand
ros2 service call /navigation/start_follow std_srvs/srv/Trigger '{}'
ros2 service call /navigation/stop_follow std_srvs/srv/Trigger '{}'
```

云台命令带当前 header.stamp，mode 为 DISABLE=0、HOLD=1、POSITION=2、RATE=3、HOME=4；pan_target_rad 相对车体左正，tilt_target_rad 相对重力向下正，RATE 使用 pan_rate_ff_radps，速率单位弧度/秒。持续命令推荐 20 Hz，默认 0.2 秒无新命令转 HOLD。示例只保持当前位置：

```python
from gimbal_interfaces.msg import GimbalCommand
pub = node.create_publisher(GimbalCommand, '/gimbal/command', 10)
msg = GimbalCommand(mode=GimbalCommand.HOLD)
msg.header.stamp = node.get_clock().now().to_msg()
pub.publish(msg)
```

导航使用独立 `ros2 launch navigation_bringup navigation.launch.py`，它拥有自己的 guard，不能叠加 api.launch.py 或普通跟随生产者。Nav2/map/TF 等标准端点通过 `roscar_api interfaces` 完整列举；见 [导航文档](导航与自动绕障.md)。云台实物协议、时钟和坐标规则见 [云台协议](云台跟随设计与串口协议.md)。


## 11. 常见问题排查

| 现象 | 先检查什么 |
|---|---|
| 远程电脑 `ros2 node list` 为空 | 是否同网段能 ping 通、路由器是否隔离客户端或拦截组播、防火墙、两端 `ROS_DOMAIN_ID=182` 与 `ROS_LOCALHOST_ONLY=0`；不行就改用 SSH 登录小车调用（2.5 方式 A） |
| 服务列表没有 `/capture/*` | `with_ros_api` 默认关闭；确认 api_node、overlay、域号与 DDS |
| 拍照返回 busy | 是否已有 ROS、云端或直接 Action 任务占执行器；本接口不排队 |
| rejected / voice_mode_active | 语音舌苔任务尚未退出；先等其清理完成 |
| failed / fresh_frame_timeout | 原始 RGB 是否更新、分辨率/编码及帧时戳是否正确；采集必须晚于准备阶段 |
| failed / hold_failed 或 odom_stale | robot 的 guard、KWS、停车租约、底盘静止回传与 ESTOP 状态 |
| failed / upload_http_404 | 云端路径前缀是否正确，以及是否允许 ROS 自行生成的 task_id |
| failed / upload_http_401 或 403 | 云端凭据与权限；不要把 token 写到日志或 URL |
| `/voice/say` unavailable | 唯一 TTS 是否已启动且启用 `/voice/speak` action |
| 授权失败 | 看服务 message 与 `/control/state`；核对 epoch、零请求、运动开关与保护前提 |
| 有框但位置无效 | 看 position_valid/detail、配准深度、内参与观测新鲜度 |
| 光学位置有效但 base 无效 | 安装外参与观测时刻 TF 是否已确认 |
| 人体框可见但主人丢失 | 分别检查 target_state 的 STALE/LOST、观测年龄、轨迹 epoch，以及 person_identities 的 valid/verified；检出人体不等于确认原主人 |
| 找不到 fall_alarm | 是否运行 C Pose 节点、域号/命名空间是否一致；只有 B 检测框不代表跌倒节点已启用 |
| fall_alarm.valid=false 或断流 | 当作未知；查看相机、同步、模型和帧龄；不能把无消息当 stage=0 |

## 12. 验证范围与进一步阅读

本地拍照 ROS 服务已做 Humble x86_64 编译与合成 RGB、假 TTS、mock 云端联调。过去的真实 USB 相机采集记录另见验收文档；这些不等于本次新增接口已在 Jetson、正式云端或小车上验收。实车运动、制动、标定和实际听感仍按现场要求确认。

- [文字播报 ROS 接口](文字播报ROS接口.md)：播报专用调用手册。
- [云端拍照任务接口协议](云端拍照任务接口协议.md)：给云端开发者的 HTTP 合同。
- [云端拍照任务实现与验收](云端拍照任务实现与验收.md)：启动组合、联调记录与待验收项。
- [TargetState 接口](interfaces.md)：锁定目标消息说明。
- [motion_guard](../ros2_ws/src/motion_guard/README.md)：运动保护实现。
- [工作记录](../WORKLOG.md)：逐次变更和验证证据。


## 固定机位开机入口与跌倒播报

当前专用入口为 `scripts/run_robot_onboard.sh`，用户服务为 `roscar-robot.service`；启动/停止方法、运动确认、机位参数和准确率边界见 [10-13 板端联调记录](2026-10-13板端联调与开机跟随.md)。它复用既有控制服务，不能再叠加第二个 guard。跌倒播报订阅已有 `/perception/fall_alarm`，阶段 JSON `/system/phase` 的 `source=fall`；实际播放使用 `/voice/say`/`/voice/speak`，不增加运动接口。


### 固定机位 2/5 秒找回配置

当前 `run_robot_onboard.sh` 显式重启 ReID 找回：`pose_fixed_27cm.yaml` 的 `owner_continuity_reset_s=2.0`、入口 `OWNER_LOST_TIMEOUT_S=5.0` 从首次连续丢失起算，前者只清轨迹/VitTrack 连续性，后者清主人绑定并开放认领。`reid_owner_trial.yaml` 的 `forget_released_owner=true` 同时删除释放主人的外观库并作废旧推理结果。通用默认仍为十秒且连续性定时重置关闭；此配置不改变运动宽限、急停或故障授权。


固定机位主链速度由板端 `FOLLOW_MAX_LINEAR_MPS` 显式配置（用户本轮选择 0.5），同时传到 follow.launch.py 的 `max_linear_mps` 与串口入口；`FOLLOW_DISTANCE_GAIN` / `FOLLOW_BEARING_GAIN` 对应控制增益，通用 launch 默认仍 0.6/1.2。不能只提高请求端而遗漏 guard/串口上限；目标失效与急停仍停车。


当前固定机位入口按用户要求 VOICE_ESTOP_ENABLED=false：独立语音 KWS 关闭，语音 ESTOP 映射普通 STOP；ROS `/control/estop` 服务仍有效，显式锁存仍需 `/control/reset_estop`。普通 STOP 回 IDLE，不自动恢复跟随。通用语音启动参数 `voice_estop_enabled` 默认 true。


## 13. Demo 分支：相对运动接口（前进/后退 N 米、左转/右转 N 度）

仅 `demo/showcase-20261016` 展示分支提供，分支删除时一并删除；主线接口不变。完整字段、参数和冲突说明见 [相对运动 ROS 接口](demo/相对运动ROS接口.md)。

| 接口 | 类型 | 说明 |
|---|---|---|
| `/demo/move_distance` | `demo_interfaces/srv/MoveDistance` | `distance_m` + 前进 / − 后退；`speed_mps` 0 为默认 0.15 |
| `/demo/turn_angle` | `demo_interfaces/srv/TurnAngle` | `angle_deg` + 左转 / − 右转；`reverse=true` 倒车转弯；阿克曼车不能原地转 |
| `/demo/move_relative` | `demo_interfaces/action/MoveRelative` | `kind` MOVE/TURN，带进度反馈、可取消 |
| `/demo/motion_cancel` | `std_srvs/srv/Trigger` | 取消当前动作并停车 |

```bash
ros2 launch demo_motion relative_motion.launch.py
ros2 service call /demo/move_distance demo_interfaces/srv/MoveDistance "{distance_m: 0.5, speed_mps: 0.0}"
ros2 service call /demo/turn_angle demo_interfaces/srv/TurnAngle "{angle_deg: -90.0, yaw_rate_rps: 0.0, reverse: false}"
```

每次调用即一次明确授权：节点切 EXTERNAL、发新鲜零、绑定 epoch 授权，`/odom` 闭环到达后发零并 `/control/stop` 回 IDLE。`motion_guard` 仍是唯一 `/cmd_vel` 出口，`motion_enabled`、限速与急停照常生效。须为 `/chassis/cmd_vel` 唯一发布者，不能与语音运动适配器同时运行。当前只有纯 Python 单测，colcon 编译、ROS 运行时和实车均未验证。
