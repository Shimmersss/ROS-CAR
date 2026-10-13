# ROSCAR ROS 2 接口使用文档

更新：2026-10-13。本文对应当前源码的 B/C 感知、主人认领、受保护底盘控制、语音、舌苔、雷达、云台和导航接口。所有接口通过原生 ROS 2 topic / service / action 调用；没有另开 HTTP 控制端口。运行时的类型、发布者和服务是否就绪，以实际 ROS 图为准。

## 1. 构建、发现和启动

在 Ubuntu 22.04 / ROS 2 Humble 的项目根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
bash scripts/build_ros.sh
source install/setup.bash
export ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=1
ros2 launch roscar_api api.launch.py
```

构建及 B/C 启动统一使用仓库根目录 `install/`，不再默认加载旧 `ros2_ws/install/`。脚本需要另一套 overlay 时设置 `PERCEPTION_INSTALL=/绝对路径/install`。厂商底盘、雷达和麦克风仍分别构建、source 独立 overlay，不移除它们的 `COLCON_IGNORE`。

默认只启动一个 motion_guard，IDLE、运动关闭、不自动授权，不开启相机、串口或录音。已有 `start_project.sh` / `start_robot.sh` / 导航等入口包含 guard 时，直接使用它们的公共接口，**不要再叠加 api.launch.py**。需要单独补上播报桥时使用 `speech_api.launch.py`。

```bash
# 枚举所有已发现的接口（包含中间件参数服务及模块内部协作接口）
ros2 run roscar_api interfaces
ros2 run roscar_api interfaces --json --wait 3
ros2 service list -t
ros2 action list -t
ros2 topic info -v /perception/owner_state
ros2 interface show person_interfaces/srv/ClaimOwner
ros2 launch roscar_api api.launch.py --show-args
```

图中出现 topic/action 名称不代表存在发布者或 action server，客户端自身也会注册名称。程序调用前使用 `wait_for_service()` / `wait_for_server()`，订阅后检查时间戳与有效位；不能把无数据当作零或正常状态。局域网调用要求两端安装相同接口版本、相同 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY=0`、DDS 网络畅通、时钟同步。Foxglove WebSocket 用于展示，不等于原生 DDS 客户端。

### 可选启动项

| 参数 | 默认 | 含义 |
|---|---|---|
| `with_perception`、`route` | false、yolo | B 用 yolo，C 用 yolo_pose；相机独立启动；astra/red 仅保留旧调用兼容 |
| `model_path`、`device`、`yolo_python` | 空、cpu、空 | 明确本地模型及兼容解释器；缺模型不静默回退 |
| `depth_registered` | false | 物理配准验收后显式确认 |
| `color_topic`、`depth_topic`、`camera_info_topic` | `/camera/color/image_rect`、`/camera/aligned_depth_to_color/image_raw`、`/camera/color/camera_info` | 同步 RGB-D 输入 |
| `pose_config`、`camera_mount_config`、`reid_config` | perception_bringup 对应 YAML | 骨架/跌倒、安装/地面、身份参数；确认开关不自动置真 |
| `reid_enabled`、`reid_lock_enabled`、`reid_reacquire_enabled`、`sot_enabled` | false | 身份观察、身份锁定、主人找回、单目标联动；仍遵守互斥和模型校验规则 |
| `reid_model_path`、`sot_model_path` | 空 | 本地模型和校验文件，显式启用时必须齐全 |
| `registration_calibration_file`、`registration_expected_serial` | 空 | C 可选进程内 Gemini 配准，不能与 ReID 同开 |
| `auto_lock_single`、`owner_lost_timeout_s` | false、10.0 | 单人连续三帧自动认领开关、主人丢失等待秒数 |
| `with_follower`、`target_distance_m` | false、1.0 | 跟随请求生产者；不自动切模式或授权 |
| `expected_source`、`target_frame`、`target_topic` | yolo、camera_color_optical_frame、`/perception/target_state` | 同时配置 guard 与跟随器的目标契约；来源/坐标必须与输出一致 |
| `motion_enabled`、`safety_config` | false、motion_guard/config/safety.yaml | 运动总开关及已验收安全参数；开总开关仍需逐次授权 |
| `with_radar`、`radar_required` | false、false | 启动雷达、让 guard 要求扫描/TF/包络保护；两个开关含义不同 |
| `with_chassis`、`serial_port`、`car_mode` | false、空、空 | 启动底盘必须明确串口与车型；本车使用 mini_akm |
| `with_voice`、`voice_config` | false、offline_voice/config/offline_voice.yaml | 启动共享录音、常开 KWS、固定规则、ASR、唯一 TTS，同时开放播报 API |
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

## 2. 底盘和运动授权

### 公共服务

| 名称 | 类型 | 请求 / 返回与语义 |
|---|---|---|
| `/control/set_mode` | roscar_interfaces/srv/SetControlMode | 请求 `mode: IDLE/EXTERNAL/FOLLOW`；返回 `success/message/authority_epoch`；切换撤销旧票据并清请求 |
| `/control/authorize` | roscar_interfaces/srv/AuthorizeMotion | 请求当前 `authority_epoch` 和 `mode`；返回 `success/message`；必须有新鲜零请求及全部健康条件 |
| `/control/arm` | std_srvs/srv/Trigger | 兼容入口，默认拒绝，使用 authorize |
| `/control/stop`、`/control/disarm` | std_srvs/srv/Trigger | 停车、回 IDLE、撤销授权；不清除 FAULT/ESTOP 锁存 |
| `/control/estop` | std_srvs/srv/Trigger | 锁存紧急制动；持久化保留 |
| `/control/reset_estop`、`/control/reset_fault` | std_srvs/srv/Trigger | 条件满足时显式复位，只回未授权状态；仍需重新选模式和授权 |
| `/control/task_hold` | execution_mode_interfaces/srv/TaskHold | `ACQUIRE/RENEW/RELEASE` 停车租约；字段见下例；释放/到期不恢复运动 |

```bash
ros2 service call /control/set_mode roscar_interfaces/srv/SetControlMode '{mode: EXTERNAL}'
# 终端 A：持续发零并打印本次 epoch；等待明确授权后最多运行 2 秒，退出自动发零并 STOP
ros2 run roscar_api chassis --speed 0.05 --turn 0 --duration 2
# 终端 B：将 123 替换成上条程序实际返回的 epoch，不能复用旧值
ros2 service call /control/authorize roscar_interfaces/srv/AuthorizeMotion \
  '{authority_epoch: 123, mode: EXTERNAL}'
ros2 service call /control/stop std_srvs/srv/Trigger '{}'
ros2 service call /control/estop std_srvs/srv/Trigger '{}'
ros2 service call /control/reset_estop std_srvs/srv/Trigger '{}'
```

默认运动关闭时授权失败是正常结果。外部程序只向 `/chassis/cmd_vel` 发布 `geometry_msgs/msg/TwistStamped`：新鲜 ROS 时间，`frame_id=base_link`，`linear.x` 米/秒、`angular.z` 弧度/秒，其余分量零。推荐 20 Hz，默认请求超时 0.2 秒。限速取当前 guard 配置，超限/NaN/未来时间戳会被拒绝；不要直接向最终 `/cmd_vel` 发命令。

FOLLOW 的内部请求是 `/control/cmd_vel_request`（TwistStamped），只保留一个跟随或导航生产者。目标必须真实、新鲜、来源/坐标正确。目标短丢失宽限和目标故障自动恢复使用既有配置；**主人 10 秒认领窗口不延长运动宽限**。目标 FAULT 中发生急停健康 false 或心跳超时，会升级为不可自动恢复的故障；恢复目标及健康后仍须显式复位。

`/control/state` 为 `std_msgs/msg/String` JSON，约 20 Hz：`authority_state`（DISARMED/ARMED_MANUAL/ARMED_FOLLOW/FAULT/ESTOP）、`authority_epoch`、`command_mode`、`ready/reason/last_fault`、`motion_enabled`、`emergency_healthy`、`task_hold`。旧 `mode` 为 PERCEPTION_ONLY/STANDBY/ARMED/FAULT 兼容视图；判定授权以 authority_state 为准。

| 话题 | 类型 | 方向 / 含义 |
|---|---|---|
| `/chassis/cmd_vel` | geometry_msgs/msg/TwistStamped | 外部请求输入 |
| `/control/emergency_stop` | std_msgs/msg/Bool | true 触发 ESTOP；false 不复位 |
| `/cmd_vel` | geometry_msgs/msg/Twist | guard 唯一输出，底盘消费 |
| `/odom` | nav_msgs/msg/Odometry | 底盘反馈，米、米/秒、弧度/秒；厂商 pose.position.z 为航向，不能当高度 |
| `/imu/data_raw` | sensor_msgs/msg/Imu | 底盘惯性反馈，读实际 frame_id |
| `/PowerVoltage` | std_msgs/msg/Float32 | 电压 V，无时间戳，要自行检查接收超时 |
| `/control/performance` | person_interfaces/msg/RuntimeMetrics | 跟随耗时和吞吐统计 |

任务持有者示例（租约只保持静止）：

```bash
ros2 service call /control/task_hold execution_mode_interfaces/srv/TaskHold \
  '{operation: ACQUIRE, owner: my_app, session_id: session_001, lease_sec: 2.0}'
# 保存响应 token，定期 RENEW；结束时 RELEASE；不要伪造其他任务的 token
ros2 service call /control/task_hold execution_mode_interfaces/srv/TaskHold \
  '{operation: RELEASE, token: "替换为响应中的token"}'
```

## 3. 主人认领和十秒等待

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

超时后可调用 claim_owner 认领任何新鲜可见人物。原先显式开启 `auto_lock_single` 的入口恢复单人连续三帧自动认领；多人仍须显式选择。新主人使用新的选择代次，旧异步联动结果不能接回；机器人是否运动仍由 guard 的模式、故障和授权决定。

## 4. 感知与相机输出

这些是订阅接口，不是运动命令。B/C 的 `TargetState.source=yolo` 保持兼容。数组时间戳是原 RGB 观测时间；状态消息有单独 `observation_stamp/measurement_age_s`，不能用发布心跳代替真实测量年龄。

| 话题 | 类型 | 内容 / 启用条件 |
|---|---|---|
| `/camera/color/image_raw`、`/camera/color/image_rect` | sensor_msgs/msg/Image | 原始/校正彩色；由相机/配准入口提供 |
| `/camera/depth/image_raw`、`/camera/aligned_depth_to_color/image_raw` | sensor_msgs/msg/Image | 原深度/配准深度；16UC1 毫米或 32FC1 米，须核对 encoding |
| `/camera/color/camera_info`、`/camera/color/camera_info_rect` | sensor_msgs/msg/CameraInfo | 与实际图像尺寸及光学坐标匹配的内参 |
| `/perception/detections` | vision_msgs/msg/Detection2DArray | 全部二维框；bbox 原图像素，id=epoch:track_id，未跟踪为空 |
| `/perception/detections_image` | sensor_msgs/msg/Image | 缩放后的画框视频，不可拿其尺寸解释原框坐标 |
| `/perception/color_preview/compressed`、`/perception/detections_preview/compressed` | sensor_msgs/msg/CompressedImage | 低带宽预览 |
| `/perception/target_state`、`/perception/target_state_base` | person_interfaces/msg/TargetState | 光学框中心代表点 / 已确认安装后的车体系代表点 |
| `/perception/target_marker`、`/perception/target_marker_base` | visualization_msgs/msg/Marker | 对应目标标记 |
| `/perception/person_states` | person_interfaces/msg/PersonStateArray | C 真实检测、2D/3D 关节、姿态和跌倒状态；缺关节 NaN |
| `/perception/person_states_continued` | person_interfaces/msg/PersonStateArray | 显式开启丢失高度延续后的独立输出，不能混入真实检测 |
| `/perception/fall_alarm` | person_interfaces/msg/FallAlarm | C 每帧汇总 NONE/SUSPECTED/CONFIRMED；当前不触发语音或运动 |
| `/perception/skeleton_markers` | visualization_msgs/msg/MarkerArray | 骨架可视化 |
| `/perception/person_identities` | person_interfaces/msg/PersonIdentityArray | ReID 身份观察及有效性；开关/本地模型要求见 C 文档 |
| `/perception/owner_focus` | person_interfaces/msg/PersonIdentity | 主人找回模式向 ReID 传递当前身份，模块协作输出 |
| `/perception/person_ground_states`、`/perception/person_positions` | person_interfaces/msg/PersonGroundArray | 单目双踝接地点 / 统一脚下位置，车体系米 |
| `/perception/target_state_ground`、`/perception/target_state_unified` | person_interfaces/msg/TargetState | 对应已选择人物的位置；主选择 LOST、陈旧或冲突时同步无效 |
| `/perception/ground_markers`、`/perception/position_markers` | visualization_msgs/msg/MarkerArray | 地面/统一位置可视化 |
| `/perception/floor_plane` | person_interfaces/msg/FloorPlane | 显式开启深度地面拟合后的本帧证据，不自动确认标定 |
| `/perception/performance` | person_interfaces/msg/RuntimeMetrics | FPS、处理耗时及观测年龄 |
| `/tf`、`/tf_static` | tf2_msgs/msg/TFMessage | 已确认的坐标关系，依观测时刻查询 |

```bash
ros2 run roscar_api detections
ros2 topic echo /perception/target_state
ros2 topic echo /perception/fall_alarm
ros2 interface show person_interfaces/msg/PersonState
ros2 interface show person_interfaces/msg/PersonGround
```

`TargetState` 状态为 NOT_READY/SEARCHING/TRACKING/LOST/STALE；使用位置必须同时检查 TRACKING、position_valid、非模拟、来源/坐标及真实观测时效。光学代表点和脚下投影是不同参考点。地面/统一定位还要求确认安装和地面；即使全员位置数组有效，LOST 的主人也不得成为有效跟随目标。细节见 [感知消息契约](interfaces.md) 与 [C 统一定位与三维姿态](方案C统一定位与三维姿态.md)。

## 5. 语音与阶段播报

| 接口 | 类型 | 调用含义 |
|---|---|---|
| `/voice/say` | roscar_interfaces/srv/SayText | 纯文字播报；请求 text，返回 success/status/utterance_id/message；完成后返回 |
| `/voice/speak` | execution_mode_interfaces/action/SpeakText | text + 唯一 utterance_id；反馈 phase；result.status=completed 表示完成；可取消自己的 goal |
| `/voice/tts_text` | std_msgs/msg/String | 兼容无需完成回执的文字播报输入 |
| `/voice/start_listening`、`/voice/cancel_listening` | std_srvs/srv/Trigger | 触发一次识别 / 取消正在监听的会话 |
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

## 6. 舌苔执行模式

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

## 7. 雷达、云台和导航

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

## 8. Python 服务调用与验证

以下调用既有认领服务，不启动额外节点栈或运动：

```python
import rclpy
from rclpy.node import Node
from person_interfaces.srv import ClaimOwner

rclpy.init()
node = Node('my_owner_client')
client = node.create_client(ClaimOwner, '/perception/claim_owner')
try:
    if not client.wait_for_service(timeout_sec=3):
        raise RuntimeError('认领服务未就绪')
    future = client.call_async(ClaimOwner.Request(target_id='0:7'))
    rclpy.spin_until_future_complete(node, future, timeout_sec=3)
    if not future.done():
        raise TimeoutError('认领响应超时，请先查询 owner_state 再决定是否重试')
    response = future.result()
    print(response.success, response.message)
finally:
    node.destroy_node()
    rclpy.shutdown()
```

其他 service 替换消息类型、路径及字段即可。Service 客户端超时不撤销服务端已执行的操作；状态写入后先查询实际状态，不盲目重发。需要取消和完成反馈的长任务使用 action，保留自己的 goal handle。

回归入口：`bash scripts/test_flow_regressions.sh`（先在 Humble 构建当前源码），覆盖五项缺陷、完整 10 秒主人生命周期、播报去重、定位门禁、API 模块与舌苔流程。`PYTHON=.venv/bin/python scripts/ci_unit_tests.sh` 为无 ROS 纯单测。容器 ROS 测试使用合成 RGB-D/里程计、可控语音回执和本地假云端；通过仅证明软件流程，不等于麦克风、扬声器、模型识别、云台硬件或实车验收。
