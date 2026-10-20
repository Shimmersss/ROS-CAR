# Demo 按需模式 ROS 接口调用文档

更新日期：2026-10-20。模式运行实现：`31d3693`（10-20 另修复共享收音自动恢复），分支：`demo/showcase-20261016`。

仅适用于 `demo/showcase-20261016`，不替换 main 的通用 API。板端开机入口为 `scripts/run_demo_onboard.sh` → `run_demo_runtime.sh`。语音、文字播报、唯一底盘驱动、跟随控制器及唯一 motion_guard 常驻；相机、Pose、ReID、VitTrack 和检测执行器按需启动。不开录制。

## 1. 连接与调用环境

推荐从电脑 SSH 到小车，再执行本页命令。小车当前地址为 `192.168.31.145`，部署路径为 `/home/wheeltec/ROSCAR-current`。

```bash
ssh wheeltec@192.168.31.145
source /opt/ros/humble/setup.bash
source /home/wheeltec/ROSCAR-current/install/setup.bash
# 与当前服务使用相同的 ROS 域与 DDS 配置；不要打印或外传私有文件内容
set -a
source "$HOME/.config/roscar/robot.env"
set +a
export ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=0
```

先检查服务和模式。服务已经自启动，不要再启动第二套 launch：

```bash
systemctl --user is-active roscar-robot.service
ros2 topic echo /robot/state --once
ros2 topic echo /control/state --once
ros2 service list -t
```

正常待机为 `/robot/state` 的 `mode=IDLE, phase=IDLE`，以及 `/control/state` 的 `command_mode=IDLE, authority_state=DISARMED`。`motion_enabled=true` 仅表示底盘运动能力已打开，不代表当前正在运动。首次启动可能短暂为 BOOTING，等到 IDLE 再调用。

其他 Linux ROS 2 Humble 客户端也可接入，但必须构建同一版本的接口包、配置可通信的 DDS 与 domain 182；不要将 Jetson 的 aarch64 install 直接用于 x86 电脑。以下命令默认在小车 SSH 终端执行。

## 2. 跟随、跌倒与模式切换

```bash
# 开始跟随：中央认领，启动当前跟随和跌倒检测，并完成零请求/当前 epoch 授权
ros2 service call /follow/start std_srvs/srv/Trigger '{}'
# 精确认领可通过统一入口指定 target_id；空字符串选中央新鲜轨迹
ros2 service call /robot/set_mode demo_interfaces/srv/SetRuntimeMode "{mode: FOLLOW, target_id: ''}"
# 停车并卸载当前模式（不关闭语音和底盘驱动）
ros2 service call /robot/stop std_srvs/srv/Trigger '{}'
# 只看感知，不授权运动
ros2 service call /robot/set_mode demo_interfaces/srv/SetRuntimeMode '{mode: PERCEPTION}'
# 进入 ROS 速度控制，返回成功后才接受速度
ros2 service call /robot/set_mode demo_interfaces/srv/SetRuntimeMode '{mode: ROS_CONTROL}'
```

| mode | 调用成功后的行为 |
|---|---|
| `IDLE` | 停车、撤销运动授权、卸载当前模式，语音/底盘常驻 |
| `FOLLOW` | 启动真实 RGB-D、Pose、ReID/VitTrack、跌倒检测，认领主人并开启跟随 |
| `PERCEPTION` | 启动同一感知链，但不认领、不授权运动 |
| `ROS_CONTROL` | 不开相机，接受 `/robot/cmd_vel` 流式前进/转向 |
| `RELATIVE` | 不开相机，加载 `/demo/*` 距离/角度闭环接口，按每个动作授权 |
| `FACE` | 预热面诊原图采集，不调用模型 |
| `FACE_UPLOAD` | 预热旧面诊上传；现有上传端仍返回 404 |
| `PCB / HELMET / TONGUE` | 预热对应采集/推理链，等待具体检测调用 |

`/robot/set_mode` 类型为 `demo_interfaces/srv/SetRuntimeMode`。请求字段：`mode`、`target_id`；响应字段：`success`、`mode`、`detail`。只有 `success=true` 才表示该次切换完成；失败时应读 detail 和 `/control/state`，不要直接发送运动请求。

`/follow/start`、`/follow/stop`、`/robot/stop` 均为 `std_srvs/srv/Trigger`，返回 `success/message`。`/follow/stop` 与全模式停止等价；可随时在另一个已加载环境的终端调用 `/robot/stop`。

未知模式、非法目标编号拒绝且不改变当前任务。跟随启动包含最多 50 秒模型启动、20 秒认领及运动就绪等待；无人时明确失败并回到停车状态。返回成功才表示已启动。重复 FOLLOW 调用会先停车、卸载、重新认领；同一空闲采集模式可复用预热资源。模式切换会重建感知进程，旧 target_id 不应作为跨模式持久身份；常规使用 `target_id: ''` 中央认领。

跟随沿用本轮用户确认的固定机位配置：1.3 m、最高 0.2 m/s、角速度最高 0.5 rad/s、Ackermann 最小软件半径 0.4 m、ReID 丢失找回、主人 20 秒保留、跌倒暂停与当前快速下降规则。三项几何确认仍是原现场临时配置，不代表正式测距/机械半径标定。

## 3. ROS 前进、后退与转向

`ROS_CONTROL` 下发布 `/robot/cmd_vel`，类型 `geometry_msgs/msg/TwistStamped`，`header.frame_id=base_link`，时间戳必须是当前 ROS 时间。推荐 20 Hz 持续发送；输入超过 0.2 秒未更新、未来/过期时间戳、不支持轴、NaN 均输出零。退出再进入不复用旧请求。正 linear.x 前进，负值后退；正 angular.z 左转，负值右转。仅支持 linear.x/angular.z，最大 ±0.2 m/s、±0.5 rad/s，并按 `|w| <= |v| / 0.4` 限制转弯；零车速不原地转向。

```bash
# 在 ROS_CONTROL 成功后，示例：低速前进；Ctrl-C 停止发布后会制动
ros2 topic pub -r 20 /robot/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: now, frame_id: base_link}, twist: {linear: {x: 0.1}}}"
```

同一时间只运行一个速度发布命令。以下示例分别执行，结束后 Ctrl-C；切换到下一条前先确认已停车：

```bash
# 后退
ros2 topic pub -r 20 /robot/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: now, frame_id: base_link}, twist: {linear: {x: -0.1}}}"
# 前进左转（弧线）
ros2 topic pub -r 20 /robot/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: now, frame_id: base_link}, twist: {linear: {x: 0.1}, angular: {z: 0.15}}}"
# 前进右转（弧线）
ros2 topic pub -r 20 /robot/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: now, frame_id: base_link}, twist: {linear: {x: 0.1}, angular: {z: -0.15}}}"
# 停车并退出控制模式
ros2 service call /robot/stop std_srvs/srv/Trigger '{}'
```

Python/C++ 调用方需要在每帧填写当前时间戳，并在 ROS_CONTROL 成功后才开始发送。消息在授权前生成，即使授权后重发也不会作为新控制请求使用。

本入口的 `/chassis/cmd_vel` 由调度器独占；外部程序使用 `/robot/cmd_vel`。勿同时运行旧 `roscar_api api.launch.py`、motion_adapter、整车入口或自行发布最终 `/cmd_vel`。重复输入发布者触发停车。

## 4. 按距离与角度执行动作

需要按距离/角度闭环时，先切 `RELATIVE`，再调用原 `/demo/move_distance`、`/demo/turn_angle` 或 `/demo/move_relative`；它们复用原里程计闭环和每任务授权，任务结束 guard 回 IDLE；相对控制器保持就绪，直到显式切换/停车时卸载。详细控制规则见 [相对运动 ROS 接口](../demo/相对运动ROS接口.md)。`RELATIVE` 与流式速度控制互斥；Ackermann 转向是弧线，存在位移。

```bash
ros2 service call /robot/set_mode demo_interfaces/srv/SetRuntimeMode '{mode: RELATIVE}'
# 前进 0.3 米；distance_m 为负数时后退，speed_mps=0 使用配置默认值
ros2 service call /demo/move_distance demo_interfaces/srv/MoveDistance \
  '{distance_m: 0.3, speed_mps: 0.1}'
# 前进弧线左转 30 度；负角度为右转
ros2 service call /demo/turn_angle demo_interfaces/srv/TurnAngle \
  '{angle_deg: 30.0, yaw_rate_rps: 0.15, reverse: false}'
# 取消当前相对动作
ros2 service call /demo/motion_cancel std_srvs/srv/Trigger '{}'
# 卸载相对控制器
ros2 service call /robot/stop std_srvs/srv/Trigger '{}'
```

MoveDistance 返回 `success/status/message/travelled_m`；TurnAngle 另返回 `turned_deg`。状态为 SUCCEEDED、REJECTED、ABORTED、CANCELED、TIMEOUT。接口是阻塞调用，需要反馈或 action 取消时使用 `/demo/move_relative`（`demo_interfaces/action/MoveRelative`）：

```bash
ros2 action send_goal /demo/move_relative demo_interfaces/action/MoveRelative \
  '{kind: MOVE, distance_m: 0.3, angle_deg: 0.0, speed: 0.1, timeout_s: 10.0}' --feedback
```

当前配置单次距离上限 2 m、角度上限 180°、速度上限 0.2 m/s；`speed=0` 使用默认速度。车辆不能原地旋转，转角任务会产生位移。里程计与机械半径仍未完成卷尺/地面标记标定，返回 SUCCEEDED 只表示达到当前里程计容差。

## 5. 面诊原图、PCB、安全帽与舌苔

以下公共服务常驻，调用会自动停车、切入对应模式、启动原始 RGB 和拍照执行器，完成后卸载并回 IDLE。不需要先调用 set_mode。也可先 set_mode 预热该模式，然后调用服务。只允许一个拍照/推理任务；新的模式请求取消旧任务，等待旧 cgroup 与停车租约释放后再启动。

| 服务 | 类型 | 能力 |
|---|---|---|
| `/face/capture` | `demo_interfaces/srv/CaptureFace` | 面诊采集并直接返回原始 JPEG；success/captured 仅表示采集成功，明确 model_not_integrated |
| `/capture/face` | `capture_task_interfaces/srv/RunCapture` | 面诊采集上传，detail 始终标记 `model_not_integrated`；成功只表示上传成功 |
| `/inspect/face` | `capture_task_interfaces/srv/RunInspection` | 立即返回 unavailable / model_not_integrated，不伪造面诊结论 |
| `/inspect/pcb` | `capture_task_interfaces/srv/RunInspection` | 原 PCB 网关推理 |
| `/inspect/helmet` | 同上 | 原安全帽网关推理 |
| `/inspect/tongue` | 同上 | 原舌苔网关推理 |

```bash
ros2 service call /face/capture demo_interfaces/srv/CaptureFace \
  "{task_id: '', announce: '', prepare_sec: -1.0, timeout_sec: 60.0}"
ros2 service call /inspect/pcb capture_task_interfaces/srv/RunInspection \
  "{task_id: '', announce: '', prepare_sec: -1.0, timeout_sec: 60.0}"
```

安全帽和舌苔使用相同请求字段，分别调用 `/inspect/helmet`、`/inspect/tongue`：

```bash
ros2 service call /inspect/helmet capture_task_interfaces/srv/RunInspection \
  "{task_id: 'helmet-001', announce: '', prepare_sec: -1.0, timeout_sec: 60.0}"
ros2 service call /inspect/tongue capture_task_interfaces/srv/RunInspection \
  "{task_id: 'tongue-001', announce: '', prepare_sec: -1.0, timeout_sec: 60.0}"
# 取消时 task_id 必须与该门面正在执行的任务一致；也可使用 /robot/stop 全局停车取消
ros2 service call /inspect/cancel capture_task_interfaces/srv/CancelCapture "{task_id: 'tongue-001'}"
```

| 请求字段 | 含义 |
|---|---|
| `task_id` | 空字符串自动生成；自填 ID 使用 1–64 位字母、数字、下划线或连字符 |
| `announce` | 空字符串使用默认提示；非空时使用符合原播报校验规则的提示文本 |
| `prepare_sec` | `-1.0` 使用配置默认等待；`0.0` 不额外等待；非负值最大 30 秒 |
| `timeout_sec` | `0.0` 使用默认任务时限；正值最大 600 秒；不包含模式启动和退出等待 |

返回值区分：

- `/face/capture`：`success/status/detail/task_id/capture_id/image`；成功为 `captured`，`image` 是带采集时间戳、`format=jpeg` 的 `sensor_msgs/CompressedImage` 原图，不是标注图。`model_not_integrated` 是能力说明，成功采集也会保留此标记。
- `/inspect/*`：`success/status/detail/task_id/capture_id/result_json/annotated_image`；`analyzed` 表示完成分析，`result_json` 是 JSON 字符串，`annotated_image` 为返回的 JPEG 标注图。`retake` 表示需要重拍，success=false，不代表已经完成有效分类。
- `/capture/face`：旧上传响应为 `success/status/detail/task_id/capture_id/upload_json`，上传成功为 `uploaded`。它不返回原图、不做面诊分类。
- 取消返回 `success/detail`；detail 可为 cancel_requested、not_active、not_owner。失败/超时/取消后不能将残留结果当作成功分析。

CLI 会展开图像字节数组，程序接入建议直接读取响应中的字节，按需保存。例如在调用方接收 `/face/capture` 的成功响应 `response` 后：

```python
from pathlib import Path
if response.success and response.status == 'captured':
    Path('face.jpg').write_bytes(bytes(response.image.data))
```

这只在调用方主动保存；车端执行器默认不保存 JPEG。此示例只展示响应处理，不会自行发起 ROS 调用。

面诊推荐使用 `/face/capture`，不依赖上传端、不调用推理、不在车端保存 JPEG。`FACE` 预热此采集模式；`FACE_UPLOAD` 是可选旧上传模式。旧 `/capture/face` 仍保留上传契约，但现有 5011 接收端实测返回 `upload_http_404`（未确认接收 ROS 自建任务 ID），该上传能力未验收，不影响直接 ROS 采集。

空 task_id 自动生成；prepare_sec=-1 沿用原配置，0 为无额外准备等待。等待唯一 TTS 播报完成后才采集新鲜原始 RGB；robot 安全配置使用 TaskHold 和新鲜静止里程计，租约释放不会续跟。结果保留原 status/result_json/annotated_image 契约。超时、取消、相机/网关/播报/停车失败明确返回失败。`/capture/cancel`、`/inspect/cancel` 复用 CancelCapture，只取消本门面拥有且 task_id 匹配的任务；`/robot/stop` 为全模式停止。面诊采集原图通过返回的 image 交给调用者；可选上传不在项目保存；其他模式也默认不保存图像。直接使用 `/capture_task/run` 是内部接口，启动的执行器只接受当前模式的 task_type。

网关地址和 token_file 从板端私有配置读取；部署默认沿用现有网关和上传接收端。缺模型或远端不可达会返回实际错误，不降级模拟。语音唤醒、ASR 与 `/voice/say`（SayText）、`/voice/speak`（SpeakText）始终复用原唯一队列，语音 STOP 同时取消待启动模式。此入口未把方向口令改成自动运动授权；ROS 运动由上述显式调用启动。

## 6. 常开语音与文字播报

普通文字播报调用 `/voice/say`，类型 `roscar_interfaces/srv/SayText`，不经过运动意图解析：

```bash
ros2 service call /voice/say roscar_interfaces/srv/SayText "{text: '你好，我现在处于待机模式。'}"
```

请求只有 `text`；响应为 `success/status/utterance_id/message`。`success=true, status=completed` 表示合成及 ALSA 播放完成。文字播报不打开跟随，也不授予运动权限。不要另启一套 TTS。

需要 action 反馈/按 goal 取消时使用同一队列的 `/voice/speak`：

```bash
ros2 action send_goal /voice/speak execution_mode_interfaces/action/SpeakText \
  "{utterance_id: 'external-speech-001', text: '准备开始检测。'}" --feedback
```

此 action 返回 `status/detail`，反馈字段为 `phase`；completed 同样表示播放完成。跟随、主人阶段和跌倒提示使用现有唯一队列。当前 demo 语音急停配置沿用现场设置：口头急停转普通 STOP，显式 `/control/estop` 接口仍保留。

收音恢复（2026-10-20）：麦克风启动占用、录音进程退出或连续 2 秒无音频时，采集器关闭旧进程并按 1、2、4、5 秒上限重试；不启动多个录音所有者、不发布伪造音频。重连时序号留出缺口，旧半句作废；不改变现有运动权限或清除急停/故障锁存。通用 KWS 的失效锁存也不会被本修复解除。

判断收音是否恢复应检查真实数据，不能只看 service active 或 topic 存在：

```bash
ros2 topic hz /voice/audio
```

正常为约 25 帧/秒（16 kHz、单声道、每帧 1280 字节）；采集/播报的完成与 ASR 内容准确率分别验证。本轮实车短暂占用解除后约 2.1 秒恢复，单独结束录音子进程后也自动恢复，核心无需重启；播报期间音频持续采集。

## 7. 状态、停止与故障处理

| 话题 | 类型 | 用途 |
|---|---|---|
| `/robot/state` | `std_msgs/msg/String`，JSON | 模式生命周期；不等于底盘授权状态 |
| `/control/state` | 同上 | 最终 command_mode、authority_state、ready、reason、fall_paused、TaskHold |
| `/control/follow_state` | 同上 | 当前跟随原因、目标、距离、偏角、速度请求；常驻控制器可能在待机时也发布 |
| `/perception/owner_state` | `person_interfaces/msg/OwnerState` | 未认领、跟踪、丢失、找回、释放；仅感知模式运行时有数据 |
| `/perception/fall_alarm` | `person_interfaces/msg/FallAlarm` | valid、stage、track_id、detail；NONE=0、SUSPECTED=1、CONFIRMED=2 |
| `/perception/target_state` | `person_interfaces/msg/TargetState` | 选中目标及位置有效性；不是电机命令 |

```bash
ros2 topic echo /perception/owner_state
ros2 topic echo /perception/fall_alarm
# 普通停车：取消模式并释放资源
ros2 service call /robot/stop std_srvs/srv/Trigger '{}'
# 显式急停：锁存，普通模式切换不会清除
ros2 service call /control/estop std_srvs/srv/Trigger '{}'
```

有意复位前应先检查 `/control/state` 的故障原因。`/control/reset_estop`、`/control/reset_fault` 均为 Trigger，复位成功仍不代表已授权；排除原因后再显式调用所需模式。不要用反复启动模式代替故障处理。具体锁存与授权规则沿用 [既有接口手册](../ROS接口使用文档.md)。

| detail / 现象 | 处理 |
|---|---|
| `booting` | 等 `/robot/state.phase=IDLE` 后重试 |
| `owner_claim_failed:*` | 人在画面中央站好后重新调用 FOLLOW；检查画面和主人状态 |
| `motion_not_ready:*` / `authorize_failed:*` | 查 `/control/state`，解决目标、请求时效、配置确认或锁存问题 |
| `old_resources_not_released:*` | 旧进程/发现信息/租约尚未清理，保持停车，检查 module 和 camera 日志 |
| `previous_operation_still_stopping` | 上一任务仍在清理；等 IDLE 后再调用，不并发重试 |
| `capture_start_timeout` | 检查相机、TTS 和执行器日志，以及唯一发布者是否成立 |
| `model_not_integrated` | 面诊模型未接入；使用 `/face/capture` 获取原图 |
| `upload_http_404` | 旧上传端尚未确认接受 ROS 自建任务；直接采集不受影响 |
| `retake` | 根据 result_json 的重拍提示，重新发起检测 |

## 8. 自启动、日志与资源管理

`/robot/state` 为 JSON：mode、phase、detail、generation、recording=false。phase 包含 BOOTING/IDLE/STARTING/ACTIVE/STOPPING/ERROR。最终运动状态仍读 `/control/state`，跟随细节读 `/control/follow_state`，主人和跌倒沿用原 `/perception/*` 契约。

开机 `roscar-robot.service` 只启动常驻核心，完成 STOP 后进入 IDLE/DISARMED；不启动相机、推理、跟随或录像。重型模式归属唯一 `roscar-demo-module.service`，整个 cgroup 绑定核心服务；模式切换等待旧进程全部退出、相机发布者消失、TaskHold 释放。核心退出连带停止模块，进程故障不重启授权。保留显式 ESTOP 和所有非目标故障锁存；普通停止不清锁存。

FOLLOW/PERCEPTION 才加载 Pose/ReID/VitTrack；采集模式只开 RGB，关闭深度流、配准和本机检测模型；ROS_CONTROL/RELATIVE 不开相机。不调用 empty_cache 伪装卸载，直接终止模型进程释放 CUDA 内存。语音持续占用的内存是常驻基线。原 `roscar-capture-inference.service` 和旧全栈/适配服务必须停用，避免额外执行器和相机所有者。

部署、实测数据及验证边界见 WORKLOG 的 2026-10-19 按需模式记录。模式集成验证不替代新一轮实车移动、检测准确率或绝对测距验收。

新增 `CaptureTask.Result.original_image` 仅供 collect 模式使用，与 annotated_image 区分；同步重编译 capture_task_interfaces 和所有客户端。旧 overlay 不得与新接口混用。通用 upload/infer 默认与原服务类型保持不变。

采集/推理门面超时预算从执行器接收任务时开始，另有模式停车与资源启动等待；程序调用方的外层等待必须覆盖任务时限及模式启动、清理时间；默认 60 秒任务可将外层等待先设为 300 秒，再按实际耗时调整。外部取消优先调用 `/robot/stop` 或带 task_id 的 cancel；不要将客户端超时误认为服务端已取消。

```bash
# 服务状态与日志；无需为日常模式切换重启核心
systemctl --user status roscar-robot.service --no-pager
systemctl --user status roscar-demo-module.service --no-pager
tail -n 50 /home/wheeltec/ROSCAR-current/artifacts/demo-runtime/manager.log
ls /home/wheeltec/ROSCAR-current/artifacts/demo-runtime/module-*.log
```

没有激活模式时，`roscar-demo-module.service` 可能显示 inactive 或不存在，这是 transient unit 正常清理后的状态。核心服务 enabled，用户 Linger 保持原配置；服务故障不会自动重启。维护时如确需重启核心，恢复后仍先回待机，必须重新调用模式。接口服务可能需要数秒完成 ROS 发现。

## 9. 本轮验证记录（2026-10-19）

| 检查 | 结果与边界 |
|---|---|
| Jetson Humble 构建 | 五个变更包实际构建通过 |
| 模式/取消/授权回归 | 隔离 ROS 域 228，真实 manager/guard + 合成资源通过；不算真人跟随复验 |
| 既有采集/推理接口 | 14 项、6 项隔离 ROS 回归通过；相对动作与 launch 回归通过 |
| 面诊直接采集 | 真实车载相机 JPEG 返回，包含时间戳、captured / model_not_integrated；无模型推理 |
| PCB/安全帽 | 真实采集→网关→分析/标注图→卸载完成；仅空场景接口联通，未验准确率 |
| 舌苔 | 无舌体画面返回 retake，未伪造分类 |
| 语音 | `/voice/say` 完成真实 ALSA 播放并返回 completed |
| 停车与资源释放 | 静止接口测试非零速度=0；完成后相机=0、唯一 /cmd_vel、IDLE/DISARMED |
| 核心故障 | 无运动模式下强制结束核心，子模式退出；再启动只回待机，不自动跟随 |

保留原用户已确认的跟随/跌倒效果；本轮未重新做有位移的跟随或断电重启。旧面诊上传端仍返回 404，使用 `/face/capture` 可独立取得图像。
