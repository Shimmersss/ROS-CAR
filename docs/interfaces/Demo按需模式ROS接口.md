# Demo 按需模式 ROS 接口

仅适用于 `demo/showcase-20261016`，不替换 main 的通用 API。板端开机入口为 `scripts/run_demo_onboard.sh` → `run_demo_runtime.sh`。语音、文字播报、唯一底盘驱动、跟随控制器及唯一 motion_guard 常驻；相机、Pose、ReID、VitTrack 和检测执行器按需启动。不开录制。

## 模式与调用

先加载 `/opt/ros/humble/setup.bash`、当前部署的 `install/setup.bash`，使用板端私有环境中的 ROS_DOMAIN_ID（182）及 DDS 配置。以下调用在车上执行。

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

统一模式为 `IDLE / FOLLOW / PERCEPTION / ROS_CONTROL / RELATIVE / FACE / FACE_UPLOAD / PCB / HELMET / TONGUE`。未知模式、非法目标编号拒绝且不改变当前任务。跟随启动包含最多 50 秒模型启动、20 秒认领及运动就绪等待；无人时明确失败并回到停车状态。返回成功才表示已启动。重复调用是一次新的切换，会先停车、卸载、重新认领。

跟随沿用本轮用户确认的固定机位配置：1.3 m、最高 0.2 m/s、角速度最高 0.5 rad/s、Ackermann 最小软件半径 0.4 m、ReID 丢失找回、主人 20 秒保留、跌倒暂停与当前快速下降规则。三项几何确认仍是原现场临时配置，不代表正式测距/机械半径标定。

## 前进与转向

`ROS_CONTROL` 下发布 `/robot/cmd_vel`，类型 `geometry_msgs/msg/TwistStamped`，`header.frame_id=base_link`，时间戳必须是当前 ROS 时间。推荐 20 Hz 持续发送；输入超过 0.2 秒未更新、未来/过期时间戳、不支持轴、NaN 均输出零。退出再进入不复用旧请求。正 linear.x 前进，负值后退；正 angular.z 左转，负值右转。仅支持 linear.x/angular.z，最大 ±0.2 m/s、±0.5 rad/s，并按 `|w| <= |v| / 0.4` 限制转弯；零车速不原地转向。

```bash
# 在 ROS_CONTROL 成功后，示例：低速前进；Ctrl-C 停止发布后会制动
ros2 topic pub -r 20 /robot/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: now, frame_id: base_link}, twist: {linear: {x: 0.1}}}"
```

本入口的 `/chassis/cmd_vel` 由调度器独占；外部程序使用 `/robot/cmd_vel`。勿同时运行旧 `roscar_api api.launch.py`、motion_adapter、整车入口或自行发布最终 `/cmd_vel`。重复输入发布者触发停车。

需要按距离/角度闭环时，先切 `RELATIVE`，再调用原 `/demo/move_distance`、`/demo/turn_angle` 或 `/demo/move_relative`；它们复用原里程计闭环和每任务授权，任务结束 guard 回 IDLE；相对控制器保持就绪，直到显式切换/停车时卸载。接口字段见 [Demo 说明](../demo/README.md)。`RELATIVE` 与流式速度控制互斥；Ackermann 转向是弧线，存在位移。

## 面诊与检测

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

面诊推荐使用 `/face/capture`，不依赖上传端、不调用推理、不在车端保存 JPEG。`FACE` 预热此采集模式；`FACE_UPLOAD` 是可选旧上传模式。旧 `/capture/face` 仍保留上传契约，但现有 5011 接收端实测返回 `upload_http_404`（未确认接收 ROS 自建任务 ID），该上传能力未验收，不影响直接 ROS 采集。

空 task_id 自动生成；prepare_sec=-1 沿用原配置，0 为无额外准备等待。等待唯一 TTS 播报完成后才采集新鲜原始 RGB；robot 安全配置使用 TaskHold 和新鲜静止里程计，租约释放不会续跟。结果保留原 status/result_json/annotated_image 契约。超时、取消、相机/网关/播报/停车失败明确返回失败。`/capture/cancel`、`/inspect/cancel` 复用 CancelCapture，只取消本门面拥有且 task_id 匹配的任务；`/robot/stop` 为全模式停止。面诊采集原图通过返回的 image 交给调用者；可选上传不在项目保存；其他模式也默认不保存图像。直接使用 `/capture_task/run` 是内部接口，启动的执行器只接受当前模式的 task_type。

网关地址和 token_file 从板端私有配置读取；部署默认沿用现有网关和上传接收端。缺模型或远端不可达会返回实际错误，不降级模拟。语音唤醒、ASR 与 `/voice/say`（SayText）、`/voice/speak`（SpeakText）始终复用原唯一队列，语音 STOP 同时取消待启动模式。此入口未把方向口令改成自动运动授权；ROS 运动由上述显式调用启动。

## 生命周期、自启动与内存

`/robot/state` 为 JSON：mode、phase、detail、generation、recording=false。phase 包含 BOOTING/IDLE/STARTING/ACTIVE/STOPPING/ERROR。最终运动状态仍读 `/control/state`，跟随细节读 `/control/follow_state`，主人和跌倒沿用原 `/perception/*` 契约。

开机 `roscar-robot.service` 只启动常驻核心，完成 STOP 后进入 IDLE/DISARMED；不启动相机、推理、跟随或录像。重型模式归属唯一 `roscar-demo-module.service`，整个 cgroup 绑定核心服务；模式切换等待旧进程全部退出、相机发布者消失、TaskHold 释放。核心退出连带停止模块，进程故障不重启授权。保留显式 ESTOP 和所有非目标故障锁存；普通停止不清锁存。

FOLLOW/PERCEPTION 才加载 Pose/ReID/VitTrack；采集模式只开 RGB，关闭深度流、配准和本机检测模型；ROS_CONTROL/RELATIVE 不开相机。不调用 empty_cache 伪装卸载，直接终止模型进程释放 CUDA 内存。语音持续占用的内存是常驻基线。原 `roscar-capture-inference.service` 和旧全栈/适配服务必须停用，避免额外执行器和相机所有者。

部署、实测数据及验证边界见 WORKLOG 的 2026-10-19 按需模式记录。模式集成验证不替代新一轮实车移动、检测准确率或绝对测距验收。

新增 `CaptureTask.Result.original_image` 仅供 collect 模式使用，与 annotated_image 区分；同步重编译 capture_task_interfaces 和所有客户端。旧 overlay 不得与新接口混用。通用 upload/infer 默认与原服务类型保持不变。

采集/推理门面超时预算从执行器接收任务时开始，另有模式停车与资源启动等待；调用方应允许至少 120 秒响应窗口。外部取消优先调用 `/robot/stop` 或带 task_id 的 cancel；不要将客户端超时误认为服务端已取消。

## 本轮验证记录（2026-10-19）

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
