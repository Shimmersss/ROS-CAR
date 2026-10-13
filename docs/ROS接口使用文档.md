# ROS-CAR 接口使用手册

更新日期：2026-10-13。本文以本地 `main` 的当前代码为准，包含新合并的拍照 ROS 接口。它说明软件如何接入；合并代码不等于已经更新小车上的程序。

## 1. 先选你需要的接口

| 你想做的事 | 应调用的接口 | 返回什么 |
|---|---|---|
| 拍面部、舌体、安全帽场景或电路板并上传 | `/capture/face`、`/capture/tongue`、`/capture/helmet`、`/capture/pcb` | 等任务结束，返回上传成功或失败及回执 |
| 取消自己通过上述服务发起的拍照 | `/capture/cancel` | 是否已请求取消；最终结果由原拍照调用返回 |
| 让小车读出一段文字 | `/voice/say` | 等合成与播放结束，返回完成或失败 |
| 订阅检测框、锁定目标的位置 | `/perception/detections`、`/perception/target_state` | 持续发布的 ROS 消息 |
| 切换控制来源、明确授权、停车 | `/control/set_mode`、`/control/authorize`、`/control/stop` | 控制服务的执行结果 |
| 外部程序发送速度请求 | `/chassis/cmd_vel` | 请求话题；最终速度由唯一 `motion_guard` 决定 |
| 读取里程计、IMU、电压或雷达 | `/odom`、`/imu/data_raw`、`/PowerVoltage`、`/scan` | 需要对应硬件驱动已启动 |

这里有两套通信方式：

- **ROS 接口**给本机或局域网内的 ROS 2 程序使用，本文说明它们的调用方法。
- **HTTP 接口**给云端服务使用。云端下发任务、小车上传 JPEG 的协议见[云端拍照任务接口协议](云端拍照任务接口协议.md)。服务器上提供的文档下载链接只是静态文件地址，不是拍照业务接口。

拍照接口只负责“播报、准备、拍照、上传”。`/capture/tongue` 不返回舌苔分类或诊断；语音口令“舌苔检测”走另一套 `tongue_mode` 流程，两者共享相机和播报资源并互斥。

## 2. 调用前先准备环境

### 2.1 构建与加载

在项目根目录执行。服务端和调用端都要有对应的 ROS 2 Humble 消息定义；Python 客户端使用与 `rclpy` 兼容的解释器。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
# 只构建本文的 API、拍照模块及其依赖；构建产物统一位于 ros2_ws 下
bash scripts/build_ros.sh --packages-up-to roscar_api capture_task
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=182
export ROS_LOCALHOST_ONLY=1
```

`ROS_LOCALHOST_ONLY=1` 适合单机调试。电脑通过局域网调用小车时，两端使用相同域号和接口版本，并设为 `ROS_LOCALHOST_ONLY=0`；允许 DDS 通信，保持时钟同步。域号 182 是项目常用值，必须与正在运行的服务一致。

Foxglove 的 WebSocket 连接只负责展示，不能替代 `rclpy` 或 `ros2 service call`。Windows 没有 ROS 环境时，可以通过 SSH 在已经安装 ROS 的小车终端调用。

### 2.2 服务不会因构建完成而自动启动

| 启动入口 | 会启动什么 | 前提与默认行为 |
|---|---|---|
| `ros2 run capture_task api_node` | 仅拍照 ROS 服务节点 | 复用已有 `/capture_task/run`；不新启执行器、相机或 TTS |
| `ros2 launch capture_task capture_task.launch.py with_ros_api:=true` | 拍照执行器、云端 bridge、ROS 服务节点 | 默认 `bench`；相机与唯一 TTS 需另行提供；可设 `with_bridge:=false` |
| `ros2 launch roscar_api speech_api.launch.py` | 仅 `/voice/say` 服务节点 | 默认复用已有 `/voice/speak` |
| `ros2 launch roscar_api api.launch.py` | 一个 `motion_guard` | 默认 IDLE、运动关闭，不自动启相机、底盘、语音、拍照或跟随器 |

按需要选择入口。已有整机入口包含的节点不能重复启动，尤其是 `motion_guard`、相机和 TTS。

**不能把“默认控制入口”和“bench 拍照入口”简单同时运行：**前者会发布 `/cmd_vel`，后者要求该话题的发布者为零。接有底盘或已有 guard 时，拍照必须选择 `robot` 配置，见下节。

### 2.3 确认接口确实存在

```bash
ros2 service list -t
ros2 action list -t
ros2 interface show capture_task_interfaces/srv/RunCapture
ros2 interface show roscar_interfaces/srv/SayText
```

找不到服务时，先确认相应节点已启用、overlay 已加载、域号和 DDS 网络一致。不要先反复重发业务请求。

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

整机脚本的开关为 `WITH_CAPTURE_TASKS=true CAPTURE_ROS_API=true`，默认均为 false；后者不能脱离前者开启。接底盘还必须显式 `WITH_FOLLOWER=true` 以复用唯一 guard；该入口同时启动跟随器，但不会自动开启跟随或运动。整机配置见[实现与验收](云端拍照任务实现与验收.md)。

### 3.2 四个服务怎么选

四个服务都使用 `capture_task_interfaces/srv/RunCapture`，选择服务名就选择了任务类型，不必再传 `task_type`。

| 服务名 | 拍什么 | 默认准备时间 | 默认候选帧数 | 图像宽度质检门槛 |
|---|---|---:|---:|---:|
| `/capture/face` | 面部 | 3 秒 | 3 | 640 像素 |
| `/capture/tongue` | 舌体 | 2 秒 | 3 | 640 像素 |
| `/capture/helmet` | 安全帽场景 | 1 秒 | 1 | 640 像素 |
| `/capture/pcb` | 电路板 | 3 秒 | 5 | 1280 像素 |

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

默认入口只启唯一 guard：`command_mode=IDLE`、运动关闭、不自动授权。相机、底盘、感知、跟随器和雷达分别由 `with_perception / with_chassis / with_follower / with_radar` 显式开启，全部默认 false。

`with_chassis:=true` 还须显式填写已核对的 `serial_port` 和 `car_mode`；本项目实物按 `mini_akm` 配置。厂商底盘/雷达工作区分别由 `scripts/build_chassis.sh`、`scripts/build_radar.sh` 构建，不取消 `COLCON_IGNORE`。

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

正式语音入口使用共享录音、Qwen3-ASR、Silero VAD、固定语义规则、常开 KWS 和唯一 MeloTTS。固定规则经 `MotionRequest` 和车辆能力适配后提交请求；`motion_guard` 是最终授权和速度出口。方向口令不会自动授权，不允许 LLM 生成底盘运动控制。

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

YOLO 锁定当前新鲜、带跟踪 ID、最接近图像中央的候选；无候选返回失败，失去目标不会默认悄悄换人。展示图 `/perception/detections_image` 与 Marker 只用于观察，不能替代原始采集图或有效位置契约。

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

## 8. 常见问题排查

| 现象 | 先检查什么 |
|---|---|
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

## 9. 验证范围与进一步阅读

本地拍照 ROS 服务已做 Humble x86_64 编译与合成 RGB、假 TTS、mock 云端联调。过去的真实 USB 相机采集记录另见验收文档；这些不等于本次新增接口已在 Jetson、正式云端或小车上验收。实车运动、制动、标定和实际听感仍按现场要求确认。

- [文字播报 ROS 接口](文字播报ROS接口.md)：播报专用调用手册。
- [云端拍照任务接口协议](云端拍照任务接口协议.md)：给云端开发者的 HTTP 合同。
- [云端拍照任务实现与验收](云端拍照任务实现与验收.md)：启动组合、联调记录与待验收项。
- [TargetState 接口](interfaces.md)：锁定目标消息说明。
- [motion_guard](../ros2_ws/src/motion_guard/README.md)：运动保护实现。
- [工作记录](../WORKLOG.md)：逐次变更和验证证据。
