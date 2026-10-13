# 文字播报 ROS 接口使用手册

更新日期：2026-10-13；对应当前 PR 合并远端 `main` 后的代码。这个接口让程序发送一段文字，交给已有 MeloTTS 队列合成并播放。无需先喊唤醒词，也不会把正文交给语音意图或底盘运动解析。

## 1. 选择哪种调用方式

| 接口 | 类型 | 适合什么情况 |
|---|---|---|
| `/voice/say` | `roscar_interfaces/srv/SayText` | 最常用：发送文字，等播放结果 |
| `/voice/speak` | `execution_mode_interfaces/action/SpeakText` | 需要阶段反馈、自己取消某一次播报 |
| `/voice/tts_text` | `std_msgs/msg/String` topic | 只发送，不需要完成确认 |

三者共用唯一 TTS 队列。不要为每个调用者另启 TTS 节点。拍照模块使用第二种方式，必须等准备提示播放完成后才拍照。

## 2. 启动前提

先在项目根目录构建并加载当前接口：

```bash
source /opt/ros/humble/setup.bash
bash scripts/build_ros.sh --packages-up-to roscar_api
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=1
```

如果通过局域网从另一台电脑调用，两端使用同一域号、相同接口版本，设置 `ROS_LOCALHOST_ONLY=0` 并允许 DDS 通信。小车上实际工作区路径、运行域号以当前服务为准；历史试运行目录见[实机记录](2026-10-12语音舌苔实机试运行.md)，不把历史路径当作新版本已部署的位置。

### 已有 TTS：只加服务节点

```bash
ros2 action list -t  # 确认已有 /voice/speak
ros2 launch roscar_api speech_api.launch.py
```

该 launch 默认 `with_tts=false`，只启动 `/voice/say`；已有 TTS 必须启用 `enable_speech_action=true`。整机或语音试运行入口已提供 `/voice/say` 时，直接调用，不要再启动第二个服务节点。

B/C 整机 `scripts/start_project.sh` 在 `WITH_VOICE=true` 且 `VOICE_TTS_ENABLED=true` 时，已启用 `/voice/speak` 并同时启动 `/voice/say`，不必另外运行本节 launch。C 历史部署使用 `/home/wheeltec/ROSCAR-c/install/setup.bash` 和域号 183；调用时必须核对当前服务，不能套用本机示例域号。

### 尚无 TTS：独立启动

仅在没有任何 TTS 节点时使用：

```bash
ros2 launch roscar_api speech_api.launch.py with_tts:=true
```

前提是本机已经准备 `sherpa_onnx` 等离线依赖、MeloTTS 模型与 ALSA 播放设备。模型默认从 `/home/wheeltec/ROSCAR-offline/models/vits-melo-tts-zh_en` 加载；也可以在启动前设置 `ROSCAR_OFFLINE_MODEL_ROOT` 为本机模型根目录。不会自动下载模型。

| launch 参数 | 默认 | 作用 |
|---|---|---|
| `with_tts` | false | 是否额外启动一个离线 TTS；已有 TTS 时保持 false |
| `playback_timeout_sec` | 120.0 | 服务等待本次 action 的秒数，合法范围 1–300 |
| `playback_device` | `plughw:CARD=Device,DEV=0` | `with_tts=true` 时使用的 ALSA 设备，不会改动已有 TTS 的设备 |

独立播报入口不启动底盘、录音或 KWS，也不自动授权运动。

## 3. 最简单的调用

另一个已加载相同 ROS 环境的终端：

```bash
ros2 service call /voice/say roscar_interfaces/srv/SayText \
  "{text: '你好，现在开始拍照。'}"
```

请求只有一个字段：`text`。正文去掉空白后须非空，原字符串最多 300 字符；超长直接拒绝，不截断后假报成功。命令行中的引号按 YAML 规则转义，程序调用可直接设置字符串。

服务会等待合成与 ALSA 播放结束，然后返回：

```yaml
success: true
status: completed
utterance_id: say-<本次唯一编号>
message: Playback completed
```

| 响应字段 | 含义 |
|---|---|
| `success` | 只有 action 成功且 status=completed 才为 true |
| `status` | 业务状态，见下表 |
| `utterance_id` | 服务生成的本次播报 ID；输入无效或服务 busy 时可能为空 |
| `message` | 具体完成说明或错误原因 |

| status | 含义 / 建议 |
|---|---|
| `completed` | 软件合成与播放完成；现场是否能听到仍需确认 |
| `invalid_text` | 空白或超过 300 字符；修正文字 |
| `busy` | 前一次 `/voice/say` 调用尚未结束；服务不排队 |
| `unavailable` | 约 2 秒内未找到 `/voice/speak`；检查 TTS 节点和 action 开关 |
| `rejected` | TTS 拒绝接收，可能队列忙或节点正在退出 |
| `failed` | 合成、播放或通信失败；看 message |
| `cancelled` | 被取消，未完整播放 |
| `timeout` | 等待超时，已请求取消自身 goal；不代表已经收到最终取消确认 |

服务自身同时处理一个调用；与其他入口共用的 TTS 队列仍可能排队或拒绝。播放等待上限不包括前面的服务发现等待；默认发现最多约 2 秒，action 等待最多 120 秒。

## 4. Python 调用

```python
import rclpy
from rclpy.node import Node
from roscar_interfaces.srv import SayText

rclpy.init()
node = Node('my_speech_client')
client = node.create_client(SayText, '/voice/say')
try:
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('文字播报服务未启动或 ROS 域不一致')
    request = SayText.Request()
    request.text = '请面向相机，保持姿势。'
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if result is None:
        raise RuntimeError('未收到播报服务响应')
    print(result.success, result.status, result.utterance_id, result.message)
finally:
    node.destroy_node()
    rclpy.shutdown()
```

关闭客户端或客户端自行超时，不会自动撤回服务端收到的播报。`/voice/say` 没有按 ID 取消的公共服务；需要自行取消时，请直接使用下面的 action。

## 5. 获取反馈与取消：SpeakText Action

```bash
ros2 action send_goal /voice/speak execution_mode_interfaces/action/SpeakText \
  "{utterance_id: 'manual-20261013-001', text: '你好，这是一次直接播报。'}" --feedback
```

| 字段所在部分 | 字段 | 含义 |
|---|---|---|
| 请求 | `utterance_id`、`text` | 自己指定本次唯一 ID 和正文 |
| 反馈 | `phase` | 合成或播放阶段 |
| 结果 | `status`、`detail` | completed / cancelled / failed 等结果及原因 |

程序端使用 `rclpy.action.ActionClient` 保存 `send_goal_async()` 返回的 goal handle，取消时调用该 handle 的 `cancel_goal_async()`；等最终结果确认。不要调用全局取消接口去取消别人的播报。

底层最多同时保留两个 action 任务，播放依然按唯一队列顺序执行。每个在途任务使用不同 `utterance_id`。确认 action 成功状态和 `result.status=completed` 后，才把它当成完整播放。

## 6. 不需要结果：topic

```bash
ros2 topic pub --once /voice/tts_text std_msgs/msg/String \
  "{data: '你好，这是一次普通提示。'}"
```

topic 不返回播放确认，超过配置长度会截断，队列满时会丢弃。因此需要“读完后再做下一步”的场景，选择 service 或 action。

## 7. 使用边界与排查

语音整机运行时，播放会暂停普通 ASR，常开 KWS 仍会监听。`/voice/say` 的正文不经过运动解析，但扬声器声音可能重新进入麦克风，包含“停止、停下、急停、别动”等词可能触发紧急监听。此服务不主动过滤这些词；自动准备提示应使用不含保护词的文字。

| 现象 | 排查方向 |
|---|---|
| 看不到 `/voice/say` | 服务节点、overlay、域号、DDS 网络 |
| unavailable | `/voice/speak` 是否存在，TTS 是否开启 action |
| failed | 模型依赖、模型文件、ALSA 设备及权限；查看服务 message 和 TTS 日志 |
| completed 但现场听不到 | 扬声器连接、实际播放设备和音量；软件确认不能代替现场听感 |
| 超时后仍短暂有声音 | 取消请求与执行确认存在时间差；不要据此启动第二个 TTS |

本地模拟运行时回归已通过，真实小车历史播报记录见[实机记录](2026-10-12语音舌苔实机试运行.md)。本轮文档更新与本地主线合并没有重新部署或验证实际扬声器。

更多接口见[ROS 接口使用手册](ROS接口使用文档.md)。
