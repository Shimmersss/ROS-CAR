# 文字播报 ROS 2 接口

分支：`codex/voice-speak-api`。你发送一段文字，小车用已有 MeloTTS 播放。无需先喊唤醒词，不经过 ASR、聊天模型或运动指令解析。

## 最常用的调用方法

在小车终端（或通过 SSH 登录后）执行：

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/ROSCAR-voice-tongue-20261012-066c2ad/install/setup.bash
export ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=0
ros2 service call /voice/say roscar_interfaces/srv/SayText \
  "{text: '你好，我是小车，现在开始播报。'}"
```

将 `text` 换成你想说的话即可。正文含引号时按 YAML 规则转义；程序调用可直接设置字符串字段。每次最多 300 个字符，空白文本或超长文本直接拒绝，不会截断后假报成功。

返回示例：

```text
success: true
status: completed
utterance_id: say-<本次唯一编号>
message: Playback completed
```

服务会等合成和 ALSA 播放结束后返回。`success=true` 表示软件播放完成；扬声器是否可听、音量是否合适仍由现场确认。不同字符的实际读法由 TTS 决定，例如数字会按语音模型规则朗读。

| status | 含义 |
|---|---|
| `completed` | 合成与播放完成 |
| `invalid_text` | 文字为空白或超过 300 字符 |
| `busy` | 上一个 `/voice/say` 调用尚未结束，请稍后重试 |
| `unavailable` | 现有 `/voice/speak` TTS action 未启动或未开启 |
| `rejected` | TTS 拒绝接收，例如播放队列繁忙 |
| `failed` / `cancelled` | 合成/播放失败，或被既有语音取消逻辑中断 |
| `timeout` | 超过等待时间，已请求取消本次播报；不等同于已收到取消完成确认 |

默认播放等待上限 120 秒，超时只取消本次 action，不调用全局取消服务，也不取消其他舌苔播报。服务自身同时接收一个播报任务；与既有任务共用唯一 TTS 队列，可能排队或被拒绝。需要精细控制进度、取消或多客户端调度时，直接使用下面的 action。

## 程序调用

```python
import rclpy
from rclpy.node import Node
from roscar_interfaces.srv import SayText

rclpy.init()
node = Node('my_speech_client')
client = node.create_client(SayText, '/voice/say')
try:
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('文字播报服务未启动')
    request = SayText.Request()
    request.text = '欢迎使用舌苔检测，请面向相机。'
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    print(result.success, result.status, result.message)
finally:
    node.destroy_node()
    rclpy.shutdown()
```

客户端退出不会自动取消已经提交的 ROS service。需要自行取消时，直接用 action，并保存本次 goal handle。

## Action 和 topic

已有 action 作为底层可调用接口公开：

```bash
ros2 action send_goal /voice/speak execution_mode_interfaces/action/SpeakText \
  "{utterance_id: 'manual-001', text: '你好，这是直接通过 action 播报。'}" --feedback
```

每个正在执行的任务使用不同 `utterance_id`。结果 `status=completed` 才表示播放结束；action 客户端可通过本次 goal handle 取消自身任务。底层最多同时保留两个 action 任务，最终仍顺序播放。

只需发送、不关心结果，也可以用已有 topic：

```bash
ros2 topic pub --once /voice/tts_text std_msgs/msg/String \
  "{data: '你好，这是 topic 播报。'}"
```

topic 不返回播放确认，原有 topic 逻辑对超过配置长度的内容会截断、队列满时会丢弃；需要可靠结果请选择 `/voice/say` 或 `/voice/speak`。

## 构建和启动

在项目根目录构建（使用当前工作区的 install 路径）：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths ros2_ws/src --packages-up-to roscar_api \
  --build-base build --install-base install --symlink-install --executor sequential
source install/setup.bash
export ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=0
ros2 launch roscar_api speech_api.launch.py
```

默认只启动 `/voice/say` 桥接节点，复用已运行的 TTS；TTS 必须启用 `enable_speech_action:=true`。车上语音舌苔试运行入口已启用该 action，并加入桥接节点。此入口不调用总运动开关，也不建立新的速度出口。

如果尚未运行任何 TTS，可以独立启动播报和接口：

```bash
export PYTHONPATH=/home/wheeltec/ROSCAR-offline/venv/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}
ros2 launch roscar_api speech_api.launch.py with_tts:=true
```

`with_tts:=true` 只用于没有现有 TTS 的情况。默认模型位置沿用 `/home/wheeltec/ROSCAR-offline/models/vits-melo-tts-zh_en`；播放设备默认 `plughw:CARD=Device,DEV=0`。可通过 `playback_device:=...` 和 `playback_timeout_sec:=120.0` 配置。

电脑直接走 ROS 2 调用时，要安装/构建相同版本的 `roscar_interfaces` 和 `execution_mode_interfaces`，保持同一局域网、域 182、`ROS_LOCALHOST_ONLY=0`，并允许 DDS 发现通信。Windows 上没有 ROS 环境时，通过 SSH 在小车执行调用即可；Foxglove WebSocket 连接不能代替 ROS service 客户端。

整车语音链路正在运行时，播放仍会暂停普通 ASR、保持紧急关键词监听；独立 `with_tts:=true` 入口本身不启动录音或 KWS。正文不会进入运动解析，但扬声器声音可能被麦克风再次收录；包含“停止、停下、急停、别动”等词的播报可能触发紧急监听，现有声学漏检/回声限制仍需现场验证。
