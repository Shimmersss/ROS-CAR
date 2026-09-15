# 讯飞 WebAPI 语音链路

该包通过讯飞官方 WebSocket API 提供流式语音听写和语音合成，不依赖 AIUI
专有动态库。`deepseek_ros2` 负责文字问答，两者由
`voice_assistant.launch.py` 连接。

## 凭据

在讯飞同一个 WebAPI 应用中开通“语音听写（流式版）”和“在线语音合成”。
启动前从本机私有文件导出变量，不要写入仓库、ROS 参数或启动文件：

```bash
export XFYUN_APP_ID='...'
export XFYUN_API_KEY='...'
export XFYUN_API_SECRET='...'
export DEEPSEEK_API_KEY='...'
```

Orin 上还需要 `alsa-utils` 与 Python `websocket-client`。Ubuntu 22.04 可安装
`alsa-utils python3-websocket`。

## 构建与启动

```bash
cd /home/wheeltec/ROSCAR/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select deepseek_ros2 voice_command_router xfyun_speech --symlink-install
source install/setup.bash
ros2 launch xfyun_speech voice_assistant.launch.py enable_tts:=false enable_buzzer:=false
```

板子启动脚本会额外启动厂商 `wheeltec_mic` 串口节点，但仅使用其硬件唤醒事件，
不会启动厂商离线识别、反馈音频或运动控制。默认唤醒词是“小微小微”：说出
唤醒词、停顿约 1 秒后再说问题，`/awake_flag` 会触发一轮录音。没有唤醒驱动时
也可以手动触发：

```bash
ros2 service call /voice/start_listening std_srvs/srv/Trigger '{}'
```

分层测试：

```bash
# 只测 DeepSeek → TTS
ros2 topic pub --once /voice/asr_text std_msgs/msg/String "{data: '介绍一下你自己'}"

# 只测讯飞 TTS 与声卡
ros2 topic pub --once /voice/tts_text std_msgs/msg/String "{data: '语音合成测试'}"
```

Orin 实测讯飞阵列声卡为 `plughw:CARD=XFMDPV0018,DEV=0`，支持
16 kHz、16-bit、单声道采集，已写入默认配置。换麦克风后再用 `arecord -l`
确认设备，并调整 `capture_device` 和能量阈值。当前链路不发布 `cmd_vel`，也不
启动底盘控制节点；没有扬声器时保持 `enable_tts:=false`。

DeepSeek 最终回答同时发布到 `/voice/assistant_text`，并按 JSON Lines 追加保存到
`/home/wheeltec/ROSCAR/logs/deepseek_responses.jsonl`。可持续查看：

```bash
tail -f /home/wheeltec/ROSCAR/logs/deepseek_responses.jsonl
```

DeepSeek 当前只开放一个 `buzz(duration_ms)` 工具。工具调用必须经过
`voice_command_router` 二次校验，100--2000 ms 以外的请求会被拒绝。GPIO
适配器默认不启动。现已确认蜂鸣器属于下位机，GPIO 适配器仅保留为可选占位，
后续应根据底盘协议另写适配器，不能猜测 Jetson GPIO。
