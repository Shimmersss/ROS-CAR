我现在有一套运行在 NVIDIA Jetson Orin Nano Super 8GB 上的 ROS 2 小车语音控制系统。请基于现有代码进行分析和优化，不要默认推倒重来，也不要为了“更先进”而盲目替换已经稳定运行的模块。

## 一、当前系统现状

当前语音链路是：

硬件唤醒 → ALSA 录音与音量分段 → Qwen3-ASR 0.6B INT8 本地识别 → 固定中文规则解析 → ROS 控制授权 → motion_guard → 底盘 → 本地 MeloTTS 播报。

环境：

- NVIDIA Jetson Orin Nano Super 8GB
- Ubuntu 22.04
- JetPack 6.2
- CUDA 12.6
- ROS 2 Humble
- Python 3.10
- ASR：Qwen3-ASR 0.6B INT8，sherpa-onnx，CPU 两线程
- TTS：MeloTTS/VITS，sherpa-onnx，CPU 两线程
- YOLO 等视觉模型同时运行
- 麦克风/唤醒设备：XFM-DP-V0.0.18
- USB 音频采集
- `/dev/wheeltec_mic` 接收硬件“小微小微”唤醒事件
- 底盘为阿克曼结构，驱动为 `mini_akm`
- STM32 下位机通过 `/dev/wheeltec_controller` 通讯
- 当前已有 `motion_guard`
- 有 IDLE / EXTERNAL / FOLLOW 等控制模式

当前录音：

- 16 kHz
- 16 bit
- mono
- 40 ms frame
- 约 80 ms 连续超过音量阈值判断开始讲话
- 约 400 ms pre-roll
- 约 1.2 s 连续静音判断讲话结束
- 最长等待起音 30 s
- 单次最长语音 15 s

Qwen3-ASR 当前短句解码约 1.64～2.21 秒，因此加上 1.2 秒句末静音，目前一条命令实际响应可能达到数秒。

当前固定规则已经进行了大量扩展，本机有 1546 项逻辑测试，人工测试集支持指令 96/96 命中，144 条非执行文本未产生动作。

Qwen3-ASR 只负责转写，不负责运动决策。不要让 LLM 直接控制底盘。

---

# 二、总体结论

不要推倒当前架构。

以下模块原则上保留：

- 硬件唤醒
- Qwen3-ASR 0.6B INT8
- sherpa-onnx
- 固定规则 Intent 解析
- ROS 2 架构
- motion_guard
- MeloTTS
- 400 ms 左右的 pre-roll
- “识别不确定则拒绝，不允许 AI 猜测运动动作”的设计原则

当前最需要优化的不是换更大的 ASR，而是：

1. 音频前端和 VAD
2. 急停链路
3. 全局运动授权状态机
4. 阿克曼底盘动作语义适配
5. 持续会话策略
6. 实车语音 benchmark

---

# 三、第一优先级：取消 auto_arm，统一运动授权

检查当前 `start_robot.sh`、FOLLOW、voice_command_router、motion_guard 和相关控制节点。

目前存在 `auto_arm=true`，同时语音节点自己又维护“开始遥控 → ARM”的授权逻辑。

这意味着可能存在多个组件分别决定“车辆是否允许运动”。

需要重构为唯一的 Global Motion Authority / Motion State Machine。

建议状态至少包括：

```text
DISARMED
ARMED_MANUAL
ARMED_FOLLOW
ESTOP
FAULT
```

要求：

- 系统上电默认必须为 `DISARMED`
- 禁止默认 auto-arm
- Voice、FOLLOW、Navigation、Joystick 等模块都不能直接决定最终运动授权
- 所有运动源必须通过统一的 Motion Authority
- “开始遥控”进入 `ARMED_MANUAL`
- “开始跟随”需要确认视觉目标、深度、控制器状态等有效后进入 `ARMED_FOLLOW`
- STOP 可以退出当前运动状态
- ESTOP 必须具有 latch 行为
- ESTOP 后普通“前进”等命令不能自动重新授权
- 必须经过明确的解除停止/重新授权流程

请优先保证这一部分的安全性和状态一致性。

---

# 四、第二优先级：增加独立 Emergency KWS

当前“停止 / 急停”只是普通 ASR 命令：

麦克风 → 分段 → 等待约 1.2 秒静音 → Qwen3-ASR → 固定规则 → STOP

这不是真正的实时急停。

请增加一条和主 ASR 并行的 Emergency Keyword Spotting 通道。

例如识别：

```text
停止
停下
急停
别动
```

可优先考虑 sherpa-onnx KWS。

目标架构：

```text
Mic
 ├─ Normal voice pipeline → VAD → Qwen3-ASR → Rules
 │
 └─ Emergency KWS → ESTOP
```

Emergency KWS 要求：

- 常驻运行
- CPU 优先
- 不等待完整句子
- 不等待 1.2 秒 endpoint
- 不经过 Qwen3-ASR
- 不经过普通 command router 才决定是否停
- 检测到后立即触发最高优先级 stop
- 撤销运动授权
- 输出零速度
- 下游 motion_guard 和底盘 watchdog 仍继续作为额外保护

非常重要：

`/voice/speaking=true` 或 TTS 播放期间，Emergency KWS 不能被暂停。

即使机器人正在自己讲话，用户喊“停止”，仍然必须能够立即被检测。

---

# 五、第三优先级：替换当前纯音量 VAD

当前基于 RMS/音量阈值的起音和静音判断，对移动机器人不够可靠。

因为小车存在：

- 电机
- 舵机
- 风扇
- 齿轮
- 轮胎
- 机械振动
- 环境噪音

这些都可能提高音量，但不代表人在讲话。

请考虑将主语音分段改为 Neural VAD。

优先考虑：

- Silero VAD
- 或 FSMN-VAD

保持现有约 300～500 ms pre-roll。

不要一开始删除现有音量检测，可以作为辅助特征或 fallback，但主 endpoint 应由 Neural VAD 决定。

当前句末静音约 1.2 秒明显偏慢。

建议测试：

```text
300 ms
400 ms
500 ms
600 ms
700 ms
800 ms
```

初始可先从 500～700 ms 尝试。

需要在真实语音中验证：

- 是否截断句尾
- 是否把正常停顿误认为一句结束
- latency P50/P95
- command success rate

不要只追求最低延迟。

---

# 六、先调查 XFM-DP-V0.0.18 音频能力，不要急着更换麦克风

目前系统最终输入是 16 kHz mono，但 XFM-DP-V0.0.18 很可能本身属于多麦阵列设备。

请检查实际 ALSA 设备能力：

- `arecord -l`
- `arecord -L`
- `hw_params`
- 支持的 channel 数
- USB 实际暴露的是 mono 还是 multichannel
- 当前代码指定 `-c 1` 后到底取的是什么数据

重点确定：

### 情况 A

```text
多麦阵列
→ 硬件 DSP
→ Beamforming / NS / AEC
→ 输出处理后的 mono
```

如果是这种情况，当前 mono 是合理的。

### 情况 B

```text
多麦原始音频
→ USB 多通道
→ 软件只取第一个 channel
```

如果是这种情况，目前相当于浪费了麦克风阵列能力，需要重新设计音频前端。

同时检查硬件唤醒模块是否能够输出：

- 唤醒角度
- DOA
- 降噪音频
- 主麦信息
- Beamforming 后音频

如果已有这些能力，应优先复用硬件能力，不要重复在 Orin 上做昂贵算法。

---

# 七、验证 AEC 是否真的有效

目前麦克风和扬声器可能是不同 USB 设备。

不能因为硬件宣称支持 AEC 就默认系统实际已经在做 AEC。

请进行实际测试：

让机器人通过 MeloTTS/扬声器循环播放：

```text
前进
后退
左转
右转
停止
开始跟随
```

同时录制麦克风输入。

检查录音中机器人自己的 TTS 是否仍然非常清晰。

如果非常清楚，说明当前 AEC 不充分或没有 reference。

需要进一步确认：

- XFM-DP 是否有硬件 AEC
- 是否需要播放 reference PCM
- USB 输出链路是否能够提供 reference
- 是否只能通过软件 AEC 实现

最终目标是支持：

```text
机器人正在播报
+
用户说话
→ 用户仍然能够打断机器人
```

也就是 barge-in。

在 AEC 完成前，普通 ASR 可以继续在 TTS 时暂停，但 Emergency KWS 必须一直工作。

---

# 八、暂时不要替换 Qwen3-ASR

当前 Qwen3-ASR 已经：

- 成功运行在车上
- INT8
- CPU 两线程
- sherpa-onnx
- 已和 ROS 主链路集成

目前没有足够证据说明换 Paraformer、Whisper 或其他模型一定会让最终控制效果更好。

因此第一阶段保留 Qwen3-ASR。

优先尝试 Qwen3-ASR hotwords / contextual biasing。

热词建议只加入机器人领域关键字，例如：

```text
前进
后退
左转
右转
停止
急停
开始遥控
手动驾驶
跟随
取消跟随
电压
目标
锁定目标
半秒
一秒
两秒
```

不要无节制添加大量热词。

需要测试开启 hotwords 前后：

- robot command CER
- command success rate
- negative samples
- hallucination / insertion
- 空音频输出异常
- 非指令语音误命中率

不要只测普通中文 CER。

---

# 九、暂时保留 Qwen3-ASR 在 CPU，不要贸然抢 GPU

GPU 当前还运行 YOLO 等视觉模型。

Jetson Orin Nano 8GB 的 GPU 和 CPU 共用内存，并且没有适合将 YOLO 完全迁移出去的独立 DLA 路线。

因此不要为了减少几百毫秒 ASR 延迟，就直接把 Qwen3-ASR 长期迁到 GPU。

如果以后需要测试 GPU ASR，请必须同时观察：

- YOLO FPS
- GPU utilization
- shared memory
- latency jitter
- OOM
- 温度
- 整机功耗

ASR 速度不能以牺牲视觉实时性为代价。

---

# 十、重构 Voice Command → 底盘参数的接口

这是当前应该进行的关键架构重构。

目前 voice_command_router 把：

```text
左转
```

直接转换成：

```text
linear.x = 0
angular.z > 0
```

但当前底盘是阿克曼底盘，无法原地转弯。

因此“左转”是一个运动语义，而不是 Twist 参数。

请将架构改为：

```text
ASR
↓
Intent Parser
↓
MotionRequest
↓
Chassis Motion Adapter
↓
Twist / 底盘控制
```

例如：

```text
MotionRequest:
  action: TURN
  direction: LEFT
  style: NORMAL
  duration: 1.0
  speed: NORMAL
```

由：

```text
ackermann_motion_adapter
```

根据当前底盘能力生成：

```text
linear.x > 0
angular.z > 0
```

以后如果换差速底盘，则同样的：

```text
TURN_LEFT
```

可以映射成：

```text
linear.x = 0
angular.z > 0
```

Voice 层不应该知道具体底盘运动学。

---

# 十一、增加 Chassis Capability Validator

例如用户说：

```text
原地左转
```

ASR 和语义解析本身可以正确输出：

```text
TURN
LEFT
IN_PLACE
```

然后由 Ackermann Capability Validator 返回：

```text
UNSUPPORTED
```

TTS：

```text
当前底盘无法原地转向。
```

不要让 parser 为了适配底盘而错误理解用户语义。

语音理解和底盘能力应该分层。

---

# 十二、改造持续语音会话

目前：

```text
小微小微
→ 进入持续会话
→ 无限循环监听
```

长期存在环境语言误触发风险。

建议改为 follow-up window。

例如：

```text
唤醒
↓
听取命令
↓
执行 / TTS
↓
继续监听 5～10 秒
↓
如果没有新语音
↓
退出 active voice session
```

每次成功命令可以刷新 timeout。

不要永久保持 active session。

同时：

“再次说小微小微”不建议作为 toggle，用于“退出会话”。

Wake word 的用户直觉应该始终是：

```text
我要叫醒机器人
```

再次唤醒可以：

- 刷新 session
- 打断当前普通 TTS
- 重新进入 listening

退出会话应该使用独立命令，例如：

```text
不用了
退出语音
休眠
```

STOP / ESTOP 则独立负责运动停止。

---

# 十三、重新定义“一下 / 一点 / 稍微”

当前：

```text
向前挪一下
```

仍然使用默认 2 秒。

这不符合自然语言直觉。

建议增加：

```text
MICRO_MOVE
```

或类似语义。

例如：

```text
一下
一点
稍微
挪一下
```

映射到：

- 低速度
- 短 duration

例如先测试：

```text
0.4～0.7 秒
```

具体数值必须通过实车调试确定。

而普通：

```text
前进
```

才使用默认 duration。

---

# 十四、中长期增加基于里程计的距离动作

目前主要按照：

```text
速度 × 时间
```

控制。

长期建议支持：

```text
前进半米
后退 30 厘米
```

解析为：

```text
MOVE_DISTANCE
distance = 0.5 m
```

然后基于 odometry 闭环停止。

不要简单通过：

```text
distance / nominal_speed
```

转换成固定时间，因为实际距离受：

- 电池电压
- 地面
- 载荷
- 打滑
- 电机差异

影响。

这项优先级低于前面的安全和语音优化。

---

# 十五、不要让 LLM 进入底盘主控制链路

当前固定规则设计是正确的。

继续保持：

```text
ASR
↓
固定 Intent/Slot
↓
Validator
↓
MotionRequest
↓
Motion Authority
↓
motion_guard
```

未来如果需要复杂自然语言：

```text
绕过前面的箱子，然后找到红色瓶子
```

可以增加 LLM fallback。

但 LLM 只能生成严格结构化数据，例如：

```json
{
  "steps": [
    {
      "action": "avoid",
      "target": "box"
    },
    {
      "action": "find_object",
      "class": "bottle",
      "color": "red"
    }
  ]
}
```

必须经过：

```text
JSON Schema
→ Validator
→ Capability Check
→ Planner
```

LLM 绝对不能直接发布 `/cmd_vel`，也不能直接决定 PWM、电机速度或绕过安全状态机。

---

# 十六、建立真实 Robot Speech Benchmark

当前 1546 个 parser 测试非常有价值，继续保留。

但接下来测试重点必须从：

```text
Text → Parser
```

扩展为：

```text
真人语音
→ 麦克风
→ VAD
→ ASR
→ Parser
→ Motion Request
```

建议记录：

- Wake FAR
- Wake FRR
- VAD false start
- VAD truncation
- ASR CER
- Robot keyword accuracy
- Command Success Rate
- Dangerous False Action Rate
- STOP latency P50
- STOP latency P95
- Normal command latency P50/P95
- TTS self-trigger rate
- Moving-noise command success rate

其中最重要的指标不是普通 ASR CER，而是：

```text
Dangerous False Action Rate
```

以及：

```text
STOP latency P95
```

---

# 十七、测试数据必须覆盖实车噪声

至少测试：

```text
1. 小车静止 + 安静环境
2. 风扇运行
3. 电机运行
4. 小车直行
5. 小车转弯
6. 舵机动作
7. 用户距离 0.5 m
8. 用户距离 1.5 m
9. 用户距离 3 m
10. 用户背对机器人
11. 多人讲话
12. 电视播放中文
13. 机器人自己播放 TTS
14. TTS 过程中用户喊停止
15. 小车运动过程中用户喊停止
```

尤其需要采集真实机器人运行时的音频。

不要只拿电脑播放 wav 测 ASR。

---

# 十八、ASR 替换属于第二阶段实验，而不是当前主任务

等完成：

- Neural VAD
- Emergency KWS
- XFM 麦阵列调查
- AEC 调查
- Ackermann adapter
- Motion Authority
- benchmark

之后，再对比：

```text
A：Qwen3-ASR 0.6B INT8
B：Streaming Zipformer
C：Streaming ASR + Qwen3 fallback
```

重点比较最终：

```text
Command Success Rate
Latency
False Action Rate
CPU/GPU/Memory
```

而不是只比较公开数据集 WER。

如果轻量 streaming ASR 对机器人几十个核心命令已经足够准确，可以考虑：

```text
Streaming ASR
→ 高置信度简单指令直接执行

低置信度 / 长句
→ Qwen3-ASR 二次识别
```

但这属于后续优化。

---

# 十九、建议实施顺序

请严格按下面优先级推进，不要一上来大范围重构所有模块：

### P0：安全与整车控制

1. 查清并移除不合理的 `auto_arm=true`
2. 建立统一 Global Motion Authority
3. 增加 ESTOP latch
4. 增加独立 Emergency KWS
5. 确认 TTS 时 Emergency KWS 仍然工作

### P1：语音前端

6. 音量 VAD → Neural VAD
7. 保留 pre-roll
8. 优化 endpoint latency
9. 调查 XFM-DP 的真实 channel / beamforming / NS / DOA 能力
10. 验证 AEC

### P2：运动语义架构

11. Voice Parser 不再直接生成底盘 Twist
12. 新增 MotionRequest
13. 新增 Ackermann Motion Adapter
14. 新增 Capability Validator
15. 修复左右转实际执行

### P3：识别质量

16. Qwen3 加少量 robot hotwords
17. 建立真人实车语音 benchmark
18. 测各种噪声环境
19. 测 STOP P95 latency
20. 测 Dangerous False Action Rate

### P4：体验

21. 无限持续会话 → 5～10 秒 follow-up window
22. Wake word 不再作为 toggle
23. 增加“一下 / 一点 / 稍微”的 MICRO_MOVE
24. 后续支持 odometry 距离控制

### P5：模型实验

25. 最后才评估 Qwen3 是否需要替换
26. 对比 Streaming Zipformer / Qwen3 / 双 ASR

---

# 二十、最终目标架构

最终希望逐步演进为：

```text
                    XFM-DP 麦克风阵列
                           │
                 Beamforming / NS / AEC
                           │
             ┌─────────────┴─────────────┐
             │                           │
             ▼                           ▼
       Emergency KWS                 Neural VAD
       停止 / 急停                       │
             │                           ▼
             │                       Ring Buffer
             │                           │
             │                           ▼
             │                      Qwen3-ASR
             │                   + robot hotwords
             │                           │
             │                           ▼
             │                    Text Normalizer
             │                           │
             │                           ▼
             │                   Intent / Slot Rules
             │                           │
             │                           ▼
             │                     MotionRequest
             │                           │
             │                           ▼
             │                 Chassis Capability
             │                           │
             │                           ▼
             │                 Ackermann Adapter
             │                           │
             └──────────────┐            │
                            ▼            ▼
                     Global Motion Authority
                            │
                            ▼
                       motion_guard
                            │
                            ▼
                         /cmd_vel
                            │
                            ▼
                       mini_akm / STM32
```

请以“在现有项目上渐进修改、保证每一步都能独立测试和回滚”为原则。

不要为了重构而重构。

首先阅读现有代码、launch 文件、配置和日志，明确每个模块当前真实行为，然后按上述 P0 → P5 顺序提出具体修改方案。

每项修改请给出：

1. 当前问题
2. 修改目标
3. 涉及的文件/节点
4. 推荐实现方式
5. 是否改变 ROS topic/service/interface
6. 风险
7. 测试方法
8. 验收标准

对于涉及安全的改动，例如 ESTOP、Motion Authority、auto_arm 和 motion_guard，优先保证 fail-safe：

```text
任何异常 → 零速度 / DISARMED
```

而不是尝试继续运动。

除非通过实车 benchmark 证明现有 Qwen3-ASR 是主要瓶颈，否则不要优先替换 ASR 模型。