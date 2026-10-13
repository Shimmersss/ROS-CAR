# 云端拍照任务：增加对外 ROS 接口的执行方案

> 交给执行 agent。2026-10-13。
> 本机仓库：`/home/smxvjv/Desktop/works/5060wslworks/jiekou/ROS-CAR`
> 基线：`codex/cloud-capture-tasks@08464e1`（已验收；同一分支已带回 WSL 仓库）
> 开工前必读：`AGENTS.md`、`docs/云端拍照任务实现与验收.md`、`docs/云端拍照任务接口协议.md`、`docs/云端拍照任务修复说明.md`、`docs/文字播报ROS接口.md`、`docs/ROS接口使用文档.md`
> 本文件尚未提交，请随新分支第一个提交入库，并在 README 文档索引登记。

---

## 0. 目标与边界

### 0.1 要做的

现在四类拍照任务（face / tongue / helmet / pcb）只能由云端经 HTTP 下发。本次要让**小车上的其他 ROS 2 节点**也能发起这四类任务，使用标准的 ROS 服务。流程不变：

```
ROS 调用 → 小车播报 → 准备 → 采集新鲜图像 → 上传云端 → 返回结果
```

做法参照 `/voice/say`（SayText）：新增一个**门面节点**，自身不执行任何拍照逻辑，只把 ROS 请求转成现有 `/capture_task/run` action 的 goal。

### 0.2 "不影响现在的接口"的具体含义（硬约束）

以下内容的行为和字段必须完全不变：

1. **云端 HTTP 协议 v0**：路径、字段、去重、TTL、取消语义都不变。bridge 代码不改。
2. **`/capture_task/run`**（`CaptureTask.action`）：goal、result、feedback 的字段不增、不删、不改名。
3. **话题**：`/capture_task/state`、`/capture_task/result`、`/capture_task/bridge_state` 的现有字段不变。允许新增字段，但不鼓励。
4. **启动行为**：`capture_task.launch.py` 和 `start_project.sh` 的默认行为不变。新接口默认**关闭**，需要显式打开。
5. **受保护的包**：以下包零改动——`offline_voice`、`voice_command_router`、`xfyun_speech`、`deepseek_ros2`、`execution_mode_manager`、`execution_mode_interfaces`、`tongue_mode`、`motion_guard`、`roscar_interfaces`、`roscar_api`。

   为什么不放进 `roscar_interfaces` / `roscar_api`：这两个包被语音、控制等多条链路依赖。往里加 srv 会让这些包重新生成接口、重新部署，影响面超出本任务。
6. **不新增任何速度出口**，也不触碰授权和急停，原有约束继续有效。

**唯一允许的执行器改动**：`executor_node.py` 中 `request.source` 的白名单从 `('cloud','local_test')` 扩展为再接受 `'ros_api'`。这是纯新增，旧取值行为不变。此外上传时附加 `X-Task-Source` 请求头，见 §3.3。

---

## 1. 接口定义

### 1.1 新增 srv（放在 `capture_task_interfaces`，只新增文件）

`srv/RunCapture.srv`：

```
# Request one capture job through the existing executor. Never moves the vehicle.
string task_id        # 可空：门面自动生成 "ros-<32位hex>"；非空须匹配 [A-Za-z0-9_-]{1,64}
string announce       # 可空：用目录默认播报；非空须通过与云端覆盖相同的禁用词/长度校验
float64 prepare_sec   # <0 表示用目录默认；否则 0–30
float64 timeout_sec   # 0 表示用目录默认 task_timeout_sec；否则 (0, 600]
---
bool success          # 仅 status == uploaded 时为 true
string status         # uploaded | failed | cancelled | rejected | busy | unavailable | invalid | timeout
string detail         # 执行器原因码，例如 voice_mode_active、hold_failed、upload_http_500
string task_id        # 实际使用的 task_id（自动生成时回填）
string capture_id
string upload_json    # 与 action result 的 upload_json 相同（已校验的回执数组）
```

`srv/CancelCapture.srv`：

```
string task_id        # 必填；只取消本门面发起且正在执行的同 task_id 任务
---
bool success
string detail         # cancel_requested | not_active | not_owner
```

修改 `CMakeLists.txt`：在 `rosidl_generate_interfaces` 中追加这两个 srv。action 文件不动。

### 1.2 服务名

| 服务 | 类型 | 说明 |
|---|---|---|
| `/capture/face` | RunCapture | 面诊拍照 |
| `/capture/tongue` | RunCapture | 舌诊拍照（与语音舌苔模式 `tongue_mode` 无关，详见 §4） |
| `/capture/helmet` | RunCapture | 安全帽拍照 |
| `/capture/pcb` | RunCapture | PCB 拍照 |
| `/capture/cancel` | CancelCapture | 取消本门面发起的任务 |

- 服务名与 task_type 一一对应。服务列表从 `tasks.yaml` 的 tasks 自动生成，以后新增任务类型不需要改门面代码。
- 现有 `/capture_task/run` action 仍然保留，作为需要实时进度或主动取消的高级接口，在文档中一并说明。
- 状态订阅继续使用现有 `/capture_task/state` 和 `/capture_task/result`。

---

## 2. 实现：新节点 `capture_api_node`（放在 `capture_task` 包内）

```
ros2_ws/src/capture_task/capture_task/
  api_logic.py      # 纯 Python：请求校验、参数→goal 映射、结果映射（不 import rclpy）
  api_node.py       # ROS 门面：服务 → CaptureTask action client
ros2_ws/src/capture_task/test/test_api_logic.py
tests/test_capture_api_runtime.py
```

在 `setup.py` 新增 entry point `api_node`。不新建 ROS 包，避免再动 `check_project.py` 的包白名单；如果确实需要新包，要在 WORKLOG 说明理由。

### 2.1 行为

1. **校验**（`api_logic.py`，复用 `catalog.speech_text`、`catalog.number`、`protocol.identifier`）：
   - task_id 为空时自动生成。
   - announce 非空时做禁用词校验。
   - prepare_sec 取 <0 或 0–30。
   - timeout_sec 取 0 或 (0, 600]。

   不合法时立即返回 `status=invalid`，不发 goal。`params_json` 只放白名单字段 `announce` / `prepare_sec`。
2. **转发**：发送 `CaptureTask.Goal(task_id, task_type, source='ros_api', params_json, ttl_sec=timeout_sec)`。
3. **goal 被拒**：执行器会在 `/capture_task/result` 发布 `status=rejected` 和原因。门面订阅该话题，按 task_id 匹配，且接收时间要晚于本次调用开始的时刻。原因映射为：

   | 执行器原因 | 返回 status |
   |---|---|
   | `busy` | `busy` |
   | `voice_mode_active`、`invalid` | `rejected`，detail 为原因 |
   | 等待 0.2 秒仍未匹配到 | `rejected`，detail=`rejected` |

   参考 `bridge_node.py` 中 rejection 的处理方式。**不要修改 bridge**，在 `api_logic.py` 里独立实现小函数即可。
4. **goal 被接受**：阻塞等待 action result，直接映射为 `status/detail/capture_id/upload_json`，`success = (status == 'uploaded')`。
5. **服务端整体时限**：`timeout_sec`（或目录默认值）加 `speech_timeout_sec` 再加 5 秒余量。超时后**只取消自己发出的 goal**，返回 `status=timeout`。迟到的 accept 要立即取消，与 `speech.py` 的 `cancel_when_accepted` 同理。
6. **服务不可用**：`/capture_task/run` 未就绪（`wait_for_server` 2 秒）时返回 `status=unavailable`。
7. **并发**：门面同时只允许一个在途请求，第二个调用立即返回 `busy`。执行器本身也一次只跑一个任务，ROS、云端谁先来谁执行，后来的一方得到 busy。不做排队。
8. **取消**：`/capture/cancel` 只取消门面自己记录的当前 goal（task_id 匹配）。云端发起的任务不归门面所有，返回 `not_owner`；门面没有在途任务时返回 `not_active`。门面不能取消云端任务，云端也不能通过 ROS 取消门面任务——各自只取消自己的。
9. **执行器与回调线程**：使用 `MultiThreadedExecutor` 和 `ReentrantCallbackGroup`。服务回调阻塞期间，action 的回调必须仍能执行，可参照 `roscar_api/speech.py` 的写法。SIGINT/SIGTERM 时先取消自己的在途 goal，再关闭；可以复用 `capture_task/runtime.py`。
10. **不播报、不拍照、不上传**：门面的全部副作用都经由执行器完成，门面自己不调用 TTS、不订阅相机、不连接云端。

### 2.2 启动开关（默认关闭）

- `capture_task.launch.py` 新增参数 `with_ros_api`，默认 `false`；为 `true` 时追加 `api_node`。
- `start_project.sh` 新增 `CAPTURE_ROS_API=false`（只接受 true/false），仅在 `WITH_CAPTURE_TASKS=true` 时生效，透传为 `with_ros_api:=`；`WITH_CAPTURE_TASKS=false` 时设为 true 要报错。同步更新 `--help`。
- 也可以单独运行：`ros2 run capture_task api_node`（前提是执行器已在运行）。

---

## 3. 与云端的关系

### 3.1 图像仍然上传云端

ROS 发起的任务同样会上传到 `/v1/tasks/{task_id}/images`，因为推理在云端。task_id 由小车生成，统一以 `ros-` 开头，避免与云端下发的 ID 冲突。

### 3.2 云端看不到 ROS 任务的指令和事件

ROS 任务不经过 bridge，所以云端不会收到这些任务的 command/ack/events，只会收到图片。心跳中的 busy 状态仍然会反映出来，因为它来自执行器状态。本轮不让 bridge 补报这些事件，否则就要改 bridge。

### 3.3 协议的增量修改（向后兼容，只加不改）

- 执行器上传时增加请求头 `X-Task-Source: cloud | ros_api | local_test`，取值来自 goal 的 source。
- 协议文档新增一条：云端必须接受**未经自己下发**、task_id 以 `ros-` 开头的图片上传，并按 X-Task-Source 区分来源。
- mock 已经接受任意 task_id 的上传。增加一个可选的 `serve --reject-unknown-task` 开关，用来模拟严格的云端：只接受自己下发过的 task_id，否则返回 404。这样可以验证门面在这种情况下返回 `failed` / `upload_http_404`，而不是挂住。
- 这是协议 v0 的**新增**内容，旧客户端和旧云端都不受影响。务必在协议文档的"待云端确认"中写明：云端如果不接受 ROS 任务的上传，ROS 接口就只能拿到 `upload_http_404`。

---

## 4. 与现有功能的互斥

| 场景 | 预期 |
|---|---|
| 云端任务执行中，ROS 发起调用 | 返回 `busy`；云端任务不受影响 |
| ROS 任务执行中，云端下发指令 | 云端收到 ack `busy`（现有逻辑） |
| 语音舌苔模式 active | 返回 `rejected` / `voice_mode_active`（执行器现有门控） |
| robot 配置下急停锁存 | 执行器返回 `failed` / `hold_failed`（guard 拒绝租约，现有行为） |
| `/capture/tongue` 与语音"舌苔检测" | 两套独立流程：前者只拍照上传，不做本地结果播报；后者是 tongue_mode，有自己的云端合同。文档中必须写清楚，避免使用者混淆 |

---

## 5. 测试与验收

### 5.1 纯 Python 单测（`test_api_logic.py`，不 import rclpy）

- 请求校验：task_id 为空时自动生成；非法 ID；含禁用词的 announce；prepare_sec 与 timeout_sec 的边界值。
- goal 映射：生成的 `params_json` 只含白名单字段，`source='ros_api'`。
- 结果映射：
  - 执行器的 uploaded / failed / cancelled 原样映射；
  - rejection 原因映射到 busy、rejected；
  - success 只在 uploaded 时为 true。
- 服务时限计算。

### 5.2 ROS 运行时测试（`tests/test_capture_api_runtime.py`）

- 隔离要求：`ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1`，不满足时拒绝运行。
- 测试夹具：进程内运行假 TTS、合成相机、mock 云端、执行器和门面，**不启动 bridge**；需要验证"与云端并发"的用例再单独启动 bridge。

| 用例 | 断言 |
|---|---|
| 四个服务各调用 1 次 | uploaded；mock 收到的 JPEG SHA 一致；`X-Task-Source=ros_api`；task_id 以 `ros-` 开头 |
| 指定 task_id 与 announce | 假 TTS 收到的文本即为该 announce |
| 非法 announce（含"停车"） | invalid；执行器从未收到 goal |
| 门面在途时第二次调用 | 立即返回 busy |
| 云端任务执行中调用门面 | busy；云端任务照常 uploaded |
| 语音模式 active | rejected / voice_mode_active |
| `/capture/cancel` 同 task_id | cancelled；不播报失败提示（沿用修复第 2 项的规则） |
| `/capture/cancel` 云端任务的 task_id | not_owner；云端任务不受影响 |
| 执行器未运行 | unavailable（约 2 秒内返回） |
| 服务端超时（mock `--slow-upload` 配合很短的 timeout_sec） | timeout；之后执行器回到空闲，没有遗留 goal |
| mock `--reject-unknown-task` | failed / upload_http_404 |
| 门面 SIGTERM | 在途 goal 被取消，执行器回到空闲 |

### 5.3 回归（必须全部通过，证明"不影响现有接口"）

```bash
cd /home/smxvjv/Desktop/works/5060wslworks/jiekou/ROS-CAR
bash scripts/ci_unit_tests.sh
python3 scripts/check_project.py
git diff --check codex/cloud-capture-tasks..HEAD
source /opt/ros/humble/setup.bash
colcon build --base-paths ros2_ws/src --packages-up-to capture_task roscar_api motion_guard tongue_mode
source install/setup.bash
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 python3 tests/test_capture_task_runtime.py      # 原 14 项必须照旧全过
ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1 python3 tests/test_capture_api_runtime.py
ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 python3 tests/test_speech_api_runtime.py
# 以下必须为空：
git diff --stat main..HEAD -- ros2_ws/src/{offline_voice,voice_command_router,xfyun_speech,deepseek_ros2,execution_mode_manager,execution_mode_interfaces,tongue_mode,motion_guard,roscar_interfaces,roscar_api}
git diff codex/cloud-capture-tasks..HEAD -- ros2_ws/src/capture_task_interfaces/action ros2_ws/src/capture_task/capture_task/bridge_node.py ros2_ws/src/capture_task/capture_task/bridge_logic.py
```

额外检查：

- `executor_node.py` 相对基线的 diff 只能有两处：source 白名单，以及 `X-Task-Source` 请求头。
- `with_ros_api` 默认 false 时，`ros2 service list` 中**没有** `/capture/*`，节点集合与基线一致。

---

## 6. 文档

- **`docs/ROS接口使用文档.md`**：新增"拍照任务接口"一节，内容包括服务列表、srv 字段、CLI 与 Python 调用示例、互斥规则、"图像上传云端、推理结果不经此接口返回"的说明，以及验证范围。CLI 示例：

  ```bash
  ros2 service call /capture/pcb capture_task_interfaces/srv/RunCapture \
    "{task_id: '', announce: '', prepare_sec: -1.0, timeout_sec: 0.0}"
  ```

  注意：只是修改文档，不改 `roscar_api` 包。
- **`docs/云端拍照任务接口协议.md`**：写入 §3.3 的新增内容。
- **`docs/云端拍照任务实现与验收.md`**：补充门面节点、开关和验收结果。
- **README**：文档索引登记本文件；目录说明中给 capture_task 补一句"含可选 ROS 服务门面"。
- **AGENTS.md**：在"独立云端拍照任务"一节追加一条长期约定：ROS 门面只转发给 `/capture_task/run`，默认关闭，不改 action、bridge、HTTP 协议的既有字段。
- **WORKLOG**：新增一节，写明验证层级——仍然只是本机 Humble x86_64 上的模拟验证。

---

## 7. 分支与交付

- 从 `codex/cloud-capture-tasks` 新建 `codex/capture-ros-api`。不改基线分支，以免影响已带回 WSL、准备上车测试的版本。
- 只做本地提交，不 push、不 SSH 小车、不部署。完成后向用户汇报，由验收方复核。之后要带回 WSL 时，生成 `git bundle create capture-ros-api.bundle codex/cloud-capture-tasks..codex/capture-ros-api`。
- 交付清单：
  - [ ] RunCapture / CancelCapture 两个 srv
  - [ ] `api_logic.py`、`api_node.py` 及 entry point
  - [ ] launch 中的 `with_ros_api` 与 start_project 中的 `CAPTURE_ROS_API`（默认均为 false）
  - [ ] 执行器的 source 白名单与 `X-Task-Source` 请求头（仅此两处）
  - [ ] mock 的 `--reject-unknown-task`
  - [ ] 单测、运行时测试、全部回归
  - [ ] 文档、README、AGENTS、WORKLOG
