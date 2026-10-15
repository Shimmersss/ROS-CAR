# ROSCAR · 室内人体跟随与跌倒检测

**当前主线：C 路线**。Gemini Pro RGB-D 加 YOLO26s-pose，实现人体骨架、姿态与跌倒检测，并沿用 B 的跟踪、测距和选人契约。项目默认入口仍为 B（YOLO26s 检测 + ByteTrack）。A 骨架和红色目标路线已退役，资料在 [归档](docs/archive/README.md)。

各模块的验证程度不同。容器编译、CI 单测、合成测试和离线录像回放都不等于 Jetson 实机或实车验收；具体证据见 [WORKLOG](WORKLOG.md)。

## 当前状态（2026-10-12，本地合并）

| 模块 | 状态 |
|---|---|
| C 跌倒检测 | 已在 Mac 上用车载机位重拍并人工标注 20 次跌倒和 19 个反例动作，正在做离线评估与规则调整；Jetson 真人/跌倒实测未做。见 [整体逻辑](docs/跌倒检测整体逻辑.md)、[评估记录](docs/跌倒检测评估记录.md) |
| C 定位、三维姿态、ReID | 软件已实现，安装外参、重力和地面确认开关默认关闭；ReID 默认关闭 |
| B 人体检测/跟踪 | Jetson 上 YOLO26s/ByteTrack 有真人在线检测；RGB-D 配准、外参和绝对测距精度待验收 |
| 语音 | 共享录音 + 常开 KWS + Qwen3-ASR / Silero VAD + 固定规则 + MeloTTS；8 秒续聊、显式授权；舌苔流程和文字播报接口保留，讯飞仅识别回退 |
| 底盘、N10P、运动保护、导航 | 底盘串口在 Jetson 收发里程计；`start_robot.sh` 启动底盘和保护层，运动默认关闭、禁止自动授权（无雷达）。motion_guard、N10P、SLAM/Nav2 有软件和本机测试，实车跟随效果、避障与停车距离未验收 |
| 云台 | 设计与串口协议草案、桥接软件（仅仿真），无硬件 |

## 快速入口

**C 路线（Jetson）**

```bash
GEMINI_SERIAL=AY2755200PW \
GEMINI_CALIBRATION="$PWD/ros2_ws/src/perception_bringup/config/gemini_AY2755200PW.json" \
bash scripts/start_c.sh
```

`start_c.sh` 等价于 `PERCEPTION_ROUTE=c bash scripts/start_project.sh`。默认要求 Jetson 上导出的 Pose FP16 engine，缺失时报错，不会退回 detect。语音、雷达和底盘开关与 B 相同；运动默认关闭，跌倒状态不接语音或运动。Foxglove 导入 `foxglove/c-layout.json`。参数、模型准备和验收边界见 [方案 C 实现与验收](docs/方案C实现与验收.md)。

**C 离线评估（Mac）**

```bash
.venv/bin/python scripts/record_gemini_mac.py <段名>
```

上面是录制命令。标注用 `scripts/label_recording.py`，回放与评估用 `scripts/replay_falls.py` 和 `scripts/evaluate_falls.py`，本机骨架预览用 `scripts/preview_pose_mac.py`。录制与交付要求见 [重拍视频要求](docs/重拍视频要求20261010.md)。

**B 路线 / 整机（Jetson）**

```bash
bash scripts/start_project.sh
```

这条命令启动相机、B、Foxglove 和语音。默认不接底盘、不启动跟随、运动关闭；`WITH_FOLLOWER=true` 启动唯一保护层，允许在运动关闭下运行舌苔停车保持。查看选项用 `--help`。

> `scripts/start_robot.sh` 会打开底盘串口、语音和跟随器，但默认运动关闭、几何确认关闭且不自动授权。开启运动能力仍需显式模式授权；相机/制动参数须现场核验。它默认仍走 B；新 Gemini 摄像头请用 C 入口。此轮未更改小车正在运行的服务。

**对外 ROS 2 接口**：`ros2 launch roscar_api api.launch.py` 默认 IDLE、运动关闭、不启动硬件。外部速度只发 `/chassis/cmd_vel`，由唯一的 motion_guard 输出 `/cmd_vel`。不要与已包含 motion_guard 的入口叠加启动。详见 [ROS 接口使用文档](docs/ROS接口使用文档.md)。

主人认领使用 `ros2 run roscar_api target claim [--id epoch:track_id]`；丢失满 10 秒释放旧主人并允许重新认领，不延长运动保护宽限。启用语音/TTS 后，主人和跟随阶段自动简短播报，舌苔逐阶段播报并等待播放完成。`ros2 run roscar_api interfaces --json` 可枚举全部运行中接口；API 可选接入语音、舌苔和云台，参数及调用示例集中在上述手册。

## 构建与测试

Jetson 或 Ubuntu（Humble）：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
# 从仓库根目录构建，匹配 start_project/start_c 的 install 路径
bash scripts/build_ros.sh
source install/setup.bash
```

Mac：

```bash
PYTHON=.venv/bin/python scripts/ci_unit_tests.sh
```

这与 GitHub Actions `unit-tests` 同口径：纯 Python 单测，不含 ROS、硬件、权重和录像。`python3 scripts/check_project.py` 做结构检查，`bash scripts/test_container.sh` 在 Linux ARM64 Humble 容器里编译并运行 ROS 测试。三者都不能代替 Jetson 验证。

已构建的 Humble 环境运行 `bash scripts/test_flow_regressions.sh`，验证主人 10 秒生命周期、播报、运动/识别/定位/晚到语音五项回归及公共接口、舌苔整合。全部使用隔离 ROS 域和合成输入，不启动真实硬件。

同步到 Jetson 默认只预览差异，核对后追加 `--apply`；不传厂商包、权重、录像和构建产物，也不做远端删除。已有整机目录为 `/home/wheeltec/ROSCAR-current`，另有语音舌苔隔离试运行副本；本轮未核对或修改设备服务。验证时优先同步到隔离目录，例如：

```bash
python3 scripts/sync_to_jetson.py --host wheeltec@192.168.1.240 --dest /home/wheeltec/ROSCAR-c
```

## 目录

```text
ros2_ws/src/
  person_interfaces/     TargetState、PersonState 等公共消息
  yolo_person_tracker/   B 检测跟踪与 C 骨架/跌倒、地面与统一定位、ReID
  perception_bringup/    路线选择（yolo_pose / yolo / astra / red / demo）与 pose.yaml
  roscar_interfaces/     SetControlMode 服务
  roscar_api/            对外统一入口与示例
  motion_guard/          速度保护，唯一 /cmd_vel 出口
  navigation_bringup/    SLAM Toolbox、AMCL、Nav2
  gimbal_interfaces/     云台消息
  gimbal_bridge/         云台串口桥接（仅仿真验证）
  execution_mode_interfaces/ execution_mode_manager/ tongue_mode/
                         舌苔任务、模式调度、停车保持
  capture_task_interfaces/ 独立拍照 action 与 RunCapture / CancelCapture 服务
  capture_task/          云端任务→完成确认播报→新鲜原始RGB→上传，含可选 ROS 服务门面（默认关闭）
  offline_voice/         本地 ASR/TTS 语音入口
  xfyun_speech/          讯飞流式 ASR/TTS（回退）
  deepseek_ros2/         DeepSeek 文本桥（历史源码，不作为运动回退）
  voice_command_router/  语音控制路由
  astra_body_adapter/    A 骨架适配与 person_follower（历史路线）
  bodyreader_msg/        厂商骨架消息（历史路线）
  red_object_tracker/    红色目标跟踪（历史路线）
  chassis_vendor/ radar_vendor/ wheeltec_mic*/ tts_make_ros2/
                         迁入的厂商包，COLCON_IGNORE，独立脚本构建
scripts/   启动、构建、同步、录制/标注/评估
tests/     本机测试
foxglove/  布局与连接说明
models/    权重来源与哈希（权重不入库）
data/      录像与外部数据集（不入库）
docs/      方案、接口、计划；docs/archive/ 为历史文档
```

厂商原包 `JP6.2_wheeltec_ros2_src_20260903/` 和 `淘宝信息/` 只在本地保留，不进 Git，也不参与构建。

## 文档索引

**Demo 展示分支（`demo/showcase-20261016`，展示后删除）**：[Demo 文档入口](docs/demo/README.md) · [跌倒暂停与静止延续验收](docs/demo/跌倒暂停与静止延续验收.md) · [T2.6 离线优化评估](docs/demo/T2.6离线检测优化评估.md) · [相对运动 ROS 接口](docs/demo/相对运动ROS接口.md) · [Codex 任务清单](docs/demo/Codex任务清单.md)
- [拍照推理闭环实现与验收](docs/拍照推理闭环实现与验收.md)：5060 网关、增量 ROS/HTTP 合同与分层验证记录
- [推理网关使用说明](services/inference_gateway/README.md)：隔离部署、认证和模型状态

- [拍照推理闭环执行指南](docs/拍照推理闭环执行指南.md)：5060 隔离推理与增量闭环实施入口

- [10-15 WSL 模拟云端拍照联调](docs/2026-10-15WSL模拟云端拍照联调.md)：四类真实采集上传、播报、PCB 分辨率告警与测试清理
- [云端拍照任务修复说明](docs/云端拍照任务修复说明.md)（验收后七项修复要求）
- [云端拍照任务 ROS 接口方案](docs/云端拍照任务ROS接口方案.md)（可选服务门面实施入口）
- [云端拍照任务实现与验收](docs/云端拍照任务实现与验收.md)（独立链路、本机联调与待现场项）
- [云端拍照任务接口协议](docs/云端拍照任务接口协议.md)（云端开发接入：任务来源、HTTP v0、JPEG 回执与 mock）
- [云端拍照任务执行方案](docs/云端拍照任务执行方案.md)（本机独立链路实施入口）

**C 路线（主线）**
- [10-13 板端联调与开机跟随](docs/2026-10-13板端联调与开机跟随.md)：当前部署、负载优化、固定机位跌倒与开机服务
- [方案 C 实现与验收](docs/方案C实现与验收.md)：入口、模型、参数、骨架与跌倒首版
- [跌倒检测整体逻辑](docs/跌倒检测整体逻辑.md) · [跌倒检测评估记录](docs/跌倒检测评估记录.md)
- [统一定位与三维姿态](docs/方案C统一定位与三维姿态.md) · [单目地面定位独立验收](docs/方案C单目地面定位独立验收.md) · [身份 ReID](docs/方案C身份ReID接入与验收.md) · [单目标跟踪联动](docs/方案C单目标跟踪联动.md)
- [多源融合设计](docs/方案C多源人体感知融合设计.md) · [文献与开源项目评估](docs/方案C文献与开源项目评估.md)
- 数据：[重拍视频要求](docs/重拍视频要求20261010.md) · [Gemini 彩色偏蓝排查](docs/Gemini彩色偏蓝排查20261010.md) · [现场实测要求](docs/现场实测要求20261008.md) · [C 上车跟随交接](docs/C上车跟随交接20261012.md)
- [云台跟随设计与串口协议](docs/云台跟随设计与串口协议.md)（草案）

**语音与本次合并**
- [语音控制说明书](docs/语音控制使用说明书.md) · [当前语音入口与回退](docs/语音入口与回退.md)
- [舌苔整合与验收](docs/语音舌苔流程整合与验收.md) · [文字播报 ROS 接口](docs/文字播报ROS接口.md)（启动、CLI/Python、完成确认与取消）
- [远端 C 与语音合并说明](docs/远端C与语音合并说明20261012.md) · [原语音舌苔实机记录](docs/2026-10-12语音舌苔实机试运行.md)
- [语音优化无车验收](docs/语音优化无车实施与验收.md) · [Orin Nano 优化方案](docs/Orin Nano 小车语音控制系统优化.md)

**公共与其他模块**
- [完整方案](人体跟随感知方案.md) · [架构](docs/architecture.md) · [分阶段计划](docs/roadmap.md) · [开发工作流](docs/development.md)
- 接口：[TargetState / OwnerState](docs/interfaces.md) · [ROS 接口使用手册](docs/ROS接口使用文档.md)（拍照上传、播报、停车租约、epoch 授权、主人认领、跌倒字段与 CLI/Python 订阅）
- [方案 B 实现与验收](docs/方案B实现与验收.md) · [语音入口与回退](docs/语音入口与回退.md) · [端侧 ASR 候选实测](docs/端侧ASR候选实测.md)
- [motion_guard](ros2_ws/src/motion_guard/README.md) · [导航与自动绕障](docs/导航与自动绕障.md) · [N10P 雷达接入](docs/N10P雷达接入.md) · [底盘物理串口协议](docs/物理串口协议说明.md)
- [构建与部署](deploy/README.md) · [Foxglove](foxglove/README.md) · [原厂代码索引](docs/vendor_inventory.md) · [工作记录](WORKLOG.md) · [归档](docs/archive/README.md)

## 许可

代码托管在 GitHub `Shimmersss/ROS-CAR`。项目新增文件暂未授予开源许可证，ROS 包许可证暂写为 Proprietary；第三方代码和模型保留各自原许可。正式发布前由项目所有者确定许可证。
