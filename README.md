# ROSCAR · 室内人体跟随感知

当前稳定版本：V5.1。本分支试验端侧语音，默认入口改为离线 ASR/TTS 和本机 Qwen3；`VOICE_BACKEND=online` 保留原讯飞/DeepSeek 路径。语音控制支持模式切换、短时运动、停止和状态查询；蜂鸣器代码保留为独立模块，不接入语音控制。部署与回退见 [离线语音部署与回退](docs/离线语音部署与回退.md)。

当前目标：Orin Nano Super 8GB 上的 B 人体感知、N10P 雷达、语音、底盘回传和 Foxglove。总入口不启动车辆跟随；B 的测距仍等待 RGB-D 配准实物验收。各模块验证范围见 WORKLOG。

## 对外 ROS 2 接口

统一入口 `ros2 launch roscar_api api.launch.py` 默认 IDLE、运动关闭、不启动硬件。新增 `/chassis/cmd_vel`（TwistStamped）、`/control/set_mode`（IDLE/EXTERNAL/FOLLOW）和 RGB-D `/perception/detections`（全部二维候选框）；原 TargetState 保持不变。字段、QoS、中文 CLI/Python 示例见 [ROS 接口使用文档](docs/ROS接口使用文档.md)。不要与已有包含 motion_guard 的入口叠加启动。

## 框架状态

| 内容 | 状态 |
|---|---|
| 公共 TargetState/模式服务、13 个主动 ROS 2 包、A/B/demo 启动选择 | 已建立 |
| B 节点 | 已在 Jetson 运行 YOLO26s/ByteTrack，真人检测和临时内参三维坐标有在线样本；连续跟踪与绝对精度待验收 |
| demo | 显式模拟数据：9 秒目标可见、3 秒丢失，用于验证消息与展示 |
| Mac → Jetson 同步脚本、模型清单、测试脚本 | 已建立 |
| 离线语音分支 | sherpa-onnx 流式 ASR、MeloTTS 与本机 Ollama/Qwen3；实机验收状态见 WORKLOG |
| 原讯飞/DeepSeek 语音助手 | V5.1 已跑通；本分支通过 `VOICE_BACKEND=online` 回退 |
| 语音控制工具 | 支持受限运动、模式切换、停止和状态查询；不提供蜂鸣器工具 |
| 相机与测距 | 彩色/深度 CameraInfo 已加载用户临时值；B 在线有有效 XYZ，仍需重新标定和物理量距 |
| 底盘串口驱动 | 已在 Jetson 构建并运行，仅收发里程计；当前无速度发布者 |
| Foxglove | 当前使用 B + 雷达布局；网口地址 `ws://192.168.100.2:8765` |

B 当前选用官方预训练 **YOLO26s 检测版 + ByteTrack**，默认免 NMS 推理；目标 Jetson 加速使用 TensorRT FP16（引擎尚未在板端构建）。权重准备、本机测试和导出命令见 [B 方案实现与验收](docs/方案B实现与验收.md)。

具体测试结果见 [工作记录](WORKLOG.md)。容器编译通过不等于 Jetson 相机或 GPU 已验证。

## 目录

```text
ros2_ws/src/
  roscar_interfaces/    对外 SetControlMode 服务
  roscar_api/           统一安全默认入口与四个 Python 示例
  person_interfaces/     公共目标观测消息
  astra_body_adapter/    A 路线真实 /bodylist 适配器
  yolo_person_tracker/   B 路线（YOLO/ByteTrack/配准深度）
  perception_bringup/    单路线启动与显式 demo
  xfyun_speech/          讯飞 WebSocket 流式 ASR/TTS
  deepseek_ros2/         DeepSeek 文本对话桥
  voice_command_router/  语音控制路由；独立蜂鸣器源码保留但不接入语音
  chassis_vendor/       原厂串口驱动与依赖，COLCON_IGNORE 暂不编译
scripts/                 构建、检查、同步与模型准备
foxglove/                连接说明和面板计划
models/                  权重来源、哈希；大文件留本地
data/                   录像目录与实验元数据
deploy/                 容器测试与部署约定
docs/                   架构、接口、开发计划、原厂资料索引及流程图
JP6.2_wheeltec_ros2_src_20260903/  本地厂商参考，排除在普通 Git 与日常同步外
淘宝信息/                 原始商品资料，本地保留
```

厂商目录保持原路径，已有文档链接继续可用。主动开发代码放 ros2_ws/src，构建时只扫描该目录，不把 114 个厂商包一次性编入新工作区。

## 在已安装 Humble 的 Jetson / Ubuntu 上

```bash
# 仓库根目录
source /opt/ros/humble/setup.bash
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
bash scripts/build_ros.sh
source ros2_ws/install/setup.bash

# 默认 B 入口未配置模型/配准时报告 NOT_READY
ros2 launch perception_bringup perception.launch.py route:=yolo
# 当前整机入口：Astra + B + N10P + 语音 + 底盘回传，运动关闭
bash scripts/start_robot.sh
# 显式启用模拟数据：route:=demo（仅测试）
```

红色目标路线已退出当前启动链路；旧脚本和历史资料保留作版本记录，不用于现行部署。Foxglove 导入 [B + 雷达布局](foxglove/b-radar-layout.json)。

另一个终端加载相同环境后检查：

```bash
ros2 topic echo /perception/target_state
```

如需 Foxglove，先安装 ros-humble-foxglove-bridge，再在启动命令追加 `with_foxglove:=true`。当前 Wi-Fi 直连地址与 SSH 隧道备用方案见 [Foxglove 说明](foxglove/README.md)。

## Mac 上的检查

```bash
python3 scripts/check_project.py
# 已有 Docker 服务时，构建真正的 Humble 测试环境并执行 ROS 运行测试
bash scripts/test_container.sh
```

容器只挂载主动开发源码、脚本和测试。构建输出保存在容器内，结果日志保存到 artifacts/，不会运行厂商节点或接硬件。

## 代码与数据管理

Mac 保存代码、文档和 Git 历史，Jetson 保存运行副本并执行硬件测试。默认只做同步预览：

```bash
python3 scripts/sync_to_jetson.py --host 用户名@IP --dest /home/用户名/ROSCAR
# 核对预览后，同一命令追加 --apply 实际同步
```

同步不会传输厂商原包、权重、录像、Git 和构建产物，也不执行远端删除。远端目录应为本项目专用目录；不同时在两台机器修改同一文件。

## 文档入口

- [完整方案](人体跟随感知方案.md) · [两方案对比 PNG](docs/diagrams/人体跟随两方案对比.png)
- [架构与包边界](docs/architecture.md) · [消息接口](docs/interfaces.md)
- [分阶段开发计划](docs/roadmap.md) · [Mac / Jetson 工作流](docs/development.md)
- [串口代码与协议](ros2_ws/src/chassis_vendor/README.md)
- [原厂代码索引](docs/vendor_inventory.md) · [工作记录](WORKLOG.md)

仓库尚未配置代码远端。项目新增文件暂未授予开源许可证；ROS 包许可证占位为 Proprietary，不改变任何第三方代码或模型的原许可证。实际发布前由项目所有者确定许可。

## 原骨架方案 A 历史联调（2026-09-14）

以下为历史骨架路线记录，不属于当前启动链路。ASTRA S 深度流、真实人体骨架、叉腰锁定、质心测距、掩码和 Foxglove 展示均已实机跑通。`route:=astra` 现启动已验证的 `/bodylist` 适配器；厂商 bodyreader 仍由安全组合脚本单独启动，不包含底盘节点。SDK 授权提示没有阻止本次输出，但仍待厂商解释。详细入口与限制见 [方案 A 联调记录](docs/方案A联调记录.md)。

B 方案的输入契约、依赖、锁定服务与验证范围见 [方案 B 实现与验收](docs/方案B实现与验收.md)。

## 项目总启动（Jetson）

```bash
bash scripts/start_project.sh
```

当前总入口启动 Astra、B 人体检测/ByteTrack、N10P、Foxglove 与语音；`start_project.sh` 默认不接底盘，`start_robot.sh` 还启动底盘串口收发。两者均不启动跟随，拒绝 `MOTION_ENABLED=true`。现行 `start_robot.sh` 按用户要求使用临时内参与驱动注册深度发布 B 三维坐标，精度待重新标定；运动仍关闭。布局见 [B + 雷达 Foxglove](foxglove/README.md)，日志在 `artifacts/project/`；查看选项：`bash scripts/start_project.sh --help`。

总入口在小车本机运行，依赖已构建的项目与厂商相机包；不自动部署。若现有 A systemd 服务运行，先停止该服务以释放相机。默认相机原始彩色流可用于检测可视化；控制测距仍要求实际校正/配准输入，通过 COLOR_TOPIC、DEPTH_TOPIC、CAMERA_INFO_TOPIC 指定，不能把启动驱动当作配准验证。

### N10P 雷达

已接入雷神 N10Plus 驱动的独立构建和启动入口：`bash scripts/build_radar.sh`、`bash scripts/run_radar.sh`。与感知组合使用 `with_radar:=true`（默认关闭），输出 `/scan`、`/radar/points` 和 `/radar/status`；不启动底盘。端口、标定与本机测试见 [N10P 雷达接入](docs/N10P雷达接入.md)。

### 受保护跟随

跟随节点现仅发 `/control/cmd_vel_request`，最终速度由 `motion_guard` 审查后发布。`ros2 launch motion_guard follow.launch.py` 默认仅感知模式，不启动底盘/传感器或自动授权；尺寸、安装 TF、停车模型未确认时禁止运动。启动、服务和故障边界见 [motion_guard](ros2_ws/src/motion_guard/README.md)。记录与隔离回放分别使用 `scripts/record_follow.sh`、`scripts/replay_follow.sh`。

导航与自动绕障代码入口：`scripts/run_navigation.sh`，支持 SLAM 建图、AMCL 地图定位、Nav2 人体目标跟随及 Foxglove 地图/路径布局。默认不启用运动。详见 [导航与自动绕障](docs/导航与自动绕障.md)。
