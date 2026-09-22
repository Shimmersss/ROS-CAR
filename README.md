# ROSCAR · 室内人体跟随感知

当前目标：Orin Nano Super 8GB + RGB-D 感知，通过 Foxglove / SSH 调试。默认感知与运动关闭入口保留；底盘、雷达及受保护控制均须显式启用。各路线本机/实车验证范围见 WORKLOG。

## 对外 ROS 2 接口

统一入口 `ros2 launch roscar_api api.launch.py` 默认 IDLE、运动关闭、不启动硬件。新增 `/chassis/cmd_vel`（TwistStamped）、`/control/set_mode`（IDLE/EXTERNAL/FOLLOW）和 RGB-D `/perception/detections`（全部二维候选框）；原 TargetState 保持不变。字段、QoS、中文 CLI/Python 示例见 [ROS 接口使用文档](docs/ROS接口使用文档.md)。不要与已有包含 motion_guard 的入口叠加启动。

## 框架状态

| 内容 | 状态 |
|---|---|
| 公共 TargetState/模式服务、13 个主动 ROS 2 包、A/B/demo 启动选择 | 已建立 |
| A / B 节点 | A 已接真实骨架适配器；B 已有本地真实算法实现，待实机验收 |
| demo | 显式模拟数据：9 秒目标可见、3 秒丢失，用于验证消息与展示 |
| Mac → Jetson 同步脚本、模型清单、测试脚本 | 已建立 |
| 讯飞流式 ASR/TTS → DeepSeek 语音助手 | Orin 真人语音 → 讯飞 IAT → DeepSeek 回答已跑通；TTS 暂停 |
| DeepSeek 蜂鸣器工具链 | 白名单路由已建立；蜂鸣器属于下位机，协议适配延后 |
| 相机、骨架与测距 | A 已在 Jetson 真人验收；B 已本机接入 YOLO26s/ByteTrack，待相机及 Jetson 验收，SDK 授权提示待厂商解释 |
| 下位机串口驱动及两个依赖包 | 已迁入 chassis_vendor，默认跳过构建，未启动、未实机验证 |
| Foxglove | A 路线布局已验收，Wi-Fi 直连 `ws://192.168.1.240:8765` |

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
  voice_command_router/  模型工具白名单与蜂鸣器适配
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
# 新 A：外部配准 RGB-D + 红色物体，默认无底盘
ros2 launch perception_bringup route_a.launch.py
# 原骨架兼容入口：另行运行 bodyreader/main
ros2 launch perception_bringup perception.launch.py route:=astra
# 显式启用模拟数据：route:=demo
```

方案 A 一键入口在本分支改为红色目标节点和 Foxglove Bridge，等待外部配准 RGB-D，默认不启动底盘或 `/cmd_vel`。可选串口与运动配置见 [方案 A 红色物体跟随](docs/方案A红色物体跟随.md)。本轮仅本机实现，未更新在线服务：

```bash
# 在 Jetson 仓库根目录
bash scripts/route_a.sh start
bash scripts/route_a.sh status
bash scripts/route_a.sh logs
bash scripts/route_a.sh stop

# 在 Mac 仓库根目录，通过 SSH 一键远程拉起
bash scripts/route_a_remote.sh start
```

Mac 也可直接双击仓库根目录的 `启动方案A.command`。启动成功后 Foxglove 连接 `ws://192.168.1.240:8765`。

需要 Jetson 开机自动启动方案 A 时，在代码同步到 `/home/wheeltec/ROSCAR` 后执行一次：

```bash
bash scripts/install_route_a_autostart.sh install

# 后续管理
bash scripts/install_route_a_autostart.sh status
bash scripts/install_route_a_autostart.sh logs
bash scripts/install_route_a_autostart.sh remove
```

该 systemd 服务以 `wheeltec` 用户运行，开机启动并在异常退出后等待 5 秒重启。它仍只运行人体感知和 Foxglove，不启动底盘或 `/cmd_vel`。

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

以下为历史骨架路线记录，新 A 红色路线尚未实机验收。ASTRA S 深度流、真实人体骨架、叉腰锁定、质心测距、掩码和 Foxglove 展示均已实机跑通。`route:=astra` 现启动已验证的 `/bodylist` 适配器；厂商 bodyreader 仍由安全组合脚本单独启动，不包含底盘节点。SDK 授权提示没有阻止本次输出，但仍待厂商解释。详细入口与限制见 [方案 A 联调记录](docs/方案A联调记录.md)。

B 方案的输入契约、依赖、锁定服务与验证范围见 [方案 B 实现与验收](docs/方案B实现与验收.md)。

## 项目总启动（Jetson）

```bash
bash scripts/start_project.sh
```

统一启动 Astra 彩色/深度相机、红色方案 A、Foxglove 与语音助手；Ctrl-C 停止整组。日志在 `artifacts/project/`。语音需要私有凭据，不需要语音时使用 `WITH_VOICE=false bash scripts/start_project.sh`。底盘默认关闭，通过 `WITH_CHASSIS=true SERIAL_PORT=实际串口 CAR_MODE=实际车型` 启用收发；运动另需 `MOTION_ENABLED=true` 和已验证的 `DEPTH_REGISTERED=true`。查看全部选项：`bash scripts/start_project.sh --help`。

总入口在小车本机运行，依赖已构建的项目与厂商相机包；不自动部署。若现有 A systemd 服务运行，先停止该服务以释放相机。默认相机原始彩色流可用于检测可视化；控制测距仍要求实际校正/配准输入，通过 COLOR_TOPIC、DEPTH_TOPIC、CAMERA_INFO_TOPIC 指定，不能把启动驱动当作配准验证。

### N10P 雷达

已接入雷神 N10Plus 驱动的独立构建和启动入口：`bash scripts/build_radar.sh`、`bash scripts/run_radar.sh`。与感知组合使用 `with_radar:=true`（默认关闭），输出 `/scan`、`/radar/points` 和 `/radar/status`；不启动底盘。端口、标定与本机测试见 [N10P 雷达接入](docs/N10P雷达接入.md)。

### 受保护跟随

跟随节点现仅发 `/control/cmd_vel_request`，最终速度由 `motion_guard` 审查后发布。`ros2 launch motion_guard follow.launch.py` 默认仅感知模式，不启动底盘/传感器或自动授权；尺寸、安装 TF、停车模型未确认时禁止运动。启动、服务和故障边界见 [motion_guard](ros2_ws/src/motion_guard/README.md)。记录与隔离回放分别使用 `scripts/record_follow.sh`、`scripts/replay_follow.sh`。

导航与自动绕障代码入口：`scripts/run_navigation.sh`，支持 SLAM 建图、AMCL 地图定位、Nav2 人体目标跟随及 Foxglove 地图/路径布局。默认不启用运动。详见 [导航与自动绕障](docs/导航与自动绕障.md)。
