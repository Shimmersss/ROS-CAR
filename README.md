# ROSCAR · 室内人体跟随感知

当前目标：Orin Nano Super 8GB + Astra 深度相机，比较 A（原厂骨架）和 B（YOLO + 深度）路线，通过 Mac 上的 Foxglove / SSH 调试。当前不接下位机，不输出车辆控制指令。

## 框架状态

| 内容 | 状态 |
|---|---|
| 公共 TargetState 消息、四个 ROS 2 包、A/B/demo 启动选择 | 已建立 |
| A / B 节点 | A 已接真实骨架适配器；B 仍为 NOT_READY |
| demo | 显式模拟数据：9 秒目标可见、3 秒丢失，用于验证消息与展示 |
| Mac → Jetson 同步脚本、模型清单、测试脚本 | 已建立 |
| 讯飞流式 ASR/TTS → DeepSeek 语音助手 | Orin 真人语音 → 讯飞 IAT → DeepSeek 回答已跑通；TTS 暂停 |
| DeepSeek 蜂鸣器工具链 | 白名单路由已建立；蜂鸣器属于下位机，协议适配延后 |
| 相机、骨架与测距 | A 已在 Jetson 真人验收；YOLO 路线待实现，SDK 授权提示待厂商解释 |
| 下位机串口驱动及两个依赖包 | 已迁入 chassis_vendor，默认跳过构建，未启动、未实机验证 |
| Foxglove | A 路线布局已验收，Wi-Fi 直连 `ws://192.168.1.240:8765` |

具体测试结果见 [工作记录](WORKLOG.md)。容器编译通过不等于 Jetson 相机或 GPU 已验证。

## 目录

```text
ros2_ws/src/
  person_interfaces/     公共目标观测消息
  astra_body_adapter/    A 路线真实 /bodylist 适配器
  yolo_person_tracker/   B 路线入口（NOT_READY）
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

# 默认 B 入口，当前只报告 NOT_READY
ros2 launch perception_bringup perception.launch.py route:=yolo
# A 入口：需另行运行厂商 bodyreader/main
ros2 launch perception_bringup perception.launch.py route:=astra
# 显式启用模拟数据：route:=demo
```

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

## 方案 A 最新联调（2026-09-14）

ASTRA S 深度流、真实人体骨架、叉腰锁定、质心测距、掩码和 Foxglove 展示均已实机跑通。`route:=astra` 现启动已验证的 `/bodylist` 适配器；厂商 bodyreader 仍由安全组合脚本单独启动，不包含底盘节点。SDK 授权提示没有阻止本次输出，但仍待厂商解释。详细入口与限制见 [方案 A 联调记录](docs/方案A联调记录.md)。
