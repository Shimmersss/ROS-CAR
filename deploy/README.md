# 构建与部署

humble-test.Dockerfile 仅用于 Mac 上的 Linux/Humble 构建和消息链路测试，不是 Jetson GPU 部署镜像。
Jetson 的实际 CUDA、TensorRT、相机依赖应根据 JetPack 版本安装。已配置 SSH 并安装远程调试工具，尚未向设备部署本项目的 ROS 感知代码。
测试镜像基础可通过 ROSCAR_ROS_IMAGE 指定官方 ROS 镜像的可访问镜像源；不将镜像标签视为永久可复现版本，成功测试时记录实际摘要。

## 小车调试工具（2026-09-14）

- 系统实测：Ubuntu 22.04.5 LTS，aarch64，用户 `wheeltec`。
- Mac 连接：`ssh roscar-wifi`（192.168.1.240）或 `ssh roscar-ethernet`（192.168.100.2）。网线曾中断，安装与最终检查使用 Wi-Fi。
- Codex CLI 0.154.0：官方 ARM64 musl 完整包安装于小车 `/home/wheeltec/.local/share/codex/0.154.0`；启动器为 `/home/wheeltec/.local/bin/codex`。新 SSH 会话可直接运行 `codex`，已确认使用 ChatGPT 登录。
- 启动器为 Codex 及子进程设置 HTTP/HTTPS 代理 `http://127.0.0.1:7897`，并设置本机及内网 `NO_PROXY`。系统原有 Node.js 未替换。Bash 原配置备份为小车 `/home/wheeltec/.bashrc.before-codex-20260914`。
- Clash Verge Rev 2.5.2：官方 arm64 DEB，APT 补齐 WebKitGTK 4.1 等依赖，并安装 ripgrep。订阅导入 102 节点，规则模式，当前选择“新加坡-IEPL 01”；节点可用性会变化，可在应用内切换。
- Clash 配置目录位于小车 `/home/wheeltec/.local/share/io.github.clash-verge-rev.clash-verge-rev`，目录权限 700，订阅及主要配置权限 600。此目录包含敏感订阅信息，不同步到项目或分享。
- 代理只监听 `127.0.0.1:7897`，TUN 关闭，控制接口使用本机 Unix socket。已启用 GNOME 系统代理及桌面登录自启动，文件为小车 `/home/wheeltec/.config/autostart/clash-verge.desktop`。本次运行由临时用户服务 `roscar-clash-verge.service` 管理；此服务不等于无需桌面登录的开机服务。
- 已验证：软件版本、Clash 实际运行及重启后节点恢复、代理监听范围、配置权限、登录状态；经代理访问 OpenAI 登录服务返回 HTTP 200，未携带 API 密钥访问模型列表返回 HTTP 401（网络可达，不代表 API 密钥已配置）。Codex 使用默认模型执行只读最小请求，成功返回 `OK`，未调用工具。未执行系统重启验收或相机/车辆测试。

安装来源：[Codex 官方发行版](https://github.com/openai/codex/releases/tag/rust-v0.154.0)、[Clash Verge Rev 官方发行版](https://github.com/clash-verge-rev/clash-verge-rev/releases/tag/v2.5.2)。两个安装包均核对 GitHub release 提供的 SHA-256 后安装。

## 项目首次部署（2026-09-14）

项目已同步到小车 `/home/wheeltec/ROSCAR`，保留原厂 `/home/wheeltec/wheeltec_ros2`。四个感知框架包在 Jetson 原生 Humble 编译通过。串口三个包保留 COLCON_IGNORE，未编译或启动；未设置项目自启动。A/B 仍为 NOT_READY 占位入口，真实相机和 YOLO 算法尚未接入。日常同步仅包含 deploy/README.md 和 humble-test.Dockerfile，不复制代理节点 YAML。权重及录像仍不在默认同步范围内。

## Foxglove 用户目录运行环境

已在 Jetson tools/foxglove-root 解包 ROS 仓库 ARM64 foxglove_bridge 3.4.3 和 rosx_introspection 2.3.0；无 sudo，未改系统安装。下载文件 SHA-256 与 apt 元数据一致。手动入口 `bash /home/wheeltec/ROSCAR/scripts/run_foxglove.sh`，默认监听 `0.0.0.0:8765`；同一 Wi-Fi 上的 Mac 连接 `ws://192.168.1.240:8765`。需要与感知节点使用相同 ROS_DOMAIN_ID。当前桥使用 foxglove.sdk.v1 协议，局域网 WebSocket 101 握手实测通过；未自启动。tools 运行库不在日常源码同步白名单内。离开该 Wi-Fi 时可用 `scripts/open_foxglove_tunnel.sh` 临时建立 SSH 隧道。

方案 A 可视化的手动启动入口为 `scripts/run_astra_foxglove.sh`，Wi-Fi 直连检查入口为 `scripts/connect_foxglove_roscar.sh`。前者只启动 `bodyreader/main`、bodylist_adapter 和 Bridge；其子进程全部由前台脚本的 Ctrl-C 清理。详见 `foxglove/README.md`。

## 分支 a 的红色方案 A（本轮未部署）

默认 A runner 与仓库 systemd 模板改为 `run_red_foxglove.sh`，不再启动 bodyreader，串口、运动、深度配准确认默认 false。原骨架组合脚本保留。新配置不会自行改变小车已安装的服务；现场切换前先核对旧服务及串口占用。

可选串口使用 `scripts/build_chassis.sh` 构建到 `ros2_ws/chassis_install`；不要移除原始包的 COLCON_IGNORE。Ubuntu Humble 构建所需额外依赖由 `deploy/humble-test.Dockerfile` 列出；本机测试容器已安装。完整配置及实机待验收项见 [红色方案 A](../docs/方案A红色物体跟随.md)。

N10P 本机验证运行 `bash scripts/test_radar_container.sh`；Linux/Jetson 独立构建 `bash scripts/build_radar.sh`，启动 `bash scripts/run_radar.sh serial_port:=/dev/wheeltec_lidar`。先按 `docs/N10P雷达接入.md` 核对实物串口。默认感知不启雷达，未安装自启；本轮没有部署远端。

受保护跟随入口为 `ros2 launch motion_guard follow.launch.py`，默认仅感知，不启动底盘或雷达；已有带底盘红色路线也接入 guard。`motion_enabled=true` 不等于已授权，须先填写确认 safety.yaml，再调用 `/control/arm`。停止用 `/control/stop`。本地源码更新未部署到在线服务；部署前阅读 motion_guard/README.md 的接口迁移和停车模型限制。

可选导航环境：安装 `ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox`，再构建主动工作区。`deploy/navigation-test.Dockerfile` 与 `scripts/test_navigation_container.sh` 提供本机 ARM64 Humble 的真实导航节点测试。未自动部署或新增自启动，设备启动、标定及 Foxglove 说明见 `docs/导航与自动绕障.md`。

### 2026-09-20 本机审查修复验证

`bash scripts/test_container.sh` 包含控制来源/时效、GPIO 假设备及 A 启动互斥测试。随后可运行 `bash scripts/test_chassis_container.sh`，补齐独立容器依赖并在临时副本编译串口三包、运行串口字节检查；不移除工作区 COLCON_IGNORE、不访问串口。

`route_a.sh` 发现 systemd 服务已安装时不再启动手动副本，包括自动重启退避期。已停止/失败服务应使用 `sudo systemctl start roscar-route-a.service`；手动模式生命周期通过 Linux `flock` 互斥。

## 对外 API 构建清单

主动工作区新增 `roscar_interfaces`（SetControlMode）和 `roscar_api`（统一入口与示例），总计 13 个主动包。`build_ros.sh` 使用 colcon 自动扫描；Humble 系统依赖新增 `ros-humble-vision-msgs`。`sync_to_jetson.py` 的 `ros2_ws/src`、`scripts`、`tests`、`docs` 白名单已覆盖新增包与手册，不新增权重/录像或代理配置同步。本轮仅本机验证，没有执行同步或部署。使用方法见 [ROS 接口使用文档](../docs/ROS接口使用文档.md)。
