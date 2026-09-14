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

已在 Jetson tools/foxglove-root 解包 ROS 仓库 ARM64 foxglove_bridge 3.4.3 和 rosx_introspection 2.3.0；无 sudo，未改系统安装。下载文件 SHA-256 与 apt 元数据一致。手动入口 `bash /home/wheeltec/ROSCAR/scripts/run_foxglove.sh`，只监听127.0.0.1:8765；Mac 使用 `ssh -N -L 127.0.0.1:8766:127.0.0.1:8765 roscar-wifi`，客户端连接 ws://localhost:8766。需要与感知节点使用相同 ROS_DOMAIN_ID。当前桥使用 foxglove.sdk.v1 协议，WebSocket 101握手实测通过，旧 foxglove.websocket.v1 探针返回400；测试已停止，未自启动。tools运行库不在日常源码同步白名单内。

方案 A 可视化的手动启动入口为 `scripts/run_astra_foxglove.sh`，Mac 隧道为 `scripts/open_foxglove_tunnel.sh`。前者只启动 `bodyreader/main`、bodylist_adapter 和 Bridge；其子进程全部由前台脚本的 Ctrl-C 清理。详见 `foxglove/README.md`。
