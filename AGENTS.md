# 项目协作约定

1. 每次完成项目更新后，及时更新 AGENTS.md 和 WORKLOG.md，并做一轮最小程度的审查。
2. 缺少必要环境时主动补齐；不得将静态检查描述为编译通过或实机验证。

## 当前上下文（2026-09-14）

- 目标：室内人体跟随小车，计划使用 Orin Nano Super 8GB，通过 Mac 上的 Foxglove 和 SSH 调试。
- 用户最初称相机为 Astra Pro；保存的商品资料涉及 Astra Pro Plus 和 Gemini Pro。实际到手型号及 USB 标识仍需核对。
- 厂商代码位于 `JP6.2_wheeltec_ros2_src_20260903/`，来自 Humble 源码目录，文件夹标注 JP6.2。
- 底盘具体型号、实物下位机协议兼容性尚未确认；不要将源码默认 `mini_mec` 当作用户车型。已 SSH 确认小车为 Ubuntu 22.04.5 LTS、aarch64，JetPack 6.2 / CUDA 12.6 已通过 SSH 核对。
- SSH 地址已由用户确认：Wi-Fi 为 `wheeltec@192.168.1.240`，网线为 `wheeltec@192.168.100.2`，端口均为 22。本机 `~/.ssh/config` 已添加 `roscar-wifi`、`roscar-ethernet` 别名；两地址均曾登录成功，安装期间网线连接中断，后续使用 Wi-Fi。密码不写入项目文档或 SSH 配置。此前 Codex 应用内新增连接受界面工具安全限制，未由本任务完成。
- 小车已安装 Codex CLI 0.154.0、Clash Verge Rev 2.5.2 与必要桌面依赖。Codex 已确认 ChatGPT 登录；启动器自动使用 `127.0.0.1:7897` 代理。Clash 已导入用户订阅、使用规则模式和新加坡节点，关闭 TUN/局域网代理访问，配置桌面登录自启动。配置位置及验证范围见 `deploy/README.md`；订阅 URL、节点凭据和登录凭据禁止写入项目文件。
- 已完成源码静态初查，具体证据与待办见 WORKLOG.md；尚未在 Jetson 编译或连接硬件。
- 当前任务边界：用户要求先不考虑下位机；先研究和验证人体感知、目标跟踪、深度测距及 Foxglove 展示，暂不接车辆运动。
- 当前路线建议：相机驱动 + YOLO11n + ByteTrack + 配准后的深度测距，按需求再评估 Pose、分割或 ReID。此路线尚未实施。
- 方案主文档为 `人体跟随感知方案.md`，包含硬件区别、骨架 SDK 路线、源码现状、YOLO/深度流程、Foxglove/SSH 配置和感知验收顺序。更新方案时同步维护此文档。
- 已补充骨架与滤波逻辑：可见人体应用代码为姿势锁定、平均 RGB 恢复和简化 PD，没有显式人体卡尔曼；SDK 内部未知。ByteTrack 自带检测框卡尔曼，空间深度滤波需独立设计；整车 EKF 不等于人体滤波。
- 两条路线的 SVG 图位于 `docs/diagrams/`，主文档已链接。图中必须区分“已有源码/库”“待开发/接入”“待实机验证”，避免将库已附带表述为已部署成功。
- 两图并排合并的 PNG 为 `docs/diagrams/人体跟随两方案对比.png`（5780×3800）；修改 SVG 后如需分享合并图，应同步重新导出。
- 主动开发工作区为 `ros2_ws/src/`，包含 person_interfaces、astra_body_adapter、yolo_person_tracker、perception_bringup。厂商目录保持原样并排除普通 Git；构建仅扫描主动工作区。
- 正式 `route:=astra` 已接入实测 bodylist_adapter；`route:=yolo` 仍为 NOT_READY。只有显式 `route:=demo` 才输出 `is_simulated=true` 的演示数据，禁止将 demo 或容器验证描述为实机人体识别。
- 四个包已在本机 Docker 的 Linux ARM64 ROS 2 Humble 环境编译通过；A/B、demo 与非法 route 运行检查通过。尚未在 Jetson 连接相机、验证 SDK 或 CUDA。
- 项目入口见 README.md，接口见 docs/interfaces.md，任务顺序见 docs/roadmap.md。Mac 同步脚本默认 dry-run，不传厂商原包、权重、录像或构建产物，不执行远端删除。

- 用户已授权迁入串口相关代码但暂不启动：ros2_ws/src/chassis_vendor/ 包含原样复制的 turn_on_wheeltec_robot、wheeltec_robot_msg、serial，42 个源文件由 SOURCE_MANIFEST.json 记录哈希。COLCON_IGNORE 默认跳过这三个包，未接入 perception_bringup；仅原四包有既往编译验证，迁入串口包尚未编译或实机验证。

- 新增约 4000 字课程设计报告 `docs/室内人体跟随小车课程设计.md`，汇总识别、锁定、深度、滤波、控制与实验设计；该文档为方案及阶段总结，不能视为算法或实车已完成。

- 用户澄清需要选题框架而非完整课设论文。新增 `docs/智能车方向大作业选题.md`，按示例 PDF 的场景、技术栈、完成要求、评分与交付物结构提供 8 个独立选题；先前课设报告保留，仅作背景资料。
- 新增 `deploy/clash-hk-vps.yaml`：仅使用香港 VPS `8.217.15.181` 的 VMess/Hysteria2 节点，供 Clash Verge 导入；未修改服务器现有业务服务。

- 2026-09-14 环境检查见 docs/小车环境检查.md：GPU 张量运算及 torchvision CUDA NMS 成功；系统 Python 缺 ultralytics，未发现 Foxglove bridge，torchvision 图像扩展缺 libnvjpeg.so.12。USB 实测 ORBBEC ASTRA S（2bc5:0402），与先前型号描述不一致，RGB 来源及实物标签待核对。厂商工作区已有安装包但未启动验证；本次未更改远端配置。

- 2026-09-14 已首次部署项目到 Jetson `/home/wheeltec/ROSCAR`，原生 Humble 四包编译通过（16.5s）；原厂工作区未改，串口继续 COLCON_IGNORE，未设置自启动。同步脚本 deploy 改为明确文件白名单，排除代理节点配置。真实感知算法仍未接入。
- 首次部署后 Jetson 本机隔离 ROS 域的 A/B/demo 与非法 route 运行检查全部通过，测试已退出；不是相机或人体识别实测。

- 2026-09-14 方案A联调：用户授权3个Sol子代理协助。ASTRA S深度流实测640×480/16UC1，15秒窗口收到345帧；SDK main报0x50000a19授权无效，用户需找厂商确认。禁止把SDK帧编号当骨架成功。
- 新增bodyreader_msg和独立bodylist_adapter（默认A占位入口不变），5包Jetson编译成功，5个纯逻辑单测与Jetson合成ROS状态测试通过。毫米转米，Y轴取反是待实机验证的约定；无传感器时间戳。测试不启动底盘，厂商bodydata_process也发布cmd_vel，因此不使用。见docs/方案A联调记录.md。

- Foxglove bridge 3.4.3 与 rosx_introspection 2.3.0 已解包到 Jetson tools/foxglove-root，脚本 scripts/run_foxglove.sh 手动启动、仅监听本机。foxglove.sdk.v1握手通过，旧协议探针失败，客户端展示待验收；未设置自启动。骨架错误摘录 docs/方案A骨架错误日志.txt 可转厂商，完整日志保存在 artifacts。

- 用户在场复测：40秒窗口1005条Bodylist全部count=0，无ID/关节识别结果；授权错误文字仍出现，代码本轮0x50000739。未跑通但原因未定，需继续核对视野/配置/厂商程序，不能仅根据日志认定授权必然阻塞。测试已停止。
- 随后人体实测复测成功：40秒 1004 条 Bodylist，707 条检测到 1 人（ID 93）、707 条有效质心、706 条有效关节、最大19关节，叉腰判定1帧，Z约0.99–1.18m。Invalid Orbbec Body Tracking license 仍出现（0x50000719），但未阻止实际骨架输出；记录为待厂商解释的SDK异常/授权提示，不再作为当前功能硬阻塞。未启底盘或任何控制节点。
- 方案 A Foxglove 链路已补齐：bodyreader/main → bodylist_adapter → TargetState + visualization Marker → 用户目录 foxglove_bridge。启动脚本 `scripts/run_astra_foxglove.sh` 使用 ROS 域182、ROS_LOCALHOST_ONLY=0、独立进程组；无底盘、cmd_vel或自启动。Mac 隧道使用本地8766（8765被其他本地服务占用），实际 WebSocket握手通过。骨架和RGB并开时本机实测Bodylist不出数据，当前启动默认骨架流；Image面板待单独排查。当前会话已手动启动该链路，Foxglove客户端布局仍需人在画面中叉腰后验收 TRACKING/Marker。
- Foxglove Desktop 自动深链接没有产生 Bridge 客户端连接记录；改为在应用内手动选择 Foxglove WebSocket、填写 ws://localhost:8766。在线链路当前无人时验证为 SEARCHING + Marker DELETE，实际人体 TRACKING/Marker 待用户配合叉腰。
- 本轮继续联调确认：Jetson bridge 持续监听 127.0.0.1:8765，并已广播 `/perception/target_state`、`/perception/target_marker`、`/bodylist`；Mac 8766 隧道可达。Foxglove 当前界面曾回到“没有数据源”，需通过“打开连接”重新选择 `ws://localhost:8766`，再验收面板消息。
- 用户指出需要 SSH ROS 小车数据源；新增 `foxglove/ssh-ros-datasource.json` 与 `scripts/connect_foxglove_roscar.sh`，明确 `roscar-wifi`、ROS 域 182、远端 bridge 8765 和本地 WebSocket 8766。该配置已写入仓库，但尚未发布到外部服务或远端仓库。
- 用户要求迁入示范中的语音模块：已将 `wheeltec_mic`（含 `wheeltec_mic_msg`、`wheeltec_mic_ros2`）、`wheeltec_mic_aiui` 和 `tts_make_ros2` 原样复制到 `ros2_ws/src/`，保留讯飞/AIUI 原生库、ASR/TTS 资源和反馈音频，排除 Git/Python 缓存。三个顶层包新增 `COLCON_IGNORE`，默认不参与主动工作区构建，未接入 `perception_bringup`，未启动麦克风、声卡、语音节点或底盘控制。迁入包尚未编译或实机验证；依赖实际麦克风串口、ALSA 声卡、厂商动态库和讯飞配置。

- 2026-09-14 20:12 修复 Foxglove 断连：Mac 8766 无监听，重建 SSH 隧道后客户端自动恢复。修正掩码适配器尺寸：厂商将 640×480 下采样为 320×240，Maskdata 固定 76800 项；旧 640×480 检查丢弃所有帧。Jetson 编译成功，6 秒收到 164 帧 mono8 图像，Foxglove 已显示；当前人数 0、黑色掩码、SEARCHING，真人轮廓/锁定仍待验收。链路以 RGB_STREAM=false 手动运行，无底盘；SDK 旧进程未响应 TERM，经核对 PID 后 KILL 清理才重启。
- 用户随后要求用小车 Wi-Fi 地址替代 localhost。Bridge 已改为默认监听 `0.0.0.0:8765`，Mac 直连 `ws://192.168.1.240:8765`；TCP 与 foxglove.sdk.v1 WebSocket 101 握手实测通过。`connect_foxglove_roscar.sh` 现检查直连，SSH 隧道脚本仅作离开当前 Wi-Fi 后的备用。该端口可被同一局域网设备访问；仍为手动启动、未设自启动、未启动底盘。
- 20 秒真人锁定验收中，Bodylist 521/521 帧检测到人体，掩码 528/528 帧有前景，出现人体 ID 135、237；但未触发叉腰，适配器仍锁定旧 ID 96，554 条 TargetState 全为 LOST，无有效位置或 Marker ADD。链路和人体分割正常，本次目标锁定未通过；需要再次保持标准叉腰姿势并采集关节条件，另需关注单人 ID 在窗口内变化的问题。
- 随后 15 秒复测通过真人锁定验收：人体 ID 41 有 404 帧，33 帧满足全部叉腰条件；约 1.93 秒进入 TRACKING，363 条 TargetState 均位置有效，距离约 0.865–1.264 m、偏角约 -0.140–0.007 rad，目标球和检测体积框各 363 条 ADD。Foxglove 客户端同步显示 status=2、target_id=41、position_valid=true、人体掩码和 3D 面板。当前 Foxglove 标签仍显示旧 localhost 数据源，用户表示自行改为已验证的 `ws://192.168.1.240:8765`。
- 正式 A route 接入完成：perception_bringup 的 `route:=astra` 改为 bodylist_adapter，方案 A 组合脚本也通过该 route 启动。Jetson 两包编译成功；合成 A 状态机测试、A/B/demo 与非法 route 回归均通过。当前在线进程已是正式 route，6 秒收到 108 条 Bodylist 和 147 条 SEARCHING，`/cmd_vel` 不存在；当时画面无人。启动脚本清理增加 3 秒后进程组 KILL 兜底，处理厂商 SDK 不响应 TERM 的情况。
- 用户要求叉腰锁定更宽松：默认阈值由厂商等价 50/100/50 mm 调整为手高于脊柱 20 mm、手肩横差小于 160 mm、肩高于手 20 mm；单帧触发改为最近 10 帧中 3 帧投票。五项参数由正式 astra launch 暴露。Jetson 两包编译、7 个逻辑测试和正式 route 合成测试通过；本机 Linux ARM64 Humble 容器五包编译、7 个逻辑测试、正式 A 合成测试、A/B/demo 与非法 route 回归也全部通过。小车因没电离线，在线进程尚未重启加载新值。
- 已整理 `docs/物理串口协议说明.md` 并生成可交付 ZIP，包含底盘串口字节表、ROS 映射、厂商源码、消息定义和配置。当前串口包仍未编译或实机验证，方案 A 没有打开串口或发布 `/cmd_vel`。静态审查发现厂商安全新协议帧尾赋值被注释、机械臂路径构造 10 字节却发送 11 字节，启用前必须修复并核对固件协议。
- 已核对用户提供的 `R550_C30D(2.0)_Mini小车STM32源码_GMR编码器_2026.08.21.zip`：这是 STM32F407ZG + FreeRTOS 的 R550/C30D 2.0/GMR 下位机工程，115200、11 字节控制帧、24 字节基础回传、`0x7B/0x7D` 和异或 BCC 均与迁入的 ROS 2 驱动匹配，可视为当前底盘的对应固件候选。固件通过电位器选择 Mec/4WD/MecV/4WDV 等模式，仍须上电核对 OLED/实物档位。安全 `0xB0/0xB1` 和机械臂 `0xAA/0xBB` 扩展未在该固件中检出对应解析，不计入基础协议匹配结论。
- 2026-09-14 语音方案 B 已新增 `xfyun_speech`、`deepseek_ros2`、`voice_command_router` 三个主动包：讯飞流式 IAT、DeepSeek 文本桥和受限工具路由。Orin 实测 XFM-DP-V0.0.18 可用 `plughw:CARD=XFMDPV0018,DEV=0` 采集 16 kHz/16-bit/单声道音频，三包原生 Humble 编译成功，真人语音 → 讯飞 IAT → `/voice/asr_text` → DeepSeek 中文回答已跑通。静音底噪峰值约 2441，阈值由 500 调至 2800，并增加连续 160 ms 起音判定；本地 VAD 未确认语音时丢弃云端误识别文本。TTS 与蜂鸣器启动参数默认 false；蜂鸣器属于下位机，本轮延后，不使用 Jetson GPIO，不启动底盘。凭据只存于板子权限 0600 的私有文件，不得写入仓库或日志。

- 嘈杂车内环境改用精确 `/voice_words = 小车唤醒` 事件授权一轮识别，不直接信任厂商会对多种 AIUI 事件发布的 `/awake_flag`；交互为“小微小微”后停约 1 秒再提问。手动服务触发仍要求本地 VAD，纯噪声云端文本不会进入 DeepSeek。启动脚本仅附带厂商串口唤醒节点，不启动其离线识别、反馈音频、运动控制或灯光控制。DeepSeek 最终回答同时发布 `/voice/assistant_text` 并追加到 `/home/wheeltec/ROSCAR/logs/deepseek_responses.jsonl`，只存 UTC 时间和回答正文。
- 新增方案 A 一键管理：Jetson 使用 `scripts/route_a.sh start|stop|restart|status|logs`，Mac 使用 `scripts/route_a_remote.sh`，也可双击根目录 `启动方案A.command`。入口后台管理现有 bodyreader + 正式 astra route + Foxglove Bridge，默认 `RGB_STREAM=false` 以保留已验证的骨架输出；不启动底盘、不发布 `/cmd_vel`。已通过 Bash 语法、ShellCheck、模拟进程生命周期和模拟 SSH 调用检查。
- 新增并已安装 `roscar-route-a.service` 与 `scripts/install_route_a_autostart.sh`，用于 Jetson 开机自动启动方案 A，并在前台进程异常退出后延迟 5 秒重启。服务固定使用 `wheeltec` 和 `/home/wheeltec/ROSCAR`，保持 `ROS_DOMAIN_ID=182`、`RGB_STREAM=false`；手动入口会识别 systemd 服务，避免重复启动。
- 2026-09-15 实际部署结果：同步白名单已加入 `deploy/systemd/`；Jetson 服务为 `enabled/active/running`、`NRestarts=0`，骨架、正式 astra 适配器和 Foxglove Bridge 均在其 cgroup 内。8765 从 Mac 可达，五个感知话题存在，`TargetState` 为真实 Astra 的 SEARCHING，`/cmd_vel` 不存在。尚未为验收而重启整车，因此“重启后自动拉起”目前依据 systemd enabled 状态，未做断电重启验证。
- 2026-09-15 用户将范围扩展到无避障人体跟随。`astra_body_adapter` 新增默认禁用的 `person_follower`，将已验证 `TargetState` 转换为 `/cmd_vel`；仅 TRACKING+有效+未超时时动作，2.0 m 目标距离、0.15 m/s 前进上限、0.5 rad/s 转向上限、不倒车。Jetson 已构建并运行21项逻辑测试；底盘与跟随以临时用户服务运行，并执行过1秒 0.15 m/s 的受限直行指令及后续零速度。未由远程日志证明实际位移，不得描述为完整实车跟随验收；当前无避障，且 STM32 指令丢失保护仍需独立验证。
