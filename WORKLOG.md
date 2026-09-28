# 工作记录

## 2026-09-14：生成香港 VPS 独立 Clash 配置

- SSH 检查确认 `8.217.15.181` 为 Ubuntu 22.04.5，现有 Xray 与 sing-box 服务正常运行，端口和 UFW 规则已存在；未重启、覆盖或修改原有业务。
- 从服务器现有模板生成 `deploy/clash-hk-vps.yaml`，将节点地址统一固定为 `8.217.15.181`，保留 5 个 VMess 出口和 2 个 Hysteria2 节点，不包含其他 VPS 或订阅地址。
- 最小审查：确认 YAML 中包含 7 个目标 IP、无旧域名 `luxurira.cc`，并核对端口与服务器监听状态。配置含敏感凭据，仅存于项目部署文件，未写入 SSH 配置。

## 2026-09-13：厂商 Humble 源码初查

### 范围与结果

检查 `JP6.2_wheeltec_ros2_src_20260903/` 的包清单、相机启动链、检测与跟随程序、底盘接口和关键二进制架构。未修改厂商代码，未运行机器人节点。

- 114 个 package.xml 均可解析，未发现重复包名。此检查不能证明依赖齐全或编译成功。
- `ros2_astra_camera-master/astra_camera` 包含 OpenNI 驱动与 ARM64 动态库；`file` 确认 `openni2_redist/arm64/libOpenNI2_astra.so` 为 Linux AArch64 ELF。
- 同时附带 `OrbbecSDK_ROS2-main`；当前 `turn_on_wheeltec_robot/launch/wheeltec_camera.launch.py:29` 实际选择的是 `astra_camera`。需根据设备型号选择驱动，不同时打开同一相机。
- `turn_on_wheeltec_robot/config/wheeltec_param.yaml:17` 默认车型为 `mini_mec`，第 50 行相机为 `astra_pro`，不能据此认定实物配置。
- `astra_pro.launch.xml:5` 默认不启用深度配准，第 48 行不启用颜色/深度同步。人体 RGB 检测框测距前须验证配准与时间对应，不能直接假定同像素即同位置。
- 相机总启动文件已有 RGB JPEG 与深度 compressedDepth 重发布；深度压缩流被映射到 `/camera/depth/image_raw/compressed`，接入显示端时应确认编码支持，不能按普通 JPEG 处理。
- 相机启动文件传入 `enable_d2c_viewer=True`，但本份相机 C++ 实现未搜索到对应参数使用，不能据此断言它会弹窗。
- `turn_on_wheeltec_robot/src/wheeltec_robot.cpp:761` 起发布 `odom`、`imu/data_raw`、`PowerVoltage`，第 790 行订阅 `/cmd_vel` 的 Twist，经厂商串口帧发送到底盘。电压不是电量百分比，里程计速度不是独立左右轮速。
- 静态搜索未发现 Foxglove 桥接或布局配置；桥接可以后续安装，车端已有基础话题可复用。

### 跟随相关模块

- `simple_follower_ros2/simple_follower_ros2/visualTracker.py` 是 HSV 颜色目标跟踪加深度测距，不是人体识别。
- `wheeltec_bodyreader/bodyreader` 是 Astra SDK 骨架检测、姿态锁定与跟随链；包含 ARM64 的 `libastra.so` 和 `libOrbbecBodyTracking.so`。
- `wheeltec_bodyreader/bodyreader/src/main.cpp:254` 的授权字符串仍为占位符，SDK 授权状态及运行兼容性待核实，不能断言现成示例一定可用或一定不可用。
- `wheeltec_bodyreader/bodyreader/src/follower.cpp:12` 跟随距离固定为 2000 mm，速度限制为 ±0.5 m/s，参数只在启动时读取。不能声称 Foxglove 参数修改会即时应用。
- 跟随节点仅在目标消息回调中控制，没有数据超时定时停车；切换睡眠模式也未立即发零速度。`bodydata_process.cpp:222` 有未锁定时发零速度的处理，但这不能覆盖上游断流；下位机超时停车能力尚未确认。
- `ultralytics_ros2/ultralytics_ros2/detection_node.py` 是 YOLO predict 检测，发布 `detected_image` 和 `detections`，未集成目标跟踪、深度或速度控制。标注图发布时未复制输入 header，后续需补时间戳与 frame_id。
- `ultralytics_ros2/launch/yolo.launch.py:11` 默认使用绝对路径的交通标志权重，输入为 `/image_raw`，与 Astra 的 `/camera/color/image_raw` 不同。附带 `model/yolo11n.pt` 等权重，但尚未加载验证；setup.py 未安装 model 目录。

### 建议下一步

初查时建议同时考虑感知与底盘。用户随后明确先不考虑下位机，当前顺序更新为：确认 Jetson 系统与相机 USB 标识，建立 ROS/CUDA 环境，验证相机、人体检测、目标跟踪与深度测距，再通过 Foxglove 展示。底盘通信、控制仲裁及车辆停车验证留待后续阶段。

### 最小审查

已回读关键源码，核对 114 个包清单和 3 个关键动态库架构；区分了源码事实、建议和实机未知项。当前仅在 Mac 做静态检查，未做 Jetson 编译、性能测试或硬件动作测试。

## 2026-09-13：整理人体感知方案文档

- 根据用户要求新建 `人体跟随感知方案.md`，整理硬件与版本基线、Astra SDK 骨架流程、结构光深度测距、YOLO11 模型选择、目标跟踪、Foxglove/SSH 配置和分阶段验收。
- 记录源码中双手叉腰锁定人体 ID、使用质心定位的逻辑；区分二维 Pose 关键点与三维位置。
- 明确现有实现与建议设计的边界，未承诺实测帧率，未把模型附带权重或 ARM64 库视为已运行验证。
- 同步更新 AGENTS.md，将当前范围收敛到感知和可视化，暂不涉及下位机。
- 最小审查：检查文档本地路径、Markdown 代码围栏、行尾空白及三份文档的范围一致性。本次仅更新文档，未修改厂商源码或执行部署命令。

## 2026-09-14：核对骨架跟随与卡尔曼滤波

- 回读 main、bodydata_process、follower、display 和整车 EKF 启动文件，确认人体 ID 锁定、平均 RGB 恢复、质心定位与简化 PD 的链路。
- 可见人体应用层无显式卡尔曼实现，Astra SDK 内部算法无法从现有源码确定；整车 robot_localization EKF 是独立的自身状态估计。
- 核对 Ultralytics 官方 ByteTrack 实现，其内置卡尔曼用于检测框状态，并不自动融合深度。将该区别与可选空间位置滤波设计补充到方案第 10 节。
- 同步更新 AGENTS.md；最小审查核对源码依据、文档格式和相机自运动/测量有效性的说明。未改动厂商代码、未执行动态测试或硬件操作。

## 2026-09-14：绘制两条技术路线 SVG

- 新建 `docs/diagrams/方案A_Astra骨架跟随.svg`：展示 SDK、骨架输出、姿势锁定、质心、颜色恢复及待补全的感知/可视化功能。
- 新建 `docs/diagrams/方案B_YOLO深度跟随.svg`：展示现有相机与检测基础，以及待接通的人体配置、ByteTrack、深度区域提取、空间位置和 Foxglove。
- 图中注明所有已有实现尚未实机验证、SDK 内部卡尔曼未知、ByteTrack 卡尔曼不自动融合深度；下位机留到后续阶段。
- 更新主文档链接及 AGENTS.md。完成 SVG XML/链接检查与渲染排版审查，本次未修改厂商代码。

## 2026-09-14：导出两方案合并 PNG

- 将两张 SVG 以 2 倍宽度渲染，再按原比例、顶部对齐并排合成 `docs/diagrams/人体跟随两方案对比.png`，尺寸 5780×3800。
- 使用 rsvg-convert 和 Pillow 完成格式转换与拼接，保留两张图的全部内容；源 SVG 未修改。
- 同步更新主文档和 AGENTS.md。最小审查：验证 PNG 可解码、尺寸及链接，查看整体预览确认文字正常、无裁切。

## 2026-09-14：建立初始开发框架

- 保留原厂 2.4 GB 源码及淘宝资料原路径，增加 .gitignore 将原包、模型、录像、构建目录和本地部署配置排除普通 Git；未删除或改动原包。
- 新建 ros2_ws/src 四个包：person_interfaces 公共观测消息；astra_body_adapter 和 yolo_person_tracker 的 NOT_READY 入口；perception_bringup 单路线启动和独立 synthetic demo。
- 消息明确 source、is_simulated、状态、位置有效性、坐标、观测时间与数据年龄；无效距离为 NaN，框架不发布 cmd_vel。
- 新建 README、架构/接口/开发计划/原厂索引、Foxglove 面板计划、Mac/Jetson 同步说明。Foxglove 客户端布局尚未导入验证，真实相机与算法仍未接入。
- 新建构建、结构检查、同步预览、模型校验复制及容器测试脚本。复制附带 yolo11n.pt 到被 Git 忽略的 models/weights，校验 SHA-256，未加载执行权重。
- 环境补齐：Docker Hub 与镜像源直连超时，使用临时 crane 工具经主机现有代理获取官方 Linux ARM64 Humble 镜像，再导入 Docker；未修改用户代理配置。基础镜像 ID：sha256:d81954770c20b5b114ec07a92e9922146e91a6373f91876bd09e3cd118b0c39c。
- 验证：四个包 colcon build 成功；A/B 各收到 2 条 NOT_READY 消息；demo 收到 90 条并覆盖 TRACKING/LOST，验证模拟标记、坐标、有效性与时间；非法 route 被拒绝；未发现 cmd_vel 话题。测试日志位于本地 artifacts/humble-test.log。
- 同步脚本 3 项单元检查通过：默认 dry-run、显式 apply、危险目录拒绝。Python/XML/JSON 结构、Shell 语法、文档本地链接及模型 Git 忽略检查通过。
- 本次未创建远端、提交或推送，不向 Jetson 部署。尚待实际设备环境、相机、SDK 授权与 GPU 验证。

## 2026-09-14：迁入下位机串口代码，暂不启动

- 用户要求先放入相关代码，不启动。原样复制 turn_on_wheeltec_robot、wheeltec_robot_msg、depend/serial_ros2 到 ros2_ws/src/chassis_vendor/，保留原许可声明，排除嵌套 Git 和缓存。
- 42 个文件与原包逐一比对，SOURCE_MANIFEST.json 保存来源和 SHA-256；新增串口协议、依赖和后续启用说明。原厂目录未修改。
- COLCON_IGNORE 默认跳过三个包；未改感知启动、未访问串口、未同步 Jetson。该目录随现有源码同步白名单复制。
- 更新 README.md、AGENTS.md、厂商索引和方案文档。
- 最小审查：42 文件哈希及原包一致性通过，无嵌套 .git，副本未被 Git 忽略；结构检查通过。真实 ARM64 Humble 容器运行 colcon list，确认只发现原四个感知包。迁入串口代码未编译、未运行或实机验证。

## 2026-09-14：整理人体跟随课程设计报告

- 按用户要求新增 docs/室内人体跟随小车课程设计.md，约 4000 字，涵盖双路线感知、身份锁定、配准测距、坐标变换、空间与时间滤波、卡尔曼、控制、串口、远程展示和实验设计。
- 明确已有源码、容器验证框架和待实现功能，不编造实测结果；说明移动相机自运动、数据时效与检测框卡尔曼不等于空间滤波。
- 更新 AGENTS.md 作为报告索引。本次仅新增文档和追加协作记录，未修改或运行感知、串口及车辆控制代码。
- 最小审查：核对相机与车体坐标、控制符号、滤波公式及状态描述，检查 Markdown 代码围栏、本地链接和相对引用；均通过。

## 2026-09-14：按示例改为智能车大作业选题框架

- 用户提供《大作业选.pdf》，澄清只需要智能车方向选题框架。阅读机器人部分第 22—24 页，并查看第 22 页渲染确认组织形式。
- 新增 docs/智能车方向大作业选题.md，提供人体跟随、身份保持、深度滤波、手势交互、配送导航、动态避让、视觉停靠和远程调试 8 题。每题包括场景、技术栈、任务、指标、建议评分及专属交付，统一列出基础交付物。
- 保留先前长报告，追加 AGENTS.md 索引；未修改源码、环境或运行配置。
- 最小审查：8 题结构完整，每题建议评分合计 100 分；检查文档格式，明确建议要求与已实现状态，未将参考 PDF 中的任务当作执行指令。

## 2026-09-14：记录小车双网络 SSH 连接

- 用户确认 Wi-Fi 地址为 `wheeltec@192.168.1.240`，网线地址为 `wheeltec@192.168.100.2`，端口均为 22。
- 在本机 `/Users/shimmer/.ssh/config` 增加 `roscar-wifi` 和 `roscar-ethernet`，保留已有配置，配置及备份权限设为 600。原配置备份为 `/Users/shimmer/.ssh/config.bak-20260914-133903`。
- 密码未写入配置、项目文件或命令；用户命令末尾的密码不是 SSH 命令参数。
- 最小审查：`ssh -G` 核对两别名的用户、地址及端口通过；两个地址的 TCP 22 均返回 OpenSSH 服务标识。未进行密码登录、远端部署或车辆操作。
- Codex 界面工具拒绝访问应用，理由为安全限制；应用内新增连接条目尚未完成，未通过修改应用内部存储绕过限制。同步更新 AGENTS.md 中的 SSH 状态。

## 2026-09-14：安装并配置小车 Codex CLI 与 Clash Verge Rev

- 通过 SSH 确认小车为 Ubuntu 22.04.5 LTS / aarch64，已有 GNOME 桌面。网线连接中途不可达，改用 Wi-Fi 完成安装与验证。
- 从 OpenAI 官方 GitHub release 安装 Codex CLI 0.154.0 ARM64 musl 完整包，包含配套运行资源；安装路径为小车 `/home/wheeltec/.local/share/codex/0.154.0`。包 SHA-256：`97d93e11df72d3c26772db019e6ea8bb72c246500d46b98c760839f3240355e6`，下载校验通过。
- 安装 Clash Verge Rev 2.5.2 官方 arm64 DEB，SHA-256：`598a5a852d7bf9dc40a976780ef2afc9a4e5bfe7b99533e5f956f9e2f9def72f`，下载校验通过。APT 补齐 WebKitGTK 4.1、JavaScriptCore、libsoup 依赖和 ripgrep；保留原 Node.js 环境。
- 导入用户订阅（102 节点、6 组），配置规则模式、本机 7897 端口、关闭 TUN/局域网代理访问、GNOME 系统代理及桌面登录自启动。比较节点后保存“新加坡-IEPL 01”，重启 Clash 后确认恢复该选择。
- Codex 启动器 `/home/wheeltec/.local/bin/codex` 设置本机代理及内网 NO_PROXY；备份并更新 `.bashrc` 的 PATH，新 SSH 会话可直接调用。订阅与登录凭据未写入项目文件，临时订阅副本已删除，实际配置保存在小车私有目录。
- 用户完成账号授权后，`codex login status` 返回 `Logged in using ChatGPT`。最小实际请求采用 read-only 沙箱、不调用工具，默认模型返回 `OK`，进程退出码 0。
- 最小审查：核对版本、Clash 运行状态、重启后的选中节点、自启动文件、loopback TCP/UDP 7897、关闭 TUN、配置权限；OpenAI 登录服务 HTTP 200，未带密钥的 API 探测 HTTP 401。更新 AGENTS.md 和 deploy/README.md，检查文档格式与敏感信息边界。
- 未重启小车，未部署项目 ROS 源码、运行相机算法或操作车辆；桌面自启动配置不等于无桌面登录的开机服务。

## 2026-09-14：SSH 环境核查

- Wi-Fi SSH 成功，核对系统、JetPack/CUDA/TensorRT/cuDNN、ROS、资源与 USB，结果见 docs/小车环境检查.md。
- 实际执行 PyTorch GPU 张量运算、torchvision CUDA NMS，均通过。未加载模型、未启动硬件节点。
- 记录 ultralytics / Foxglove 缺口和 libnvjpeg.so.12 图像扩展警告；相机 USB 标识为 ASTRA S，不能沿用未确认的 Astra Pro 配置。
- 更新 AGENTS.md。最小审查：报告逐项核对 SSH 输出，区分包可发现、GPU 算子运行和未验证的相机/整套算法；未记录凭据。

## 2026-09-14：首次部署到 Jetson

- 用户授权部署必要代码。预览后同步到新目录 /home/wheeltec/ROSCAR，未删除远端文件或改动厂商工作区。源码、文档、测试及串口暂存副本已部署；未传模型权重和原厂大包。
- 同步脚本将 deploy 改为明确文件白名单，避免复制代理节点 YAML。
- Jetson 原生 Humble colcon 编译四包成功（16.5s）。在 ROS_DOMAIN_ID=182、ROS_LOCALHOST_ONLY=1 下运行 A/B 占位和 demo 测试：astra/yolo 各 2 条，demo 88 条，非法 route 拒绝，全部通过，测试进程退出。未启动相机、串口、车辆运动或设置自启动。
- 最小审查：本地结构检查及 3 个同步单测通过；串口 COLCON_IGNORE 保留。更新部署说明和 AGENTS.md。真实人体感知未实现，此次通过的是框架运行检查。

## 2026-09-14：方案A分段联调与错误日志

- 3个Sol子代理分别核查相机、SDK、实现独立适配器。深度实机收到345帧640×480/16UC1和344条内参。SDK main报0x50000a19授权无效，用户待找厂商确认。错误摘录已保存，未包含授权密钥。
- 新增bodyreader_msg、bodylist_adapter与测试，Jetson五包编译24秒成功；5个逻辑单测、合成ROS状态测试和原A/B/demo回归通过。第一次合成ROS测试发现固定6人体数组赋值错误，修正测试后通过。
- Foxglove依赖经Mac镜像下载再传Jetson，SHA256核对apt元数据成功；用户目录解包运行，无系统sudo改动。本地代理7897未监听。桥接首次旧协议探针400，核对库内协议后foxglove.sdk.v1返回101，通过握手，未验收客户端布局。
- 所有本轮硬件/适配/桥接测试已退出，未启动底盘或新增自启动。更新AGENTS、方案、部署和联调文档；最小审查为结构检查、shell语法、逻辑测试与实机合成ROS回归；真实骨架仍受授权阻塞。

## 2026-09-14：方案 A 人体实测复测

- 用户在相机前重测，单独运行 bodyreader/main 40 秒：1004 条 Bodylist 中 707 条检测到 1 人，ID 93；有效质心 707 条、有效关节 706 条，最大 19 关节，叉腰判定 1 帧。深度质心 Z 约 0.99–1.18 m。
- SDK 仍输出 Invalid Orbbec Body Tracking license（本轮 0x50000719），但真实人体骨架、ID 与质心已输出；将其从“当前功能硬阻塞”修正为待厂商说明的异常/授权提示。未对长期授权或部署合规性作推断。
- 测试停止后无 bodyreader 进程；未启动下位机、bodydata_process、follower 或任何车辆控制。

## 2026-09-14：方案 A Foxglove 可视化链路

- 新增标准 Marker 输出：bodylist_adapter 对有效锁定目标发布 `/perception/target_marker`（绿色球），无有效目标发布 DELETE；Foxglove 可用 Raw Messages、Plot 和 3D 面板，不需要自定义插件。
- 新增手动启动脚本 `run_astra_foxglove.sh`，只启动 bodyreader/main、bodylist_adapter、Bridge；各子进程使用独立进程组，脚本收到结束信号时清理整组。默认只开骨架流，避免当前已观察到的 RGB+骨架同时启用时 Bodylist 不出数据的问题。未设置自启动。
- 本机8765被无关服务占用，因此新增 `open_foxglove_tunnel.sh`，使用 Mac 127.0.0.1:8766 → Jetson 127.0.0.1:8765。端到端 `foxglove.sdk.v1` WebSocket握手返回101。
- Jetson重新编译5个包（7.87秒）。合成ROS测试验证目标 Marker ADD/DELETE、状态与无 cmd_vel；真实运行链路8秒收到242条 Bodylist和238条 TargetState，当前无人时为 SEARCHING。初次进程清理只终止了 ros2 包装器，留下4个厂商SDK子进程；已精确停止这些本轮残留，启动脚本改为 setsid 进程组，后续清理覆盖子进程。
- 最小审查：Python结构检查、三个脚本bash语法、Jetson编译、合成ROS适配测试、真实Bodylist→TargetState计数与Mac隧道握手通过。Foxglove桌面客户端面板及真人 TRACKING/3D Marker待用户站入画面并叉腰后验收。

- 后续在线复核：5 秒收到153条 Bodylist、119条 TargetState；30 秒可视化状态监测在无人画面时收到 SEARCHING 894 条、Marker DELETE 890 条，符合未锁定语义。Mac 8766 隧道握手成功；Foxglove Desktop 的自动深链接未产生 Bridge 客户端连接记录，需在应用中手动选择 Foxglove WebSocket 并填写 `ws://localhost:8766` 后完成面板验收。

## 用户在场骨架测试（2026-09-14 17:17）

用户确认已就位后，从SDK lib工作目录单独运行main，关闭RGB，隔离ROS域182。40秒测试窗口收到1005条Bodylist，positive_frames=0、max_count=0、IDs为空、有效质心和叉腰均0。启动仍报0x50000739 Invalid Orbbec Body Tracking license（与上一轮错误码不同，文字相同）。当前未识别到人体；不能单凭结果把原因确定为授权，仍需核对深度视野、SDK配置与厂商指定程序。退出阶段另有ROS publisher析构错误，应与识别失败分开看。测试已停止，底盘未启动。

## 2026-09-14：继续打通 Foxglove

- 复核 Jetson bridge 监听 127.0.0.1:8765 并广播 target_state、target_marker、bodylist；Mac 8766 隧道可达。
- Foxglove 当前界面显示“没有数据源”，bridge 日志显示客户端曾连接后被重置；README 已补充重新连接与两端检查命令。
- 尚未完成真人 TRACKING/3D Marker 最终画面验收。

## 2026-09-14：补齐 SSH ROS 小车数据源

- 新增 `foxglove/ssh-ros-datasource.json`，明确 Foxglove WebSocket、`roscar-wifi` SSH、远端 8765、本地 8766、ROS 域 182 和布局文件。
- 新增 `scripts/connect_foxglove_roscar.sh`，可重复建立或复用 SSH 隧道并输出 Foxglove 连接地址。
- 验证 JSON、Shell 语法及现有 8766 隧道复用成功。配置已写入本地仓库，未发布到外部服务或远端 Git。

## 2026-09-14：迁入示范语音模块，暂不启用

- 用户要求迁入项目示范中的语音代码。原样复制 `wheeltec_mic`（含 `wheeltec_mic_msg` 与 `wheeltec_mic_ros2`）、`wheeltec_mic_aiui`、`tts_make_ros2` 到 `ros2_ws/src/`。
- 保留语音识别、唤醒、命令识别、AIUI、TTS、讯飞 ASR/TTS 资源、原生库和反馈 WAV；排除 `.git`、Python 缓存以及构建产物。迁入规模约 420 MB。
- 三个顶层包新增 `COLCON_IGNORE`，因此默认结构检查和 `colcon build --base-paths src` 仍只覆盖主动感知包；未接入 `perception_bringup`，不启动 `/dev/wheeltec_mic`、ALSA 声卡、`cmd_vel` 或车辆控制。
- 依赖与风险：需要实际 M2/M07/NEW_M2 麦克风、串口别名、声卡名称、ALSA/采样率库、厂商动态库和讯飞/AIUI 配置；当前仅完成文件迁入，未编译、未连接硬件、未做语音识别或 TTS 实测。
- 最小审查：迁入目录无嵌套 `.git` 或 Python 缓存；`scripts/check_project.py` 已调整为跳过带 `COLCON_IGNORE` 的包；待运行结构检查确认其余项目完整性。

## 2026-09-14：尝试接入 RGB 与检测框可视化

- 启动脚本改为 `RGB_STREAM` 可配置，默认 true；Jetson 已部署并重启验证。
- `bodyreader` 当前声明 `/image_raw` 为 `sensor_msgs/msg/Image`，但实测无图像帧；同时 RGB 开启后 `/bodylist` 也未发布，符合此前 RGB+骨架冲突现象。不能把空话题描述为摄像头图像已打通。
- `bodylist_adapter` 新增 `/perception/detection_box` 3D CUBE Marker，代表目标人体的近似三维体积；明确不是经过标定的二维图像检测框。
- Foxglove 布局增加 `/image_raw` Image 面板，并把无效的 `3D Panel` 修正为 `3D`，同时显示目标球和检测体积框。
- Jetson 适配器编译成功；当前剩余阻塞是厂商 bodyreader 的 RGB/骨架并发输出，需要继续核对相机驱动或厂商参数。

## 2026-09-14：增加独立 Astra 相机入口

- 按“本地修改后再同步远程”执行：本地新增 `scripts/run_astra_camera.sh`，同步前完成 Shell/JSON 检查，再部署到 Jetson。
- Jetson 已确认存在 `astra_camera astra_camera_node`，能识别 ASTRA S（USB 2bc5:0402，序列号 17121710036）。短时测试显示驱动默认 depth/IR/color 均未启用，且与 bodyreader 并行会发生 Resource busy；尚未把该节点并入主启动脚本。
- 当前图像链路仍未完成：需要继续确定厂商驱动的正确启流参数或使用其 launch/config；不能把空 `/image_raw` 话题当作图像已发布。

## 2026-09-14：掩码与检测框可视化桥接

- 本地新增 bodylist_adapter 的 `/perception/body_mask_image`，将 SDK `/body/mask` 的 640x480 int32 掩码转换为 `sensor_msgs/Image` mono8；检测体积框继续发布 `/perception/detection_box`。
- Foxglove 布局图像面板改为 `/perception/body_mask_image`，不再依赖空的 `/image_raw`。
- 本地 Python/JSON 检查通过，按约定同步到 Jetson，`astra_body_adapter` 原生 Humble 编译通过。
- 已启动远程骨架链路；本轮 `ros2 topic list` 查询受到远端 ROS CLI daemon `!rclpy.ok()` 异常影响，需下一轮清理 daemon 后补做掩码频率验证。未修改厂商 SDK。

## 2026-09-14 20:12：修复 Foxglove 断连及掩码零输出

- 断连证据：Mac 8766 没有监听，小车 bridge 仍监听 127.0.0.1:8765。运行 scripts/connect_foxglove_roscar.sh 重建隧道，Foxglove 自动恢复，问题提示清空，目标消息刷新。
- 补齐客户端空白 Raw Messages 面板为 /bodylist；随后界面图像主题已选择 /perception/body_mask_image，但仍等待帧。直接 rclpy 采样避开 CLI daemon：5 秒原始 mask 154 条、Bodylist 153 条、TargetState 148 条，图像 0 条。
- 根因：Maskdata.msg 固定 int32[76800]，厂商 output_body_mask 对 640×480 SDK 掩码横纵各抽样 2 倍，实际为 320×240。适配器旧长度检查为 640×480，静默丢弃每帧。修正长度、width、height、step 为实际尺寸，未修改厂商代码。
- 本地语法检查后同步该 Python 文件；Jetson 原生 Humble 编译 astra_body_adapter 成功（3.59 秒）。重启原感知链路时 SDK PID 5947 未响应 TERM，核对后 KILL 清理，以 RGB_STREAM=false 重新手动启动。未启动底盘或设置自启动。
- 最小审查：核对消息定义与厂商采样循环；修复后 6 秒收到原始 mask 178、图像 164、Bodylist 178、TargetState 166 条。断言图像为 320×240/mono8、step 320、76800 字节通过。Foxglove 实际画面无连接错误/等待图像提示，显示黑色掩码，两个原始消息面板持续刷新。当前人数 0、前景像素 0、SEARCHING；尚未验收真人轮廓与锁定。

## 2026-09-14 20:16：改用小车 Wi-Fi 地址直连 Foxglove

- 按用户要求取消 localhost 作为主连接方式。`run_foxglove.sh` 默认监听 `0.0.0.0:8765`，数据源更新为 `ws://192.168.1.240:8765`；`connect_foxglove_roscar.sh` 改为检查 Wi-Fi 直连，不再自动建立隧道。原隧道脚本保留为异网备用。
- 本地 Shell、JSON、diff 检查通过后同步小车并重启感知链路，未启底盘。Jetson `ss` 实测监听 `0.0.0.0:8765`；Mac 到 `192.168.1.240:8765` 的 TCP 连接成功，`foxglove.sdk.v1` 握手返回 HTTP 101，Bridge 日志登记客户端来源 `192.168.1.238`。
- 直连仅适用于 Mac 与小车位于当前同一 Wi-Fi，且 8765 可被该局域网内设备访问。未设置自启动或公网转发。

## 2026-09-14：方案 A 真人锁定验收未通过

- 用户发出“开始”后采样 20 秒：Bodylist 521 帧且全部检测到人体；人体掩码 528 帧且全部有前景，说明相机、SDK 人体分割、适配器和 ROS 链路在线。
- 窗口内观测到人体 ID 135、237，但叉腰判定没有触发；适配器继续保留旧锁定 ID 96。554 条 TargetState 全为 LOST，`position_valid` 始终 false，距离/偏角无有效值，目标球与检测体积框均无 ADD。
- 随后的 6 秒关节诊断窗口已无人，因此没有取得失败条件的关节样本。本次不能判定具体是哪一项姿势阈值未满足。下一轮需人在画面中保持双手叉腰，同时实时统计六项厂商姿势条件。
- 最小审查：计数、状态转换和 Marker 语义互相一致；未修改算法、未启底盘。本次结果证明检测与掩码链路正常，不代表目标锁定验收通过。单人窗口出现两个 SDK ID 也需在后续稳定性测试中复核。

## 2026-09-14：方案 A 真人锁定复测通过

- 用户再次保持叉腰后采样 15 秒。SDK 人体 ID 41 共 404 帧，其中 33 帧同时满足厂商六项叉腰阈值；约第 1.93 秒适配器从旧目标切换并进入 TRACKING。
- 采到 363 条 TRACKING，全部 `position_valid=true`；水平距离范围约 0.865–1.264 m，偏角范围约 -0.140–0.007 rad。`/perception/target_marker` 和 `/perception/detection_box` 各收到 363 条 ADD，统一状态、数值和可视化 Marker 一致。
- Foxglove 客户端现场可见 `status=2`、`target_id=41`、`position_valid=true`、人体掩码、距离/偏角曲线和 3D 面板。客户端当前标签仍是可用的旧 localhost 隧道地址；小车 Wi-Fi 直连已另行完成 TCP 与 WebSocket 101 验证，用户表示自行把 Foxglove 地址改为 `ws://192.168.1.240:8765`。
- 最小审查：检查状态转换时间、有效位置计数、数值范围、两个 Marker ADD 计数及 Foxglove 实际消息，互相吻合。未启动底盘、控制节点或自启动。下一阶段可将已验证的适配器接入正式 `route:=astra`，同时保留后续 ID 稳定性、多人与遮挡测试。

## 2026-09-14：正式接入 route:=astra

- `perception_bringup/perception.launch.py` 的 astra route 从 NOT_READY 占位入口切换为实测 `bodylist_adapter`。默认 route 仍为 yolo，且 yolo 继续明确报告 NOT_READY；demo 仍需显式选择并标记模拟数据。
- `scripts/run_astra_foxglove.sh` 改为通过正式 `ros2 launch perception_bringup ... route:=astra` 启动适配器。厂商 bodyreader 和 Bridge 仍由脚本单独管理，不包含 bodydata_process、follower、底盘或 `/cmd_vel`。
- 补齐 astra_body_adapter 的 sensor_msgs 运行依赖及包说明。合成适配测试改为从正式 route 启动；路由回归现在要求 astra 无输入时为 STALE、yolo 为 NOT_READY。测试停止改为只向 launch 父进程发 SIGINT，避免父子同时收到信号造成清理 traceback。
- 启动脚本清理逻辑增加 TERM 后最多 3 秒等待和进程组 KILL 兜底。原因是厂商 bodyreader 实测可能不响应 TERM；该兜底仅作用于本脚本创建并记录的三个独立进程组。
- 本地检查：项目结构、5 个纯逻辑单测、Shell/JSON、SVG 解析和 diff whitespace 全部通过。方案主文档、README、接口、架构、路线图、包说明和课程设计阶段描述已更新；方案 A SVG 状态同步，并重新导出 5780×3800 两方案对比 PNG，视觉审查无裁切或重叠。
- Jetson 原生 Humble 编译 astra_body_adapter、perception_bringup 成功（两包 6.79 秒）。隔离域合成正式 A route 状态机/单位/Marker/无 cmd_vel 测试通过；A/B/demo 路由回归与非法 route 拒绝通过，第二轮退出干净。
- 当前在线链路已用正式 route 重启：进程命令包含 `route:=astra with_foxglove:=false`，Bridge 监听 0.0.0.0:8765。无人画面 6 秒收到 108 条 Bodylist、147 条 SEARCHING，`/cmd_vel` 不存在，证明正式入口正在消费真实上游而不是 demo。此前真人 TRACKING 验收无需重复冒充本轮结果；重启后目标锁定状态已清空，下一次需重新叉腰。

## 2026-09-14：放宽叉腰锁定判定

- 用户要求叉腰更容易触发。默认三项空间阈值从厂商等价的 50/100/50 mm 改为 20/160/20 mm：手高于脊柱基点最小值、手肩最大横向差、肩高手最小值。
- 增加逐人体短窗口投票：最近 10 帧中满足 3 帧才锁定或切换，替代单帧触发。人体 ID 离开当前 Bodylist 时清除其未完成历史，避免带着旧票数重新出现。空间阈值、窗口和票数均通过正式 astra launch 参数暴露，并验证非负阈值及 `1 <= min_votes <= window_frames`。
- 新增宽松姿势通过、原严格姿势不通过、孤立单帧不锁定测试；原锁定、切换、丢失、单位和无效质心测试适配投票逻辑。本地 7 个逻辑测试、结构检查、Python 编译和 diff 检查通过。
- 代码同步 Jetson 后，astra_body_adapter 与 perception_bringup 原生 Humble 编译成功（两包 5.61 秒）。Jetson 7 个逻辑测试和正式 route 合成测试通过；运行日志显示 `akimbo requires 3/10 matching frames`，状态机、Marker、单位、未知时间戳和无 `/cmd_vel` 均通过。
- 准备重启在线链路时，小车 Wi-Fi `192.168.1.240` 突然无响应，网线 `192.168.100.2` 也超时；SSH、ICMP 和 8765 均不可达。新代码已安装到 Jetson，但本轮尚未确认在线常驻进程重启并加载新参数，不能把合成测试描述为真人宽松手势验收。
- 用户确认小车没电，后续停止网络重试，改做本机离线验证。扩充 `scripts/test_container.sh`，让 Linux ARM64 ROS 2 Humble 容器在五包编译后依次运行 7 个姿势逻辑测试、正式 A route 合成状态机测试、A/B/demo 路由回归和非法 route 拒绝。五包 8.68 秒编译成功，全部测试通过，进程退出干净；完整日志保存于 `artifacts/humble-test.log`。这证明 ARM64 Humble 软件构建与合成消息行为，不替代小车上电后的真人宽松手势验证。

## 2026-09-14：打包物理串口资料

- 新增 `docs/物理串口协议说明.md`，逐字节整理速度、回充、安全、灯带、机械臂发送帧，以及基础状态、超声波、回充接收帧，并列出对应 ROS 话题、换算单位和人体跟随接口边界。
- 生成 `artifacts/ROSCAR-物理串口资料-20260914.zip`，包含说明、完整 `chassis_vendor` 迁入源码与 SHA-256 清单，不包含密码、代理配置或运行日志。
- 静态审查确认当前感知启动脚本没有打开 `/dev/wheeltec_controller` 或发布 `/cmd_vel`。另发现厂商安全新协议路径的帧尾赋值被同行注释吞掉，机械臂回调初始化 10 字节但按 11 字节发送；仅记录问题，未擅自修改原样迁入代码。
- 本轮只做资料整理、压缩包完整性和哈希验证；底盘包仍受 `COLCON_IGNORE` 隔离，未编译、未连接小车、未发送串口数据。

## 2026-09-14：核对 R550 C30D 2.0 STM32 固件

- 只读解包检查用户提供的 `R550_C30D(2.0)_Mini小车STM32源码_GMR编码器_2026.08.21.zip`；SHA-256 为 `e7b578c913104e3622bb8f16e4391d65b3fdfa10007b4f03fc98649b407f1910`。
- 工程目标为 STM32F407ZG，使用 FreeRTOS；包含四路正交编码器接口、GMR 对应参数、R550/V550 麦轮及四驱车型参数。固件按电位器 ADC 选择车型，并在 OLED 显示 Mec、4WD、MecV 或 4WDV。
- 与 ROS 2 `turn_on_wheeltec_robot` 对照确认基础链路匹配：115200；上位机发送 11 字节 `0x7B ... BCC 0x7D`；STM32 回传 24 字节速度、IMU、电压帧；多字节数高位在前；BCC 为逐字节异或。
- 因此该包是 R550 + C30D 2.0 + GMR 编码器组合的对应下位机固件候选。尚未读取实物板卡、OLED 车型或串口回传，不能把静态匹配描述为已烧录或实车确认。
- 扩展差异：该 STM32 包未检出 ROS 驱动中安全设置 `0xB0/0xB1` 和机械臂 `0xAA/0xBB` 的解析路径；后续只先采用已对上的基础速度/状态协议。

## 2026-09-14：讯飞 WebAPI + DeepSeek 语音链路

- 放弃依赖缺失 AIUI 配置和厂商闭源库的主链，新增独立 `xfyun_speech`、`deepseek_ros2`、`voice_command_router` 三包。链路为讯飞流式 IAT → `/voice/asr_text` → DeepSeek Chat Completions → `/voice/assistant_text`，另保留默认不启动的 TTS 和硬件适配入口。
- DeepSeek 只声明 `buzz(duration_ms)` 一个工具，工具 JSON 还需路由节点做名称、字段、请求 ID 和 100--2000 ms 范围校验；不发布 `cmd_vel`。用户确认蜂鸣器由下位机控制，故硬件实现降级为后续任务，本轮不猜 GPIO 或串口协议。
- SSH 复核 Orin 上 `python3-websocket` 1.9.0、`arecord`/`aplay` 可用。XFM-DP-V0.0.18 声卡以 16 kHz、S16_LE、单声道录制 3 秒成功，48000 帧，RMS 993、峰值 8055；默认采集设备更新为 `plughw:CARD=XFMDPV0018,DEV=0`。板子直连 DeepSeek 与讯飞端点的 TLS 均成功，未带凭据返回预期 HTTP 401。
- 三包部署至 `/home/wheeltec/ROSCAR/ros2_ws/src`，Orin 原生 Humble `colcon build --packages-select deepseek_ros2 voice_command_router xfyun_speech --symlink-install` 成功。启动后可见 `/xfyun_asr`、`/voice_command_router`、`/deepseek_chat`；默认无 TTS、无蜂鸣器节点。注入测试文本后在缺少密钥时明确报错，未生成硬件命令；修复重复 shutdown 后三个节点均干净退出。
- 新增 `scripts/run_voice_assistant.sh`，从板子私有 `~/.config/roscar/voice.env` 读取四项凭据，缺项时只报告变量名、不打印内容。模板与凭据文件权限为 0600；修复 ROS Humble `setup.bash` 与 nounset 不兼容后可正常启动。
- 首次静音测试暴露误触发：环境底噪 150 个 40 ms 帧的 RMS 中位数 1041、P95 1522、峰值 2441，旧阈值 500 会把底噪当语音。阈值提高到 2800，要求连续 160 ms 超阈值；本地 VAD 未确认语音时，即使讯飞返回“批评”等噪声文本也丢弃，不发布 `/voice/asr_text`，因而不会调用 DeepSeek。
- 真实 API 验证通过：文本注入得到 DeepSeek 精确回答“DeepSeek联调成功”；随后用户真人说话，讯飞发布“问一下你自己请介绍一下，你自己先介绍一下你自己。”，DeepSeek 返回语义正确的中文自我介绍。ASR 文本存在少量重复，后续可继续做阵列参数与标点优化，但端到端链路已成立。
- 最小审查：13 个协议、客户端和命令校验单测通过；8 包/58 个 Python 文件结构检查通过；启动节点、话题/服务、静音抑制、真人 IAT、DeepSeek 回答和干净退出已在 Orin 验证。TTS、蜂鸣器、下位机和车辆运动均未启动。

### 嘈杂环境唤醒与回答文件

- 用户说明麦克风藏在车内且环境嘈杂，不采用灯光提示。单独启动厂商 `wheeltec_mic` 串口节点后，“小微小微”硬件唤醒成功并给出方向 73°/87°；唤醒后停约 1 秒再提问可完成 IAT 与 DeepSeek 问答。
- 厂商源码会在较宽泛的 AIUI 事件分支提前发布 `/awake_flag`，因此 ASR 改为默认禁用该原始订阅，只接受精确 `/voice_words = 小车唤醒` 作为可信硬件唤醒。可信唤醒后接受讯飞的低音量文本；手动 `/voice/start_listening` 仍受连续 160 ms 本地 VAD 约束，兼顾嘈杂环境与远程静音测试。
- `run_voice_assistant.sh` 现自动叠加厂商工作区，并只启动 `wheeltec_mic_ros2/wheeltec_mic` 串口唤醒可执行程序；没有启动 `voice_control`、离线命令、反馈 WAV、灯光、蜂鸣器或车辆运动。
- DeepSeek 节点新增追加式 UTF-8 JSONL 输出 `/home/wheeltec/ROSCAR/logs/deepseek_responses.jsonl`，每行仅含 UTC `timestamp` 和 `answer`。实机文本注入得到 `{"answer":"文件输出成功"}` 并成功落盘；ROS `/voice/assistant_text` 保持不变。相关单测总数增至 14 个并全部通过，结构检查为 8 包/60 个 Python 文件。
## 2026-09-15：方案 A 一键启动入口

- 新增 Jetson 端 `scripts/route_a.sh`，默认 `start`，支持 `stop`、`restart`、`status`、`logs`；后台管理 `run_astra_foxglove.sh`，记录监督 PID 和日志，重复执行 start 不会重复拉起已记录实例。
- 新增 Mac 端 `scripts/route_a_remote.sh`，默认通过 SSH 别名 `roscar-wifi` 调用 Jetson 入口；新增可双击的根目录 `启动方案A.command`，启动成功后打开 Foxglove 并显示 `ws://192.168.1.240:8765`。
- `run_astra_foxglove.sh` 默认改为 `RGB_STREAM=false`，与 ASTRA S 上已验证的骨架流配置一致；就绪信息补齐目标框和人体掩码话题。
- 所有入口只启动 bodyreader、正式 astra route 和 Foxglove Bridge，不启动底盘、跟随控制器或 `/cmd_vel`。
- 小车仍处于离线状态。本轮完成 Bash 语法、ShellCheck、非法命令拒绝、模拟启动/重复启动/状态/日志/停止、POSIX 远端命令模拟和项目结构检查；尚未在 Jetson 或真实相机上运行新入口。

## 2026-09-15：方案 A 开机自启

- 新增 `deploy/systemd/roscar-route-a.service`，以 `wheeltec` 用户从 `/home/wheeltec/ROSCAR` 前台运行现有方案 A 组合入口；等待网络上线，骨架、适配器或 Bridge 任一进程异常退出时清理整组，5 秒后重启，systemd 按控制组停止全部子进程。
- 新增 `scripts/install_route_a_autostart.sh`，提供 `install`、`remove`、`status`、`logs`；安装时先停止已有手动实例，避免两套进程争抢相机，再执行 daemon-reload 和 enable --now。
- `route_a.sh` 会识别正在运行的 `roscar-route-a.service`，避免双击或远程 start 重复拉起另一套进程；服务日志改从 journal 读取。
- 服务保持 `ROS_DOMAIN_ID=182`、`RGB_STREAM=false`，不启动底盘或 `/cmd_vel`。安装前已完成离线语法、ShellCheck、systemd 单元结构和模拟安装检查。
- 用户随后要求直接执行。小车 `192.168.1.240:22` 恢复可达；首次同步发现白名单未包含 `deploy/systemd/`，补充后再次同步成功，并将服务安装到 `/etc/systemd/system/roscar-route-a.service`。
- 在线验收：`UnitFileState=enabled`、`ActiveState=active`、`SubState=running`、`NRestarts=0`；bodyreader、正式 astra adapter、foxglove_bridge 均在服务 cgroup 内，8765 在 Jetson 监听且 Mac TCP 检查成功。
- ROS 域 182 中发现 `/bodylist` 和五个预期感知话题；抽样 `TargetState` 为 `source=astra`、`is_simulated=false`、SEARCHING（等待叉腰），并确认 `/cmd_vel` 不存在。没有为了验收重启或断电小车，开机自动拉起仅由 systemd enabled 状态确认，待自然重启时再观察一次。

## 2026-09-15：接入低速 Astra 人体跟随

- 用户明确暂不增加避障，先实现朝被锁定人体运动。复用厂商 `wheeltec_robot_node` 的 `/cmd_vel` -> 11 字节 UART3 链路，不修改 STM32 固件或复制串口协议。
- `astra_body_adapter` 新增 `person_follower`：订阅 `/perception/target_state`，以20 Hz发布 `/cmd_vel`。默认 `enabled=false`；只在 TRACKING、`position_valid=true`、距离/偏角有限且消息年龄不超过 0.5 秒时运动。
- 控制默认保持 2.0 m，距离死区 0.15 m，偏角死区 0.08 rad；最高前进 0.15 m/s，最高转向 0.5 rad/s，偏角超过 0.6 rad 时原地转向，人过近时停车而不倒车。目标无效、丢失、超时、非有限或非正距离均持续发零速度。
- 按 TDD 先观察缺少实现、参数校验和非正距离测试失败，再实现最小修正。Jetson 上 `astra_body_adapter` 构建成功，原7项锁定测试加新14项控制测试，共21项全部通过。
- 在 Jetson 以临时用户服务启动底盘与跟随节点；验证 `/cmd_vel` 为1个发布者到1个订阅者、禁用/SEARCHING 时输出全零。用户确认安全后发10 Hz、1秒、0.15 m/s 直行指令10次，随后发零速度并恢复跟随服务；远程仅能确认指令链，未观测实际位移。
- 当前跟随参数为 `enabled=true`，感知为 SEARCHING，等待叉腰锁定，`/cmd_vel` 实测为零。两个用户服务均非开机持久化；本功能不含避障，且未完成真人跟随验收。
- 最小审查：确认光学 X 向右与 ROS 正角速度方向相反；限速、只前进、大偏角原地转向、动态启用、断流停车和非正距离停车均有直接测试或运行证据。
## 2026-09-15：Foxglove 掩码面板修复

- 用户报告掩码无消息。在线检查确认 systemd 仍为 active、`NRestarts=0`；`/bodylist` 约 29.6 Hz，`/perception/body_mask_image` 约 27.1 Hz，发布链路本身正常。
- Foxglove 已连接 `ws://192.168.1.240:8765`，TargetState 持续更新，但 Image 面板“主题”为空且掩码发布者订阅数为 0。将主题设为 `/perception/body_mask_image` 后等待提示消失、图像开始显示，foxglove_bridge 订阅数变为 1。
- 当前读取 `Bodylist.count=0`，因此掩码为黑色，含义是没有人体前景而非没有消息。
- 当前 Foxglove 工作配置显示图像主题存于 `imageMode.imageTopic`；仓库 `foxglove/astra-layout.json` 已补该字段并保留旧 `topic` 字段，避免新电脑再次导入后主题为空。

## 2026-09-15：叉腰识别在线诊断与控制节点清理

- 运行参数实查：`akimbo_hand_above_base_min_mm=20.0`、`akimbo_hand_shoulder_max_dx_mm=160.0`、`akimbo_shoulder_above_hand_min_mm=20.0`、窗口 10 帧、最少 3 票；叉腰识别已开启且使用放宽值。
- 当场 `/bodylist` 抽样 `count=0`，TargetState 为 SEARCHING。SDK 当前没有输出人体及关节，因此未进入叉腰几何条件判断；现有日志不记录每帧 count/关节条件，不能从日志还原用户“刚才”的具体手位差值。
- 安全复核发现两个不属于方案 A systemd cgroup 的独立手动进程：厂商 `base_serial.launch.py`/`wheeltec_robot_node` 与 `person_follower(enabled=true)`；当时 `/cmd_vel` 有 1 个发布者和 1 个订阅者，存在锁定后驱动车辆的可能。
- 已向两个启动父进程发送 TERM。刷新 ROS 发现后只剩 `/main`、`/perception/astra_bodylist_adapter`、`/foxglove_bridge`，`/cmd_vel` 返回 Unknown topic，方案 A systemd 服务仍为 active。未改动底盘固件或自启服务。

## 2026-09-15：恢复原始叉腰条件并复查骨架

- 按用户要求将默认叉腰条件从放宽值恢复为厂商等价值：手高于脊柱基点 50 mm、手肩横差小于 100 mm、肩高于手 50 mm；投票窗口恢复为 1 帧 1 票，即单帧满足立即锁定。
- 同步更新 `AkimboConfig`、ROS 节点参数默认值、正式 astra launch、两包 README、主方案文档和逻辑测试。保留通用投票实现，后续仍可通过 launch 参数调整。
- 本机 7 个叉腰/状态机 unittest 通过，Python 语法和项目结构检查通过。Jetson 原生 Humble `astra_body_adapter`、`perception_bringup` 两包编译成功；`colcon test` 未注册测试、实际为 0 项，因此另行直接运行 unittest，共 21 项通过，其中叉腰状态机 7 项。
- 重启 `roscar-route-a.service` 后实查参数为 50.0、100.0、50.0、1、1；服务 active、`NRestarts=0`，仅保留 A 方案三节点，`/cmd_vel` 不存在。
- 骨架问题未随阈值回退改变：15 秒收到 388 帧 Bodylist，positive=0、max_count=0。USB 正常枚举 `2bc5:0402 ASTRA S`，仅 bodyreader 相关进程占用相机；SDK 启动记录 `0x500007c9 Invalid Orbbec Body Tracking license`。历史实测中同类授权提示未阻止骨架输出，因此目前记录为关联异常，不能在没有进一步复测的情况下认定为唯一根因。
- 用户现场再次执行叉腰，使用临时只读订阅器连续监视两轮各 60 秒。第一轮 `frames=1785、body_positive=365、max_count=1`，观察到 ID 161、3；此前目标 ID 210 已锁定但人体消失后为 LOST。第二轮 `frames=1757、body_positive=641、max_count=1`，观察到 ID 171，仍未重新 TRACKING。
- 有人体的帧中，双手高于脊柱基点多数满足；手肩横差多次超过原始 100 mm，尤其初始右侧约 148～278 mm；更主要的是 `shoulder.y-hand.y` 经常为负值，SDK 将一只或两只手估计在肩膀上方，未满足肩高手 50 mm。后段横差改善到约 16～86 mm，但右侧肩高手仍约 -36～-110 mm。
- 现场结论：叉腰逻辑开启且参数生效，未锁定由人体仅在约 20%～36% 帧出现、ID 从 210 跳到 161/3/171，以及关节高度条件不满足共同造成。临时监视器不发布任何 ROS 话题或控制命令，测试结束后清理。

## 2026-09-15：方案 B 本地实现（codex/route-b）

- 用户要求开始 B 后澄清先在本机补齐方案与代码。本轮新增真实 YOLO11n/ByteTrack 后端、配准 RGB-D 输入校验、中央目标显式锁定/释放、epoch 隔离、躯干稳健测距，以及 TargetState、检测框图像、目标球输出。
- 默认模型为空且配准开关关闭时继续 NOT_READY；只有显式配置本地权重和已验证输入才推理。单任务异步推理避免图像堆积，完成后检查 RGB 与深度采集年龄；失效位置 NaN、Marker DELETE，不保留旧的有效位置。
- 新增 docs/方案B实现与验收.md，维护主方案、README、接口和路线图；图示保留旧方案快照并指明最新实现文档。没有新增任意 ID 服务占位、姿势识别、ReID、位置滤波、相机自启或车辆控制。
- 用户收窄为本机前只读确认小车 CUDA 可用、A active、ASTRA S 存在且无 /dev/video*。远端创建了隔离 .venv-yolo，但应用依赖下载超时退出；A 未停止、未切换服务。此后只操作本机。
- 最小审查覆盖配准显式确认、P 内参/尺寸/frame/时间验证、16UC1 毫米和 32FC1 米换算、异常深度拒绝、断流/失败后的 ID 复用隔离，以及无 cmd_vel 输出。
- 本机 7 项 B 逻辑测试通过。Linux ARM64 ROS 2 Humble 容器的 8 个主动包编译完成（8.58 秒）；7 项 A 逻辑、7 项 B 逻辑、B 合成 RGB-D/服务/状态/Marker 测试、A 合成适配器、14 项语音逻辑、A/B/demo 与非法 route 回归全部通过。A 使用当前恢复后的 50/100/50 mm、1 帧 1 票，未覆盖同期 A 改动。
- 本机 Python 3.12.13 独立环境的 36 个依赖通过兼容性检查；使用 torch 2.6.0、torchvision 0.21.0、Ultralytics 8.3.203 和 OpenCV 4.10.0.84。厂商 yolo11n.pt 的 SHA-256 校验通过，真实 CPU 模型连续两帧空白图推理、ByteTrack 调用和 reset 成功；这不是人体识别率、相机或 Jetson GPU 验收。日志为 artifacts/route-b-humble-test.log 与 artifacts/route-b-model-smoke.log。
- 本轮必要环境已补齐：启动本机 Colima 并构建含 cv_bridge/message_filters/OpenCV 的 Humble 测试镜像；新增可选 yolo_python 解释器参数，避免虚拟环境依赖被 ROS console-script 的系统 shebang 绕过。未将本机 Python 3.12 环境用于 ROS Humble Python 3.10。

## 2026-09-16：同步远端合并结果

- 拉取 origin，将当前 `codex/route-b` 从 `65a68bc` 快进至 `cb0b87a`（`feat: add safe Astra person follower (#2)`），本地 `main` 同步至同一提交。
- 同步前使用 `pre-sync-remote-main-2026-09-16` stash 备份全部已跟踪和未跟踪改动，恢复后保留该备份；工作区仍为未提交状态，未推送。
- 合并 AGENTS.md、WORKLOG.md 与 astra_body_adapter/README.md 三处文档冲突，保留远端控制功能与本地诊断/B 方案记录；README 叉腰默认采用本地已恢复的 50/100/50 mm、1 帧 1 票。
- 最小审查：非冲突本地文件逐字节对比 stash 一致，未跟踪文件完整恢复，无未解决冲突；HEAD、main 与 origin/main 相同。A 锁定及跟随控制 21 项 unittest 通过，git diff --check 通过；本轮未执行 ROS 编译或硬件验收，未操作 Jetson 服务。

## 2026-09-16：分支 a，红色物体跟随与串口闭环（仅本机）

- 用户确认新 A 要实现实际跟随完整链路，自动选择最大红块，但本轮只完成本机。已从干净提交 `909123d` 创建 `a`；包含此前合并 `cb0b87a` 的控制器与已提交 B 实现，未改 B 算法。
- 新增 red_object_tracker：H=0–10/170–179、S≥100、V≥70，形态学去噪、最小面积 0.1%；三帧确认后锁定，位置/重叠关联保持同一红块，单帧丢失立即失效，一秒后重新搜索并生成新 ID。阈值可通过 ROS launch 调整。
- 红块掩码内统计配准深度；验证相同光学 frame、尺寸、CameraInfo.P、时间及深度编码，拒绝空洞/混合深度。发布 `source=red_object` 的 TargetState、标注图、mono8 掩码和目标 Marker。缺少配准确认与输入时 NOT_READY；不启动人体 SDK、不使用模型、不以红块大小猜距离。
- 新增 `route_a.launch.py` 与 `run_red_foxglove.sh`；管理脚本和仓库 systemd 模板切到红色路线。原 astra route 与骨架组合脚本保留，两种组合入口共享运行锁。串口、运动、配准确认默认 false；底盘启用要求显式串口和车型，不能从厂商默认推断实车车型。任一 launch 子进程退出会关闭整组。
- 控制器保留 2 m、0.15 m/s、0.5 rad/s 和不倒车策略；新增 source、非模拟、发布时间/观测时间/测量年龄检查，拒绝重发旧观测，多个目标发布者时发零。原 Astra 仅在明确选择该来源时保留无传感器时间戳的兼容方式。
- 可选 `build_chassis.sh` 复制三个串口包到忽略的独立构建目录，原 COLCON_IGNORE 保留。修改迁入驱动头文件和 wheeltec_robot.cpp，厂商原始目录未改，SOURCE_MANIFEST.json 保留原始哈希作为来源快照。
- 驱动基本 11 字节发送限幅、非有限输入发零，20 Hz 检查命令与 24 字节回传，任一超过 0.5 秒或 cmd_vel 发布者数量不是一个时清除缓存并发零。串口设备路径加进程锁；读取超时从两秒改为 20 ms，检查实际返回字节数，滑动定长/帧尾/BCC 解析支持坏帧重新同步。I/O 异常退出且不重放旧速度，退出只发基本停车帧，不注册机械臂/回充/灯光/安全扩展命令。
- 新增 Foxglove `red-layout.json`，展示标注图、掩码、目标状态与位置、速度命令、里程计和电压；3D 坐标系需选实际相机光学 frame。本轮仅验证 JSON 结构，没有连接 Foxglove 客户端或相机。
- 最终容器验证：Linux ARM64 ROS 2 Humble，9 个主动包编译 7.38 秒；3 个串口包独立构建 12.5 秒。7 个红色逻辑、25 个 A 锁定/控制/新鲜度、7 个 B 逻辑及 14 个语音测试全部通过（合计 53 项）。保留原厂 serial 的 signedness/unused 编译警告，未将警告写成失败或零警告。
- 真实 C++ wheeltec_robot_node 使用伪终端：验证正负速度、限幅、BCC、24 字节里程计/IMU/电压、坏帧/分片恢复、指令与回传超时停车、多个速度发布者停车、重复打开拒绝；合成 RGB-D 经真实红色节点→跟随器→驱动产生串口帧，并验证深度单位、失效/丢失/重锁、过期/错 frame/不同步以及重发旧观测停车。
- 新 A launch 默认无 bodylist/cmd_vel/odom、非法运动使能拒绝、串口打开失败后整组退出均通过。原 A 合成适配器、B 合成推理、A/B/red/demo/非法 route 回归通过。完整日志为 `artifacts/route-a-red-test-final.log`；同步边界 3 项测试、Python/JSON 项目结构、Bash 语法、ShellCheck（仅排除外部 ROS source 无法读取的 SC1091）、git diff --check 通过。
- 最小审查：默认无控制节点；真实来源与模拟输入测试边界明确；旧观测不能因状态重发恢复运动；原始厂商清单与本地补丁区分；文档同步说明历史底盘链路已运行、随后曾停止控制，以及本轮未核对在线部署。未 SSH、未同步 Jetson、未安装服务、未做实车运动；相机彩色流/配准、车型和真实收发、STM32 断线保护及现场跟随仍待后续验收。

## 2026-09-16：红色检测原始/画框视频与 Foxglove

- 解释运动默认关闭来自已确认方案：with_chassis=false 不启动驱动和控制器；显式开底盘后 motion_enabled 仍默认 false。用户本轮没有要求修改运动默认值，因此保留。
- 新增 `/perception/color_image` 原样转发彩色帧；RGB 回调独立输出 `/perception/detections_image` 和 red_mask_image，不依赖深度/CameraInfo/配准。黄色框为检测候选，同帧配准跟踪及深度有效时可显示绿色目标框；视频可见不等于位置有效或运动使能。
- Foxglove red-layout 顶部并排原始视频和检测框视频，下方保留掩码、目标与底盘面板；同步更新接口、Foxglove README 和方案文档。
- 新增 RGB-only ROS 检查，验证原始像素不变、原图/框图 header 匹配、黄色检测框实际存在，无深度时 NOT_READY 且无 cmd_vel。最小审查包含布局面板引用、Python 项目结构与 diff 检查；ARM64 Humble 构建、原有逻辑及真实驱动伪串口/红色闭环回归日志为 artifacts/route-a-video-test.log。本轮未部署小车，未在在线 Foxglove 中导入布局，真实视频仍需相机彩色话题。

## 2026-09-16：更新红色方案 A 一键启动脚本

- Mac 双击入口显示红色目标路线、上下位机开关边界；成功后打开 Foxglove，给出 red-layout 的本地绝对路径及原始/画框视频话题，提示彩色相机输入仍需发布。
- remote 入口检查新版 runner 和 manager；manager 检查已运行 systemd 的 ExecStart，旧骨架服务不能被误报为新红色路线。runner 明确区分进程启动与视频输入就绪，输出串口/运动/配准配置。
- 最小审查及验证：四个脚本 Bash 语法、ShellCheck（排除外部 ROS source SC1091）、git diff --check 通过；临时假 SSH/systemctl 验证旧部署拒绝、新入口转发及两路话题显示。测试未调用真实 SSH，未部署或启动车辆，未实现下位机实体开关协议。

## 2026-09-16：项目总启动入口

- 新增 scripts/start_project.sh，Jetson 本机一条命令统一启动相机、红色检测/Foxglove、语音助手；语音可通过 WITH_VOICE=false 禁用。复用现有模块，不启动 B 或旧骨架避免重复目标发布者。
- 显式底盘/运动参数、环境/包/凭据文件检查、进程锁、旧 A 服务冲突提示、独立日志、Ctrl-C/子模块退出整组清理。默认不启动车辆；下位机实体开关尚未接入。
- 相机入口改为厂商 astra.launch.xml，固定 camera namespace 和彩色/深度开启，避免裸节点默认话题与新 A 输入不一致；注册开关仅由 DEPTH_REGISTERED 显式传入。测距仍要求实测校正/配准输入，原始相机流首先用于视频检测。
- 最小审查及本机验证：Bash 语法、ShellCheck（排除外部 source SC1091）、help、非法 bool/缺串口车型/不完整运动使能拒绝、diff 检查通过。未在 Mac 安装 Jetson 硬件环境，未 SSH 或部署；相机及整组实机启动尚待验收，不将脚本检查视为硬件运行通过。

## 2026-09-16：低频性能统计与 Foxglove 曲线

- 用户授权补齐系统效率指标，并询问统计对效率的影响。新增 person_interfaces/RuntimeMetrics 及共享 Performance 汇总器，红色/控制节点按一秒实际单调时间窗口发布，不逐帧发送性能消息或刷日志；每类样本上限4096。
- 红色记录实际输入与成功画框输出 FPS、彩色回调耗时、RGB-D 回调耗时、画框输出时观测年龄；控制端记录有效使能周期从采集到速度 publish 的延迟。每项平均/P95，未知/未来时间不采样、无样本 NaN。控制延迟不代表电机响应，回调耗时也不等于纯 HSV 计算时间。
- performance_enabled 默认 true，ROS launch 和 PERFORMANCE_ENABLED 环境变量可显式关闭（重启生效）；关闭时不创建性能定时器/发布者。Foxglove red-layout 增加 FPS、检测耗时、观测与控制延迟三组曲线，同步更新接口与主方案。指标功能不采集 CPU/GPU/内存，资源占用仍需 tegrastats。
- 测试最初发现整数测试样本触发 ROS float64 字段断言，汇总器现统一浮点转换。最终 Linux ARM64 Humble 主动及串口包编译通过；新增指标窗口/均值/P95/非法样本/禁用检查、RGB-only 实际性能输出、有效控制延迟样本均通过，原有红色闭环、A/B/demo/非法 route、语音回归通过。日志 artifacts/performance-test.log。
- 最小审查：计数无同步控制副作用，耗时使用单调计时，延迟限定同一 ROS 时间基准，输入 FPS 不声称为硬件原始 FPS；现有 RGB 与 RGB-D 两条检测路径分别计时，不隐藏重复计算。JSON 面板引用、结构、ShellCheck、diff 检查通过。原未提交空串口参数修复保留，未部署任何代码、未实测 Jetson 性能差异；只给出预期开销较小的判断，没有编造百分比。

## 2026-09-16：在线三维坐标只读检查

- 用户明确授权上车核对，SSH roscar-wifi 成功。实际运行目录 /home/wheeltec/ROSCAR-red，相机硬件2bc5:0402 Astra，厂商相机已开彩色/深度及depth_registration；新A启用配准确认、底盘参数car_mode=mini_akm。这只是读取当前配置，不代表本轮核验实物车型。
- 第一条目标状态为TRACKING但detail=Red target; depth rejected、position_valid=false、XYZ NaN。12秒只读订阅240条状态：182条深度拒绝、31条Registered mask depth有效、21条LOST、6条SEARCHING；随后6秒122条均无有效位置。
- 读取RGB/深度/CameraInfo：640×480、rgb8/16UC1、frame均camera_color_optical_frame；彩色P的fx/fy约570.342，cx319.5、cy239.5。深度P与彩色P相同，但深度K包含NaN。仅相同frame/P不能证明真实像素对齐或测距准确。
- 247次最新图像配对诊断（诊断采样并非message_filters精确同步）：最大红块约x304–364/y247–268，面积约800像素，多数目标区域深度100%为零，全图有效深度约23–24%。偶然有效样本掩码有效率72.7%、中位深度1.107m、计算XYZ约(0.0243,0.0437,1.107)m；该值不是物理精度验收，可能仍受对齐/背景影响。
- 当前person_follower动态enabled=true（启动命令曾为false），240条cmd_vel里5条非零；已明确告知用户深度恢复可能驱动车辆。本轮仅新建临时只读订阅器，结束即退出，未改参数、未发指令、未重启或部署。
- 结论：当前不能稳定发布可确认准确的坐标，主要直接证据是目标ROI缺失深度；下一步需在运动禁用的受控条件下，结合目标材质/距离/现场真值、深度图和标定配准检查。最小审查区分有效坐标、稳定性和绝对精度，未把推测的硬件原因当定论。

## 2026-09-16：红纸板稳定坐标复测

- 用户已放红纸板并要求下一步修复；先关闭当前在线 person_follower enabled（参数设置成功），没有启用或重启任何运动节点。
- 初始坐标有效，随后15秒302条TargetState全部TRACKING、Registered mask depth、position_valid=true，目标ID均red:17；300条Marker ADD，坐标系camera_color_optical_frame。301条cmd_vel全部零。
- 中位XYZ为(0.05039,-0.04548,1.40200)m；X范围0.04548–0.05531m、Y范围-0.04793至-0.04302m，Z该窗口均为1.40200m。观测年龄中位40.7ms、范围24.6–115.8ms。这是观测统计，不是物理精度验证。
- 结论：更换目标后有效深度和坐标已稳定恢复，没有证据需要放宽测距过滤或修改算法。本轮未修改远端源码；已请求镜头到纸板实测距离以判断绝对误差。若Foxglove仍无3D，先检查target_marker与实际光学坐标系。
- 最小审查确认状态/Marker/零速度三者相符。运动保持禁用，待现场验证后再决定是否恢复。

## 2026-09-16：按用户要求恢复跟随使能

- 用户要求打开追踪，按上下文恢复跟随运动。先只读确认 enabled=false、目标red:22 TRACKING/position_valid=true、距离1.402m、偏角0.0184rad、cmd_vel全零。
- 在线设置 /person_follower enabled=true 成功并读回True；随后一条cmd_vel仍全零，符合2m保持距离与当前居中目标。未改控制参数或部署代码；未据此声称实际位移或完整跟随验收。
- 最小审查：变更仅运行期enabled，目标状态与速度行为一致。当前跟随已使能，后续目标远离或偏转可能产生运动。

## 2026-09-16：超过1米跟随

- 按用户要求将默认 target_distance_m 从2.0改为1.0、distance_deadband_m 从0.15改为0；保持限速、偏角死区、目标失效停车和不倒车逻辑。该共享默认也适用于未显式覆盖参数的骨架跟随器。
- 先在线关闭 enabled；远端仅定点修改 follow_control.py（保留 before-1m 备份），Humble原生构建 astra_body_adapter 成功（总体4.08秒）。停止旧项目进程后，以原相机/串口配置且 MOTION_ENABLED=false 重启；新 tmux 会话 roscar-red-1m，语音关闭。
- 读回 target_distance_m=1.0、distance_deadband_m=0.0、enabled=false 后按既有授权设为true并读回。有效红色目标抽样 XYZ=(0.03244,-0.01140,1.0)m、水平距离1.000526m、偏角0.03243rad，速度抽样前进0.000351m/s、转向0。没有据此认定实车位移；测距偏差未校正。
- 新增默认阈值边界测试（0.8/1.0m不前进、1.01m前进、1.4m限速）；原2m参数测试显式保留配置，串口闭环停止样例改为0.8m，保持32FC1单位测试。同步更新红色方案文档。
- 验证完成：本机 Linux ARM64 Humble 9个主动包（13.5秒）与3个可选底盘包（18.1秒）构建成功；默认1米边界、A控制/锁定/新鲜度、真实C++驱动PTY与合成红色RGB-D闭环、A/B/red/demo及非法路由、视频/性能与14项语音逻辑回归全部通过。日志 artifacts/follow-1m-test.log。最小审查和 git diff --check 通过；未修改开机服务或将运动使能设为启动默认。

## 2026-09-16：跟随阈值进一步调整为30厘米

- 用户要求将1m改为30cm。修改共享FollowConfig默认target_distance_m=0.3，距离死区0，保留限速、不倒车及目标失效停车；更新方案文档。
- 先在线禁用运动，定点修改远端源码并备份至/tmp/follow_control.before-30cm.py。Jetson astra_body_adapter原生构建成功（总体3.75秒），停止旧栈后以运动关闭启动tmux roscar-red-30cm；相机、串口和语音开关沿用上一轮。读回0.3/0.0/false后恢复enabled=true并确认，cmd_vel抽样为前进0.15m/s、转向0，不代表已验证实际位移或30cm停止精度。相机测距偏差仍未校正。
- 新默认边界测试覆盖0.25/0.30m不前进、0.31m前进、0.70m限速；串口闭环32FC1停止样例调整为0.25m。本机26项适配器/控制测试通过。
- 本机Linux ARM64 Humble完整回归通过：9个主动包与3个底盘包编译、真实驱动PTY和红色RGB-D闭环、A/B/red/demo/非法路由、视频/性能及语音测试，日志artifacts/follow-30cm-test.log。最小审查和git diff --check通过。

- 2026-09-16：按用户要求将厂商目录中的雷达/激光雷达相关包复制到 `ros2_ws/src/radar_vendor/`，包含 `wheeltec_radar`、LS/LD LiDAR、RPLIDAR、pointcloud_to_laserscan 和双雷达融合；新增来源哈希清单与说明。默认保留 `COLCON_IGNORE`，未接入当前启动路线、未编译或实机验证。

## 2026-09-16：雷达源码迁入完整性复核

- 用户再次要求将雷达源码移入工作目录。检查确认 `ros2_ws/src/radar_vendor/` 已包含 wheeltec_radar、lslidar_ros2、ldlidar_ros2、rplidar_ros、pointcloud_to_laserscan-humble 和 double_lidar_fusion 六组目录，共7个ROS包，无需重复复制。
- 对照厂商 wheeltec_radar 与 wheeltec_lidar_ros2，排除Git/Python缓存和.DS_Store后，238个文件全部存在且逐字节一致；SOURCE_MANIFEST.json的239项（含本地README）SHA-256均匹配。
- 最小审查确认COLCON_IGNORE存在，默认构建继续跳过雷达目录；未修改启动入口，未连接小车、编译或进行硬件验证。git diff --check通过。

## 2026-09-20 N10P 雷达接入 ROS

- 用户确认 N10P，本次聚焦已迁入雷神 N10Plus 驱动，串口配置依据厂商源码 460800/10 Hz；其他雷达包继续暂存。
- 新增独立 radar_src/build/install/log 覆盖工作区构建，不取消源目录 COLCON_IGNORE；补齐 PCL、pcap、yaml-cpp、Boost 等 ARM64 Humble 测试环境依赖。
- 新增 radar.launch.py、run_radar.sh 和现有 perception.launch.py 的 with_radar（默认 false），提供 LaserScan、点云和只读 JSON 健康状态。默认不发布安装占位 TF、不改感知目标、不启动底盘或避障。
- 检查发现厂商扫描对 Y 取反而点云未取反；修复为同 frame 一致，并用左右不对称的合成串口回归覆盖。原始迁入哈希保留，本地补丁单独记录。
- 9 主动包及 lslidar_msgs/lslidar_driver 构建通过，真实驱动 PTY 解码 108 字节合成协议通过，1m 左侧/2m 其余、扫描和点云 frame 检查通过。健康节点 ROS 收发、过期/空回波/错误几何和未标定 TF 拒绝，以及 A/B/red/demo/非法路由回归通过，日志 artifacts/radar-test.log。不是实物测距/安装标定验收。
- 新增 N10P 接入说明、Foxglove 雷达布局和接口文档，更新同步白名单。最小审查：构建隔离、默认启动行为、扫描坐标方向、无 cmd_vel、ShellCheck、结构检查和 git diff --check；客户端布局未实测。
- 本轮无 SSH、远端部署或硬件访问；实际串口别名、扫描方向、帧率、TF 和拔线 STALE 需现场验收。

## 2026-09-20 跟随控制保护与故障回归

- 新增 motion_guard 主动包及配置、启动、说明；请求 TwistStamped 与最终 Twist 分离。已有红色组合启动接入 guard，单独跟随节点不能再直接向底盘发速度。纯感知默认行为保留。
- 实现 PERCEPTION_ONLY/STANDBY/ARMED/FAULT，arm/stop/disarm 服务，故障锁存和重启未授权。检查唯一发布者、源/帧/有效性、发布/观测/接收时效、速度分量及限幅、时刻 TF 和扫描覆盖；配置占位必须显式确认后方能授权。
- 采用保守全方向车体圆形停车包络，考虑速度上限、反应期、制动、几何和采样余量；反应参数至少覆盖扫描最大年龄+底盘0.5秒命令超时+保护0.05秒周期。不实现绕障，不代替 STM32 失联停车。
- 新增录包/隔离回放脚本，真实 rosbag 测试验证最终速度只发 /replay/cmd_vel，非白名单话题不回放。
- 10 个主动包、3 个底盘包在本机 Linux ARM64 Humble 编译通过；新增 6 项纯逻辑及服务/TF/故障/重启测试通过，并连接真实 C++ 底盘驱动的 PTY 检查运动和停车帧。记录 artifacts/motion-guard-test.log。
- 测试修正：TF Buffer.clear 保留静态 TF，改以空缓冲区注入 TF 丢失；串口检查改为最新帧已零，避免以故障前的固定三帧历史误判停车失败。新增测试与旧感知/串口回归分开验证，未把测试专用桥用于生产。
- 最小审查覆盖默认未授权、配置只读、源时间兼容、停止/故障恢复、发布者唯一性、无绕过启动、回放隔离；结构检查、ShellCheck/Bash 语法和 git diff --check 通过。本轮无远端或硬件操作。
- 最终完整容器回归通过：新增保护/录包回放专项之外，原性能、RGB 视频、Route A 启动、真实驱动串口协议、红色合成闭环、A/B 逻辑及合成状态、14 项语音逻辑、A/B/red/demo/非法路由均通过；容器脚本退出码 0。

## 2026-09-20 SLAM、Nav2、自动绕障与 Foxglove

- 新增 navigation_bringup：mapping/localization/external 模式、SLAM Toolbox、AMCL、NavFn 全局规划和 DWB 局部控制；自定义无倒车恢复行为树，Nav2 原始速度经授权状态与时效检查进入 motion_guard，不直发底盘。
- 新增观测时刻相机/车体 TF 到地图的留距目标、目标更新先取消再发送、目标丢失/ID变化/里程计过期故障锁存；接近目标 HOLD 后允许目标重新远离时继续导航。默认标定确认与运动关闭；无 SSH 或硬件操作，原感知入口保持不变。
- 新增 odom TF 桥（忽略厂商 position.z 航向），安装 TF 仅在显式确认后发布；保存地图脚本拒绝覆盖文件。Foxglove 布局和地图/路径/代价地图/导航状态录包已补充，回放新增 Nav2 原始速度隔离。
- 环境安装时 Colima VM 退出导致镜像导出失败；清理失效 hostagent 并重启同一测试 VM 后，ARM64 Humble Nav2 1.1.20 与 SLAM Toolbox 2.6.10 安装成功。未删除其他项目资源。
- 运行审查修复 ROS Node.handle 名称冲突、Nav2 through-poses 默认恢复服务依赖，以及 SIGINT 先关闭 ROS 上下文导致停车消息发布失败的问题；最后一项通过禁用 rclpy 自动信号关闭、在 finally 中先清理后 shutdown 处理。
- 最终导航专项退出码 0：11 个主动包编译、3 项几何测试；真实 SLAM Toolbox 生成占用/空闲地图；真实 NavFn 生成 197 点绕障路径；BT/DWB 输出仅进入隔离 raw 话题、默认最终速度为零；真实 map_server/AMCL 激活并输出定位。ROS 桥接测试覆盖观测时刻 TF、平面 odom、留距、HOLD 后重新走远、目标丢失取消、ID变化、速度断流与里程计过期。日志 artifacts/navigation-test.log / navigation-run.log。
- 完整原功能回归退出码 0：11 主动包与3底盘包编译，保护+真实 C++ 驱动 PTY、含 Nav2 原始速度的真实 rosbag 隔离回放、视频/性能/原A启动/红色串口闭环、A/B合成和语音及各路由回归通过，日志 artifacts/navigation-regression.log。
- 最小审查通过：配置只读与默认未授权、目标序列化取消、车体/目标 frame 一致性、保护圆盘与 costmap 半径一致性、无自动恢复运动、启动无底盘串口、回放隔离；结构检查、YAML/XML/JSON 解析、ShellCheck 与 git diff --check 通过。未做实物雷达/相机/里程计标定、真实客户端展示或实车导航验收。

## 2026-09-20 当天新增代码审查与修复

- 覆盖 N10P、停车保护、Nav2/SLAM 桥、直接跟随整合、构建/同步/录包及 Foxglove 配置，具体问题、影响和修复见 docs/2026-09-20代码审查.md。
- 核心修复：雷达近盲区覆盖；SIGTERM 发零并清理、重复停止信号不打断清理；跟随请求 base_frame 与红色 target_frame 显式传递；performance_enabled 保持透传；TF 最多等待0.15秒且期间零速度、持续缺失锁存；导航 JSON 严格类型检查；雷达健康 steady timer 与只读配置。
- 配套：补 astra_body_adapter 的 rcl_interfaces 依赖、导航测试基础镜像构建、导航 Dockerfile 同步白名单，缩小雷达暂存 COLCON_IGNORE 删除范围。新增独立进程 SIGTERM/组合启动与 frame/指标测试、盲区和畸形状态单测、TF 短延迟/持续缺失与暂停时钟测试。
- 初轮运行发现 ros2 launch 再次转发 SIGINT 会打断 guard.destroy_node；已补重复信号处理，并将组合退出日志中的 Traceback 纳入失败判定，随后执行最终回归。
- 最终导航专项通过（退出码0）：11包构建、4项单测、真实SLAM/NavFn/BT/DWB/map_server/AMCL，以及TF短延迟零速等待/持续缺失锁存、目标停走/丢失/ID变化和过期输入测试。日志 artifacts/review-navigation-final.log。
- 最终原功能回归通过（退出码0）：11主动包+3底盘包构建，7项保护逻辑、独立进程SIGTERM最终零速/自定义frame/组合指标开关、真实驱动PTY停车与协议、rosbag隔离回放、A/B/red/demo/非法路由及视频/性能/语音等回归；最终组合退出日志无Traceback。日志 artifacts/review-regression-final.log。
- 雷达最终专项通过（退出码0）：11主动包及 lslidar_msgs/lslidar_driver 构建，雷达健康状态/暂停ROS时钟仍持续发布、安装确认、真实N10Plus驱动伪串口解码和扫描/点云方向，以及各路线回归通过。日志 artifacts/review-radar.log；PCL输出可选pcap功能警告，N10P UART链路测试通过，未验证pcap回放。
- 最小复审：结构检查（11包/165个Python文件）、YAML/XML/Foxglove JSON解析、ShellCheck、同步3项单测与 git diff --check 通过。AGENTS.md、WORKLOG.md、导航/保护说明与独立审查清单已更新。所有结果均为本机软件测试；未部署、未启真实底盘。

## 2026-09-20：方案 B 本机 RGB-D 输入预检

- 用户确认继续仅本机完善；未 SSH 小车、未部署或切换 A 服务，未启动车辆控制。
- 新增 `scripts/check_rgbd_input.py`，通过 ROS 只读订阅统计 RGB/深度/CameraInfo 数量和窗口频率、缺流/陈旧状态、同步对时间差与原始消息年龄、最近两路时间戳差、最新标定及全幅深度有效比例；可写 JSON，不保存图像，不加载 YOLO。缺消息时正常返回诊断和退出码 1。
- 抽出 `input_contract.py` 供正式 B 节点与预检共同使用；保留 frame/尺寸/编码/时间等校验，增加非有限 P、非零 skew、非标准投影最后一行、空尺寸拒绝。B 的深度测量、选人逻辑与新鲜度兜底保持原有行为。
- 预检通过要求最近合格同步对仍新鲜，但不等于证明相机配准/校正正确；`registration_verified=false`。全幅有效深度比例只是诊断指标，不能代替躯干测距验收。当前域 `/cmd_vel` 发现结果也不是整车控制状态证明。
- 厂商原包只读审查：默认 raw 话题及配准/同步关闭；配准设置失败后可能继续使用 aligned frame；OpenNI 回调用 `node_->now()` 打时间戳。未改厂商代码，相关实机核验要点同步到 B 文档、主方案和路线图。
- 恢复 Colima Docker 运行环境并补齐 Humble 镜像所需依赖。首轮 8 包编译和逻辑/合成预检通过，但 CLI 测试因容器未挂载 scripts 失败；已补挂载并完整重跑。
- 最终验证：Linux ARM64 ROS 2 Humble 8 包编译成功（7.26 秒）；21 项 A 逻辑、11 项 B 逻辑、14 项语音逻辑通过；B 合成 RGB-D 覆盖正常输入、错 frame、旧时间戳、断流、80 ms 无法配对、缺流 CLI JSON、锁定/丢失/epoch、Marker；A 合成和 A/B/demo/非法 route 回归全部通过。日志 `artifacts/route-b-input-test.log`。
- 最小审查覆盖共享校验与正式节点行为、只读订阅/无控制发布、数据保存范围、诊断内存有界、配准结果不夸大；项目结构检查和 git diff --check 通过。本轮不涉及真实相机、真人识别或 Jetson GPU 验证。

## 2026-09-20：YOLO26s 官方预训练权重与免 NMS 接入

- 用户选 s 版，要求寻找官方已训练权重并使用加速模式；延续仅本机范围，未 SSH 或部署小车，未切换 A 服务。
- 核对官方 YOLO26 模型页、TensorRT 文档、Ultralytics assets v8.4.0 发布与 GitHub release API。下载 `yolo26s.pt`（COCO 预训练检测，含 person），20,422,725 字节，SHA-256 `646f8bc3fe0a656803d95c294f7852321748cb29d13466a1af8862e2db384a1b` 与官方 asset digest 一致。模型留本地忽略目录，来源、哈希及 AGPL-3.0/Enterprise 许可记录在 manifest。
- `prepare_model.py` 默认准备 YOLO26s，下载到临时文件并校验后落盘；已有不同文件不覆盖。旧 YOLO11n 仍支持厂商目录复制。ROS 初始化不自动下载权重。
- 本机升级 Ultralytics 到 8.4.156；首次解析发现旧 thop 2.0.17 不满足新版要求，已同步升级至 2.1.6。保留本机 torch 2.6.0/torchvision 0.21.0/numpy 1.26.4/OpenCV 4.10.0.84，43 项已安装依赖兼容性检查通过。未触碰 Jetson CUDA 环境。
- B 增加默认 true 的 nms_free 参数，调用 nms=False 明确启用 YOLO26 端到端头；若实际模型无端到端输出则失败，避免静默退回。rect=False 固定方形输入，便于匹配未来 640×640 静态 TensorRT 引擎。ByteTrack、目标锁定、深度算法、配准开关及超时保护保持；旧 YOLO11n 需 nms_free=false。
- 新增 `export_yolo26_engine.py`：默认打印计划；--execute 要求目标 Jetson Linux/aarch64/L4T、CUDA torch 和 TensorRT 10.x。固定 640、batch=1、quantize=16、nms=False、静态尺寸、2 GiB workspace、opset 17、不做额外图简化；关闭库自动安装，不替换系统 CUDA/TensorRT。构建后空图加载检查并记录 GPU/L4T/版本/哈希。FP16 为构建精度，输入和个别层可能保留 FP32，不能根据输入 dtype 判定整个 engine 是否 FP16。
- Mac 真实模型测试通过：空图无检测，官方随包 bus.jpg 重复四帧均检测 4 个 person、保持 ID 1–4；reset 通过、实际 backend.end2end=true。保留的 YOLO11n 在新版库下相同测试通过（end2end=false）。这是静态示例重复输入，不是现场真人、移动、多人交叉或遮挡跟踪验收；日志内 CPU 计时不当作 Jetson 性能。
- Linux ARM64 Humble：8 包编译成功（8.56 秒），21 项 A、13 项 B、14 项语音逻辑通过，A/B 合成、RGB-D 预检 CLI、A/B/demo/非法 route 回归全部通过。日志 `artifacts/yolo26-ros-regression.log`、`artifacts/yolo26s-model-smoke.log`、`artifacts/yolo11-baseline-regression.log`。
- 导出计划参数检查及 Mac 执行拒绝保护通过；未生成 TensorRT engine。缺少的是目标 NVIDIA 硬件且本轮限制仅本机，不能把本机验证表述为 Jetson FP16 加速成功。
- 同步更新 README、主方案、B 文档、路线图和权重说明。最小审查覆盖真实模式参数、旧模型兼容、engine/CPU 错配拒绝、下载哈希、无自动依赖替换及无新增车辆控制；结构检查与 git diff --check 通过。

## 2026-09-20：B 受控并发、三维 tf2 与未标定兼容

- 用户要求把 B 多线程/TF 代码补齐，未填外参用单位变换占位；随后强调没标定完不能影响原有效果。按此约束将 TF 做成独立派生进程，不修改原目标话题的坐标契约。
- 跟踪器新增两线程 MultiThreadedExecutor，输入与状态回调各自互斥组，共享 Selection/future/snapshot 状态由 RLock 保护。图像转换、YOLO、ByteTrack、躯干测距放到单个 ThreadPoolExecutor 工作线程；backend 初始化/reset/infer 顺序执行，忙时丢新输入，不积累推理队列。ROS 定时器/释放服务不等待推理结果。
- 新增 yolo_person_tracker/target_tf.py，正式 B launch 自动启动独立 target_transform；输入 /perception/target_state，输出 /perception/target_state_base 与 /perception/target_marker_base。原光学目标、检测图、Marker 和锁定/释放服务保持。tf2 查询使用 observation_stamp，零/未来/过期时间、非有限点、错误源 frame、缺 TF/外推失败或未确认标定均不产生有效 base 位置。节点不发速度。
- 配置 perception_bringup/config/camera_mount.yaml：translation=[0,0,0]、quaternion=[0,0,0,1] 为待填安装参数，extrinsics_calibrated=false；publish_mount_tf=false，默认不向真实 TF 树广播单位外参，避免影响现有坐标。camera_link 到 optical 的真实轴转换继续由驱动/URDF 提供。已有发布者时不要重复广播安装边。
- base 输出 XYZ 为前/左/上，距离 hypot(X,Y)、偏角 atan2(Y,X) 左正；原 optical 输出及右正偏角不变。消息布局未改，仅补注释及文档；旧光学 follower 不可直接 remap 为新 base 输入。有效 base Marker 使用观测时间戳，避免在其他动态固定坐标系显示时拿发布时间作观测时间。
- 补齐 Humble 容器 tf2_ros_py/tf2_geometry_msgs 运行依赖和包声明。最终 Linux ARM64 Humble 8 包编译完成（7.28 秒），21 项 A + 13 项 B + 14 项语音逻辑通过。
- 新 TF 专项运行测试通过：未标定/缺 TF 拒绝、光学轴旋转和安装平移、额外安装 yaw、XYZ/偏角符号、错误 frame/NaN/陈旧/未来消息、源断流、观测时刻动态 TF 外推失败无 latest 回退。
- 新并发运行测试用阻塞检测后端验证释放服务和状态定时器仍响应，重复输入不并发更新跟踪器，超时后完成的推理结果不能变为有效位置。原 B 合成测试新增未标定派生节点，并断言原 TRACKING、2m 测距、检测图和 Marker ADD 正常，同时 base 无效且未创建安装 TF 广播器。A/B 合成、RGB-D 预检、A/B/demo/非法路由完整回归通过。
- 最终日志 artifacts/route-b-tf-concurrency-final.log。最小审查覆盖共享状态锁、worker 顺序、TF 时间及方向、默认不发布占位 TF、接口隔离、无控制输出；结构检查和 git diff --check 通过。仅本机代码和合成验证，未访问/部署小车；不能宣称物理标定完成或实机性能完全不变。

## 2026-09-20 本机代码审查

- 审查主动包、脚本与厂商控制接入边界，报告 `docs/代码审查-2026-09-20.md`，记录 2 项 P1、3 项 P2 及 2 项既有厂商协议问题；未声称逐行审计全部厂商/第三方实现。
- 在隔离 ROS 容器用拦截发布方法复现模拟/旧目标仍生成非零速度，用假 GPIO 复现 disabled 后仍输出；未发送硬件指令。
- 标准容器回归与静态检查日志保存在 artifacts；初次单独复现容器漏挂 scripts 导致 probe 启动失败，随后通过标准脚本补齐挂载重跑，不计为项目缺陷。
- 本轮仅新增审查文档和上下文记录，业务代码保持不变；没有 SSH、远端部署或实机验证。最小审查核对报告路径、代码行号与复现证据，并执行 git diff --check。

## 2026-09-20 修复代码审查全部发现

- 修复 2 项 P1、3 项 P2 与 2 项历史串口问题，具体行为与边界见 `docs/代码审查-2026-09-20.md` 修复结果。
- 控制边界增加模拟/来源/发布及观测时效检查，A 零观测时间戳继续兼容；新增节点级测试拦截速度输出，未连接硬件。
- GPIO 用定时器管理脉冲，禁用和销毁收尾；A 手动生命周期加 flock，识别 systemd 过渡状态，已安装服务不再退回手动运行。
- 厂商机械臂拒绝长度/非有限/范围异常；机械臂正常和析构路径发送 10 字节，安全扩展补帧尾。保留来源哈希并记录本地补丁；同步修正串口协议说明。
- 主动工作区 8 包编译、48 项既有逻辑与新增控制/GPIO/生命周期、TF/并发/A/B/路由回归通过，日志 artifacts/review-fixes-regression.log。
- 新增 deploy/chassis-test.Dockerfile 与 scripts/test_chassis_container.sh，补齐 turtlesim/nav2_msgs/ackermann_msgs，在临时副本中编译 serial、wheeltec_robot_msg、turn_on_wheeltec_robot 三包通过（保留源 COLCON_IGNORE）；厂商仍有既有警告。真实回调+析构路径的串口替身检查通过 ASan/UBSan，日志 artifacts/vendor-frames-regression.log。
- 测试编写中修正了测试消息共用时间戳对象、补齐串口替身 close/log 接口；修正后回归通过。最小审查核对新鲜度兼容、锁描述符关闭、串口源哈希与新帧长；ShellCheck、Bash 语法、结构检查及 git diff --check 通过。
- 全程本机，无 SSH、远端部署或车辆启动；修复不代表扩展协议固件支持或实机跟随验收。

## 2026-09-21 radar 与 route-b 合并至 main

- 按用户要求将 codex/radar 和 codex/route-b 合并至最新 origin/main（32afd91）。先将 radar 工作区已有雷达、运动保护、导航及审查修复完整提交为 a3140dc，再分别保留分支历史合并；未删除原分支或历史 stash。
- 冲突合并保留两边文档和测试，测试环境依赖与同步白名单取并集。保留 motion_guard 唯一正式速度出口、红色目标来源、性能统计、串口超时/进程锁及退出仅基本停车帧；同时保留 B 的 YOLO26、输入预检、并发、TF 派生输出、GPIO 与生命周期修复。
- 修正自动合并产生的来源检查重复（B 原检查会拒绝 red_object）、重复 Docker 挂载及 red route 多余 nms_free 参数。适配 B 安全测试的显式来源/发布者契约、systemd 测试的红色入口核验，以及串口回调测试的基本停车退出策略；更新合并后驱动补丁哈希。
- AGENTS.md 项目规则按用户明确约定统一为每次更新 WORKLOG、仅长期规则/入口变化更新 AGENTS，保留两分支已有上下文。
- 验证：结构检查 11 包/176 Python 文件、同步 3 项测试、JSON/XML/Bash 解析、ShellCheck（忽略外部 source 路径 SC1091）、git diff --check 和真实串口回调 ASan/UBSan 已通过。合并后完整 ARM64 Humble 回归退出码0：11主动包（9.14秒）与3底盘包（14.5秒）编译通过；控制保护/故障锁存、SIGTERM最终零速、真实驱动PTY、红色合成闭环、真实rosbag隔离回放、B输入预检/TF/并发、GPIO/生命周期、A/B/red/demo/非法route及14项语音测试全部通过。日志 artifacts/merge-main-regression.log。导航4项逻辑测试通过；本轮未重复运行真实SLAM/Nav2栈和N10P驱动专项，沿用2026-09-20已记录的专项证据，不作为本次新增验收。
- 本轮不访问小车、不部署或启动真实设备。测试镜像此前已清空，复用现有 Colima，下载前磁盘可用约38 GiB；测试容器自动删除，随后清理本轮新建测试镜像与其基础镜像，验证日志保留。

## 2026-09-23：实施对外 ROS 2 接口计划（仅本机）

- 依据用户指定 `PLAN (3).md` 实施；不访问 SSH、不部署或启动车辆。
- 新增 `roscar_interfaces/SetControlMode` 和 `roscar_api`。统一入口默认 IDLE、运动关闭、不启动硬件；可显式组合现有感知/跟随/雷达/底盘，只有一套 guard。四个安装后的 Python 示例涵盖底盘、检测、锁定/释放和雷达。
- motion_guard 增加 EXTERNAL/FOLLOW 来源选择，保留旧 FOLLOW 默认值；切换模式立即停车、清缓存、解除授权，拒绝切换前的排队旧请求。EXTERNAL 不要求视觉，但保留雷达/TF/尺寸/停车模型/新鲜度/重复发布者保护，只接受 base_link 的前进和 yaw。状态 JSON 增加 command_mode；故障不自动续动。
- YOLO/red 沿用同步 RGB-D 增加 Detection2DArray。YOLO 单次 predict 后由 ByteTrack 原检测索引映射身份，保留未跟踪框，未跟踪框不能锁定；原始图像坐标和 header 不随显示缩放。红色全部候选输出，score=NaN，不虚构概率或身份。空检测发布空数组，异常/断流不伪装空检测，原 TargetState 结构不变，未新增 RGB-only 流水线。
- 新增中文 `docs/ROS接口使用文档.md`；同步 README、接口索引、构建/同步说明、结构检查与测试入口，AGENTS 仅补长期 API 入口。
- 已完成：Linux ARM64 Humble 13 包编译（9.70s）；服务/模式互斥/非法速度/时效/TF/障碍/故障锁存专项、四个已安装示例与默认 launch、真实 C++ 底盘伪串口 EXTERNAL 运动及停车帧；YOLO/red 同步 RGB-D 多框、未跟踪、空检测、ID epoch、缩放坐标与原三维回归；Mac CPU 真实 YOLO26s 权重空图及 bus.jpg 连续帧推理成功。
- 旧 A/B/red/demo/非法路由、TF、并发、控制退出、性能、语音、真实驱动 PTY、隔离 rosbag 回放回归通过。回放首次批量执行错用域号，改按脚本约定 ROS_DOMAIN_ID=174 后通过；示例测试首次仅向 ros2 run 父进程发信号，修正测试为进程组信号后退出检查通过。均非实车验收。
- 导航最终通过：真实 SLAM Toolbox 地图、NavFn 绕障路径、BT/DWB 速度、map_server/AMCL，以及观测时刻 TF、目标切换/丢失和故障不自动续动。雷达 2 包编译 44.9s，健康/时效/标定拒绝及真实 N10Plus 驱动伪串口的扫描/点云方向和距离一致性通过。都只使用合成输入，不计为硬件验收。
- 最小审查完成：模式切换无旧请求复用、EXTERNAL 不绕过雷达、未跟踪框不能锁定、消息坐标不随显示缩放、失败不伪报空检测、默认入口无硬件/授权、包依赖和同步白名单完整。13 包结构检查、3 项模拟同步单测、ShellCheck、git diff --check 通过；同步单测拦截 subprocess，没有实际访问远端。
- 验证证据：`artifacts/api-build.log`、`api-runtime.log`（包含初次单测 mock 修正前记录）、`api-examples-pty.log`、`api-final-vision.log`、`api-regression.log`（包含初次回放域号问题）、`api-regression-runtime.log`、`api-navigation.log`、`api-radar.log`、`api-model-smoke.log`。ROS 基础镜像 digest `sha256:1813d3c85d7f96ff7d3012d865204583255740182db5d0065f8f8cd029a83138`，完整镜像身份见 `api-test-environment.log`。
- 本轮创建的容器、镜像和构建缓存已清理，Docker 显示 images/containers/volumes/build cache 全部为 0；保留既有 Colima 环境，回收其已释放块。源码、已有权重与验证日志保留。清理记录 `artifacts/api-cleanup.log`。
## 2026-09-16：底盘串口与短距运动冒烟入口

- 新增 `scripts/chassis_motion_smoke_test.sh`，只启动加固后的 `wheeltec_robot_node`，不启动 `person_follower`。运行前拒绝已有底盘节点或 `/cmd_vel` 发布者，避免双开串口或两个速度源触发驱动安全停车。
- 脚本要求显式 `--car-mode` 且必须匹配 `robot_model.yaml`；默认 `--check` 只等待 `/PowerVoltage` 有效回传。只有显式 `--move` 才创建唯一发布者，以默认 0.05 m/s 前进 0.5 秒，随后连续 0.75 秒发零并关闭驱动；速度硬限制 0.08 m/s、时长硬限制 1 秒。
- 当前板上已只读确认 `/dev/wheeltec_controller` 解析为 `/dev/ttyCH343USB0`。独立部署最初缺少 `chassis_vendor`，首次构建在复制前立即退出；补同步三套源码后，Jetson 原生 Humble 的 `serial`、`wheeltec_robot_msg`、`turn_on_wheeltec_robot` 全部构建成功，驱动可执行文件已安装。原厂 serial 仍有既存 signedness/unused 编译警告。
- Jetson 上 Bash 语法、帮助、缺车型及非法车型拒绝检查通过；原在线 ROS 域 182 中没有 `/cmd_vel`，也没有已运行的 `wheeltec_robot` 节点。板上未安装 ShellCheck，因此未宣称通过该项。
- 用户照片确认 OLED 为 `Akm`，底盘可见转向舵机，选择仓库键 `mini_akm`。照片同时显示约 11.37 V；ROS 驱动隔离测试收到 `/PowerVoltage=11.337`。直接只读串口还采到连续 24 字节 `0x7B...BCC...0x7D` 帧，抽查 BCC 正确。
- 冒烟脚本修正 ROS 2 `topic echo --once` 参数位置，默认改用本机隔离域 183，并在 ROS 图检查之外增加 `fuser` 串口占用拒绝；运动发布者先持续 1 秒发送零速度，等待 DDS 双向发现后才允许非零命令。
- 首次两轮 0.05 m/s、0.5 秒测试分别在加入零速握手前后执行，里程计都基本为零，未形成有效运动。没有直接提高到驱动 0.15 m/s 上限；第三轮使用冒烟脚本硬上限 0.08 m/s、1 秒，`/odom.twist.twist.linear.x` 出现连续正值，峰值约 0.088 m/s，随后逐级下降并最终回到 0.0。
- 第三轮证明 ROS 指令、串口、下位机和编码器反馈链路产生了运动响应，但远程没有视觉观察车身是否在地面实际位移。测试结束后 `wheeltec_robot_node` 和测试发布器均退出，`fuser` 确认串口无人占用；`roscar-red` tmux 感知会话仍运行。后续需由用户现场确认实际位移和前进方向。
- 版本收尾检查发现 Windows 工作区会将 Shell 脚本检出为 CRLF，直接 SCP 后 Jetson Bash 报 `\r` 语法错误；新增 `.gitattributes` 固定 `*.sh` 和 `*.command` 为 LF，并在提交前用暂存区内容重建、同步及复测相关脚本。该问题只影响后续从 Windows 再部署的文件，既有在线进程未因检查而中断。

## 2026-09-16：分支 a 红色跟随真机部署

- 用户要求将分支 `a` 的跟随功能部署到小车测试。Jetson 的旧 `/home/wheeltec/ROSCAR` 骨架服务自动启动并占用相机；按用户授权停止 `roscar-route-a.service`。旧服务仍为 enabled，停止后因旧脚本响应 TERM 的退出码显示 failed，但其进程已退出、相机已释放。
- 核对 `/home/wheeltec/ROSCAR-red` 中总入口、相机、红色感知与底盘冒烟脚本和本地哈希一致，Bash 语法通过；所需红色跟踪、person_follower、底盘驱动和 Foxglove 可执行文件均存在。先以 `WITH_CHASSIS=false`、`MOTION_ENABLED=false` 运行相机预检，再以 `WITH_CHASSIS=true`、`SERIAL_PORT=/dev/wheeltec_controller`、`CAR_MODE=mini_akm`、`MOTION_ENABLED=false` 重新启动全链路。
- Astra 启用 depth_registration 后，彩色和深度均为 640×480，彩色、深度与 CameraInfo 使用 `camera_color_optical_frame`；真实 `/perception/target_state` 来源为 `red_object`、`is_simulated=false`，观测年龄约 0.03 s。当前无红色目标，状态为 SEARCHING/Confirming largest red component。
- 底盘串口成功打开，`/PowerVoltage` 实测约 12.03 V；`/cmd_vel` 恰有 person_follower 一个发布者和 wheeltec_robot 一个订阅者，禁用时消息全零。动态设置 `/person_follower.enabled=true` 成功，随后再次确认无目标时速度仍全零。
- 当前 `tmux` 会话 `roscar-red` 保持运行，Foxglove Bridge 为 `ws://192.168.1.240:8765`。本轮证明真实相机、注册 RGB-D、红色状态、控制节点和底盘串口已组成在线链路；尚未由用户现场确认红色物体引导下的实际位移、方向和转向效果，不将其写为完整实车跟随验收。系统无避障，测试需清空场地并随时断电或将 enabled 设回 false。
- 最小审查：检查五个 ROS 节点、唯一目标发布者、唯一速度发布/订阅对、真实来源标记、电压回传、跟随参数和零速度；未修改 B 或语音代码，未把本机未跟踪的根目录 `red-layout.json` 纳入版本。

## 2026-09-16：恢复语音助手与 TTS 播报

- 用户明确要求暂停跟随工作，只处理语音模块。检查发现当前项目以 `WITH_VOICE=false` 启动，因此只有相机、感知、底盘和 Foxglove 节点；语音私有配置仍存在且权限为 0600。
- 在现有 tmux 会话中独立启动语音，不重启其他模块；在线节点包括 `wheeltec_mic_wake`、`xfyun_asr`、`voice_command_router`、`deepseek_chat`、`xfyun_tts`。麦克风串口成功打开，蜂鸣器保持禁用。
- 初次注入 TTS 时状态虽为 `SPEAKING→IDLE`，用户未听到声音。检查发现默认 `playback_device=default` 被 PulseAudio 指向板载声卡；板上唯一 USB 播放端为 `plughw:CARD=Device,DEV=0`，与旧部署成功配置一致。USB PCM 已 100% 且未静音；用户确认 12 秒测试音和修复 DNS 后的讯飞中文 TTS 均可听。
- 当前 Wi-Fi 从路由器取得的 DNS 一度无响应，公网 IP 可达但讯飞/DeepSeek 域名解析卡住。临时将当前接口 DNS 切到 223.5.5.5 和 119.29.29.29 后，两域名约 50 ms 解析，讯飞 TTS 恢复；这是运行时设置，Wi-Fi 重连后可能丢失。
- 硬件唤醒已多次输出角度，但 ASR 报 `write operation timed out`。根因是 `_receive_one()` 为非阻塞轮询设置 1 ms WebSocket 超时后没有恢复，下一帧 `send()` 继承 1 ms；现保存并恢复原超时，避免网络轻微抖动造成发送失败。
- 修复同步到 `/home/wheeltec/ROSCAR-red` 后，xfyun_speech 原生 Humble 构建成功，协议和 WebSocket 超时恢复共 5 项测试通过。为避免与他人正在调整的跟随会话耦合，语音改为独立 `roscar-voice` tmux 会话；未重启或修改跟随进程。
- 真人连续完成两轮完整链路：“你是人类吗？”与“你好吗？”均收到硬件唤醒、LISTENING、ASR_TEXT、DeepSeek ANSWER、TTS `SPEAKING→IDLE`，用户现场听到播报，日志未再出现发送超时。中间两次只唤醒未发出超过阈值的语音被安全丢弃。
- `scripts/run_voice_assistant.sh` 从硬编码 `enable_tts:=false` 改为默认开启，可用 `VOICE_TTS_ENABLED=false` 恢复纯文本模式；配置固定已验证 USB 播放设备。同步更新 README。未修改跟随代码或参数。

## 2026-09-23：当前 main 上车部署与相机内参来源

- 本地 `main` 为 `376f5b8f86459c16a9d29c63d04c468826fa3978`。用同步白名单将当前源码和配置部署到 Jetson 独立目录 `/home/wheeltec/ROSCAR-current`；本地未跟踪的 `camera_info/` 不在部署内容内。复用板上兼容的 `.venv-yolo`，将已有官方 `yolo26s.pt` 复制到新目录，SHA-256 为 `646f8bc3fe0a656803d95c294f7852321748cb29d13466a1af8862e2db384a1b`。未替换现有 `/home/wheeltec/ROSCAR-red` 运行栈或修改自启动。
- 部署前发现旧栈 `person_follower` 与底盘均在运行，在线设置 `/person_follower.enabled=false` 并读回确认。结束时再次读回为 false；新 API 仅在隔离 ROS 域 191 以默认 IDLE 运行 6 秒后正常退出，未启用新底盘节点或跟随。
- Jetson 原生 ROS 2 Humble 构建：13 个主动包、独立底盘 3 包、独立雷达 2 包全部成功。结构检查覆盖 13 个主动包与 187 个 Python 文件；YOLO26s 在 Jetson GPU 设备 0 对空白帧完成推理，返回 0 个检测。该检查仅证明模型和推理依赖可用，不是实景识别或 TensorRT 验收。
- 现有相机节点 `/camera/camera` 的 `color_info_url` 与 `ir_info_url` 均为空；`/camera/get_camera_params` 服务返回左右内参、畸变和外参全部为 NaN。在线 `/camera/color/camera_info` 为 640×480、`fx=fy=570.3422047415297`、`cx=319.5`、`cy=239.5`、D 全零、R 单位阵、P 与 K 同焦距主点。数值与厂商 ROS 2 Astra 驱动 `getDefaultCameraInfo()` 的视场角焦距及默认中心点公式吻合；当前使用的是驱动默认估算内参，不是已确认的设备标定，也未加载本地 `camera_info/` YAML。
- 只读 RGB-D 输入探针改用实际 `/camera/color/image_raw` 和 `/camera/depth/image_raw`：8.23 秒收到彩色 76 帧、深度 74 帧、66 对同步帧，均为 640×480、同一 optical frame；66 对的 0.2–8 m 深度有效比例全部为 0，报告 `metadata_pass=false`。另取一帧 16UC1 深度图，307200 个像素全部为 0。这表明当前深度测距不可验收；同 frame/时间接近也不能证明物理配准。探针默认的 `/camera/color/image_rect`、`/camera/aligned_depth_to_color/image_raw` 在线无流，不应把默认话题当作本机已验证输入。
- 最小审查：核对部署版本、权重哈希、各构建摘要、GPU 空帧、API 启退日志、相机参数服务与 CameraInfo、RGB-D 探针报告及旧跟随开关；`git diff --check` 通过。后续需对当前 ASTRA S 做对应分辨率的实际标定和 RGB-D 对齐/已知距离验证，并排查全零深度。

## 2026-09-23：调整相机摆放后的深度复测

- 用户提示先前全零深度可能与相机摆放有关，故未重启相机或关闭配准。保持原在线配置 `depth_registration=true`，只读采样 20 帧：每帧约 119303–120043 个非零像素，0.2–8 m 有效像素约 119292–120032 个；样本最小有效深度约 0.592 m。先前全零现象已消失，不能据之前快照认定设备或配准故障。
- 再用当前实际话题做 10 秒 RGB-D 输入预检：彩色 193 帧、深度 194 帧、同步并接受 184 对；均无缺流、陈旧流或拒绝原因，深度有效比例均值 36.83%（范围 36.05%–38.23%），`metadata_pass=true`。报告在 Jetson `/home/wheeltec/ROSCAR-current-rgbd-resample.json`。这只证明当前可接收有效深度和基础时间/格式条件，`registration_verified` 仍为 false，物理配准与绝对距离还需另验。相机未重启，旧跟随开关此前已设为 false。

## 2026-09-23：临时相机内参与深度 CameraInfo 修复

- 将用户提供的 Astra YAML 数值复制到已跟踪的 `astra_s_provisional_color.yaml`、`astra_s_provisional_depth.yaml`，通过 `run_astra_camera.sh` 的 color/ir info URL 加载；未把本地原始 `camera_info/` 文件纳入同步。
- 厂商驱动 `getDepthCameraInfo()` 在工厂参数无效时错误地用 NaN 覆盖 YAML 的 K，保存源码备份后用 `deploy/patches/astra_depth_camera_info.patch` 修正条件并在 Jetson 原生重建 `astra_camera`（35.9 秒）。在线重启后，彩色和深度 `/camera_info` 的 K/P/D 均与用户 YAML 一致；深度单帧 307200 像素中有 85012 个 0.2–8 m 有效像素。
- 这些数值是用户提供的临时内参，尚未证明对应实物 ASTRA S、绝对测距精度或 RGB-D 像素配准。重启时旧 systemd 预设使跟随重新启用；已立即关闭并将旧远端 `start_robot.sh` 改为强制 `MOTION_ENABLED=false`，现场读回 false。该安全修改当时仅在旧部署目录，现行本地新入口也强制运动关闭。

## 2026-09-23：取消红色目标，切换当前方案为 B + 雷达 + 语音 + 底盘 + Foxglove

- 用户明确取消红色目标路线。撤回本轮对 `red-layout.json` 的未提交重构；改写 `start_project.sh` 为 Astra + YOLO26s/ByteTrack + N10P + Foxglove + 语音的统一入口，新建 `start_robot.sh` 加入底盘串口收发及 `run_chassis_io.sh`。拒绝 `MOTION_ENABLED=true`，不启动 person_follower 或 motion_guard 的运动模式。旧红色源文件及历史资料保留作为版本记录，不属于当前入口。
- 更新 `b-radar-layout.json`：原始/检测视频、目标状态与 XYZ/距离曲线、相机光学坐标下的目标球、`laser` 平面 N10P 扫描及健康状态、语音识别/回答、底盘里程计/速度和待 SLAM 启用的地图/路径。目标无效时 NaN 和 Marker 消失属于正常状态；目前没有实际三维重建或雷达到相机的已标定外参。
- 旧 `roscar-robot.service` 最后一次可达时仍运行红色 tracker、person_follower（enabled=false）、语音和底盘。尝试通过无密码 sudo 停服务被拒；随后停止了独立的临时雷达 user service。之后 SSH 握手超时/被关闭，尚未完成 B 部署或服务切换，不能声称在线已经运行 B。下一次连接恢复时应先改旧服务入口指向新 B、确认运动继续关闭，再重启并验收话题和 Foxglove。
- 本机最小审查：Bash 语法和 ShellCheck 检查总入口、底盘 I/O 及 B 子入口；布局 JSON 结构覆盖所有面板；待完成在线检查后补记录。临时相机内参仍须重新标定和深度/彩色配准实测。

### 网口完成 B 在线切换与三维坐标验收

- 用户指定改用网口 `wheeltec@192.168.100.2`，链路恢复。Jetson `/home/wheeltec/ROSCAR-current` 同步 B 入口与代码，原生 Humble 重建 `yolo_person_tracker` 成功（4.42 秒）。合成 ROS 专项检查确认：`depth_registered=false` 时仍能输出检测框视频，但目标锁定和三维坐标保持关闭。用户随后明确要求按当前临时内参与驱动配准正常发布坐标，因此在线 `start_robot.sh` 设置 `DEPTH_REGISTERED=true`，同时强制 `MOTION_ENABLED=false`。该开关表示接受当前近似配准用于感知，不代表物理精度验收。
- 旧 systemd 单元的用户可写入口已备份并改为转发 `/home/wheeltec/ROSCAR-current/scripts/start_robot.sh`。杀掉旧主进程触发该单元既有的失败重启机制；中间因新底盘脚本 `set -u` 早于 ROS setup、以及新目录缺 Foxglove 用户运行时，出现数次启动失败。修复脚本次序并链接到已有 `/home/wheeltec/ROSCAR/tools/foxglove-root` 后，单元为 active，当前运行进程为 YOLO tracker/target_transform、Astra、N10P、Foxglove、语音及底盘驱动；无 `red_object_tracker` 或 `person_follower` 进程。单元的 systemd Description 仍是旧红色文字，因为 `/etc/systemd/system/roscar-robot.service` 修改需管理员权限；实际 ExecStart 已转发 B。
- 在线 `/perception/target_state` 为 `source=yolo`、`is_simulated=false`；真实人体候选检测置信度示例 0.81。调用 `/perception/lock_target` 成功锁定 `1:1`，得到有效相机光学 XYZ 约 `(-0.126,-0.120,0.643)m`；该 ID 消失后状态正确变 LOST，需要重新锁定。再次锁定 `1:51` 后 6 秒内采集 75 条有效 TRACKING，XYZ 示例首尾约 `(-0.426,-0.330,1.584)` 与 `(-0.423,-0.330,1.584)m`。上述为发布稳定性样本，不是已知距离的准确度测量。
- 8 秒独立采样收到目标状态 136、检测图 37、雷达点云 52、雷达健康状态 34、里程计 130 条；雷达状态为 OK、约 10 Hz，串口成功打开。`/cmd_vel` 只有底盘驱动 1 个订阅者、0 个发布者；未启动车辆运动。语音节点与话题在线，但本轮未做真人问答复测。
- `foxglove/b-radar-layout.json` 已同步到 Jetson，并作为本轮交付文件；用户表示自行导入 Mac Foxglove，故未声称客户端已导入或面板已视觉验收。布局中的地图/路径面板为预留，当前未运行 SLAM/Nav2；N10P 点云是二维扫描平面。
- 本轮最终回归：Jetson 隔离 ROS 域 191 的未确认配准二维检测测试、域 192 的合成 B 锁定/测距/丢失/异常测试、域 193 的并发响应测试均通过。合成 B 测试初次直接在 Jetson 运行因硬编码 `/workspace/scripts` 容器路径报错，改为相对测试文件定位脚本后复跑通过；该错误不是 B 算法失败。Bash 语法、ShellCheck、Python 编译、布局 JSON 和 `git diff --check` 均通过。线上目标需由真人再次进入画面并在轨迹变化后重锁；本轮未做已知距离精度标定。
- 用户指定网口调试后，Foxglove 主连接地址改为 `ws://192.168.100.2:8765`（Wi-Fi 仍可备用）；Mac 到网口 8765 TCP 连接成功。布局不包含连接地址，用户自行在 Foxglove 导入和连接，客户端实际显示仍待用户确认。

## 2026-09-23：Foxglove“没有消息”与 B 坐标 NaN 复查

- 小车服务保持 active、NRestarts 不再增长，Bridge 监听 `0.0.0.0:8765`，网口和 Wi-Fi TCP 均可达。Bridge 日志显示 Foxglove 客户端订阅了原始彩色图、检测图和雷达，但对 Wi-Fi 客户端持续报 `outbox ... full`；5 秒 ROS 直读同时收到彩色 83、检测图 25、雷达点云 26、目标状态约 84 条。判断为客户端/桥接高带宽排队，非 ROS 话题整体断流。
- B 跟踪器新增两个 320×240、质量 70 的 JPEG CompressedImage 预览话题 `/perception/color_preview/compressed` 和 `/perception/detections_preview/compressed`；原有未压缩接口保留。Foxglove B 布局改订压缩预览，不再订 640×480 原始视频。Jetson 原生重建通过，隔离 ROS 的未配准二维检测、合成 B 运行和并发回归通过。在线 6 秒每个预览收到 36 帧，平均每帧约 11.8/12.6 KB；Foxglove 官方 Image 面板支持 ROS 2 `sensor_msgs/msg/CompressedImage`。用户仍需重新导入更新的布局；本轮未声称 Mac 客户端已显示成功。
- 当用户报告 `position_valid=false`、XYZ NaN 时，在线状态为 SEARCHING；一次手动锁定返回 `No fresh tracked candidates`。短窗 23 帧检测中 19 帧有人，但部分候选无 ByteTrack ID；一次深度帧全零。随后 5 秒复测深度每帧有约 5.6–6.7 万个非零像素，检测出现 ID `1:27`。再次调用锁定成功选中 `1:116`；5 秒中 64 条 TRACKING/坐标有效、37 条 LOST/无效，最近有效 XYZ 约 `(-0.052,-0.124,0.989)m`。相机预览中人体部分被前景物体遮挡、头部靠近画面边缘，轨迹不稳定；坐标无效是目标/深度当前观测条件，不是内参矩阵回到 NaN。运动继续关闭。

## 2026-09-23：单人自动锁定、唤醒应答与开机入口核对

- B 自动锁定设为整机入口默认开启：唯一已跟踪候选连续 3 帧后锁定；多人时等待手动选人；手动释放会抑制自动重锁。自动模式遇到 ByteTrack epoch 更新时先丢弃旧 ID 再重新取得当前单人。Foxglove 布局增加 ASR/TTS 状态面板，两个状态话题每秒重发当前状态。
- 当前运行期间自动取得 ID `1:1`，TargetState 为 TRACKING、位置有效；6 秒只读采样 98 条状态，98 条位置有效，期间唯一目标 ID 为 `1:6`，末值约 `(0.222,-0.165,0.717)m`。目标 ID 可随跟踪器重置而更新；坐标采用临时内参与驱动配准，样本未校验真实物理距离。
- 对照旧语音实现后确认当前版本原本未订阅唤醒词发布“我在”。DeepSeek 节点现对精确 `/voice_words=小车唤醒` 发布本地应答到 TTS，ASR 等待 1 秒并避开仍在播放时才开始识别。在线真实唤醒日志确认发布应答，用户确认听到“我在”；随后 ASR 返回文本并触发 DeepSeek/TTS 完整回答，但该次样本识别为“我操。”，识别准确率仍需短句复测。ASR/TTS 状态话题现持续显示 IDLE/LISTENING/SPEAKING 等状态。
- 原生 Jetson Humble 重建 `yolo_person_tracker`、`perception_bringup`、`xfyun_speech`、`deepseek_ros2` 成功；构建前后服务重启成功。整机含 Astra、B、N10P、Foxglove、语音和 `mini_akm` 底盘遥测，无跟随/运动节点，`/cmd_vel` 发布者数为 0。
- `roscar-robot.service` 已是 enabled，实际经 `/home/wheeltec/ROSCAR-red/scripts/start_robot.sh` 的兼容入口转发到 `/home/wheeltec/ROSCAR-current/scripts/start_robot.sh`；新入口强制 `MOTION_ENABLED=false`。冲突的旧 `roscar-route-a.service` 为 disabled/inactive。开机启动已配置并验证服务重启，不执行整车重启，因此尚未做断电后的冷启动实测。系统级 unit 文件仍有旧红色描述及 `MOTION_ENABLED=true` 环境字段，但入口脚本无条件覆盖为 false。
## 2026-09-28：ROS 外部接口在线核对

- 按用户要求检查此前编写的 ROS 外部调用接口。在线小车通过 `roscar-wifi`、ROS 域 182 读取到 Route B 的 YOLO、Astra、N10P、语音和 `mini_akm` 底盘遥测；`/cmd_vel` 发布者数为 0，底盘只有一个订阅者，未触发运动。
- 实际调用 `/perception/release_target` 成功；无人/无新鲜跟踪轨迹时调用 `/perception/lock_target` 按设计返回 `No fresh tracked candidates`，随后再次 release 成功。`/perception/target_state` 能发布真实 `source=yolo`、`is_simulated=false` 状态；当前无目标时为 SEARCHING、位置无效；`/perception/detections`、`/scan`、`/odom`、`/PowerVoltage` 均可单次读取，雷达约 10 Hz，电压约 11.95 V。
- 在线服务清单没有 `/control/set_mode`、`/control/arm`、`/control/stop`、`/control/disarm`，话题清单没有 `/chassis/cmd_vel`、`/control/state`；因此供外部程序使用的 `roscar_api` 控制接口尚未部署到当前 `/home/wheeltec/ROSCAR-current`，本轮不将其描述为在线通过，也未尝试发送速度。
- 本机直接运行 API Python 集成测试因 Mac 环境没有 `rclpy` 无法执行；在线只读调用和安全锁定/释放调用完成。最小审查为 `git diff --check`，待具备 ROS 2 Humble 容器或 Jetson 隔离测试环境后再运行 `tests/test_api_control.py` 与 `tests/test_api_examples.py`。
- 随后复用 Jetson 已安装的 Humble 工作区，在隔离域 194 运行 `tests/test_api_control.py` 与 `tests/test_api_red.py`，分别通过外部 EXTERNAL/FOLLOW 模式、速度校验、看门狗、障碍/TF/重复发布者故障和红色检测接口检查。`tests/test_api_examples.py` 的默认 2 秒启动等待在 Jetson 上未收到首条状态而失败；手动将 API launch 等待 5 秒后，`/control/state`、`/control/set_mode`、`/control/arm`、`/control/stop`、`/control/disarm` 及零速输出均实测通过，`motion_enabled=false` 时 arm 正确拒绝。
## 2026-09-28：真实外部控制保护测试与 N10P 自启动移除

- 在在线 ROS 域 182 启动临时 `motion_guard`，使用 `EXTERNAL` 模式、0.02–0.03 m/s、0.3–0.5 秒短脉冲测试 `/chassis/cmd_vel` 外部调用。保护层正确拒绝 arm，原因是 N10P 扫描含大量 `+inf` 无回波值且 `allow_infinite_clear=false`（`unknown_scan_return`）；未向底盘发送非零 `/cmd_vel`，测试进程和临时 TF 已清理。未为测试放宽雷达安全规则。
- 已将当前开机入口的 N10P 改为显式可选，`WITH_RADAR` 默认 `false`；保留 `scripts/run_radar.sh` 和手动 `with_radar:=true` 路径。同步到 Jetson 后通过杀掉旧入口触发服务重启，`roscar-robot.service` 保持 active，`roscar_n10p`/`radar_health` 节点及 `/scan`、`/radar/*` 话题均已从自启动在线图消失，`/cmd_vel` 仍无发布者。
- 用户要求恢复原 A 语音版本；已在 Jetson `/home/wheeltec/ROSCAR-current` 备份当前语音源码到 `backups/voice-before-a-20260928-114524`，复制 `/home/wheeltec/ROSCAR-red` 的 A 版本 `xfyun_speech`、`deepseek_ros2`、`voice_command_router`、启动脚本，原生构建 3 包成功，并重启在线服务。A 版本唤醒延时参数为 `wake_cycle_delay_s=2.0`，保留“我在”唤醒应答。
## 2026-09-28：移除 N10P 后的 ROS 接口复测

- 在线服务 `roscar-robot.service` 保持 active；节点为 Astra、YOLO、相机、Foxglove、A 版本语音和 `mini_akm` 底盘。确认 N10P 节点、`/scan`、`/radar/*` 均不在 ROS 图中。
- 复测 `/perception/target_state`（真实 `source=yolo`、当前 LOST、位置无效）、`/perception/detections`（空检测数组）、彩色/深度/CameraInfo 话题、目标 Marker/压缩预览、`/PowerVoltage`（11.565 V）和 `/odom`，均可读取；`/cmd_vel` 发布者仍为 0、底盘只有 1 个订阅者。
- 复测 `/perception/release_target` 成功；当前无新鲜跟踪候选时 `/perception/lock_target` 正确返回 `No fresh tracked candidates`。语音节点和 `/voice/start_listening` 服务存在。
- 当前在线入口没有 `/control/set_mode`、`/control/arm`、`/control/stop`、`/control/disarm` 或 `/chassis/cmd_vel`；外部控制 API 仍只在隔离域完成过，N10P 关闭后不能进行真实 arm，因为保护链路缺少扫描输入。没有绕过保护发车。
## 2026-09-28：N10P 关闭后的真实底盘 ROS 脉冲复测

- 用户现场观察并授权直接做真实控制链路测试。当前在线 `/cmd_vel` 预检为 0 个发布者、1 个 `wheeltec_robot` 订阅者；通过 ROS 域 182 发布端完成 DDS 发现后，先发零速，再发 `0.02 m/s × 0.30 s`，最后持续零速。收到 27 条 `/odom`，但速度和位置无变化。
- 第二次以 `0.05 m/s × 0.50 s` 重测，发布端确认订阅者数为 1，运动期间收到 11 条 `/odom`，`max_abs_vx=0`、`x_delta=0`，结束后 `/cmd_vel` 恢复 0 发布者、底盘仍为唯一订阅者。结论：ROS 发布/订阅发现链路正常，但本次未观察到下位机运动或里程计响应，不能称真实位移通过；未继续提高速度或延长时间。
- 由于 N10P 已关闭，正式 `motion_guard` 无 `/scan` 输入，不能进行带保护的 `arm` 测试；本次直接脉冲仅验证底盘 ROS 接口，后续应检查驱动命令帧、底盘固件使能/急停状态和串口回传，而不是绕过保护反复发车。
## 2026-09-28：按用户要求默认关闭雷达运动门禁

- `motion_guard` 新增只读参数 `radar_required`，并按用户要求将默认值改为 `false`；EXTERNAL/FOLLOW 均可在无 `/scan` 时完成 arm，仍保留显式 arm、请求时效、速度限制、唯一发布者和底盘命令超时。设置为 `true` 时恢复雷达数据、TF 和包络检查。
- `roscar_api` launch 增加同名参数，默认 false；README 与 ROS 接口文档已说明无雷达模式没有障碍保护，必须人工清空环境并准备急停。
- 已同步 Jetson `/home/wheeltec/ROSCAR-current`，原生重建 `motion_guard` 与 `roscar_api` 成功。在在线 ROS 域 182 启动临时 guard（无 N10P、无 `/scan`），正式调用 `/control/set_mode EXTERNAL` 和 `/control/arm`，arm 返回 `ready_without_radar`；`roscar_api chassis` 随后正常退出并调用 stop，未留下测试发布者。该次证明无雷达正式控制门禁已打开；实际里程计样本仍未形成可观测运动，不能称底盘位移验收通过。
## 2026-09-28：无雷达外部接口状态机复测

- 在在线 ROS 域 182、未启动 N10P 的情况下，正式启动临时 `motion_guard(radar_required=false)` 并运行外部接口验证。`IDLE → EXTERNAL` 成功，`/control/arm` 返回 `ready_without_radar`；外部 `/chassis/cmd_vel` 请求经 guard 转为 `/cmd_vel`，共收到 51 条输出，其中 8 条非零，最大 `linear.x=0.03 m/s`；调用 stop 后回到 STANDBY，测试发布者清理。
- 重新验证请求超时：arm 成功后停止发送约 0.5 秒，guard 进入 `FAULT`，`last_fault=request_timeout`，输出速度回零。无雷达模式的门禁和失联停车状态机均通过。
## 2026-09-28：现场观察方向控制脉冲

- 用户现场观察并明确要求执行直走、左转、右转。通过正式无雷达 `motion_guard` 外部接口顺序发送并在段间归零：直走 `0.03 m/s × 0.5 s`，左转 `+0.20 rad/s × 0.5 s`，右转 `-0.20 rad/s × 0.5 s`。
- 三段均完成 DDS 发现和请求发布：直走 guard 输出 11 条、最大 `linear.x=0.03`；左转输出 10 条、最大 `angular.z=0.20`；右转输出 10 条、最大 `angular.z=-0.20`。最后调用 stop、持续零速并清理临时 guard。
- `/odom` 仍未显示位置或速度变化，因此记录为 ROS 方向命令链路通过、实际车体位移/转向未由里程计证明；未继续提高速度或延长时长。
## 2026-09-28：底盘节点与串口复核

- 针对“是否没开底盘”的疑问，在线复核确认 `/wheeltec_robot` 正在运行，实际进程打开 `/dev/wheeltec_controller`（解析到 `/dev/ttyCH343USB1`），车型参数为 `mini_akm`，命令/反馈超时均为 0.5 s。
- `/PowerVoltage` 可读约 11.40 V，`/odom` 持续发布，驱动日志显示 `serial port opened`；因此底盘 ROS 节点和串口通信是开启的。当前临时控制节点已清理，`/cmd_vel` 目前回到 0 个发布者，这是正常待机状态。
## 2026-09-28：提高速度后的真实直行测试

- 用户现场观察并要求提高速度、延长时间。经 `/cmd_vel` 预检无其他发布者且底盘唯一订阅者存在后，通过正式无雷达 guard 发送 `0.08 m/s × 2.0 s` 直行，随后持续零速 1.5 秒并调用 stop。
- `arm=ready_without_radar`，guard 记录 40 条输出，最大 `linear.x=0.08 m/s`、角速度为 0。结束后临时 guard 清理，`/cmd_vel` 恢复 0 个发布者。
- 结束时 `/odom` 位置约 `x=0.0856 m, y=-0.0061 m`，速度已回到 0；这是本轮首次观察到与直行指令一致的非零里程计位移。实际车体方向和地面位移仍以用户现场观察为准。
## 2026-09-28：mini_akm 左右弧线真实动作完成

- 纯角速度 `linear.x=0` 的左/右转只验证了指令输出，`mini_akm` 不会原地旋转，里程计姿态无变化。随后改用实际转向车动作：左弧线 `linear.x=0.08 m/s, angular.z=+0.20 rad/s, 2 s`，右弧线 `linear.x=0.08 m/s, angular.z=-0.20 rad/s, 2 s`，两段之间持续零速。
- 正式无雷达 guard 两段均 arm/stop 成功，各收到 40 条 guard 输出。里程计：起点约 `(x=0.0854,y=-0.0061,yaw=-0.0273)`；左弧线后约 `(0.2001,0.0063,0.2018)`；右弧线后约 `(0.3563,0.0012,-0.2144)`，已观察到前进和左右转向姿态变化。
- 测试结束后临时 guard 清理，`/cmd_vel` 恢复 0 个发布者，底盘保留唯一订阅者。

## 2026-09-28：语音 ROS 控制与模式编排源码改造

- 新增 `roscar_interfaces/msg/VoiceCommand` 与 `VoiceCommandResult`，语音动作可通过类型化 ROS 消息进入统一路由；保留旧 `/voice/tool_call` 蜂鸣器 JSON 兼容入口。
- `voice_command_router` 改为本地中文规则优先、DeepSeek 未识别文本兜底的控制状态机，支持手动/跟随/待机切换、前进/倒车/左右转、停止、状态查询和蜂鸣器；手动请求仅发布 `/chassis/cmd_vel`，最终 `/cmd_vel` 仍由 `motion_guard` 独占。连续 2 秒没有新的运动命令会归零并撤销授权，启动不自动 arm。
- DeepSeek 工具扩展为 drive、set_control_mode、arm、stop、query_status、buzz，严格校验结构化参数并等待 `VoiceCommandResult`；语音配置和文档已同步更新。
- `motion_guard`/`SafetyConfig` 增加 `max_reverse_mps`，默认允许手动负线速度 `-0.15..0.15 m/s`；跟随和导航生产者仍可保持不倒车策略，纯安全测试增加负速度边界。
- 本机已完成 Python 编译、XML/YAML 解析、git diff --check 和本地解析/倒车边界检查；通过 Colima 启动的 ARM64 ROS 2 Humble 容器完成消息生成、主动包构建及既有控制/导航/底盘回归。第三次全量回归在既有 `test_api_examples.py` 的目标服务示例出现一次时序性断言失败，随后单独重跑该测试通过；语音专属 pytest 在容器中执行，未部署 Jetson 或执行实车语音控制。
- 新增 `docs/语音控制交接文档.md`，记录接口、启动顺序、安全状态机、DeepSeek 工具约束、验证证据和现场验收边界，供后续部署与实车测试使用。
## 2026-09-28：自启动跟随门禁按用户要求关闭

- 自启动入口 `start_robot.sh` 现在启用 `WITH_FOLLOWER=true`、`MOTION_ENABLED=true`、`auto_arm=true`、`radar_required=false`，并仅在该入口传入 `geometry_confirmed=true`、`mount_calibrated=true`、`stopping_model_confirmed=true`；安全配置文件默认值仍不改。
- `follow.launch.py` 新增三个确认参数并传给 guard。Jetson 服务重启后 `motion_guard`、`person_follower`、YOLO 和底盘均在线；参数读回 geometry/mount/stopping/auto_arm 均为 true。当前无人/无有效目标时状态为 `STANDBY`、`target_invalid`、速度保持零，等待有效 YOLO 目标后自动 arm。
## 2026-09-28：自启动跟随现场诊断

- 用户站到相机前后，YOLO 确实检测到人（例如 ID `1:32`，置信度约 0.84），但 `/perception/target_state` 连续返回 `TRACKING / Tracked; torso depth rejected`，`position_valid=false`。10 秒采样 168 帧中仅 3 帧有有效三维位置，跟随请求保持零速。
- `motion_guard` 因 `target_invalid` 进入 FAULT；调用 `/control/stop` 后，目标深度仍无效，自动 arm 不能恢复。当前问题是注册深度的躯干 ROI 质量不足，不是跟随节点未启动或门禁参数未生效。
## 2026-09-28：自启动跟随目标重锁并自动 arm

- 用户现场开始跟随后，初始目标因 RGB-D 深度拒绝和 ByteTrack 流重置进入 `target_invalid`/`Selected track absent`；调用 `/perception/lock_target` 成功锁定 `1:116`。
- 重锁后目标状态为真实 `TRACKING`、`position_valid=true`、`detail=Tracked; registered torso depth`，示例坐标 `(0.198,0.020,0.934)m`、水平距离约 `0.954m`。`motion_guard` 自动进入 `ARMED`，`ready=true`、`reason=ready_without_radar`，跟随链路已具备运动条件。
## 2026-09-28：跟随运行中目标轨迹丢失

- 用户反馈开机跟随不动作。在线状态为 `motion_guard=FAULT`、`reason=target_invalid`；`person_follower` 仍 enabled，但 `/control/cmd_vel_request` 和 `/cmd_vel` 均为零。
- `/perception/target_state` 返回 `Selected track absent; explicit relock required after stream reset`，当前目标 ID 已变化到 `1:268`/`1:270`；调用 `/perception/lock_target` 时返回 `No fresh tracked candidates`。原因是 YOLO/深度轨迹在运行中重置，自动跟随不会在故障后自动换人续动，需目标稳定后重新锁定。
## 2026-09-28 完全移除 motion_guard 运行门禁

- 按用户要求，跟随启动链路改为 `person_follower → /cmd_vel → wheeltec_robot`，`follow.launch.py` 不再启动 `motion_guard`。
- `person_follower` 新增 `direct_cmd_vel` 参数；直接模式发布 `geometry_msgs/Twist`，仍保留真实目标、观测时效、唯一目标发布者检查、跟随速度上限和退出归零。
- Jetson `/home/wheeltec/ROSCAR-current` 已重建 `astra_body_adapter`、`motion_guard` 并重启自启动。在线核对无 `/motion_guard`，`/cmd_vel` 仅有 `person_follower` 一个发布者和底盘一个订阅者。
- 本轮本机 Python/Bash 语法检查、`git diff --check` 通过；Jetson 原生构建通过。已明确失去 motion_guard 的请求 watchdog、故障锁存、雷达/障碍检查和单出口保护。
## 2026-09-28 Foxglove 可视化恢复

- 在线检查确认 `foxglove_bridge` 正在 `0.0.0.0:8765` 监听，ROS `/perception/target_state`、检测框和压缩图像话题均有数据。
- Foxglove 原连接的发送队列因旧原始图像面板持续出现 `outbox full`，导致图像面板等待消息；重启 B/Foxglove 子模块、重新连接 `ws://192.168.1.240:8765` 后，压缩检测图像主题恢复订阅。
- 当前 Foxglove 仍会显示目标状态；目标是否有效由 YOLO 跟踪状态决定，与 Bridge 连接问题分开。
## 2026-09-28 Foxglove 队列问题复核

- 截图所示状态确认为客户端数据流卡住：ROS 话题与 Bridge 频道存在，客户端也成功订阅了 `/perception/color_preview/compressed`、`/perception/target_state` 和 Marker，但 Bridge 随后持续报告该客户端 `outbox full`。
- 重启 Foxglove 客户端和 Bridge 后可重新建立订阅，约 20 秒后队列再次填满；当前未将其误判为相机或 YOLO 节点停止。需要降低 Foxglove 图像流负载或进一步调整 Bridge/客户端队列策略。

## 2026-09-28 Foxglove 网络诊断补充

- 更正此前将 `outbox full` 直接归因于客户端或原始图像的判断：该日志仅证明发送积压，不能单独确定根因。
- 本轮 `ss -tin` 实测 Foxglove 连接 bytes_sent=916754、bytes_retrans=273672、Send-Q=92672，delivery_rate=13976bps、cwnd=1；ping 16–422ms，SSH 偶发连接超时。证据支持网络传输受阻，尚未确定具体无线干扰来源。
- Jetson 无线接口 wlP1p1s0 使用 2412MHz、信号 -65dBm、收发协商速率 6Mbps、Power save on；有线 enP8p1s0 DOWN。尝试关闭无线省电被 sudo 密码要求阻止，未修改该设置。本轮未重启运行栈。
## 2026-09-28 Astra 深度躯干测距容错调整

- 在线采样确认深度为 640×480 `16UC1`，全图有效深度约 52%–56%；躯干 ROI 有效率常在 27%–31%，原 `measure()` 的 30% 门槛和 IQR 阈值会频繁触发 `Tracked; torso depth rejected`。
- `yolo_person_tracker/depth.py` 将最低有效比例由 0.30 调为 0.15，并将离散度上限调整为 `max(0.35, 0.3*z)`；仍保留 0.2–8m 范围、中位数和最少样本检查，避免把全背景/稀疏噪声当作距离。
- 本机 3 项深度单测通过；Jetson 原生重建 `yolo_person_tracker` 成功。重启后在线连续采样已恢复 `Tracked; registered torso depth`。
## 2026-09-28 YOLO 人体框多区域深度测距

- 按用户要求，`yolo_person_tracker` 不再只依赖躯干 ROI；同一 YOLO 人体框内依次尝试躯干、头肩、下身、左侧和右侧区域，选择有效深度支持率最高的稳定区域。
- 每个候选区域仍限制在对应 YOLO 框内，并保留 0.2–8m 范围、最少样本、深度离散度和中位数过滤，避免跨目标或背景深度混入。
- TargetState 细节改为 `Tracked; registered body-part depth` / `Tracked; body-part depth rejected`，明确不再声称一定来自躯干。
- 本机深度单测和语法检查通过；Jetson 原生 `yolo_person_tracker` 构建成功并已重启。重启后在线当时无人/未形成稳定候选，尚未完成真人多部位深度实测。
## 2026-09-28 背对相机跟随优化

- 多区域测距进一步改为在同一 YOLO 框内优先选择“最近的稳定深度区域”，降低背景墙/地面有效像素较多时抢占人体距离的风险，适合背部、肩部或腿部部分深度可见的情况。
- 新增深度单测：躯干区域为空时，能从同一人体框的下身区域恢复距离；4 项深度单测通过。
- Jetson 已原生重建并重启 `yolo_person_tracker`。当前目标已释放，现场没有稳定目标可作真人背对相机验收；需要用户站到相机前重新锁定后验证跟随轨迹。
## 2026-09-28 数据集与 DJI Neo 方案评估

- 检索并核对 Bonn RGB-D Dynamic Dataset：单个 `rgbd_bonn_crowd` 约 515.9 MB，RGB/深度已配准；本机可用空间约 31 GB，但官方 ZIP 下载速度低于约 20 KB/s，无法在本轮合理完成，已清理未完成的部分文件。
- DJI Neo 官方规格/手册确认：原版 Neo 的跟随主要是视觉人体跟踪，只有向下视觉定位，明确不支持避障；Follow Me 丢失目标时悬停，目标接近时不后退。它可借鉴目标保持、丢失悬停和从后方跟随策略，不能直接替代当前 RGB-D 深度方案。
- 公开 RGB-D 数据集可用于算法回放，但不同相机的深度空洞和标定分布与 Astra S 不同；针对当前问题，优先录制 Astra S 的真实背对相机 ROS bag 更有价值。

## 2026-09-28 Mac 端 Astra RGB-D 录制入口

- 新增 `scripts/mac_record_rgbd.py` 与 `docs/Mac端Astra RGB-D录制.md`。脚本使用 OpenNI2 同步采集彩色/深度，硬性要求深度到彩色配准，保存 16 位毫米深度 PNG、彩色 PNG、帧时间戳和元数据；缺少 SDK、相机或配准能力时明确退出，不把 RGB-only 视频冒充 RGB-D 数据。
- 本机已确认 Pillow/numpy 可用，但当前 USB 列表没有 Astra S，Python 也没有 OpenNI2 绑定，因此只完成脚本语法/帮助检查，尚未进行实机录制。插入相机并准备对应 OpenNI2 动态库后再做现场采集。

## 2026-09-28 拓展坞识别 Astra S

- Mac 通过 `ioreg -p IOUSB` 已识别 `ORBBEC ASTRA S`，说明拓展坞 USB 透传正常；`system_profiler SPUSBDataType` 未显示完整条目只是枚举工具差异。
- 已创建本地 `.venv-mac-rgbd`/`.venv-mac-rgbd39` 并安装 `openni`、Pillow、numpy。Python 3.14 的 `openni` 绑定不兼容，Python 3.9 可导入，但因缺少 `libOpenNI2.dylib` 尚不能初始化；官方 OpenNI_SDK 当前发布资产没有 macOS 二进制包。
- 尝试从上游源码构建 OpenNI2：补齐 Homebrew `libusb` 后，编译阶段通过，链接阶段因上游旧 Makefile 默认包含已废弃 i386 架构且 macOS CoreFoundation/IOKit 链接配置过时失败。临时源码和构建目录已清理；未把未验证二进制放入项目。

## 2026-09-28 找到 Astra S 的 macOS 适配库

- 从官方 OrbbecSDK v1.10.16 macOS ARM64/x86 发布包验证 `libOrbbecSDK.1.10.16.dylib`：Mac 实际识别并创建 `Astra S`，PID `0x0402`，固件 `RD109Y-007`，列出 IR/Color/Depth 三个传感器。该库内部兼容 Astra S 的传统 OpenNI 协议，绕过了缺少 macOS `libOpenNI2.dylib` 的问题。
- 新增 `scripts/mac_record_orbbec_rgbd.cpp` 与 `scripts/mac_record_orbbec_rgbd.sh`：使用 Orbbec SDK v1 的硬件 D2C 对齐和 `waitForFrames` 成对取帧，保存 RGB8/16 位毫米深度原始帧、时间戳和元数据。实测确认 Astra S 不支持新版 SDK 的显式 `enableFrameSync`，已移除该硬失败条件；项目录制器尚未完成一段正式数据采集。
- 录制器短测发现 Astra S 在当前拓展坞/USB2.0 链路下，SDK 能枚举并创建设备，但启动彩色流时报 `Match openni video mode failed`；尝试硬件/软件/关闭 D2C、320×240 与 640×480 配置均未形成帧。官方枚举与设备打开已验证，正式录制仍需调整 USB 直连/供电或使用 SDK 支持的确切模式。

## 2026-09-28 Astra OpenNI Viewer 实机验证

- Mac 已安装 `/Applications/Astra OpenNI Viewer.app`。通过拓展坞连接的 Astra S 已在该应用中显示实时深度伪彩色画面，底部状态显示 `Capture Formats - Depth: Lossless | Image: Lossy | IR: Lossless`，证明 Viewer 的旧 OpenNI 适配链路能实际取流。
- 当前 Viewer 画面显示 `Image registration is off`；它适合先做深度/彩色流与录制验证，但要用于人体框深度测距仍需打开配准或保存原始流后按标定离线配准。其 File 菜单提供 Save/Save As，尚未把保存动作当作 RGB-D 数据录制完成。
- 已通过 Viewer 快捷键 `i` 打开配准，状态栏确认 `Image registration is on`；再用 `s` 开始、`x` 停止录制约 10 秒，生成约 87 MB 的 `Captured.oni`。日志显示彩色/深度约 30 FPS，已归档到 `data/rgbd-recordings/astra-viewer-20260928-152549/`，附带录制说明，可用于后续 ONI 原始流回放。
- 覆盖行为已实测：再次按 `s` 录制约 3 秒、按 `x` 停止后，固定路径 `~/Library/Orbbec/OpenNI-MacOSX-x64-2.3/Tools/Captured.oni` inode 未变、mtime 更新、大小由 91,036,130 变为 74,888,746 字节，确认新录制会原地覆盖旧文件。后续必须先归档再开始下一次录制。

## 2026-09-28 语音交接续办与本版本无雷达入口

- 用户明确本版本不启动雷达。start_robot.sh 固定 WITH_RADAR=false；start_project.sh 与 B/Foxglove 脚本仅在显式启用时加载 radar_install、检查 lslidar_driver，关闭时不再依赖雷达构建环境。运行 guard 保持 radar_required=false，交接文档同步说明无雷达障碍保护。
- 最小审查发现既有 guard 故障注入测试依赖旧默认值：未启用雷达却断言障碍 FAULT；已只为合成测试显式设置 radar_required=true，保留障碍/TF 故障覆盖，不影响实际启动配置。此前将该失败描述为时序问题不准确，现予更正。
- ARM64 Humble 容器主动包与底盘包构建、guard/真实 C++ 驱动伪串口专项及语音 pytest 通过，证据 artifacts/voice-targeted-verification.log。全量回归另在 API 示例模式切换时失败，未声明全量通过，日志 artifacts/voice-follow-regression-final.log。
- Bash 语法、ShellCheck、git diff --check 通过。复用原有镜像，临时容器及内部构建输出随 --rm 清理。Wi-Fi SSH 连接超时，未部署、未启动实物雷达或执行实车运动；远端最新控制架构仍需在线核对。

## 2026-09-28 Astra Viewer 自动归档

- 修改 `/Applications/Astra OpenNI Viewer.app/Contents/MacOS/launcher`：启动时先归档遗留 `Captured.oni`；Viewer 运行期间轮询文件，检测到录制停止且大小/mtime 稳定 3 秒后自动复制到 `data/rgbd-recordings/astra-viewer-auto-YYYYMMDD-HHMMSS/`。
- 已实测一轮 `s` 录制约 5 秒、`x` 停止，自动生成一份约 125 MB 的归档；启动初始化产生的两份旧测试归档已清理。`zsh -n` 与 `git diff --check` 通过。

## 2026-09-28 Astra Viewer 完整录制可用性检查

- 用户刚完成的一段自动归档录制位于 `data/rgbd-recordings/astra-viewer-auto-20260928-155338/Captured.oni`，文件大小 131,013,270 bytes（约 125 MiB），伴随自动归档 README。
- ONI 内容包含 `Astra`、`Depth`、`Image`、`oniPixelFormat`、`RegistrationType` 等容器/流元数据；文件不是空文件或截断到零长度。对应 Astra Viewer 会话约 104 秒，稳定区间 RGB/Depth 约 29.5–30 FPS。
- 会话日志有两段短暂 USB 帧损坏/序号跳变告警，主要集中在启动早期和约 92 秒处；约 90 秒处 FPS 短降至 26–27，随后恢复约 30 FPS。结论：录制可用于 ONI 回放和离线算法验证，但不视为逐帧无损数据；若训练需要连续无缺帧，建议改用直连 USB、避免扩展坞并分段录制。
- 本轮只读检查，未修改录制文件；`git diff --check` 通过。

- 2026-09-28 15:53 录制回放：为 Astra Viewer 启动器增加可选 `ASTRA_VIEWER_URI`，可通过应用方式加载指定 ONI 文件；已打开 `astra-viewer-auto-20260928-155338/Captured.oni`，画面同时显示深度伪彩与 RGB，确认该段可直接回放。`git diff --check` 通过。

- 2026-09-28 最新录制复核：归档中最新文件为 `astra-viewer-auto-20260928-160126/Captured.oni`，584,212,592 bytes；15:59:02、15:59:31、16:00:13、16:01:26 四个归档内容 SHA-1 相同，属于同一段录制的重复归档。已加载 16:01:26 回放，Astra Viewer 正常显示深度与 RGB。

- 2026-09-28 16:01 录制归档清理：用户确认保留 `astra-viewer-auto-20260928-160126`；删除 15:59:02、15:59:31、16:00:13 三个经 SHA-1 确认完全相同的重复目录，15:35 和 15:53 两段独立录制保留。

- 2026-09-28 录制归档去重：启动器增加 SHA-1 标记，重复启动/回放同一 `Captured.oni` 时跳过重复复制；文档同步说明。清理后保留四段不同内容（15:25、15:35、15:53、16:01），`git diff --check` 通过。

- 2026-09-28 配准状态核对：日志确认最新 16:01 录制会话曾成功设置 `Depth.Registration=1`（约会话 30.22–32.48 秒、33.20–33.68 秒），但随后又切回 0；因此该录制不是全程配准。回放状态栏显示 off 与文件实际曾短时开启并不矛盾。

- 2026-09-28 Astra Viewer 启动修复：回放请求清除后发现启动器 heredoc 终止符缩进导致 zsh parse error，已修正并用 `zsh -n`、实际启动和 Astra 实时画面验证；应用恢复正常打开，URI 为 `(NULL)`。

## 2026-09-28 网口部署语音版本（未完成最终语音验收）

- 用户接入网口后授权实际部署。通过 `roscar-ethernet` 同步白名单到 Jetson `/home/wheeltec/ROSCAR-current`，未同步凭据、权重或录制文件；远端 13 个主动包原生 Humble 构建成功。
- 重启后的项目栈确认相机、YOLO、person_follower、mini_akm 底盘、ASR/TTS 和 `motion_guard(radar_required=false)` 启动，未发现 `/scan` 或 `/radar/*`，无雷达节点。
- 现场语音启动暴露远端旧 overlay 的 Python 路径问题：`voice_command_router` 与 `deepseek_chat` 导入 `roscar_interfaces.msg` 失败并退出，ASR/TTS 仍在。当前已在本地源码加入活动工作区接口路径优先导入，并准备重新同步/重建。
- 修复同步过程中网口 SSH 被 `192.168.100.2` 主动断开，后续连接仍失败；因此本轮不能声称语音控制已在车上可用，也未发出任何运动指令。待 SSH 恢复后只需重新同步语音两个节点、重建并验收 `/voice/command`、`/voice/command_result` 和 `/chassis/cmd_vel`。

- 2026-09-28 16:27 新录制核对：Astra Viewer 会话先保持 `Depth.Registration=0`，约 11.67 秒成功切换为 `1`，之后至关闭保持开启；因关闭过快未触发自动归档，已从 `Tools/Captured.oni` 补归档并保留原始文件。

## 2026-09-28 网口语音部署完成

- 网口 SSH 恢复后，将接口路径和启动脚本修复同步到 `/home/wheeltec/ROSCAR-current`；远端清理旧 build/install 后重建语音接口、路由和 DeepSeek 包成功。
- 修复两处部署问题：语音脚本改用项目根 `install/`（不再误用 `ros2_ws/install`），并显式设置当前 `roscar_interfaces` Python/C 库路径；总入口同步改用项目根 install。
- 最终在线核对：`/motion_guard`、`/person_follower`、`/voice_command_router`、`/deepseek_chat`、ASR、TTS 和 `wheeltec_robot` 均运行；`/cmd_vel` 1 个发布者、1 个底盘订阅者；`/voice/command` 与 `/voice/command_result` 各 1 发布者/1 订阅者；`radar_required=false`。
- `/scan` 仅有 motion_guard 订阅者、无发布者；未启动 lslidar/radar 进程。未发送语音运动命令，未执行实车动作。

## 2026-09-28 Foxglove 语音状态面板补齐

- 网口核对确认 Jetson Foxglove Bridge 仍监听 `0.0.0.0:8765`，客户端已建立连接并订阅语音相关话题；桥接进程未失效。
- 当前语音失败点在唤醒后的 ASR 音频发送：日志出现 `WebSocketTimeoutException during SENDING_AUDIO`，随后本地 VAD 报未检测到超过能量阈值的语音；这不是 Foxglove 显示层故障。
- 为 `foxglove/b-radar-layout.json` 增加 `/voice_words`、`/voice/chat_state`、`/voice/command_result` Raw Messages 配置，并同步到 `/home/wheeltec/ROSCAR-current`。未启动雷达，未发送底盘运动指令。
- JSON 校验通过；尚未重新加载用户当前 Foxglove 窗口布局。

- 2026-09-28 16:53 新录制检查：最新归档 `astra-viewer-auto-20260928-165254/Captured.oni`，849,448,902 bytes，约 112 秒；会话约 1.46 秒处为 `Depth.Registration=0`，5.20 秒成功切换为 `1`，之后至关闭未再切回 0；RGB/Depth 长段约 29–30 FPS。该段后半程可用于配准 RGB-D 跟随优化，开头约 5 秒需单独标记为未配准。

- 2026-09-28 跟随锁定连续性优化首轮：YOLO 选择器新增默认 0.35 秒短时锁定保持窗口，选中轨迹短暂漏检时保留身份但没有当前深度位置，控制层仍因 `position_valid=false` 保持停车；轨迹恢复可继续使用同一锁定。新增 `track_hold_s` launch 参数，完善丢失状态测试。Python 编译及 yolo_person_tracker 18 项逻辑测试通过；未接入车辆、未宣称录制回放已完成识别率验收。

- 2026-09-28 跟随连续性优化第二轮：新增按 ByteTrack ID 的 XYZ 时间滤波（默认 alpha=0.35），大于 0.8 m 的跳变重置；深度无效不延续为有效位置。新增 `depth_smoothing_alpha`、`depth_jump_reset_m` launch 参数。YOLO 包 Python 编译及 20 项逻辑测试通过，未实机部署。

- 2026-09-28 跟随连续性优化第三轮：新增短窗口轨迹换 ID 重捕获，要求新框与上一目标中心距离（默认不超过 max(30 px, 25% 图像宽度)）及面积比 0.35–2.8 合理；远处或尺寸不符目标不继承锁定。新增 `reacquire_s`、`reacquire_center_fraction` 参数。22 项 YOLO/深度/状态逻辑测试通过，未实机部署。

## 2026-09-28 本版本取消雷达控制门禁

- 按用户要求，将语音交接文档、ROS 接口文档和 `motion_guard` 说明明确为：本版本不启动雷达，`radar_required` 固定为 `false`，模式切换、arm 和故障判定不等待 `/scan`、雷达 TF 或雷达净空结果。
- 保留模式、请求/目标时效、重复发布者、底盘自身停车和人工急停要求；未修改雷达驱动代码，也未启动雷达。
- 文档改动通过 `git diff --check` 审查。
## 2026-09-28：V5.1 语音控制移除蜂鸣器入口

- 在 V5 (`8510805`) 上定点修改，未应用基于旧版本的 stash：DeepSeek 不再向模型提供 `buzz`，结构化调用白名单也拒绝 `buzz`；语音路由不再订阅旧 `/voice/tool_call` 蜂鸣器 JSON 或接受 `BUZZ`，也不创建蜂鸣器发布者。
- 语音启动文件与配置删除蜂鸣器参数及 GPIO 节点，`run_voice_assistant.sh` 不再传 `enable_buzzer`。V5 的中文命令、本地规则、结构化运动/状态工具、讯飞 ASR/TTS 和唤醒应答保持原链路。
- 保留独立 `buzzer_gpio_node.py`、`parse_buzz_command` 及对应测试，便于以后单独评估硬件；更新项目和语音文档。当前版本标记为 V5.1。
- 本机结构检查通过（13 包）；语音相关纯逻辑测试 `21 passed, 2 skipped`，新增的两项路由行为测试因当前 WSL 缺少 ROS 2 `rclpy` 而跳过。Python 编译、语音 YAML 解析、Bash 语法及定点 `git diff --check` 均通过；未在 Jetson 部署或实车执行。
