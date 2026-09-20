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
