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

## 2026-09-28 PR 复审修复：跟踪保鲜与连续语音超时

- 修复 `yolo_person_tracker` 的短时深度持有保鲜语义：为每个 track 额外记录真实测量时间戳，持有位置时把 `TargetState.observation_stamp` 回填到原测量时刻，避免“新发布头 + 旧位置”绕过下游 `target_timeout_s` 判定。
- 修复 `voice_command_router` 连续会话下的运动超时行为：`EXTERNAL` 模式超时后始终走 `_stop_motion(..., set_idle=True)`，撤销授权并退出手动模式；持续唤醒会话仍保留，仅取消“超时后继续保留授权”的不安全分支。
- 最小审查：本机为测试补齐 `pytest`/`numpy` 后，`yolo_person_tracker` 23 项与 `voice_command_router` 14 项测试全部通过；`parallel_validation` 的 Code Review/CodeQL 均无新增告警。

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

## 2026-09-28：V5.1 语音控制移除蜂鸣器入口

- 在 V5 (`8510805`) 上定点修改，未应用基于旧版本的 stash：DeepSeek 不再向模型提供 `buzz`，结构化调用白名单也拒绝 `buzz`；语音路由不再订阅旧 `/voice/tool_call` 蜂鸣器 JSON 或接受 `BUZZ`，也不创建蜂鸣器发布者。
- 语音启动文件与配置删除蜂鸣器参数及 GPIO 节点，`run_voice_assistant.sh` 不再传 `enable_buzzer`。V5 的中文命令、本地规则、结构化运动/状态工具、讯飞 ASR/TTS 和唤醒应答保持原链路。
- 保留独立 `buzzer_gpio_node.py`、`parse_buzz_command` 及对应测试，便于以后单独评估硬件；更新项目和语音文档。当前版本标记为 V5.1。
- 本机结构检查通过（13 包）；语音相关纯逻辑测试 `21 passed, 2 skipped`，新增的两项路由行为测试因当前 WSL 缺少 ROS 2 `rclpy` 而跳过。Python 编译、语音 YAML 解析、Bash 语法及定点 `git diff --check` 均通过；未在 Jetson 部署或实车执行。

## 2026-09-28：离线语音独立分支和隔离实机验证

- 从 V5.1 `c84d838` 建立 `codex/offline-voice-v5.2`，工作树在 WSL `/home/ubuntu/works/ROS-CAR-offline-voice`；原 `/mnt/e/works/newcar/ROS-CAR` 工作树中的换行差异和 stash 未动。
- 新增 `offline_voice`：sherpa-onnx 流式 Paraformer INT8 ASR、MeloTTS、本机 Ollama/Qwen3 1.7B。沿用 V5.1 的本地命令路由和受限 `VoiceCommand` 工具；离线入口不读取讯飞/DeepSeek 密钥，HTTP 客户端只允许本机地址且禁用代理。`VOICE_BACKEND=online` 保留原入口作为回退。
- Jetson `/home/wheeltec/ROSCAR-offline` 隔离安装 sherpa-onnx 1.13.8、Ollama 0.34.0 JetPack 6、本地 Qwen3 模型及 ASR/TTS 模型。官方 Ollama 归档哈希已核对；模型和权重未加入 Git。端侧 ASR 样例识别为“昨天是 monday today is 零八二 the day after tomorrow 是星期三”；TTS “我在”生成 44.1 kHz、23552 样本；Qwen3 回答和 `query_status` 工具调用成功，热态工具调用约 0.75 秒。
- 发现车上现有 `/home/wheeltec/ROSCAR-current` 安装层比仓库 V5.1 旧，仍有 `buzz`。隔离 overlay 因此从本分支重建 `xfyun_speech`、`deepseek_ros2`、`voice_command_router`、`offline_voice` 四包成功；新安装的聊天与路由节点代码不含蜂鸣器调用。
- 在 ROS 域 183、`enable_wake_driver=false`、`voice_control_enabled=false` 下四个离线节点启动成功；`/voice/unhandled_text` 经本机 Qwen3 返回 `/voice/assistant_text`，TTS 节点无报错；手动 ASR 空唤醒正确忽略。单独发送“查询小车状态”时 Qwen3 发出结构化 `QUERY_STATUS`，禁用的路由返回“语音控制已关闭”，未产生车体动作。未验证真人发音的识别率和扬声器主观音质。
- 在线整车栈同时运行且 Qwen3 GPU 加载时，Jetson RAM 约 5.4 GiB 已用、1.7 GiB 可用、swap 约 101 MiB；停止隔离语音后约 3.4 GiB 可用。离线启动脚本的独立 Ollama 与 ROS 进程组经停止测试均退出，原 `/home/wheeltec/ROSCAR-current` 的在线语音、相机、感知和底盘进程仍运行，正式部署未切换。
- 本机 `offline_voice` 的 2 个 HTTP 客户端单测、14 包结构检查、Python 编译、Bash 语法和 `git diff --check` 通过；隔离 overlay 的 4 包 Jetson Humble 构建通过。根据用户要求，后续远端操作改为 WSL 中的 Paramiko 密钥连接。
- 代码已在 WSL 专用分支本地提交；尝试用 WSL Git 推送到贡献者 fork 时因 WSL 未配置 GitHub HTTPS 凭据而停止，尚未推送远端。车上正式部署和原仓库工作树未改。

- 2026-09-28 离线 ONI 评估入口：新增 `scripts/analyze_oni.cpp` / `scripts/analyze_oni.sh`，使用 Astra OpenNI 2.3 回放读取 RGB/Depth，输出帧数、时间跨度、分辨率和有效深度比例。最新 16:53 录制前 300 帧约 10.107 秒，640x480 RGB/Depth，深度 0.2–8m 有效率约 54.4%；短样本读取成功。完整 112 秒扫描在本机旧 x86 OpenNI 回放库上超过单次检查窗口，未宣称已完成全片逐帧识别率统计。

## 2026-09-28 语音改为持续控制会话

- `voice_command_router` 新增持续会话状态：首次 `/voice_words: 小车唤醒` 开启会话，ASR 每轮完成后自动调用 `/voice/start_listening` 进入下一轮；再次唤醒事件退出会话并执行停止/回到 IDLE。
- 持续会话不取消速度请求超时和底盘停车保护；无新运动指令时速度归零，只有会话保持语音监听授权。
- 已更新语音交接文档和路由器 README；本机 Python 语法检查通过，尚未完成 Jetson 编译和现场连续语音验收。

## 2026-09-28 修复持续语音监听衔接

- 发现持续会话中 ASR 发布文本时仍处于 busy 状态，路由器立即调用 `/voice/start_listening` 会被 ASR 拒绝，造成下一轮识别间歇失效。
- 路由器现将下一轮监听延迟约 350 ms，并由定时器在服务可用时发起，避免与 ASR 清理上一轮请求竞争。
- 本机语法检查通过；需在 Jetson 重编译并现场连续说多条命令验证。

- 2026-09-28 跟随连续性优化第四轮：深度测量加入人体候选区域小孔局部中值修复、多部位深度簇投票，并将默认人体区域有效阈值降至 8%；锁定保持默认 0.8 秒、换 ID 重捕获默认 1.2 秒。保留大跳变重置和无效深度停车语义。22 项逻辑测试及 Python 编译通过，未部署实车。

## 2026-09-28 放宽遥控模式语音同义词

- 现场 ASR 将“开始遥控”识别为“遥控模式”，原本地解析器将其判为不明确指令。
- 新增“遥控模式”“进入遥控”到 `EXTERNAL` 授权命令，并同步更新语音文档。

- 2026-09-28 有限位置保持与录制评估：新增 `position_hold_s` 默认 0.25 秒；深度短暂无效时输出带 `measurement_age_s` 的短时保持位置，超过窗口自动失效，控制 freshness 检查仍可停车。最新 16:53 ONI 以每 5 帧抽样（680 帧、约 113.3 秒）运行当前 YOLO26s + ByteTrack，检测到人 678/680 帧，单人 598 帧，多人 80 帧，零人 2 帧；按当前单人连续 3 帧确认的保守规则锁定约 92.3 秒（81.5%）。该统计未包含真实深度区域有效性，也未替代完整 ROS 节点实机验收。

## 2026-09-28 增加“开启遥控”语音别名

- Foxglove 实测 ASR 输出为“开启遥控”，解析器原先未收录，现加入 `EXTERNAL` 授权别名；同时加入“遥控”。

## 2026-09-28 修复首轮 ASR 失败后的持续监听恢复

- 持续会话首轮 ASR 若因能量阈值或网络错误失败，不会发布 `/voice/asr_text`，原路由器无法安排下一轮监听。
- 路由器现订阅 `/voice/asr_state`，持续会话中检测到 ASR 回到 `IDLE` 即重新排队监听，识别失败后也能恢复。
- 本机语法检查通过，待 Jetson 重编译和连续会话验收。

## 2026-09-28 增加持续监听保活

- 现场持续会话只有开启日志、没有后续 ASR 事件；在状态回调之外增加约 1 秒低频监听保活。
- ASR 忙时服务请求会被安全拒绝，空闲时自动接收下一轮；不改变速度、授权和停车保护。

## 2026-09-28 当前跟随逻辑部署

- 将人体跟随优化涉及的 `yolo_person_tracker`（检测框短暂保持、ID 近邻重捕获、深度多区域投票/小孔洞修复/平滑、0.25 秒有界位置保持）和 `perception_bringup` 启动参数部署到 Jetson `/home/wheeltec/ROSCAR-current`；远端源码与本地 SHA-256 一致。
- 远端 `yolo_person_tracker` 与 `perception_bringup` 原生 Humble 构建成功；远端跟踪器单元测试 22 项全部通过。在线 B 路线已由现有 supervisor 重新拉起，读回 `position_hold_s=0.25`、`depth_smoothing_alpha=0.35`、`depth_min_fraction=0.08`；`/cmd_vel` 抽样为零速，本轮未执行运动测试。
- 远端另有 `/home/wheeltec/ROSCAR` 开发副本同步并完成依赖构建；实际运行副本为 `/home/wheeltec/ROSCAR-current`。清理了错误复制到 `ros2_ws/src/` 顶层的临时 Python 文件。最小审查：`git diff --check` 通过。



## 2026-09-29 YOLO 三维卡尔曼与 TensorRT

- 用户确认仅修改 YOLO 三维位置滤波与 TensorRT；随后授权上车测试，要求与正在修改语音的朋友隔离。未修改语音源码或配置。板端代码验证使用 `/home/wheeltec/roscar-yolo-validation-20260929`，不覆盖正式源码，不再重启整套服务。
- `DepthTrackFilter` 替换为 XYZ+速度六维恒速卡尔曼，按实际观测间隔预测、Joseph 协方差更新；深度统计/异常值处理和 ByteTrack 框滤波保留。支持缺测有界预测、跳变/时钟倒退/长间断重置、过期轨迹清理。初始测量标准差 0.08m、加速度标准差 2m/s² 尚未实车标定。
- 修正发布测量年龄覆盖问题：计入观测到发布的延迟以及预测距离上次真实测量的间隔；发布时再次检查预测窗口，不让定时器刷新旧测量有效期。
- 本地 B 管理入口默认 `yolo26s-fp16.engine`，缺引擎报错；显式 MODEL_PATH 可选 PyTorch 基线。导出器增加跨进程锁和 4GiB 剩余磁盘检查。新增只读采帧、同帧后端基准和独立命名空间真实 RGB-D 验证脚本。
- 已检查 Jetson 约 98GiB 可用空间；复用 CUDA torch 2.6.0-rc1、TensorRT 10.3.0、Ultralytics 8.4.156，仅在 `.venv-yolo` 补 ONNX 1.17.0/protobuf 3.20.3。pip check 同时报告已有系统可见依赖不一致（系统 OpenCV 无 opencv-python 分发记录、jupyter/anyio、pipx/argcomplete），未为此替换系统环境或语音依赖。
- 板端开始验证前调用 `/control/disarm`；发现终止总入口会被 systemd 自动重启后，再次 disarm，并设 `/person_follower enabled=false`。用户后续限定只改本模块，本轮不再操作总服务。没有发送运动指令，也不自动恢复跟随。
- 引擎初次构建时网络安装完成与重试重叠，短暂出现两个构建进程；已终止重复进程，并给导出器补互斥锁。初次构建日志含内存不足跳过 tactic，最终性能必须以实际加载/同帧测试为准，不根据 FP16 标签宣称加速。
- 本机 Linux ARM64 Humble 8 个相关包构建通过；26 项逻辑测试、真实 ROS 合成 RGB-D（含预测总年龄与超时失效）及并发测试通过，日志 `artifacts/kalman-20260929.log`。Jetson 隔离目录两包原生构建与同组测试通过；并发测试退出时出现一条 rclpy Destroyable 清理警告，进程退出码为 0，断言通过，不据此宣称无任何运行告警。

- 2026-09-29 上车 B 隔离验证：在 `/home/wheeltec/roscar-yolo-validation-20260929` 使用独立 namespace `/validation_yolo_20260929`，只启动真实 YOLO RGB-D 感知，不启动跟随/底盘，不修改语音。TensorRT FP16 engine 成功加载，60 秒无推理错误；收到 568 条检测消息，参数读回 `kalman_measurement_std_m=0.08`、`kalman_acceleration_std_mps2=2.0`。现场窗口无人，状态为 SEARCHING/NOT_READY/STALE，未产生有效人体坐标，不能视为真人跟踪或卡尔曼坐标验收。结果保留在远端隔离目录 `artifacts/live-kalman.json`。
- 同一批 30 帧真实彩色图的后端对比：PyTorch 平均 35.508 ms、P95 40.058 ms；TensorRT 平均 25.653 ms、P95 28.273 ms；平均约 1.384 倍，检测数量一致率 1.0。画面无人，因此仅是推理/链路性能对比，不是人体识别准确率验收。结果保留在远端隔离目录 `artifacts/backend-comparison.json`。
- 首次 TensorRT 构建生成约 23 MB engine，Jetson 日志显示引擎生成约 848 秒、加载成功；构建期间出现低可用内存跳过 tactic 的警告，但最终引擎通过加载和空图 smoke test。正式运行仍未切换到该 engine；需后续有人在画面前时重复独立验证，并再评估内存与长期稳定性。
- 2026-09-29 真人 TensorRT+卡尔曼只读验收：用户现场对准人体后，在隔离目录与 `/validation_yolo_20260929` namespace 运行 45 秒，未启动跟随/底盘，语音不变。收到 152 条检测消息，状态 TRACKING 414 次，产生 414 条有效 XYZ；Z 范围 0.854–0.891m，中位数约 0.882m。`measurement_age_s` 最大约 0.494s，表明卡尔曼短时预测未刷新真实测量年龄；节点 error 为空。结果 `/home/wheeltec/roscar-yolo-validation-20260929/artifacts/live-kalman-person.json`。TensorRT 加载有跨设备 plan 通用性警告，当前 Jetson 上加载和推理成功；正式部署应继续使用本机生成的 engine。

## 2026-09-28：正式 ROS 域切换为离线语音

- 用户授权上机替换当前 DeepSeek 语音流程。先在 Jetson `/home/wheeltec/ROSCAR-backups/voice-online-20260928-214514.tar.gz` 备份在线脚本、三个在线包源码和安装结果，再通过 Paramiko 同步本分支的 `offline_voice`、V5.1 路由/语音包及启动脚本到 `/home/wheeltec/ROSCAR-current`。
- Jetson 正式工作区 `deepseek_ros2`、`voice_command_router`、`xfyun_speech`、`offline_voice` 四包构建成功；安装层检查确认聊天和路由实现不包含 `buzz`。项目总入口按原有 ROS 域 182、相机、跟随、底盘参数重启，语音后端改为 `VOICE_BACKEND=offline`，没有修改相机、跟随或底盘代码。
- 当前正式语音节点为 `offline_asr`、`offline_chat`、`offline_tts`、`voice_command_router` 和唤醒串口；原 `deepseek_chat`、`xfyun_asr`、`xfyun_tts` 已停止。实测文本“请查询小车当前状态，只返回状态，不要运动”返回结构化 `QUERY_STATUS`，结果为 FOLLOW/STANDBY、11.32 V、无跟随目标，未发送运动命令；TTS 话题“离线语音测试”发布成功，节点无错误。
- 切换后整车观察到约 1.8 GiB 可用内存、swap 使用约 49 MiB，Qwen3 1.7B 显示 100% GPU；未出现 OOM。真人说话识别、实际扬声器主观音质和运动口令仍需现场验收。若需回退，停止当前总入口并恢复备份，或将 `VOICE_BACKEND=online` 后按原参数重启。


## 2026-09-29：离线中文 ASR 更新

- 默认 ASR 切换为 sherpa-onnx SenseVoice Small INT8，固定中文、CPU 两线程；保留 Paraformer 后端和旧模型。
- 修复等待收音时限截断起音的问题，加入 400 ms 句首缓冲、80 ms 起音和 30 秒等待；TTS 打断时不发布残缺识别。新增实际收音方法的三个回归测试，全部通过。
- Jetson 已下载官方模型，`offline_voice` 原生构建成功；备份位于 `/home/wheeltec/ROSCAR-backups/asr-20260929-124403`，先热替换 ASR。之后 12:45:35 服务记录模块退出（143）并于 12:45:41 自动重启整套，临时 ASR 管理程序退出；当前由正常 launch 管理。启动日志确认 `sensevoice`，进程检查确认仅一个 ASR。未发送控制指令；未进行现场真人准确率验收。
- 同一 5.592 秒样例：SenseVoice 解码 0.49 秒/峰值 356 MiB，Paraformer 0.87 秒/343 MiB。切换后全车可用内存约 856 MiB，swap 使用约 759 MiB；该快照不能证明长期负载稳定，需观察现场运行。
- 回退步骤及临时进程生命周期已补充至离线语音部署文档。未提交或推送 Git。


## 2026-09-29：按用户要求恢复讯飞 ASR

- 默认混合方案为讯飞 ASR + 本地 Qwen + 本地 MeloTTS；保留完全离线开关 `ASR_BACKEND=offline`。未改变语音动作解析或运动参数。
- 复用车上私有讯飞凭据；仅检查是否配置，不输出密钥。真实 WebSocket 静音请求返回 code=0、status=2，鉴权和接口通信成功；未发布识别结果至 ROS，未发送运动指令。
- offline_voice 车上构建成功，脚本语法检查通过。旧方案备份至 `/home/wheeltec/ROSCAR-backups/xfyun-asr-20260929-130232/before.tar.gz`。旧后台 launch 未响应 SIGINT，随后向语音脚本发送 SIGTERM，触发现有清理与守护重启流程加载新方案。现场真人识别效果仍需验收。
## 2026-09-29：语音控制改为固定规则

- 按语音交接文档保留唤醒持续会话、遥控授权、跟随、停止、四方向驾驶、状态查询和导航未接入提示；识别不到固定指令时由路由器回复固定提示，不发运动命令。
- 离线语音启动文件不再启动 `offline_chat`；语音脚本不再启动或检查 Ollama；项目总入口不再要求 Ollama 文件。讯飞 ASR 与本地 TTS 保留，历史聊天源码保留供回退。
- 本地 `voice_command_router` 规则测试 14 项通过；修正其中一项沿用 0.15 m/s 旧上限的断言，使其与当前 0.2 m/s 上限一致。Shell 语法和 Python 编译检查通过。
- 车上修改前备份为 `/home/wheeltec/ROSCAR-backups/fixed-voice-20260929-135822/before.tar.gz`；Jetson 上 `voice_command_router`、`offline_voice` 两包构建成功。服务正常重启后，进程与日志显示讯飞 ASR、路由器、本地 TTS，未启动聊天节点或项目隔离的 Ollama。未发送运动指令；真人语音仍需现场验收。
## 2026-09-29：端侧 ASR 与 CUDA 可行性

- 整车运行时可用内存约 3.4 GiB、swap 未使用；隔离下载并测试 Qwen3-ASR 0.6B INT8、FireRedASR2 CTC INT8。模型比较与限制详见 `docs/端侧ASR候选实测.md`。
- 五条仓库播报样例中，FireRedASR2 与 Qwen3-ASR 均识别到方向或停止，SenseVoice 在停止样例误识别。FireRedASR2 CPU 峰值 RSS 约 929 MiB、解码 1.07–1.30 秒；Qwen3-ASR 约 1489 MiB、1.64–2.21 秒。样本不代表真人准确率。
- 将 FireRedASR2 接为可选离线后端；车上备份 `/home/wheeltec/ROSCAR-backups/fire-red-asr-20260929-144623/before.tar.gz`，原生构建成功，独立 ROS 域启动确认加载。正式讯飞方案未切换，未发送运动命令。
- 车上安装独立 CUDA 评测 wheel 后，实际加载报缺少 `libcublas.so.10`；当前 JetPack 6.2 为 CUDA 12.6，需另按官方文档构建兼容版本才能测 GPU 加速。正式 Python 环境未替换。

## 2026-09-29：正式语音 ASR 切换为 Qwen3-ASR

- 按用户要求将 `offline_voice` 增加 Qwen3-ASR 0.6B INT8 本地后端，并将配置及语音脚本默认入口改为离线 Qwen3；固定规则控制和本地 TTS 保持运行，讯飞 ASR 仍可通过 `ASR_BACKEND=xfyun` 回退。
- 模型移入车上 `/home/wheeltec/ROSCAR-offline/models/sherpa-onnx-qwen3-asr-0.6B-int8-2026-03-25`；改动前备份为 `/home/wheeltec/ROSCAR-backups/qwen3-asr-20260929-145122/before.tar.gz`。正式工作区 `offline_voice` 原生 Humble 构建成功，独立 ROS 域启动确认模型加载，正式守护服务重启后日志再次确认 `qwen3` 且无讯飞 ASR 进程。
- 本地 Python 编译、ASR 收音回归测试 3 项、脚本语法和差异检查通过。正式切换后快照：7.4 GiB 内存中约 2.3 GiB 可用，swap 使用约 177 MiB，项目服务运行中。模型使用 CPU 两线程；真人准确率、持续运行和实际车辆动作留给现场测试。本轮未发布运动指令，也未提交或推送 Git。

## 2026-09-29：连续收音与简化语音控制

- 路由器改为单一会话管理者：重复硬件唤醒只刷新 30 秒会话；首次明确方向口令自动请求 `EXTERNAL` 与 `/control/arm`，必须等新鲜 `/control/state` 确认 `ARMED` 后执行同一条命令。“停止”停车但保留会话，“急停/退出控制”停车并结束会话。方向成功不逐条播报，常见短口令同义词增加；否定、疑问与含糊指令继续拒绝。
- 新 `continuous_asr_node` 保持麦克风采集，Silero VAD 分句（结束静音 0.7 秒），Qwen3-ASR 在独立线程解码；最多排队两句，结果带会话代次和 6 秒时效，TTS 播放与回声冷却期间暂停分句。旧 `asr_node` 和讯飞入口保留；`CONTINUOUS_ASR=false` 可回退旧逐轮收音。官方 Silero 模型 643854 字节，SHA-256 `9e2449e1087496d8d4caba907f23e0bd3f78d91fa552479bb9c23ac09cbb1fd6`。
- 本地 29 项相关测试通过；Python、Shell 语法与差异检查通过。Jetson 隔离工作区两包原生 Humble 构建成功，ROS 域 184 使用模拟 `arecord` 连续重放两句真实播报，每句均完整识别，解码约 1.8–2.0 秒；ROS 域 185 使用假的 guard 服务验证自动授权、重复唤醒、停止后重授权、急停、会话外拒绝与 `ARMED` 未确认时不发非零速度。
- 正式项目备份为 `/home/wheeltec/ROSCAR-backups/continuous-voice-20260929-192944/before.tar.gz`，SHA-256 `460409fba541dd5b05e4af8633080dfd27c72089a4086b221f3b93c0fdcaeb34`。源码同步后在正式根目录构建两包并由 systemd 重启加载；核实正式节点为 `continuous_asr_node`，参数 `vad_silence_s=0.7`、`session_timeout_s=30`、`legacy_listen_polling=false`，服务 active，`/voice/session_active=false`、`/cmd_vel` 零速。一次误用 `ros2_ws/install` 构建未被正式入口读取，发现后在根目录 `install` 正确重建。隔离测试残留的两个进程组已清理，可用内存恢复；正式切换后快照约 2.8 GiB 可用、733 MiB swap 已用。
- 本轮未发布实际运动指令；真人麦克风、嘈杂环境、实体底盘运动与长期稳定性仍待用户现场验收。
- 车载网络恢复后补传旧 ASR 唤醒归属和“识别中刷新会话时限”修正，在正式根目录重新构建 `voice_command_router`、`offline_voice`；隔离 ROS 域 185 假 guard 测试新增 0.5 秒会话超时场景并通过。重启正式守护服务后读回 `continuous_asr_node` 已加载、会话未激活、`/cmd_vel` 为零，约 2.4 GiB 内存可用、19 MiB swap 已用。尚未进行真人语音与实车动作验收。

## 2026-09-29：无意义 ASR 结果静默忽略

- 用户现场日志出现“真。”“嗯。”等无关识别，路由器之前会播报整段可用指令提示。现对未匹配固定指令或无法解析的含糊口令只记录日志，不发 TTS，继续监听；有效动作被运动门禁拒绝时仍播报失败原因。
- 本地 29 项相关测试、Python 编译与差异检查通过；车上隔离 ROS 域 185 假 guard 测试确认“嗯。”“真。”“前任”不产生 TTS，随后“前进”仍能自动授权，原有停止、急停和授权未确认安全路径通过。
- 改动前备份 `/home/wheeltec/ROSCAR-backups/silent-unknown-20260929-222043/before.tar.gz`，SHA-256 `52768f17c09e5c68d213a74c67347ccf2b5bb84f049c4d921fc16a63556caa63`。正式 `voice_command_router` 原生 Humble 构建成功；重启服务后读回新版源码与安装包哈希一致、连续 ASR 已加载、会话未激活、`/cmd_vel` 零速。未执行真人语音和实际底盘运动测试。


## 2026-10-01：本机新增 C Gemini/Pose 路线

- 用户确认 C 继承 B，其检测模型替换为 YOLO26s-pose；画面内所有跟踪人体输出站立、坐蹲、躺卧、跌倒、未知。本阶段只检测和显示，不接语音/运动；真人验收按用户回复延后，本轮不部署或切换小车。默认入口仍 B，C 显式 `route:=yolo_pose` / `scripts/start_c.sh`。
- Mac USB 与现有 Orbbec SDK v1.10.16 实测设备内部名 SV1301S_U3、序列号 AY2755200PW、固件 RD3013，深度 2bc5:0614 / UVC 2bc5:0511。10 秒 300 张原生 640×400 深度帧；保存样本有效深度约 51.6–52.2%。SDK libuvc 彩色打开失败 -3，改由 AVFoundation USB Camera 获得真实 640×480 JPEG；两路分别采样，不宣称同步 RGB-D 或物理配准已通过。SDK 不支持新版显式硬件同步。
- 深度单开时 Pipeline 内参为空；从设备 getCalibrationCameraParamList 第0项取得实际 RGB 640×480 / 深度640×400内参、畸变、深度到彩色外参，保存到序列号专用 JSON。显式软件校正/投影/z-buffer 保留原输入时间，严格核对序列号、尺寸、frame、编码、时间和标定；不继承 Astra 临时内参。Gemini 安装外参独立且未确认，DEPTH_REGISTERED 默认 false。
- 官方 Pose 权重 24,151,790 字节与官方 release digest 一致，SHA-256 a083adb42303728ae14c4bd6bd56d80da46f82fb2564dbd6f31dcc92ea321646；真实 Mac CPU 官方 bus.jpg 4 人各17点，重复静态帧 ID 延续，Gemini 实拍无人。不是现场人体/跌倒验收或 Jetson 性能验证；FP16 Pose 导出只准备入口，未在 Jetson 执行。
- 新增 PersonState/PersonStateArray、骨架 MarkerArray、逐人有界时序规则与性能统计，延续 ByteTrack 原检测索引、epoch ID、B 目标/选人/滤波/TF 契约。关节只用当帧真实邻域深度，拒绝洞/背景/跳变；静态躺卧不补报跌倒，安装未确认最多疑似；断流/遮挡/缺点中断连续计时。
- 官方 legacy Gemini ROS2 驱动固定 f7e71d9ce806e788cb48d8580aac2c778fba4214，独立下载源码在 Linux ARM64 Humble 编译 2 包成功（约71秒）；补齐 upstream 未声明的 nlohmann-json 与相关依赖，并为已安装 Ubuntu libuvc/glog 生成缺失 pkg-config 元数据。原驱动源码未改。证据 artifacts/gemini-driver-build.log；没有驱动实机运行证据。
- 最小审查修正骨架叠加污染原始预览、DELETE frame、ROS固定数组浮点类型、空/失效人体列表、缺配置错误及 C overlay 路径。回归发现 B measurement_age 从 NaN 起算，已按观测时间与真实测量年龄修复；两项既有测试限值假设与当前0.2m/s默认不一致，测试改读配置/固定PTY夹具限值，未改运动默认值；并发测试先确认状态订阅发现再计响应，保留原并发/过期要求。
- 本机 Python 逻辑37项、C入口3项通过；结构检查、ShellCheck、git diff --check通过。ROS最终回归结果在本条后补充。资料见 docs/方案C实现与验收.md；真实证据 artifacts/gemini-input-report.json、gemini-probe/、gemini-color.jpg、c-model-smoke.json。
- 最终 C 专项 `bash scripts/test_c_container.sh` 退出码0：14主动包编译（约11秒）、37项 B/C逻辑、C骨架/3D/失效/epoch/锁定/性能、真实ROS配准节点的单位/时间/尺寸/frame拒绝、B同步RGB-D/TF/并发、A适配器、35项语音测试、A/B/C/red/demo/非法route及对外API C默认IDLE/零速/缺模型拒绝全部通过。日志 artifacts/c-acceptance.log。
- 完整旧 `scripts/test_container.sh` 未取得单次全绿：旧API/guard测试间歇触发 request_timestamp 看门狗，旧并发测试曾受话题发现/完成时序影响。没有放宽产品时效或运动门禁；记录 artifacts/c-regression-watchdog-failure.log、c-regression-guard-timing-failure.log、c-acceptance-legacy-api-timing-failure.log。此前分段已观察到3底盘包构建、真实PTY、保护与回放测试通过，但不据此称整个旧套件本轮全通过。C最终专项按独立ROS域验证通过。
- 临时容器均 --rm，复用原有Humble镜像/本机Python/Orbbec SDK；任务产生的可重建相机下载源码和临时探针可执行文件清理，保留源码交付、Pose权重、设备标定和验证证据。实物配准/量距、真人动作、Foxglove客户端及Jetson部署/GPU仍待验收。

## 2026-10-02：记录 C 融合方案并评估论文/开源项目

- 先将讨论方案记录为 `docs/方案C多源人体感知融合设计.md`：覆盖近距离骨架辅助深度、远距离地面/关节几何、轨迹与身份分层、二维/三维状态融合；明确参考点、来源、误差、时间和未实现边界。
- 新增 `docs/方案C文献与开源项目评估.md`，依据 CCF 官方目录核对 TPAMI/TIP A 类期刊，筛选 SPNet、Metric3Dv2、OSNet 期刊扩展、HybrIK-X、MS-AAGCN、EfficientGCN 共 6 篇；KPR(ECCV)、Mono-RPF(ICRA)和 ROS2 接入项目单列，不混算期刊数量。OSNet 区分作者 2021 标注与正式 2022 卷期。
- 只读核查 9 个 GitHub 仓库的 README、提交、许可元数据与接口，读取 SPNet/KPR/Mono-RPF 三个相关源文件。发现 EfficientGCN 官方 GitHub 仅留迁移说明；Mono-RPF 局部源码有 reserve 后索引写入及过程噪声索引的静态风险，未运行验证。快照保留于 `artifacts/c-literature-20261002.json`，文档中保留可追溯链接和固定提交。
- 核对当前 `joints3d` 的人体参考深度前置条件，将其列为联合观测改进点；修正 C 验收文档“躯干深度”表述为实际的五区域测距，同步方案主文档链接。未修改生产算法、入口、接口或 AGENTS.md。
- 本轮为文档/研究工作，不下载权重、不安装环境、不执行外部仓库代码、不访问小车。未创建临时容器/镜像或构建产物；保留文档和约 212 KB 核验快照。最小审查完成：4 份相关文档的本地链接/代码块、两份新增文档空白、9 个固定提交与证据一致性、git diff --check 均通过；仅文档更新，未运行软件构建或算法测试。


## 2026-10-02：C 第一阶段骨架辅助真实深度融合

- 按用户“做吧”推进已说明的第一阶段，仅本机代码、测试和文档；未访问/部署小车、未采集新的真人动作。地面反投影、ReID、三维状态判断留在后续阶段，当前姿态与跌倒仍用二维规则。
- C 新增 `fusion.py`，统一收集当前帧 17 点真实邻域、肩髋缩小多边形及五个区域候选。优先独立且一致的肩髋深度，依次允许躯干区域、膝踝、无重叠时区域兜底；其他检测框像素排除，冲突拒绝，不用孤立手部或重复像素建立骨架深度依据。所有关节保持自己的真实深度，缺失 NaN，不补洞、不借预测。
- C 目标代表点改为框中心射线＋融合轴向深度，避免不同采样部位的像素坐标直接混合；它是虚拟点而非骨盆、质心或地面位置，后续几何定位须另建观测关系。沿用原卡尔曼测量噪声、短时保持和测量年龄，不因相关采样重复增加置信度。每人 detail 增加来源/拒绝原因；消息结构、锁定及 epoch 保持兼容。
- `pose.yaml` 默认 fusion_enabled=true，可关闭后重启对照原 C。B 五区域代码只提取共用采样/选择函数，200 个固定种子场景与提交 fb164b56d22032c74e2fc33090624765f16c3f15 的原函数 XYZ 逐值一致。合成对照证据 artifacts/c-depth-fusion-comparison.json：两肩稀疏深度从无效恢复 2m；肩髋2m/背景4m场景从背景4m改为2m；无深度保持无效。不是实际相机准确度或 Jetson 性能结论。
- 新增 15 项融合逻辑测试（总52项），覆盖稀疏肩/膝深度、背景、各关节独立Z、空洞、孤立手部、重复关键点、深度冲突、低置信度、混合邻域、骨架区域兜底、无骨架回退、多人重叠及非法输入。ROS 增加融合开/关、稀疏恢复、NaN、精确观测时间及保持超时验证。
- 最小审查修正 snapshot 扩展后的解包；ROS 新测试发现原新鲜时间戳经浮点还原会损失纳秒，改为新测量直接保留传感器原时间，只有旧测量保持沿用原转换。首次失败证据 artifacts/c-depth-fusion-stamp-failure.log。另一次旧并发测试等待完成超时，未改产品门限/测试时限，完整复跑通过；失败证据 artifacts/c-depth-fusion-concurrency-failure.log，不宣称已定位该间歇超时根因。
- 最终 `ROSCAR_C_TEST_LOG=.../c-depth-fusion-acceptance.log bash scripts/test_c_container.sh` 退出0：Linux ARM64 Humble 14主动包构建、52项感知逻辑、C开/关融合ROS、Gemini配准契约、B测距/选人/失效、TF、并发、A适配器、35项语音测试、所有感知路由及C对外API默认IDLE/缺模型/零速通过。不是完整旧 test_container.sh 全套或真实设备验收。
- 同步方案主文档、C验收、融合设计和研究实施状态；AGENTS仅新增长期C融合代表点/真实关节约定。复用现有 .venv 和 Humble 镜像，测试容器 --rm，构建输出随容器清理；临时重复控制台日志清理，保留测试/对照/失败证据，无新权重或环境下载。

## 2026-10-03：C 单目地面定位

- 上一轮对话只留下未测试、未接入的 `ground.py` 草稿；本轮核对后重写并补齐。仅本机，未访问/部署小车。
- `ground.py`：脚踝射线（含踝高补偿）→站立/未知姿态的框底→肩髋身高先验三级接地点估计；拒绝近水平射线、背后/超范围交点、图像下边缘截断、过大不确定度；`std_m` 由像素/踝高/身高先验有限差分传播；坐蹲仅脚踝，躺卧/跌倒不输出。
- 新增 `ground_localizer` 节点、PersonGround/PersonGroundArray 消息，随 `route:=yolo_pose` 启动。独立输出 `person_ground_states`、`ground_markers`、`target_state_ground`（source=yolo_ground）；按观测时间查 TF，每人恒速卡尔曼、逐样本噪声、真实测量龄和短时预测；`extrinsics_calibrated`/`ground_plane_confirmed` 默认 false，两份 mount yaml 新增 `ground_localizer` 段。不修改 C/B 原目标、不接运动。
- 验证：新增 `test_ground.py` 13 项、`tests/test_ground_runtime.py`（已加入 `scripts/test_c_container.sh`）。C 专项容器回归退出码0：14 主动包构建、65 项逻辑、含新 ROS 测试及 A/B/C/red/demo/路由、公共 API；`route:=yolo_pose` 实际启动三个节点，默认 `target_state_ground` NOT_READY 且无 `/cmd_vel`。日志 `artifacts/c-ground-test.log`。均为合成输入，不是实机精度；git diff --check、ShellCheck 通过。
- 首轮 ROS 测试失败两次均为测试自身问题（保持预测使新轨迹仍有效；墙钟间隔污染合成时间戳），已改测试，未放宽产品门限。
- 未做：现场安装外参/地面标定与已知距离量测；坡道/台阶；RGB-D 与单目来源切换迟滞；近距 RGB-D 个体身高学习；跟随器消费。


## 2026-10-03：独立验收其他 agent 的 C 单目地面定位

- 用户要求验收；本轮仅复核、执行测试和记录结果，不修改生产算法，不部署小车。新增 `docs/方案C单目地面定位独立验收.md`，在C验收文档标记暂不通过。
- 实际复现4项待修：脚遮挡仍用框底接地（真实X=3m估成5.656m）；UNKNOWN坐姿使用站立身高先验（3m估成5.285m）；肩3m/髋5m冲突直接聚合4m且std约0.437m；position_hold_s=0.25s时，真实测量龄约0.351s的预测仍position_valid=true。源码位置、修正方向及复现条件见独立验收文档；`artifacts/c_ground_review_probes.py`、`c-ground-review-probes.json`、`c-ground-review-probes.log`保留证据。
- 本轮完整C专项重跑：14主动包编译、65项逻辑、融合开/关、Gemini配准、B测距和TF通过；既有并发测试test_yolo_concurrency.py第79行assert done超时，整体退出1，记录 `artifacts/c-ground-review-regression.log`。该症状前次已有，不认定由新增地面功能引入，也不宣称本轮整套全绿。
- 后续项目独立补测退出0：地面ROS、A适配器、35项语音、A/B/C/red/demo/非法路由及C公共API通过，日志 `artifacts/c-ground-review-remaining.log`。四项反例使用实际算法及ROS节点执行；不是实物相机验收。当前默认未标定门槛、独立地面参考点、无运动消费的边界合理。
- 最小审查：新增报告与C验收链接、git diff --check通过。复用现有Humble镜像，测试容器均--rm，临时构建随容器删除，清理本轮重复控制台日志；保留复现代码/JSON/日志，不下载环境或权重，不修改AGENTS长期规则。


## 2026-10-03：修正 C 单目定位四项验收问题

- 按用户授权保留ROS接入框架，修改ground估计及失效策略，不访问/部署小车。删除框底接地路径，旧allow_box_bottom_fallback=true报错，默认必须双踝可用且一致；宁可输出无效，也不从被遮挡框底补定位。
- 身高先验默认关闭，增加height_prior_confirmed、height_prior_track_id及standing_stable_s。仅确认的epoch:track_id持续站立才允许，UNKNOWN/坐蹲不使用先验；该先验产生的预测在UNKNOWN时立即清除。断流、时间倒退和轨迹消失重置连续站立依据，epoch改变不会继承个体授权。
- 双脚/肩髋增加两两空间一致性和硬距离门槛，冲突不平均成虚构点，也不借另一来源掩盖；std加入候选离散且注明条件误差，不假称覆盖接地/标定模型错误。躺卧、跌倒及冲突立即清除旧预测。
- 预测生成与定时发布均按当前真实测量年龄检查min(position_hold_s,max_age_s)，保留精确原ROS测量时间戳。补传输延迟、仅定时器运行后的过期反例；姿态错误不再用上一帧位置掩盖。
- 修复前3项几何反例测试失败，证据artifacts/c-ground-fix-before.log；新增6项反例后逻辑总71项通过。原单踝可用的7m场景因现要求双踝且误差含离散被拒绝，距离单调性测试采用2m/6m，保留超远距及严格误差拒绝，并未放宽算法门槛。
- 最终C专项退出0，artifacts/c-ground-fix-final.log：Linux ARM64 Humble 14主动包构建（12.1秒）、71项感知逻辑、C融合两路径、配准、B测距/TF/并发、地面ROS、个体先验绑定/稳定站立/UNKNOWN/epoch测试、A适配器、35项语音及路由/C公共API通过。第一轮完整回归亦通过，日志c-ground-fix-regression.log；不宣称完整旧test_container.sh全套或实物验收。
- 同步两份mount配置、消息注释、C文档/主方案/独立验收修复状态；AGENTS仅补充长期接地/先验/预测契约。最小审查及git diff --check、脚本语法、文档链接检查完成；复用既有环境，无大文件下载，--rm清理测试构建，移除临时重复输出，保留全部验证证据。


## 2026-10-03：C 独立 OSNet ReID 身份观察层

- 用户同意在ByteTrack上方增加持久身份，本轮按已说明范围实现独立观察层。原epoch:track_id保留；新增会话UUID前缀身份、3帧确认、余弦距离/次佳间隔/多人一对一门槛、同轨迹换人核验、初始锚点防图库漂移。失踪保留默认30秒，每人8样本/最多64身份。缺质量和竞争时不输出确认ID，原可见轨迹遮挡时仍占用身份，防止旁人抢占；恢复需重新确认。时间倒退清空，换epoch只能凭外观重新匹配。
- 模型使用作者Torchreid固定提交f8cd150fdf77e8d9e1ed143b7f308c2c609ded50的OSNet-x0.25/MSMT17 ReID权重，不是ImageNet预训练。checkpoint 3,057,863字节，SHA256 6f57607fed9f502b9efed546108132ee715df5a5b6e6932c6269bacb47f59f99；架构原文件/许可证/SOURCE哈希保留在vendor。显式prepare_reid_model.py下载校验/严格加载/导出，运行无自动下载或备用模型。
- 本机已有PyTorch2.6/.venv复用，pip不可用改用已有uv补onnx1.17.0（下载15.9MiB，另安装protobuf依赖），未新建大型环境。启动前本机约18GiB可用。ONNX 891,011字节，SHA256 417218eb180df62da2eb178972118583149c69e52fe3b5d2003fd891e159f1a1；OpenCV DNN CPU与PyTorch归一化特征最大差1.9e-7，证据artifacts/reid-export.log。
- 新reid_observer只消费精确时间/frame对应的原RGB和PersonStateArray；独立单worker、消息队列20、最多8人、默认5Hz，忙时丢弃，过期完成/无效Pose不发布身份。质量门槛包括框尺寸/越界、检测置信度、可见肩髋与点数、躯干非退化、已跟踪框重叠、模糊。当前是骨架检查完整人体裁剪，不是部位训练网络，不利用骨长判断身份，也未加入不可靠的空间门控。
- 新PersonIdentity/PersonIdentityArray及person_identities话题，携带可见/核验状态、持久身份、原轨迹、余弦距离（非概率）、观测年龄与耗时；LOST身份无轨迹或位置。默认关闭时不加载模型、不订阅RGB；C launch和一键入口以REID_ENABLED/REID_MODEL_PATH/REID_CONFIG显式开启，缺模型独立层报错。Foxglove C布局增加身份面板。不改TargetState、lock/release、身高先验或运动。
- 真模型测试使用本机Ultralytics bus.jpg：整图4人被重叠/躯干缺点质量门槛拒绝，未放宽产品阈值。测试记录该限制，另用2个独立真实人体裁剪验证特征提取与ROS；相互距离约0.463，重复裁剪换轨迹能接回，换另一裁剪拒绝继承。这是静态照片/脚本造出的消失重现，不能称真实遮挡或跨视角准确率。Mac单次CPU约12.4/5.4ms；Mac上Linux ARM64 Humble OpenCV4.5.4约6.9/3.5ms，非Jetson性能/稳态FPS。跨平台特征差小于3.1e-7。证据artifacts/reid-smoke/及reid-arm64-model.log。
- 纯逻辑新增22项（身份18＋裁剪4），感知合计93项。ROS合成特征测试覆盖精确帧、缺模型、换轨迹、换人、低质量/模拟/过期拒绝，另阻塞worker验证心跳仍工作且不发布过期完成；真实ONNX ROS测试独立通过。最小审查补强多帧质量低下期间的身份保留与重新确认，避免只保留一帧后被其他人接走。
- 最终完整C专项退出0，artifacts/c-reid-final.log：Linux ARM64 Humble14包构建13.3秒、93项感知逻辑、C融合开/关、Gemini配准、B测距/TF/并发、地面定位/先验、ReID ROS、A适配器、35项语音、A/B/C/red/demo/非法路由及C公共API通过。真实模型专项test_reid_model_container.sh构建4包并运行实际ONNX/ROS通过；不是完整旧test_container.sh全套或现场验收。
- 已同步README、接口、模型清单、C/主方案和docs/方案C身份ReID接入与验收.md；AGENTS只新增长期身份隔离/显式模型入口规则。源码与模型来源哈希、JSON布局、文档链接、ShellCheck和git diff --check通过。未访问/部署小车、未设置自动锁定转移、未做真人/Jetson验收；ReID仍是外观假设，现场误认/漏认和时延待验证。
- 清理本任务onnx下载缓存、临时控制台重复输出和新模块Python缓存；保留可复用的工作环境依赖、权重/ONNX/校验文件及真实/合成验证证据。测试容器均--rm，构建输出随容器移除，未触碰其他业务容器或数据卷。


## 2026-10-03：C 可选 ReID 身份锁定恢复

- 用户在进度说明后授权继续，实施身份闭环阶段；没有扩展到车辆运动、三维姿态或近远定位混合。
- 新增C专用IdentitySelection，持久身份与epoch:track_id分离。仅在显式REID_LOCK_ENABLED/reid_lock_enabled开启时启用，组合launch要求同时开启ReID观察层；默认关闭，B及原C选人兼容。
- 新模式禁用原Selection的邻框自动接续、缺人保持和目标丢失后自动换人。初次绑定、换轨迹和失效恢复默认需3个不同采集时间的核验结果；检查精确已知RGB header、当帧/当前轨迹、一对一映射、采集年龄与重放。最多保留64个帧记录。
- 同ID外观矛盾、歧义、质量不足、断流或过期立即撤销位置授权；失踪不搬旧位置，新轨迹只用自己的深度/滤波。epoch后必须重新确认，释放清除身份且抑制自动锁定，显式重锁重新建立身份。没有转移个体身高先验。
- 最小审查发现ground_localizer原来只读取锁定ID，会绕过身份LOST状态；增加可选身份门控并由C组合launch同步传入，要求新鲜真实yolo TRACKING消息。光学深度无效不阻止已确认身份的地面几何估计；全体人员地面输出不受影响。
- 新增10项身份锁定逻辑测试和ROS锁定恢复测试，扩展地面目标身份门控与组合launch拒绝测试。初版ROS测试使用固定等待，确认帧数随调度抖动导致断流恢复断言失败；改成等待精确Pose帧进入已处理缓存及对应身份消息消费，保留逐次确认日志。针对性ROS锁定/地面测试已通过，证据artifacts/c-identity-lock-focused.log。
- 环境：既有Colima VZ日志明确报virtual machine is no longer live，docker查询挂起；清理已失效VM运行进程并重启原实例，复用roscar-humble-test镜像，没有安装新环境/删除镜像或卷。磁盘检查约19GiB可用。没有连接Jetson。
- 验收：最终Linux ARM64 Humble专项全回归退出0，14包构建13.2秒、103项感知逻辑、35项语音测试及C融合开/关、RGB-D配准契约、B测距/TF/并发、地面/先验、ReID观察层、身份锁定、A适配器、各route和公共API全部通过。证据artifacts/c-identity-lock-final.log；ShellCheck与git diff --check通过。真实多人、相似衣着和跨视角重识别准确率仍待验收；观察层旧模型验证证据沿用，本次未重跑模型导出或宣称实机通过。

- 清理：所有测试容器使用--rm，确认无残留测试容器；删除本轮临时控制台副本与两个新增模块的本机字节码，保留源码、模型和验收日志。原有三个业务容器随原Colima恢复后均healthy，未改配置/数据卷。


## 2026-10-04：C 近远统一定位与三维姿态增强

- 用户明确要求完成下一步1/2，实施软件接入、回归和文档；未连接Jetson、部署或启动车辆。
- 新增unified_localizer与纯逻辑unified.py。统一参考点为双踝中点的地面投影，输出person_positions、target_state_unified（source=yolo_unified）与position_markers；保留原光学框中心、独立单目接口和B行为。当前测量可来自真实双踝、身体深度投影或单目双踝；缺脚不做框底回退，不混入身高先验，不填补三维关节。
- 来源相关，不按独立测量加权平均。加入一致性/平面检查、初始与切换连续3次确认、位移限制、带距离项的条件误差估计、断流/倒退/epoch重建。深度重新出现的确认期仍可使用当帧有效单目；其它确认中无旧位置冒充，统一定位不预测。ReID身份门控传递到统一目标，失效/消失/超时删除Marker并清空列表。
- PersonState新增body_depth_valid/body_depth_m/body_depth_source以传递当帧躯体观测，缺失NaN、排除保持值；需要重建消息包。原TargetState结构与锁定服务兼容。
- 新增pose3d.py，真实双肩双髋/膝深度支持解剖检查、重力躯干角、大腿角、距地高度和米制下降量。重力/地面确认默认关闭；有可靠三维时使用basis=3d，缺失时basis=2d，矛盾几何未知。二维/三维依据切换清空候选时序，避免尺度跳变触发；未利用地面估计合成人体关节。
- 审查发现旧融合把沿光学Z倾斜的合理躯干当成深度冲突，且原跌倒规则不能跨越中间未知倾斜帧。增加完整四点躯干解剖一致性例外，保留各关节自己的测量及明显分离表面拒绝；时序仅在原1秒窗口内保留最近稳定基线。新测试覆盖向镜头方向连续倾倒、静态横卧、缓慢躺下、弯腰、缺点、错误深度、重力旋转和模式/epoch变化。
- 验证：本机124项感知逻辑通过。Linux ARM64 Humble最终14包构建15.0秒，124项逻辑、35项语音及C融合开/关、注册契约、B测距/TF/并发、地面/先验、ReID、身份锁定、统一定位、三维姿态、A适配器、各route、公共API全部通过，整套退出0。证据artifacts/c-unified-pose3d-final.log；逻辑单独证据artifacts/c-unified-pose3d-unit.log。
- 新ROS三维测试使用合成RGB-D深度片段，经实际邻域采样、融合和时序节点输出；统一ROS测试使用合成已知几何。不是相机标定、真人或实车准确率。现场真实多人、误报/漏报/触发延迟、Jetson驱动/TensorRT/性能仍待验收。
- 原ReID回归的固定短等待出现确认计数/等待断言失败；改为等待匹配RGB header的结果且保持采样间隔，独立ReID复测及最终全套通过。保留初轮和针对性日志；未放宽产品身份阈值。
- 文档：新增docs/方案C统一定位与三维姿态.md，同步README、接口、C验收、多源设计、方案主文档和AGENTS长期入口规则；Foxglove增加统一接地点、来源/误差与目标面板。配置保留安装/重力/地面未确认状态。
- 最小审查：确认原光学点不混入接地点滤波、源观测时间未刷新、缺失关节未补值、来源切换计数有界、身份隔离、旧B默认兼容。Python语法/布局JSON引用、ShellCheck与git diff --check通过。构建与回归复用现有镜像；运行前可用空间约21GiB，未创建新大型环境。

- 收尾补查：新增body_depth当前观测/无效NaN/排除预测的ROS断言，C融合开启及关闭两种路径均通过，证据artifacts/c-body-depth-contract.log。所有本轮测试容器已自动删除，构建输出随容器清理；删除两份/tmp控制台重复日志，保留artifacts验收证据及原三个业务容器。

## 2026-10-04：导出 C 整体流程图

- 按用户要求新增 docs/diagrams/方案C_整体流程.svg 与 2700×2400 PNG，覆盖采集、Pose/ByteTrack、联合深度、ReID锁定恢复、近远统一定位、三维状态和Foxglove；标明默认确认开关及实机验收边界。
- 使用已有 rsvg-convert 渲染，已目视检查文字、布局与连线；保留可编辑SVG，清理本轮临时生成脚本。未修改算法或部署。

- 按用户反馈简化 C 流程图措辞，保留结构和技术内容，同步重新导出 PNG；已目视检查文字无溢出。未改算法。

## 2026-10-04：C 二维/三维连续判断与位置融合

- 二维姿态逐帧连续记录，三维使用独立米制历史。短缺深度默认容忍0.35秒，缺失时间不计入横卧确认；超时只清理三维候选。可靠三维直立/高位证据可否决二维透视误判。三维确认的跌倒须恢复三维站立持续2秒才清除；新轨迹不继承事件。
- 原当前身体深度投影到可见躯干中心射线，再利用同轨迹此前身体与脚部的对应关系换算为脚下参考点。连续3次一致配对后可用，最多8条样本，关系默认0.75秒过期；仅站立时短暂缺脚可用，输出跟随当帧深度，不保持旧坐标、不填关节、不跨身份转移。
- 统一节点对身体换算、真实踝深度及单目接地点进行一致性检查和保守协方差交集融合，加入来源偏差项；相关观测不当独立样本累计置信度。有共同来源时连续输出，全部替换才重新确认。原光学TargetState保留。
- 新增14项连续性/融合逻辑测试并扩展ROS测试。初轮统一定位ROS确认断言失败，补充诊断输出后针对性及最终全套通过；未放宽产品阈值，单次失败根因未确定，初轮日志保留。
- 最终Linux ARM64 Humble专项退出0：14包构建15.2秒、138项感知逻辑、35项语音测试，以及C融合、配准、B测距/TF/并发、地面/先验、ReID/锁定、统一定位、三维姿态、A适配器、各route和公共API通过。证据artifacts/c-continuity-final.log。这是软件与合成输入验证，真实精度及误报率仍待实测；未连接或部署小车，安装确认仍默认关闭。
- 同步统一定位文档、接口、主方案、C验收及AGENTS长期规则，流程SVG/PNG重新导出并目视检查。最小审查检查参考点、当帧测量、来源相关性、过期及身份隔离；git diff --check通过。复用现有环境，测试容器和构建输出自动删除，保留验收证据，原三个业务容器保持healthy。

## 2026-10-07：当前电脑验证 C 方案

- 用户改为本机验证，停止继续车上部署。本轮未SSH、未切换在线B或启动车辆。恢复日志明确失效的原Colima，复用现有Humble镜像与Python环境；可用磁盘约20GiB，无新增大型下载。
- Gemini SDK真实采集300帧，640×400约29.97FPS，保存样本有效深度84.48%–85.81%；序列号AY2755200PW及设备标定与仓库一致。UVC640×480成功，画面朝天花板无人；无真人姿态、同步RGB-D或物理量距验收。
- 官方Pose真实示例4人/17点/静态ID延续通过，Mac CPU约122–132ms；相机无人静态图约143–216ms。OSNet真实ONNX独立裁剪测试通过，不代表现场身份恢复准确率。
- 14包构建、138项感知逻辑及多数ROS检查通过。首轮身份测试意外epoch变化失败；复测身份通过但三维跌倒同样因epoch变化失败。独立补测三维、A适配器、各route、公共API及35项语音测试全部通过。未修改代码或放宽阈值，保留失败，原因待查；不得称整套稳定通过。
- 新增docs/方案C本机验证20261007.md，证据artifacts/local-validation-20261007/。最小审查核对统计、原始日志和验收范围；git diff --check通过。测试容器--rm、构建随容器清理，删除本轮/tmp重复控制台日志，保留真实采样及失败证据，不删业务容器/镜像/卷。

- 2026-10-07 按用户要求打开本机Gemini实时彩色预览：AVFoundation USB Camera，640×480/30FPS，ffplay 960×720窗口。运行日志确认持续解码与显示时钟推进；仅原始彩色预览，无骨架/深度叠加、无ROS/Foxglove接入，不录制视频。用户按Q或关闭窗口结束。未改算法或车上服务。

- 2026-10-07 新增scripts/preview_pose_mac.py，本机RGB实时人体框/COCO17骨架/ByteTrack ID/二维状态预览，复用C后端和pose.yaml规则。后台只保留最新帧，显示CPU推理速度；过期观测状态未知，采集间隔超过门槛重建epoch。未接同步深度、ReID、ROS或车辆，安装未确认时保持禁止确定跌倒。已启动真实USB相机并验证持续推理日志；启动样本无人，不能宣称真人状态验收。最小语法/差异检查通过。按Q或关闭窗口释放相机。

- 2026-10-07 用户授权录制：本机原始彩色预览与MJPEG录像已启动，随后SDK原生深度逐帧记录，目录data/recordings/gemini-20261007-session1。已验证两路时间表持续增加，保留主机接收时间用于后续匹配，不宣称硬件同步。开始前20GiB可用，最多180秒约3GB深度上限；停止后核验并清理临时采集程序，真实记录保留。

- 同次采集已按用户要求停止：彩色3657帧，ffprobe完整解码计数通过，实际接收约121.69秒；深度3248帧约108.02秒，3248个文件尺寸全部正确，两路主机时间重叠约107.99秒，总计1.6GB。末尾SDK发生USB请求超时并未自行退出，已对本任务确认PID发送TERM终止，缺失尾段不补造。capture-summary.json保留统计与同步限制，临时采集程序已清理，真实录像/深度保留。

## 2026-10-07：基于真实录像的优化评估

- 新增离线回放脚本analyze_gemini_recording.py，复用实际Pose/ByteTrack、二维规则、设备去畸变/D2C和深度融合。原始录像每6帧取1，共610帧，542帧通过主机接收时间40ms候选配对；不代表曝光同步，未启用未标定的三维/地面或ReID。
- 发现站立短暂跳坐蹲、转身轨迹1变4、坐地转躺卧误报站立。29–37秒40/40坐蹲；94–98秒19/20躺卧。对站立窗口有ID最大框189样本做0.3秒显示层切换确认试算，坐蹲输出7变0；仅同录像候选实验，有延迟，不接入正式算法或宣称泛化精度。
- 彩色最大采集间隔95.97ms，深度82.96ms，无本段超过1秒采集间断；不能据此解释先前ROS epoch失败。相机倾斜明显，三维改善需要实际标定。详见docs/采集录像分析20261007.md。
- 已生成610帧骨架/状态MP4及逐帧JSONL、统计和抽样图，位于artifacts/recording-analysis-20261007。实际脚本运行退出0，ffprobe完整解码610帧；Python语法及git diff --check通过。最小审查核对离线时间基准、缺深度不填值、采样率和未标定边界。删除可重建的中间AVI，原始采集及分析证据保留。没有改正式阈值或车上程序。

## 2026-10-08：已有录像的深度、三维关节、ReID及epoch补充验证

- 用户要求完成无需重录的检查。本机实际Pose+OSNet回放610帧，新增置信度、三维坐标/有效性、ReID质量及身份输出；缓存重放逐帧校验Pose/深度/状态一致，ReID遮挡范围对齐正式已跟踪人体输入。新增summarize_recording_validation.py，不修改正式算法阈值。
- 542帧主机时间候选配对；有主体最大框的510个样本中488个有身体深度。应用正式关节跳变拒绝后，497个有ID样本中几何合理躯干148、有效双膝4、双踝107。黑色裤子区域大量空洞，不能称完整三维骨架通过。
- 90.122/90.320/90.518秒来源从躯干切下半身再返回，Z约2.909/1.826/2.948m；真实默认卡尔曼缓存重放仍保留约1.076/1.122m跳变。测距稳定性未通过，绝对误差无真值不能计算。49帧±100ms配对敏感性检查最大Z差约8.66/10.38cm，不等于真实同步误差。
- ReID在新轨迹0:4出现约2.01秒后接回person_0003，但同主体全段分裂成4个持久身份；歧义/质量拒绝保留，不能称身份稳定验收通过。5Hz离线回放不等于真实worker时延或锁定服务验收。
- epoch取证确认两次on_pair重置为ROS时间倒退72.7/22.1ms，而单调到达间隔97/114ms；Colima校时日志记录约119–180ms回拨。首个取证容器/tmp挂载不可见导致脚本未执行，换成项目内只读挂载后取得复现。
- 仅修正两个合成测试：MonotonicRosClock配合use_sim_time避免VM墙钟跳变，保留超时测试，新增主动回拨0.5秒失效断言。两项各连续3次通过；最终完整C专项退出0，14包构建21.2秒、138项感知逻辑、35项语音和相关ROS回归通过。正式时间倒退保护和虚拟机校时服务均未改。
- 报告docs/已有录像补充验证20261008.md，证据artifacts/recording-validation-20261008/。最小审查核对原始/过滤后统计、候选时间配对、ReID输入质量、缓存一致性、时钟保护与实机边界；Python语法和git diff --check通过。未SSH/部署。测试容器和构建自动删除，清理本轮临时脚本、重复控制台日志及可重建AVI；保留原始数据、JSONL、图和失败/通过日志，不清理业务镜像/数据卷。

## 2026-10-08：三维位置时间门控与关节跳变修正

- 依据同日录像验证的测距跳变，`DepthTrackFilter` 新增可选 `DepthGate`（B/C共用，`depth_gate_enabled` 默认 true）：新息超过 0.35m+2m/s×距上次接受时间即拒绝，被拒不刷新测量时间、按真实年龄保持预测；重定位需一致证据跨 0.3s 后干净重初始化，拒绝期间超出基础噪声的点不以大增益混入。C 中 `pose_lower_body`/`regions` 视为无躯干佐证的退路来源：门限×0.6、噪声×2、不能单独起始轨迹；被门控的当帧人体深度不写入 PersonState。`GroundTrackFilter` 未启用门控，行为不变；`depth_gate_enabled:=false` 恢复旧逻辑。
- 关节跳变改为 `JointJumpGate`：按关节保存上次接受值，被拒值不再成为下一帧参考，重新出现的关节同样检查，允许量为 0.5m+2m/s×年龄（新参数 `pose_joint_jump_speed_mps`），超过 `pose_max_gap_s` 参考过期。
- 同录像复测（gate_check.py，正式类+节点默认参数，非ROS全流程、无真值）：5Hz 发布Z相邻变化>0.5m 由5次降为0，最大1.122→0.352m，输出528→525；新跑的30fps全帧回放（3657帧，未开ReID）由4次>0.5m（最大1.134m）降为0次>0.25m，拒绝23个（17个为新轨迹开头退路深度待确认），输出3218→3195；关节拒绝27→22，躯干/双膝/双踝可用数不变。结果写入 docs/已有录像补充验证20261008.md，参数语义见 docs/方案B实现与验收.md 末节。
- 新增11项门控/关节单元测试（感知逻辑由138增至149项），test_pose_runtime 增加“退路深度1m跳变被扣留、不成为目标位置”ROS检查。test_identity_lock_runtime 原有 0:8 轨迹单帧 2→3m 并立即要求位置有效，与新设计冲突；改为断言该帧不发布3m、持续一致后重定位到3m，其余断言不变，连续3次通过。
- 回归：第一次完整C专项 test_yolo_runtime 墙钟边界断言失败（当时全帧回放占用CPU，追踪确认滤波未拒绝、行为同旧逻辑）；第二次为既有间歇的 test_yolo_concurrency 第79行超时（日志 artifacts/depth-gate-concurrency-failure.log）；第三次 identity lock 如上失败（artifacts/depth-gate-identity-failure.log）。修正后最终完整C专项退出0：14包构建、149项感知逻辑、35项语音及全部C/B/A路由与公共API ROS检查通过，日志 artifacts/depth-gate-regression-final.log。前两类失败未修改产品门限，不宣称其根因已定位。
- 门限为单段录像加常识初值，未经实车噪声标定；>5m 远距离可能偏紧。没有SSH/部署或车辆操作。最小审查核对拒绝不刷新时间、保持年龄、退路起始、地面滤波未变及接口兼容；结构检查与 git diff --check 通过。删除可重建的全帧 annotated.avi，保留 frames.jsonl 与对比结果。
- 2026-10-08 新增 docs/补录视频要求20261008.md：按 ReID 两人（衣着差异/相近）、浅色裤子姿态、卷尺测距真值、空场景五段给出脚本、安装记录、标注格式和优先级。核对发现 10-07 Gemini 采集程序已清理、不在仓库，正式补录前需先恢复为仓库脚本并试录核对格式；当前磁盘约 17 GiB 可用，五段约 9–10 GB。仅文档，未录制。
- 2026-10-08 用户确认后执行 `git gc --prune=now`：.git 由 1.1G 降至 225M。清除的 3844 个无引用松散对象中约 915MB 与工作区现有文件（主要为曾暂存的厂商目录）逐字节相同，其余约 53MB 为未提交旧版本及与 65a68bc 重复的孤立提交 437f2e7。stash 保留，git fsck 无错误，工作区未改动。未清理 uv 缓存、Docker 镜像或 Astra 旧录像。

## 2026-10-08：ChArUco 标定检查入口

- 用户询问标定板与厂商“读取出厂内参”方法。从用户照片实测识别板为 OpenCV CharucoBoard((7,5))、DICT_5X5（ID 0–16 全检出、24 角点），标记边长按照片单应估算约 22 mm，需用户实尺复核；另一块 9×9/18 mm 棋盘格方向对称，只建议用于内参。
- 新增 `scripts/charuco_calib_mac.py`：Mac UVC 彩色实时显示板中心距离（出厂内参 PnP）与重投影误差，保存视图后对比独立标定与 `gemini_AY2755200PW.json` 出厂彩色内参，报告写入 artifacts。尺寸必须显式输入；分辨率不符直接报错。只检查彩色内参，不含深度、D2C 配准或安装外参。
- 验证：按出厂内参渲染 40 张合成视图离线运行，fx 偏差 −0.31%、主点约 1 px、出厂内参逐视图 PnP 中位 0.31 px。仅为合成数据，未连接相机实拍；UVC 640×480 与 SDK 出厂模式是否同一成像几何待实拍对比确认。
- 2026-10-08 用户复现门控漏洞：默认 0.5 秒超时下先接受 2m，再以 30Hz 交替 3/4m，第 16 帧直接接受 4m。原因是以“距上次接受”判断断流。已改为按最后一次输入（含被拒）计时；输入持续但一直被拒超过 `max_age_s` 时丢弃旧状态，之后任何新位置都需 0.3 秒一致确认。新增 3 项默认超时测试（含用户复现，90 帧全拒），单元测试 152 项通过；两段录像对比结果不变（该录像无超过 0.5 秒的连续拒绝，此前回放未覆盖这一情形）。
- 回归中 test_pose_runtime 第 93 行间歇失败：仅区域深度、无关键点的帧在断流超过 1 秒后到达时，按“退路不能单独起始轨迹”需 0.3 秒确认，这是上一轮门控引入的行为变化（此前靠时序通过）。测试改为先刷新轨迹再检查“已有轨迹缺关键点仍有效”，起始规则由单元测试覆盖；方案B/C文档已注明该延迟。另一次 test_yolo_runtime 第 165 行间歇失败经追踪与深度滤波无关（发布时墙钟新鲜度判断，疑似 Colima 回拨），单独 4 次全部通过，日志 artifacts/depth-gate-streak-yolo-runtime-failure.log。最终完整 C 专项退出 0，日志 artifacts/depth-gate-streak-regression.log。仅本机，无部署。
- 用户实拍 15 帧（A4 适合页面打印放大，实测格 40.3 mm/标记 29.6 mm，距离 0.36–1.33 m），证据 `artifacts/charuco-20261008-150604/`。独立标定 fx 453.0（出厂 452.4，+0.12%）、主点 cx +2.7/cy −1.3 px，均在独立标定主点不确定度约 ±2 px 内；同批视图出厂内参 PnP 中位 0.27 px、最大 0.78 px，独立标定 0.22/0.78 px。结论：出厂彩色内参与实拍一致，继续使用，不替换。覆盖集中于画面中心、四角未达（最大归一化半径 0.82），因此独立标定畸变不作为替换依据；本结论不涉及深度、D2C 配准或安装外参。首帧平均灰度 18/255 且蓝色通道显著偏高，开灯后是否仍偏蓝需另查 UVC 通道顺序。
- 新增 `scripts/charuco_mount_mac.py`：ChArUco 平放地面（红轴朝车前、绿轴朝车右），以卷尺量得的板原点位置及出厂彩色内参求 `base_footprint`→`camera_color_optical_frame` 外参，输出高度/俯仰/偏航/横滚、多帧高度离散及姿态告警，复用 `charuco_calib_mac.py` 的检测与内参读取。合成渲染（真值高 0.25 m、俯仰 20°、偏航 3°、横滚 1°，板距 0.30/0.38 m）恢复高度误差约 1 mm、俯仰 0.1–0.2°、偏航 0.06–0.24°、x 约 3 mm；板距 0.45 m 时合成图过小无法识别，需近放。四元数含 180° 情形往返误差 ≤1e-16，修正了一处退化分支错误。未实拍；x/y/偏航精度取决于卷尺与对齐，`camera_link` 换算需板上驱动 TF，未改任何 `extrinsics_calibrated`/`confirmed` 开关。厂商 mini_akm 静态 TF 以 base_footprint 为父系，base_link z+0.019 m，相机名义位置为原厂支架值，不代表 Gemini 实际安装。

## 2026-10-08：云台方案与串口协议草案

- 用户确认相机改装到车上独立云台（360° 舵机水平 + 180° 舵机俯仰），计划在相机支架加 MPU6050，由单独 MCU 读取后经串口与 Jetson 双向通信。新增 `docs/云台串口协议草案.md`：坐标链 odom→base_footprint→pan_link→tilt_link→{imu_link, camera_link}；云台 yaw 指令为相对车体关节角、pitch 为相对重力，distance 不下发云台；底盘跟随人在 base_footprint 的位置而不直接跟随云台角，目标丢失时底盘停车、云台单独搜索。
- pan 角两种来源同帧格式：方案 E AS5600 单圈绝对编码器（±170° 限位内无需回零，推荐）；方案 G MPU6050 竖直分量积分减 Jetson 下发的车体偏航率，零位霍尔开关分方向边沿回零并上报漂移。协议 460800 8N1 小端、CRC16-CCITT-FALSE，STATUS 60 字节载荷 200 Hz、COMMAND 28 字节 50 Hz、最小 RTT 时间同步、300 ms 指令超时停转保持、MCU 内软限位；`pan_valid=0` 时 Jetson 停发 pan 关节使相机到车体/odom 的 TF 缺失并失效。
- 校验：Python struct 核对各载荷长度（60/28/12/9/5 字节）与 200 Hz 带宽约占 30%；文中 C CRC 函数本机编译，`123456789` 得 0x29B1 与标准校验值一致。仅为设计草案，无固件、ROS 节点、URDF 或硬件验证；舵机/MCU 型号与 pan 方案待定。
- 按用户要求扩充并改名为 `docs/云台跟随设计与串口协议.md`（原 `docs/云台串口协议草案.md`）：新增坐标系原点/轴向/来源表、TF 发布归属及启用云台时须关闭的三处相机 TF（厂商 base_to_camera、感知 publish_mount_tf、导航 sensors.yaml camera），完整数据流（odom 中滤波、按观测时刻查 TF、ByteTrack 用 K·R·K⁻¹ 转动补偿），云台几何指向+画面 PI 微调+车体转速前馈+限速（运动模糊估算）+限位，底盘阿克曼纯追踪（κ=2sinα/d、cosα 减速、v=0 时 ω=0、不倒车、测量年龄>0.3 s 停车、侧方近距等待），状态机与失效处理、时延预算，以及云台运动学/最小转弯半径标定和 6 项系统级验收。仍为设计文档，无代码或实车验证；参数均为初值。
- 核对厂商 mini_akm URDF：`base_footprint` 原点为两轴之间车体中心的地面投影（后轴 x≈−0.0697 m、前轴 x≈+0.0736 m，轴距约 0.143 m，轮距约 0.16 m，轮半径约 0.034 m），并非先前口头所说的后轴中点。设计文档已改正：沿用厂商原点以与 base_link/laser 静态 TF 一致，纯追踪先平移到后轴 `x_r = x + 0.070`，跟随距离从后轴起算。
- 新增 `docs/diagrams/云台坐标系3D.html`（three.js r128 交互图，已发布为私有 Artifact）：按 URDF 尺寸建车，云台/相机/IMU 为示意位置；显示 odom→base_footprint→pan_link→tilt_link→{imu_link,camera_link}→光学系坐标轴与标签，可切换显示；手动或自动几何指向 pan/tilt（限位 ±170°/−30°…45°），实时显示 base_footprint→光学系变换、人相对后轴的 d/α 和纯追踪 v/ω；跟随演示按文档控制律积分自行车模型。本机 http 预览确认渲染、标签分离、自动对准、演示中“前进跟随”和限位提示正常；仅示意几何，不是实车数据。
- 用户暂定跟随距离 1.9 m（后轴起算）。设计文档更新：依 Gemini 出厂彩色内参（竖直 55.9°、水平 70.5°）与 1.7 m 身高，相机到人约 1.65–1.75 m 才能整人入画；tilt 改为对准头脚中点（1.9 m 时约抬头 15°）；pan 限位加回卷滞回（越过正后方且距限位朝向 ≤ 半视场−5° 时保持，超出才回卷）；新增 §4.1 位置来源优先级（光学躯干深度为控制主输入，统一接地点为后备/显示/跌倒，单目地面独立对照，单目仍在统一定位中作双踝无深度时的后备）与 §4.2 深色裤子应对（不依赖腿部深度、脚部改单目几何、待测红外曝光/激光参数、不做大空洞填补）。深度依据为既有录像 488/510 身体深度样本与 1.8–2.9 m 实际距离，绝对精度仍待 1/2/3 m 对比。
- 3D 图同步：d*=1.9 m、tilt 头脚居中、pan 45°/s 与 tilt 30°/s 限速、限位保持与回卷提示、整人入画判定读数。本机 http 预览（面板可见时）用滑块脚本验证：前方 2 m 时 tilt −16° 整人入画；跨越正后方小角度时保持 170° 并画面内跟踪；超出 30° 后回卷约 7 s 并提示暂时丢失。已重新发布到同一 Artifact。
- 设计文档补充：§4.3 用出厂内参指出深度竖直视场仅 45.3°（彩色 55.9°），横放 0.25 m 高时深度覆盖全身需约 2.14 m，1.9 m 跟随时脚在彩色内但出深度视场，双踝深度来源基本不可用；§4.4 候选“深度 RANSAC 地面在线拟合”，作为平面证据供单目双踝求交、核对 IMU 与地面确认，不取代单目、不自动确认地面；§4.5 云台对跌倒规则的影响（重力方向须逐帧取自 TF/IMU，直立改为逐帧 IMU 横滚判定，云台转动期间暂停二维下坠计时或改用重力系度量）；§4.2 加“确认 SDK 空洞填补关闭”；§14 记录相机竖装方案（彩色/深度整人入画 1.16/1.24 m）暂不实施。仅文档，未改代码。

## 2026-10-08：深度地面拟合与云台动态重力（初步）

- 新增 `yolo_person_tracker/floor_plane.py`：已配准深度下部采样、排除人体框，RANSAC（一半假设局部邻域取点）+ 法向预筛 35° + MAD 自适应收紧内点（5 mm–2 cm）+ 平面内两主方向 std ≥0.2 m 的窄条拒绝；连续 3 帧一致才 stable。新消息 `person_interfaces/FloorPlane`，pose 节点 `floor_fit_enabled`（默认 false）时按彩色帧 header 发布 `floor_plane`。
- `Pose3DConfig.up_source`（默认 static，行为不变）新增 floor/tf：逐帧重力与相机高度；缺失时本帧退二维且不复用旧值；横滚 >3° 时二维未知；重力方向变化 >2° 清空二维像素基线与疑似（保留已确认事件）；二维确认仍需 `pose_upright_confirmed`，三维仍需 gravity/ground 确认。tf 源按观测时刻查 `gravity_frame`。
- 统一定位 `floor_plane_enabled`（默认 false）：仅用完全相同时间戳的稳定实测地面，与双踝射线在光学系求交得 `mono_ankles_floor`，替代标定平面单目；与已确认地面不符则不用并注明。配置注释写入 pose.yaml 与 gemini/camera_mount.yaml。
- 验证（合成数据）：本机与 Humble 容器 171 项单元测试通过（新增地面 9、动态重力 7、实测地面统一定位 3）；抬头 15°/横滚 2°/墙 2.5 m/4 mm 噪声/10% 空洞的 10 个种子法向误差 ≤0.3°、高度 ≤1 cm，窄地板条判无效；2° 安装俯仰误差下标定平面单目偏 >0.3 m，实测地面 <5 cm。新增 `tests/test_floor_runtime.py` 并纳入 `scripts/test_c_container.sh`，完整 C 验收（含二维姿态两种融合、三维、统一、身份、路由与公共 API）退出码 0，日志 `artifacts/floor-gravity-c-acceptance.log`。开发中修正：初版 150 次全局随机采样在地板仅占约 18% 时会选错平面（最大 3.35°），墙脚一侧点使平面偏 0.4°。Mac 上每次拟合约 20 ms，Jetson 开销未测；未在真实地板、反光瓷砖或云台上验证，未部署。

## 2026-10-08：云台链路软件（无硬件）

- 已提交此前全部改动到分支 `claude/gimbal-floor-design`（3d57133）。
- 新增 `gimbal_interfaces`（GimbalCommand/GimbalStatus）与 `gimbal_bridge`：协议编解码与逐字节重同步、MCU 计数回绕展开与最小 RTT 时钟同步（含主机时钟跳变即时重新锚定，`clock_steps` 上报）、termios 原始串口、桥接节点（50 Hz 指令转发与 HOLD 心跳、ACK 服务、无同步/无效/CRC 超限/状态超时不发布 pan 或全部关节）、无 xacro 的 URDF 生成与 `gimbal_mount.yaml`（占位值，confirmed=false 时 launch 不发布 TF）、方案 E/G 的 MCU 模拟器、独立 `gimbal.launch.py`。`check_project.py` 包清单加入两包。
- 排查：Colima 虚拟机墙钟跳约 164 ms 导致 joint 时间戳偏未来，确认为环境时钟步进而非解回绕错误，据此加入跳变重锚（也覆盖 Jetson 无 RTC 开机 NTP 步进）；Node 子类方法名 `handle` 覆盖 rclpy 属性已改名；测试改为查 0.1 s 前的关节帧，避免追最新 TF；`ros2 run` 不转发 SIGTERM，测试直接运行 rsp 可执行文件。
- 验证：`scripts/test_gimbal_container.sh` 15 项单元测试与伪终端端到端测试连续 3 次通过（CRC 重同步、MCU 时钟回绕后时间戳单调且延迟 0–30 ms、TF 跟随指令 ±0.01 rad、HOLD 心跳不超时、归零服务、磁铁丢失/断口撤销 TF、陀螺方案未回零不发布 pan、launch 未确认不发布 TF 且非仿真拒绝放行），日志 artifacts/gimbal-test.log；结构检查 16 包通过。纯软件仿真，无真实舵机/编码器/IMU/USB 串口，未部署。
- 2026-10-08 新增 docs/现场实测要求20261008.md：列出板端前置条件（相机 ID/序列号、驱动提交、标定、chrony 校时方式、Pose FP16 engine）及 T1–T9 实测（物理配准、安装外参与地面、重力方向、测距与门控含 6m、同步延迟、Jetson 性能、两人身份锁定、受控姿态、Foxglove），给出依赖顺序、交付物与建议通过线（标明为待确认初值），各确认开关只在对应实测通过后打开。运动/雷达/导航不在本轮范围。仅文档，未上车。
- 按用户意见暂缓标定板深度精度实验，重写 `docs/补录视频要求20261008.md`：新增 A 场次车载机位（实测高度、抬头使 1.9 m 处整人入画、地面入镜）下的模拟跌倒正例（A1/A2 1.9 m 侧向/朝向/背向/行走/椅子滑落，A3 3 m 含两段式跌倒）、易误报快速动作 A4、日常慢动作 A5（原 S2）、深色裤子 A6、遮挡出画 A7，B 场次保留两人 ReID（原 S3/S4）；原测距真值 S1 暂缓、空场景并入每段开头 5 s。写明垫子 ≥10 cm 与受控跌倒等安全要求、五点跌倒标注格式、优先级与约 16 GB 磁盘估算，以及采集程序与标注工具两项待写前提。仅文档，未录制。

## 2026-10-09：Mac 录制与标注工具

- 新增 `scripts/record_gemini_depth.cpp`（SDK v1.10.16 深度 640×400、存储前 zlib 1 级无损压缩为 `N.depth.z`、写 device.json/timestamps.csv、stop 文件或信号停止）、`scripts/record_gemini_mac.py`（编排：自动编译辅助程序、开录前磁盘检查、等深度出帧后录 UVC 彩色、实时预览含深度小图、停止时 SDK 卡死则 TERM/KILL、写 capture-summary.json 和 notes.md 模板）、`scripts/gemini_recording.py`（共享读取 raw/zlib 深度与汇总：帧数、重叠、最大间断、抽样解码校验）、`scripts/label_recording.py`（逐帧标注，五点跌倒与动作起止，顺序/成对检查，保存与续标）。`analyze_gemini_recording.py` 改用共享读取，兼容两种深度格式。
- 验证：10-07 录像抽 10 帧 zlib 1 级压缩比 7.16、约 3.1 ms/帧且无损；共享汇总在 10-07 录像复现 3657/3248 帧、重叠 107.99 s、最大间断 0.096/0.083 s；截取 90 帧彩色 + 95 帧压缩深度跑 `analyze_gemini_recording.py`，15/15 抽样帧配对深度并正常输出；`tests/test_recording_tools.py` 6 项（压缩/原始读取、间断/缺失/损坏报告、标注时间基准、顺序检查、撤销、保存续标）通过；无设备时辅助程序报“需要一台 SDK 深度设备”，越界时长拒绝。相机未连接，**未真机试录**，彩色采集需在终端获摄像头权限。补录文档更新命令、磁盘估算（约 3 MB/s，全部约 3.5 GB）与文件格式（labels.csv 增 frame、host_wall_ns）。
- 用户要求测真实跌倒：补录安排改为防护垫（≥20 cm）上真实速度跌倒，不再要求先屈膝放慢；A2 增绊倒前扑、晕倒式瘫软、后仰坐倒；标注方向新增 collapse、backward_sit（标注工具同步，6 项测试仍通过）。`docs/现场实测要求20261008.md`（另一会话所写，此前未入库）去掉“不做真实跌倒”，T8 改为日常姿态，新增 T10 板端真实跌倒：1.9 m/3 m 七类跌倒各 2 次与五类反例、侧拍手机视频定触垫时刻、检出率/疑似与确认延迟/误报的初始通过线。两份文档统一底线：只跌在防护垫上、有人看护、不在硬地面跌；结论不推广到无防护的意外跌倒。
- 按审阅意见修订补录安排：A0 增垫上侧躺可见性检查；A1 拆为 A1a 侧向、A1b 朝相机、A1c 背对相机行走前扑（跟随场景最常见，垫子纵放，1.6 m 起步避免低机位切头），每种 5 次；跌倒循环缩为约 20 秒；A4 拆两段并加踉跄后站稳、躺“沙发”；新增 A8 第二名参与者重复 A1 各 3 次；注明低机位加 20 cm 垫子的遮挡与垫面高于地面；优先级表改为压缩后用量（全部约 5 GB）。现场实测 T10 同步三种主要跌倒各 5 次与新增反例；标注工具加 walking_away、stumble_recover、sofa_lie，notes 模板加垫子厚度/朝向/位置。仅文档与工具枚举，测试 6 项通过。
- 2026-10-09 用户在终端真机试录 10 秒（gemini-20261009-test-10s）：设备 AY2755200PW/RD3013，读出标定与仓库 JSON 一致；彩色 294 帧约 29.4 FPS，深度 336 帧（比彩色早约 1.0 s 开始），重叠 9.997 s，深度最大间断 67 ms；zlib 存储 10 秒 35 MB；抽样深度有效 40–56%。辅助程序正常输出 DEPTH_DONE 退出（SDK 停流时仍报 usb_request_cancel 超时警告，未需强杀），SDK Log 写在录像目录内。`analyze_gemini_recording.py` 直接读取该录像，30/30 抽样帧配到深度，配对差 ≤16 ms，检出人体。彩色唯一较大间断 232 ms 出现在第 3 帧，系首次 imshow 建窗阻塞；已改为计时前先建窗并预读 3 帧，其余偶发约 60 ms 为单帧丢失。录制链路可用于正式补录。
- 录制程序支持只输入补录段号（A0…A8、B1、B2，不区分大小写）自动展开目录名与建议时长；同名目录非空时自动加 -partN，不再拒绝退出；自定义段名仍可用并拒绝空名、路径分隔和隐藏名。新增段名测试，录制工具测试 7 项通过；文档命令同步。

## 2026-10-09：跌倒检测首轮优化（补录录像，未标注）

- 新增 `scripts/replay_falls.py`（按录像时间重放 2d/3d-fixed/3d-floor/height 规则与变体开关，缓存地面拟合和身体高度）与 `scripts/evaluate_falls.py`（按 labels.csv 计检出、触垫后延迟、反例误报、标注外检出，含 1 项合成测试）。
- 跌倒规则新增默认关闭开关：`pose_gap_pause`、`pose_handover_s`（躺卧断轨时继承下方附近唯一新轨迹的状态）、`pose_confirm_lying_only`、`pose3d_knee_fallback`；新增 `height_fall.py` 无骨架身体顶高度线索，pose 节点 `height_fall_enabled`（默认 false），骨架直立否决下蹲，重力与地面未确认时最多疑似，结果仅写入 PersonPose.detail。
- 6 段录像目测：原规则跌倒段几乎全漏（低机位遮挡肩髋、躺卧低置信断轨、无直立基线、跌倒约 1.6 s 超过 1 s 窗口）；骨架开关仅小幅改善，高度+交接+骨架否决在 A1a/A1b/A1c 疑似 3/7/5、确认 1/6/3，A4a 无误报，A5 剩 2 次疑似。阈值在同批数据上调，地面拟合偏差约 0.3–0.5 m，录像段名与内容不符、看护人入画、相机移动。默认值待标注后复评。详见 `docs/跌倒检测优化20261009.md`。
- 验证：单元 189 项通过；容器内 floor/身份/统一/地面运行测试各 3/3、ros_runtime yolo_pose 路由与 C API launch（读新 pose.yaml）通过。完整 C 验收在主机高负载时 pose3d/pose 运行测试偶发失败，HEAD 同样复现，记为既有时序不稳。未在 Jetson 或实车运行。

## 2026-10-10：标注评估与 E-FPDS 基线

- 用户已标 A1a（4 次，实为朝相机 2 次、背对相机前扑 2 次，方向已按画面改正，原文件备份于 artifacts/fall-eval-20261009/A1a-labels-orig.csv）与 A1b-part2（8 次左向侧倒）。
- `evaluate_falls.py` 改为只统计在本次跌倒窗口内新升起的告警（每人最后状态保持 1 s，避免不同轨迹交替出现造成假上升沿），上一事件遗留未解除的告警记为 carried_in、不算检出；多变体同批评估不再互相覆盖。录制工具测试 9 项通过。
- 标注评估（同批数据调参，仅初步）：原骨架规则 A1a 0/4、A1b 疑似 1/8 确认 0；最佳骨架开关组合 A1b 疑似 3 确认 2。高度+交接：A1a 疑似/确认 3/3（另 2 次标注外告警），A1b 7/7，确认延迟中位约 1.0 s；加骨架否决：A1a 3/1、A1b 7/6，标注外告警为 0。两段各有 1 次因前一次告警未解除（恢复需持续 2 s 直立）而不计检出。
- 下载 E-FPDS 测试集与老人子集（未下训练集、未下 YOLOv3 权重）至 data/external/fpds（不入库），新增 `scripts/evaluate_fpds.py`（尚未提交）。现有 YOLO26s-pose 对倒地人在 IoU≥0.5、置信度≥0.25 时召回 92%（中位 0.77），但在自录 A1a 躺卧帧中位仅 0.14–0.32；同帧转灰度后 detect 中位 0.37→0.75，提示自录画面严重偏蓝（平均 BGR≈165/123/40）可能是主因，通道互换与灰度副作用待查。另见 pose 模型漏检 E-FPDS 走廊远处/半身站立者约 36%（detect 约 1%）。
- 10-10 续：用户标完 A1c、A2a、A4a、A5，按画面改正方向与动作名（原件备份 artifacts/fall-eval-20261009/labels-orig/）。20 次跌倒/25 个反例汇总：原规则疑似 2、确认 0；骨架开关全开 5/2；高度+交接 15/13，A5 反例误报疑似 6、确认 3；加骨架否决 15/10，误报疑似 2、确认 0。A2a 椅子滑落因坐姿顶高低于 0.8 m 基线门限全漏；远处站立顶高偏低同时导致告警不解除与 A5 误报。详见 docs/跌倒检测优化20261009.md §8。
- 10-10 续：高度线索的骨架否决改为仅高框（框高 ≥1.5×框宽）生效，`recovery_s` 默认 1 s，`replay_falls.py` 增加 upright/low/drop/recovery/confirm 与 `veto2d=<比值>` 变体。标注 20 跌倒/25 反例：交接+新否决疑似 16、确认 14、反例误报疑似 2/确认 0、确认延迟中位 1.06 s（旧否决 15/10）。单元 191 项通过，容器 floor 运行测试与 C API launch 通过；height_fall_enabled 仍默认关闭，门限同批数据选取，未上车。详见文档 §9。

## 2026-10-10：Gemini 彩色偏蓝排查（Mac）

- 新增 `scripts/uvc_controls.c`（libusb 读写 UVC 白平衡/曝光，不取流）、`scripts/color_check_mac.py`（实时 R/B 与白平衡切换、`--sweep`）、`scripts/gemini_color_probe.cpp`（SDK 彩色属性；Mac 上彩色接口被系统驱动占用不可用）。
- 结论：相机 UVC 参数均为出厂默认；同一场景 1920×1080 与 2592×1944 每次正常，1280×720/1280×960/640×480 大多偏青蓝（R/B 约 0.15–0.23），偶尔正常，手动色温无法纠正，疑为固件缩放模式色彩异常。10-09 正式录像全部偏蓝而试录恰好正常。640×480 未指定帧率时为 60 fps、画面偏暗。1280×960 与 640×480 同视场 2 倍，2592×1944 约 2560×1920 区域对应 640×480，1080p 上下被裁。建议 Mac 录制改用 2592×1944 裁剪缩放并加开录偏色检查；车上需按实际驱动与分辨率另查。详见 `docs/Gemini彩色偏蓝排查20261010.md`。仅 Mac 实测，未上车、未改录制程序。
- 10-10 续（接管另一会话后）：发现反光瓷砖使地面深度为倒影、且录制中相机被移动，单段地面拟合不可用。新增 `scripts/estimate_camera_geometry.py`（竖直线消失点逐帧重力 + 站立双踝相机高度），六段相机高度中位 0.47–0.63 m；`replay_falls.py --geometry` 与 `3d-geometry` 模式。高度线索新增 `hint_upright_min_m`（站立骨架低顶高基线/恢复）、未测时高框站立骨架恢复、`top_edge_px` 框顶截断只作下界。标注 20 跌倒/25 反例：疑似 17、确认 15、反例误报 0、标注外告警 0，确认延迟中位 1.15 s；三维骨架规则同几何下仍仅疑似 2。单元 194 项、录制工具 9 项通过，Humble 容器构建与 floor 运行测试 2/2、C API launch 通过；`height_fall_enabled` 仍默认关闭，同批数据调参，未上车。详见 docs/跌倒检测优化20261009.md §10。另提交 `scripts/evaluate_fpds.py`（E-FPDS 召回，含 --preprocess gray）。
- 10-10 修复 Mac 录制偏蓝：新增 `scripts/gemini_color.py`（2592×1944 取流裁剪缩小到 640×480、开录偏色检查、`measure` 测量与原 640×480 模式的几何映射，结果存 `scripts/config/gemini_color_mode_AY2755200PW.json`）。`record_gemini_mac.py` 默认 `--color-mode full`，开录 R/B<0.5 拒绝、录中偏色预览警告并计入 problems，`device.json` 写换算内参（cx/cy 较出厂约 −8.2/−3.5 px）并保留出厂值；`charuco_calib_mac.py` 增 `--color-mode`。录制工具测试 13 项通过（新增裁剪、内参换算一致性、合成映射恢复、device.json 幂等）；真机 10 秒试录颜色正常（R/B 1.03–1.05，约 25 fps），分析脚本可读。换算内参未经标定板复核；车上彩色未查。
- 10-10 标定板复核录制彩色内参：`charuco_calib_mac.py` 报告增加只拟主点（固定焦距与畸变）及形式误差、4×4 覆盖格数、逐视图倾斜（左右/上下）与实时倾斜显示；`gemini_color.py apply-charuco` 按报告更新几何映射，要求 ≥12 视图、覆盖 ≥12 格、主点误差 <1.5 px、焦距差 <1%、左右与上下各 ≥3 张倾斜 ≥20°。用户首轮 13 张：cx 317.76±0.34 与换算值 317.55 一致，cy 235.5 比换算值 241.5 低 6 px，但 13 张倾斜都约 11° 且同一姿态（相机平移），主点与姿态退化，形式误差不可信，未应用；10-08 原模式数据同法得 cy 247.2±1.2（出厂 245.0）。录制工具测试 14 项通过。
- 10-10 第二轮 18 张（artifacts/charuco-full-20261010-b）：板距 0.24–0.34 m、均为上下倾斜 10–17°、左右约 0°，仍不满足倾斜门槛，未应用；只拟主点 cx 316.8、cy 242.8，与换算值 317.55/241.53 相差 −0.8/+1.3 px（第一轮 cy 235.5，两轮差 7 px，说明低倾斜下主点不稳定）；自由标定 fx 559 退化。结论：保持 SIFT 映射换算的录制内参不变，其精度依托 10-08 已核对的出厂内参（主点差约 2 px）与 0.25 px 级模式间映射。
- 10-10 将 `docs/Gemini彩色偏蓝排查20261010.md` 重写为完整的原因分析与解决方案报告（现象、影响、逐步排除过程、根因与不确定部分、取流与内参换算方案、防护、验证、遗留问题、相关文件）。仅文档。

## 2026-10-10：重拍安排

- 新增 `docs/重拍视频要求20261010.md`：按 10-09 录像暴露的问题（彩色偏蓝、相机中途移动、架设未记录、段名与内容不符、躺地过短、跌倒间隔过短、看护人入画、缺 A0/A3/A4b/A6–A8/B1 及真正快速躺下和踉跄）重排全部重拍；跌倒节奏为站稳 4 s → 跌倒 → 躺 4 s → 起身 → 站稳 4 s；朝相机前扑改为垫子纵放、从约 3.2 m 起倒；A2 拆为 A2a（后仰/绊倒/瘫软）、A2b（后仰坐倒/椅子滑落 ×3）；A0 增加地面平放标定板与两种躺姿可见性检查。10-08 安排加指向说明。
- `record_gemini_mac.py`：`--set-camera 高度cm 俯仰° 横滚°` 记录当天相机架设（`camera-setup-YYYYMMDD.json`，范围检查防止误填米），各段自动写入 notes.md 与 capture-summary；未记录时开录提醒。段号 A2 改为 A2a/A2b，A1 等段时长 150 s、A4a 170 s。录制工具测试 15 项通过。仅本机代码与文档，未录制。
