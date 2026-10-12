# 项目协作约定

1. 每次项目更新后更新 WORKLOG.md，并做与改动范围相称的最小审查；仅在长期规则或项目入口变化时更新 AGENTS.md，避免堆积历史日志。
2. 缺少必要环境时主动补齐；不得将静态检查描述为编译通过或实机验证。
3. 当前主线为 C（Gemini Pro + YOLO26s-pose 骨架/跌倒）；项目默认入口仍为 B。A 骨架与红色目标路线已退役，资料在 `docs/archive/`。历史上下文见 `docs/archive/AGENTS历史上下文.md`，逐次记录见 WORKLOG.md。
4. 文档新增或退役时同步维护 README.md 的文档索引；过时文档移入 `docs/archive/` 并在其 README 登记原因，不直接删除。

## 项目概况

- 目标：室内人体跟随小车（含跌倒检测）。主控 Orin Nano Super 8GB：Ubuntu 22.04.5 aarch64、JetPack 6.2 / CUDA 12.6、ROS 2 Humble；Mac 上用 Foxglove 与 SSH 调试。
- SSH：Wi-Fi `wheeltec@192.168.1.240`（别名 `roscar-wifi`）、网线 `wheeltec@192.168.100.2`（`roscar-ethernet`），端口 22。密码、代理订阅 URL、节点和登录凭据、讯飞/DeepSeek 凭据不得写入项目文件或日志；语音凭据只存板上权限 0600 的私有文件。小车上的 Codex CLI 与 Clash Verge 配置见 `deploy/README.md`。
- 相机：C 使用 Gemini Pro（序列号 AY2755200PW）；早期 Astra 实物为 ORBBEC ASTRA S（2bc5:0402），两者标定不能混用。底盘 OLED 显示 `Akm`，车型键按 `mini_akm`，不要用源码默认 `mini_mec`。底盘商品型号仍待核对。
- 代码：主动工作区 `ros2_ws/src/`，构建只扫描此处。厂商原包 `JP6.2_wheeltec_ros2_src_20260903/` 保持原样并排除 Git 和同步。迁入的厂商包（chassis_vendor、radar_vendor、wheeltec_mic、wheeltec_mic_aiui、tts_make_ros2）保留 `COLCON_IGNORE`，由独立脚本构建；`SOURCE_MANIFEST.json` 保留原始哈希，本地补丁记 `local_modifications`。
- Jetson 正式目录 `/home/wheeltec/ROSCAR-current`（含在运行的语音服务），离线语音依赖在 `/home/wheeltec/ROSCAR-offline`。Mac 同步脚本默认 dry-run、按白名单，不传厂商包、权重、录像、构建产物或代理配置，不执行远端删除。旧 A 的 `roscar-route-a.service` 若在运行会占用相机，启动其他相机入口前先停止。
- 只有显式 `route:=demo` 输出 `is_simulated=true` 的模拟数据；demo、容器编译、CI 单测、合成测试和离线录像回放都不能描述为实机识别或实车验收。感知验收中不得混入跟随或底盘控制节点。
- 流程图位于 `docs/diagrams/`，必须区分“已有源码/库”“待开发/接入”“待实机验证”；修改 SVG 后如需分享合并图，同步重新导出 PNG。

## 当前状态（2026-10-11）

- C：已部署到板上隔离目录 `/home/wheeltec/ROSCAR-c`（Pose FP16 engine、Gemini 驱动均为板上构建），10-12 真人二维骨架与实车低速跟随（1.3 m、0.2 m/s、无避障、三项确认沿用临时授权）由用户现场确认可用；配准与绝对测距未用卷尺验收，Pose 实际约 6–7 帧/秒，现场调试 Foxglove 须走网线。跌倒检测处于离线评估阶段。10-10 按车载机位（镜头离地 27 cm、俯仰 15.1°）重拍并人工标注 20 次跌倒、19 个反例动作；高度线索及可选开关的复评见 `docs/跌倒检测评估记录.md`，按代码现状的逻辑见 `docs/跌倒检测整体逻辑.md`。C 尚未在 Jetson 上做真人或跌倒实测，现场项见 `docs/现场实测要求20261008.md`。
- B：Jetson 上 YOLO26s/ByteTrack 有真人在线检测；RGB-D 配准、安装外参和绝对测距精度待验收。
- 语音：正式入口为本地 Qwen3-ASR + Silero VAD + 固定规则 + 本地 TTS，见 `docs/语音入口与回退.md`。
- `scripts/start_robot.sh` 是板上整车配置，会启动底盘、语音和经 motion_guard 的自动跟随（`MOTION_ENABLED=true`、`auto_arm:=true`，三项跟随确认为用户临时授权，不启 N10P）；感知验证只用 `start_project.sh` / `start_c.sh`，文档不得把 `start_robot.sh` 写成不运动。
- 底盘串口已在 Jetson 收发里程计；N10P、motion_guard、导航和云台桥接有软件与本机测试，实车运动、避障和云台硬件均未验收。

## C 路线入口约定

- C 显式 `route:=yolo_pose` / `scripts/start_c.sh`，默认项目仍为 B。C 复用 B 的跟踪/测距/选人契约，使用官方 YOLO26s-pose；禁止静默回退 detect。`TargetState.source=yolo` 保留消费者兼容。
- Gemini 使用独立固定版本驱动及设备自身标定，不继承 Astra S 临时内参。要求明确序列号与标定文件，物理配准和安装外参分别确认；不能只改 frame_id。
- C 单目地面定位为独立 `ground_localizer`：输出脚下接地点，`target_state_ground.source=yolo_ground`，不混入 `yolo` 光学目标或卡尔曼；安装外参与地面平面须分别确认，默认 NOT_READY；身高先验不是实测个体身高。设计和边界见 `docs/方案C实现与验收.md`。
- 骨架/跌倒首版只发布人体状态与可视化，不接语音或运动。相机直立未确认最多疑似；静态躺卧不补报跌倒。新增入口及边界见 `docs/方案C实现与验收.md`。
- 跌倒鲁棒开关 `pose_gap_pause`/`pose_handover_s`/`pose_confirm_lying_only`/`pose3d_knee_fallback` 与无骨架高度线索 `height_fall_enabled`（及其可选项 `height_lost_hold_s`/`height_sit_hold_s`/`height_side_edge_px`，0 为关）默认全关；丢人后继续测量 `height_continue_s` 的延续中人员只发布到单独话题 `person_states_continued`，不得混入 `person_states`（ReID/地面定位只消费真实检测），节点只用最后检出框，VitTrack 延续仅离线回放 `sot`；下游跌倒告警统一读每帧汇总的 `fall_alarm`（FallAlarm），首版仍不接语音/运动；高度线索需重力与地面均确认才可确认，否则最多疑似，只写 detail。默认值须经标注录像 `scripts/evaluate_falls.py` 复评后再定，见 `docs/跌倒检测评估记录.md`。离线回放的相机几何用 `scripts/estimate_camera_geometry.py`（消失点重力 + 站立双踝高度，反光地面不能用深度地面拟合），结果仅诊断，不得据此置任何 `*_confirmed`。

## C 深度融合、定位与三维姿态

- C 默认 `fusion_enabled=true`，统一关联当帧关节和区域真实深度；关节缺失保持 NaN。目标是框中心射线上的虚拟代表点，不是骨盆或地面点，后续地面定位不得直接混入同一观测。`pose.yaml` 中关闭融合可恢复原 C 路径；B 区域测距保持兼容。参数与验收边界见 `docs/方案C实现与验收.md`。
- C 单目地面定位默认要求双踝一致，禁止无接地证据的框底回退。身高先验默认关闭，需绑定已确认的 epoch:track_id 且持续站立；UNKNOWN 不套站立先验。预测发布必须按真实测量年龄复核保持期限。
- C ReID默认关闭；显式REID_ENABLED/REID_MODEL_PATH启用本地OSNet ONNX及校验文件，不自动下载。另需REID_LOCK_ENABLED=true才接入身份锁定恢复：禁用邻框接续，连续新鲜身份确认后才关联当前轨迹，歧义/失效保持LOST，释放清除身份。持久身份与epoch:track_id分离，不转移身高先验或旧轨迹滤波状态。`reid.yaml` 启用 10-11 录像评估得出的同框互斥、已绑定核验门槛、重叠骨架掩码等选项（`IdentityConfig` 默认值仍是原逻辑）；衣着相近的多人场景未验证安全，不得据此开启锁定。入口与测试见docs/方案C身份ReID接入与验收.md。
- C统一定位使用独立 `/perception/person_positions` 与 `target_state_unified`，参考点为双踝中点的地面投影；不把原框中心代表点混入。原躯体深度需先通过同轨迹近期确认的关系换算为脚下参考点，才与脚部观测作保守相关融合；映射有龄期且不跨身份。各来源必须有一致性和新鲜度；未确认安装/地面不发布有效统一位置，身高先验不进入统一输出。
- C三维姿态仅使用真实有效关节；`pose3d_gravity_confirmed` / `pose3d_ground_confirmed` 默认false，依据不足降级二维并标注basis，异常几何未知。二维时序持续更新，三维时序独立保存，短时缺深度暂停确认计时、超时仅清三维候选；不跨像素/米搬用基线。已确认三维事件需三维直立证据恢复。参数及边界见 `docs/方案C统一定位与三维姿态.md`。
- C深度地面拟合（`floor_fit_enabled`，发布`floor_plane`）与动态重力（`pose3d_up_source=floor|tf`）、统一定位实测地面（`floor_plane_enabled`）默认全部关闭；实测地面只作逐帧证据，替代而非叠加标定平面上的单目双踝，按完全相同时间戳匹配，永不自动置任何`*_confirmed`。动态来源缺失时本帧退二维且不复用旧值，横滚超限二维未知，相机转动重置二维像素下坠参考。见`docs/方案C统一定位与三维姿态.md`。

- 单目标跟踪联动（VitTrack + `BridgeArbiter`）默认关闭，`sot_enabled`/`SOT_ENABLED` 显式开启并需本地模型与校验文件；仲裁器只能保持、确认接续或释放目标，不得自行挑人，曾与目标同框的轨迹不得接续，冲突帧暂停输出；与 ReID 锁定合用时只在跟踪器连续一致、无反证、身份可信（用户锁定或 ReID 持续确认 2 秒）且目标未与别人重叠时延续身份（`reid_lock_bridge_carry_s`），冲突只作废授权、由 ReID 自动重新确认。以跟随稳定为目标时用 `reid_reacquire_enabled`（`OwnerSelection`，需 `sot_enabled`，与 ReID 锁定互斥）：联动负责连续性，ReID 只否决和丢失后找回主人；主人身份经 `owner_focus` 固定在 ReID 节点并补充多视角。Jetson 自带 OpenCV 4.5.4 无法加载该模型，需 onnxruntime 或 TensorRT，尚未在板上安装和测速。见 `docs/方案C单目标跟踪联动.md`。

- 跟随运动：`motion_guard` 的 `target_grace_s`（默认 0）只对 FOLLOW 中目标短暂无效/超时生效，宽限内衰减到零并可自动恢复，超时仍 FAULT 锁存，其他故障不享受宽限；`start_project.sh` 跟随入口按用户确认设 0.6 s（`FOLLOW_TARGET_GRACE_S`）。`target_auto_recover`（默认 false）只让目标类 FAULT 在目标重新有效后自动回 ARMED（同方案 A 跟随器），其他故障仍锁存；跟随入口按用户 10-12 确认默认开启（`FOLLOW_TARGET_AUTO_RECOVER`）。跟随距离 `FOLLOW_TARGET_DISTANCE_M`（默认 1.0）。控制律仍为 A 实测过的 `follow_control.compute_command`。不得扩大宽限或自动恢复范围、延长上限而不经用户确认。

## 云台约定

- 相机云台接口见 `docs/云台跟随设计与串口协议.md`（草案，未实现；含坐标系、数据流、云台与底盘运动方法）：云台 yaw 为相对车体关节角、pitch 相对重力，人位置估计在 `odom`；底盘跟随目标位置而非云台角。pan 角无可信来源时不得发布相机到车体 TF，云台转动后不得再用光学系偏角直接转向。跟随距离暂定 1.9 m（后轴起算，整人入画），tilt 以头脚居中为准；控制主输入为躯干深度，脚部深度缺失（深色裤子）时由统一定位退到躯干投影/单目双踝。云台桥接为独立入口 `gimbal_bridge/gimbal.launch.py`，`gimbal_mount.yaml` 未确认时不发布云台 TF，`allow_unconfirmed_mount` 仅限 `mock:=true` 仿真；关节时间戳必须是 MCU 采样时刻换算值。

## Mac C 调试、录制与离线评估

- `.venv/bin/python scripts/preview_pose_mac.py` 打开本机USB相机的框、骨架和二维状态预览，默认camera=0，可显式`--camera`选择；使用本地Pose模型和pose.yaml，不自动下载。它没有同步深度、ROS、ReID或车辆输出，不作为三维/实车验收。按Q或关闭窗口退出。
- `scripts/analyze_gemini_recording.py RECORDING --output OUTPUT`用于Mac采集格式离线回放，默认每6帧推理一次，按采集时间驱动二维规则。主机接收时间最近邻配对仅作诊断；没有曝光同步和安装标定不能声称真实三维姿态/地面验收。
- `.venv/bin/python scripts/charuco_calib_mac.py --square 实测米 --marker 实测米` 用 7×5 DICT_5X5 ChArUco 检查 Gemini 出厂彩色内参（S保存、C对比、Q退出），不含深度配准或安装外参，不能据此设置 `extrinsics_calibrated`。
- `scripts/charuco_mount_mac.py --square --marker --board-x --board-y` 用地面平放 ChArUco 求 `base_footprint`→彩色光学系安装外参；只输出报告，确认开关须在实拍复核后人工修改。
- Mac 录制：在“终端”运行 `.venv/bin/python scripts/record_gemini_mac.py <段名>`（SDK 深度 zlib 无损压缩 + UVC 彩色，主机时间配对非曝光同步；彩色默认 2592×1944 裁剪缩小到 640×480 避开原 640×480 模式偏蓝，`device.json` 的 `color_intrinsic` 为换算值、出厂值存 `factory_color_intrinsic`，换相机须重跑 `scripts/gemini_color.py measure`）；标注 `scripts/label_recording.py <录像目录>`（可先用 `scripts/fall_candidates.py` 预填 `notes=auto` 候选，未确认的不计入评估）。共享读取/汇总在 `scripts/gemini_recording.py`，分析脚本同时支持 `.depth` 与 `.depth.z`。重拍安排与交付格式见 `docs/重拍视频要求20261010.md`；每天开录前用 `--set-camera 高度cm 俯仰° 横滚°` 记录相机架设，各段自动写入 notes.md。
- 本机合成ROS时序测试可使用`tests/monotonic_ros_clock.py`及`use_sim_time=True`，避免Colima校时回拨扰动；正式时间倒退失效保护不得因此关闭，测试须保留显式回拨与超时检查。离线回放`--reid`显式加载本地OSNet，`--cached-poses`仅复用同录像的带置信度Pose结果；汇总入口为`scripts/summarize_recording_validation.py`。

## B 路线约定

- B 管理入口默认 `models/weights/yolo26s-fp16.engine`，由目标 Jetson 导出；缺文件必须报错，不静默回退。显式 `MODEL_PATH=.../yolo26s.pt` 可做基线对比。
- YOLO 三维位置使用独立恒速卡尔曼；`kalman_measurement_std_m` / `kalman_acceleration_std_mps2` 替代 `depth_smoothing_alpha`。预测必须保留真实测量年龄和超时失效。默认时间门控拒绝不一致测量（被拒不刷新测量时间），重定位须一致证据确认后干净重初始化；断流按最后一次输入（含被拒）计时，输入持续但一直被拒超过 `max_age_s` 时丢弃旧状态，新位置仍须确认；无躯干佐证的退路深度门限更严且不能单独起始轨迹。`depth_gate_enabled:=false` 恢复旧跳变重初始化。
- 与语音并行协作时只改 YOLO 相关文件，不重启整套服务；板端验证优先使用隔离目录和独立命名空间。

## 对外接口与语音

- 对外 ROS 2 入口为 `ros2 launch roscar_api api.launch.py`，默认 IDLE/运动关闭/不启硬件。接口手册 `docs/ROS接口使用文档.md`；外部速度只发 `/chassis/cmd_vel`，模式服务 `/control/set_mode`，由唯一 motion_guard 输出 `/cmd_vel`。禁止与已有包含 guard 的组合入口重复启动。检测列表沿用同步 RGB-D，未跟踪框不参与目标锁定。
- V5.1 语音入口保留 V5 的本地控制与 DeepSeek 结构化控制工具，但不再声明或接受蜂鸣器动作；`voice_assistant.launch.py` 不启动蜂鸣器适配节点。独立 `voice_command_router` 包内的蜂鸣器解析与 GPIO 源码保留，历史条目中关于语音蜂鸣器的描述不代表当前入口行为。
- 当前正式 `/home/wheeltec/ROSCAR-current` 的语音入口默认为本地 Qwen3-ASR 0.6B INT8、Silero VAD 连续收音、固定规则控制和本地 TTS；不启动 Ollama。路由器独占语音会话，首条明确运动口令自动请求模式和授权，必须读回 guard 的 `ARMED` 状态才发速度。`CONTINUOUS_ASR=false` 回退旧逐轮本地收音，`ASR_BACKEND=xfyun` 回退讯飞识别。离线依赖在 `/home/wheeltec/ROSCAR-offline`；部署和回退见 `docs/语音入口与回退.md`。

## 测试与 CI

- GitHub Actions `unit-tests` 在 PR 和 main 推送时运行 `scripts/ci_unit_tests.sh`（主动包纯 Python 单测 + 录像工具，Python 3.10，无 ROS/硬件/权重/录像）；本机 `PYTHON=.venv/bin/python scripts/ci_unit_tests.sh` 同口径。通过不等于 colcon 编译、ROS 运行时或实机验证；需要 rclpy 的测试列在脚本 `NEEDS_ROS`。
