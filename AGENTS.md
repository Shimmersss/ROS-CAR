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
- Jetson 已有整机目录 `/home/wheeltec/ROSCAR-current`（实时服务状态须现场核对，语音舌苔试运行副本见下文），离线语音依赖在 `/home/wheeltec/ROSCAR-offline`。Mac 同步脚本默认 dry-run、按白名单，不传厂商包、权重、录像、构建产物或代理配置，不执行远端删除。旧 A 的 `roscar-route-a.service` 若在运行会占用相机，启动其他相机入口前先停止。
- 只有显式 `route:=demo` 输出 `is_simulated=true` 的模拟数据；demo、容器编译、CI 单测、合成测试和离线录像回放都不能描述为实机识别或实车验收。感知验收中不得混入跟随或底盘控制节点。
- 流程图位于 `docs/diagrams/`，必须区分“已有源码/库”“待开发/接入”“待实机验证”；修改 SVG 后如需分享合并图，同步重新导出 PNG。

## 当前状态（2026-10-12，本地合并）

- C：已部署到板上隔离目录 `/home/wheeltec/ROSCAR-c`（Pose FP16 engine、Gemini 驱动均为板上构建），10-12 真人二维骨架与实车低速跟随（1.3 m、0.2 m/s、无避障、三项确认沿用临时授权）由用户现场确认可用；配准与绝对测距未用卷尺验收，Pose 实际约 6–7 帧/秒，现场调试 Foxglove 须走网线。跌倒检测处于离线评估阶段。10-10 按车载机位（镜头离地 27 cm、俯仰 15.1°）重拍并人工标注 20 次跌倒、19 个反例动作；高度线索及可选开关的复评见 `docs/跌倒检测评估记录.md`，按代码现状的逻辑见 `docs/跌倒检测整体逻辑.md`。合并后的 C 尚未实车复验，跌倒尚未在 Jetson 上做真人实测，现场项见 `docs/现场实测要求20261008.md`。
- B：Jetson 上 YOLO26s/ByteTrack 有真人在线检测；RGB-D 配准、安装外参和绝对测距精度待验收。
- 语音：正式入口为本地 Qwen3-ASR + Silero VAD + 固定规则 + 本地 TTS，见 `docs/语音入口与回退.md`。
- `scripts/start_robot.sh` 启动底盘、语音和受保护跟随器，默认 `MOTION_ENABLED=false`、`auto_arm:=false`，几何确认默认 false，不启 N10P。显式打开运动能力后仍须逐次授权；本轮仅本地合并，没有更新板端服务。
- PR #19 已于 10-13 合并到 main（`3b40939`）。板端 `ROSCAR-c` 停在 `57a6fb7`，不含急停通道故障不被自动恢复的修复 `5dd75b1`；开运动跟随前必须先同步并重新构建。
- 底盘串口已在 Jetson 收发里程计；N10P、motion_guard、导航和云台桥接有软件与本机测试，实车运动、避障和云台硬件均未验收。

## 当前语音开发约定（2026-10-12，运动关闭试运行）

- 实机试运行入口为 `scripts/run_voice_tongue_trial.sh`，由手动用户服务 `roscar-voice-tongue-trial.service` 监督，固定运动关闭且不自动授权；不能叠加旧整车服务。合并后试运行默认使用 Gemini 独立驱动，必须指定 GEMINI_SERIAL / GEMINI_CALIBRATION；旧 UVC 仅显式 TRIAL_CAMERA_BACKEND=astra 兼容。口头唤醒词“小微小微”与内部“小车唤醒”事件须区分。部署路径、实际验证范围和待验收项见 `docs/2026-10-12语音舌苔实机试运行.md`。

- 以 `docs/语音优化无车实施与验收.md` 为本轮入口和验收边界。没有小车时只做本机软件/模拟/数字音频回放，不宣称板端收音、AEC、制动或物理运动通过。
- 固定规则运行链使用 `parse_intent -> MotionRequest -> Capability Validator/Ackermann adapter`；旧 `parse_local_text` 仅供兼容文本回归。禁止 LLM 生成底盘运动控制。
- `motion_guard` 是唯一最终 `/cmd_vel` 发布者和运动授权来源。默认启动 IDLE/DISARMED、不自动授权；显式授权必须绑定当前模式返回的 epoch，且有新鲜零请求与 KWS 健康心跳。停止/故障/模式切换/重启使旧票据失效。例外（用户 10-12 合并 PR #18 时确认保留）：跟随入口可 `command_mode=FOLLOW` 启动并 `auto_arm`，仅从 DISARMED+FOLLOW 且全部检查通过时授权；STOP 回 IDLE 后不会被自动授权撤销，ESTOP/急停通道/非目标故障仍按锁存与显式复位处理。
- ESTOP 锁存持久化，普通 STOP/模式切换不能清除；复位仅回到未授权状态。KWS 常开，包括 TTS 期间；共享录音只允许一个 ALSA 所有者。
- Qwen3-ASR CPU2 保留；用户 2026-10-14 选择 Matcha 中文女声，源码默认 TTS 改为 Matcha-baker + Vocos、CPU 两线程，仍复用唯一队列与原 ROS 接口。模型缺失须明确失败，不静默恢复旧 Melo；历史数字音频工具仍可显式准备 Melo 权重。热词、阿克曼转向、微动标定及总运动开关默认关闭，未经真人实车验收不得默认开启。测试中的校准/免 KWS 参数只用于隔离模拟域。
- 主入口 `VOICE_BACKEND=offline`；讯飞 ASR 仅通过显式 `ASR_BACKEND=xfyun` 回退，不能通过旧 online 路径恢复 LLM 控制。

- ASR 运行开关为 `/voice/set_asr_enabled`（SetBool），由轻量 asr_manager 管理独立识别进程；关闭成功须确认进程退出，释放 ASR/VAD 及原生会话内存。TTS、共享录音和 KWS 原配置独立保留；关闭清普通会话，重新开启须重新唤醒，不自动加载崩溃进程或恢复旧动作。完整入口使用 manager，直接 `asr_node` 仅保留兼容/隔离测试。`VOICE_ASR_ENABLED`/`asr_enabled` 只设置启动初值，运行时状态及 PID/RSS 见 `/voice/asr_status`。

## 语音与舌苔执行模式整合约定（2026-10-04）

- 新分支以 `docs/语音舌苔流程整合与验收.md` 为整合入口；原舌苔模块来自 `codex/tongue-execution-mode` / `1b422eb`，来源分支的旧验证记录不作为本分支验收。
- 舌苔流程复用当前 ASR、共享录音、常开 KWS、唯一 TTS 队列和原始 RGB；仅新增 execution_mode_interfaces / execution_mode_manager / tongue_mode，不建立新的最终速度出口。
- 模式进入先确认 STOP，再获得 TaskHold；保持期间禁止运动授权。急停/故障/急停健康失效终止检测，释放租约不能清除锁存或恢复运动。
- 保留唤醒刷新与 8 秒普通会话，超时不取消检测任务；READY 只为语音交互开放一次有界续聊窗口。明确退出语音、停车和急停清理模式。检测中的唤醒不取消有完成确认的任务播报。
- 整机入口仅在 `WITH_TONGUE_MODE=true` 时接入；robot 要求底盘静止回传，bench 必须运动关闭。测试中的假输入/模型不能称为实车或舌象分类准确率验证。
- 模式切换等待旧 RunMode 的最终清理，播报顺序使用 SpeakText 完成确认；云端冻结版本与八项多标签结果契约必须验证，图像及凭据不提交 Git。

## 文字播报接口约定

- 文字播报公共服务为 `/voice/say`（`SayText`），独立入口 `ros2 launch roscar_api speech_api.launch.py` 默认仅复用现有 `/voice/speak` action。正文只交给唯一 TTS 队列，不经过意图/运动解析；超时仅取消本次 goal。只有没有现有 TTS 时才使用 `with_tts:=true`，避免重复播放所有者。B/C 总入口 `start_project.sh` 在语音与 TTS 启用时自动开启 SpeakText action 和 `/voice/say` 桥接，复用唯一队列。调用说明见 `docs/文字播报ROS接口.md`。
- B/C 总入口同时启动主人/跟随阶段观察器，按状态变化调用 `/voice/say`，心跳不重复播；舌苔阶段使用同一队列的 `/voice/speak` 完成确认，READY 提示播放完成后才开放续聊。跌倒播报消费新鲜 `/perception/fall_alarm` 上升沿，疑似/确认分别简短提醒；无效帧与编号交接不重播，新鲜 NONE 持续 2 秒才重新开放。不由播报器控制运动。

## C 路线入口约定

- C 显式 `route:=yolo_pose` / `scripts/start_c.sh`，默认项目仍为 B。C 复用 B 的跟踪/测距/选人契约，使用官方 YOLO26s-pose；禁止静默回退 detect。`TargetState.source=yolo` 保留消费者兼容。
- Gemini 使用独立固定版本驱动及设备自身标定，不继承 Astra S 临时内参。要求明确序列号与标定文件，物理配准和安装外参分别确认；不能只改 frame_id。
- C 可选 `GEMINI_INPROCESS_REGISTRATION=true` 把 Gemini 配准并入 Pose 进程（默认 false）；按已完成推理发布同帧校正 RGB 到 `/camera/color/image_rect`，保留原采集时间戳，供 ReID 精确配对，不受可视化帧率开关影响。可与 `REID_ENABLED` 同用，不能同时启动独立配准发布者；显式 `GEMINI_REGISTRATION_BACKEND=cuda` 可用 Pose Python 的 PyTorch/CUDA 做 float64 投影与 z-buffer，默认 numpy，缺 CUDA 拒绝。`PERCEPTION_AUXILIARY=false` 可移出地面/统一定位及坐标转换（默认 true），ReID 已启用时仍保留。板上帧率用只读 `scripts/measure_c_rates.py` 测量，未经对比不得设为默认。
- C 单目地面定位为独立 `ground_localizer`：输出脚下接地点，`target_state_ground.source=yolo_ground`，不混入 `yolo` 光学目标或卡尔曼；安装外参与地面平面须分别确认，默认 NOT_READY；身高先验不是实测个体身高。设计和边界见 `docs/方案C实现与验收.md`。
- 骨架/跌倒发布人体状态与可视化，独立阶段观察器接语音，不由跌倒模块输出运动。相机直立未确认最多疑似；静态躺卧不补报跌倒。新增入口及边界见 `docs/方案C实现与验收.md`。
- 跌倒鲁棒开关 `pose_gap_pause`/`pose_handover_s`/`pose_confirm_lying_only`/`pose3d_knee_fallback` 与无骨架高度线索 `height_fall_enabled`（及其可选项 `height_lost_hold_s`/`height_sit_hold_s`/`height_side_edge_px`，0 为关）默认全关；丢人后继续测量 `height_continue_s` 的延续中人员只发布到单独话题 `person_states_continued`，不得混入 `person_states`（ReID/地面定位只消费真实检测），节点只用最后检出框，VitTrack 延续仅离线回放 `sot`；下游跌倒告警统一读每帧汇总的 `fall_alarm`（FallAlarm），阶段观察器可接语音，不接运动；高度线索需重力与地面均确认才可确认，否则最多疑似，只写 detail。默认值须经标注录像 `scripts/evaluate_falls.py` 复评后再定，见 `docs/跌倒检测评估记录.md`。离线回放的相机几何用 `scripts/estimate_camera_geometry.py`（消失点重力 + 站立双踝高度，反光地面不能用深度地面拟合），结果仅诊断，不得据此置任何 `*_confirmed`。

## C 深度融合、定位与三维姿态

- C 默认 `fusion_enabled=true`，统一关联当帧关节和区域真实深度；关节缺失保持 NaN。目标是框中心射线上的虚拟代表点，不是骨盆或地面点，后续地面定位不得直接混入同一观测。`pose.yaml` 中关闭融合可恢复原 C 路径；B 区域测距保持兼容。参数与验收边界见 `docs/方案C实现与验收.md`。
- C 单目地面定位默认要求双踝一致，禁止无接地证据的框底回退。身高先验默认关闭，需绑定已确认的 epoch:track_id 且持续站立；UNKNOWN 不套站立先验。预测发布必须按真实测量年龄复核保持期限。
- C ReID默认关闭；显式REID_ENABLED/REID_MODEL_PATH启用本地OSNet ONNX及校验文件，不自动下载。另需REID_LOCK_ENABLED=true才接入身份锁定恢复：禁用邻框接续，连续新鲜身份确认后才关联当前轨迹，歧义/失效保持LOST，释放清除身份。持久身份与epoch:track_id分离，不转移身高先验或旧轨迹滤波状态。`reid.yaml` 启用 10-11 录像评估得出的同框互斥、已绑定核验门槛、重叠骨架掩码等选项（`IdentityConfig` 默认值仍是原逻辑）；衣着相近的多人场景未验证安全，不得据此开启锁定。入口与测试见docs/方案C身份ReID接入与验收.md。
- C统一定位使用独立 `/perception/person_positions` 与 `target_state_unified`，参考点为双踝中点的地面投影；不把原框中心代表点混入。原躯体深度需先通过同轨迹近期确认的关系换算为脚下参考点，才与脚部观测作保守相关融合；映射有龄期且不跨身份。各来源必须有一致性和新鲜度；未确认安装/地面不发布有效统一位置，身高先验不进入统一输出。
- C三维姿态仅使用真实有效关节；`pose3d_gravity_confirmed` / `pose3d_ground_confirmed` 默认false，依据不足降级二维并标注basis，异常几何未知。二维时序持续更新，三维时序独立保存，短时缺深度暂停确认计时、超时仅清三维候选；不跨像素/米搬用基线。已确认三维事件需三维直立证据恢复。参数及边界见 `docs/方案C统一定位与三维姿态.md`。
- C深度地面拟合（`floor_fit_enabled`，发布`floor_plane`）与动态重力（`pose3d_up_source=floor|tf`）、统一定位实测地面（`floor_plane_enabled`）默认全部关闭；实测地面只作逐帧证据，替代而非叠加标定平面上的单目双踝，按完全相同时间戳匹配，永不自动置任何`*_confirmed`。动态来源缺失时本帧退二维且不复用旧值，横滚超限二维未知，相机转动重置二维像素下坠参考。见`docs/方案C统一定位与三维姿态.md`。

- 单目标跟踪联动（VitTrack + `BridgeArbiter`）默认关闭，`sot_enabled`/`SOT_ENABLED` 显式开启并需本地模型与校验文件；仲裁器只能保持、确认接续或释放目标，不得自行挑人，曾与目标同框的轨迹不得接续，冲突帧暂停输出；与 ReID 锁定合用时只在跟踪器连续一致、无反证、身份可信（用户锁定或 ReID 持续确认 2 秒）且目标未与别人重叠时延续身份（`reid_lock_bridge_carry_s`），冲突只作废授权、由 ReID 自动重新确认。以跟随稳定为目标时用 `reid_reacquire_enabled`（`OwnerSelection`，需 `sot_enabled`，与 ReID 锁定互斥）：联动负责连续性，ReID 只否决和丢失后找回主人；主人身份经 `owner_focus` 固定在 ReID 节点并补充多视角。Jetson 自带 OpenCV 4.5.4 无法加载该模型，需 onnxruntime 或 TensorRT；板端显式复用 Pose Python 并安装 onnxruntime，ReID 可用目标板构建且验证来源清单的 OSNet TensorRT engine，不能静默回退。见 `docs/方案C单目标跟踪联动.md`。

- 跟随运动：`motion_guard` 的 `target_grace_s`（默认 0）只对 FOLLOW 中目标短暂无效/超时生效，宽限内衰减到零并可自动恢复，超时仍 FAULT 锁存，其他故障不享受宽限；`start_project.sh` 跟随入口按用户确认设 0.6 s（`FOLLOW_TARGET_GRACE_S`）。`target_auto_recover`（默认 false）只让目标类 FAULT 经 `MotionAuthority.recover_follow()` 在目标重新有效后自动回 ARMED_FOLLOW（同方案 A 跟随器），其他故障仍锁存；跟随入口 `FOLLOW_AUTO_ARM`/`FOLLOW_TARGET_AUTO_RECOVER` 默认随 `MOTION_ENABLED`（用户 10-12 确认）。`FOLLOW_EMERGENCY_KWS_REQUIRED` 默认 true，无语音运行跟随须显式置 false。跟随距离 `FOLLOW_TARGET_DISTANCE_M`（默认 1.0）。控制律仍为 A 实测过的 `follow_control.compute_command`。不得扩大宽限或自动恢复范围、延长上限而不经用户确认。

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
- 主动构建和 B/C 运行统一使用仓库根目录 `install/`；API 可选启动语音、舌苔及云台，导航保留独立且互斥的 guard 入口。`roscar_api interfaces` 枚举运行中接口，`target claim --id epoch:track_id` 和 `mode` 提供认领与模式调用。
- 主人生命周期由 `/perception/owner_state` 发布，`/perception/claim_owner` 支持中央/精确轨迹认领。丢失默认等待 10 秒，期限内找回保留主人，到期清除旧绑定和联动代次并开放重新认领；仅原先启用的单人自动认领策略恢复三帧确认，主动释放仍抑制自动认领。这个窗口不得延长 motion_guard 的目标运动宽限或直接授予运动权限。主选择 LOST/陈旧/冲突时，地面及统一定位的选中输出也必须无效。
- V5.1 语音入口保留 V5 的本地控制与 DeepSeek 结构化控制工具，但不再声明或接受蜂鸣器动作；`voice_assistant.launch.py` 不启动蜂鸣器适配节点。独立 `voice_command_router` 包内的蜂鸣器解析与 GPIO 源码保留，历史条目中关于语音蜂鸣器的描述不代表当前入口行为。
- 本地源码采用共享录音 + Qwen3-ASR / Silero VAD + 固定语义规则 + 常开 KWS + 唯一 Matcha 中文女声/Vocos；8 秒有界续聊，运动显式绑定 epoch 授权，不能由方向口令自动授权。ASR_BACKEND=xfyun 只替换识别，仍消费共享音频。远端旧 continuous_asr 实现不合入，CONTINUOUS_ASR 不切换录音方案。本地 C 新鲜 FallAlarm 可触发阶段语音，不触发底盘动作。

## 测试与 CI

- GitHub Actions `unit-tests` 在 PR 和 main 推送时运行 `scripts/ci_unit_tests.sh`（主动包纯 Python 单测 + 录像工具，Python 3.10，无 ROS/硬件/权重/录像）；本机 `PYTHON=.venv/bin/python scripts/ci_unit_tests.sh` 同口径。通过不等于 colcon 编译、ROS 运行时或实机验证；需要 rclpy 的测试列在脚本 `NEEDS_ROS`。
- `scripts/test_flow_regressions.sh` 在已构建的 Humble overlay 中运行五项流程回归、真实 10 秒主人等待、播报、公共 API 与舌苔合成 ROS 验证；仍不等于实机播放或运动验收。


## 独立云端拍照任务

- `capture_task` / `capture_task_interfaces` 为独立拍照上传链路，见 `docs/云端拍照任务实现与验收.md` 与协议。不经过 ASR/router/manager，不依赖 tongue_mode，不推理、不发布速度、不授权或清急停；只共享原始 RGB、唯一 SpeakText、robot 下的 TaskHold。
- `WITH_CAPTURE_TASKS=false` 默认关闭；接底盘强制 robot，须显式 WITH_FOLLOWER=true 复用唯一 guard（该入口同时启动跟随器，不自动打开跟随）；bench 要求 /cmd_vel 零发布者。robot 始终停车租约+新鲜静止回传；当前guard拒绝ESTOP下申请，报hold_failed而不改guard。语音舌苔 active 时拒绝云端拍照，执行中也监测互斥。两个bench入口因guard要求不同不能并开。
- 播报须完成确认且禁含紧急/RESERVED_STOP词；采集须晚于播报及准备并校验帧龄。JPEG/凭据不提交Git，图像默认不在执行器落盘；mock调试图像位于忽略的artifacts。fake/synthetic仅显式隔离本机测试；Humble x86_64与相机采集不能描述为Jetson或小车验收。
- 本机usb_cam0.8.1时间戳修正仅通过 scripts/build_capture_usb_cam.sh 的固定源码独立overlay，不改系统驱动或车端相机标定。车端部署和push须用户另行授权。

- ROS拍照门面只转发给 `/capture_task/run`，`with_ros_api=false` / `CAPTURE_ROS_API=false` 默认关闭，不改既有action、bridge及HTTP字段；取消仅针对门面自身goal。ROS任务上传带 `X-Task-Source=ros_api`，不经bridge产生云端ack/events；`/capture/tongue`只拍照上传，与语音tongue_mode独立。

- 拍照整机兼容约定：`WITH_CAPTURE_TASKS=true` 时强制 `FOLLOW_AUTO_ARM=false`、`FOLLOW_TARGET_AUTO_RECOVER=false`、`FOLLOW_EMERGENCY_KWS_REQUIRED=true`，任务释放租约不自动续跟；拍照关闭时保留远端 C 跟随入口配置。

## 固定机位开机入口

- 正式板端入口 `/home/wheeltec/ROSCAR-current` 当前指向 `/home/wheeltec/ROSCAR-matcha-20261014-d5f2096`，切换前 current 保留备份。10-14 语音试运行通过用户服务 drop-in 强制运动、自动授权和自动续跟关闭，私有 env 原值保留；以下开机跟随配置描述不代表当前已恢复运动。验证及回退见 `docs/2026-10-14Matcha语音部署与验证.md`。用户服务从 current 启动，底盘使用该部署独立重编译的 overlay。串口显式上限可至 0.5 m/s、默认仍 0.2；跟随器/guard/串口三层同限，角速度仍 0.5。速度和增益配置不放宽感知龄期、运动宽限或急停。

- `scripts/run_robot_onboard.sh` / `deploy/systemd/roscar-robot.service` 是显式板端 C 入口，默认参数仍运动关闭。板端私有 env 按用户授权开机 FOLLOW 自动授权，并沿用 1.3 m、无避障；用户最新明确选择最高 0.5 m/s（私有 FOLLOW_MAX_LINEAR_MPS），距离/转向增益 1.0/1.6；三项确认仅表示本次临时授权，不作为正式标定验收。启动器不清 ESTOP，进程故障不自动重启授权。见 `docs/2026-10-13板端联调与开机跟随.md`。
- 当前固定机位入口按用户最新确认重新开启 ReID 找回（REID_ENABLED / REID_REACQUIRE_ENABLED=true，锁定 false）：首次连续丢失 2 秒清轨迹/VitTrack，保留主人身份至同一次丢失 5 秒；期限内通过新鲜身份确认返回，5 秒到期清主人及该身份外观库并开放单人三帧新认领。通用默认仍十秒、连续性定时重置关闭；不改 0.6 秒运动宽限或非目标故障锁存。
- 固定 27 cm / 上仰 15.1° / 横滚 0.5° 使用显式 `pose_fixed_27cm.yaml`；通用 pose.yaml 默认不变。身体高度线索无需头脚全入画；运动时关闭旧像素框延续。安装变化或非平坦地面须重新确认；已有标注录像复评不等于现场跌倒验收。

- 用户在当前固定机位主链明确关闭语音自动急停：VOICE_ESTOP_ENABLED=false，不启动独立 KWS，FOLLOW_EMERGENCY_KWS_REQUIRED 随此开关关闭；ASR/VoiceCommand 的 ESTOP 转普通 STOP，不写急停锁存。通用语音入口默认仍开启；显式 /control/estop 接口保留，目标/测距/请求时效与串口看门狗仍停车。

## 拍照推理闭环

- 独立 services/inference_gateway 只运行 5060 端推理，不启动 ROS 或控制运动；冻结舌诊 passthrough_debug 的 digest/八标签不得修改；GPU/CPU 数值比较未达指南要求须报告并保留失败记录；用户2026-10-16已明确授权舌诊继续使用GPU，该例外不代表数值对齐或准确率验收通过，不改冻结阈值/配置，不默认切CPU。第三方 .pt 仅在已确认可信来源的隔离环境加载。
- capture_task 默认 pipeline_mode=upload；infer 必须显式配置 inference_url，API 仅注册 /inspect/*，upload 仅 /capture/*，同进程禁止混用。Action 新增 JSON/JPEG 字段后消费者须整体重编译。
- 结果须核对请求/任务/图像 ID、结构化字段、JPEG 尺寸与哈希；播报由本地固定模板生成，不接受任意模型文本。默认只返回标注图，原图传输须显式选择，不默认落盘；独立有界图片队列不阻塞命令轮询。指南 B4 车端部署须另行授权，不改主服务。
