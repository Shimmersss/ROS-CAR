# Demo 分支 Codex 任务清单（2026-10-16）

分支 `demo/showcase-20261016`，基于 `codex/onboard-integration-20261013@eac6d9a`，**只为展示，之后删除，不合并 main**。
仍须遵守 AGENTS.md：`motion_guard` 是唯一 `/cmd_vel` 出口；跌倒模块不直接输出运动；不得把单测/合成测试写成实车验收；凭据不入库。每项完成后在 WORKLOG.md 记一条。

已搭好的框架：

| 路径 | 内容 | 状态 |
|---|---|---|
| `ros2_ws/src/demo_interfaces/` | `MoveRelative.action`、`MoveDistance.srv`、`TurnAngle.srv` | Humble 容器编译通过，非实车 |
| `ros2_ws/src/demo_motion/demo_motion/relative_motion.py` | 校验、`/odom` 进度、减速曲线、阿克曼弧线（纯 Python） | 单测 17 项通过 |
| `ros2_ws/src/demo_motion/demo_motion/relative_motion_node.py` | action + 2 个 service + 取消；逐次 EXTERNAL 授权 | 合成 ROS 回归已接入，见下文 |
| `ros2_ws/src/demo_motion/{launch,config}/` | `relative_motion.launch.py`、`relative_motion.yaml` | launch 条件验证；整机入口未上板 |
| `docs/demo/相对运动ROS接口.md` | 接口文档 | — |

## 2026-10-16 续接进度

T1.1–T1.3 已实现：新增 `scripts/test_demo_container.sh` 和合成 ROS 测试、发布者冲突拒绝、语音适配器开关及 `scripts/run_demo.sh`。另加里程计时间戳/四元数校验、action 接收时原子占位、停车确认失败返回 ABORTED。T2.1 的 1.8 m 距离与 T2.7 的 0.35 m/s 仅设为 demo 入口默认，现场效果待验证。T1.4–T1.6、T2.5 尚未完成；T2.6 离线评估已完成，关键点阈值/下沿保留原设置；后续高度确认与坐地误报优化见 [A2a/A4 评估](A2a与A4离线优化.md)；不部署正式目录，不代替现场标定。

T2.2–T2.4 已实现，配置和验证边界见 [跌倒暂停与静止延续验收](跌倒暂停与静止延续验收.md)。与最初设想相比，demo 目标宽限设为 0，避免 guard 衰减旧速度；运动后旧框永久作废，必须静止重新检出才可延续。固定机位录像对比疑似/确认为 18/17 → 19/18，反例误报仍 2/1。录像无里程计，门控仅完成合成 ROS 验证；现场项保持待验收。

---

## T1 相对运动接口（前进/后退 N 米、左/右转 N 度）

- **T1.1 编译与 ROS 运行时测试**：在 Humble 容器/Jetson 上 `bash scripts/build_ros.sh --packages-up-to demo_motion`。写 `tests/test_demo_relative_motion_runtime.py`：启动真实 `motion_guard`（`motion_enabled:=true`，隔离 ROS_DOMAIN_ID）+ 假 `/odom` 积分器（按 guard 输出的 `/cmd_vel` 积分位姿），验证：前进 0.5 m / 后退 0.3 m / 左转 90° / 右转 45° 到达容差内；取消、`/odom` 断流、外部 `/control/stop`、ESTOP 时立刻发零并返回对应 status；结束后 guard 为 IDLE/DISARMED；同时只接受一个动作。
- **T1.2 唯一发布者冲突**：`offline_voice.launch.py` 增加 `with_motion_adapter`（默认 true，主线行为不变）；demo 入口传 false。若 `motion_adapter` 仍在运行，节点启动时检测 `/chassis/cmd_vel` 发布者数 > 1 应直接报错拒绝。
- **T1.3 Demo 一键入口** `scripts/run_demo.sh`：基于 `scripts/run_robot_onboard.sh`，加 `WITH_DEMO_MOTION=true` 启动本节点、关闭语音运动适配器；默认仍运动关闭，需要显式 `MOTION_ENABLED=true`。
- **T1.4（可选）跟随中插入动作**：参数 `restore_follow`：若调用前是 FOLLOW，结束后 `set_mode FOLLOW`。注意 guard 的 `auto_arm` 只在启动时生效，需确认是否要重新授权；没确认前保持“结束回 IDLE”。
- **T1.5（可选）语音口令**：固定规则里的“前进一米/左转九十度”可改为调用本 action（仍走 `parse_intent`，不让 LLM 生成运动）。
- **T1.6 现场标定**：实测 `min_turn_radius_m`、负线速度下转向符号（倒车转弯方向）、`/odom` 距离误差（卷尺 1 m）、角度误差（地面标记 90°）。结果写入 yaml 和本清单。

验收（实车，需用户逐次授权并清场）：1 m 前进误差 ≤ 5 cm，90° 转向误差 ≤ 5°，取消 / 急停按钮生效。

---

## T2 固定机位跟随与跌倒优化（“跌倒后看不到人”）

### 2.1 原因分析（固定机位：离地 27 cm、上仰 15.1°，Gemini 彩色 fy≈452、640×480）

| 量 | 数值 |
|---|---|
| 垂直视场 | ≈ 55.9°：视线上沿高于水平 43.2°，下沿低于水平 12.6° |
| 地面最近可见距离 | **≈ 1.20 m**（更近的地面在画面下沿之外） |
| 1.70 m 人全身入画的最近距离 | **≈ 1.52 m** |
| 当前跟随距离 1.3 m 时 | 1.70 m 的人头顶在画面外约 48 px；脚底在第 471 行（距下沿 9 px） |
| 1.3 m 处躺地的人（身体顶 0.25 m） | 只占第 372–471 行，是贴着画面下沿的一条窄带 |

结论：

1. **1.3 m 跟随距离太近**：站立时头已出画；倒地后人落在 0.3–1.3 m 的地面上，大部分低于画面下沿，YOLO 骨架基本检测不到，`height_continue_s=0`（固定机位为防止运动中误用旧框而关闭）时丢人后不再有任何测量，只能靠 `lost_hold_s` 等待重新检出。
2. **跌倒瞬间底盘仍在跟随**：目标框突然下坠、深度跳变，0.6 s 宽限后 FAULT，而 `target_auto_recover` 在目标重新有效后会**自动续跟**——如果此时检出了躺地的人，会朝倒地者开过去，人更进一步出画，也有碰撞风险。
3. 主人 5 秒丢失期限到期后会清掉主人身份，倒地者重新出现时 ReID 外观（躺姿）很难匹配。

### 2.2 任务

- **T2.1 加大跟随距离（配置即可，先做）**：demo 入口 `FOLLOW_TARGET_DISTANCE_M=1.8`（全身入画，且倒地者在 ≥1.2 m 时仍在画面内）。现场检查 1.8 m 时测距深度、ReID 稳定性和 5–7 FPS 下的响应。
- **T2.2 跌倒即暂停跟随**：`person_follower_node` 增加 `pause_on_fall`（默认 false，demo 开）：订阅 `/perception/fall_alarm`，新鲜 SUSPECTED/CONFIRMED 时只发零请求、不触发自动续跟；告警恢复为 NONE 持续 `fall_resume_s`（默认 3 s）且目标有效才恢复，或由显式服务 `/demo/resume_follow` 恢复。这是“减少运动”，不是跌倒模块输出运动；guard 逻辑不改。
- **T2.3 静止门控的延续测量**：`pose_node` 增加 `height_continue_requires_stationary`（默认 false）：订阅 `/odom`，只有底盘连续静止 ≥0.5 s（|v|<0.02 m/s、|ω|<0.05 rad/s、`/odom` 新鲜）时才允许 `height_continue_s` 的延续框测量；运动或 `/odom` 不新鲜立即停止延续。之后 demo 配置 `pose_fixed_27cm_demo.yaml` 打开 `height_continue_s: 6.0`。配合 T2.2，跌倒后车停住，倒地者仍能在原位继续测量直到确认。
- **T2.4 主人身份冻结**：跌倒告警活跃期间暂停 `owner_lost_timeout_s` 计时（`OwnerLifecycle` 增加 hold 接口），避免 5 s 后清掉主人；告警解除后从剩余时间继续。
- **T2.5（可选）后退找人**：新节点 `demo_fall_response`（默认关闭）：跌倒告警且 `person_states` 中已无该人（只剩 `person_states_continued` 或无）且底盘静止 → 调用一次 `/demo/move_relative` 后退 0.4–0.6 m，让近处地面进入画面，再播报“检测到跌倒，请查看”。只调用一次、无雷达时须在 README 中写明后方清场；不得重复后退或循环。
- **T2.6 下沿截断的检测（离线评估完成）**：复评低 `pose_confidence`（0.5→0.35）在 10-10 标注录像上的误报；框底贴下沿时 `body_heights` 的处理（类似 `height_top_edge_px` 的下沿规则）。用 `scripts/replay_falls.py` + `scripts/evaluate_falls.py` 给出疑似/确认/误报数字，再决定是否写进 demo 配置。六组配对结果均为疑似 19/20、确认 18/20；15 px 下沿候选将误报从 2/1 增至 3/2，其他候选无改善，保留 0.5/无下沿规则。详见 [评估记录](T2.6离线检测优化评估.md)。后续 A2a/A4 优化采用 demo 下降门限 0.6 m、仅确认阶段上限 0.75 m，达到 19/19 检出、误报 1/1；仍有一次坐撑跌倒漏检和一次主动躺下误报，见 [后续评估](A2a与A4离线优化.md)。
- **T2.7 跟随平滑（展示效果）**：当前 0.5 m/s、增益 1.0/1.6 有摆动风险。demo 可设 `FOLLOW_MAX_LINEAR_MPS=0.35`，在现场对比摆动后定值。

### 2.3 验收

- 离线：10-10 录像（27 cm/15.1°）上 T2.3+T2.6 的疑似/确认召回与误报，与当前 18/17、2/1 对比。
- 合成 ROS：跌倒告警 → 跟随器零请求、不自动续跟；静止门控下延续测量开/关；运动中延续立即停止。
- 现场（用户授权）：1.8 m 跟随下侧向、背向、正面朝车三种跌倒各 3 次，记录告警延迟和小车是否停住。正面朝车倒地若仍看不到，属于 T2.5 范围。

---

## T3 收尾

- WORKLOG.md 每项一条；README 文档索引已指向 `docs/demo/`。
- 不部署到 `/home/wheeltec/ROSCAR-current`；板端用独立目录（如 `/home/wheeltec/ROSCAR-demo`），切换前停 `roscar-robot.service`，展示后恢复。
- 分支删除时连同 `demo_interfaces`、`demo_motion`、`docs/demo/` 一起删除。
