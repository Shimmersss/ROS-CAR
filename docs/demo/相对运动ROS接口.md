# Demo：相对运动 ROS 接口（前进/后退 N 米、左转/右转 N 度）

> 仅 `demo/showcase-20261016` 分支，面向展示，分支删除时一并删除。不改变主线的 `/chassis/cmd_vel`、`motion_guard` 或语音运动约定。
> 状态（2026-10-16）：接口定义、纯 Python 核心与 ROS 节点已写好；纯 Python 单测 17 项通过。**colcon 编译、ROS 运行时、实车均未验证**，见 [Codex 任务清单](Codex任务清单.md) T1。

## 1. 接口一览

| 名称 | 类型 | 用途 |
|---|---|---|
| `/demo/move_relative` | action `demo_interfaces/action/MoveRelative` | 通用入口，带进度反馈、可取消 |
| `/demo/move_distance` | service `demo_interfaces/srv/MoveDistance` | 前进（+）/后退（−）N 米，阻塞到结束 |
| `/demo/turn_angle` | service `demo_interfaces/srv/TurnAngle` | 左转（+）/右转（−）N 度，阻塞到结束 |
| `/demo/motion_cancel` | service `std_srvs/srv/Trigger` | 取消当前动作（立即发零并停车） |
| `/demo/motion_state` | topic `std_msgs/msg/String`（JSON） | `phase`、已走距离/已转角度 |

节点：`ros2 launch demo_motion relative_motion.launch.py`（可执行文件 `relative_motion`，节点名 `demo_relative_motion`）。

## 2. 字段

**MoveRelative.action**

| 部分 | 字段 | 说明 |
|---|---|---|
| goal | `kind` | `MOVE` 或 `TURN` |
| | `distance_m` | MOVE：+ 前进 / − 后退（米）。TURN：< 0 表示倒车转弯，数值本身忽略 |
| | `angle_deg` | TURN：+ 左转 / − 右转（度）；MOVE 必须为 0 |
| | `speed` | 0 = 默认。MOVE 为 m/s，TURN 为偏航角速度 rad/s |
| | `timeout_s` | 0 = 自动（名义时间 ×2 + 3 s） |
| result | `success`、`status`、`message` | status：SUCCEEDED / REJECTED / ABORTED / CANCELED / TIMEOUT |
| | `travelled_m`、`turned_deg` | 由 `/odom` 实测：沿起始朝向的位移、偏航变化（带符号） |
| feedback | `phase`、`travelled_m`、`turned_deg`、`remaining` | phase：AUTHORIZING / MOVING；remaining 为剩余米或度 |

**MoveDistance.srv**：请求 `distance_m`、`speed_mps`；响应 `success / status / message / travelled_m`。
**TurnAngle.srv**：请求 `angle_deg`、`yaw_rate_rps`、`reverse`；响应 `success / status / message / turned_deg / travelled_m`。

## 3. 调用示例

```bash
# 前进 0.5 m
ros2 service call /demo/move_distance demo_interfaces/srv/MoveDistance "{distance_m: 0.5, speed_mps: 0.0}"
# 后退 0.3 m，0.1 m/s
ros2 service call /demo/move_distance demo_interfaces/srv/MoveDistance "{distance_m: -0.3, speed_mps: 0.1}"
# 左转 90°（阿克曼：边前进边转弯）
ros2 service call /demo/turn_angle demo_interfaces/srv/TurnAngle "{angle_deg: 90.0, yaw_rate_rps: 0.0, reverse: false}"
# 右转 45°，倒车转弯
ros2 service call /demo/turn_angle demo_interfaces/srv/TurnAngle "{angle_deg: -45.0, yaw_rate_rps: 0.0, reverse: true}"
# action 形式，带反馈
ros2 action send_goal --feedback /demo/move_relative demo_interfaces/action/MoveRelative \
  "{kind: MOVE, distance_m: 1.0, angle_deg: 0.0, speed: 0.0, timeout_s: 0.0}"
# 中途取消
ros2 service call /demo/motion_cancel std_srvs/srv/Trigger '{}'
```

## 4. 执行流程与安全边界

1. 拒绝条件：已有动作在执行、`/odom` 或 `/control/state` 不新鲜、guard 处于 ESTOP/FAULT、参数越界（见下表）。
2. `authorize_per_goal=true`（默认）：每个请求视为一次明确的操作员授权——`/control/set_mode EXTERNAL` 取得 epoch → 发 0.3 s 新鲜零请求 → `/control/authorize`（绑定该 epoch）→ 确认 ARMED_MANUAL。`false` 时要求外部已完成授权。
3. 20 Hz 闭环：按 `/odom` 计算剩余量，末段 0.15 m / 20° 线性减速，到达容差（3 cm / 3°）即停。
4. 中止：取消、`/odom` 超时、授权丢失或 epoch 变化、超时 → 立即发零。
5. 结束：发零 0.5 s 后调用 `/control/stop`，guard 回 **IDLE、撤销授权**。不会自动恢复 FOLLOW（T1.4 可选）。
6. `motion_guard` 仍是唯一 `/cmd_vel` 发布者，其 `motion_enabled`、限速、请求时效、急停照常生效；guard 的限速比本节点更严时以 guard 为准。

| 参数（`config/relative_motion.yaml`） | 默认 | 说明 |
|---|---|---|
| `max_distance_m` / `max_angle_deg` | 2.0 / 180 | 单次上限 |
| `default_mps` / `max_mps` / `min_mps` | 0.15 / 0.2 / 0.04 | 直行速度 |
| `default_yaw_rps` / `max_yaw_rps` | 0.2 / 0.5 | 转向角速度；阿克曼实际上限 = `turn_linear_mps / min_turn_radius_m` = 0.25 |
| `ackermann`、`turn_linear_mps`、`min_turn_radius_m` | true、0.1、0.4 | mini_akm 不能原地转：转 90° 约走 0.63 m 弧长；最小半径为占位值，需现场测量 |

## 5. 与现有入口的冲突（必须注意）

- guard 要求 `/chassis/cmd_vel` **恰好一个发布者**。语音 `offline_voice.launch.py` 会启动 `chassis_motion_adapter`，它也持续发布 `/chassis/cmd_vel`，两者不能同时运行 → T1.2 给语音入口增加关闭运动适配器的开关。
- 正在 FOLLOW 跟随时调用本接口会切到 EXTERNAL，跟随中断，结束后回 IDLE。
- 无雷达避障；转弯时车头会扫过前方约 0.4 m 半径区域，展示场地需清空。
