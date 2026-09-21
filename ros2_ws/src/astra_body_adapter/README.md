# A：Astra 骨架适配入口

正式 `route:=astra` 启动 `bodylist_adapter`，订阅厂商 `/bodylist`，通过叉腰手势选择或切换目标，输出统一 TargetState、人体掩码和 Foxglove Marker。节点不发布 `/cmd_vel`。

厂商 `bodyreader/main` 必须另行启动，并从包含 SDK 配置和动态库的原厂目录运行。项目的 `scripts/run_astra_foxglove.sh` 已按此方式组合启动 bodyreader、正式 A route 和 Bridge，不包含厂商 `bodydata_process`、follower 或底盘 launch。

Bodylist 没有源时间戳或置信度，因此 `observation_stamp` 为零，`measurement_age_s` 和 `confidence` 为 NaN。质心从毫米转换为米；Y 轴转换和 SDK 授权提示仍需后续长期验证。

## 人体跟随控制（可选）

`person_follower` 默认 `expected_source=astra`，新 A 组合入口设置为 `red_object`。拒绝模拟目标，并校验发布/观测时间和测量年龄；原 Astra 缺传感器时间戳时，仅在明确选择该来源后使用接收和发布时间。

`person_follower` 订阅 `/perception/target_state`，20 Hz 发布 `/control/cmd_vel_request`（TwistStamped，base_link），不再直接发布 `/cmd_vel`。仍检查真实来源、唯一目标发布者和观测时效；光学正右偏角对应负角速度。

运动必须通过 `motion_guard`，见 `ros2_ws/src/motion_guard/README.md`。保护入口默认未授权，尺寸/安装/停车参数未确认时只允许感知；授权后失效会停车并锁存。默认纯感知入口不启动跟随或保护节点。
