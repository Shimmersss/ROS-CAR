# A：Astra 骨架适配入口

正式 `route:=astra` 启动 `bodylist_adapter`，订阅厂商 `/bodylist`，通过叉腰手势选择或切换目标，输出统一 TargetState、人体掩码和 Foxglove Marker。节点不发布 `/cmd_vel`。

厂商 `bodyreader/main` 必须另行启动，并从包含 SDK 配置和动态库的原厂目录运行。项目的 `scripts/run_astra_foxglove.sh` 已按此方式组合启动 bodyreader、正式 A route 和 Bridge，不包含厂商 `bodydata_process`、follower 或底盘 launch。

Bodylist 没有源时间戳或置信度，因此 `observation_stamp` 为零，`measurement_age_s` 和 `confidence` 为 NaN。质心从毫米转换为米；Y 轴转换和 SDK 授权提示仍需后续长期验证。
