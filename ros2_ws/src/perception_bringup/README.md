# 启动与演示

perception.launch.py 的 route 只能为 astra、yolo 或 demo，默认 yolo。
astra 启动真实 `/bodylist` 适配器；yolo 当前仍发布 NOT_READY；只有显式 route:=demo 才会产生模拟位置。
with_foxglove:=true 时需要另行安装 ros-humble-foxglove-bridge。
当前 launch 不自动启动相机、厂商 SDK 或底盘；方案 A 的安全组合入口为 `scripts/run_astra_foxglove.sh`。
日常使用推荐 `scripts/route_a.sh start`，它在后台管理该组合入口，并提供 `stop`、`restart`、`status` 和 `logs`。Mac 可用 `scripts/route_a_remote.sh start` 通过 `roscar-wifi` 远程调用。两者都不启动底盘或 `/cmd_vel`。

astra route 暴露 `akimbo_hand_above_base_min_mm`、`akimbo_hand_shoulder_max_dx_mm`、`akimbo_shoulder_above_hand_min_mm`、`akimbo_window_frames` 和 `akimbo_min_votes`，默认分别为 20、160、20 mm、10 帧和 3 票。
