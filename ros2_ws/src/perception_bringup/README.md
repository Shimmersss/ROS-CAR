# 启动与演示

> 方案 A 在分支 `a` 改为红色目标 + 配准深度；原 `route:=astra` 保留，B 不变。最新入口、串口与本机/实机边界见 [红色方案 A](../../../docs/方案A红色物体跟随.md)。本轮未部署小车。


perception.launch.py 的 route 只能为 astra、yolo 或 demo，默认 yolo。
astra 启动真实 `/bodylist` 适配器；yolo 已接入真实 RGB-D 跟踪节点，未配置模型或未确认配准时发布 NOT_READY；只有显式 route:=demo 才会产生模拟位置。
with_foxglove:=true 时需要另行安装 ros-humble-foxglove-bridge。
当前 launch 不自动启动相机、厂商 SDK 或底盘；方案 A 的安全组合入口为 `scripts/run_astra_foxglove.sh`。
日常使用推荐 `scripts/route_a.sh start`，它在后台管理该组合入口，并提供 `stop`、`restart`、`status` 和 `logs`。Mac 可用 `scripts/route_a_remote.sh start` 通过 `roscar-wifi` 远程调用。两者都不启动底盘或 `/cmd_vel`。

astra route 暴露 `akimbo_hand_above_base_min_mm`、`akimbo_hand_shoulder_max_dx_mm`、`akimbo_shoulder_above_hand_min_mm`、`akimbo_window_frames` 和 `akimbo_min_votes`，默认恢复为 50、100、50 mm、1 帧和 1 票。

B route 参数与验收步骤见 `docs/方案B实现与验收.md`，不自动启动相机或切换方案 A。
