# A：Astra 骨架适配入口

正式 `route:=astra` 启动 `bodylist_adapter`，订阅厂商 `/bodylist`，通过叉腰手势选择或切换目标，输出统一 TargetState、人体掩码和 Foxglove Marker。节点不发布 `/cmd_vel`。

厂商 `bodyreader/main` 必须另行启动，并从包含 SDK 配置和动态库的原厂目录运行。项目的 `scripts/run_astra_foxglove.sh` 已按此方式组合启动 bodyreader、正式 A route 和 Bridge，不包含厂商 `bodydata_process`、follower 或底盘 launch。

Bodylist 没有源时间戳或置信度，因此 `observation_stamp` 为零，`measurement_age_s` 和 `confidence` 为 NaN。质心从毫米转换为米；Y 轴转换和 SDK 授权提示仍需后续长期验证。

## 人体跟随控制（可选）

`person_follower` 订阅 `/perception/target_state` 并以20 Hz发布 `/cmd_vel`。它默认禁用，且只在目标处于 `TRACKING`、位置有效且最近 0.5 秒内有消息时输出非零速度。默认保持2米、最高前进 0.15 m/s、最高转向 0.5 rad/s，不自动倒车。

```bash
ros2 run astra_body_adapter person_follower
ros2 param set /person_follower enabled true
```

底盘驱动需单独启动并订阅 `/cmd_vel`。该节点不提供避障；无雷达、Nav2 或超声波保护时只能在受控空旷区域使用。

叉腰判定默认恢复为厂商等价条件并单帧锁定：手高于脊柱基点 50 mm、手与对应肩膀横向差小于 100 mm、肩高于手 50 mm。可通过正式 launch 参数调整：

```bash
ros2 launch perception_bringup perception.launch.py route:=astra \
  akimbo_hand_above_base_min_mm:=50.0 \
  akimbo_hand_shoulder_max_dx_mm:=100.0 \
  akimbo_shoulder_above_hand_min_mm:=50.0 \
  akimbo_window_frames:=1 akimbo_min_votes:=1
```
