# A：Astra 骨架适配入口

正式 `route:=astra` 启动 `bodylist_adapter`，订阅厂商 `/bodylist`，通过叉腰手势选择或切换目标，输出统一 TargetState、人体掩码和 Foxglove Marker。节点不发布 `/cmd_vel`。

厂商 `bodyreader/main` 必须另行启动，并从包含 SDK 配置和动态库的原厂目录运行。项目的 `scripts/run_astra_foxglove.sh` 已按此方式组合启动 bodyreader、正式 A route 和 Bridge，不包含厂商 `bodydata_process`、follower 或底盘 launch。

Bodylist 没有源时间戳或置信度，因此 `observation_stamp` 为零，`measurement_age_s` 和 `confidence` 为 NaN。质心从毫米转换为米；Y 轴转换和 SDK 授权提示仍需后续长期验证。

叉腰判定使用较宽松的默认阈值，并以最近 10 帧中满足 3 帧作为锁定条件：手高于脊柱基点 20 mm、手与对应肩膀横向差小于 160 mm、肩高于手 20 mm。可通过正式 launch 参数调整：

```bash
ros2 launch perception_bringup perception.launch.py route:=astra \
  akimbo_hand_above_base_min_mm:=20.0 \
  akimbo_hand_shoulder_max_dx_mm:=160.0 \
  akimbo_shoulder_above_hand_min_mm:=20.0 \
  akimbo_window_frames:=10 akimbo_min_votes:=3
```
