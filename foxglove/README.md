# Foxglove：B 方案、N10P、语音与底盘

> C 路线（`scripts/start_c.sh`）使用 [c-layout.json](c-layout.json)，只放跟随要看的 5 块：骨架画面（框、骨架、姿态/跌倒阶段、LOCKED）、目标说明、距离/偏角曲线、`/control/state`、最终 `/cmd_vel`。逐人状态、ReID、统一定位、性能等调试数据不进布局，需要时用 `ros2 topic echo`。实际客户端显示待现场验收。

当前布局为 [b-radar-layout.json](b-radar-layout.json)。Mac Foxglove 经网口连接 `ws://192.168.100.2:8765`，从文件导入布局；同一 Wi-Fi 下也可用 `ws://192.168.1.240:8765`。`bash scripts/start_project.sh` 默认启动 Astra、YOLO26s/ByteTrack 和语音，不接底盘、不跟随；N10P 需 `WITH_RADAR=true`。`bash scripts/start_robot.sh` 是整车配置：另开底盘串口和**受保护自动跟随**（`MOTION_ENABLED=true`、auto-arm），不启 N10P，车会动。布局本身不控制车辆。

| 区域 | 话题 | 当前含义 |
|---|---|---|
| 视频 | `/perception/color_preview/compressed`、`/perception/detections_preview/compressed` | 低带宽 JPEG 彩色预览与 B 人体检测框；原始图和未压缩框图仍在 ROS 中，布局不订阅高带宽原始图 |
| 目标 | `/perception/target_state`、`/perception/target_marker`、XYZ/距离图 | 需要 `/perception/lock_target` 显式选人，且配准深度有效后才有坐标；`position_valid=false` 时 NaN 是预期无效值 |
| 雷达 | `/radar/points`、`/radar/status` | 仅在 `WITH_RADAR=true` 时有数据；N10P 的二维激光扫描，在独立 `laser` 坐标系展示；安装外参未验证，不能叠加到相机或地图 |
| 语音 | `/voice/asr_text`、`/voice/assistant_text` | 唤醒/提问后出现识别和回答文本 |
| 底盘 | `/odom`、速度图 | 串口里程计回传；`start_project.sh` 默认不启动底盘和跟随。`start_robot.sh` 下跟随器经 motion_guard 发布非零速度 |
| 地图/路径 | `/map`、`/plan`、`/local_plan` | 预留给 SLAM/Nav2；当前未运行时面板为空，不代表已有三维建图 |

B 的原始目标坐标采用 `camera_color_optical_frame`；`laser` 扫描与 `map` 建图各使用独立 3D 面板。N10P 产生平面扫描，Foxglove 的 3D 视图只是显示这种数据，并非相机点云或真正三维重建。没有有效目标时，目标球应消失。`start_robot.sh` 按用户要求启用驱动注册深度和临时内参，B 可锁定并发布三维坐标；重新标定前仍须将距离视为近似值，而跟随正是基于这个近似距离。调用 `ros2 service call /perception/lock_target std_srvs/srv/Trigger '{}'` 选中当前中心轨迹。轨迹 ID 改变后需再次锁定；未锁定或躯干深度无效时 XYZ 为 NaN。`start_project.sh` 默认运动关闭；`start_robot.sh` 运动开启。

仓库中的旧 `red-layout.json`、`astra-layout.json` 和 `navigation-layout.json` 仅保留为历史参考，不是当前运行配置。若客户端显示“没有数据源”，重新连接上述 WebSocket；若图像面板提示等待消息，检查面板的图像主题字段和小车话题是否有帧。
