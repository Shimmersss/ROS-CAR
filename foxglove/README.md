# Foxglove：B 方案、N10P、语音与底盘

当前布局为 [b-radar-layout.json](b-radar-layout.json)。Mac Foxglove 经网口连接 `ws://192.168.100.2:8765`，从文件导入布局；同一 Wi-Fi 下也可用 `ws://192.168.1.240:8765`。总入口 `bash scripts/start_robot.sh` 启动 Astra、YOLO26s/ByteTrack、N10P、语音和底盘串口收发；`bash scripts/start_project.sh` 可按开关运行。布局本身不控制车辆。

| 区域 | 话题 | 当前含义 |
|---|---|---|
| 视频 | `/perception/color_preview/compressed`、`/perception/detections_preview/compressed` | 低带宽 JPEG 彩色预览与 B 人体检测框；原始图和未压缩框图仍在 ROS 中，布局不订阅高带宽原始图 |
| 目标 | `/perception/target_state`、`/perception/target_marker`、XYZ/距离图 | 需要 `/perception/lock_target` 显式选人，且配准深度有效后才有坐标；`position_valid=false` 时 NaN 是预期无效值 |
| 雷达 | `/radar/points`、`/radar/status` | N10P 的二维激光扫描，在独立 `laser` 坐标系展示；安装外参未验证，不能叠加到相机或地图 |
| 语音 | `/voice/asr_text`、`/voice/assistant_text` | 唤醒/提问后出现识别和回答文本 |
| 底盘 | `/odom`、速度图 | 串口里程计回传；当前入口不启动跟随，也不发布非零速度 |
| 地图/路径 | `/map`、`/plan`、`/local_plan` | 预留给 SLAM/Nav2；当前未运行时面板为空，不代表已有三维建图 |

B 的原始目标坐标采用 `camera_color_optical_frame`；`laser` 扫描与 `map` 建图各使用独立 3D 面板。N10P 产生平面扫描，Foxglove 的 3D 视图只是显示这种数据，并非相机点云或真正三维重建。没有有效目标时，目标球应消失。当前 `start_robot.sh` 按用户要求启用驱动注册深度和临时内参，B 可锁定并发布三维坐标；重新标定前仍须将距离视为近似值。调用 `ros2 service call /perception/lock_target std_srvs/srv/Trigger '{}'` 选中当前中心轨迹。轨迹 ID 改变后需再次锁定；未锁定或躯干深度无效时 XYZ 为 NaN。运动始终关闭。

仓库中的旧 `red-layout.json`、`astra-layout.json` 和 `navigation-layout.json` 仅保留为历史参考，不是当前运行配置。若客户端显示“没有数据源”，重新连接上述 WebSocket；若图像面板提示等待消息，检查面板的图像主题字段和小车话题是否有帧。
