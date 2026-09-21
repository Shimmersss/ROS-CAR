# 厂商雷达与激光雷达代码

从 `JP6.2_wheeltec_ros2_src_20260903/` 迁入，原始文件哈希保留在 SOURCE_MANIFEST.json 的 items 中，本地修改另记 local_modifications。

用户确认 N10P 后，项目通过 `scripts/build_radar.sh` 在独立副本中构建 `lslidar_msgs`、`lslidar_driver`，由 `perception_bringup/radar.launch.py` 使用 N10Plus 配置启动。父目录 COLCON_IGNORE 继续保留，其他雷达/融合包不自动编译或启动。

本地修复 X10 驱动 LaserScan 的 Y 方向镜像，使其与同 frame 的点云一致。Linux ARM64 Humble 编译和真实驱动伪串口协议测试已通过；尚未部署或读取实物。串口别名、物理方向、安装 TF 仍待现场确认。

完整入口和验收说明见根目录 `docs/N10P雷达接入.md`。
