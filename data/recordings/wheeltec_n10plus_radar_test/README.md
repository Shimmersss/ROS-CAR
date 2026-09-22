# WHEELTEC N10Plus 本机雷达测试

测试设备：`/dev/ttyACM0`（WCH.CN USB Single Serial，序列号 `0001`）

测试结果：驱动源码和 `lslidar_msgs` 已在本机编译成功，N10Plus 驱动成功打开 `/dev/ttyACM0`。

- 话题：`/scan`
- 类型：`sensor_msgs/msg/LaserScan`
- 录制时长：约 7.6 秒
- 消息数量：77 条
- 数据包：`scan_bag/scan_bag_0.db3`
- 单帧样例：`scan_once.yaml`

## Foxglove 点云回放

- 点云话题：`/x10/lslidar_point_cloud`
- 点云类型：`sensor_msgs/msg/PointCloud2`
- 发布频率：约 `10 Hz`
- 回放数据：`pointcloud_bag/pointcloud_bag_0.db3`
- 本次同时录制：`/scan` 和 `/x10/lslidar_point_cloud`，各 78 条消息
