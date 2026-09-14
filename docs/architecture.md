# 架构与边界

## 四个 ROS 包

- person_interfaces：公共 TargetState 消息，不依赖算法或硬件。
- astra_body_adapter：未来将厂商骨架结果适配为公共消息；当前仅 NOT_READY。
- yolo_person_tracker：未来组合 YOLO、ByteTrack、RGB-D 测距；当前仅 NOT_READY。
- perception_bringup：选择一个入口，提供独立且明确标记的 demo，按需启动 Foxglove 桥接。

A/B 都发布 /perception/target_state，但启动文件每次只选择一个路线，避免互相覆盖。需要同时对比时先改为各自命名空间与独立输入来源，不直接同时占用相机。

## 后续真实数据流

A：相机 → Astra SDK/bodyreader → 目标锁定与质心 → astra_body_adapter → TargetState。
B：相机 RGB → YOLO → ByteTrack/选人 → 对齐深度区域统计 → yolo_person_tracker → TargetState。
共同输出 → Foxglove 观察、日志与离线验证。当前框架不包含 cmd_vel 发布者。

demo 是单独的 synthetic 数据源，不能作为 A/B 已实现的证据。它不产生图像或点云，仅验证目标消息发布与丢失状态展示。

## 厂商与主动开发代码

厂商目录 2.4 GB，保持原路径，仅本地参考；不移动、覆盖或整体编入新工作区。需要验证 SDK/相机时可单独建立厂商工作区并选择必要包，再提供 ROS 消息给主动开发节点。移植代码时记录来源并保留许可证。

macOS Docker 验证 Linux/Humble 包结构与 ROS 消息链路；Jetson 验证 CUDA/TensorRT、USB、真实相机同步和测距。两种验证不能互相代替。
