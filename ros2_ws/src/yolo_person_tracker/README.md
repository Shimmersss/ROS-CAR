# B：YOLO 人体跟踪

已实现 YOLO26s + ByteTrack、显式锁定/释放、配准深度筛选、TargetState 和 Foxglove 检测框/目标球输出。默认模型路径为空且 depth_registered=false，保持 NOT_READY；配置有效模型与经过验证的相机输入后运行真实推理。

完整输入契约、依赖、启动命令、测试范围和待实机验收项见 [方案 B 实现与验收](../../../docs/方案B实现与验收.md)。

当前为本地软件实现，不代表 Jetson 真人验收通过。默认 CPU；Jetson CUDA 需要独立验证。没有运动控制输出。


B launch 附带独立 `target_transform`，原光学坐标输出不变，新增 `target_state_base` / `target_marker_base`。外参未确认时新增结果无效，原检测/锁定/测距继续运行；默认不发布占位安装 TF。配置见 perception_bringup/config/camera_mount.yaml。

跟踪器使用 2 线程 ROS executor + 单一顺序推理工作线程，状态交接受互斥锁保护。图像转换、YOLO、ByteTrack、躯干测距在工作线程顺序运行，不并发更新轨迹。
