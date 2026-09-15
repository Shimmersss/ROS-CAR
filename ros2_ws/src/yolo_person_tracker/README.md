# B：YOLO 人体跟踪

已实现 YOLO11n + ByteTrack、显式锁定/释放、配准深度筛选、TargetState 和 Foxglove 检测框/目标球输出。默认模型路径为空且 depth_registered=false，保持 NOT_READY；配置有效模型与经过验证的相机输入后运行真实推理。

完整输入契约、依赖、启动命令、测试范围和待实机验收项见 [方案 B 实现与验收](../../../docs/方案B实现与验收.md)。

当前为本地软件实现，不代表 Jetson 真人验收通过。默认 CPU；Jetson CUDA 需要独立验证。没有运动控制输出。
