# B：YOLO 人体跟踪入口

当前节点只发布 NOT_READY，尚未加载模型、相机或 ByteTrack。

后续接入步骤：
1. 配置 Astra RGB、对齐后的深度和内参话题，验证时间与单位。
2. 加载 models/weights/yolo11n.pt，限制 person 类，保留输入 header。
3. 接入 ByteTrack，增加显式目标选择、锁定与丢失状态。
4. 躯干有效深度筛选和稳健统计，反投影到 optical 坐标。
5. 输出 TargetState，评估位置平滑、深度异常与端到端延迟。
6. 实测后再决定 TensorRT、Pose、分割或 ReID。

现有 ultralytics_ros2 节点仅作参考；不要直接使用其交通标志启动配置。
