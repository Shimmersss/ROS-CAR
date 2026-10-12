# 本机模型目录

默认模型是官方 COCO 预训练检测权重 `yolo26s.pt`（含 person 类）。执行 `python3 scripts/prepare_model.py` 从 Ultralytics assets v8.4.0 下载，并校验固定大小与 SHA-256。ROS 节点不会自动下载。

原始 `yolo11n.pt` 保留作对比；`python3 scripts/prepare_model.py --model yolo11n.pt` 从本地厂商目录校验复制。使用旧模型时设置 `nms_free:=false`。

权重和 TensorRT engine 不进入普通 Git，也不在默认远端同步范围内。`models/manifest.json` 记录来源、哈希和许可；官方许可为 AGPL-3.0 / Enterprise。

Jetson TensorRT FP16 入口：`python3 scripts/export_yolo26_engine.py` 默认只显示计划；在目标 Jetson 的匹配环境显式加 `--execute` 构建。输出 `yolo26s-fp16.engine` 和包含 GPU、CUDA/TensorRT、权重/引擎哈希的 `.engine.json`。不使用其他 GPU 上生成的通用 engine。

本机已验证 PyTorch 免 NMS 与 ByteTrack；Jetson FP16 engine 构建、性能和相机输入仍待实测。

C的可选ReID使用独立OSNet-x0.25 MSMT17模型：`python scripts/prepare_reid_model.py --download` 显式准备，输出ONNX及`.onnx.json`校验文件。运行时不下载，须两文件一起提供；不是用YOLO检测权重代替ReID。来源、许可和验证见 [ReID接入与验收](../../docs/方案C身份ReID接入与验收.md)。
