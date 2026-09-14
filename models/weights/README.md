# 本机模型目录

此目录的权重和 TensorRT engine 不进入普通 Git。原始 yolo11n.pt 位于厂商源码的 ultralytics_ros2/model/。
运行 `python3 scripts/prepare_model.py` 可验证哈希后复制到此处；脚本不加载模型。
manifest.json 记录来源和校验值，权重存在不等于已验证推理兼容性。
