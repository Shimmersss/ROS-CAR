# 当前部署的三个推理模型

用户明确要求纳入仓库的模型资产，使用Git LFS保存实际二进制，不只是本机路径或下载链接。完整清单、字节数、SHA256及来源见 [manifest.json](manifest.json)。

| 后端 | 文件 | 大小 | 内容 |
|---|---|---:|---|
| PCB | `pcb/best.pt` | 44,047,001字节 | 实际部署YOLO26m权重，六类缺陷，imgsz640/conf0.2 |
| 安全帽 | `helmet/helmet-yolov8l.pt` | 87,661,852字节 | 实际部署权重，helmet/no_helmet，conf/iou0.45 |
| 舌诊 | `tongue/tongue_cloud_server_20261015.zip` | 1,596,070,132字节 | 原始自包含交付包：分割器、10个分类器、冻结阈值、服务代码及45文件校验 |

舌诊ZIP保持原件，不重新打包；冻结 `cloud_r50_384_cvAC10_passthrough_debug`，digest `1a897fa5e7967eef62d60de50c98a1909224008e27054dec8e31a10e8f76aef1`。GPU数值比较历史失败和用户允许继续使用GPU的例外仍保留，不称准确率通过。其他未使用模型、ONNX副本、虚拟环境、凭据、实拍图片和录像不在此目录。

```bash
git lfs install --local
git lfs pull --include='models/inference/**'
python3 scripts/prepare_inference_deployment.py --verify-only
```

原始第三方模型保留原权利与许可；安全帽权重由用户确认可信，仓库纳入不额外授予模型许可。模型文件由用户提供，不能把项目代码许可证自动套用到权重。

新电脑部署、迁移云端、小车URL/端口/token更新及回退见[云端推理部署与迁移](../../docs/云端推理部署与迁移.md)。解压副本只放忽略的运行目录，避免重复入库约1.7GB权重。
