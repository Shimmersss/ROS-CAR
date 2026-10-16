# 拍照推理网关

普通 Python HTTP 服务，不启动 ROS，不控制小车。默认不保存图像。模型只在显式配置后加载；缺失或校验失败的后端单独报告 `model_unavailable`。面诊尚无模型。

仓库内完整模型现位于 [models/inference](../../models/inference/README.md)，新机器推荐按[云端推理部署与迁移](../../docs/云端推理部署与迁移.md)使用准备脚本生成本机路径和私有配置；下面保留现有5060手动环境说明。

## 5060 WSL 启动

先校验原始模型包，解压到 `/root/roscar-models`，不要改写 `/mnt/e/models` 原件。第三方 `.pt` 只加载用户确认可信的权重。PCB 和安全帽使用独立 `venv-yolo`，不要使用 WSL 系统 ROS/Jazzy Python 环境。

```bash
python3 -m venv /root/roscar-models/venv-yolo
/root/roscar-models/venv-yolo/bin/python -m pip install \
  torch==2.8.0 torchvision==0.23.0 --index-url https://download.pytorch.org/whl/cu128
/root/roscar-models/venv-yolo/bin/python -m pip install -r services/inference_gateway/requirements.txt
```

这是 [PyTorch 官方提供的 CUDA 12.8 组合](https://pytorch.org/get-started/previous-versions/)；安装后必须运行一次 CUDA 矩阵运算，单看 `is_available()` 不算验证。舌诊单独运行原包的 HTTP 服务，监听 `127.0.0.1:8096`；只接 `passthrough_debug` 冻结版本。先做 CPU 参考与 GPU 数值比较，不通过须按指南暂停接入并报告，不能更改阈值使其通过。本轮比较失败记录保留（最大差 0.008267 > 0.001）；用户 2026-10-16 明确授权直接使用 GPU 继续接入。此为用户选择的例外，不代表数值对齐或准确率验收通过。示例配置要求 cuda，实际服务报告 CPU 时拒绝该后端，不静默切换。

复制 `config.example.yaml` 到忽略目录（例如 `artifacts/inference-loop/gateway.private.yaml`），核对实际 PCB 权重哈希。认证令牌使用所有者权限 0600 的私有文件，不能进 Git。默认只监听本机；需要开发机访问时，显式绑定可信内网或 SSH 隧道，并设置令牌。

```bash
bash services/inference_gateway/start_gateway.sh /path/to/gateway.private.yaml
```

## HTTP 合同

- `GET /health/live`：存活。
- `GET /health/ready`：各后端就绪状态、设备、GPU 可用/总显存、队列长度。部分后端可用不代表全部可用。
- `GET /v1/models`：模型版本、哈希、实际类别名、阈值。
- `POST /v1/infer/{face|tongue|helmet|pcb}`：JPEG/PNG 原始字节，至多 12 MiB。头部为 `X-Request-Id`、`X-Task-Id`、`X-Capture-Id`、`X-Robot-Id`、`X-Capture-Time`。配置令牌后所有请求须带 Bearer 认证。

业务状态通过 JSON 的 `status` 表达，HTTP 200；认证、路径、协议、请求过大使用 4xx。成功返回原图尺寸、结构化结果、安全摘要数据、模型信息、耗时及标注 JPEG 的 base64/尺寸/SHA256。标注在原图坐标绘制，最长边超过 1280 才等比缩小，JPEG 质量 90；响应最多 8 MiB。错误和重拍不返回标注图。

单一工作线程串行执行模型，等待队列默认 4；排队过期的请求不会推理。运行中的 CUDA 调用不能强行中断，过期/放弃后丢弃结果。HTTP 连接数也有上限。模型异常只返回固定错误状态，日志不输出图像或认证头。

## 本地软件验证

```bash
PYTHONPATH=. python3 -m pytest -q services/inference_gateway/tests
```

这些测试用假后端和合成图，不加载 torch/Ultralytics，不代表模型准确率、WSL GPU 或实车通过。实际阶段结果见 [闭环验收记录](../../docs/拍照推理闭环实现与验收.md)。
