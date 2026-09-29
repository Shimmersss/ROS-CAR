# Orin Nano 8GB 端侧 ASR 候选实测

日期：2026-09-29。以下候选对比数据来自隔离脚本和 ROS 域 183；随后按用户要求将 Qwen3-ASR 切到正式语音入口。本轮没有发布运动命令，真人麦克风效果待用户现场测试。

## 正式切换结果

当前默认 `VOICE_BACKEND=offline ASR_BACKEND=offline`，`offline_voice` 的 `asr_backend=qwen3`，模型位于 `/home/wheeltec/ROSCAR-offline/models/sherpa-onnx-qwen3-asr-0.6B-int8-2026-03-25`。2026-09-29 车上原生构建成功，隔离域启动确认模型加载，正式服务重启后再次确认 Qwen3-ASR 节点加载且讯飞 ASR 未运行。切换后快照：总内存 7.4 GiB、已用约 4.8 GiB、可用约 2.3 GiB、swap 已用约 177 MiB；服务运行中。此次备份为 `/home/wheeltec/ROSCAR-backups/qwen3-asr-20260929-145122/before.tar.gz`。通过 `ASR_BACKEND=xfyun` 可切回讯飞识别。五条短播报样例和启动检查不等于真人语音或长期稳定性验收。

## 当前整车内存

运行相机、YOLO 跟踪、运动门禁、讯飞 ASR 和本地 TTS 时，`free -h` 显示总内存 7.4 GiB、已用约 3.8 GiB、可用约 3.4 GiB，zram swap 3.7 GiB、当时使用 0。YOLO 跟踪进程 RSS 约 1.21 GiB，TTS 约 330 MiB。模型下载所在 NVMe 分区还有约 98 GiB 空间。

## 模型对比

五条测试音频来自仓库 `wheeltec_mic_ros2/feedback_voice/`，为约两秒的已录制播报，不是用户经车载麦克风现场说话。解码时间不含录音、静音判定与 ROS 处理。三个模型都在整车服务运行时使用 CPU 两线程测量。

| 模型 | 进程峰值 RSS | 五条样例的文字 | 单句解码时间 |
| --- | ---: | --- | ---: |
| SenseVoice Small INT8 | 约 492 MiB | 4 条含正确方向；`stop.wav` 识别为“好的，小车瓶。” | 0.17–0.19 秒 |
| FireRedASR2 CTC INT8 | 约 929 MiB | 5 条均含正确方向或“停” | 1.07–1.30 秒 |
| Qwen3-ASR 0.6B INT8 | 约 1489 MiB | 5 条均含正确方向或“停” | 1.64–2.21 秒 |

Qwen3-ASR 测试期间观察到约 2.0 GiB 可用内存，swap 使用约 146 MiB；测试进程结束后可用内存恢复约 3.4 GiB。一次短测不能证明长时间并行稳定，也不能由五条播报推断真人语音准确率。现有路由器只接受明确指令，样例中的“好的，小车前进”不会自动转为驾驶命令；更换模型后仍需要用真实口令检查音频分段和文字解析。

## 前一轮 FireRedASR2 候选部署记录

`offline_voice` 的 `fire_red_ctc` 后端使用本地 sherpa-onnx 1.13.8 CPU 运行时。模型位于车上 `/home/wheeltec/ROSCAR-offline/models/sherpa-onnx-fire-red-asr2-ctc-zh_en-int8-2026-02-25`。车上原生构建成功；在独立 ROS 域启动 ASR 节点后，日志确认加载 `fire_red_ctc`。正式入口仍默认为 `ASR_BACKEND=xfyun`。把语音入口设为 `ASR_BACKEND=offline` 并正常重启，才会使用 FireRedASR2；恢复 `xfyun` 可切回。此次部署前备份为 `/home/wheeltec/ROSCAR-backups/fire-red-asr-20260929-144623/before.tar.gz`。

建议先在安全的现场条件下录制 10–20 条真实口令，逐条比较讯飞与 FireRedASR2 的识别文字、漏词、误触发和总响应时间，再决定是否切换正式入口。

## CUDA

JetPack 6.2 具有 CUDA 12.6、cuDNN 9。当前 CPU wheel 不含 CUDA 执行提供者。独立虚拟环境安装 `sherpa-onnx 1.13.8+cuda` arm64 wheel 后，加载 FireRedASR2 时报 `libcublas.so.10` 缺失：该 wheel 与本机 CUDA 版本不匹配。官方给出 Jetson Orin Nano JetPack 6.2 的源码构建方法，使用 ONNX Runtime 1.18.1 与 CUDA 12.6。若后续需要 GPU 加速，应按该方法单独构建，再测 GPU 内存占用和 YOLO 并行时延；当前 FireRedASR2 CPU 单句约 1.1–1.3 秒。

模型来源：[Qwen3-ASR 官方仓库](https://github.com/QwenLM/Qwen3-ASR)、[sherpa-onnx Qwen3 模型页](https://k2-fsa.github.io/sherpa/onnx/qwen3-asr/pretrained.html)、[sherpa-onnx FireRedASR2 模型页](https://k2-fsa.github.io/sherpa/onnx/FireRedAsr/pretrained.html)、[sherpa-onnx Linux CUDA 构建说明](https://k2-fsa.github.io/sherpa/onnx/install/linux.html)。
