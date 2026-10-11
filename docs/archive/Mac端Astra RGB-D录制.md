# Mac 端 Astra S RGB-D 录制

`mac_record_rgbd.py` 用 OpenNI2 同时采集彩色和深度帧，并要求设备开启“深度到彩色”配准。每帧保存为 `color/000001.png` 与 `depth/000001.png`，另写入 `timestamps.csv` 和 `metadata.json`。深度 PNG 保留 16 位毫米值，适合后续回放和分析。

## 使用

先把 Astra S 直接插到 Mac（尽量不要经过无供电 Hub），再检查：

```bash
system_profiler SPUSBDataType | grep -A8 -i 'astra\|orbbec\|2bc5'
```

Astra S 使用旧 OpenNI 协议。官方 OpenNI SDK 的发布包目前没有 macOS 二进制包，因此需要准备可用的 macOS OpenNI2 动态库，再安装 Python 绑定。项目已验证 Python 3.9 可安装 `openni` 包，但仅安装 Python 包仍会因缺少 `libOpenNI2.dylib` 而无法打开设备；将完整 OpenNI2 运行目录放入 `OPENNI2_REDIST` 后再运行：

```bash
python3 scripts/mac_record_rgbd.py \
  data/rgbd-recordings/back-follow-$(date +%Y%m%d-%H%M%S) \
  --seconds 60 \
  --openni-path "$OPENNI2_REDIST"
```

脚本缺少 OpenNI2、找不到相机、无法配准或帧尺寸不符时会退出，并且不会偷偷改录成只有 RGB 的视频。录制时让人从正面、背面、侧面和不同距离缓慢移动，保留相机视野内的完整人体；每段 30–60 秒即可。录制结束后把整个目录保留，特别是 `metadata.json`、`timestamps.csv` 和两组 PNG。

当前 Mac 检查到 Pillow/numpy，但尚未检测到 Astra S，也没有安装 OpenNI2 Python 绑定；因此在相机插入并安装绑定前不能宣称已完成实录。

## Astra S 的可用适配库

实测发现官方 `OrbbecSDK_C_C++ v1.10.16` 的 macOS ARM64/x86 发布包可以打开本机 Astra S（PID `0x0402`），并提供彩色、深度和 IR 传感器。它走 Orbbec SDK v1 的兼容 OpenNI 协议，不依赖 `libOpenNI2.dylib`。将该 SDK 解压到 `~/.local/share/roscar/orbbec-sdk-v1.10.16`，或设置 `ORBBEC_SDK_ROOT`，然后运行：

```bash
scripts/mac_record_orbbec_rgbd.sh \
  data/rgbd-recordings/back-follow-$(date +%Y%m%d-%H%M%S) 60
```

录制器启用硬件深度到彩色对齐，并用 `waitForFrames` 成对取帧；Astra S 不支持新版 SDK 的显式 frame-sync 控制，因此元数据会明确记录这一点。录制器保存原始 RGB8 与 16 位毫米深度，后续可用 Python/Pillow 转成 PNG，避免在录制时增加压缩延迟。SDK 发布页：[OrbbecSDK v1.10.16](https://github.com/orbbec/OrbbecSDK/releases/tag/v1.10.16)。

## 自动归档

Astra Viewer 的启动器已增加自动归档。它会在启动时保护遗留的 `Captured.oni`，并在每次录制停止、文件大小和修改时间连续稳定 3 秒后自动复制到：

```text
data/rgbd-recordings/astra-viewer-auto-YYYYMMDD-HHMMSS/Captured.oni
```

正常操作仍然是：打开配准（`i`）→开始（`s`）→停止（`x`）。不需要手动复制文件；启动器会保留每段录制。若 Viewer 应用被重新安装，需重新安装项目中的自动归档启动器。

启动器会按文件 SHA-1 跳过同一份 `Captured.oni` 的重复归档，避免仅因重启 Viewer 或回放而产生多个副本。
