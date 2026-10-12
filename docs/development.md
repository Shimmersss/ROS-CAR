# Mac / Jetson 开发与 Git 工作流

更新：2026-10-11。

## 分工

- Mac 存放主动开发源码、配置、文档、Foxglove 布局和 Git 历史，同时负责 Gemini 录像的采集、标注与离线评估。录像保存在 `data/recordings/`，不进 Git。
- Jetson 存放运行副本、权重、本机导出的 TensorRT engine，并执行硬件测试。正式目录 `/home/wheeltec/ROSCAR-current` 上有在运行的语音服务，验证时使用隔离目录和独立 ROS 域。

## Git

- 代码托管在 GitHub `Shimmersss/ROS-CAR`，`main` 为基线。功能在短期分支上开发（例如 `claude/*`），通过 PR 合入。
- GitHub Actions `unit-tests` 在 PR 和推送 main 时运行 `scripts/ci_unit_tests.sh`。
- 厂商原包、权重、录像、构建产物和代理配置不进 Git。

## 同步

```bash
python3 scripts/sync_to_jetson.py --host wheeltec@192.168.1.240 --dest /home/wheeltec/ROSCAR-c
```

不加参数时只预览差异（需要 SSH 可连接），核对后追加 `--apply` 才实际传输，父目录须已存在。同步按白名单进行，不传厂商包、权重、录像和构建产物，也不使用 `--delete`；删除文件需要另外明确清理或换新目录。

权重单独复制，用 `models/manifest.json` 校验。`prepare_model.py` 只读取字节计算哈希，不执行 pickle，也不加载模型。

## 编译与启动

在 Jetson 的 Bash 中依次执行 `source /opt/ros/humble/setup.bash`、`bash scripts/build_ros.sh`、`source ros2_ws/install/setup.bash`。构建只扫描 `ros2_ws/src`；迁入的厂商包分别用 `build_chassis.sh`、`build_radar.sh` 等独立构建。C 的启动入口为 `scripts/start_c.sh`，B 和整机为 `scripts/start_project.sh`。

## 测试层级

1. `scripts/ci_unit_tests.sh`：纯 Python 单测（Mac 与 CI 同口径），不含 ROS 运行时。
2. `scripts/check_project.py`：结构检查，覆盖 Python/XML/JSON、包边界和文件布局。
3. `scripts/test_container.sh` 等：在 Linux ARM64 Humble 容器中编译并运行 ROS 测试，不依赖相机。
4. 离线录像回放：`scripts/replay_falls.py`、`scripts/evaluate_falls.py`。只证明规则在已录数据上的表现。
5. Jetson：CUDA/TensorRT、相机、配准、延迟与真人验收，项目见 [现场实测要求](现场实测要求20261008.md)。

## 数据

每段录像目录自带 `notes.md`（相机架设、场地、衣着、异常）、`device.json` 和 `labels.csv`，格式见 [重拍视频要求](重拍视频要求20261010.md) §9。只保存有代表性的片段。外部数据集放 `data/external/`。
