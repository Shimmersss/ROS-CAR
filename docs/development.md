# Mac / Jetson 开发与 Git 工作流

## 分工

Mac 保存主动开发源码、配置、文档、Foxglove 布局和 Git 历史；Jetson 保存运行副本、权重、生成的 TensorRT engine 以及原始录像。代表性的录像回传 Mac 做回归数据集。

A/B 在同一仓库不同 ROS 包中共存。main 保存经过检查的基线；按阶段使用 codex/astra-body、codex/yolo-follow 等短期分支，完成后合回，而非永久维护两套公共代码。当前创建的是公共框架，不自动建立远端、推送或批量提交原厂资源。

## 同步

```bash
python3 scripts/sync_to_jetson.py --host user@192.168.1.10 --dest /home/user/ROSCAR
python3 scripts/sync_to_jetson.py --host user@192.168.1.10 --dest /home/user/ROSCAR --apply
```

第一条为 dry-run（需要 SSH 可连接才能列出差异），第二条实际传输。父目录应已存在。脚本只同步代码、文档、清单和脚本，不传构建、Git、厂商包、录像和模型。不使用 --delete，删除文件需要后续明确清理或换新的部署目录，避免旧文件长期残留。

权重用独立复制/同步处理，通过 models/manifest.json 校验；不要将权重混入普通 Git。prepare_model.py 只读取字节计算哈希，不执行 pickle 或加载模型。

## 编译与启动

在 Jetson 的 Bash 中 source /opt/ros/humble/setup.bash，执行 scripts/build_ros.sh，再 source ros2_ws/install/setup.bash。
构建扫描限定 ros2_ws/src；不自动启动厂商程序或修改系统设置。

## 测试层级

1. check_project.py：Mac 可运行，检查 Python/XML/JSON、包边界与文件结构。
2. test_container.sh：真实 Linux ARM64 Humble 编译 + ROS 运行测试，不依赖相机。
3. Jetson：CUDA/TensorRT、SDK、USB、RGB-D 配准与延迟；当前尚未验证。

## 数据

录 ROS bag 时保存所用提交、参数、模型哈希、硬件与场景信息；元数据提交到 data/catalog，录像保存 data/recordings 并单独备份。调试时不用一直录像，只保存具有代表性的片段。
