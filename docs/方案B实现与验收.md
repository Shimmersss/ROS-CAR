# 方案 B：本地实现与实机验收边界

## 目标与当前范围

链路：校正后的 RGB + 配准到同一彩色光学坐标系的深度 → YOLO11n person 检测 → ByteTrack → 显式锁定 → 躯干稳健测距 → TargetState、检测框图像、Foxglove 目标球。

本分支 `codex/route-b` 补齐软件实现与本机测试，不部署小车、不切换 A 的开机服务。相机输入、Jetson 推理性能和真人跟踪必须另行实测。车辆控制、避障、语音和串口不在本次范围。

## 输入契约

| 参数 | 默认值与要求 |
|---|---|
| yolo_python | 空，使用构建时的解释器；可指定与 ROS 同版本的虚拟环境 Python 路径 |
| model_path | 空；须指定本地已有 yolo11n.pt 文件，禁止自动下载权重 |
| device | cpu；本机验证使用 CPU，Jetson 完成环境验证后可设 0 |
| image_size | 640 |
| color_topic | /camera/color/image_rect，RGB/BGR/RGBA/BGRA 8 位校正图 |
| depth_topic | /camera/aligned_depth_to_color/image_raw，16UC1 毫米或 32FC1 米 |
| camera_info_topic | /camera/color/camera_info，使用校正后的 P 矩阵 |
| depth_registered | false；人工验证配准后才显式开启 |
| sync_slop_s | 0.06，RGB/深度最大采集时间差 |
| max_age_s | 0.5，消息年龄和本地接收后年龄上限 |

默认话题是接口约定，**尚未确认 ASTRA S 驱动实际提供这些名称及校正/配准语义**。订阅传感器数据采用 best-effort QoS，RGB/深度近似同步队列为 5。CameraInfo 缓存作为静态标定使用，分辨率和 frame_id 必须与输入一致；P 必须有有效焦距及零平移项。不接受零时间戳、未来时间戳、过期消息、深度原始 frame 与彩色 frame 混用或尺寸不一致。跨主机播放时必须同步时钟。

同分辨率、同 frame_id 和 `depth_registered=true` 不能从软件上证明配准正确；这些只是防误接条件。驱动输出原始 RGB 时应先通过 image_proc 等相机校正节点处理，深度必须匹配校正后的彩色像素。首轮实机用近/远物体边缘检查对应关系，不能只重命名话题或 frame_id。

## 检测、跟踪与选人

- Ultralytics 固定 8.3.203，加载 YOLO11n，`classes=[0]`、`conf=0.1`、`tracker='bytetrack.yaml'`、`persist=True`。低阈值候选交给 ByteTrack 关联；没有轨迹 ID 的框不参与锁定。
- 单个后台推理任务运行时丢弃新图像对，不无限堆积。输出是否有效还检查推理结束后的采集年龄。性能不足时应调小输入或优化推理，不能伪造时间戳掩盖延迟。
- 初始 SEARCHING。调用 `/perception/lock_target`（std_srvs/Trigger），选择当前新鲜候选中最靠近图像水平中心的轨迹；同距离按 ID 排序。响应返回选定 ID。
- 调用 `/perception/release_target` 释放；再次调用 lock_target 可显式切换。当前版本没有点击任意框/任意 ID 服务，也没有叉腰/Pose 手势。
- 轨迹 ID 表示为 `epoch:track_id`。流中断、时间倒退、分辨率/frame/标定变化和推理失败都会使后续轨迹进入新 epoch；旧锁定不会因 ID 复用而恢复，须重新锁定。
- 同一 epoch 内短时丢失后 ByteTrack 若重新给出同一 ID，可恢复观测；ByteTrack 仍可能串人，不能承诺持久身份或跨遮挡可靠 ReID。

## 深度与输出

取检测框横向 30%–70%、纵向 25%–60% 的中央躯干区域。保留 0.2–8 米的有限深度，至少 12 像素且有效比例不低于 30%。使用中位数 Z，四分位距超过 max(0.25 米, 0.2Z) 时拒绝；从接近中位深度的像素计算代表像素，用 P 中的 fx/fy/cx/cy 反投影。

这个位置是躯干区域代表点，既非脚点也非检测框中心。当前不加额外位置卡尔曼或平滑；ByteTrack 的框卡尔曼不能视为 3D 位置滤波。后续用固定距离与移动实验评估抖动后再决定。

| 状态 | 条件 |
|---|---|
| NOT_READY | 未配置模型/未确认配准、初始化或推理异常、输入校验失败 |
| STALE | 没有新鲜的同步推理结果或断流 |
| SEARCHING | 输入与算法有效，尚未锁定 |
| TRACKING | 锁定 ID 仍在；深度无效时 position_valid=false |
| LOST | 锁定 ID 未匹配，或已经换 epoch |

所有无效位置、距离和偏角为 NaN，Marker 发 DELETE。有效位置单位米，光学 X 右、Y 下、Z 前；水平距离 sqrt(X²+Z²)，偏角 atan2(X,Z)。header.stamp 是状态发布时间，observation_stamp 是被采用 RGB 的采集时间；只有新鲜结果保留观测时间。锁定 ID 不自动改成另一个人。

输出 `/perception/target_state`、`/perception/detections_image`（bgr8，保留 RGB header）、`/perception/target_marker`。Marker 为 15cm 目标球、寿命 0.2 秒；消费者仍须检测整个节点退出导致的话题断流。Foxglove 使用 Image、Raw Messages、3D 面板，3D 固定坐标设为实际彩色 optical frame。

## 本机启动与验证

本机 ROS 2 使用 Linux ARM64 Humble 容器：

```bash
bash scripts/test_container.sh
```

该脚本编译主动工作区并运行纯逻辑、合成 RGB-D/检测结果注入、A/B/demo 路由回归。合成检测后端只能从测试代码构造，不是 launch 参数或可部署演示模式；不能把测试中的 `is_simulated=false` 当真实识别证据。

真实模型本机独立冒烟测试：

```bash
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r ros2_ws/src/yolo_person_tracker/requirements-local.txt
.venv/bin/python scripts/test_yolo_model.py
```

这是 CPU 上加载权重、空白图推理与 ByteTrack 调用兼容性检查，没有真人图像，不能作为识别率验收。

接入**已经验证**的相机话题后，ROS 终端示例：

```bash
ros2 launch perception_bringup perception.launch.py route:=yolo \
  model_path:=/absolute/path/yolo11n.pt device:=cpu depth_registered:=true \
  color_topic:=/camera/color/image_rect \
  depth_topic:=/camera/aligned_depth_to_color/image_raw
ros2 service call /perception/lock_target std_srvs/srv/Trigger '{}'
ros2 service call /perception/release_target std_srvs/srv/Trigger '{}'
```

如果模型依赖安装在虚拟环境，追加 `yolo_python:=/absolute/venv/bin/python3`；仅 activate 不保证 ROS 安装脚本的 shebang 会使用该环境。解释器必须与 ROS 的 rclpy/cv_bridge ABI 匹配（Humble Ubuntu 22.04 为 Python 3.10）。Mac 的 Python 3.12 虚拟环境仅用于独立 CPU 冒烟测试。

launch 不启动相机。A/B 不能同时争用相机或共同向同一目标话题输出；未来切换时先停止 A，B 验收结束后恢复原服务。此阶段不提供自动停 A 或改开机服务的脚本。

## 后续 Jetson 验收顺序

1. 停 A 后验证实际 RGB 来源、校正图、配准深度、P 内参、frame、编码、时间差；RGB 缺失先解决相机输入。
2. 在独立 `--system-site-packages` 虚拟环境补齐 requirements 中应用依赖，保留 Jetson 已验证的 CUDA torch/torchvision，不能让通用 pip 解析替换 CUDA 版本。补齐依赖后检查 `pip check` 与 CUDA NMS。
3. 先空画面跑通真实模型，再真人检测、显式锁定、稳定距离、遮挡恢复、多人交叉；录制带时间的结果。
4. 核对 0.8/1/1.5/2 米位置误差、深度有效率、采集到输出延迟和资源占用。当前 0.5 秒为失效门限，不是已达性能指标。
5. 断 RGB、断深度、改 frame、拔相机、重启模型，确认无效位置及 epoch 重锁行为；确认 `/cmd_vel` 不存在。

参考：[Ultralytics 跟踪 API](https://docs.ultralytics.com/modes/track/)。本实现显式选择 ByteTrack，而不是依赖默认跟踪器。

## 2026-09-15 本机验证结果

- Linux ARM64 Humble：8 个主动包编译完成（8.58 秒）。A/B 各 7 项逻辑测试、B 合成 RGB-D/锁定服务/Marker 测试、A 合成状态机、14 项语音逻辑和 A/B/demo/非法 route 回归全部通过。
- Mac Python 3.12.13：36 项依赖兼容性检查通过；torch 2.6.0、torchvision 0.21.0、Ultralytics 8.3.203、OpenCV 4.10.0.84。校验厂商权重 SHA-256 后，实际 CPU 模型完成两帧空白图推理与 ByteTrack reset。
- 日志保存于 `artifacts/route-b-humble-test.log` 与 `artifacts/route-b-model-smoke.log`。合成消息及空白图没有验证真人识别、空间配准或 Jetson GPU 性能。
