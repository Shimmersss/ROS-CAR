# 方案 B：本地实现与实机验收边界

## 目标与当前范围

链路：校正后的 RGB + 配准到同一彩色光学坐标系的深度 → YOLO26s person 检测（免 NMS） → ByteTrack → 显式锁定 → 躯干稳健测距 → TargetState、检测框图像、Foxglove 目标球。

本分支 `codex/route-b` 补齐软件实现与本机测试，不部署小车、不切换 A 的开机服务。相机输入、Jetson 推理性能和真人跟踪必须另行实测。车辆控制、避障、语音和串口不在本次范围。

## 输入契约

| 参数 | 默认值与要求 |
|---|---|
| yolo_python | 空，使用构建时的解释器；可指定与 ROS 同版本的虚拟环境 Python 路径 |
| model_path | 空；须指定本地已有 yolo26s.pt 或本机 Jetson 构建的 yolo26s-fp16.engine 文件，禁止自动下载权重 |
| device | cpu；本机验证使用 CPU，Jetson 完成环境验证后可设 0 |
| image_size | 640；固定方形输入，与引擎构建尺寸一致 |
| nms_free | true；YOLO26 免 NMS 输出；旧 YOLO11 基线显式设 false |
| color_topic | /camera/color/image_rect，RGB/BGR/RGBA/BGRA 8 位校正图 |
| depth_topic | /camera/aligned_depth_to_color/image_raw，16UC1 毫米或 32FC1 米 |
| camera_info_topic | /camera/color/camera_info，使用校正后的 P 矩阵 |
| depth_registered | false；人工验证配准后才显式开启 |
| sync_slop_s | 0.06，RGB/深度最大采集时间差 |
| max_age_s | 0.5，消息年龄和本地接收后年龄上限 |

默认话题是接口约定，**尚未确认 ASTRA S 驱动实际提供这些名称及校正/配准语义**。订阅传感器数据采用 best-effort QoS，RGB/深度近似同步队列为 5。CameraInfo 缓存作为静态标定使用，分辨率和 frame_id 必须与输入一致；P 必须有有效焦距及零平移项。不接受零时间戳、未来时间戳、过期消息、深度原始 frame 与彩色 frame 混用或尺寸不一致。跨主机播放时必须同步时钟。

同分辨率、同 frame_id 和 `depth_registered=true` 不能从软件上证明配准正确；这些只是防误接条件。驱动输出原始 RGB 时应先通过 image_proc 等相机校正节点处理，深度必须匹配校正后的彩色像素。首轮实机用近/远物体边缘检查对应关系，不能只重命名话题或 frame_id。

## 检测、跟踪与选人

- Ultralytics 固定 8.4.156，加载 YOLO26s，`classes=[0]`、`conf=0.1`、`tracker='bytetrack.yaml'`、`persist=True`、`nms=False`、`rect=False`。当前固定版本的 `nms=False` 选择端到端检测头；无此输出会报错，避免静默退回常规 NMS。低阈值候选交给 ByteTrack 关联；没有轨迹 ID 的框不参与锁定。
- ROS 使用 2 线程执行器，输入/状态回调分组且共享状态用互斥锁保护。单个后台工作线程顺序完成图像转换、YOLO/ByteTrack 和躯干测距；任务运行时丢弃新图像对，不无限堆积。输出是否有效还检查推理结束后的采集年龄。性能不足时应调小输入或优化推理，不能伪造时间戳掩盖延迟。
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

真实模型本机独立冒烟测试（先准备官方预训练权重）：

```bash
python3 scripts/prepare_model.py
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r ros2_ws/src/yolo_person_tracker/requirements-local.txt
.venv/bin/python scripts/test_yolo_model.py
```

这是 CPU 上加载权重、空白图和官方随包 bus.jpg 人体图的 ByteTrack 兼容性检查；重复同一张图验证 ID 延续，不能作为动态场景、遮挡或识别率验收。

接入**已经验证**的相机话题后，ROS 终端示例：

```bash
ros2 launch perception_bringup perception.launch.py route:=yolo \
  model_path:=/absolute/path/yolo26s.pt device:=cpu depth_registered:=true \
  color_topic:=/camera/color/image_rect \
  depth_topic:=/camera/aligned_depth_to_color/image_raw
ros2 service call /perception/lock_target std_srvs/srv/Trigger '{}'
ros2 service call /perception/release_target std_srvs/srv/Trigger '{}'
```

如果模型依赖安装在虚拟环境，追加 `yolo_python:=/absolute/venv/bin/python3`；仅 activate 不保证 ROS 安装脚本的 shebang 会使用该环境。解释器必须与 ROS 的 rclpy/cv_bridge ABI 匹配（Humble Ubuntu 22.04 为 Python 3.10）。Mac 的 Python 3.12 虚拟环境仅用于独立 CPU 冒烟测试。

launch 不启动相机。A/B 不能同时争用相机或共同向同一目标话题输出；未来切换时先停止 A，B 验收结束后恢复原服务。此阶段不提供自动停 A 或改开机服务的脚本。

## 三维 TF 与未标定兼容性

正式 B launch 同时启动独立 `target_transform` 进程。跟踪器仍发布原有相机光学坐标 `/perception/target_state`、`/perception/target_marker` 和检测图；TF 节点仅订阅原始目标状态，另外发布 `/perception/target_state_base` 与 `/perception/target_marker_base`。**未标定、TF 缺失或 TF 节点退出均不会阻断原有检测/锁定/测距输出**；新输出无效不会反向修改跟踪器状态。

配置在 `ros2_ws/src/perception_bringup/config/camera_mount.yaml`，也可传 `camera_mount_config:=/absolute/path/custom.yaml`：

- `mount_translation: [0,0,0]`（文件中以浮点数填写）和 `mount_quaternion: [0,0,0,1]` 是单位变换占位，仅用于保存待填安装参数。
- `extrinsics_calibrated=false`：新增车体位置保持无效；原光学位置不受此开关影响。只有测量及验证完成后才改 true。
- `publish_mount_tf=false`：默认不把单位变换写入 `/tf_static`，不覆盖已有机器人 TF。确认没有 URDF/其他发布者提供相同边之后，才开启本节点的安装外参发布。
- `target_frame=base_link`，`mount_child_frame=camera_link`，`expected_source_frame=camera_color_optical_frame`，均需对照实机驱动核对名称。安装变换表示 **camera_link 原点及姿态在 base_link 中的表示**。
- `camera_link → camera_color_optical_frame` 由驱动/URDF 提供。这个已知的光学轴转换不是待标定单位矩阵，不能随意省略或重复换轴。

TF 节点按 `observation_stamp` 查询变换，不使用最新 TF 代替缺失的历史变换。查询不阻塞等待。只有原消息为真实 YOLO、TRACKING、位置有效、时间新鲜、源 frame 匹配，且标定确认/TF 查询成功时，才输出有效车体位置。停止更新、未来/零/旧时间戳、非有限点、TF 缺失或外推失败均输出无效位置与 Marker DELETE。不发布速度或启动底盘。

车体位置遵循 X 前、Y 左、Z 上；距离为 `hypot(X,Y)`，偏角为 `atan2(Y,X)`，左侧为正。原光学输出继续 `hypot(X,Z)`、`atan2(X,Z)`、右侧为正。不得把新增 base 话题直接 remap 给按光学轴设计的旧跟随控制器。

并发中，YOLO/ByteTrack 只由一个工作线程访问，初始化、reset 和 infer 不并发。ROS 的服务/定时器与输入回调通过短临界区交接状态，重型图像转换/推理/测距不持有状态锁；图像标注发布仍在结果消费回调。未改成多路并发推理。实际吞吐与延迟改善仍待真实硬件测量。

标定步骤：先验证 RGB-D 配准与内参，再按现有 `base_link` 定义测量相机安装平移和旋转，填入 YAML；用已知位置的标记板在中线、左右及不同距离验证转换后的 XYZ 和偏角。不是通过填单位矩阵就完成标定；相机固定安装后外参为静态 TF。

## YOLO26s 权重与加速模式（2026-09-20）

官方 [assets v8.4.0](https://github.com/ultralytics/assets/releases/tag/v8.4.0) 提供 COCO 预训练 `yolo26s.pt`，无需自行训练。已下载的文件为 20,422,725 字节，SHA-256 为 `646f8bc3fe0a656803d95c294f7852321748cb29d13466a1af8862e2db384a1b`，与 GitHub 官方 release asset digest 一致。检测仅筛选 person；未改成 Pose/分割。权重不进 Git，不由 ROS 节点自动获取。

本机使用免 NMS 的 PyTorch 输出进行兼容验证；目标 Jetson 路线是 **免 NMS + TensorRT FP16、640×640、batch=1、静态尺寸**。FP16 构建允许部分层及输入保持 FP32，不宣称全网络每一层均半精度。先不使用需要代表性数据校准的 INT8。官方说明见 [YOLO26](https://docs.ultralytics.com/models/yolo26/) 和 [TensorRT](https://docs.ultralytics.com/integrations/tensorrt/)。

```bash
# 在任意主机仅查看导出计划，不会连接小车
python3 scripts/export_yolo26_engine.py

# 以下仅供未来目标 Jetson 环境准备完成后执行，本轮尚未执行
# 解释器需要可用的 CUDA torch、JetPack 原生 TensorRT 10.x、onnx 和固定版 Ultralytics
.venv-yolo/bin/python scripts/export_yolo26_engine.py --execute

# 完成相机校正/配准验收，并生成引擎后，未来 ROS 启动示例
ros2 launch perception_bringup perception.launch.py route:=yolo \
  yolo_python:=/home/wheeltec/ROSCAR/.venv-yolo/bin/python \
  model_path:=/home/wheeltec/ROSCAR/models/weights/yolo26s-fp16.engine \
  device:=0 image_size:=640 nms_free:=true depth_registered:=true
```

导出脚本只允许目标 Jetson 本机执行，关闭 Ultralytics 自动装依赖，不替换系统 CUDA/TensorRT。保留 JetPack 已验证的 torch/torchvision；应用包升级后必须运行 `pip check`，按缺项补齐普通 Python 依赖（本版新增 cloudpickle/filelock/nvidia-ml-py 等），不得直接用普通 pip 解析替换 CUDA 轮子。导出另需 onnx（例如 1.17.0），`simplify=False` 避免因图简化额外安装 ONNX Runtime。当前版本以 `quantize=16` 指定 FP16。

导出在临时目录完成，加载引擎跑一帧空图并核对免 NMS/FP16 导出元数据后才写入正式 engine，附 `.engine.json` 记录 GPU、L4T、CUDA、TensorRT 和哈希。已有引擎不会覆盖。Mac 无 NVIDIA CUDA，不能在本机生成或验证 Jetson 的 engine；不要从网上下载其他 GPU 的 engine 当作通用权重。本轮只验证导出计划与平台拒绝保护，真实 TensorRT 构建与加速幅度仍待板端验收。

默认空模型路径和 `depth_registered=false` 保持 NOT_READY；切换权重不会放宽输入要求。旧 YOLO11n 对比命令：`.venv/bin/python scripts/test_yolo_model.py --model yolo11n.pt --legacy-nms`；ROS 旧模型需 `nms_free:=false`。

## 只读 RGB-D 输入预检（2026-09-20）

在已构建并 source 的 ROS Humble 工作区中运行；无需模型和 CUDA。工具只订阅，不启动相机或车辆，也不替你切换 A 服务：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 scripts/check_rgbd_input.py --duration 15 \
  --color-topic /camera/color/image_rect \
  --depth-topic /camera/aligned_depth_to_color/image_raw \
  --camera-info-topic /camera/color/camera_info \
  --output artifacts/rgbd-input-report.json
```

报告包含每路接收数量/窗口平均频率、缺流/停止更新、原始消息年龄、同步对时间差、最近两路时间戳差、最新尺寸/编码/frame/内参、输入拒绝原因与整幅深度 0.2–8 m 有效比例。只保存统计和标定信息，不保存图像。时间差统计只包含已配对的消息；`latest_stamp_skew_s` 是各路最近一帧的差，不能单独当作同步误差。CameraInfo 按静态标定缓存，不要求持续刷新。

`metadata_pass=true` 仅表示至少一个同步对通过、窗口内没有校验/解码错误、三路均出现且 RGB/深度及最近合格同步对在结束时仍新鲜；退出码为 0，否则为 1。有效深度比例只是全图诊断指标，零深度帧仍可通过格式校验；能否测人体距离由 B 节点的躯干采样决定。`registration_verified` 始终为 false，必须另行用实物边缘验证配准及校正。`cmd_vel_topic_observed` 仅记录当前 ROS 域的发现结果，false 不能证明其他域或整个硬件没有控制程序。

预检与 B 正式节点共用 `input_contract.py`，投影矩阵要求有限、正焦距、零 skew/平移和标准最后一行，避免采用反投影公式不支持的 P。工具未发现消息时，先检查当前 ROS 域、真实话题名称和发布者；两路都有帧但 pairs=0 时，检查时间戳差与同步容差，不能通过重写时间戳掩盖问题。

### 厂商相机代码审查发现（不是硬件结论）

本地 `ros2_astra_camera-master/astra_camera` 源码存在以下行为：

- `launch/astra.launch.xml` 使用 `/camera` 命名空间，默认 `depth_registration=false`、`color_depth_synchronization=false`，彩色/深度由 `src/ob_camera_node.cpp::setupPublishers` 发布为 `color/image_raw` 和 `depth/image_raw`。开启配准不自动变成本文示例的 `aligned_depth_to_color/image_raw`。
- `setImageRegistrationMode` 遇到不支持或设置失败只打印日志；`onNewFrameCallback` 仍按请求参数 `depth_registration_` 选择 aligned frame。因此 frame 一致不足以证明配准成功，必须同时检查驱动错误和实物边缘。
- OpenNI 回调使用 `node_->now()` 给图像打时间戳，而非直接使用传感器采集时间。预检测得的是消息时间差和传输年龄，不能据此宣称硬件同步或曝光到输出延迟达标。
- 驱动有深度缩放路径；回调修改 CameraInfo 的尺寸并不等于完整校准变换已验证。首轮保持原生分辨率，核对实际 K/P；不能只改宽高或 frame 通过检查。

本轮保持厂商原包不变，未启动相机、未访问或部署小车。RGB 来源、校正方法和真实配准仍是上车前的待验事项。

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

## 2026-09-20 本机输入预检回归

Linux ARM64 Humble 8 包编译完成（7.26 秒），21 项 A、11 项 B、14 项语音逻辑通过。B 合成测试验证正常 RGB-D、错 frame、陈旧时间戳、断流、80 ms 不配对与缺流 CLI JSON，原有选人/深度/epoch/Marker 和 A/B/demo/非法路由回归通过。日志：`artifacts/route-b-input-test.log`。这些是本机合成验证，没有新增真实相机或真人识别证据。

## 2026-09-20 YOLO26s 验证结果

- Mac Python 3.12：Ultralytics 8.4.156 + thop 2.1.6，43 项依赖兼容性检查通过；官方权重 SHA-256 匹配。
- 实际 YOLO26s 的空图、官方 bus.jpg 检测/ByteTrack/reset 通过，实际 end2end=true；重复四帧检测 4 人，ID 均为 1–4。旧 YOLO11n 常规 NMS 对比入口通过。日志 `artifacts/yolo26s-model-smoke.log` 与 `artifacts/yolo11-baseline-regression.log`；静态图重复输入不能证明动态跟踪能力。
- Linux ARM64 Humble 8 包编译（8.56 秒）、21 项 A + 13 项 B + 14 项语音逻辑、合成 RGB-D/预检与 A/B/demo/非法路由回归通过，日志 `artifacts/yolo26-ros-regression.log`。
- TensorRT 仅完成导出计划和平台保护检查；引擎、Jetson GPU 性能、真实相机与真人验收未完成。

## 2026-09-20 TF / 并发兼容回归

8 包 Linux ARM64 Humble 编译（7.28 秒）、48 项逻辑测试、TF 专项、阻塞推理并发专项和 A/B/demo/非法路由回归通过。未标定兼容测试明确断言原光学 TRACKING、2 m 位置、检测图和 Marker ADD 均正常，而新增 base 位置无效、安装 TF 广播未启用。日志 `artifacts/route-b-tf-concurrency-final.log`。这证明合成条件下的接口隔离与状态行为，不替代实机安装标定和性能复验。
