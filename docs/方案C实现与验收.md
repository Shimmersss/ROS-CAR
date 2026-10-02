# 方案 C：Gemini Pro、人体骨架与跌倒检测

C 显式选择 `route:=yolo_pose`，继承 B 的 ByteTrack、epoch ID、选人/释放、单人自动锁定、躯干深度、三维卡尔曼、目标 TF、雷达及其他模块。C 使用 YOLO26s-pose，同时输出框和 COCO17 关键点。默认项目入口仍为 B，原 A/B 不改为 Pose。跌倒功能本阶段仅输出状态与可视化，不连接语音或运动。

## 设备核对与输入

2026-10-01 Mac 实测：用户所称 Gemini Pro 的 SDK 内部名称为 `SV1301S_U3`，序列号 `AY2755200PW`，固件 `RD3013`；USB 深度 `2bc5:0614`、UVC `2bc5:0511`。这仍不是实物铭牌核验。SDK 为现有 Orbbec v1.10.16，属于旧 OpenNI 设备，不套用 Gemini 2/330 的 v2 驱动。

- 原生彩色 640×480，由 AVFoundation `USB Camera` 取得一张真实 JPEG；SDK libuvc 彩色接口在 Mac 返回打开失败 -3。
- 原生深度 640×400，Y12 解包为 uint16，10 秒取得 300 帧。像素乘 `getValueScale()` 得毫米；保留的 10 张深度样本在 0.2–8 m 内有效率约 51.6%–52.2%。这只是全图统计。
- SDK 新版显式硬件同步不受支持。上述 RGB 与深度分别采集，不能描述为已验证同步 RGB-D。
- `getCameraParamWithProfile` 在只开深度时返回空内参；实际有效标定来自设备 `getCalibrationCameraParamList` 第 0 项，尺寸为 RGB 640×480 / 深度 640×400。
- 随仓库保存该设备的标定 `gemini_AY2755200PW.json`，包含原生内参、畸变和深度到彩色的外参。禁止复制给不同序列号、模式或更换固件后的设备而不重新核对。

Jetson 使用官方 [ros2_astra_camera](https://github.com/orbbec/ros2_astra_camera) 固定提交 `f7e71d9ce806e788cb48d8580aac2c778fba4214`，独立 overlay，`gemini.launch.xml`，不修改厂商已有工作区。`build_gemini_camera.sh` 补齐 upstream 未声明的依赖；Ubuntu libuvc-dev 缺失的 pkg-config 文件由已安装库的真实包版本生成。驱动原源码保持不变。

相机关闭硬件注册，`gemini_registration` 使用设备参数进行彩色去畸变、原生深度反投影、深度到彩色刚体变换及最近表面 z-buffer，输出：

| 输出 | 含义 |
|---|---|
| `/camera/color/image_rect` | 校正彩色，保留彩色输入时间 |
| `/camera/aligned_depth_to_color/image_raw` | 校正彩色像素上的 32FC1 米深度，保留深度输入时间 |
| `/camera/color/camera_info_rect` | 与校正输出一致的 K/P，零畸变 |

序列号、标定尺寸、矩阵、原始 frame、编码和时间检查失败时拒绝输出；不加载 Astra S 临时内参。`DEPTH_REGISTERED=false` 仍是默认值，转换程序运行不等于物理配准验收通过。实物边缘、已知距离、同步时差及像素对应关系留待现场验证。

换相机后使用独立 `gemini_mount.yaml`，安装外参默认未确认，不发布占位 TF。原光学输出和新车体 TF 失效边界与 B 一致。

## 模型与人体接口

官方 `yolo26s-pose.pt` 来自 assets v8.4.0，24,151,790 字节，SHA-256 `a083adb42303728ae14c4bd6bd56d80da46f82fb2564dbd6f31dcc92ea321646`。准备脚本显式下载并验哈希；ROS 缺模型报错，禁止自动下载或退回 detect。软件固定 Ultralytics 8.4.156；[官方 Pose 文档](https://docs.ultralytics.com/tasks/pose/)定义 COCO17。

```bash
python3 scripts/prepare_model.py --model yolo26s-pose.pt
.venv/bin/python scripts/test_pose_model.py
python3 scripts/export_yolo26_engine.py --task pose  # 只显示导出计划
```

Jetson 本机才允许追加 `--execute` 生成 `yolo26s-pose-fp16.engine`。默认 FP16、640×640、batch=1、静态尺寸、免 NMS；预留出口并非已有 Jetson engine 验证。

保留 `TargetState.source=yolo`、原目标坐标、锁定服务和消费者契约。关键点缺失不会使有效躯干测距失效。`PersonStateArray` / `PersonState` 为新增接口：

- `/perception/person_states` 的 header 为真实彩色观测时间；每个已有轨迹含 `epoch:track_id`、框、检测置信度、固定 17 点二维坐标/置信度及三维坐标/有效位、姿态、跌倒阶段和原因。
- `posture`：0 未知、1 站立、2 坐蹲、3 躺卧、4 跌倒；`fall_stage`：0 无、1 疑似、2 确认。未知二维点与无效三维点为 NaN；不得把 0 坐标当有效关节。
- 三维关节只用当前配准深度的 7×7 邻域：至少 8 个且 50% 有效像素，0.2–8 m，中位数，P90–P10 不超过 max(0.1 m, 8% Z)。与当帧人体深度相差过大、关节跳变超过 0.5 m、预测保持而无新测量时拒绝；不填补关节深度洞，不以躯干深度代替关节。
- 无新鲜结果时发送 `valid=false` 的空人体列表并删除三维骨架；观测时间为零表示无当前有效观测，不刷新旧姿态时间。
- `/perception/skeleton_markers` 为 MarkerArray，只连两端都有有效三维位置的骨段；轨迹消失立即 DELETE，并设置 0.5 秒寿命覆盖节点退出。
- `/perception/performance` 每秒汇总完成推理次数/FPS、RGB-D 处理平均/P95 耗时、有效输出 FPS 和观测年龄。空采样 NaN；`performance_enabled:=false` 关闭采样/发布。此处 input_fps 是被接受处理的输入，不是相机原始 FPS。

## 姿态与跌倒规则

规则是初始、可解释的图像时序判定，未经过真实跌倒数据集标定，不把其当作可靠医学或监护结论。相机运动、透视、遮挡、朝向及前后方向跌倒均可能影响效果，首版不输出绝对离地高度。

`pose.yaml` 可配置全部阈值：关键点置信度 0.5；双肩/双髋齐全才判定躯干；站立/坐蹲另需双膝。躯干对图像竖直角小于 35°，膝到髋的图像竖直距离不足框高 20% 时标记坐蹲，否则站立；躯干超过 60°且框宽高比至少 1.2 才标记横卧，中间或缺信息输出未知。

每个轨迹有最多 120 条的历史，最多维护 128 个轨迹，失联 0.5 秒清理。稳定站立或坐蹲至少 0.3 秒后，在 1 秒内出现横卧且肩髋中心比基线下降超过此前框高 25%，进入疑似。持续低位横卧 1 秒，且 `pose_upright_confirmed=true`，才确认跌倒；确认后需同 ID 连续站立 2 秒清除。遮挡/缺点中断连续计时；断流、时间倒退、epoch 改变清空历史。

初始已躺卧的人只显示躺卧，不补报跌倒。相机正常直立安装未确认时最多疑似，不确认跌倒。不得为了显示“检测成功”跳过安装确认。

## 启动与回退

先在 Jetson 构建项目与独立驱动并配置设备权限。以下为待上车命令，本轮未执行：

```bash
# 在项目根目录，沿用已准备的 Humble/应用依赖，构建到总入口读取的根目录 install：
source /opt/ros/humble/setup.bash
colcon build --base-paths ros2_ws/src --symlink-install
bash scripts/build_gemini_camera.sh
# 独立 Gemini driver 和根目录 install/setup.bash 均准备好之后：
GEMINI_SERIAL=AY2755200PW \
GEMINI_CALIBRATION="$PWD/ros2_ws/src/perception_bringup/config/gemini_AY2755200PW.json" \
MODEL_PATH="$PWD/models/weights/yolo26s-pose.pt" \
WITH_VOICE=false bash scripts/start_c.sh
```

管理入口默认要求 Jetson Pose FP16 engine；以上显式 `.pt` 用于基线验证，不会自动替换系统 CUDA 环境。`start_c.sh` 调用现有总入口继承 B 的语音/雷达/底盘开关；底盘与运动仍默认关闭，跌倒状态不消费到运动或语音。配置通过 `POSE_CONFIG=/absolute/pose.yaml` 指定。原有 B 命令不加 `PERCEPTION_ROUTE=c` 即保持 B。

独立 ROS C 感知入口：

```bash
ros2 launch perception_bringup perception.launch.py route:=yolo_pose \
  model_path:=/absolute/yolo26s-pose.pt device:=cpu \
  color_topic:=/camera/color/image_rect \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info_rect \
  camera_mount_config:=/absolute/gemini_mount.yaml
```

完成物理配准检查后才显式追加 `depth_registered:=true`。`roscar_api` 同样接受 `route:=yolo_pose`，但不启动相机；指定上述 C 输入与 Gemini 安装配置，不与已有 guard 总入口重复运行。

Foxglove 导入 `foxglove/c-layout.json`，包含骨架视频、逐人状态、三维骨架、目标状态与性能。实际客户端显示留待现场验收。

## 验证证据与待验收

- 真实设备信息和深度/彩色基础采样：`artifacts/gemini-probe.log`、`gemini-probe/`、`gemini-color.jpg`、`gemini-input-report.json`。
- 真实 Mac CPU Pose：官方 bus.jpg 4 人、每人 17 点，重复静态帧 ID 延续；Gemini 实拍图无人。实拍无人不计入人体骨架验收。`artifacts/c-model-smoke.json`。
- 算法/ROS/构建证据以 WORKLOG 最新测试结果为准。合成跌倒、合成标定及 ROS 输入只验证软件行为。
- C 及受影响 B/A/语音/入口专项：`bash scripts/test_c_container.sh`，使用已有 `roscar-humble-test` 镜像；首次先用 `deploy/humble-test.Dockerfile` 构建该镜像。完整旧套件仍为 `scripts/test_container.sh`，不能将 C 专项通过写成整个旧套件全部通过。
- 用户明确将真人站立、坐蹲、躺卧及显示验收留到下次。物理配准/量距、真实多人交叉/遮挡、真实跌倒录像误报漏报与延迟、Jetson GPU/engine 性能和实车相机安装仍待验收；本轮不访问或切换小车服务。
