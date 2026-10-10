# 方案 C：Gemini Pro、人体骨架与跌倒检测

C 显式选择 `route:=yolo_pose`，继承 B 的 ByteTrack、epoch ID、选人/释放、单人自动锁定、人体框内多区域深度测距、三维卡尔曼、目标 TF、雷达及其他模块。测距候选包括躯干、头肩、下半身和左右侧区域，并非只采样躯干。C 使用 YOLO26s-pose，同时输出框和 COCO17 关键点。默认项目入口仍为 B，原 A/B 不改为 Pose。跌倒功能本阶段仅输出状态与可视化，不连接语音或运动。

后续讨论已记录为 [多源人体感知融合设计](方案C多源人体感知融合设计.md)；针对深度不足、远距离定位、身份恢复和三维状态的 [文献与开源项目评估](方案C文献与开源项目评估.md)包含 6 篇 A 类期刊候选和 9 个仓库核查。2026-10-02 已接入第一阶段骨架辅助真实深度融合，2026-10-03 增加第二阶段单目地面定位（均见下节）；另新增可选 [ReID身份观察层](方案C身份ReID接入与验收.md)，输出会话内身份；可显式开启身份锁定恢复，2026-10-04 已实现独立统一接地点输出与三维姿态增强，详见 [统一定位与三维姿态](方案C统一定位与三维姿态.md)。

## 第一阶段：骨架辅助真实深度融合（2026-10-02）

C 默认启用 `fusion_enabled: true`。同一帧内先收集真实深度候选，再关联人体，最后同时产生目标测量和有效关节；解决旧路径“目标测距失败时，可靠关节也无法帮助恢复定位”的依赖问题。B 的五区域算法及数值保持原样。

1. 在人体框内采样 17 点邻域、肩髋围成并缩小的躯干区域，以及 B 的五个框内候选区域。C 不填深度空洞；其他人体框覆盖的像素被排除，包括尚无轨迹 ID 的检测框。
2. 优先使用至少两个位置分开、深度一致的肩髋点；邻域不能重复投票。同等支持的深度簇互相冲突时拒绝。肩髋支持不足时尝试骨架躯干区域，再尝试膝踝；手部孤立小片深度不能单独建立人体深度。
3. 缺少上述依据时允许区域测距兜底；若人体框重叠且没有明确骨架深度，或已有肩髋点与区域深度冲突，则拒绝。此策略不能保证识别所有前景家具：错误二维骨架和同深度遮挡物仍需实测检查。
4. 一致的区域深度可补充骨架深度中位数，不用背景像素数量压过人体关键点，也不因为同一深度图被重复采样而降低卡尔曼测量噪声。
5. C 的目标代表点统一定义为“检测框中心像素射线＋融合的轴向深度 Z”，经相机内参反投影后进入原三维卡尔曼。它是虚拟代表点，不是骨盆、质心或脚下地面点。相较原 C，X/Y 定义有变化；框本身抖动仍会影响位置。后续单目地面位置不能直接与此点混合。
6. 各关节保留自己的实测深度，按人体深度关联后输出；缺失关节仍为 NaN，不借目标/预测深度补全。目标短时预测保持沿用原时间上限和真实测量年龄；预测不会产生新关节。

在 `pose.yaml` 中配置：`fusion_min_anchor_joints=2`、`fusion_cluster_abs_m=0.15`、`fusion_cluster_rel=0.12`、`fusion_torso_scale=0.65`。深度一致性门槛为 `max(绝对门槛, 相对门槛×Z)`，均为待实测初值。将自定义 `POSE_CONFIG` 中的 `fusion_enabled` 设为 `false` 可重启恢复原 C 采样/代表点路径；不是运行时热切换。

`/perception/person_states` 各人的 `detail` 增加深度来源及拒绝原因：`pose_anchors`、`pose_torso`、`pose_lower_body`、`regions`、`invalid`。ROS 消息结构、锁定服务、epoch 和二维状态机不变；本阶段没有地面反投影、身份 ReID 或三维跌倒判断。

合成对照（人体设为 2 m、背景设为 4 m，比较滤波前测量，非实机误差评估）：

| 输入 | 原 C 目标 Z | 融合后目标 Z | 关节处理 |
|---|---|---|---|
| 仅两肩 7×7 深度有效 | 无效 | 2 m | 仅两肩有效 |
| 四个肩髋邻域 2 m，大片背景 4 m | 4 m | 2 m | 背景关节无效 |
| 完全无深度 | 无效 | 无效 | 全部 NaN |
| 无骨架、区域深度充分 | 2 m | 2 m | 全部 NaN |
| 人体框完全重叠／肩髋出现同等支持冲突 | 无效 | 无效 | 保守拒绝 |

对照证据：`artifacts/c-depth-fusion-comparison.json`，另包含与更新前 B 算法逐值一致的 200 个固定随机种子场景。测试夹具见 `test_fusion.py`；软件回归结果见 WORKLOG。合成成功不代表实际 Gemini 定位精度提升已经验收。

## 第二阶段：单目地面定位（2026-10-03）

独立复核发现的四项问题已修正并补充反例测试，历史问题和修复对应关系见 [独立验收记录](方案C单目地面定位独立验收.md)。软件回归不替代相机/真人测距验收。

独立节点 `ground_localizer` 随 `route:=yolo_pose` 一并启动，只订阅 `person_states`、锁定目标 `target_state` 和 CameraInfo，不依赖深度；不修改原 `target_state`、`target_state_base`，不接入运动或语音。参考点是人的脚下接地点（z 为已确认地面高度），不同于 RGB-D 的框中心射线代表点，两者不混入同一滤波。

- 默认只接受两侧可靠、可见且几何一致的脚踝射线，以 `ankle_height_m` 作踝高补偿。缺少一侧脚踝时不使用框底；`allow_box_bottom_fallback=true` 明确报错，因为现有输入没有证明框底接地的依据。
- 身高先验默认关闭。启用需 `enable_height_prior=true`、`height_prior_confirmed=true`，并明确填写 `height_prior_track_id`（例如 `0:7`），配置该人的身高及误差；仅该轨迹持续站立至少 `standing_stable_s=0.3` 秒时可用。UNKNOWN/坐蹲不使用站立先验；换人、epoch改变、断流/时间倒退会重新验证或拒绝。配置为启动时读取，不是动态服务。
- 脚踝/肩髋候选聚合前做两两一致性检查：距离不得超过 `min(consistency_max_m, 0.4m + consistency_sigma × hypot(std_a,std_b))`；默认0.8m硬上限、2倍误差系数，0.4m允许初始身体横向分离。冲突拒绝且不再借另一来源掩盖；这些是待现场调整的保守初值，不保证所有错误姿态都能识别。
- 拒绝近平行射线、背后/超范围交点、边缘截断关节及过大不确定度。`std_m` 由像素/高度误差传播及候选离散程度共同构成，不因重复采样降低；它以脚接地、地面与安装正确为条件，**不包含所有模型/标定误差**。双脚可见不等于证明双脚着地，跳跃、抬腿及台阶仍需实机反例验证。
- 每人独立卡尔曼，逐样本噪声。缺点可在 `position_hold_s` 内短时预测，但在生成和定时发布两个环节都按真实测量年龄核对 `min(position_hold_s,max_age_s)`，保留原测量时间戳。躺卧、跌倒、观测冲突立即清除预测；基于身高先验的轨迹转UNKNOWN也立即失效。
- TF 按图像观测时间查 `base_link`←光学系，缺失即无效，不用最新 TF 代替。
- 输出（`/perception/` 下）：`person_ground_states`（PersonGroundArray，全体人员）、`ground_markers`、`target_state_ground`（TargetState，`source=yolo_ground`，仅对已锁定目标；故意不是 `yolo`，避免现有跟随器误用）。偏角左为正。
- 必须同时满足 `extrinsics_calibrated` 与 `ground_plane_confirmed`（`gemini_mount.yaml` / `camera_mount.yaml` 的 `ground_localizer` 段，默认均为 false，需由现场测得的安装位姿与地面高度确认）；否则 `NOT_READY`，不发布位置。

验证：原13项地面逻辑加6项反例检查，感知逻辑共71项。`tests/test_ground_runtime.py` 覆盖延迟输入、仅定时器运行时的保持超时与躺卧立即失效；`tests/test_ground_prior_runtime.py` 覆盖指定个体、站立持续时间、UNKNOWN退出及epoch隔离。完整C专项结果以WORKLOG/日志为准。所有几何输入为合成，**没有**实际相机接地点精度结论。现场外参/地面标定、量距、坡道/台阶、RGB-D来源切换及个体身高学习仍待完成；未接跟随器。

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

保留 `TargetState.source=yolo`、原目标坐标、锁定服务和消费者契约。关键点缺失不会使有效的人体框内多区域测距失效；但自 2026-10-08 门控起，仅有区域深度时不能单独起始新轨迹，需约 0.3 秒一致确认（见方案B文档末节）。`PersonStateArray` / `PersonState` 为新增接口：

- `/perception/person_states` 的 header 为真实彩色观测时间；每个已有轨迹含 `epoch:track_id`、框、检测置信度、固定 17 点二维坐标/置信度及三维坐标/有效位、姿态、跌倒阶段和原因。
- `posture`：0 未知、1 站立、2 坐蹲、3 躺卧、4 跌倒；`fall_stage`：0 无、1 疑似、2 确认。未知二维点与无效三维点为 NaN；不得把 0 坐标当有效关节。
- 三维关节只用当前配准深度的 7×7 邻域：至少 8 个且 50% 有效像素，0.2–8 m，中位数，P90–P10 不超过 max(0.1 m, 8% Z)。与当帧人体深度相差过大、关节跳变超过 `pose_joint_jump_m + pose_joint_jump_speed_mps × 距该关节上次接受样本的时间`（默认 0.5 m + 2 m/s，参考在 `pose_max_gap_s` 后过期）、预测保持而无新测量时拒绝；被拒绝的值不作为下一帧参考，重新出现的关节也与其上次接受值比较；不填补关节深度洞，不以躯干深度代替关节。
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

2026-10-04 继续修订：二维/三维时序分开保存，避免深度切换重启二维跌倒过程；统一位置已将原躯体深度换算到共同脚下参考点后融合，短时映射有龄期与姿态限制。详见 [连续性与位置融合说明](方案C统一定位与三维姿态.md)。
