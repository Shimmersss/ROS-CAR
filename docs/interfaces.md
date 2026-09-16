# TargetState 公共接口 v0.1

> 方案 A 在分支 `a` 改为红色目标 + 配准深度；原 `route:=astra` 保留，B 不变。最新入口、串口与本机/实机边界见 [红色方案 A](方案A红色物体跟随.md)。本轮未部署小车。


话题：/perception/target_state；消息：person_interfaces/msg/TargetState。
A/B/demo 使用同一接口。输出描述观测状态，不是控制命令。

| 字段 | 约定 |
|---|---|
| header.stamp | 本次状态发布时间，不冒充采集时间 |
| header.frame_id | position 所属坐标系；没有有效坐标时可为空 |
| observation_stamp | 最近用于本次有效观测的源采集时间；未知为零 |
| source | astra / yolo / demo |
| is_simulated | 只有显式 demo 为 true；下游必须区分 |
| status | 0 NOT_READY、1 SEARCHING、2 TRACKING、3 LOST、4 STALE |
| detail | 可读状态原因，不用字符串代替状态机判断 |
| target_id | 当前选定轨迹 ID；不代表持久身份 |
| position_valid | 只有新鲜且通过验证的观测为 true；未来预测值不得设为 true |
| position | 米，optical 坐标：X 右、Y 下、Z 前；无效为 NaN |
| horizontal_distance_m | sqrt(X² + Z²)，不是光轴 Z；无效为 NaN |
| bearing_rad | atan2(X, Z)，向相机右侧为正；无效为 NaN |
| measurement_age_s | 发布时间减源观测时间，秒；无法计算时为 NaN |
| confidence | 0–1；不同算法不保证可直接比较，未知为 NaN |

## 状态规则

- NOT_READY：算法未初始化/未接入，不应显示为检测不到人。
- SEARCHING：数据与算法可用，尚未选定目标。
- TRACKING：持有目标轨迹；只有通过时效与质量检查的真实观测或明确模拟观测才 position_valid=true。
- LOST：输入正常，但选定目标未匹配；保留 ID 可用于解释，不保留伪装成新观测的旧位置。
- STALE：相机/上游消息过期，位置无效。由后续真实适配器负责超时检测。
- 整个节点退出时，它无法继续发布 STALE；Foxglove 或后续消费者还必须监测话题断流。

当前 A route 已接入真实 `/bodylist` 适配器并实现 SEARCHING/TRACKING/LOST/STALE；B route 已实现真实 RGB-D 节点，未配置模型/配准时 NOT_READY，有效输入时支持 SEARCHING/TRACKING/LOST/STALE（本地验证，待实机验收）。demo 周期性展示模拟 TRACKING/LOST。

Astra 原 Bodyposture 没有 header，不得直接把回调接收时间称为传感器采集时间。真实适配时补源时间，或明确暴露时间未知。不同 optical/SDK 轴约定必须先确认后转换。

B 提供 `/perception/lock_target` 与 `/perception/release_target`（std_srvs/Trigger），锁定最新画面水平中央轨迹。ID 为 epoch:track_id，流重置需显式重锁；不提供任意 ID 的 SetTarget 服务。详见 [B 实现](方案B实现与验收.md)。

## 红色方案 A 接口（2026-09-16）

`route:=red` 或 `route_a.launch.py` 发布相同 TargetState，source 为 `red_object`，target_id 为 `red:<递增编号>`；三个连续关联帧锁定，丢失一秒后重新搜索。SEARCHING/LOST/STALE/NOT_READY 及无效深度均无有效位置。

- `/perception/detections_image`：sensor_msgs/Image，bgr8 标注图。
- `/perception/red_mask_image`：sensor_msgs/Image，mono8 红色掩码。
- `/perception/target_marker`：visualization_msgs/Marker，红色目标位置；无效时 DELETE。
- 底盘可选输出 `/odom`（Odometry）、`/imu/data_raw`（Imu）、`/PowerVoltage`（Float32）；`/cmd_vel` 仅可选控制链存在时发布。

跟随器的 `expected_source` 默认 astra，新 A 显式设为 red_object；timestamped 来源必须同时通过观测时间、发布年龄和 measurement_age_s 检查，禁止把陈旧观测重发当作新观测。

## 原始视频与检测框视频（2026-09-16）

Foxglove 导入 `foxglove/red-layout.json`（仓库根目录下）后，上方并排显示原始彩色视频 `/perception/color_image` 与画框视频 `/perception/detections_image`，下方保留掩码、状态和底盘信息。原始帧保持相机输入的像素、编码和 header；画框帧为 bgr8 并保留相同 header，黄色框标识红色候选，绿色框仅标识同帧被 RGB-D 跟踪接受且深度有效的目标。

视频仅依赖配置的 `color_topic`，无需深度或配准确认即可显示与检测；缺少深度时 TargetState 仍为 NOT_READY，不会因此允许运动。必须有真实相机发布彩色话题才能看到实时画面。本轮完成本机代码和布局，未修改小车或在线 Foxglove 配置。
