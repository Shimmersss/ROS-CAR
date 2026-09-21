# Foxglove 可视化：方案 A

> 方案 A 在分支 `a` 改为红色目标 + 配准深度；原 `route:=astra` 保留，B 不变。最新入口、串口与本机/实机边界见 [红色方案 A](../docs/方案A红色物体跟随.md)。本轮未部署小车。


这套可视化仅展示人体感知数据，不启动底盘、不发布 `/cmd_vel`，也不设置自启动。

```text
ASTRA S → bodyreader/main → /bodylist
       → bodylist_adapter → /perception/target_state、/perception/target_marker
       → foxglove_bridge → Wi-Fi 局域网 → Mac Foxglove
```

`bodyreader/main` 直接占用 Astra 设备，因此不要和 `astra_camera` 同时运行。现有厂商 `bodyfollow`、`bodyinteraction`、`final` launch 会包含底盘或控制节点，不能用于这里。

## 启动与连接

小车上手动启动，前台保持该终端运行：

```bash
cd /home/wheeltec/ROSCAR
bash scripts/run_astra_foxglove.sh
```

Mac 与小车连接同一 Wi-Fi 后，可检查直连地址：

```bash
cd /Users/shimmer/Documents/ChatGPT/ROSCAR
bash scripts/connect_foxglove_roscar.sh
```

在 Mac Foxglove 新建 **Foxglove WebSocket** 连接，地址填写 `ws://192.168.1.240:8765`。Bridge 监听小车所有网络接口的 8765 端口；停止小车端启动终端即可同时停止骨架、适配器和桥接。

如果 Foxglove 显示“没有数据源”，重新通过“打开连接”选择 Foxglove WebSocket 并填写上述地址。检查命令：`nc -vz 192.168.1.240 8765`，以及 `ssh roscar-wifi 'ss -ltn | grep 8765'`。离开这一路由器网络后该地址不可达，可按需使用 `scripts/open_foxglove_tunnel.sh` 作为 SSH 隧道备用方案。

仓库中的 [ssh-ros-datasource.json](ssh-ros-datasource.json) 是小车 SSH ROS 数据源清单，布局使用 [astra-layout.json](astra-layout.json)。可运行 `bash scripts/connect_foxglove_roscar.sh` 建立隧道并打印连接地址。

图像需要在 Jetson 另一个终端单独启动厂商 `astra_camera` 节点：

```bash
bash scripts/run_astra_camera.sh
```

该节点与 `bodyreader` 可能争用同一个 USB 相机；若启动后 `/camera/color/image_raw` 没有帧，应停止其中一个节点并按厂商驱动支持方式选择单一相机入口。

## 建议面板

| 面板 | 话题 / 设置 | 用途 |
|---|---|---|
| Raw Messages | `/perception/target_state` | 查看 `status`、`target_id`、`position_valid`、`detail`，确认当前不是模拟数据 |
| Plot | `/perception/target_state.horizontal_distance_m`、`bearing_rad` | 看人与车的距离和左右偏角；无效阶段应为 NaN，不是零 |
| 3D | Fixed frame=`astra_depth_optical_frame`；添加 `/perception/target_marker` | 绿色球表示锁定的人体质心；球删除表示非 TRACKING 状态 |

开始时，先让人全身进入相机画面；未叉腰时状态应为 `SEARCHING`。双手叉腰会锁定人体 ID，状态变为 `TRACKING`，距离、偏角和绿色球开始更新。人离开画面会变为 `LOST` 或 `STALE`。

默认只启骨架流，因为当前实测 RGB 与骨架流同时开启时 `/bodylist` 没有数据。RGB 需要单独排查，不是当前可视化验收的前置条件。无论哪种模式，都不应并行启动 `astra_camera`。

人体掩码面板的主题必须为 `/perception/body_mask_image`。若面板显示“正在等待图像消息”且设置中的“主题”为空，重新选择该话题；纯黑画面表示消息正常但当前 `Bodylist.count=0`、没有人体前景。仓库布局同时保留旧 `topic` 字段和当前 Foxglove 使用的 `imageMode.imageTopic`，以兼容导入。

`Bodylist` 没有原始时间戳和置信度，因此 `TargetState.observation_stamp` 为零，`measurement_age_s`、`confidence` 为 NaN。质心使用米；坐标为光学系 X 向右、Y 向下、Z 向前。Y 方向是根据厂商 SDK 行为推断，仍需在实际画面中确认。

## 开机自启

代码同步到 Jetson 后执行 `bash scripts/install_route_a_autostart.sh install`，会安装并立即启用 `roscar-route-a.service`。状态和日志分别使用 `bash scripts/install_route_a_autostart.sh status`、`bash scripts/install_route_a_autostart.sh logs`；移除使用 `bash scripts/install_route_a_autostart.sh remove`。服务固定以 `wheeltec` 用户从 `/home/wheeltec/ROSCAR` 启动，默认 `ROS_DOMAIN_ID=182`、`RGB_STREAM=false`，不启动底盘或 `/cmd_vel`。

当前已经实测骨架可输出人体 ID、质心和关节，但 SDK 仍打印授权提示；该提示未阻止本次输出，具体含义待厂商说明。Foxglove Bridge 只以 `foxglove.sdk.v1` 协议握手，已完成本机 WebSocket 握手测试；下面面板的客户端展示需要本轮实际连接验收。

新 A 导入 `red-layout.json`：标注图、红色掩码、目标、控制速度、里程计与电压。3D 面板需选择输入 CameraInfo 的实际光学 frame；默认名称仅为示例。默认未开启底盘时，速度/里程计/电压无消息属于预期。

## 原始视频与检测框视频（2026-09-16）

Foxglove 导入 `foxglove/red-layout.json`（仓库根目录下）后，上方并排显示原始彩色视频 `/perception/color_image` 与画框视频 `/perception/detections_image`，下方保留掩码、状态和底盘信息。原始帧保持相机输入的像素、编码和 header；画框帧为 bgr8 并保留相同 header，黄色框标识红色候选，绿色框仅标识同帧被 RGB-D 跟踪接受且深度有效的目标。

视频仅依赖配置的 `color_topic`，无需深度或配准确认即可显示与检测；缺少深度时 TargetState 仍为 NOT_READY，不会因此允许运动。必须有真实相机发布彩色话题才能看到实时画面。本轮完成本机代码和布局，未修改小车或在线 Foxglove 配置。

性能图表已加入 red-layout 底部：输入/输出 FPS、检测平均/P95 耗时，以及观测年龄与有效控制延迟。来源为节点每秒汇总的 RuntimeMetrics，而非 Mac 接收视频速率；没有测量样本时曲线为空，不能解释为零延迟。需要重新编译 person_interfaces 和节点包后运行；旧布局需重新导入。

N10P 雷达布局：导入 `radar-layout.json`，在现有 Bridge 连接查看 `/scan`、`/radar/points` 和 `/radar/status`。默认固定坐标 laser，不需要未标定的 base_link TF；布局文件已生成，客户端显示仍待实际连接验收。

导航布局：导入 `navigation-layout.json`，查看 `/map`、`/scan`、`/plan`、`/local_plan`、`/local_costmap/costmap`、`/navigation/follow_goal` 与导航/保护状态。沿用现有 Bridge；map/odom TF 缺失时需补齐真实定位和安装标定。布局客户端展示尚待现场验收，详见 `docs/导航与自动绕障.md`。
