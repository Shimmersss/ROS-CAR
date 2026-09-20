# 下位机串口代码（暂存，未启用）

从本地 WHEELTEC Humble / JP6.2 原包迁入，保留来源和许可证；2026-09-20 已对 wheeltec_robot.cpp 应用安全修复。逐文件原始哈希及本地修复哈希分别见 SOURCE_MANIFEST.json 的 files 和 local_modifications。清单： [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)。

| 目录 | 用途 |
|---|---|
| turn_on_wheeltec_robot | 底盘串口收发、速度指令、里程计、IMU、电压及原厂启动配置 |
| wheeltec_robot_msg | 驱动依赖的厂商消息 |
| serial | 原包 depend/serial_ros2，ROS 包名 serial，底层串口库 |

父目录中的 `COLCON_IGNORE` 让默认 colcon 构建跳过这三个包。感知启动文件未引用它们，没有新增自动启动或串口访问。日常同步脚本会随 `ros2_ws/src` 复制本目录；本次没有向 Jetson 同步。

## 已有协议

默认设备 `/dev/wheeltec_controller`，波特率 115200。帧头 `0x7B`，帧尾 `0x7D`，BCC 为校验位之前所有字节异或。

速度发送帧共 11 字节：

| 字节下标 | 内容 |
|---|---|
| 0 | 帧头 |
| 1–2 | AutoRecharge、SecurityPLY 控制标志 |
| 3–4 / 5–6 / 7–8 | X / Y 线速度、Z 角速度，数值乘 1000，有符号 16 位，高字节在前 |
| 9 | 前 9 字节 BCC |
| 10 | 帧尾 |

输入来自 `/cmd_vel`（Twist），线速度 m/s、角速度 rad/s。标准回传帧共 24 字节，携带状态、三轴底盘速度、六轴 IMU 原始数据、电池电压及校验。具体实现见 [wheeltec_robot.cpp](turn_on_wheeltec_robot/src/wheeltec_robot.cpp)。

## 后续启用前的工作

- 核对实物下位机固件协议、车型、串口和 IMU 配置，原厂默认 mini_mec 不代表实际车型。
- 2026-09-20 已补齐独立测试容器依赖，三个包在 Linux ARM64 Humble 编译通过；尚未对修复版本进行 Jetson/串口实测。运行 `bash scripts/test_chassis_container.sh` 可在副本中编译，不移除源目录 COLCON_IGNORE。
- 整理独立底盘启动入口。原厂 launch 保留了相机、外置 IMU、超声波等条件依赖，本次未复制整个整车功能栈，不应直接把原厂整车 launch 当作已可用入口。
- 后续再接目标状态到速度控制、指令仲裁和失联停车逻辑。本次不生成车辆指令。

目前保留 `COLCON_IGNORE`。取消忽略只影响包发现和构建，并不等于启动节点。

本地修复拒绝不足/超出四项、非有限或越界机械臂输入；正常与析构路径均只发送 10 字节机械臂帧；安全扩展显式填写帧尾。用真实回调代码和串口记录替身完成 ASan/UBSan 检查。目标固件是否支持这些扩展仍待核对。
