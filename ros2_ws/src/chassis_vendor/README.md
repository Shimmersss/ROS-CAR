# 下位机串口代码（暂存，未启用）

从本地 WHEELTEC Humble / JP6.2 原包原样复制，保留原文件、注释和许可证声明；排除嵌套 Git 与缓存。逐文件来源及 SHA-256 见 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)。

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
- 在 Humble 环境补齐 package.xml / CMakeLists.txt 声明的依赖，再单独编译这三个包；当前迁入代码尚未编译或实机验证。
- 整理独立底盘启动入口。原厂 launch 保留了相机、外置 IMU、超声波等条件依赖，本次未复制整个整车功能栈，不应直接把原厂整车 launch 当作已可用入口。
- 后续再接目标状态到速度控制、指令仲裁和失联停车逻辑。本次不生成车辆指令。

目前保留 `COLCON_IGNORE`。取消忽略只影响包发现和构建，并不等于启动节点。
