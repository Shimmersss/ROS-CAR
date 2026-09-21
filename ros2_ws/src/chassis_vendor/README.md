# 下位机串口代码（可选构建）

最初从本地 WHEELTEC Humble / JP6.2 原包原样复制。分支 a 已修改驱动的基本收发、超时停车与扩展隔离；厂商原包未改。初始来源及 SHA-256（用于对照原始快照，不代表当前改动后哈希）见 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)。

| 目录 | 用途 |
|---|---|
| turn_on_wheeltec_robot | 底盘串口收发、速度指令、里程计、IMU、电压及原厂启动配置 |
| wheeltec_robot_msg | 驱动依赖的厂商消息 |
| serial | 原包 depend/serial_ros2，ROS 包名 serial，底层串口库 |

各包的 COLCON_IGNORE 保留，默认主动工作区不构建。执行 `bash scripts/build_chassis.sh` 在独立副本中构建三个包；新 A 仅在 `with_chassis:=true` 时启动驱动，运动仍需另行使能。本轮未向 Jetson 同步。详见 [红色方案 A](../../../docs/方案A红色物体跟随.md)。

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

## 本分支改动

- 基本帧限速发送、命令/回传 0.5 秒超时停车、多个 cmd_vel 发布者时停车、串口路径进程锁。
- 20 ms 读取超时，检查实际读取长度；滑动校验与坏帧重同步，接收异常退出且不重放缓存速度。
- 不注册机械臂、回充、灯光或安全扩展命令，退出只发基本停车帧。
- 车型必须显式给出；原厂 launch 仍是参考配置，不能替代现场核验。
- 源码原始清单不覆盖当前补丁；本机伪串口测试不等于实车验收。

合并 B 的扩展回调修复：机械臂输入长度/数值校验、10 字节发送及安全扩展帧尾均保留；这些回调仍未注册，退出只发送基本停车帧。SOURCE_MANIFEST.json 的 local_modifications 记录合并后的源码哈希。
