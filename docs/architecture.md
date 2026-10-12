# 架构与边界

更新：2026-10-11。包清单见 [README](../README.md#目录)，话题和服务的字段见 [interfaces](interfaces.md) 与 [ROS 接口使用文档](ROS接口使用文档.md)。

## 感知路线

`perception_bringup/perception.launch.py` 每次只启动一条路线，因为各路线都发布 `/perception/target_state`，同时启动会互相覆盖：

| route | 状态 | 链路 |
|---|---|---|
| `yolo_pose`（C，主线） | 软件完成，离线评估中 | Gemini RGB → YOLO26s-pose → ByteTrack/选人 → 骨架辅助深度融合 → 三维卡尔曼 → TargetState；另有 PersonState（姿态/跌倒）、地面定位、统一定位和可选 ReID |
| `yolo`（B，默认） | Jetson 在线检测 | 相机 RGB → YOLO26s → ByteTrack/选人 → 配准深度区域测距 → 三维卡尔曼 → TargetState |
| `astra`、`red` | 已退役 | Astra SDK 骨架、HSV 红色目标；代码保留，资料见 [归档](archive/README.md) |
| `demo` | 测试用 | 显式模拟数据，`is_simulated=true`，不能当作任何路线的实现证据 |

C 的光学目标（框中心射线代表点）、地面定位（脚下接地点，`yolo_ground`）和统一定位（双踝中点地面投影）各用独立话题，不混入同一个滤波器。跌倒状态只做发布和可视化，不进入语音或运动。

## 控制链路

```text
跟随 / 语音 / 外部程序 / Nav2
        │  /control/cmd_vel_request、/chassis/cmd_vel
        ▼
   motion_guard（模式、授权、雷达包络、超时、故障锁存）
        │  /cmd_vel（唯一出口）
        ▼
   底盘串口驱动（mini_akm）
```

默认 PERCEPTION_ONLY，不运动。尺寸、雷达安装和停车模型需要分别确认，再显式 arm 后才允许运动。云台设计见 [云台跟随设计与串口协议](云台跟随设计与串口协议.md)：底盘跟随目标位置而不是云台角；pan 角没有可信来源时，不发布相机到车体的 TF。

## 语音

唤醒 → 本地连续 ASR → `voice_command_router` 固定规则 → 模式服务和 `/chassis/cmd_vel` → motion_guard。路由器必须先读回 `ARMED` 状态才会发速度。见 [语音入口与回退](语音入口与回退.md)。

## 代码与验证边界

- 主动开发代码在 `ros2_ws/src/`。厂商原包保持原样且不参与构建；迁入的厂商包默认 `COLCON_IGNORE`，由独立脚本构建。
- Mac 负责源码、文档、录像和离线评估；Jetson 负责 CUDA/TensorRT、相机、配准、延迟和实车。容器编译、CI 单测、合成测试、离线回放与 Jetson 实测各自证明不同的事情，不能互相替代。
