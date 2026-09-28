# 语音控制路由

`voice_command_router` 先用本地规则识别常用中文操控，再把未识别文本交给
`deepseek_chat`。路由器只发布 `/chassis/cmd_vel` 的 `TwistStamped` 请求，最终
`/cmd_vel` 仍由 `motion_guard` 独占发布。

常用命令包括“开始遥控”“前进/后退/左转/右转”“停止”“开始跟随”“退出遥控”
以及“电量/当前状态”。手动模式支持 -0.15 到 0.15 m/s 的线速度和 ±0.5 rad/s
角速度；没有新的运动命令超过 2 秒会自动归零并撤销授权。启动节点不会自动 arm。

结构化输入为 `roscar_interfaces/msg/VoiceCommand`，结果为
`roscar_interfaces/msg/VoiceCommandResult`。旧的 `/voice/tool_call` 蜂鸣器 JSON
入口已从语音路由中移除；`BUZZ` 动作会被拒绝。独立的蜂鸣器解析和 GPIO
适配源码仍保留在包内，语音启动文件不会加载它们。控制命令继续经过本地校验
和 motion_guard 服务。
