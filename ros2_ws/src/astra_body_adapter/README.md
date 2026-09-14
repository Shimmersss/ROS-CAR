# A：Astra 骨架适配入口

当前节点只发布 NOT_READY，不读取 SDK，也不处理 /body_posture。

后续接入步骤：
1. 在 Jetson 单独验证厂商 bodyreader 的 SDK 授权与数据输出。
2. 在隔离的厂商工作区编译所需包，保留原有许可证；不要复制整套厂商源码进本项目 src。
3. 适配 Bodyposture：lock_status==2 才接受目标，坐标从毫米转米，核对坐标轴。
4. 原消息没有 header，必须补源采集时间或明确标记其不可得；接收时刻不能冒充采集时刻。
5. 加入消息超时、异常坐标剔除及显式目标恢复策略。
6. 输出统一 TargetState；整个节点不发布 cmd_vel。

参考原目录：wheeltec_bodyreader/bodyreader/src/{main,bodydata_process,follower}.cpp。
