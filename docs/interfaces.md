# TargetState 公共接口 v0.1

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

当前 A route 已接入真实 `/bodylist` 适配器并实现 SEARCHING/TRACKING/LOST/STALE；B route 仍固定 NOT_READY。demo 周期性展示模拟 TRACKING/LOST。

Astra 原 Bodyposture 没有 header，不得直接把回调接收时间称为传感器采集时间。真实适配时补源时间，或明确暴露时间未知。不同 optical/SDK 轴约定必须先确认后转换。

不提供通用 SetTarget 服务占位：选人方式、候选人体消息与 ID 生命周期确定后再实现，避免声明无法履约的接口。
