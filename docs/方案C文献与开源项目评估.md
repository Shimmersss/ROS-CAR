# C 方案文献与开源项目评估

检索与核验日期：2026-10-02。

对应设计：[方案C多源人体感知融合设计](方案C多源人体感知融合设计.md)。当前软件事实：[方案C实现与验收](方案C实现与验收.md)。本文是文献与源代码适配评估，不是已实现、已编译、已实机测试的结论。

## 1. 检索范围与证据等级

用户提出四个问题：Q1 近距离深度不足时骨架辅助定位与融合；Q2 深度范围外的纯视觉三维定位；Q3 骨架辅助身份恢复；Q4 三维信息辅助骨架和人体状态。

本次筛选 6 篇 CCF A 类期刊论文，另外单列直接相关的会议论文和工程项目。按检索时 [CCF 人工智能目录](https://www.ccf.org.cn/Academic_Evaluation/AI/)与[CCF 图形学/多媒体目录](https://www.ccf.org.cn/Academic_Evaluation/CGAndMT/)核对，TPAMI、TIP 属于 A 类期刊；CCF 级别修饰刊物，不把 arXiv、GitHub 或会议版自动归入期刊。未把用户提供的 ICRA 论文、KPR 的 ECCV 论文算进 6 篇 A 类期刊。

核验使用作者论文页面、论文方法相关章节、作者仓库 README、GitHub 元数据及 IEEE 向 Crossref 登记的书目信息。重点展开 SPNet、KPR 方法，读取 SPNet 深度读取、KPR demo、Mono-RPF 跟踪系统三个源文件；其余为方法与接口初筛，不宣称所有论文全文精读或仓库全面代码审计。

9 个仓库的 README、检查时提交、元数据和上述源文件证据保存在本机 `artifacts/c-literature-20261002.json`。该目录可能被 Git 忽略；本文内保留论文、项目和提交链接，后续仅同步源码时不要误以为证据快照已随 Git 分发。没有下载模型、安装环境、运行外部代码或访问小车。

## 2. A 类期刊候选总表

| 编号 | 论文与期刊版本 | 主要对应问题 | 本项目判断 |
|---|---|---|---|
| J1 | Scale Propagation Network for Generalizable Depth Completion，TPAMI 2025，47(3):1908–1922 | Q1；辅助 Q4 | 有真实深度锚点时的补全候选，先离线比较 |
| J2 | Metric3Dv2: A Versatile Monocular Geometric Foundation Model for Zero-shot Metric Depth and Surface Normal Estimation，TPAMI 2024，46(12):10579–10596 | Q2；辅助 Q1/Q4 | 地面几何不可用时的学习式深度对照，不作为默认替代 |
| J3 | Learning Generalisable Omni-Scale Representations for Person Re-Identification，TPAMI，作者标注 2021，正式卷期 2022，44(9):5056–5069 | Q3 | 优先评估 OSNet/OSNet-AIN 外观基线 |
| J4 | HybrIK-X: Hybrid Analytical-Neural Inverse Kinematics for Whole-Body Mesh Recovery，TPAMI 2025，47(4):2754–2769 | Q4；辅助 Q1 | 学习人体结构约束的思路；完整模型列为离线研究对照 |
| J5 | Skeleton-Based Action Recognition With Multi-Stream Adaptive Graph Convolutional Networks，TIP 2020，29:9532–9545 | Q4 | 借鉴关节、骨段、运动多流，需动作数据适配 |
| J6 | Constructing Stronger and Faster Baselines for Skeleton-Based Action Recognition，TPAMI 2023，45(2):1474–1488 | Q4 | EfficientGCN 小模型候选；官方 GitHub 已迁移代码 |

以上“对应问题”和优先级是针对 ROSCAR 的工程判断，不是论文在 Gemini/Jetson 上验证过的结论。

## 3. 各论文可采用内容与限制

### J1：SPNet——深度空洞补全

[作者论文](https://arxiv.org/abs/2410.18408) · [期刊 DOI](https://doi.org/10.1109/TPAMI.2024.3513440) · [作者代码](https://github.com/Wang-xjtu/SPNet)

方法使用 RGB 与稀疏深度，通过保持尺度信息的 SP-Norm 做跨场景深度补全。作者实验包含结构光空洞，而不只包含随机稀疏点，因此与当前深度缺失问题较贴近。

建议作为骨架引导采样之后的可选补充：先保留真实观测，再对缺失位置提供模型估计，比较其是否改善关节可用率和目标距离。作者提供 Tiny/Small/Base/Large，名字为 Tiny 不等于已满足 Orin Nano 并行负载要求。

源码 `test_utils.py` 将 16 位 PNG 除以 65535；README 要求按 max_depth 归一化（室内示例 20 m）。当前 C 是 32FC1 米，接入必须显式转换和恢复，不能直接照搬读取函数。不能把填补值写回原始相机深度冒充实测；人体轮廓、人与墙接触处需独立检查。人体完全无深度锚点时，不能把其他区域的深度当作该人的实测距离。

### J2：Metric3Dv2——无深度时的米制几何预测

[作者论文及卷期](https://arxiv.org/abs/2404.15506) · [期刊 DOI](https://doi.org/10.1109/TPAMI.2024.3444912) · [作者代码](https://github.com/YvanYin/Metric3D)

模型通过规范相机空间处理不同内参，输出单目米制深度和表面法向。仓库提供 v2-S 和 ONNX 路径，但 ONNX 存在不代表目标 JetPack/TensorRT 上已经兼容或足够快。ConvNeXt-Tiny 是仓库中的 v1，不能误标成 v2-S。

建议先作为离线第三种信息源，与真实深度、地面反投影交叉比较。处理图像缩放/裁剪时必须同步更新内参；法向可辅助地面分析，但法向本身不等于完整地面平面或相机离地高度。绝对尺度依靠学习和相机模型，仍可能偏移；需用近距离真值校验，不能承诺 3 m 外可无条件准确测距。

几何条件满足时优先保留可解释的地面定位；脚不可见、人体非站立、地面不成立时，再评估学习式估计是否提供足够可靠的补充。

### J3：OSNet / OSNet-AIN——轻量外观身份基线

[作者期刊扩展版](https://arxiv.org/abs/1910.06827) · [期刊 DOI](https://doi.org/10.1109/TPAMI.2021.3069237) · [作者代码 Torchreid](https://github.com/KaiyangZhou/deep-person-reid)

区别于 ICCV 2019 的原 OSNet 论文，这篇期刊扩展研究跨数据集泛化，使用多尺度外观特征和实例归一化。它提供外观 embedding，不直接恢复机器人目标身份；仍需本项目的目标库、时空门控和重新确认规则。

建议先选仓库支持的轻量 OSNet 配置做基线，另与 OSNet-AIN 比较泛化；缩小模型后不能直接继承期刊完整模型精度。沿用 ByteTrack，不必立即更换跟踪器；在丢失、重现和有歧义时调用 ReID，并定期检查可能的静默 ID 切换。仅“丢失后才调用”无法发现跟踪器没有报 LOST 的换人。

普通 OSNet 不以关键点为输入。骨架裁剪/筛选是本项目的附加策略；多人挤在同一框中时，整框 embedding 仍会混入遮挡者。作者 README 的旧 Python/CUDA 安装命令不适用于直接覆盖 JetPack 6.2，应复用兼容环境移植推理。

### J4：HybrIK-X——结构合理的三维人体

[作者论文](https://arxiv.org/abs/2304.05690) · [期刊 DOI](https://doi.org/10.1109/TPAMI.2025.3528979) · [作者代码](https://github.com/jeffffffli/HybrIK)

将三维关节与人体参数模型通过解析/学习混合逆运动学连接，帮助得到结构合理的姿态；HybrIK-X 扩展到手和脸。其相关性是 Q4 的关节结构约束，不是跌倒分类器。

当前 C 只有 COCO17；HybrIK-X 不是将现有 17 点直接传入就可无缝补全的插件，还涉及图像模型、关节定义、人体模型文件和坐标约定。建议先借鉴骨长/关节角约束，对照真实 RGB-D；完整 mesh、人脸和手部输出不是当前跟随/跌倒任务的必要负载。

即使恢复出合理人体，也不证明它在相机坐标中的绝对平移/尺度正确。需分别评估关节相对误差、绝对根位置和离地高度；模型推断不覆盖真实关节。单帧视频 demo 不自动具备时序稳定性，仍须测试快速转身、跌倒、遮挡等场景。

### J5：MS-AAGCN——用关节、骨段及运动识别动作

[作者论文](https://arxiv.org/abs/1912.06971) · [期刊 DOI](https://doi.org/10.1109/TIP.2020.3028207) · [作者仓库](https://github.com/lshiwjx/2s-AGCN)

方法同时利用关节、骨段及其运动，并学习骨架连接和注意力。仓库名与标题仍为 CVPR 2019 的 2s-AGCN，README 明确新增 AAGCN；应验证模型/配置对应哪个版本，不能把运行旧 2s-AGCN 写成复现了期刊全部多流结果。

对 C 的价值是将单帧阈值扩展到动作时序。不能直接把现有 COCO17 喂给 NTU25 配置，也不能把未知关节填零后视为可靠观测。需处理关节映射、缺失掩码、置信度和来源，收集站立/坐蹲/缓慢躺下/跌倒/弯腰/遮挡等样本并适配类别。

动作识别常用的根节点归一化会移除绝对高度，因此应另保留离地高度、垂直速度等物理特征。预训练动作准确率不等于现场跌倒召回率；在线使用仅含过去帧的窗口，计入窗口和确认阶段的触发延迟。

### J6：EfficientGCN——时序动作模型的效率对照

[作者论文](https://arxiv.org/abs/2106.15125) · [期刊 DOI](https://doi.org/10.1109/TPAMI.2022.3157033) · [原官方 GitHub](https://github.com/yfsong0709/EfficientGCNv1)

通过输入分支融合、网络模块和宽深度缩放获得 EfficientGCN 系列，适合纳入端侧骨架动作模型的候选比较。但本次实查 GitHub 仅保留迁移说明，代码指向作者的 [Gitee 仓库](https://gitee.com/yfsong0709/EfficientGCNv1)；本轮未核验迁移站代码/权重可用性，不将它列为已具备可直接使用的 GitHub 推理工程。

本项目应先实现可靠三维特征和规则，再在相同划分、相同因果窗口下比较 MS-AAGCN/EfficientGCN 的收益。没有必要为凑模型数量同时上车运行两套动作网络。

## 4. 直接相关的工程补充（不计入 A 类期刊数量）

### E1：KPR——最贴近“骨架辅助 ID”的新候选

[ECCV 2024 论文](https://arxiv.org/abs/2407.18112) · [作者代码](https://github.com/VlSomers/keypoint_promptable_reidentification)

KPR 将图像与目标关键点作为输入，其他人的关键点可作为负提示；输出身体各部位的外观特征及可见性，只比较双方共同可见部位。这与“骨架指定同一检测框里到底认谁”直接对应，仍属于外观重识别，而不是把骨架形状当唯一身份。

已查看 `demo.py`：输入包含 `image`、`keypoints_xyc`、`negative_kps`，输出包含 `embeddings`、`visibility_scores`、`parts_masks`，可用 C 的 Pose 输出构造适配层；必须把关键点变换到同一裁剪/缩放坐标。错误关键点提示会引导到错误人体，应有质量门控。

作者 demo 明确提示跨域泛化未在其研究中得到保证，仓库使用 Swin，不能预先断言比 OSNet 更适合 Orin。建议 OSNet 做基础对照，KPR 在多人遮挡录像上比较误绑定率、IDF1、恢复延迟和计算开销。无共同可见部位时应输出身份未知，而非强制选择最高分候选。

### E2：Mono-RPF——地面/关节几何的主要参考

[ICRA 2023 论文](https://medlartea.github.io/files/mono_rpf_track.pdf) · [作者代码](https://github.com/MedlarTea/Mono-RPF)

这是当前 Q2 最直接的几何起点。保留 YOLO26s-pose，移植观测方程、个体关节高度模型和 UKF 思路到 ROS 2；补充真实观测时间、移动相机补偿、姿态退出和有符号交点拒绝。

不能照搬整仓库。检查时提交的 `mono_tracking/include/mono_tracking/track_system.hpp` 仍依赖 ROS 1；有 `joints_heights.reserve(4)` 后直接索引写入、未先调整 vector size 的静态越界风险，以及过程噪声交叉项重复索引写入。以上为局部静态发现，未执行验证；它们进一步支持“重建并测试几何核心”的选择，不构成论文方法无效的结论。

### E3：ros2_vision_inference——Metric3D 的 ROS 2 接入参考

[项目代码](https://github.com/Owen-Liuyuxuan/ros2_vision_inference)，由 Metric3D 作者 README 链接的第三方 ROS 工程，不是论文作者本人的完整机器人系统。

README 说明支持 Humble、ONNX、Metric3D 点云，并声明可用 TensorRT/Jetson；本项目没有验证这些声明。其 Metric3D 示例接口列出 616×1064 输入及投影矩阵等参数，不能假定与 C 的 640×480/相机内参契约直接一致。只借鉴异步推理、预处理和发布结构，仍需复核源时间戳、陈旧队列、模型实际导出版本及内存预算。预测深度应发布独立话题，不冒充配准相机深度。

## 5. GitHub 核验快照

下列提交是调研时固定的阅读对象，不代表批准作为项目依赖；pushed_at 是 GitHub 仓库元数据，不等于最新算法发布日期。代码、权重、人体模型、数据集的授权范围分别核对。这里只记录影响选型的现状。

| 仓库 | 阅读提交（短 SHA） | GitHub 最后 push 日期（UTC） | 适配注意 |
|---|---|---|---|
| [SPNet](https://github.com/Wang-xjtu/SPNet/tree/b836bd044517b33d3737094acd6a1f09c2362f04) | b836bd0 | 2025-04-01 | API 未识别许可证；有模型链接，未下载 |
| [Metric3D](https://github.com/YvanYin/Metric3D/tree/eb5b6fac0dc155e4e52f576e304fbf11655ff339) | eb5b6fa | 2025-03-13 | 代码 BSD-2-Clause；ONNX 路径存在，Jetson 未测 |
| [Torchreid](https://github.com/KaiyangZhou/deep-person-reid/tree/f8cd150fdf77e8d9e1ed143b7f308c2c609ded50) | f8cd150 | 2026-01-09 | 代码 MIT；旧环境说明需适配 |
| [HybrIK](https://github.com/jeffffffli/HybrIK/tree/c281eeeb3c0689a4d619a06ed0c4488e791eda76) | c281eee | 2025-01-08 | 代码 MIT；人体模型文件另行核对 |
| [2s-AGCN](https://github.com/lshiwjx/2s-AGCN/tree/953c14fc10883cd869646328f5d522e9e9282063) | 953c14f | 2021-10-22 | 有 LICENSE，API 为 NOASSERTION；旧 PyTorch 与关节定义 |
| [EfficientGCNv1](https://github.com/yfsong0709/EfficientGCNv1/tree/35c621fdfc717c7b1a0cb2bd2930e443adf19446) | 35c621f | 2022-03-07 | GitHub 仅迁移说明，非可运行源码 |
| [KPR](https://github.com/VlSomers/keypoint_promptable_reidentification/tree/e3e6ee2ffb74fd86a39518ce9a25ff91fbd973fa) | e3e6ee2 | 2025-06-12 | README 标注 Hippocratic 自定义许可证；迁入前检查原文 |
| [Mono-RPF](https://github.com/MedlarTea/Mono-RPF/tree/7c4e2cf0bdb921a3a9d241f1f6ac01604630e97d) | 7c4e2cf | 2024-11-28 | API 未识别仓库统一许可证；ROS 1，子目录含不同许可 |
| [ros2_vision_inference](https://github.com/Owen-Liuyuxuan/ros2_vision_inference/tree/980bfdeab88fed9a27db197d5aff886970005283) | 980bfde | 2026-02-18 | API 未识别许可证；默认 onnx 分支 |

## 6. 对当前 C 的具体改进建议

### 6.1 首先修正融合的数据依赖

实施跟进：第一阶段已按此方向接入 C，原函数保留供 `fusion_enabled=false` 对照。当前实现与测试边界见 [方案C实现与验收](方案C实现与验收.md)。

调研时的原 C 路径 `yolo_person_tracker/pose.py:joints3d` 在 `reference is None` 时拒绝关节深度；`pose_node.py` 的 reference 来自当帧人体测距。这能抑制背景污染，但也意味着“目标测距失败、少数关节深度仍可靠”的情况不能帮助恢复目标位置。

建议将采样拆为“原始候选收集 → 人体关联与一致性校验 → 联合估计”，使框内区域和关节候选共同支持可靠人体深度簇，而不是先要求目标位置成功再允许关节有效。必须补多人体重叠、前景家具和孤立错误关键点测试，不能简单移除 reference 检查。

目标参考点固定后，测量模型描述身体表面/骨盆/脚下点的关系，滤波器分别处理来源、噪声和失效条件。共享 RGB/深度产生的观测存在相关性，不按独立证据反复提高置信度。发布兼容 B 的目标输出，并单独暴露更完整的人体状态。

### 6.2 四个问题的建议组合

| 问题 | 第一阶段 | 后续对照 | 不应宣称 |
|---|---|---|---|
| Q1 深度不足 | 骨架引导真实区域采样＋联合深度簇＋时序估计 | SPNet，仅将补全值作为模型观测 | 补全等于真实深度；模型自动解决背景污染 |
| Q2 无深度 | 标定地面＋接地点/站立关节高度几何＋不确定度 | Metric3Dv2-S，独立预测深度通道 | 超过 3 m 必须失效或必定有效；坐蹲仍沿用站立高度 |
| Q3 ID | ByteTrack＋独立目标身份＋OSNet＋时空门控 | KPR 的关键点提示/可见身体部位 | 骨架比例就是身份；重识别保证永不跟错 |
| Q4 状态 | 可靠三维关节＋重力/离地高度＋二维时序规则 | HybrIK-X 三维结构；MS-AAGCN/EfficientGCN 动作网络 | 地面位置就是完整三维骨架；动作预训练等于跌倒验收 |

模型不会全部同时加载。Orin Nano 8GB 还要承担现有语音/其他模块；首先测实际瓶颈，再选择按需 ReID、低频深度模型或离线对照。暂不承诺帧率、模型大小或 TensorRT 性能。

### 6.3 三维状态必须保留的信息

- 实测深度、深度补全、单目深度、人体模型、运动预测分别标注来源，缺点保持缺点。
- 单目/补全网络常输出的是可见人体表面深度；骨架定义中的关节中心可能位于体内，二者差异纳入误差模型。
- 骨长时序约束可以减少跳变，但不能制造从未观测的确定动作。跟踪 ID 或 epoch 变化后不拼接跌倒过程。
- 米制离地高度需要地面/重力基准；仅相机光学坐标不足以判断真实下降速度，尤其机器人正在运动时。
- 站立关节高度先验仅辅助站立定位。不能由它恢复“站立三维骨架”再判断是否跌倒，避免循环假设。

## 7. 可执行的验证顺序

1. **标定与数据**：先核验同步/配准、相机安装和地面；固定几个已量距的位置，覆盖可靠深度内外，记录材质、遮挡、光照及有效深度比例。动作采用已有录像或安全受控动作。
2. **轻量融合基线**：对比当前多区域采样、骨架引导采样和联合观测，报告位置误差中位数/P95、无效比例、跳变、测量年龄及恢复延迟。只有独立真值能评估深度空洞补全误差；不能用模型预测互相作真值。
3. **单目定位**：用离地接地点真值验证地面模型；对站立先验测试坐蹲/弯腰退出。深度有/无反复切换时检查参考点一致性和协方差，不以滤波平滑掩盖系统偏差。
4. **身份实验**：ByteTrack、加 OSNet、加 KPR 逐级消融；记录 IDF1、ID 切换、目标误绑定次数、找回率和恢复延迟。加入同色衣服与多人框重叠；只降低丢失但增加跟错不算改善。
5. **状态实验**：比较二维规则、加真实三维规则、加模型估计、动作网络。报告按事件的 precision/recall、每小时误报、触发延迟；不同人员/场景划分训练和测试，不能把同一录像相邻帧分到两侧。
6. **端侧评测**：固定权重与输入，在 Jetson 隔离环境测冷启动、平均/P95 推理、端到端观测年龄、内存/显存与并行负载。通过后才讨论部署，保留现有默认入口和状态接口兼容性。

## 8. 当前建议决策

优先落地的方案是“统一观测采样与过滤＋几何单目定位＋轻量外观身份层＋真实三维状态特征”。SPNet、Metric3Dv2、KPR、HybrIK-X 和动作网络先按对应问题做对照，只有相对基线有可复现收益才加入正式运行链路。

当前四个问题可以明显改善，但没有一篇论文或一个 GitHub 仓库能在未标定、无深度、遮挡和任意姿态条件下同时保证绝对位置、持久身份和跌倒状态准确。
