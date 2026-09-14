# 厂商参考资料索引

原目录：JP6.2_wheeltec_ros2_src_20260903/。来源为用户下载的 WHEELTEC Humble/Orin 源码包，约 2.4 GB；保持原样，本次框架不改其文件。

| 相对原目录 | 内容与已知事项 |
|---|---|
| ros2_astra_camera-master/astra_camera | 当前相机启动选用的 OpenNI 驱动，含 ARM64 库 |
| OrbbecSDK_ROS2-main | 另一驱动，需按相机支持选择 |
| turn_on_wheeltec_robot/launch/wheeltec_camera.launch.py | 相机启动及压缩重发布；不等于全部参数已在实现中生效 |
| turn_on_wheeltec_robot/config/wheeltec_param.yaml | 默认 mini_mec、astra_pro，不代表实物配置 |
| wheeltec_bodyreader/bodyreader | SDK 骨架调用、叉腰锁定、质心和简化 PD |
| wheeltec_bodyreader/bodyreader/scripts/display.py | 骨架绘图 + 平均 RGB 恢复目标 ID |
| wheeltec_bodyreader/bodyreader_msg | 原厂人体消息，Bodyposture 无 header |
| ultralytics_ros2 | 仅 predict 检测；默认交通标志模型，待改人体输入和跟踪 |
| ultralytics_ros2/model/yolo11n.pt | 推荐评估的附带权重，哈希见 models/manifest.json |
| simple_follower_ros2 | visualTracker 是 HSV 颜色目标测距，不是人体识别 |

厂商许可证不统一，保留原许可与出处；本项目的目录组织不代表重新授权。当前文档仍保留旧绝对路径链接，避免移动原包导致失效。

新环境恢复时，先克隆/复制主动项目，再从原始备份放回这个厂商目录；Git 不负责保存 2.4 GB 原包。淘宝信息目录同理作为原始资料本地备份。

## 已迁入的串口代码（2026-09-14）

`ros2_ws/src/chassis_vendor/` 保存 turn_on_wheeltec_robot、wheeltec_robot_msg 和 depend/serial_ros2（目标目录 serial）的副本，原包不变。42 个文件逐一核对内容并记录 SHA-256，保留原许可声明，排除嵌套 .git 和缓存。默认 COLCON_IGNORE，未启动、未编译；这份小型副本纳入普通 Git 和日常源码同步范围。协议与启用待办见 [串口说明](../ros2_ws/src/chassis_vendor/README.md)。
