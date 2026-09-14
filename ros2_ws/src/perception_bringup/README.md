# 启动与演示

perception.launch.py 的 route 只能为 astra、yolo 或 demo，默认 yolo。
A/B 当前仅发布 NOT_READY；只有显式 route:=demo 才会产生模拟位置。
with_foxglove:=true 时需要另行安装 ros-humble-foxglove-bridge。
当前没有自动启动相机、SDK 或底盘的行为。
