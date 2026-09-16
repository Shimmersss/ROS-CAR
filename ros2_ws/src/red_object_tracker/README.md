# 红色目标跟踪

独立 OpenCV HSV/连通区域跟踪节点，无模型权重或骨架依赖。输入为校正彩色图、配准深度、CameraInfo，默认 `depth_registered=false`，输出 NOT_READY；显式确认后自动锁定最大红块并测距。

完整参数、启动及串口边界见 [方案 A 红色物体跟随](../../../docs/方案A红色物体跟随.md)。纯算法测试在 `test/test_vision.py`，ROS 与真实驱动伪串口闭环在 `tests/test_red_serial_runtime.py`。本轮未部署小车。
