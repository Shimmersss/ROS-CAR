# navigation_bringup

可选的 SLAM Toolbox / AMCL / Nav2 人体跟随入口；详见仓库 `docs/导航与自动绕障.md`。

- `navigation.launch.py mode:=mapping|localization|external`；默认不运动、不打开底盘串口。
- `odom_tf`：使用真实里程计时间，广播平面 odom→base_footprint，兼容厂商 position.z 存航向。
- `follow_goal`：观测时刻 TF、1m 留距、目标序列化取消、目标/里程计失效锁存。
- `velocity_adapter`：仅在已授权导航状态下，把 Humble 原始 Twist 转换为时戳请求；不发布最终速度。
- `motion_guard`：仍独占 /cmd_vel。不能并开其他直接跟随请求节点。

安装配置中的外参、里程计和车辆几何确认默认均关闭。Foxglove 布局位于 `foxglove/navigation-layout.json`。运行软件测试使用 `scripts/test_navigation_container.sh`，不需连接硬件。
