cmake_minimum_required(VERSION 3.8)
project(capture_task_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "action/CaptureTask.action"
  "srv/RunCapture.srv" "srv/RunInspection.srv" "srv/CancelCapture.srv"
  "msg/ResultImage.msg"
  DEPENDENCIES sensor_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
