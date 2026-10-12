cmake_minimum_required(VERSION 3.8)
project(roscar_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetControlMode.srv"
  "srv/AuthorizeMotion.srv"
  "srv/SayText.srv"
  "msg/VoiceCommand.msg"
  "msg/VoiceCommandResult.msg"
  "msg/MotionRequest.msg"
  "msg/AudioFrame.msg"
  DEPENDENCIES builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
