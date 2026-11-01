cmake_minimum_required(VERSION 3.8)
project(camera_path_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PathRow.msg"
  "msg/PathTable.msg"
  "msg/PathIkStatus.msg"
  "msg/PlaybackState.msg"
  "msg/ArmStatus.msg"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
