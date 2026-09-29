cmake_minimum_required(VERSION 3.8)
project(dagger_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PersonDetection.msg"
  "msg/PersonDetections.msg"
  "msg/PersonTrack.msg"
  "msg/PersonTracks.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
