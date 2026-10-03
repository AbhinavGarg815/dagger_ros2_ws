cmake_minimum_required(VERSION 3.8)
project(dagger_perception)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(safety_scan src/safety_scan.cpp)
target_include_directories(safety_scan PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(safety_scan rclcpp sensor_msgs geometry_msgs tf2 tf2_msgs)

add_executable(low_scan src/low_scan.cpp)
target_include_directories(low_scan PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(low_scan rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_msgs)

install(TARGETS safety_scan low_scan DESTINATION lib/${PROJECT_NAME})

ament_package()
