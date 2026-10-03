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
find_package(dagger_msgs REQUIRED)
find_package(small_gicp REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(safety_scan src/safety_scan.cpp)
target_include_directories(safety_scan PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(safety_scan rclcpp sensor_msgs geometry_msgs tf2 tf2_msgs)

add_executable(low_scan src/low_scan.cpp)
target_include_directories(low_scan PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(low_scan rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_msgs)

add_executable(lidar_odom src/lidar_odom.cpp)
target_include_directories(lidar_odom PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(lidar_odom rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_msgs dagger_msgs)
target_link_libraries(lidar_odom small_gicp::small_gicp OpenMP::OpenMP_CXX)

install(TARGETS safety_scan low_scan lidar_odom DESTINATION lib/${PROJECT_NAME})

ament_package()
