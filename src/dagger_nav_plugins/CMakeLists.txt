cmake_minimum_required(VERSION 3.8)
project(dagger_nav_plugins)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
set(deps rclcpp rclcpp_lifecycle nav2_core nav2_util nav2_costmap_2d pluginlib geometry_msgs angles tf2 tf2_geometry_msgs)
foreach(d ${deps})
  find_package(${d} REQUIRED)
endforeach()

add_library(dagger_nav_plugins SHARED src/sticky_goal_checker.cpp)
target_include_directories(dagger_nav_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(dagger_nav_plugins ${deps})
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS dagger_nav_plugins ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sticky_goal_checker test/test_sticky_goal_checker.cpp)
  target_link_libraries(test_sticky_goal_checker dagger_nav_plugins)
  ament_target_dependencies(test_sticky_goal_checker ${deps})
endif()

ament_export_include_directories(include)
ament_export_libraries(dagger_nav_plugins)
ament_export_dependencies(${deps})
ament_package()
