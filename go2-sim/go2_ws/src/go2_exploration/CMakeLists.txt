cmake_minimum_required(VERSION 3.8)
project(go2_exploration)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(frontier_explorer src/frontier_explorer.cpp)
ament_target_dependencies(frontier_explorer
  rclcpp rclcpp_action nav_msgs geometry_msgs visualization_msgs
  nav2_msgs std_srvs tf2 tf2_ros)

add_executable(self_filter src/self_filter.cpp)
ament_target_dependencies(self_filter
  rclcpp sensor_msgs tf2 tf2_ros)

install(TARGETS frontier_explorer self_filter DESTINATION lib/${PROJECT_NAME})

ament_package()
