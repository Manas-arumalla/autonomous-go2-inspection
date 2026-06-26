cmake_minimum_required(VERSION 3.5)
project(go2_description)

find_package(ament_cmake REQUIRED)

# WendyOS port: meshes/dae/launch (Gazebo viz assets) stripped to slim the arm64 image. We only need the
# kinematic tree for robot_state_publisher -> base_link->utlidar_lidar/camera TF (it never loads geometry).
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY xacro DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()