#!/usr/bin/env bash
# Stage-1 dependencies for go2-sim (Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic).
# All available as Jazzy apt binaries (verified). Needs sudo.
set -euo pipefail

sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-robot-localization \
  ros-jazzy-twist-mux \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-rviz2 \
  python3-colcon-common-extensions

echo
echo "Done. Next:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  cd go2-sim/go2_ws && colcon build && source install/setup.bash"
