# go2_ws — colcon workspace (ROS 2 Jazzy + Gazebo Harmonic)

Packages:
- `go2_description` — official Unitree Go2 URDF/SDF + meshes + gz sensors (LiDAR/IMU/cam)
- `go2_worlds`      — realistic Gazebo Harmonic worlds for exploration/inspection
- `go2_bringup`     — gz sim + ros_gz bridge + slam_toolbox + Nav2 launch + configs + rviz
- `go2_exploration` — custom frontier-based exploration node (autonomous map building)

Build:
    ../scripts/install_deps.sh            # one-time (sudo): Jazzy stack
    source /opt/ros/jazzy/setup.bash
    colcon build && source install/setup.bash

Entry points (added during Stage 1):
    ros2 launch go2_bringup sim.launch.py        # world + Go2 + bridge + RViz (drive it)
    ros2 launch go2_bringup explore.launch.py    # full autonomy: SLAM + Nav2 + frontier
