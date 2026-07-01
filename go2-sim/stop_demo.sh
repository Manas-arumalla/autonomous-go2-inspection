#!/usr/bin/env bash
# =============================================================================
#  Stop the autonomous-inspection demo — reliable, complete teardown.
#
#  A hard-killed `ros2 launch` does NOT always cascade to its children: a stray
#  gz-sim or ros_gz *clock bridge* can survive and fight the next run's /clock
#  (the "jump back in time" failure, CP65). This sweeps every stack component,
#  bridges included, so the next `./run_demo.sh` starts from a clean slate.
# =============================================================================
echo "Stopping the autonomous-inspection demo (full teardown incl. clock bridges)..."

# SIGTERM first (let nodes shut down cleanly), then SIGKILL the stragglers.
PATTERNS=(
  "demo.launch.py" "inspection_nav.launch" "mission_control.launch"
  "gz sim" "ros_gz_bridge" "parameter_bridge"
  "rtabmap_slam/rtabmap" "octomap_server" "pointcloud_to_laserscan"
  "nav2_" "controller_server" "planner_server" "bt_navigator" "behavior_server"
  "smoother_server" "velocity_smoother" "lifecycle_manager" "map_server"
  "rviz2 -d" "robot_state_publisher" "footprint_to_odom" "base_to_footprint"
  "state_estimation_node" "quadruped_controller" "self_filter"
  "inspection_mission" "mission_control_server" "zone_inspector" "inspection_markers"
)
for sig in TERM KILL; do
  for p in "${PATTERNS[@]}"; do pkill -"$sig" -f "$p" 2>/dev/null || true; done
  sleep 1
done

# Report what (if anything) survived.
left=$(ps -eo args 2>/dev/null | grep -E "gz sim|ros_gz_bridge|rtabmap_slam|nav2_|rviz2 -d|demo.launch" | grep -vE "grep|/bin/bash" | wc -l)
if [ "$left" -eq 0 ]; then echo "  clean — no stack processes remain."; else
  echo "  WARNING: $left process(es) still alive:"; ps -eo pid,args 2>/dev/null | grep -E "gz sim|ros_gz_bridge|rtabmap_slam|nav2_|rviz2 -d" | grep -vE "grep|/bin/bash" | sed 's/^/    /'
fi
