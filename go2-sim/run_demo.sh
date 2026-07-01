#!/usr/bin/env bash
# =============================================================================
#  Autonomous Go2 Inspection — ONE-COMMAND flagship demo
#  Brings up Gazebo (simulation) + RViz (internal planning/perception/state)
#  + RTAB-Map localization + Nav2 + the 14-service control layer, together.
#
#    ./run_demo.sh                  # maze world, Gazebo + RViz, ready for a mission
#    ./run_demo.sh mission:=true    # ALSO auto-run the full autonomous inspection mission
#    ./run_demo.sh rviz:=false      # Gazebo only
#    ./run_demo.sh use_safety:=true # + twist_mux / collision_monitor safety chain
#  Any demo.launch.py argument can be appended (world:=, map_yaml:=, zones_file:=, headless:=).
#
#  Then start a mission yourself (if not auto):
#    export T=go2_inspection_interfaces/srv/ZoneTask
#    ros2 service call /run_mission $T "{zone_id: all, read: false}"
# =============================================================================
# NB: no `set -euo pipefail` — ROS 2's setup.bash references unbound vars and returns non-zero on some
# overlays, which strict mode would treat as fatal and abort before the launch.

# resolve paths relative to this script so the demo runs from any checkout
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HERE/go2_ws"
MAPS="$HERE/maps"

export FASTDDS_BUILTIN_TRANSPORTS=UDPv4   # REQUIRED (stale DDS shm otherwise stalls the robot)
export DISPLAY="${DISPLAY:-:1}"           # a window server for Gazebo + RViz

# NB: an earlier revision capped OMP/BLAS thread pools here to curb RTAB-Map's ~170-thread CPU thrashing.
# That OVER-throttled rtabmap's ICP registration so the 88 MB-DB localization never converged (no map->odom
# at all) -- a worse failure than the thrashing. The TF-lag it was meant to fix is already handled by
# rtabmap's wait_for_transform=2.0 (CP65). So we do NOT cap threads: rtabmap gets full parallelism to
# localize quickly, exactly like the config that reliably localized in ~65 s.

# Pre-launch cleanup: a previous run that was killed hard can leave an orphan gz-sim / ros_gz clock bridge
# alive. A second /clock publisher fights the new one -> "jump back in time" -> TF buffers clear -> Nav2
# can't activate (CP65). Sweep stale stack processes first (patterns can't match this script itself).
for _p in "gz sim" "ros_gz_bridge" "parameter_bridge" "rtabmap_slam/rtabmap" "nav2_" "rviz2 -d" \
          "octomap_server" "pointcloud_to_laserscan" "robot_state_publisher" "mission_control_server"; do
  pkill -KILL -f "$_p" 2>/dev/null || true
done
sleep 1

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

# localize on the maze map by default (skip if the user pointed map_yaml at another world)
if [[ "$*" != *map_yaml:=* ]]; then
  if [[ -f "$MAPS/maze.db" ]]; then
    mkdir -p "$HOME/.ros"
    cp "$MAPS/maze.db" "$HOME/.ros/rtabmap.db"
  elif [[ ! -f "$HOME/.ros/rtabmap.db" ]]; then
    echo "NOTE: no RTAB-Map localization database found ($MAPS/maze.db is absent — it is large and not"
    echo "      committed by default). The robot will not localize until you build the map once:"
    echo "      map the maze in SLAM mode and save it (see go2-sim/RUN-SIM.md, mapping mode), then re-run."
  fi
fi
# make sure ~/.go2_maps points at the maps (no-space path the map_server/zones use)
ln -sfn "$MAPS" "$HOME/.go2_maps"

echo "Launching the autonomous-inspection demo (Gazebo + RViz). Nav2 + localization take ~90 s to come up."
cd "$WS"
exec ros2 launch go2_bringup demo.launch.py "$@"
