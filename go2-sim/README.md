# go2-sim — autonomous Go2 SLAM / exploration / inspection (Gazebo Harmonic + ROS 2 Jazzy)

The simulation + autonomy workspace for the Go2 autonomous-inspection project. Built on
**Gazebo Harmonic + ROS 2 Jazzy** with a **sim-agnostic autonomy stack**: the simulator is the
swappable *provider*, and the SLAM / Nav2 / exploration / inspection code depends only on the
standard ROS 2 topic contract (`/cmd_vel`, `/odom`, `/scan`, `/camera`, TF, `/clock`). The same
graph runs on the real Go2 via WendyOS (see `../apps/`). Full project overview: `../README.md`.

## What works today

- **SLAM** — RTAB-Map pure-LiDAR graph SLAM (`map→odom`, 2D grid + 3D octomap), mapping &
  localization modes.
- **Exploration** — `frontier_explorer` (C++) drives Nav2 to autonomously map an unknown facility.
- **Navigation** — Nav2 (DWB controller, frontier-friendly costmaps) over RTAB-Map's `/map`.
- **Zone segmentation** — `go2_zones` watershed-segments the grid into room polygons.
- **Inspection** — map-driven `zone_wall_follower`: walks each wall facing it (`vy`+`vyaw`), runs
  **live YOLOE** open-vocab segmentation, and stops to crop every detected gauge.
- **Reading & report** — Claude reads each crop (type · unit · value · risk) into a report.
- **Control** — a 12-service `mission_control` layer and an **MCP server** exposing it as
  natural-language Claude tools.

## Read these

- `RUN-SIM.md` — the run guide (mapping mode, inspection mode, the natural-language MCP path).
- `docs/01-DESIGN-DECISIONS.md` — ADRs (Gazebo vs MuJoCo/Isaac; the sim-agnostic stack).
- `docs/02-ROADMAP.md` — architecture + ROS 2 topic/TF contract.
- `docs/03-INSPECTION-ROADMAP.md` — the inspection pipeline design.
- `docs/04-SERVICE-LAYER.md` — the 12-service catalog (the MCP tool surface).
- `PROGRESS-LOG.md` — historical development log / checkpoints (newest first).

## Quick start

```bash
cd go2_ws && colcon build --symlink-install && source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# Mapping + autonomous exploration:
ros2 launch go2_bringup sim_mapping.launch.py world:=maze.sdf headless:=false
ros2 run go2_exploration frontier_explorer --ros-args -p use_sim_time:=true -p autostart:=true

# Inspection on a saved map (see RUN-SIM.md for the full sequence):
ros2 launch go2_bringup inspection_nav.launch.py world:=maze.sdf map_yaml:=~/.go2_maps/maze_map.yaml
ros2 launch go2_bringup mission_control.launch.py zones_file:=~/.go2_maps/maze_zones.yaml map_name:=maze
```

> Note: `../simulation/` (a separate MuJoCo locomotion experiment) and the `../go2-*-stack`/
> `../go2-autonomous-inspection-sim` directories are obsolete side-tracks, not part of this stack.
