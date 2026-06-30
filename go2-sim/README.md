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
- **Inspection** — `zone_inspector`: samples safe viewpoints in each zone, drives Nav2 to each, and does a
  360° in-place spin running **live YOLOE** open-vocab detection, projecting every detection to a **3D map
  position** through the RGBD depth camera (map-position validation + persistence dedup), cropping each
  gauge. *(The legacy `zone_wall_follower` wall-following path is kept as a fallback — see RUN-SIM.md.)*
- **Reading & report** — Claude reads each detected gauge crop (type · unit · value · risk) into the
  per-zone + facility report; `score()` grades readings vs ground truth.
- **Control** — a 14-service `mission_control` layer (async task model + `cancel_task` + a structured
  **mission event stream** surfaced through `get_status`/`get_events`) and an **MCP server** exposing it as
  natural-language Claude tools.
- **Safety** *(opt-in)* — `use_safety:=true` inserts a `twist_mux` (nav < teleop < e-stop) + Nav2
  `collision_monitor` velocity chain; default-off keeps the proven control path unchanged.
- **Benchmarking & tests** — `benchmark.py` scores detection precision/recall + localization error vs world
  ground truth; a `pytest` suite (23 tests) + GitHub Actions CI gate the ROS-free core.

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
ros2 launch go2_bringup rtabmap_slam.launch.py world:=maze.sdf headless:=false
ros2 launch go2_bringup nav2.launch.py use_sim_time:=true \
  params_file:=$(pwd)/install/go2_bringup/share/go2_bringup/config/nav2_params_rtab.yaml
ros2 run go2_exploration frontier_explorer --ros-args -p use_sim_time:=true -p autostart:=true

# Inspection on a saved map (see RUN-SIM.md for the full sequence):
ros2 launch go2_bringup inspection_nav.launch.py world:=maze.sdf map_yaml:=~/.go2_maps/maze_map.yaml
ros2 launch go2_bringup mission_control.launch.py zones_file:=~/.go2_maps/maze_zones.yaml map_name:=maze
```

> Note: `../simulation/` (a separate MuJoCo locomotion experiment) and the `../go2-*-stack`/
> `../go2-autonomous-inspection-sim` directories are obsolete side-tracks, not part of this stack.
