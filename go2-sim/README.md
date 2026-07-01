# go2-sim — autonomous Go2 SLAM / exploration / inspection

The simulation and autonomy workspace for the autonomous-inspection project, built on **Gazebo Harmonic +
ROS 2 Jazzy**. The autonomy is **sim-agnostic**: the simulator is a swappable *provider*, and the SLAM /
Nav2 / exploration / inspection code depends only on the standard ROS 2 topic contract (`/cmd_vel`,
`/odom`, `/scan`, `/camera`, TF, `/clock`), so the same node graph runs unchanged on the real robot. Project
overview and results: [`../README.md`](../README.md).

## What works

- **SLAM** — RTAB-Map pure-LiDAR graph SLAM (`map → odom`, 2D grid + 3D octomap), in mapping and
  localization modes.
- **Exploration** — a C++ `frontier_explorer` drives Nav2 to autonomously map an unknown facility.
- **Navigation** — Nav2 (DWB controller) over RTAB-Map's `/map`, with a localization gate, wedge recovery,
  and costmaps tuned for tight maze doorways.
- **Zone segmentation** — `go2_zones` watershed-segments the occupancy grid into room polygons.
- **Inspection** — `zone_inspector` samples safe viewpoints in each room, drives Nav2 to each, and runs a
  360° in-place spin with **live YOLOE** open-vocabulary detection, projecting every detection to a **3D
  map position** through the depth camera (with map-position validation, persistence de-duplication, and
  an observation-aware consolidation), and cropping each instrument. An optional detect-then-approach mode
  drives close for a high-resolution, resolution-budgeted read.
- **Mission & control** — `inspection_mission` runs the full HOME → rooms → report → HOME loop; a
  `mission_control` service layer exposes the stack as ROS 2 triggers with a structured event stream, and
  an optional natural-language control surface.
- **Reading & report** — detected gauge crops can be read (type · unit · value · risk) into the per-room
  and facility report; a scoring routine grades readings against ground truth.
- **Benchmarking & tests** — `benchmark.py` scores detection precision/recall and localization error
  against world ground truth; a `pytest` suite (38 tests) and GitHub Actions CI gate the ROS-free core.

## Run it

```bash
cd go2_ws && colcon build --symlink-install && source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# One command — Gazebo + RViz + SLAM + Nav2 + the control layer (maze world):
cd .. && ./run_demo.sh              # add mission:=true to auto-run the full mission
./stop_demo.sh                      # clean teardown
```

Individual components (mapping from scratch, inspection on a saved map, the natural-language path) are in
[`RUN-SIM.md`](RUN-SIM.md).

## Documentation

- [`RUN-SIM.md`](RUN-SIM.md) — the run guide (mapping mode, inspection mode, the natural-language path).
- [`docs/`](docs/) — architecture decision records: the sim-agnostic stack, the inspection pipeline, the
  service layer, the detect-then-approach reading, reliability hardening, and detection robustness.
- [`CHANGELOG.md`](CHANGELOG.md) — milestone history.
