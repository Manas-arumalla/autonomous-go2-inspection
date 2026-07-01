# ADR-018 — Live visualization (RViz) + one-command demo

**Status:** Implemented + verified live (2026-06-30) · purely additive (nothing existing modified)

## Context

A demonstrable autonomous-inspection system needs to show *both* the physical world (Gazebo) **and** the
robot's internal state — the map it built, where it thinks it is, the costmaps it plans on, the path it's
following, and the gauges it detected in 3D. Previously RViz was launched ad-hoc with the `rtabmap.rviz`
config (mapping-oriented) and the inspection output only existed as `objects.json` files — invisible in RViz.

## Decision

Add a **dedicated inspection RViz view + a one-command demo launcher**, composed *on top of* the existing,
unchanged launches. Nothing stable is replaced.

## What was added (all additive)

- **`go2_bringup/rviz/inspection.rviz`** — a tailored view: the Go2 RobotModel, the `/map`, **Nav2 local +
  global costmaps**, `/scan`, RTAB-Map 3D octomap + 3D LiDAR (toggle), **room zones** (`/zones`),
  **frontiers** (`/explore/frontiers`), the **Nav2 plan** (`/plan`), the robot's **camera feed**
  (`/camera/image_raw`, docked panel), and **★ detected gauges** (`/inspection/objects`). Interactive
  `2D Pose Estimate` / `2D Nav Goal` tools included for live demos.
- **`go2_inspection/inspection_markers.py`** (new node, **read-only / zero-risk**) — reads the per-zone
  `objects.json` the inspection writes and republishes every localized detection as a MarkerArray on
  `/inspection/objects`: a colour-coded sphere (green = close-read **confirmed**, amber = detected but
  unconfirmed, cyan = detected/not-approached) + a text label (class, and the gauge value if read). It
  never touches the inspection/nav nodes — it only makes their output visible.
- **`go2_bringup/launch/demo.launch.py`** — ONE command brings up the whole environment:
  `inspection_nav` (Gazebo + RTAB-Map localization + Nav2) **+ RViz** (the inspection view) **+ the marker
  node + the 14-service control layer**, with opt-in `mission:=true` (auto-run the full mission),
  `rviz:=false`, `use_safety:=true`, and `world/map_yaml/zones_file` overrides. It is *purely a composition*
  of the existing launches — every component still runs on its own.
- **`run_demo.sh`** (repo root) — a convenience wrapper: sets the env, sources the overlay, points
  localization at the maze map, and launches the demo. (`set -u` is deliberately **not** used — ROS 2's
  `setup.bash` references unbound vars and would abort it.)

## Verified live

`./run_demo.sh` brought up **Gazebo + RViz together**; RViz rendered the maze map (4 rooms + hub), the
robot, the costmaps, and the **detected gauges as green markers**, with the camera feed at 22 Hz and Nav2
reaching *active*. The only RViz log line is the harmless `indexed_8bit_image.vert` GLSL warning (RViz/Ogre
8-bit costmap texture — the costmap still renders).

## Usage

```bash
./run_demo.sh                       # Gazebo + RViz + stack (maze); then call /run_mission
./run_demo.sh mission:=true         # also auto-run the full autonomous inspection
./run_demo.sh use_safety:=true      # + the twist_mux/collision_monitor safety chain
./run_demo.sh rviz:=false           # Gazebo only   |   headless:=true => RViz only
# individual pieces still work: inspection_nav.launch.py / mission_control.launch.py / rviz2 -d inspection.rviz
```
