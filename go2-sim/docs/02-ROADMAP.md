# Architecture & roadmap — Go2 autonomy simulation

## System architecture (sim-agnostic; see ADR-002)

```
 ┌──────────────────────── SIMULATOR (swappable) ────────────────────────┐
 │  Gazebo Harmonic  ── gz topics ──►  ros_gz_bridge  ──►  ROS 2 topics    │
 │   world.sdf + Go2(SDF) + LiDAR/IMU/cam + cmd_vel base controller        │
 │   [later swap: Isaac Sim  |  MuJoCo (locomotion)  |  real Go2 over DDS]  │
 └───────────────────────────────┬───────────────────────────────────────┘
                                  │  ROS 2 TOPIC CONTRACT (the boundary)
   sensors ▼                                       ▲ cmd
   /scan (LaserScan)  /points (PointCloud2)        │ /cmd_vel (Twist)
   /imu  /odom  /tf  /clock  /camera/*             │
                                  │                 │
 ┌────────────────────────── AUTONOMY STACK (sim-independent) ────────────┐
 │  SLAM (slam_toolbox 2D → RTAB-Map 3D)  ──►  /map (OccupancyGrid), TF     │
 │  Nav2 (costmaps + planner + controller)  ──►  NavigateToPose action      │
 │  frontier_explorer  ──reads /map──►  picks frontier ──►  Nav2 goal ──► /cmd_vel │
 │  [Phase 3] perception → detections → change-detection → report           │
 └────────────────────────────────────────────────────────────────────────┘
```

## The ROS 2 topic/TF contract (every simulator must satisfy this)
- **Drive in:** `/cmd_vel` `geometry_msgs/Twist` (vx, vy, wz). *(Note: matches our MuJoCo
  `Go2Velocity.set_velocity` and the real Go2 `SportClient.Move` — same everywhere.)*
- **Sensors out:** `/scan` `LaserScan` (or `/points` `PointCloud2` → flattened), `/imu`
  `Imu`, `/odom` `Odometry`, `/camera/image_raw`+`/camera/depth`+`/camera/camera_info`
  (Phase 2+), `/clock` (sim time).
- **TF:** `map → odom → base_link → {lidar_link, imu_link, camera_link}`. SLAM owns
  `map→odom`; the sim/odometry owns `odom→base_link`; URDF/SDF owns the rest (static).
- **Frames & sim time:** everything runs with `use_sim_time:=true`.

## Phased roadmap

### ✅ Stage 0 — foundation decided & documented (done)
Reference analysis, environment audit, ADRs, this roadmap, progress log. Memory checkpointed.

### ▶ Stage 1 — official Go2 + world + SLAM + frontier exploration + autonomous map building
The current milestone. Build order (each step is a checkpoint):

1. **Workspace + deps.** Create `go2_ws` (colcon). `apt install` the Jazzy stack
   (`ros-jazzy-ros-gz`, `slam-toolbox`, `navigation2`, `nav2-bringup`,
   `pointcloud-to-laserscan`, `robot-localization`, `twist-mux`, `xacro`,
   `joint-state-publisher`).
2. **Official Go2 model in Gazebo Harmonic.** Source the official Go2 description
   (Unitree `go2_description` / Menagerie URDF) → SDF for gz-sim; add a **3D LiDAR**
   (`gpu_lidar` gz sensor, mounted like the Go2 L1), **IMU**, and joints. Package:
   `go2_description`.
3. **Locomotion base (ADR-003).** A `/cmd_vel`-driven velocity base controller so the robot
   moves (gz velocity controller / planar move). Verify teleop drives it.
4. **Bridge + bringup.** `ros_gz_bridge` config mapping gz↔ROS2 (`/scan` via
   `pointcloud_to_laserscan`, `/imu`, `/odom`, `/tf`, `/clock`, `/cmd_vel`). Package:
   `go2_bringup` with `sim.launch.py` (gz + robot + bridge + RViz).
4. **Realistic world.** An indoor/industrial SDF world (warehouse/factory w/ obstacles +
   inspectable props) — start from a gz fuel world, evolve toward the inspection scene.
   Package/assets: `go2_worlds`.
5. **SLAM.** `slam_toolbox` online-async (`use_sim_time`) on `/scan` → live `/map` + `map→odom`.
6. **Nav2.** `nav2_bringup` with a Go2-tuned param file (seed from Go2-Inspector) →
   `NavigateToPose` works from RViz goals.
7. **Frontier exploration.** Port `frontier_explorer` (from Go2-Inspector) → autonomous
   `/map` → frontier → Nav2 goal loop until no frontiers. Package: `go2_exploration`.
8. **Autonomous map building + save.** Run end-to-end; `map_saver_cli` saves the map; a
   `explore.launch.py` one-shot brings up everything. **Stage-1 demo: launch → dog explores
   the unknown world autonomously → finished map.**

**Stage-1 done = one `ros2 launch go2_bringup explore.launch.py` autonomously maps the world.**

### Stage 2 — fidelity & 3D
RTAB-Map 3D LiDAR SLAM (ADR-004); depth camera; sensor noise; realistic legged locomotion
(CHAMP-port or bridge the MuJoCo policy); `robot_localization` EKF (odom+IMU);
3D→2D `map_projector` (from exploration_go2) if keeping a 3D map for Nav2.

### Stage 3 — perception & inspection (the Wendy goal)
Object detection (YOLO on the RTX, or Claude vision over HTTP per Go2-Inspector); detection
→ 3D centroid in `map` → dedup; **change detection** (NEW/MOVED/MISSING/UNCHANGED);
**inspection report** (annotated PNG + 3D PLY + JSON + PDF via `fpdf2`, Claude-written
recommendations); `watchdog_run`-style auto-export. Info-gain frontier upgrade
(exploration_go2) and/or TARE + terrain traversability (Go2_planner_suite).

### Stage 3.5 — agentic layer (dimos)
Integrate **dimos** (agentic OS, Go2-native, MCP + Claude, NL commands) as the **top
layer**: natural-language missions ("inspect the electrical room") → dimos plans → drives
the autonomy via the **same ROS 2 contract** (Nav2 `NavigateToPose` goals / exploration
services / `/cmd_vel`) and reasons over perception. This is the **teach-it-live teammate
wedge** (cf. `winning-strategy` memory). It sits cleanly *above* SLAM/Nav2/exploration
precisely because they expose a standard contract — no autonomy changes needed. dimos is
Python/WebRTC; on hardware it coexists with the ROS 2 stack (bridge NL→Nav2 goals).

### Stage 4 — sim-to-real & scale
Swap the sim provider for the **real Go2** (`unitree_ros2`/DDS, or Wendy `go2-inspection`)
behind the same contract; optional **Isaac Sim** swap for photorealistic perception;
multi-robot / fleet; map persistence + relocalization (MOLA `.mm`).

## Cross-cutting principles (the user's "before implementation" checklist)
- **Extensibility:** one package per concern (`go2_description`, `go2_worlds`, `go2_bringup`,
  `go2_exploration`, later `go2_perception`); the topic contract is the only coupling.
- **Maintainability:** Jazzy apt binaries over source builds wherever possible; configs in
  YAML; no machine-specific hardcoded paths (the reference repos' main failure mode).
- **Performance:** 2D SLAM + CPU nav first; GPU (RTX 5060) reserved for Phase-3 perception;
  watch the 8 GB VRAM budget; sensor rates decoupled from physics.
- **Fidelity:** Gazebo PBR now; Isaac swap available later without touching autonomy.
- **Ease of development:** `sim.launch.py` (drive + look) and `explore.launch.py` (full
  autonomy) as the two entry points; everything `use_sim_time`.
- **Documentation/checkpoints:** `PROGRESS-LOG.md` updated at every milestone; ADRs for
  every significant choice; memory checkpoints for durable facts.
