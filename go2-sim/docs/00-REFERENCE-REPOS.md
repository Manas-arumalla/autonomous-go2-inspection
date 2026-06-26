# Reference-repo analysis (Go2 SLAM / exploration / inspection)

Deep analysis of the four repos we're using as references. **Cross-cutting truth:** all
four are ROS 2 based, but **none run on our stack** (Ubuntu 24.04 / ROS 2 **Jazzy** /
Gazebo **Harmonic**) — they're Isaac, Gazebo **Classic**, or real-hardware, on Foxy/
Humble/Kilted. The good news: their **autonomy logic is sim-agnostic** — it depends only
on standard ROS 2 interfaces (`/scan` or `PointCloud2`, `/odom`, TF, `/map`
`OccupancyGrid`, Nav2 actions), so we port the *logic*, not the *plumbing*.

| Repo | Sim / target | Distro | SLAM | Exploration | Reuse value |
|---|---|---|---|---|---|
| **Go2-Inspector** | real robot (sim dead) | Kilted | RTAB-Map 3D ICP **+ slam_toolbox 2D** | **custom `frontier_explorer.cpp`** | ⭐ inspection + report + frontier |
| **exploration_go2** | Gazebo Classic (TB3) | Foxy | slam_toolbox 2D / LIO-SAM 3D | **`explorer_pkg`** (info-gain + FSM) | ⭐ exploration brain + 3D→2D projector |
| **Go2_planner_suite** | Gazebo Classic + CHAMP | Humble | MOLA LO / DLIO | **TARE** + terrain + FAR/local | advanced nav parts bin |
| **isaac-go2-ros2** | Isaac Sim | Humble | none | none | topic/TF contract only |

## Go2-Inspector (amberhandal) — ⭐ primary blueprint for exploration + inspection
- **MIT.** Real Unitree Go2 (Sport API over `unitree_ros2`/DDS); **no working sim** (its
  Gazebo path is dead Classic with no world/sensors).
- **SLAM:** RTAB-Map, **3D LiDAR ICP-only** (`Reg/Strategy 1`, `subscribe_scan_cloud`,
  voxel 0.1, point-to-plane), occupancy grid `Grid/CellSize 0.05`. **Also ships a
  `slam_toolbox` 2D fallback** (`pointcloud_to_laserscan` → `/scan` → async node) — the
  faster path for "map now."
- **Frontier exploration:** `src/frontier_explorer.cpp` (~470 lines, **drop-in**) — reads
  `/map`, BFS-clusters free-adjacent-to-unknown cells, picks nearest valid frontier,
  sends Nav2 `NavigateToPose`; 4-state FSM, start/stop services, RViz markers. Greedy
  (no info-gain). **Depends only on `/map` + Nav2 action + TF → portable to any stack.**
- **Nav2:** NavFn planner + DWB controller, **costmaps fed by a PointCloud2 VoxelLayer**
  (`config/nav2_params_rtabmap.yaml`) — a working quadruped footprint/tuning to start from.
- **Inspection (future phases):** SAM-3 over **HTTP** (periodic capture → remote segmenter
  → 3D centroid in `map` → dedup); **change detection** (NN match by label →
  NEW/MOVED/MISSING/UNCHANGED); **report** = annotated 2D PNG + 3D PLY + JSON + **PDF
  (`fpdf2`)**, with a Claude-recommendations variant (`inspection_report_llm.py`).
- **Orchestration:** `scripts/watchdog_run.py` — run mission, trap Ctrl-C, auto-export all
  artifacts to a timestamped folder. Adopt this demo UX.
- **Gotchas:** Go2 timestamps unreliable → a family of "restamper" nodes rewrite to host
  time before SLAM/sync. README ≠ code (inspection not wired into launch; fpdf2 not
  reportlab). Trust the code.

## exploration_go2 (gcairone) — ⭐ the exploration brain to port
- **MIT-intended (no LICENSE file).** Foxy + Gazebo Classic; **sim robot is a TurtleBot3,
  not a Go2** (they deliberately didn't model the quadruped — exploration logic transfers
  because both are planar Twist-driven).
- **`explorer_pkg`** (pure Python, Nav2-actions-only → distro-agnostic): `frontier_scanner.py`
  (frontier detect → **DBSCAN cluster** → 4 scoring metrics: euclidean, **true path-length
  via `ComputePathToPose`**, cluster size, **pseudo info-gain**) + `goal_manager.py` (FSM
  with **dynamic goal preemption** when a better frontier appears) + `dbscan.py`. This is
  richer than Go2-Inspector's greedy explorer — our upgrade path.
- **`src/map_projector_bayesian.cpp`** — 3D PointCloud2 → 2D **log-odds OccupancyGrid**
  (`/projected_map`); needed when we move to a 3D LiDAR map but want 2D Nav2/frontiers.
- **Gotcha:** Python frontier scan is O(H·W) loops + blocking action calls in a timer →
  slow on big maps; vectorize/C++ before trusting live.

## Go2_planner_suite (Quadruped-dyn-insp) — advanced-nav parts bin (later)
- **MIT (mixed; CMU components BSD).** Humble + Gazebo Classic + **CHAMP**. The **CMU
  Autonomous Exploration Development Environment** re-wired to Go2 over `/cmd_vel`:
  **TARE** (hierarchical frontier exploration, OR-Tools TSP) + **terrain_analysis**
  (near 4 m / far 40 m traversability, **Go2-tuned**: `vehicleHeight 0.4`) + **FAR** (visibility-
  graph global, `.vgh` save/load) + **local_planner** (motion-primitive reactive) +
  **MOLA LO** (CPU SLAM) / **DLIO** (only real CUDA). Web mission UI (named waypoints →
  `/goal_point`).
- **Gotchas:** README aspirational ("CUDA FAR planner" — no CUDA in code; "FastAPI" — it's
  Go/Gin); clone incomplete (empty/missing workspaces, hardcoded `/home/yasiru/` paths).
  Parts bin, not deployable. **Prefer MOLA (CPU) to dodge CUDA.** Lift TARE + terrain +
  local-planner params when we want richer exploration/traversability than a 2D frontier node.

## isaac-go2-ros2 (Zhefan-Xu) — topic/TF contract reference only
- **No license** (legal blocker for code reuse). Isaac Sim 4.5 + Humble; RL locomotion
  policy; **no SLAM, no Nav2, no exploration** — just sensors + `cmd_vel` bridge.
- **What to copy (design, not code):** the clean topic/TF contract — drive via one
  `cmd_vel` `Twist`; publish `Odometry`+`PoseStamped`, `PointCloud2` LiDAR, RGB/depth
  `Image`+`CameraInfo`, TF `map→base_link→{lidar,cam}`; sensor rates decoupled from physics.
  Warehouse/obstacle worlds are the "realistic inspection world" flavor to replicate as SDF.
- **Lesson:** it deliberately stops at the sensor/`cmd_vel` boundary and delegates SLAM+nav
  to external ROS 2 packages — validating our **sim-agnostic** decomposition.

## What we lift, concretely
1. **Frontier node:** start with Go2-Inspector's `frontier_explorer.cpp` (MIT, drop-in,
   needs only `/map` + Nav2). Upgrade to exploration_go2's info-gain scoring + preemption FSM.
2. **Nav2 + costmap config:** Go2-Inspector's quadruped-tuned params as a starting point.
3. **3D→2D projector:** exploration_go2's `map_projector_bayesian.cpp` when we add 3D SLAM.
4. **Inspection + report + watchdog:** Go2-Inspector's change-detection + `fpdf2` report +
   `watchdog_run.py` (Phase 3).
5. **Advanced exploration/traversability:** TARE + terrain_analysis (Go2-tuned) from
   Go2_planner_suite (Phase 2+), MOLA for CPU SLAM.
6. **Topic/TF/world design:** isaac-go2-ros2's contract + world flavors.
