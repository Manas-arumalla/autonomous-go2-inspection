# Run the full simulation — Gazebo → map/nav → inspect → report

All commands assume this workspace. **Every terminal** needs the env + source below.
The `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` line is REQUIRED (without it stale DDS shm can stop the robot moving).

## 0. One-time setup (run once)
```bash
WS="$HOME/autonomous-go2-inspection/go2-sim"   # adjust to your checkout
# no-space symlink the map_server + mission use:
ln -sfn "$WS/maps" ~/.go2_maps
# load the pre-built facility map so the robot can localize (skip if you re-map in step F):
cp "$WS/maps/facility_inspection_rtabmap.db" ~/.ros/rtabmap.db
# (first time only) build:
cd "$WS/go2_ws" && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```

## Per-terminal header (paste at the top of EVERY terminal)
```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
cd "$HOME/autonomous-go2-inspection/go2-sim/go2_ws"   # adjust to your checkout
source /opt/ros/jazzy/setup.bash && source install/setup.bash
# if Gazebo doesn't appear, set your display, e.g.:  export DISPLAY=:0
```

---

## ▶ The inspection engine — `zone_inspector` + the service layer

Inspection is **`zone_inspector`**: it samples safe viewpoints per zone → Nav2 to each → a 360° in-place
spin running **live YOLOE** open-vocab detection → projects each detection to a **3D map position** through
the RGBD depth camera (map-position validation + persistence dedup) → crops every gauge → the Anthropic API
reads the crops (type · unit · value · risk). It's driven by the **async service layer** (`mission_control_server`),
exposed as natural-language tools by the **14-tool MCP server**. Run it via **§ G** below; build a map with
**§ F**. `get_status` / `get_events` report the live mission phase.

> The earlier wall-follower path (`mission.launch.py` / `zone_sweeper` / `panorama_segmenter` /
> `yoloe_segmenter`) was **retired in M6** (ADR-016, `docs/05-CONVERGENCE.md`) — `zone_inspector` supersedes
> it. The optional **safety chain** (twist_mux + collision_monitor) is `use_safety:=true` on `nav2.launch.py`.

---

## F. BUILD THE RTAB-Map 3D MAP YOURSELF + watch in RViz (~15-20 min, 4 terminals + a save)
```bash
# T1: Gazebo + RTAB-Map 3D graph-SLAM (GUI). Builds /rtabmap/cloud_map + octomap voxels + 2D /map + pose graph.
ros2 launch go2_bringup rtabmap_slam.launch.py world:=facility_inspection.sdf headless:=false \
    spawn_x:=0 spawn_y:=0 spawn_yaw:=0
# T2: RViz -- 2D map + colored 3D octomap voxels + pose graph + frontiers + plan
rviz2 -d install/go2_bringup/share/go2_bringup/rviz/rtabmap.rviz --ros-args -p use_sim_time:=true
# T3: Nav2
ros2 launch go2_bringup nav2.launch.py use_sim_time:=true \
    "params_file:=$(pwd)/install/go2_bringup/share/go2_bringup/config/nav2_params_rtab.yaml"
# T4: frontier explorer (robot maps autonomously; watch it cover all 6 rooms in RViz)
ros2 run go2_exploration frontier_explorer --ros-args -p use_sim_time:=true -p autostart:=true \
    -p robot_base_frame:=base_link
```
Save the map when done (T5, header first, while the stack is still up):
```bash
WS="$HOME/autonomous-go2-inspection/go2-sim"   # adjust to your checkout
python3 "$WS/maps/map_grab.py" "$WS/maps/my_map.npz"                                     # 2D grid -> npz
python3 "$WS/go2_ws/src/go2_zones/go2_zones/zone_segmenter.py" "$WS/maps/my_map.npz"     # -> zones.yaml + viz
python3 "$WS/maps/npz_to_map.py" "$WS/maps/my_map.npz"                                   # -> my_map.pgm + .yaml
cp ~/.ros/rtabmap.db "$WS/maps/my_map.db"                                                # RTAB-Map DB (3D + graph)
```
To run the mission on YOUR map: `cp "$WS/maps/my_map.db" ~/.ros/rtabmap.db` and point map_server/zones at
`my_map.yaml` / the new zones.yaml (or overwrite the `facility_inspection_*` names).

---

## G. SMALL MAZE world + the SERVICE LAYER (fast exploration; one-call triggers)
`maze.sdf` is a compact **12×8 m, 4-room + central-hub** world (1 gauge/room, 1 anomaly) that maps in
~2–4 min instead of facility_inspection's many minutes. The **service layer** (`mission_control_server`,
14 services) turns the stack into single `ros2 service call`s — also the MCP tool surface
(see `docs/04-SERVICE-LAYER.md`). `T` below = the srv type.
```bash
export T=go2_inspection_interfaces/srv/ZoneTask     # in every terminal you call services from
```
**(Re)generate the maze (once):** `python3 go2_ws/src/go2_worlds/scripts/gen_maze_world.py` then
`colcon build --symlink-install --packages-select go2_worlds`.

### G1 — Build the maze map (mapping mode)
```bash
# T1 sim + RTAB-Map SLAM        ros2 launch go2_bringup rtabmap_slam.launch.py world:=maze.sdf headless:=false spawn_x:=0 spawn_y:=0 spawn_yaw:=0
# T2 Nav2                       ros2 launch go2_bringup nav2.launch.py use_sim_time:=true "params_file:=$(pwd)/install/go2_bringup/share/go2_bringup/config/nav2_params_rtab.yaml"
# T3 service layer             ros2 launch go2_bringup mission_control.launch.py map_name:=maze
# T4 (optional) RViz           rviz2 -d install/go2_bringup/share/go2_bringup/rviz/rtabmap.rviz
# --- drive it with services (any sourced terminal) ---
ros2 service call /start_exploration $T "{}"
ros2 service call /get_status        $T "{}"     # poll: watch map_known_pct climb + plateau, + the mission phase
ros2 service call /stop_exploration  $T "{}"
ros2 service call /save_map          $T "{}"     # -> maps/maze_map.npz + maze_zones.yaml + maze_map.pgm/.yaml + maze.db
```
### G2 — Inspect the maze (inspection mode, on the saved map)
```bash
cp maps/maze.db ~/.ros/rtabmap.db                # localize on the maze map
# T1 localization + map_server + Nav2   ros2 launch go2_bringup inspection_nav.launch.py headless:=false world:=maze.sdf map_yaml:=$HOME/.go2_maps/maze_map.yaml
#   (add use_safety:=true to T1's nav2 for the twist_mux + collision_monitor chain)
# T2 service layer                      ros2 launch go2_bringup mission_control.launch.py zones_file:=$HOME/.go2_maps/maze_zones.yaml map_name:=maze
# --- drive it with services ---
ros2 service call /list_zones       $T "{}"                              # see the maze zones (pick the gauge rooms)
ros2 service call /navigate_to_zone $T "{zone_id: zone_1}"              # zone approach only
ros2 service call /inspect_zone     $T "{zone_id: zone_1, read: false}" # zone_inspector: viewpoints -> 360° spin -> YOLOE + depth-3D (read:true also reads gauges via the Anthropic API; needs ANTHROPIC_API_KEY)
ros2 service call /run_mission      $T "{zone_id: all, read: false}"    # all gauge rooms -> one facility report
ros2 service call /get_status       $T "{}"                            # busy / task / mission phase
ros2 service call /get_events       $T "{}"                            # full structured mission event stream
ros2 service call /get_zone_image   $T "{zone_id: zone_1}"             # viewpoint frames + crop/detection paths
ros2 service call /get_report       $T "{}"
ros2 service call /cancel_task      $T "{}"                            # abort a running mission
ros2 service call /navigate_home    $T "{}"
```
The same flow works on `facility_inspection.sdf` — drop `world:=maze.sdf`, `map_name:=maze`, and the
`maze_*` paths (the defaults already point at `facility_inspection_*`).

**Outputs** land in `~/gauges/<zone>/`: `objects.json` (deduped objects + 3D world xy + any `gauge_reading`),
`detections.json` (every observation), viewpoint frame sets, `zone_map.png`, and the per-zone report;
`~/gauges/facility_inspection_manifest.json` + `facility_map.png` + `facility_report.md` aggregate the run.
Benchmark a run vs world ground truth: `ros2 run go2_inspection benchmark <world.sdf> ~/gauges`.

---

## Notes / current state
- **Solid:** mapping (§F), facility-wide Nav2 + frontier exploration, the `zone_inspector` inspection engine
  (viewpoint + 360° spin + depth-3D detection), and the 14-service / 14-tool MCP control surface. 23 unit
  tests + CI gate the ROS-free core.
- **YOLOE detection needs weights + the CLIP text backend.** Place `~/weights/yoloe-11s-seg.pt` and build the
  prompt-embedding (PE) cache once with network (`YOLOE_ALLOW_DOWNLOAD=1`). Without them the engine **degrades
  gracefully** — it still navigates + spins every viewpoint, just captures nothing (`objects.json` with
  `available:false`). The detector load is timeout-bounded (`YOLOE_PE_TIMEOUT`, default 90 s) so a missing
  backend can never hang the node. Set weights via `YOLOE_WEIGHTS=/path/to/yoloe-11s-seg.pt`.
- **Ground showing as voxels / in the costmap?** Fixed by two params: octomap `pointcloud_min_z=0.08` drops
  the floor from the 3D voxel viz; `/scan` `min_height=-0.14` (pointcloud_to_laserscan.yaml) keeps an ~8 cm
  floor-rejection margin so the L1's down-pitched ground returns stop polluting the Nav2 obstacle_layer.
- **MODE 2 map/robot ROTATED vs the static map?** Fixed: in a near-symmetric world rtabmap was doing a
  GLOBAL relocalization at startup and snapping to a *rotated* match. Added `RGBD/StartAtOrigin:true` to
  rtabmap's **localization** params — the robot always respawns at HOME = the map origin. Still ensure the
  **matching DB**: `cp maps/<map>.db ~/.ros/rtabmap.db`. **Real-robot caveat:** StartAtOrigin assumes the dog
  is placed *exactly* at HOME — a hand-placement error becomes a fixed offset; mark a HOME spot / publish an
  `/initialpose`.
- **Exploration STOPS while areas remain, or sends one long far goal?** Fixed: `frontier_explorer`
  caps each goal at `max_goal_distance` (3.0 m) — it STEPS toward distant frontiers via short reachable
  goals — and doesn't quit the instant frontiers look empty (clears the TTL blacklist and retries, requiring
  `done_confirm` consecutive empty cycles, refreshed only on real map growth). Tune `-p max_goal_distance:=<m>`
  (raise to ~6 for the big facility; 3.0 suits the maze). Sub-0.6 m aisles: lower `-p goal_clearance:=0.22`.
- **Planner stuck at a corridor mouth:** the costmap `inflation_radius` was 0.7 m, filling the narrow
  maze corridors edge-to-edge. Lowered to **0.35 m** in both costmaps (`nav2_params_rtab.yaml`). Tune live:
  `ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.35` (and `/local_costmap`).
- **Frontier goal sits against a wall / Nav2 can't reach it?** Fixed: `goal_clearance` default raised to
  **0.6 m** → frontiers are only chosen ≥0.6 m from any wall. Narrow (~0.9 m) doorways: lower with
  `-p goal_clearance:=0.45` so rooms behind them aren't skipped.
- **`/get_status` known% stuck at ~80%?** Not stale — that IS completed coverage. The grid bbox always
  contains unknown cells beyond the walls, so `map_known_pct` plateaus under 100 % even when exploration is
  COMPLETE (see `map_unknown_cells`).
- **Real-robot port note:** the whole stack currently runs `use_sim_time:=true`. On the real Go2 it must be
  `false` (nothing publishes `/clock`) — a known stack-wide bringup item to thread through the launch files.
- **Robot won't move after a crash / stop everything:** `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`,
  and CHAMP holds the last command so send one stop: `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'`.
