# Run the full simulation — Gazebo → map/nav → sweep → segment → Claude/MCP read

All commands assume this workspace. **Every terminal** needs the env + source below.
The `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` line is REQUIRED (without it stale DDS shm can stop the robot moving).

## 0. One-time setup (run once)
```bash
WS="/home/manas-reddy/Downloads/EE26 Hackathon/go2-sim"
# no-space symlink the map_server + mission use:
ln -sfn "$WS/maps" ~/.go2_maps
# load the pre-built facility map so the robot can localize (skip if you re-map in step 3):
cp "$WS/maps/facility_inspection_rtabmap.db" ~/.ros/rtabmap.db
# (first time only) build:
cd "$WS/go2_ws" && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```

## Per-terminal header (paste at the top of EVERY terminal)
```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
cd "/home/manas-reddy/Downloads/EE26 Hackathon/go2-sim/go2_ws"
source /opt/ros/jazzy/setup.bash && source install/setup.bash
# if Gazebo doesn't appear, set your display, e.g.:  export DISPLAY=:0
```

---

## ⚠️ Which path to run — CURRENT vs LEGACY

**The current / recommended inspection engine is `zone_inspector`** (samples safe viewpoints per zone →
Nav2 to each → 360° in-place spin with live YOLOE → **depth-projected 3D object positions** → Claude gauge
reading), driven by the **async service layer**. Run it via **§ G — SMALL MAZE world + the SERVICE LAYER**
below: `inspection_nav.launch.py` + `mission_control.launch.py`, then `inspect_zone` / `run_mission` (or the
14-tool **MCP** server for natural-language control). Live mission phase: `get_status` / `get_events`.

**Sections A–E are the LEGACY wall-follower path** (`mission.launch.py` / `zone_sweeper` /
`panorama_segmenter` / `yoloe_segmenter`) — superseded by `zone_inspector` (ADR-016, `docs/05-CONVERGENCE.md`)
but kept as a documented fallback. New runs should use **§ G**.

---

## A. (LEGACY wall-follower) EASIEST — full autonomous mission, one command (watch in Gazebo)
> Superseded by the `zone_inspector` service-layer path in **§ G**; kept as a fallback.

Brings up Gazebo + localization + Nav2, then runs the mission (HOME → each gauge room → sweep → segment →
read → return HOME). `zones:=zone_3` does one room; drop it for all gauge rooms.
```bash
ros2 launch go2_bringup mission.launch.py headless:=false zones:=zone_3
```
Gauge-room → zone ids: **NC=zone_3, NE=zone_0, SC=zone_2, SW=zone_1, NW=zone_10**.
(The mission starts ~100 s after launch, once localization + Nav2 are up.)

## B. Step-by-step (two terminals — see each stage)
**Terminal 1 — world + localization + navigation (GUI):**
```bash
ros2 launch go2_bringup inspection_nav.launch.py headless:=false
```
Wait ~100 s: Gazebo opens, the Go2 localizes at HOME (0,0), Nav2 goes active.

**Terminal 2 — the inspection mission:**
```bash
# (optional) live Claude reading needs a key:
# export ANTHROPIC_API_KEY=sk-...
ros2 run go2_inspection inspection_mission --ros-args -p use_sim_time:=true -p zones:=zone_3
```
You'll see the robot: Nav2 to the zone → rotate (find the gauge wall) → approach → strafe sideways across
the wall → return. Outputs land in `~/gauges/<zone>/`.

---

## C. The Claude / MCP gauge reading (the final step)
The mission auto-runs this **if `ANTHROPIC_API_KEY` is set**. Otherwise run it yourself on a swept zone:
```bash
SRC="/home/manas-reddy/Downloads/EE26 Hackathon/go2-sim/go2_ws/src/go2_inspection/go2_inspection"
# Option 1 — direct Anthropic API -> inspection_report.csv:
ANTHROPIC_API_KEY=sk-...  ~/gauge_venv/bin/python "$SRC/gauge_inspector.py" ~/gauges/zone_3
# Option 2 — MCP server for Claude Desktop/Code (then ask Claude to call get_zone_gauges):
GAUGES_ROOT=~/gauges  ~/gauge_venv/bin/python "$SRC/mcp_gauge_server.py"
```

## D. See the results
```bash
ls ~/gauges/zone_3/                                   # panorama.png, frames/, gauges/, gauges.json
xdg-open ~/gauges/zone_3/gauges_contact_sheet.png     # the cropped gauge(s)
cat  ~/gauges/zone_3/inspection_report.csv            # readings (if the Claude step ran)
cat  ~/gauges/facility_inspection_manifest.json       # whole-mission summary
```

---

## E. BONUS 1 — the validated clean perception demo (facility_gauges, 6/6)
The proven single-room pipeline (robot spawns facing a wall of 6 readable gauges). Most reliable to watch.
```bash
# Terminal 1 (GUI sim, robot in the gauge room):
ros2 launch go2_bringup rtabmap_slam.launch.py world:=facility_gauges.sdf headless:=false \
    spawn_x:=0 spawn_y:=-8 spawn_yaw:=-1.5708
# Terminal 2 (sweep -> segment -> read):
ros2 run go2_inspection zone_sweeper --ros-args -p use_sim_time:=true -p skip_nav:=true \
    -p find_wall:=false -p zone_id:=zone_1 -p half_width:=4.3
ros2 run go2_inspection panorama_segmenter ~/gauges/zone_1
GT="/home/manas-reddy/Downloads/EE26 Hackathon/go2-sim/go2_ws/src/go2_worlds/worlds/gauge_tex/gauges_groundtruth.json"
ANTHROPIC_API_KEY=sk-...  ~/gauge_venv/bin/python \
    "/home/manas-reddy/Downloads/EE26 Hackathon/go2-sim/go2_ws/src/go2_inspection/go2_inspection/gauge_inspector.py" \
    ~/gauges/zone_1 --groundtruth "$GT"
```

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
WS="/home/manas-reddy/Downloads/EE26 Hackathon/go2-sim"
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
12 services) turns the stack into single `ros2 service call`s — the future MCP tool surface
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
ros2 service call /get_status        $T "{}"     # poll: watch map_known_pct climb + plateau
ros2 service call /stop_exploration  $T "{}"
ros2 service call /save_map          $T "{}"     # -> maps/maze_map.npz + maze_zones.yaml + maze_map.pgm/.yaml + maze.db
```
### G2 — Inspect the maze (inspection mode, on the saved map)
```bash
cp maps/maze.db ~/.ros/rtabmap.db                # localize on the maze map
# T1 localization + map_server + Nav2   ros2 launch go2_bringup inspection_nav.launch.py headless:=false world:=maze.sdf map_yaml:=$HOME/.go2_maps/maze_map.yaml
# T2 service layer                      ros2 launch go2_bringup mission_control.launch.py zones_file:=$HOME/.go2_maps/maze_zones.yaml map_name:=maze
# --- drive it with services ---
ros2 service call /list_zones      $T "{}"                              # see the maze zones (pick the gauge rooms)
ros2 service call /navigate_to_zone $T "{zone_id: zone_1}"             # zone approach only
ros2 service call /inspect_zone    $T "{zone_id: zone_1, read: false}" # MAP-DRIVEN wall follow (one panorama per wall) -> YOLOE segment (read:true needs ANTHROPIC_API_KEY)
ros2 service call /run_mission     $T "{zone_id: all, read: false}"    # all gauge rooms -> one report
ros2 service call /get_zone_image  $T "{zone_id: zone_1}"             # per-wall panoramas + crop/detection paths
ros2 service call /get_report      $T "{}"
ros2 service call /navigate_home   $T "{}"
```
The same flow works on `facility_inspection.sdf` — drop `world:=maze.sdf`, `map_name:=maze`, and the
`maze_*` paths (the defaults already point at `facility_inspection_*`).

---

## Notes / current state
- **Solid:** mapping (F), facility-wide Nav2 (A/B), and the perception pipeline E (sweep→segment→read = 6/6).
- **Robot drifts/strafes sideways at launch?** It's a LEFTOVER `zone_sweeper` from a previous run still
  publishing `/cmd_vel` (the sweeper commands `linear.y` to strafe). Check + kill before launching:
  `ps -ef | grep zone_sweeper | grep -v grep` → `kill <pid>`. CHAMP holds the last command, so also send one
  stop: `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'`. (`bash /tmp/clean_kill.sh` clears all.)
- **Ground showing as voxels / in the costmap?** Fixed by two params (apply on next relaunch): octomap
  `pointcloud_min_z=0.08` drops the floor from the 3D voxel viz; `/scan` `min_height=-0.14`
  (pointcloud_to_laserscan.yaml) keeps an ~8cm floor-rejection margin so the L1's down-pitched ground
  returns stop polluting the Nav2 obstacle_layer. The SAVED rtabmap grid was already floor-clean
  (Grid/MaxGroundHeight=0.10), so a map built before this fix is still good.
- **Robot collided while strafing during a sweep?** FIXED (CP44): the strafe was open-loop (no Nav2
  collision check). The sweeper now gates STRAFE_START/SWEEP on a `/scan` lateral-clearance and HALTS before
  contact (param `side_clear`, default 0.45 m) — so a bad derived `half_width` or wrong square-up can no
  longer drive it into a wall (SWEEP ends gracefully into STITCH with the frames collected). Tune
  `-p side_clear:=<m>` if a real wall is closer than 0.45 m.
- **MODE 2 map/robot ROTATED vs the static map?** FIXED (CP45): in a near-symmetric world rtabmap was doing a
  GLOBAL relocalization at startup and snapping to a *rotated* match → the static `/map` and the live frame
  appeared rotated apart. Added `RGBD/StartAtOrigin:true` to rtabmap's **localization** params — the robot
  always respawns at HOME = the map origin, so rtabmap now starts *there* (no global-reloc snap). SLAM/continue
  mapping is untouched. Still ensure the **matching DB**: `cp maps/<map>.db ~/.ros/rtabmap.db`. (Diagnose any
  residual offset with `ros2 run tf2_ros tf2_echo map base_link` — should be ~(0,0) at HOME.) **Real-robot
  caveat:** StartAtOrigin assumes the dog is placed *exactly* at HOME — a hand-placement error becomes a fixed
  offset (no relocalization corrects it). On hardware, mark a HOME spot / publish an `/initialpose`.
- **Exploration STOPS while areas remain, or sends one long far goal?** FIXED (CP45): `frontier_explorer` now
  (a) caps each goal at `max_goal_distance` (3.0 m) — it STEPS toward distant frontiers via short reachable
  goals and re-evaluates, so far regions aren't blacklisted/abandoned; and (b) doesn't quit the instant
  frontiers look empty — it clears the (TTL) blacklist and retries (bounded), requiring `done_confirm`
  consecutive empty cycles, with the clear-budget refreshed only on real **map growth**. Tune
  `-p max_goal_distance:=<m>` (raise to ~6 for the big 30×20 facility if the short hops feel slow; the 3.0
  default suits the maze). Real-robot: in sub-0.6 m aisles, lower `-p goal_clearance:=0.22` so steps fit.
- **`/inspect_zone` is now MAP-DRIVEN (CP48):** it no longer spins 360° to *guess* the gauge wall (the old
  `FIND_WALL`, which could face the wrong wall). The new `zone_wall_follower` reads the zone **polygon** (the
  wall layout from `zones.yaml`), and for **every** real wall of that room: Nav2 to a standoff point **facing
  the wall** → `/scan` GATE (skips doorway openings) → square up → strafe the wall capturing **one panorama
  per wall** (new panorama when it rotates to the next wall). Then `yoloe_segmenter` runs YOLOE on each
  panorama (DETECT + CROP only — gauge *reading* happens downstream on the crops), writing
  `~/gauges/<zone>/panorama_NN.png`, `gauges/*.png`, `detections.json`. Concave rooms: walls you can't stand
  in front of from inside are skipped automatically. Open-loop strafe guards: `/scan` lateral gate
  (`side_clear`), a front e-stop (`front_stop`), and a no-progress/stall watchdog. Tunables:
  `-p standoff:=1.0 -p square_arc_deg:=28 -p min_wall_len:=1.2 -p max_segments:=12`. The OLD perception
  sweeper is still available as a fallback: `INSPECT_LEGACY=1 ros2 service call /inspect_zone ...` (or run
  `zone_sweeper` directly as in E). **YOLOE weights/`easyocr` are not on this laptop** → in sim it detects
  nothing and degrades gracefully (empty `detections.json`, mission continues); on the real Go2 (weights
  present, real instruments) it produces the crops. Set weights via `YOLOE_WEIGHTS=/path/to/yoloe-26s-seg.pt`.
- **Planner stuck at a corridor mouth / can't go forward (CP48):** the costmap `inflation_radius` was 0.7 m,
  which filled the narrow maze corridors edge-to-edge (gradients from both walls met → no clear lane).
  Lowered to **0.35 m** in both costmaps (`nav2_params_rtab.yaml`). Tune live without relaunch:
  `ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.35` (and `/local_costmap`).
- **Real-robot port note:** the whole stack currently runs `use_sim_time:=true`. On the real Go2 it must be
  `false` (nothing publishes `/clock`) — a known stack-wide bringup item to thread through the launch files,
  not yet done.
- **Frontier goal sits against a wall / Nav2 can't reach it?** FIXED (CP47): the explorer's `goal_clearance`
  default was 0.30 m — smaller than the footprint reach (0.35) and the costmap `inflation_radius` (0.7), so
  goals landed in the inflation and got rejected. Raised to **0.6 m** → frontiers are only chosen ≥0.6 m from
  any wall, and near-wall clusters are omitted (it looks for a freer point). Real-facility narrow (~0.9 m)
  doorways: lower with `-p goal_clearance:=0.45` (still clears the footprint) so rooms behind them aren't skipped.
- **`/get_status` known% stuck at ~80% / "stale"?** Not stale — that IS the completed coverage. The grid
  bbox always contains unknown cells beyond the walls, so `map_known_pct` plateaus well under 100% even when
  exploration is COMPLETE (see the new `map_unknown_cells` field). Polling before/after stop shows the same
  number because the map had already converged.
- **Stop everything:** `bash /tmp/clean_kill.sh` (kills sim+nav+SLAM). If the robot won't move after a crash:
  `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`.
```
```
