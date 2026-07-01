# go2_inspection — autonomous gauge inspection (Phases 2–4)

Per-zone pipeline: **sweep a zone → stitch a panorama → FastSAM-segment the gauges → the model reads each → CSV**.
Camera-frame + odometry only (no camera extrinsic/TF), so it ports to the real Go2's external camera unchanged.
Built on top of the RTAB-Map + Nav2 + frontier mapping stack (which it does **not** modify).

## Real-time split (why this is on-device-safe)
- **In-loop, lightweight** — `zone_sweeper` drives the robot at ~10 Hz (`/cmd_vel` strafe + Nav2 macro-nav). Runs live.
- **Post-sweep, heavy, OFF the control loop** — `panorama_segmenter` (FastSAM) and the LLM reader run **once per
  zone after the sweep**, so they never stall locomotion. FastSAM-s (~23 MB) is Orin-Nano-friendly.

## Two runtimes (deps differ)
| Stage | Script | Runtime | Key deps |
|---|---|---|---|
| 2. sweep → panorama + frames | `zone_sweeper` (ROS node) | system ROS Jazzy | rclpy, nav2, tf2, opencv |
| 3. FastSAM → gauge crops | `panorama_segmenter` (standalone) | system python | ultralytics(FastSAM), torch+CUDA, opencv |
| 4. LLM read → CSV | `gauge_inspector` (standalone) | **venv** `~/gauge_venv` | anthropic |
| 4. MCP transport (MCP client) | `mcp_gauge_server` (standalone) | **venv** `~/gauge_venv` | fastmcp |

Make the venv once: `uv venv ~/gauge_venv && uv pip install --python ~/gauge_venv/bin/python fastmcp anthropic opencv-python-headless`

## Run (sim)
```bash
# Phase 2 — sweep the gauge zone (robot already localized / in SLAM). Spawns in the gauge room here:
ros2 launch go2_bringup rtabmap_slam.launch.py world:=facility_gauges.sdf headless:=true \
    spawn_x:=0 spawn_y:=-8 spawn_yaw:=-1.5708          # SLAM; provides /scan, /camera, map->base_link
ros2 run go2_inspection zone_sweeper --ros-args -p use_sim_time:=true -p skip_nav:=true \
    -p zone_id:=zone_1 -p half_width:=4.3              # -> ~/gauges/zone_1/{panorama.png, frames/, frames.json}
# (production: drop skip_nav so Nav2 drives to the zone centre first; needs a full /map)

# Phase 3 — segment the panorama into clean per-gauge crops (multi-scale FastSAM + best-frame crop):
ros2 run go2_inspection panorama_segmenter ~/gauges/zone_1     # -> gauges/gauge_NN.png + gauges.json + contact sheet

# Phase 4a — robust on-device path: the model reads each crop -> CSV (needs ANTHROPIC_API_KEY):
ANTHROPIC_API_KEY=sk-... ~/gauge_venv/bin/python \
    go2_ws/src/go2_inspection/go2_inspection/gauge_inspector.py ~/gauges/zone_1 \
    --groundtruth .../gauges_groundtruth.json          # -> inspection_report.csv (+ scoring in sim)

# Phase 4b — MCP transport: serve the crops to an MCP client over MCP:
GAUGES_ROOT=~/gauges ~/gauge_venv/bin/python \
    go2_ws/src/go2_inspection/go2_inspection/mcp_gauge_server.py     # stdio MCP server
#   then register it with your MCP client (stdio) and ask the client to call get_zone_gauges("zone_1")
```

## Output (`~/gauges/<zone>/`)
`panorama.png`, `frames/` + `frames.json` (Phase 2) · `gauges/gauge_NN.png` + `gauges.json` + `gauges_contact_sheet.png`
(Phase 3) · `inspection_report.csv` + `readings.json` (Phase 4).
CSV columns: **ID, Zone, Type, Reading, Unit, SI_Unit, Range, Risk, Confidence**.

Validated in sim (facility_gauges, 6 gauges): detection 6/6, and the model's reasoning-first reading scored
**type 6/6, unit 6/6, value 6/6 (≤5 % span)** vs ground truth.
