# ADR-016 — Converge onto the `-main` inspection engine, then exceed it

**Status:** Accepted (2026-06-30) · **Supersedes:** the wall-follower inspection path (ADR-015 line)

## Context

An alternate `go2-ros2-inspection-main` line of the same project was a **more advanced, cleaner parallel
implementation** than the initial `go2-sim/` stack. A full review across both versions found that, on
most engineering axes, the `-main` line is ahead:

| Area | `-main` | Initial `go2-sim/` |
|---|---|---|
| Object localization | **depth → map 3D projection** (RGBD camera, deproject via K at image stamp) | coarse LiDAR bearing/range heuristic |
| Inspection strategy | viewpoint sampling + **360° in-place spin** (covers interior + walls) | wall-following (walls only) |
| False-positive control | **map+polygon position validation** + **persistence (min-observations)** dedup | weaker map-position dedup |
| Orchestration | **async task model + `cancel_task`** + token lock | blocking `subprocess.run` + timeouts |
| Code structure | **modular** (`detect_utils`/`report_utils`/`yoloe_tuner`) | monolithic nodes + copy-paste |
| Sim camera | **RGBD** (RGB + registered depth) | RGB only |
| Worlds | dedicated **`inspection_arena.sdf`** + hazard objects (fire/fumes/extinguisher/exit/person) | gauge worlds |
| Docs | per-package READMEs + Mermaid + demo GIFs | one top README + dev-diary |
| Legacy | none | `zone_sweeper`/`panorama_segmenter`/2 SLAM stacks/legacy xacros |

**What ONLY the `go2-sim/` line has:** (1) **gauge value-reading** (`gauge_inspector.py` — an LLM reads
analog dials to type/unit/value/risk + `score()` vs ground truth); (2) the deployment apps (`apps/`).

**What NEITHER has:** automated tests/CI, a true mission state machine (`-main` has async exec, not a plan
FSM), `twist_mux`/`collision_monitor` safety, lifecycle action servers, benchmarking, perception-sim
realism.

## Decision

**Converge the project onto the `-main` engine as the canonical base, re-add the differentiators
(gauge reading + deployment apps) on top, then add what neither version has.**

## Rationale

The `-main` line is a coherent, complete, working, better-engineered whole. Surgically grafting its
interdependent advances (the RGBD camera is coupled to a changed odometry/bridge/launch) into the
diverged, legacy-laden tree is more error-prone than adopting its coherent stack and re-layering the two
unique pieces. The end state is ONE clean, advanced tree — not two.

## Constraints (standing)

- **Do not break the running sim.** Stage the convergence; `colcon build` + launch-parse verify each
  stage; git history + a tarball backup are the safety net. Gazebo is runtime-tested at each stage.
- **Focus on the simulation;** the `apps/` re-integration comes last.
- Preserve runtime assets (`~/.go2_maps`, saved maps).

## Staged plan (each stage builds + is verified before the next)

- **M1 — Inspection engine (additive, this stage).** Copy `zone_inspector` + `detect_utils` +
  `report_utils` + `yoloe_tuner` into `go2_inspection`; register `zone_inspector`. Touches nothing
  running (wall-follower/mission_control/bringup unchanged). The new engine is present but not yet wired.
- **M2 — Sim bringup.** Adopt the RGBD camera + direct-`odom_tf` (drop the EKF) in `go2_description`,
  the matching `ros_gz_bridge_champ.yaml` (bridge `/camera/depth/image_raw`) and `go2_champ.launch.py`;
  bring in `inspection_arena.sdf` + hazard textures. **Enables depth → map.**
- **M3 — Orchestration.** Adopt the async `mission_control_server` (+ `cancel_task`), the
  `zone_inspector`-driven `inspection_mission`, and the 14-tool `mcp_mission_server`; wire
  `inspect_zone → zone_inspector`.
- **M4 — Re-add gauge reading.** Layer `gauge_inspector` (LLM reader) onto the crops `zone_inspector`
  produces (read crops whose class is gauge-like), for **general object inventory AND gauge values**.
- **M5 — Door-aware segmentation + better frontier.** Adopt the `-main` `zone_segmenter` (nav_point/label)
  + `frontier_explorer`/`self_filter`.
- **M6 — Clean legacy.** Remove `zone_sweeper`/`panorama_segmenter`/`yoloe_segmenter`/legacy launch +
  configs + xacros + the 2nd SLAM stack once the new path is validated.
- **M7 — Exceed both.** Tests + CI (pytest on pure geometry/segmentation + `score()` regression);
  mission state machine + event stream; `twist_mux` + `collision_monitor`; lifecycle action servers;
  perception-sim realism (de-emissive gauges, depth noise); benchmarking (coverage %, detection P/R vs
  the `objects.json` world positions, gauge-read accuracy).

## Preserved differentiators

- `gauge_inspector.py` + `mcp_gauge_server.py` (re-layered in M4).
- `apps/` deployment bridge + autonomy (re-integrated after the sim converges).

## Progress

- **M1 DONE (2026-06-30):** engine modules added (`zone_inspector`/`detect_utils`/`report_utils`/
  `yoloe_tuner`) + `zone_inspector` registered; build clean; purely additive. Backup in `.convergence-backup/`.
- **M2 DONE (2026-06-30, conservative):** added the **RGBD depth** only — `rgbd_camera` in `gazebo_gz.xacro`
  (RGB `camera/image` + `camera_info` PRESERVED, so the wall-follower is unaffected) + the
  `/camera/depth/image_raw` bridge entry. **Deliberately did NOT** take the odometry change (kept the EKF)
  or the `/camera/points`→costmap integration (separate, would change Nav). xacro→URDF + bridge YAML + build
  all verified. `zone_inspector`'s depth→map is now possible in the sim. `inspection_arena.sdf` deferred to
  M3 (needed when the engine is wired).
- **Cleanup (2026-06-30):** archived 372 MB+ of verified-dead duplicate stacks
  (`go2-autonomous-inspection-sim/`, `go2-rtabmap-mapping-stack/`, `go2-inspection/`, `simulation/`) + scratch
  to `~/ee26-clutter-archive/` (reversible; `rm -rf` when satisfied). Working tree untouched.
- **M3 DONE (2026-06-30):** adopted the **async orchestration** — `mission_control_server` (13 services
  incl **`cancel_task`**, token lock, task-id lifecycle, `GO2_WS` auto-detect), `inspection_mission`
  (drives `zone_inspector`, object inventory + facility rollup), 14-tool `mcp_mission_server`
  (+`cancel_task`/`get_zone_objects`); brought in `map_grab.py`+`npz_to_map.py` + `mission_control.launch.py`
  + `inspection_arena.sdf`+`fire_tex`. **`inspect_zone` now drives the depth viewpoint-spin engine** (wall-
  follower kept available; retired in M6). **Preserved** `gauge_inspector`/`mcp_gauge_server` + the
  `ZoneTask{zone_id, read}` srv (superset) for M4. Backup `orchestration-preM3-*.tar.gz`. Verified: build
  clean, 13 services + 14 MCP tools load, `save_map` paths resolve, arena installed. Note: M3 temporarily
  has NO gauge reading (M4 re-adds it).
- **M4 DONE (2026-06-30) — now exceeds both versions:** re-added gauge value-reading *on top of* the
  `-main` engine. (1) `detect_utils.PROMPTS` += 2 gauge classes (`is_gauge()` + cyan `color_for`) so
  `zone_inspector` **detects + 3D-depth-localizes + crops gauges like any object** (better placement than
  the old wall-follower). (2) `gauge_inspector` now reads gauge crops from `objects.json` (filters
  gauge-class, MERGES a `gauge_reading` {type/unit/value/range/risk/conf} back per object); legacy
  `gauges.json` still supported. (3) `read` threaded NL→tool→service→mission:
  `mcp.inspect_zone/run_mission(read_gauges=true)` → `_call(read)` → `req.read` →
  `mission_control._mission_cmd(read)` → `-p read:=true` → `inspection_mission` runs `gauge_inspector`
  (in the gauge venv) per zone after the spin, off the locomotion loop, graceful if no key/venv. Result:
  **general object inventory + gauge detection-with-depth + gauge value reading** — a superset of both the
  `-main` and old wall-follower paths. Build clean; full chain verified. Needs ANTHROPIC_API_KEY + YOLOE
  weights to exercise the reading at runtime.
- **M5 DONE (2026-06-30):** adopted the **door-aware `zone_segmenter`** (obstacle-island fill →
  distance-transform room cores → watershed → `{nav_point, label}` per region) — the `nav_point` (deepest
  free point) is what `inspection_mission`/`zone_inspector` use for viewpoint sampling; backward-compatible
  (engine falls back to `center` for old zones files). **DECISION: kept the existing `frontier_explorer` +
  `self_filter`** — a direct comparison showed it already has the full robustness surface
  (`goal_clearance`, `gain_scale`/`potential_scale` info-gain, `done_confirm` multi-stage termination,
  `blacklist_ttl`, `max_goal_distance` clamp, progress watchdog); the `-main` version is a *substantially
  different* impl (354/410 lines differ) with no demonstrable advantage, so replacing the verified-strongest
  working C++ node would be needless regression risk. Build clean. Backup `zone_segmenter-preM5.py`.
- **M6 HELD:** legacy retirement (wall-follower path, slam_toolbox 2nd stack, legacy xacros/launch/configs)
  is **deferred until the converged inspect path is runtime-tested in Gazebo** — must not delete the
  working fallback before the new path is confirmed live. (Safe sub-set — the unused slam_toolbox stack +
  legacy xacros — can go first after a green runtime check.)
- **M7a DONE (2026-06-30) — tests + CI (neither version had any):** added `pytest` suites for the pure,
  ROS-free core — `go2_zones/test/test_zone_segmenter.py` (synthetic two-room grid → exactly 2 zones;
  island-fill reclaim; sorted/stable ids; nav_point lands in free space), `go2_inspection/test/`
  (`test_detect_utils.py` gauge vocab + `is_gauge` + `color_for` groups; `test_gauge_logic.py` gauge
  filter). **8 tests pass.** Added `.github/workflows/ci.yml` — a lightweight GitHub Actions job
  (numpy + opencv-headless + pytest, no ROS/Gazebo) that gates the algorithmic core on every push/PR.
  Purely additive; it *verifies* the converged segmentation/detection logic.
- **Runtime verified + hang fix (2026-06-30) — the converged engine runs end-to-end in Gazebo.**
  Live on the maze world (sim + Nav2 + localization): async `inspect_zone` → `task_id`; `get_status`
  `task=running`; **`cancel_task` works**; mission drove **HOME → zone_0 via Nav2 → ARRIVED**; the engine
  then ran the full FSM — **2 viewpoints → NAV → 360° spin (382°) → NAV → 360° spin (383°) → DONE → clean
  exit**, writing `objects.json`/`detections.json`/4 frame sets.
  - **Fixed a node-killing hang:** `zone_inspector.__init__` loads YOLOE synchronously; with weights but
    **no PE cache** it reaches `model.get_text_pe()`, which **downloads the CLIP backend with no timeout**
    → offline it **hangs forever**, so the FSM never starts and the robot sits idle after arriving (the
    intended graceful degradation never fired because the call *hangs* rather than *raises*). Fix:
    `detect_utils._get_text_pe_bounded()` runs it on a daemon thread bounded by **`YOLOE_PE_TIMEOUT`
    (90 s)** and **raises** on timeout → `_load_detector` degrades to `model=None` (navigate + spin, no
    capture). +3 regression tests (**11 pass**). Proper cure = the on-disk PE cache (build once w/ network).
  - **Ground truth checked:** the maze has 4 wall gauges, one per room; **zone_0 = gauge_01 @ world
    (5.85, 2.0)**; the robot's viewpoint 1 (5.3, 2.3) was **0.63 m** from it and spun 360° facing it — so
    `0 objects` was *purely* YOLOE-OFF, **not** a coverage/nav/localization miss.
- **M6 unblocked (was held):** the converged inspect path is confirmed live, so legacy retirement
  (wall-follower path, 2nd SLAM stack, legacy xacros/launch/configs) can proceed.
- **M7b IN PROGRESS (2026-06-30) — ground-truth benchmarking DONE:** `benchmark.py`
  (`ros2 run go2_inspection benchmark <world.sdf> <gauges_root>`) parses GT object world-positions from
  the Gazebo SDF, loads per-zone `objects.json`, and reports **precision/recall/F1 + mean localization
  error** (greedy per-class NN match) + per-class recall → `benchmark.{md,json}`. Pure/ROS-free, **+6
  tests (17 total pass)**. On the maze it finds the 4 GT gauges and scores recall 0/4 honestly (detection
  OFF); with YOLOE/CLIP it yields real P/R vs e.g. gauge_01 @ (5.85, 2.0).
- **M7b — mission state machine + event stream DONE:** `mission_fsm.py` (pure/ROS-free, CI-tested)
  is a strict FSM `IDLE→PLANNING→NAVIGATING→INSPECTING→(READING)→…→ROLLUP→DONE` (ABORTED from any active
  phase) that appends structured events `{seq,t,state,kind,zone?,data?}` to an append-only JSONL stream
  (`~/gauges/mission_events.jsonl`) + in-memory log; `read_events()` reads it back. Wired into
  `inspection_mission` via a defensive `_ev()` (observability can never raise into the mission). **+4
  tests (21 total).** Verified live: the stream captured `PLANNING→NAVIGATING→NAV_FAILED→ROLLUP→DONE`
  honestly with Nav2 down. **Phase now surfaced via `get_status`** (read-only `read_events` + defensive):
  status gains `mission_phase`/`mission_event` + a `phase=…` message tag; verified live after a clean
  server restart (`busy=idle phase=DONE`). **Still pending:** a dedicated `get_events` history service,
  `twist_mux`+`collision_monitor`, gauge-read accuracy via `score()`, perception-sim realism.
- **M6 DEFERRED (not blanket-deletable):** a full launch-graph trace shows the legacy launches are
  entangled with the live path — `inspection_nav → nav2, rtabmap_slam → go2_champ, octomap` (octomap
  live) and `explore → nav2, slam → go2_champ, sim` (slam + sim live via frontier exploration). Only
  `mission.launch.py`/`sim_mapping.launch.py` + the 4 wall-follower nodes + `explore_lite`/`rtabmap.launch`
  are outside the closure, and `sim_mapping → mission → nodes` interlink them. Retiring them safely needs a
  dedicated verified pass (rebuild + launch-parse each active launch), following the don't-break-working rule.
