# ADR-016 — Converge onto the `-main` inspection engine, then exceed it

**Status:** Accepted (2026-06-30) · **Supersedes:** the wall-follower inspection path (ADR-015 line)

## Context

A teammate's `go2-ros2-inspection-main` (a GitHub-zip of the same project) turned out to be a **more
advanced, cleaner parallel line** of our `go2-sim/` stack. A full three-way review (our two versions +
two independent subagents) found that, on most engineering axes, **theirs is ahead of ours**:

| Area | Theirs (`-main`) | Ours (`go2-sim/`) |
|---|---|---|
| Object localization | **depth → map 3D projection** (RGBD camera, deproject via K at image stamp) | coarse LiDAR bearing/range heuristic |
| Inspection strategy | viewpoint sampling + **360° in-place spin** (covers interior + walls) | wall-following (walls only) |
| False-positive control | **map+polygon position validation** + **persistence (min-observations)** dedup | weaker map-position dedup |
| Orchestration | **async task model + `cancel_task`** + owner-token lock | blocking `subprocess.run` + timeouts |
| Code structure | **modular** (`detect_utils`/`report_utils`/`yoloe_tuner`) | monolithic nodes + copy-paste |
| Sim camera | **RGBD** (RGB + registered depth) | RGB only |
| Worlds | dedicated **`inspection_arena.sdf`** + hazard objects (fire/fumes/extinguisher/exit/person) | gauge worlds |
| Docs | per-package READMEs + Mermaid + demo GIFs | one top README + dev-diary |
| Legacy | none | `zone_sweeper`/`panorama_segmenter`/2 SLAM stacks/legacy xacros |

**What ONLY ours has:** (1) **gauge value-reading** (`gauge_inspector.py` — Claude reads analog dials to
type/unit/value/risk + `score()` vs ground truth); (2) the **WendyOS deployment apps** (`apps/`).

**What NEITHER has:** automated tests/CI, a true mission state machine (theirs has async exec, not a plan
FSM), `twist_mux`/`collision_monitor` safety, lifecycle action servers, benchmarking, perception-sim
realism.

## Decision

**Converge our project onto the `-main` engine as the canonical base, re-add our differentiators
(gauge reading + WendyOS apps) on top, then add what neither version has.** Owner authorised full reuse
of the teammate's code.

## Rationale

Theirs is a coherent, complete, working, better-engineered whole. Surgically grafting its
interdependent advances (the RGBD camera is coupled to a changed odometry/bridge/launch) into our
diverged, legacy-laden tree is more error-prone than adopting its coherent stack and re-layering our two
unique pieces. The end state is ONE clean, advanced tree — not two.

## Constraints (standing)

- **Do not break the running sim.** Stage the convergence; `colcon build` + launch-parse verify each
  stage; git history + a tarball backup are the safety net. The owner runtime-tests Gazebo.
- **Focus on the simulation;** the WendyOS/`apps/` re-integration comes last.
- Preserve runtime assets (`~/.go2_maps`, saved maps).

## Staged plan (each stage builds + is verified before the next)

- **M1 — Inspection engine (additive, this stage).** Copy `zone_inspector` + `detect_utils` +
  `report_utils` + `yoloe_tuner` into our `go2_inspection`; register `zone_inspector`. Touches nothing
  running (wall-follower/mission_control/bringup unchanged). The new engine is present but not yet wired.
- **M2 — Sim bringup.** Adopt the RGBD camera + direct-`odom_tf` (drop the EKF) in `go2_description`,
  the matching `ros_gz_bridge_champ.yaml` (bridge `/camera/depth/image_raw`) and `go2_champ.launch.py`;
  bring in `inspection_arena.sdf` + hazard textures. **Enables depth → map.**
- **M3 — Orchestration.** Adopt the async `mission_control_server` (+ `cancel_task`), the
  `zone_inspector`-driven `inspection_mission`, and the 14-tool `mcp_mission_server`; wire
  `inspect_zone → zone_inspector`.
- **M4 — Re-add gauge reading.** Layer `gauge_inspector` (Claude) onto the crops `zone_inspector`
  produces (read crops whose class is gauge-like), so we get **general object inventory AND gauge values**.
- **M5 — Door-aware segmentation + better frontier.** Adopt their `zone_segmenter` (nav_point/label) +
  `frontier_explorer`/`self_filter`.
- **M6 — Clean legacy.** Remove `zone_sweeper`/`panorama_segmenter`/`yoloe_segmenter`/legacy launch +
  configs + xacros + the 2nd SLAM stack once the new path is validated.
- **M7 — Exceed both.** Tests + CI (pytest on pure geometry/segmentation + `score()` regression);
  mission state machine + event stream; `twist_mux` + `collision_monitor`; lifecycle action servers;
  perception-sim realism (de-emissive gauges, depth noise); benchmarking (coverage %, detection P/R vs
  the `objects.json` world positions, gauge-read accuracy).

## Preserved differentiators

- `gauge_inspector.py` + `mcp_gauge_server.py` (re-layered in M4).
- `apps/` WendyOS bridge + autonomy (re-integrated after the sim converges; see ADR on WendyOS-last).

## Progress

- **M1 DONE (2026-06-30):** engine modules added (`zone_inspector`/`detect_utils`/`report_utils`/
  `yoloe_tuner`) + `zone_inspector` registered; build clean; purely additive. Backup in `.convergence-backup/`.
- **M2 DONE (2026-06-30, conservative):** added the **RGBD depth** only — `rgbd_camera` in `gazebo_gz.xacro`
  (RGB `camera/image` + `camera_info` PRESERVED, so the wall-follower is unaffected) + the
  `/camera/depth/image_raw` bridge entry. **Deliberately did NOT** take their odometry change (kept our EKF)
  or the `/camera/points`→costmap integration (separate, would change Nav). xacro→URDF + bridge YAML + build
  all verified. `zone_inspector`'s depth→map is now possible in our sim. *Owner to runtime-test that
  `/camera/depth/image_raw` publishes.* `inspection_arena.sdf` deferred to M3 (needed when the engine is wired).
- **Cleanup (2026-06-30):** archived 372 MB+ of verified-dead duplicate stacks
  (`go2-autonomous-inspection-sim/`, `go2-rtabmap-mapping-stack/`, `go2-inspection/`, `simulation/`) + scratch
  to `~/ee26-clutter-archive/` (reversible; `rm -rf` when satisfied). Working tree untouched.
- **M3 DONE (2026-06-30):** adopted their **async orchestration** — `mission_control_server` (13 services
  incl **`cancel_task`**, owner-token lock, task-id lifecycle, `GO2_WS` auto-detect), `inspection_mission`
  (drives `zone_inspector`, object inventory + facility rollup), 14-tool `mcp_mission_server`
  (+`cancel_task`/`get_zone_objects`); brought in `map_grab.py`+`npz_to_map.py` + `mission_control.launch.py`
  + `inspection_arena.sdf`+`fire_tex`. **`inspect_zone` now drives the depth viewpoint-spin engine** (wall-
  follower kept available; retired in M6). **Preserved** our `gauge_inspector`/`mcp_gauge_server` + the
  `ZoneTask{zone_id, read}` srv (superset) for M4. Backup `orchestration-preM3-*.tar.gz`. Verified: build
  clean, 13 services + 14 MCP tools load, `save_map` paths resolve, arena installed. *Owner to runtime-test
  the new inspect path in sim.* Note: M3 temporarily has NO gauge reading (M4 re-adds it).
- **M4 DONE (2026-06-30) — we now EXCEED both versions:** re-added gauge value-reading *on top of* their
  engine. (1) `detect_utils.PROMPTS` += 2 gauge classes (`is_gauge()` + cyan `color_for`) so `zone_inspector`
  **detects + 3D-depth-localizes + crops gauges like any object** (better placement than our old wall-
  follower). (2) `gauge_inspector` now reads gauge crops from `objects.json` (filters gauge-class, MERGES a
  `gauge_reading` {type/unit/value/range/risk/conf} back per object); legacy `gauges.json` still supported.
  (3) `read` threaded NL→tool→service→mission: `mcp.inspect_zone/run_mission(read_gauges=true)` →
  `_call(read)` → `req.read` → `mission_control._mission_cmd(read)` → `-p read:=true` → `inspection_mission`
  runs `gauge_inspector` (in the gauge venv) per zone after the spin, off the locomotion loop, graceful if
  no key/venv. Result: **general object inventory + gauge detection-with-depth + gauge value reading** — a
  superset of both `-main` and our old wall-follower path. Build clean; full chain verified. Needs
  ANTHROPIC_API_KEY + YOLOE weights to exercise the reading at runtime.
- **M5 DONE (2026-06-30):** adopted their **door-aware `zone_segmenter`** (obstacle-island fill →
  distance-transform room cores → watershed → `{nav_point, label}` per region) — the `nav_point` (deepest
  free point) is what `inspection_mission`/`zone_inspector` use for viewpoint sampling; backward-compatible
  (engine falls back to `center` for old zones files). **DECISION: kept our `frontier_explorer` +
  `self_filter`** — a direct comparison showed ours already has the full robustness surface
  (`goal_clearance`, `gain_scale`/`potential_scale` info-gain, `done_confirm` multi-stage termination,
  `blacklist_ttl`, `max_goal_distance` clamp, progress watchdog); theirs is a *substantially different*
  impl (354/410 lines differ) with no demonstrable advantage, so replacing our verified-strongest, working
  C++ node would be needless regression risk. Build clean. Backup `zone_segmenter-preM5.py`.
- **M6 HELD:** legacy retirement (wall-follower path, slam_toolbox 2nd stack, legacy xacros/launch/configs)
  is **deferred until the owner runtime-tests the converged inspect path in Gazebo** — must not delete the
  working fallback before the new path is confirmed live. (Safe sub-set — the unused slam_toolbox stack +
  legacy xacros — can go first once we have a green runtime check.)
- **M7a DONE (2026-06-30) — tests + CI (NEITHER version had any):** added `pytest` suites for the pure,
  ROS-free core — `go2_zones/test/test_zone_segmenter.py` (synthetic two-room grid → exactly 2 zones;
  island-fill reclaim; sorted/stable ids; nav_point lands in free space), `go2_inspection/test/`
  (`test_detect_utils.py` gauge vocab + `is_gauge` + `color_for` groups; `test_gauge_logic.py` gauge
  filter). **8 tests pass.** Added `.github/workflows/ci.yml` — a lightweight GitHub Actions job
  (numpy + opencv-headless + pytest, no ROS/Gazebo) that gates the algorithmic core on every push/PR.
  Purely additive; this is the biggest credibility-per-hour "exceed both" win and it *verifies* the
  converged segmentation/detection logic.
- **RUNTIME VERIFIED + hang fix (2026-06-30, CP51) — the converged engine runs end-to-end in Gazebo.**
  Live on the maze world (sim + Nav2 + localization): async `inspect_zone` → `task_id`; `get_status`
  `task=running`; **`cancel_task` works**; mission drove **HOME → zone_0 via Nav2 → ARRIVED**; the engine
  then ran the full FSM — **2 viewpoints → NAV → 360° spin (382°) → NAV → 360° spin (383°) → DONE → clean
  exit**, writing `objects.json`/`detections.json`/4 frame sets.
  - **🐛 Fixed a node-killing hang:** `zone_inspector.__init__` loads YOLOE synchronously; with weights but
    **no PE cache** it reaches `model.get_text_pe()`, which **downloads the CLIP backend with no timeout**
    → offline it **hangs forever**, so the FSM never starts and the robot sits idle after arriving (the
    intended graceful degradation never fired because the call *hangs* not *raises*). Fix:
    `detect_utils._get_text_pe_bounded()` runs it on a daemon thread bounded by **`YOLOE_PE_TIMEOUT`
    (90 s)** and **raises** on timeout → `_load_detector` degrades to `model=None` (navigate + spin, no
    capture). +3 regression tests (**11 pass**). Proper cure = the on-disk PE cache (build once w/ network).
  - **Ground truth checked:** the maze has 4 wall gauges, one per room; **zone_0 = gauge_01 @ world
    (5.85, 2.0)**; the robot's viewpoint 1 (5.3, 2.3) was **0.63 m** from it and spun 360° facing it — so
    `0 objects` was *purely* YOLOE-OFF, **not** a coverage/nav/localization miss.
- **M6 UNBLOCKED (was HELD):** the converged inspect path is now confirmed live, so legacy retirement
  (wall-follower path, 2nd SLAM stack, legacy xacros/launch/configs) can proceed.
- **M7b PENDING:** mission state machine + structured event stream, `twist_mux` + `collision_monitor`
  safety, benchmarking (coverage % / detection P-R vs `objects.json` world xyz — now we have ground-truth
  gauge positions / gauge-read accuracy via `score()`), perception-sim realism.
