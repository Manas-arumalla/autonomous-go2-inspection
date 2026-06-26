# Autonomous Gauge-Inspection — Mission Spec & Roadmap

Status: **Phases 0–4 implemented & validated in sim; the environment is now a realistic multi-room
facility. The autonomous MISSION (Phase 5+) is DOCUMENTED here as planned milestones — NOT yet
implemented. Do not start implementing the future phases until explicitly told to proceed.**

Detailed build history: `PROGRESS-LOG.md` (checkpoints CP29–CP38). This file is the forward plan.

---

## 1. The mission (target behaviour)

The robot performs an industrial inspection **entirely through its own perception and autonomy**:

1. **Start** every mission from a fixed, designated **HOME** position (`(0,0)` in the facility
   corridor, facing +X) — not wherever it happens to be.
2. **Localize** on the pre-built map of the facility (the robot already has the complete map).
3. **Navigate autonomously** to a requested inspection area / room (corridor → doorway → room → wall).
4. **Inspect** the gauges / sensors in that area (drive to the wall, sweep, segment).
5. **Collect the readings** (type, value, unit, range) for every asset.
6. **Detect anomalies / faults** (needle in the red danger zone, out-of-range, threshold breaches).
7. **Complete the mission** — produce the inspection report — and (optionally) return HOME.

### NO CHEATS — hard constraint on every future phase
Everything must come from the **robot's own sensors, SLAM map, localization, and planning**. Explicitly
forbidden: ground-truth poses from the simulator, teleporting/`set_pose`, SDF-derived "maps" used for
navigation, hard-coded waypoints that bypass planning, or `skip_nav`-style shortcuts in the *mission*
(they were test-only scaffolding). Allowed: the robot's RTAB-Map localization, Nav2 planning, LiDAR,
camera, odometry/EKF. The `gz` odometry plugin (mirrors the real Go2's onboard EKF) is the only
sim-provided signal and is NOT a cheat (it is real on hardware) — see PROGRESS-LOG CP13.

---

## 2. Done (implemented & validated)

| Phase | Capability | Evidence |
|---|---|---|
| 0 | Gauge sim assets (readable analog dials, ground truth) | CP32 |
| 1 | Automatic zone (room) segmentation of the occupancy grid | CP33 |
| 1.5 | Autonomous frontier exploration → real 3D map + RTAB-Map DB → zones | CP34 |
| 2 | `zone_sweeper`: approach wall → strafe → panorama + frames (frame-independent) | CP35 |
| 3 | `panorama_segmenter`: FastSAM → clean per-gauge best-frame crops | CP36 |
| 4 | `gauge_inspector` (Claude API) + `mcp_gauge_server` (FastMCP) → CSV; **6/6 type/unit/value** | CP37 |
| — | **Distributed environment**: 12 assets / 5 types across 5 rooms, 3 anomalies | CP38 |

The per-zone pipeline (2→3→4) is proven end-to-end on `facility_gauges.sdf`. The new
`facility_inspection.sdf` spreads assets across NW/NC/NE/SC/SW so the mission requires real navigation.

---

## 3. Future phases (PLANNED — documented, not implemented)

> Build only on explicit instruction. Each phase keeps the **sim==real** rule: identical ROS topic
> contract, no sim-only shortcuts, so what works in sim works on the Go2/Orin.

### ▢ Phase 5 — Autonomous mission orchestration (one command, end-to-end)
Goal: a single command runs the whole mission; individual phases still runnable standalone.
- **5a. Complete map of `facility_inspection`.** Re-run the (existing) frontier exploration on the new
  world → a complete RTAB-Map DB + occupancy grid covering all rooms (the current DB is of
  `facility_gauges` and is east-biased — west rooms unmapped). Segment → zones for the new world.
- **5b. Localization-mode navigation fix.** Resolve the deferred issue (CP35): in localization mode
  RTAB-Map publishes only a partial `/map` around the relocalized area, so Nav2 can't path to a far
  room. Fix = serve the saved grid via `map_server` (static) for Nav2 while RTAB-Map provides
  `map→odom`; or run the mission in SLAM mode. No `skip_nav`.
- **5c. Mission orchestrator** (`go2_inspection/inspection_mission`): HOME → for each target zone:
  Nav2 to the zone → orient to the wall → `zone_sweeper` → `panorama_segmenter` → `gauge_inspector` →
  append to one **facility report** (CSV/JSON/PDF) → next zone → return HOME. One launch file; each
  phase also runnable alone (already true).
- **Acceptance:** `ros2 launch ... inspection_mission.launch.py` starts at HOME, autonomously visits
  every gauge room, and produces one facility report — no manual spawns, no skip_nav, no GT poses.

### ▢ Phase 6 — Anomaly / fault detection
- Rule layer over the readings: needle in the red danger arc, value outside `[min,max]`, or above a
  per-asset threshold → flag `ANOMALY` with severity; Claude already returns a `risk` field (reuse).
- Report highlights faults; optional alert summary. Score vs the `anomaly` flags in
  `inspection_groundtruth.json` (3 seeded faults: NC-TEMP-2, NE-VOLT-2, SW-FLOW-1).
- **Acceptance:** the mission report correctly flags the seeded anomalies and none of the normal gauges.

### ▢ Phase 7 — Real Go2 + WendyOS deploy
- Swap the sim for the real Go2 behind the same ROS contract; external USB camera as `/camera/image_raw`
  (no extrinsic needed — the pipeline is camera-frame only). Containerize per service via `wendy run`
  (mind the arm64 base-image gotcha + the 8 GB Orin budget; keep models lean / offload vision to Claude).
- **Acceptance:** the same mission runs on the Go2/Orin and produces a report from the real camera.

### ▢ Phase 8 (stretch) — Natural-language missions (MCP / agentic)
- "Inspect the electrical room" → plan → drive the autonomy via the same Nav2/sweeper contract. The
  teach-it-live differentiator (see `winning-strategy` memory). Sits above the stack; no autonomy change.

---

## 4. Deferred project tasks (from an earlier request — not lost)
- **Repo restructure** into a clean production layout + a **`legacy/` archive** of obsolete files.
- A **full professional README** (overview, architecture, structure, install, usage, workflow,
  pipeline, results, benchmarks, troubleshooting, future work).
These were paused to focus on the environment redesign + this roadmap, and to keep changes incremental
and non-breaking. Resume on request.
