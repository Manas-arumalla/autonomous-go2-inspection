# ADR-017 — Detect-then-approach: resolution-driven close reading (scales to large rooms)

**Status:** Implemented (geometry + engine wiring), opt-in (`read_approach:=false` default) · 2026-06-30

## Context — why the 360° spin doesn't scale

`zone_inspector` samples interior viewpoints and does a 360° in-place spin, running YOLOE + projecting each
detection to a 3D map position. In a **small** room this reads gauges fine (verified: maze zone_0 gauge
detected + localized to **7 cm**). But the viewpoint-to-wall distance **scales with room size**, and
**detection and reading have different sensing requirements**:

- **Detection** ("there is a dial on that wall") works at range, wide-FOV, even while moving.
- **Reading** ("the needle is at 4.2 bar") needs a **close, high-res, sharp, fronto-parallel** view.

For our 640×480 camera (fx ≈ 381 px), a 0.26 m gauge needs ~120 px across to read → a **hard max readable
standoff of ~0.8 m** (`d = fx·size/px`). In a large facility room the spin viewpoint is several metres from
a wall gauge → ~25 px blob → **unreadable, however good the spin is.** No single survey pattern can both
*cover* a big room and *read* every gauge.

## Decision

Adopt **detect-then-approach**, the pattern real inspection robots (Spot Orbit / Spot CAM, ANYbotics
ANYmal) use — *navigate close to each asset and frame it*, rather than reading from a wide spin — and which
our **depth-3D localization already enables** (we know each gauge's world xy, so we can drive to it):

1. **Survey** (unchanged) — viewpoints + 360° spin DETECT + 3D-localize every gauge at range.
2. **Approach + read** (new `READ_APPROACH` phase) — for each localized gauge, plan a **close,
   fronto-parallel, reachable standoff pose**, Nav2 to it, stop (no motion blur), capture a burst, keep the
   sharpest frame, and save a high-res **read crop**; Claude reads *that* crop.

The standoff **distance is derived from a pixel budget**, so reading is **scale-invariant** — bigger room
just means the detection happens farther away, but the read pose is always pulled back to the readable
distance.

## Implementation

**`inspect_planner.py`** (pure, ROS-free, CI-tested — 7 unit tests):
- `standoff_distance(fx, size, target_px, dmin, dmax)` — `d = fx·size/px`, clamped to [0.5, 1.2] m.
- `wall_normal(occ, xy, …)` — outward wall normal from the occupancy gradient around the gauge (centroid of
  nearby non-free cells → point away), with a free-centroid fallback for free-standing assets.
- `inspection_pose(asset_xy, normal, d)` — standoff pose, yaw facing the asset.
- `plan_reading_pose(asset_xy, normal, d, is_free, arc_deg, step_deg)` — try the wall-normal standoff, then
  **rotate the standoff direction around the asset within ±arc_deg** until a free pose is found → degrades
  to the nearest reachable viewing angle instead of stranding (also addresses the nav-reachability failures
  we saw on hard-to-reach viewpoints).
- `make_is_free(plausible_mask, …)` — reachability test from the dilated obstacle mask (free + clearance).
- `sharpness(gray)` — variance-of-Laplacian; pick the sharpest burst frame so blur never reaches the reader.

**`zone_inspector` `READ_APPROACH` phase** (opt-in `read_approach:=true`; default OFF = behaviour
unchanged): after the last viewpoint spin, build a reading pose per detected gauge via `inspect_planner`
(using the camera `K` + the loaded occupancy grid), then for each: `READ_NAV` (Nav2 to the standoff, skip on
failure) → `READ_CAPTURE` (settle → burst → sharpest → re-detect for a tight crop, else centred fallback →
save `read_crops/<id>.png`). Each object gains `read_crop` + `read_standoff` + `read_dist`.
`gauge_inspector` **prefers `read_crop`** over the at-range spin crop.

Tunables: `read_target_px` (120), `read_asset_size` (0.26 m), `read_dmin`/`read_dmax` (0.5/1.2 m),
`read_arc_deg` (60°), `read_burst` (5).

## Why this beats both spin-only and wall-following

- vs **spin-only**: fixes large-room reading (always read from ~0.8 m); the spin stays for robust detection.
- vs **wall-following** (retired in M6): no fragile open-loop strafe; only approach *where a gauge is* (not
  every wall metre); doesn't miss free-standing assets; any room shape; Nav2-to-a-pose is robust.

## Real-time / on-device

Detection runs online on the Orin GPU (YOLOE 20+ FPS) during the survey; planning is millisecond geometry;
the heavy reader (Claude/VLM) runs **per asset** (a handful per room), off the locomotion loop. The robot
stops only briefly at each gauge to read — real-time on-device.

## Future (next-best-view loop)

If a reading is low-confidence (glare/occlusion/blur), trigger a **re-approach from a different angle** — a
closed-loop NBV step. Hardware lever for very large spaces: a PTZ/optical-zoom camera + the Go2's
articulated head (read far without approaching — Spot's trick).
