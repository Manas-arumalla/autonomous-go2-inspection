# Changelog

A high-level history of the autonomous-inspection stack, newest first. Design rationale lives in
[`docs/`](docs/) (numbered ADRs); this file records what changed and why at the milestone level.

---

## Detection accuracy and consistency

- **Consistent gauge recall (4/4).** Reworked the depth-localization gate: a detection is now localized
  when it has an absolute floor of valid depth samples rather than a fixed *fraction* of its bounding box.
  A small, far dial subtends few pixels, so the old 30 %-of-patch rule discarded correct detections that
  were only seen briefly; the absolute floor plus a denser viewpoint pattern lets a far-corner gauge
  accumulate many localized observations instead of one. Benchmarked against the world's ground-truth
  poses, recall went from 3/4 to **4/4**.
- **Duplicate suppression (precision 4/4).** Added an observation-aware consolidation pass: a weak
  localization outlier (seen in far fewer frames, positioned ~1 m off) is folded into the strong,
  well-observed detection of the same class within a same-object radius. The guard only merges a detection
  seen in ≤ half the frames of its neighbour, so two comparably-observed distinct gauges are never merged —
  precision improved without costing recall. The decision is a pure function with unit tests.
- Localization error settled at **~0.2 m** mean across the four maze gauges.

## Reliability hardening

- **Localization gate.** The mission now waits for a valid `map → base_link` transform before planning and
  aborts with a clear message if localization never comes up, instead of misreading "not localized yet" as
  "every room unreachable" and silently completing with nothing found.
- **Mid-mission recovery.** If localization drops during a run (heavy perception can starve the transform
  tree on a loaded machine), the mission distinguishes that from a genuinely unreachable room, waits for
  localization to recover, and retries. RTAB-Map's transform tolerance was raised so a lagging odometry
  transform is waited for rather than dropping the scan.
- **Navigation recovery.** Added a retry-via-hub pattern: a direct room-to-room traverse that wedges the
  controller at a tight doorway re-stages at the central hub (always reachable from every room) and retries.
  A fast wedge detector cancels a stalled goal in ~30 s instead of burning the full timeout.
- **Clean startup and teardown.** The demo launcher sweeps stale simulator/bridge processes before starting
  (a leftover clock bridge otherwise fights the live one and destabilises the transform tree), and a
  matching stop script tears the whole stack down cleanly.
- Navigation goal tolerance was widened so the robot registers arrival on entering a room rather than
  demanding a pinpoint centre it cannot reach; costmap inflation was tuned to keep doorways passable.

## Live visualization and one-command demo

- Added a tailored RViz configuration and a single-command launcher that brings up the simulator, SLAM,
  Nav2, the service layer, and RViz together — so a viewer sees both the physical scene and the robot's
  internal map, costmaps, plan, camera feed, and 3D-localized detections side by side.
- A lightweight marker node republishes each localized detection as a colour-coded 3D marker with a label,
  making the inspection output visible in RViz during a run.

## Inspection pipeline

- **Detect-then-approach reading.** Beyond the survey spin, the robot can plan a close, fronto-parallel
  standoff for each detected gauge — the standoff distance is derived from a pixel-resolution budget and the
  camera model, so a dial stays readable regardless of room size — then capture a sharp, high-resolution
  crop. This scales reading to large rooms where a single spin cannot resolve the dial.
- **Autonomous mission.** From HOME the robot visits each candidate room in turn, runs the viewpoint-plus-
  spin inspection, aggregates per-room results into a facility manifest, map, and report, and returns home —
  all driven from the auto-segmented map with no hard-coded poses.
- **Service and control layer.** A `mission_control` node exposes the stack as a set of ROS 2 service
  triggers (inspect a zone, run the mission, dock, cancel, status, structured event stream), with an
  optional natural-language control surface.
- **Reading and report.** Detected gauge crops can be read (type, unit, value, risk) into the per-room and
  facility report; a scoring routine grades readings against ground truth for evaluation.

## Perception and mapping foundation

- **Zone segmentation.** The occupancy grid is watershed-segmented into room polygons, and safe interior
  viewpoints are sampled inside each room for inspection.
- **Live open-vocabulary detection.** Each viewpoint runs a 360° in-place spin with live YOLOE detection,
  projecting every detection to a 3D map position through the depth camera, de-duplicating across the room,
  and cropping the best view of each instrument.
- **Frontier exploration.** A C++ frontier explorer drives Nav2 to autonomously map an unknown facility.
- **SLAM.** RTAB-Map pure-LiDAR graph SLAM provides `map → odom`, a 2D occupancy grid, and a 3D octomap, in
  both mapping and localization modes.
- **Sim-agnostic core.** The autonomy stack binds only to the standard ROS 2 topic contract (`/cmd_vel`,
  `/odom`, `/scan`, `/camera`, TF, `/clock`), so the same node graph runs in Gazebo and on the real robot.
- **Testing.** A `pytest` suite (34 tests) plus a ground-truth benchmark (detection precision/recall and
  localization error) gate the ROS-free core, run in GitHub Actions CI.
