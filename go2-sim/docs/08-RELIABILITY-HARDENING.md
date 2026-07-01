# ADR-019 — Reliability hardening: localization, Nav2 startup, and mission robustness

**Status:** Implemented and validated. All changes are additive or tuning; no working component was removed.

## Context

Running the full inspection mission on a resource-constrained machine (other workloads competing for RAM
and CPU, the simulator running below real time) surfaced a chain of latent reliability problems. The most
visible symptom was RViz reporting a global-status error with the robot never placed on the map. Tracing it
end to end revealed six distinct issues — several of which only bite under contention, but are genuine bugs
regardless — and the fixes below.

## The six failure modes and their fixes

### 1. The mission conflated "not localized" with "every room unreachable"
The mission's reachability pre-check asks Nav2's global planner for a path. That planner returns an
identical failure whether a room is genuinely unreachable *or* the robot simply is not localized yet (no
`map → base_link`). A slow localization therefore made **every** room look unreachable, and the mission
completed silently with nothing found.
- **Fix:** a **localization gate** at mission start that blocks until `map → base_link` resolves (up to a
  timeout) and aborts with a clear, distinct message ("localization is not up — the rooms are not
  unreachable") instead of skipping everything. The reachability check also retries once on a "no path"
  result to ride out a transient global-costmap update.

### 2. Localization lost mid-mission → remaining rooms wrongly skipped
During a room's detection pass, heavy inference can starve the odometry filter so the `odom → base_link`
transform lags the scan timestamps just past RTAB-Map's tolerance; RTAB-Map then drops every scan and stops
localizing. The mission read the subsequent plan failures as "unreachable" and skipped the rest of the run.
- **Fix (recover):** the navigation step now distinguishes "localization lost" from "room unreachable",
  waits for localization to recover, and retries rather than skipping the room.
- **Fix (prevent):** RTAB-Map's `wait_for_transform` was raised from 0.5 s to 2.0 s, so a lagging transform
  is waited for instead of dropping the scan. In the normal fresh-transform case the lookup returns
  immediately, so this is low risk.

### 3. Orphaned clock bridge after a hard teardown
A hard-killed `ros2 launch` does not always cascade to its children: a stray `ros_gz` clock bridge can
survive and keep publishing `/clock`. Two clock publishers fight, the simulated clock jumps backwards, TF
buffers clear repeatedly, `map → base_link` becomes intermittent, and Nav2 cannot activate.
- **Fix:** the demo launcher sweeps stale simulator/bridge processes before starting, and a dedicated stop
  script performs a complete teardown (SIGTERM then SIGKILL across the whole stack) so the next run starts
  clean.

### 4. Nav2 activated on a fixed timer, before localization was ready
The navigation lifecycle is brought up on a fixed delay. When localization was late, the global costmap's
activation timed out waiting for the robot-to-map transform and the whole navigation bringup aborted,
leaving Nav2 inactive with no automatic recovery, so every plan aborted.
- **Mitigation:** with the fixes above, RTAB-Map localizes in roughly ten seconds — well before the
  activation timer — so activation succeeds cleanly. If the race is ever hit again, a lifecycle
  reset-then-startup re-activates the stack once `map → base_link` is stable. (A fully event-driven
  activation gate is a possible future refinement; the timer plus fast localization is reliable today.)

### 5. A global thread cap that throttled localization (evaluated and rejected)
To curb RTAB-Map's high thread count under contention, I tried capping OpenMP threads globally in the
launcher. It over-throttled RTAB-Map's ICP registration so the map-database localization never converged —
no `map → odom` at all, a worse failure than the thread churn. I reverted it: the transform-lag issue it
was meant to address is already handled by the `wait_for_transform` increase (see #2), and a working stack
should not carry a speculative global throttle.

### 6. Unreachable room-centre target and hard inter-room traverses
Two navigation sub-issues surfaced once the robot could plan at all:
- **Goal tolerance too tight.** The per-room target is the room centre, which is often boxed in by the
  gauge and walls, so the robot's closest forward approach (the controller has no reverse) is 0.3–0.5 m
  away. A 0.35 m goal tolerance left it stuck at the doorway; widening it to 0.5 m accepts "inside the
  room", which is all an inspection (followed by a 360° spin) needs.
- **Doorway wedge on a direct traverse.** Going directly between two rooms, the controller can wedge in the
  hub and fail to work through a doorway from a poor approach angle. Every room, however, is reliably
  reachable from the central hub. **Fix:** a **retry-via-hub** pattern — if a direct navigation wedges,
  re-stage at the hub and retry once (a "return to the corridor, then enter the room" pattern). It is
  additive and only triggers on failure. A fast wedge detector, tracking the monotone-minimum
  distance-to-goal, cancels a stalled goal in about 30 s instead of burning the full timeout.

## Validation

- RTAB-Map localizes in about ten seconds; Nav2 reaches *active* automatically; a plan to the first room
  succeeds.
- The full autonomous mission passes the localization gate, navigates all four rooms (recovering the hard
  room via retry-via-hub), and returns home. The earlier cascade (one room, then everything "unreachable")
  is gone.
- Detection consistency is tracked separately in [ADR-020](09-DETECTION-ROBUSTNESS.md).

## Files
`go2_inspection/inspection_mission.py`, `go2_bringup/launch/rtabmap_slam.launch.py`,
`go2_bringup/config/nav2_params_rtab.yaml`, `run_demo.sh`, `stop_demo.sh`.
