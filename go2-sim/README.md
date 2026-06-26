# go2-sim — autonomous Go2 SLAM / exploration / inspection (Gazebo Harmonic + ROS 2 Jazzy)

The simulation foundation for the larger Go2 autonomous-inspection project. Built on
**Gazebo Harmonic + ROS 2 Jazzy** with a **sim-agnostic autonomy stack** (the simulator is
swappable; the SLAM/Nav2/exploration/inspection code only depends on standard ROS 2 topics).

## Goal (Stage 1)
Official Unitree Go2 in a realistic Gazebo world → **SLAM + frontier-based exploration →
autonomous map building**. Future stages add 3D SLAM, perception, inspection + change
detection + reports, and sim-to-real.

## Read these
- `docs/00-REFERENCE-REPOS.md` — analysis of the 4 reference repos + what we reuse.
- `docs/01-DESIGN-DECISIONS.md` — ADRs (why Gazebo over MuJoCo/Isaac; sim-agnostic stack).
- `docs/02-ROADMAP.md` — architecture, ROS 2 topic/TF contract, phased plan, Stage-1 steps.
- `PROGRESS-LOG.md` — milestones & checkpoints (newest first).

## Status
**Checkpoint 0 done** (foundation decided & documented). Building Stage 1 next.

> Sibling `../simulation/` is the **MuJoCo locomotion track** (velocity control + RealSense +
> ROS node, PR #1) — a different layer (ADR-006), not the autonomy simulator.
