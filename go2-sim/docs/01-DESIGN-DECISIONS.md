# Design decisions (ADRs) — Go2 autonomy simulation

Architecture Decision Records for the simulation foundation. Each is dated and has
rationale + consequences so the project history stays clear as the system evolves.

---

## ADR-001 — Simulator: **Gazebo Harmonic + ROS 2 Jazzy** (not MuJoCo, not Isaac) — 2026-06-24

**Decision.** Build the autonomy foundation on **Gazebo Sim Harmonic (gz-sim 8) + ROS 2
Jazzy**, bridged with `ros_gz`.

**Options weighed**

| Criterion | Gazebo Harmonic | MuJoCo | Isaac Sim |
|---|---|---|---|
| Native ROS 2 + Nav2 + sensor bridge | ✅ `ros_gz` (first-class) | ❌ none (hand-roll everything) | ✅ Omnigraph bridge |
| SLAM/Nav2 stack on **Jazzy** as apt binaries | ✅ all present (verified) | n/a | n/a (Humble) |
| LiDAR / depth / IMU sensor sim | ✅ gz sensors | ⚠️ manual | ✅ RTX (best) |
| Reference repos' patterns port directly | ✅ (they're ROS 2/Nav2) | ❌ | partial |
| Realism / perception fidelity | good (PBR) | low (no rendering focus) | ✅ best (photoreal) |
| Hardware cost / iteration speed | ✅ light, fast | ✅ lightest | ❌ ~30 GB, 8 GB VRAM tight |
| Sim-to-real (→ real Go2 over ROS 2/DDS) | ✅ same topics | ⚠️ custom bridge | ✅ |
| Contact-rich locomotion / RL training | ⚠️ ok | ✅ best | ✅ |

**Why Gazebo wins for THIS project.** Our four reference stacks and our entire current
goal (SLAM + Nav2 + frontier exploration + occupancy mapping) are **ROS 2-native**, and
**every required package exists as a ROS 2 Jazzy apt binary** (verified: `slam_toolbox`,
`navigation2`, `nav2_bringup`, `ros_gz`, `rtabmap_ros`, `pointcloud_to_laserscan`,
`robot_localization`). Gazebo Harmonic + `ros_gz` publishes exactly the sensor/TF topics
SLAM and Nav2 consume — so we get a working autonomous-mapping demo on the shortest path,
and the reference repos' logic drops in. MuJoCo has **no native ROS 2 / Nav2 / sensor
pipeline** — choosing it here means re-implementing the LiDAR sim, the ROS bridge, and the
nav plumbing by hand. Isaac is the most photorealistic and is viable (we have an RTX 5060),
but its install is heavy, 8 GB VRAM is tight, and it **still ships no SLAM/Nav2** — it
solves a problem (perception realism) we don't have yet.

**Consequences.** Fast Stage-1; lighter iteration; the whole Jazzy autonomy ecosystem is
available; sim-to-real is a topic remap away. Trade-off: lower visual realism than Isaac
(addressed by ADR-002 — we can swap later) and Gazebo quadruped locomotion needs a
controller (ADR-003).

---

## ADR-002 — **Sim-agnostic autonomy stack** (the key hedge) — 2026-06-24

**Decision.** The autonomy software (SLAM, Nav2, frontier exploration, later
perception/inspection) is built as ROS 2 packages that depend **only on standard
interfaces** — `sensor_msgs` (`/scan`, `PointCloud2`, `Image`, `Imu`), `nav_msgs`
(`/odom`, `/map`), TF, and Nav2 actions. The **simulator is a swappable provider** of
`{world + robot + sensors + cmd_vel}`.

**Rationale.** Every reference repo proves autonomy logic ports across sims because it only
needs ROS 2 topics. By enforcing this boundary we get: (a) Gazebo now for speed; (b) the
option to **swap in Isaac Sim later for photorealistic perception** without rewriting
SLAM/Nav2/inspection; (c) **MuJoCo** (our existing `simulation/go2_velocity.py` + ROS node)
stays usable as a **locomotion-policy/contact sandbox** feeding the same `/cmd_vel`; (d)
**sim-to-real** onto the actual Go2 (`unitree_ros2` over DDS, or our Wendy `go2-inspection`)
is the same topic contract. **No autonomy code is simulator-specific.**

**Consequences.** Slightly more upfront discipline (a documented topic/TF contract — see
`02-ROADMAP.md`), big long-term payoff in extensibility/maintainability/realism-upgrade.

---

## ADR-003 — **Decouple locomotion fidelity from the SLAM milestone** — 2026-06-24

**Decision.** For Stage 1, drive the Go2 in Gazebo with a **velocity-controlled base**
(`/cmd_vel` → base motion via a gz velocity/diff-style controller), not a full gait. Add
realistic legged locomotion (CHAMP-port or bridging our MuJoCo trot/RL policy) in a later
phase.

**Rationale.** SLAM + frontier exploration only require that the robot **translates through
the world while sensors publish** — gait realism is irrelevant to map quality.
exploration_go2 made the same call (used a TurtleBot3 for the exploration logic). This
removes the biggest Gazebo-Go2 risk (CHAMP targets Humble/Classic) from the critical path.

**Consequences.** Stage-1 ships fast and reliably; legged realism is an isolated later
upgrade behind the same `/cmd_vel` interface (ADR-002).

---

## ADR-004 — **SLAM: slam_toolbox 2D first, RTAB-Map 3D later** — 2026-06-24

**Decision.** Stage 1 uses **`slam_toolbox`** (online async, 2D) fed by `/scan`
(`pointcloud_to_laserscan` flattening the 3D LiDAR). Add **RTAB-Map** 3D LiDAR SLAM in
Phase 2 when the 3D map is needed for inspection.

**Rationale.** 2D occupancy is exactly what frontier exploration + Nav2 costmaps need, it's
the lowest-risk/fastest path to "autonomous map building," and both reference explorers
(`frontier_explorer.cpp`, `explorer_pkg`) consume a 2D `/map`. RTAB-Map (also used by
Go2-Inspector) gives the 3D cloud for inspection markers/PLY later. Both are Jazzy apt binaries.

---

## ADR-005 — **Frontier exploration: port a custom node, not explore_lite** — 2026-06-24

**Decision.** Port a **custom frontier node** starting from Go2-Inspector's
`frontier_explorer.cpp` (MIT, drop-in, `/map` + Nav2 only). Upgrade later to
exploration_go2's info-gain scoring + dynamic goal-preemption FSM.

**Rationale.** `explore_lite`/`m-explore` are **not packaged for Jazzy** (verified). We have
two clean MIT reference implementations; the greedy one ships Stage 1, the info-gain one is
the differentiator upgrade. Keeps us off an unmaintained dependency.

---

## ADR-010 — **Real walking Go2 via CHAMP (port Classic→Harmonic gz_ros2_control)** — 2026-06-25

**Decision (user-directed).** Replace the Stage-1 kinematic velocity base (ADR-008) with a
**real walking Go2 using CHAMP**, the way the reference repos do it — using the repos' code
fully. **Supersedes ADR-008's "velocity base"** for the sim's locomotion (the autonomy contract
is unchanged: CHAMP still consumes `/cmd_vel`, so SLAM/Nav2/frontier are untouched).

**Source (use repos fully).** `Go2_planner_suite/.../unitree-go2-ros2` has the **complete,
proven Go2 CHAMP config** — copied into `go2_ws/src`: `champ` + `champ_base` (gait controller)
+ `champ_msgs` + `champ_description` + **`go2_config`** (Go2 gait/joints/links + ros_control) +
**`go2_description`** (real Go2 URDF + dae meshes: trunk/hip/thigh/calf/foot). Naming is CHAMP's
`lf/rf/lh/rh`. **Feasibility CONFIRMED: CHAMP compiles clean on Jazzy** (champ_base built, 16.5s).

**Adaptation needed (Classic → Harmonic).** The repo is Gazebo **Classic** + ROS 2 Humble. On
our Jazzy + Harmonic we swap the integration only:
- `libgazebo_ros2_control.so` → **`gz_ros2_control`** (installed on Jazzy ✓; needs
  `ros-jazzy-ros2-control` apt for the controller_manager hardware iface).
- Velodyne/ray/imu/p3d Classic plugins → **gz sensors** (gpu_lidar = our L1 → `/utlidar/cloud_deskewed`,
  gz imu, gz OdometryPublisher → `/utlidar/robot_odom`) bridged via `ros_gz` (the mirrored
  contract, ADR-007). Keep the URDF kinematics + meshes + the 12-joint `ros2_control` block.
- Controllers: `joint_state_broadcaster` + `joint_trajectory_controller` (effort) driven by champ_base.

**Plan.** New `go2_gz.urdf.xacro` (Harmonic) + gz `ros_control.yaml` + a walking launch (gz +
spawn + spawners + champ_base) → Go2 walks from `/cmd_vel`; then re-point SLAM/Nav2/frontier
at it. **Risk retired:** CHAMP builds. **Remaining risk:** gait tuning + gz_ros2_control wiring
on Harmonic (test under supervision — heavy launch). MuJoCo locomotion (ADR-006) and the lean
velocity base (`.go2_description_lean_backup`) are kept as fallbacks.

---

## ADR-009 — **WendyOS deployment: self-contained ROS 2 Wendy app (arm64)** — 2026-06-24

**Decision.** The autonomy ships on the real Go2 as a **WendyOS ROS 2 app group** running
**onboard the Jetson Orin Nano**, containerized self-contained (we bring our own ROS 2).

**Verified mechanism** (from `wendy-research/WendyOS/Examples/ROS2/`): WendyOS has first-class
ROS 2 via `wendy.json` `frameworks.ros2` = `{domainId, rmw: rmw_cyclonedds_cpp, distro}` +
`isolation: shared-ipc` (shares net/IPC/`/dev/shm` for CycloneDDS zero-copy) and auto-injects
`ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION`. **The actual ROS 2 comes from the container's base image**
(the example uses `ros:humble`); `distro` only selects the `wendy device ros2` inspection sidecar.

**Plan.** Package `go2_ws` as a Wendy app group:
- base image **`ros:jazzy` (arm64)** — matches our dev env exactly → no distro-mismatch rework;
- `frameworks.ros2` (cyclonedds) + `isolation: shared-ipc`; **`network: host`** to reach the
  Go2's DDS (`unitree_ros2`), exactly like the `go2-inspection` Wendy app;
- built with the **`FROM --platform=linux/arm64 ros:jazzy`** fix (the proven `wendyos/arm64`
  build gotcha) and pre-pulled on the Orin;
- services: `bringup`/`slam`/`nav2`/`exploration` (+ the `cmd_vel→SportClient` Go2 driver).

**Consequences (why this shaped earlier ADRs).** Forces **lean** (2D slam_toolbox + light
Nav2 on the 8 GB Orin), **standard ROS 2 packaging** (no un-containerizable/heavy deps), and
**distro-portable code** (so a `ros:humble` rebuild is a no-redesign fallback if needed). The
sim→real transfer (ADR-008) lands as: same nodes, now inside Wendy containers on the dog.

**Open item (non-blocking — confirm with Wendy mentors):** whether `frameworks.ros2` officially
supports `distro: "jazzy"` or only `"humble"`. The container is self-contained either way (we
control the base image); this only affects the `wendy device ros2` debug sidecar. Insurance:
keep all packages distro-portable so a Humble container is a drop-in rebuild.

---

## ADR-008 — **Velocity base + realism (no cheats); sim→real transfer plan** — 2026-06-24

**Decision.** Sim locomotion is a **kinematic velocity base** with **realistic sensor +
motion noise**, NOT a walking gait. The autonomy is developed/validated against this and
transfers to the real Go2 with **no redesign** — only a thin driver + parameter tuning.

**Why this is "no cheats," not a shortcut.**
- **Localization/mapping is never cheated:** SLAM computes `map→odom` from real
  scan-matching; we never publish a ground-truth `map→base`. The robot genuinely
  localizes+maps from sensor data.
- **The real Go2 uses neither a velocity base nor CHAMP** — it walks via Unitree's closed
  firmware gait behind `/cmd_vel`. So a velocity base is a *faithful* model of the real
  interface ("send `/cmd_vel` → body moves + odom drifts"), and CHAMP/RL legs would be
  *cosmetic* (a non-real gait) that does **not** improve sim→real autonomy fidelity.
- **Reference repos confirm the spread:** isaac-go2-ros2 walks via an RL policy;
  Go2_planner_suite walks via CHAMP (Gazebo Classic); **exploration_go2 didn't simulate
  the legs at all (used a TurtleBot3)**; Go2-Inspector has no sim. Velocity base = the
  exploration_go2 approach, with the real Go2 body.
- **Realism we ADD (the actual "no cheats" work):** L1 LiDAR Gaussian noise, odometry
  drift, and Go2-matched velocity/acceleration limits + slip on `/cmd_vel` — so the
  autonomy is tuned against imperfect data.

**Sim→real transfer plan (bounded, no rewrite).** Autonomy nodes only read
`/utlidar/cloud_deskewed` + `/utlidar/robot_odom` + TF and write `/cmd_vel` — identical in
sim and on hardware. Real-robot step = (1) thin **Go2 driver** (`/cmd_vel→SportClient`
bridge + wire real `/utlidar` topics, ≈ Go2-Inspector's `cmdvel_to_sport_bridge.cpp`);
(2) **param tuning** (Nav2 vel/accel to the dog's limits; SLAM for noisier real data);
(3) **Go2 quirks** (timestamp restampers; L1 mount-pitch/body-sway TF + filtering — the
Go2-Inspector lessons). None is a redesign. Honest caveat: first hardware run always needs
tuning; the architecture guarantees only-tuning, never-rewrite.

**Phase-2 upgrade path (optional, cosmetic/demo):** add CHAMP or bridge the MuJoCo trot
policy so the legs visibly walk — purely for demo polish, behind the same `/cmd_vel`.

---

## ADR-007 — **Hardware-mirrored topic/frame contract + L1 LiDAR + lean for Orin/Wendy** — 2026-06-24

**Decision (from the user's hardware answers).** The sim **mirrors the real Go2 EDU's
native ROS 2 interface exactly**, the autonomy targets the **built-in Unitree L1 LiDAR**,
and the production stack runs **onboard the Jetson Orin Nano via Wendy** (lean).

**The frozen sim↔real contract** (identical in sim and on the dog):
| Signal | Topic | Type | Frame |
|---|---|---|---|
| LiDAR | `/utlidar/cloud_deskewed` | `sensor_msgs/PointCloud2` | `utlidar_lidar` |
| Odometry | `/utlidar/robot_odom` | `nav_msgs/Odometry` | `odom`→`base_link` |
| Drive cmd | `/cmd_vel` | `geometry_msgs/TwistStamped` | `base_link` |
| (derived) scan | `/scan` | `sensor_msgs/LaserScan` | `utlidar_lidar` |
| Map | `/map` | `nav_msgs/OccupancyGrid` | `map` (SLAM owns `map`→`odom`) |
TF tree: `map → odom → base_link → {utlidar_lidar, imu_link, …}`. `use_sim_time:=true`.

**Consequences & rationale.**
- These topics use **standard message types only** → the autonomy stack has **no Unitree
  custom-msg dependency**; the Unitree-specific bits (`/lowstate`, `/api/sport/request`,
  SportClient) live only in the real-robot driver layer, downstream of `/cmd_vel`. On the
  real dog: Nav2 → `/cmd_vel` → a `cmdvel→SportClient` bridge (cf. Go2-Inspector); sensors
  come from the Go2's DDS/`unitree_ros2`. **Same node graph, swapped source.**
- **L1 LiDAR (not 360°):** forward-biased FOV — exploration/Nav2 must tolerate limited
  rear/side coverage (slower frontier convergence; rely on revisiting). Sim models the L1's
  approximate FOV; LiDAR FOV is a single parameter so a future Mid-360 is a config swap.
- **Lean for 8 GB Orin + Wendy:** confirms ADR-004 (2D slam_toolbox on the critical path,
  RTAB-Map 3D optional/later). Long-term integration = **containerize `go2_ws` as a Wendy
  arm64 app** (WendyOS runs ROS 2 nodes natively) — avoid deps that won't build lean on arm64.
- **Supersedes** the "canonical + adapter" lean of ADR-002's examples: we now mirror real
  names directly. ADR-002's *principle* (autonomy depends only on a stable topic contract)
  still holds — the contract is just the Go2's real names instead of generic ones.

---

## ADR-006 — **Keep MuJoCo work; it's the locomotion track, not the autonomy track** — 2026-06-24

**Decision.** The existing `simulation/` MuJoCo Go2 (velocity control + RealSense + ROS node,
PR #1) is **retained** as the locomotion/contact-dynamics + RL-policy sandbox. It is **not**
the autonomy-stack simulator. Both feed the same `/cmd_vel` contract (ADR-002), so a MuJoCo
locomotion policy can later drive the Gazebo-developed autonomy, or vice-versa.

**Rationale.** No work is wasted; MuJoCo is genuinely better for gait/RL, Gazebo for the
ROS 2 autonomy ecosystem. Right tool per layer.

---

## ADR-011 — **Sim joint control: POSITION command interface, not effort+PID** — 2026-06-25

**Context.** The copied CHAMP `go2_config/ros_control.yaml` drives the 12 leg joints with a
`JointTrajectoryController` on an **effort** command interface + per-joint PID (`p=100, i=0.2,
d=1.0`). Those gains were tuned for **Gazebo Classic / ODE**. On our **Harmonic / DART** stack
the same effort-PID loop is unstable: the Go2 spawns, briefly stands, then **twitches
chaotically even at zero `/cmd_vel`** (so it's not a gait-speed problem — it's the joint
controller oscillating while merely holding the stand pose) and tumbles onto its back.

**Decision.** Drive the sim joints with a **position** command interface
(`<command_interface name="position"/>` in `gazebo_gz.xacro`; `command_interfaces: [position]`,
`open_loop_control: true` in `ros_control.yaml`). `gz_ros2_control` applies a position command
through gz's **stable internal joint controller**, so CHAMP's IK gait angles are tracked
smoothly with **no ROS-side PID to detune**. The repo author anticipated this — a commented-out
`joint_group_position_controller` block was already present. Supporting fixes: seed the **stand
pose at spawn** via `initial_value` (hip 0, thigh 0.9, calf −1.8 rad) so the dog is standing
from frame one; spawn at **z=0.275** (the repo's `world_init_z`); tightened launch staggering so
the gait engages right after spawn (minimise the limp-leg window).

**Why this does NOT compromise sim→real (keeps the final WendyOS deployment intact).** This is a
**sim-only** locomotion detail. On the **real Go2 EDU** we never command joints — we publish
`/cmd_vel` to the onboard **SportClient** sport-mode controller (`go2-rc/motion`), which runs
Unitree's own walking. The autonomy stack (SLAM/Nav2/frontier) is **sim-agnostic** (ADR-002) and
consumes only `/cmd_vel`, `/odom`, LiDAR `PointCloud2`, TF — unchanged. Position vs effort in
Gazebo is purely how the *simulated* legs realise the gait; it never ships to the Jetson/Wendy.

**Status.** Implemented; under supervised verification (does it stand still + walk cleanly from
`/cmd_vel`). Fallback if position tracking looks too rigid: add gz joint PID `<param>` gains or
re-tune the effort path for DART.
