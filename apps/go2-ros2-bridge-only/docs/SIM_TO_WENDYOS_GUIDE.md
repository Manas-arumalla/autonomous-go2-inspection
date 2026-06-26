# Porting A Sim ROS 2 Stack To WendyOS

## 1. Separate Hardware Bridge From Autonomy

Simulation stacks often assume perfect topics, frames, and clocks. On the real
robot, first create a bridge app that publishes a clean robot contract:

- `/odom`
- `/tf`
- `/joint_states`
- `/pointcloud`
- `/imu`
- `/cmd_vel`

Build and test that contract before adding SLAM, planners, perception, or UI.

## 2. Replace Sim Providers

Remove Gazebo-only launch files, robot spawners, `ros_gz_bridge`, fake sensors,
and simulator controllers. Your WendyOS app should consume real bridge topics
instead. If the sim package expects `/scan`, add `pointcloud_to_laserscan` or a
real sensor adapter. If it expects camera calibration, publish a real
`CameraInfo`.

## 3. Audit Frames

Write down the required TF tree before deploying. Common minimum:

```text
map -> odom -> base_link -> sensors
```

For a pure bridge app, publish only `odom -> base_link` and robot telemetry.
SLAM should own `map -> odom`. Do not publish the same transform from two nodes.

## 4. Fix Time Domains

Real robot messages may use embedded hardware time, while ROS tools in WendyOS
use container time. If TF errors mention extrapolation into the past/future,
restamp bridge outputs with local ROS time. Keep `use_sim_time=false` on real
robot launches unless a real `/clock` source exists.

## 5. Check DDS And Network

Use host networking for DDS-heavy apps. Bind CycloneDDS to the interface that
routes to the robot. Keep raw robot topics and clean app topics on separate ROS
domains when the robot exposes older or noisy DDS metadata.

## 6. Minimize The Image

Install only runtime dependencies needed by your stack. Heavy packages such as
RTAB-Map, Nav2, OctoMap, SAM, or VLA models should live in the app that actually
needs them, not in the base bridge. This keeps build/upload time reasonable.

## 7. Make Motion Explicit

Default motion bridges and autonomy to off/manual when testing on shared robots.
Enable `/cmd_vel` only after:

- `/odom` is live
- TF is sane
- emergency stop/lease state is understood
- the physical test area is clear

## 8. WendyOS Deployment Checklist

1. Add or copy ROS packages under the Docker context `ros2_ws/src`.
2. Install apt dependencies in the Dockerfile.
3. Source `/opt/ros/<distro>/setup.bash` and built workspaces in entrypoint.
4. Set `frameworks.ros2.domainId` in `wendy.json` to the clean domain.
5. Validate with `wendy json validate`.
6. Deploy detached:

```bash
wendy --device <device> run --prefix <app-path> --yes --detach --debug
```

7. Inspect topics/nodes:

```bash
wendy --device <device> device ros2 topics --domain <domain>
wendy --device <device> device ros2 nodes --domain <domain>
```

## 9. Common Failure Patterns

- `no running ROS 2 containers`: the app was not deployed with a
  `frameworks.ros2` config, or it is stopped.
- `no match for platform`: base image or build platform does not match arm64.
- TF extrapolation errors: clocks disagree or `use_sim_time` is wrong.
- Robot sees commands but does not move: command bridge disabled, wrong raw
  domain, sport lease/safety mode, or command API payload mismatch.
- ROS CLI times out: device unreachable, wrong target IP, app overloaded, or
  Wendy agent unavailable.
