# Transfer Notes

## Current Package

This folder is a bridge-only extraction from the larger
`apps/go2-ros2-stack` work. It intentionally contains no frontier exploration,
Nav2, RTAB-Map, OctoMap, `go2_bringup`, `go2_exploration`, or
`go2_description`.

## Paths

- Wendy app root: `apps/go2-ros2-bridge-only`
- Docker context: `apps/go2-ros2-bridge-only/bridge`
- ROS workspace: `bridge/ros2_ws`
- Main package: `bridge/ros2_ws/src/go2_bridge`
- App config: `wendy.json`
- Runtime entrypoint: `bridge/entrypoint.sh`

## Build Contract

The Dockerfile builds Unitree message packages from
`unitreerobotics/unitree_ros2`, then builds the local `go2_bridge` package.
The final image is arm64 and targets ROS 2 Jazzy.

## Runtime Contract

Defaults:

```text
GO2_ROS_DOMAIN_ID=0
BRIDGE_ROS_DOMAIN_ID=30
ROS_DOMAIN_ID=30
GO2_IP=192.168.123.161
RESTAMP_WITH_ROS_TIME=true
ENABLE_CMD_VEL=false
ENABLE_TOPIC_MONITOR=true
LAUNCH_FOXGLOVE=true
FOXGLOVE_PORT=8767
```

The entrypoint auto-detects the local IP route to `GO2_IP` and writes
`/tmp/cyclonedds.xml`. Set `GO2_DDS_ADDRESS=<local-ip>` only if automatic route
detection chooses the wrong interface.

## Known Lessons From Real Robot Testing

- The real Go2 may stamp LiDAR/odom messages with a clock that does not match
  the WendyOS container clock. Restamping fixed TF extrapolation errors in
  downstream Nav2/RTAB-Map tests.
- The raw Go2 DDS graph produces CycloneDDS type-hash warnings. The bridge
  domain exists so normal ROS tools use the clean graph instead.
- Wendy CLI ROS tools should query domain `30`, not domain `0`.
- `wendy run --detach` is preferred for long-running bridge apps.

## Handoff Prompt

Use this prompt for another agent:

```text
You are continuing work on /home/adyansh/ee-hackathon/apps/go2-ros2-bridge-only.
This is a WendyOS ROS 2 Jazzy bridge-only app for Unitree Go2. Do not add
frontier, Nav2, RTAB-Map, OctoMap, or simulation packages to this folder.

Architecture:
- Raw Go2 DDS is on domain 0.
- Clean bridge output is on domain 30.
- Docker context is ./bridge.
- The app builds Unitree ROS 2 interfaces from unitreerobotics/unitree_ros2.
- Runtime entrypoint generates CycloneDDS config and starts go2_bridge plus
  optional Foxglove.
- RESTAMP_WITH_ROS_TIME=true is important for downstream TF/Nav2 compatibility.

Validate with:
wendy json validate apps/go2-ros2-bridge-only
bash -n apps/go2-ros2-bridge-only/bridge/entrypoint.sh
python3 -m py_compile <all go2_bridge .py files>

Deploy with:
wendy --device <device> run --prefix apps/go2-ros2-bridge-only --yes --detach --debug

Check:
wendy --device <device> device ros2 topics --domain 30
wendy --device <device> device ros2 echo /odom --domain 30 --count 1
```
