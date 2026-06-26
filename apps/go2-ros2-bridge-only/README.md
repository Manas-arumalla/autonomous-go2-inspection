# Go2 ROS 2 Bridge Only

This WendyOS app is the clean bridge layer for a Unitree Go2. It contains only
the ROS 2 Jazzy bridge package, Unitree interface build, CycloneDDS setup, and
optional Foxglove/USB camera support. It intentionally excludes RTAB-Map, Nav2,
OctoMap, frontier exploration, and robot description packages.

## What It Publishes

Raw Go2 DDS runs on `GO2_ROS_DOMAIN_ID=0`. The bridge republishes standard ROS 2
topics on `BRIDGE_ROS_DOMAIN_ID=30`:

- `/odom` and `odom -> base_link` TF from `/utlidar/robot_odom`
- `/joint_states` from `/lowstate`
- `/pointcloud` from `/utlidar/cloud_deskewed`
- `/imu` from `/utlidar/imu`
- `/cmd_vel` to `/api/sport/request` when enabled
- `/go2_bridge/topic_status` topic availability diagnostics

The bridge restamps Go2 sensor and odometry messages with local ROS time by
default. Keep `RESTAMP_WITH_ROS_TIME=true` unless the robot and WendyOS clocks
are known to be synchronized.

## Deploy

```bash
wendy --device <wendyos-ip-or-hostname> run \
  --prefix apps/go2-ros2-bridge-only \
  --yes \
  --detach \
  --debug
```

Enable robot velocity control only when the area is safe:

```bash
wendy --device <device> run \
  --prefix apps/go2-ros2-bridge-only \
  --yes \
  --detach \
  --debug \
  --user-args enable-cmd-vel
```

Optional runtime flags:

- `enable-cmd-vel`: bridge `/cmd_vel` to Unitree Sport API.
- `disable-cmd-vel`: force command bridge off.
- `enable-usb-camera`: start `v4l2_camera_node`.
- `disable-foxglove`: run without Foxglove.
- `no-restamp`: preserve source message timestamps.

## Verify

```bash
wendy --device <device> device ros2 topics --domain 30
wendy --device <device> device ros2 echo /odom --domain 30 --count 1
wendy --device <device> device ros2 echo /joint_states --domain 30 --count 1
wendy --device <device> device ros2 echo /go2_bridge/topic_status --domain 30 --count 1
```

Foxglove defaults to `ws://<device-ip>:8767`.

## Extend

Add your own ROS 2 packages under `bridge/ros2_ws/src/` or build them in a
separate Wendy app that subscribes to this app's clean domain `30`. For autonomy,
prefer composing on top of `/odom`, `/tf`, `/pointcloud`, `/imu`, and `/cmd_vel`
rather than subscribing directly to the raw Go2 DDS domain.
