# Bridge Architecture

## Purpose

`go2-ros2-bridge-only` isolates the hardware-facing Unitree DDS graph from the
standard ROS 2 graph used by downstream packages. It is the minimal reusable
base for mapping, navigation, inspection, teleoperation, and diagnostics.

## Runtime Layers

1. **Raw Go2 domain**
   - Domain: `GO2_ROS_DOMAIN_ID=0`
   - Input topics: `/utlidar/robot_odom`, `/utlidar/cloud_deskewed`,
     `/utlidar/imu`, `/lowstate`, `/api/sport/request`
   - QoS: best-effort/volatile to match the robot publishers.

2. **Bridge domain**
   - Domain: `BRIDGE_ROS_DOMAIN_ID=30`
   - Output topics: `/odom`, `/tf`, `/joint_states`, `/pointcloud`, `/imu`,
     `/go2_bridge/topic_status`
   - QoS: reliable/volatile where practical for normal ROS tools.

3. **WendyOS app runtime**
   - `wendy.json` declares a single Docker service with ROS 2 Jazzy metadata.
   - `entrypoint.sh` generates CycloneDDS config at startup and starts bridge
     nodes plus Foxglove.

## Nodes

- `odom_tf_bridge`: republish `/utlidar/robot_odom` as `/odom` and broadcast
  `odom -> base_link`.
- `sensor_bridge`: republish LiDAR and IMU to standard topics.
- `joint_state_bridge`: convert Go2 `LowState` motor telemetry to
  `/joint_states`.
- `cmd_vel_bridge`: optional `/cmd_vel` to Unitree Sport API command bridge with
  clamps and watchdog.
- `topic_monitor`: report expected raw Go2 topic availability as JSON.
- `camera_bridge`: optional native front-video bridge, left available but not
  launched by default.

## Important Design Rules

- Keep raw Unitree DDS and clean ROS 2 output on separate domains.
- Restamp raw Go2 sensor/odom messages before feeding Nav2, SLAM, or TF
  consumers unless clocks are synchronized.
- Keep `/cmd_vel` disabled by default. Turn it on only for controlled tests.
- Use host networking in WendyOS so DDS multicast and robot APIs are reachable.
- Avoid adding autonomy packages here. Downstream stacks should depend on this
  bridge surface or be copied into a new Wendy app.
