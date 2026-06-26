# Sim -> Real: what changed from `go2-sim`

This app is a copy of the `go2-sim` ROS packages with the simulator removed and the hardware contract
swapped in. The team built the sim to mirror the real `/utlidar/*` topics and the `utlidar_lidar` frame,
so the delta is small and mostly mechanical.

## Topic / clock / frame deltas

| Concern | Sim (`go2-sim`) | Real (this app, on the bridge) | Where |
|---|---|---|---|
| Locomotion | CHAMP gait + `gz_ros2_control` consume `/cmd_vel` | bridge forwards `/cmd_vel` -> Sport API | (dropped CHAMP) |
| Point cloud | gz lidar -> `/utlidar/cloud_deskewed` | bridge `/pointcloud` (= cloud_deskewed) | `self_filter input_topic:=/pointcloud` |
| `/scan` | `pointcloud_to_laserscan` on the gz cloud | same node on `/utlidar/cloud_filtered` | `real_bringup.launch.py` |
| Odom | CHAMP + gz EKF -> `/odom`, `odom->base_link` | bridge `/odom` + `odom->base_link` TF | (consumed as-is) |
| `/joint_states` | gz_ros2_control | bridge (from `/lowstate`) | (not required by autonomy) |
| Clock | gz `/clock`, `use_sim_time:=true` | none, **`use_sim_time:=false`** | every launch + the service subprocesses |
| `base_link->sensors` TF | `go2_description` spawned in gz | **3 `static_transform_publisher`** | `real_bringup.launch.py` |
| `map->odom` | RTAB-Map | RTAB-Map (unchanged) | unchanged |
| Camera | gz camera -> `/camera/image_raw`+`/camera/camera_info` | USB C920 via `v4l2_camera` | `enable_camera:=true` |
| Discovery | local gz, default DDS | CycloneDDS, **domain 30**, host net | entrypoint + wendy.json |

## TF tree on the real robot

```
map                      <- RTAB-Map (this app)
 └─ odom                 <- bridge (/odom from /utlidar/robot_odom)
     └─ base_link        <- bridge
         ├─ utlidar_lidar       <- static TF (this app): xyz 0.30 0 -0.02, rpy 0 0.15 0
         ├─ camera_link         <- static TF (this app): xyz 0.30 0 0.06
         │   └─ camera_link_optical  <- static TF: rpy -1.5708 0 -1.5708 (ROS z-forward)
```

`base_link == trunk` on the Go2 (identity `floating_base` in the sim URDF), so the static TFs publish
straight from `base_link`. The leg/foot frames are **not** published — the autonomy stack doesn't use them.
The mount values are the sim's model of the real L1/camera; **verify on the real robot** and tune the
static-TF args in `real_bringup.launch.py` if the `/scan` looks tilted or offset.

## Code changes made to the copies (the sim originals are untouched)

- `use_sim_time` is no longer hard-coded `true`: `inspection_mission.py` reads it from the `-p` flag and
  forwards it to the sweeper/segmenter children; `mission_control_server.py` forwards it to the
  `inspection_mission` / `frontier_explorer` subprocesses; the launches default it to `false`.
- `mission_control_server.py` paths are env-driven (`GO2_WORKSPACE`, `GO2_ZONES`, `GAUGES_ROOT`) instead of
  the laptop path; `_env()` no longer forces `FASTDDS_BUILTIN_TRANSPORTS` (the bridge runs CycloneDDS).
- `inspection_mission.py` `GAUGES_ROOT` / gauge-reader python are env-driven for the persisted volumes.
- New `real_bringup.launch.py` replaces the Gazebo-coupled `rtabmap_slam` / `inspection_nav` launches.
- Dropped packages: `champ*`, `go2_config` (gait), `go2_worlds` (Gazebo), `m-explore-ros2`. Dropped from
  `go2_description`: `meshes/`, `dae/` (50 MB of viz assets `robot_state_publisher` never loads).

## Open items (see README "Known open items")

Sensor-mount verification, camera calibration, optional on-device YOLOE/torch, and the `/save_map` paths.
