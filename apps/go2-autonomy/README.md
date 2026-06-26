# Go2 Autonomy (WendyOS app)

The autonomous **mapping + navigation + gauge-inspection** stack for the Unitree Go2, ported from the
Gazebo simulation (`go2-sim/`) to run on the real robot via WendyOS. It runs **on top of** the
[`go2-ros2-bridge-only`](../go2-ros2-bridge-only) app and consumes that app's clean ROS 2 contract on
**domain 30** — it never touches the raw Go2 DDS (domain 0).

This is a *copy* of the sim ROS packages, modified for hardware. The sim stays untouched in `go2-sim/`.

## Layering

```
 Unitree Go2  --DDS dom 0-->  go2-ros2-bridge-only  --dom 30-->  go2-autonomy (this app)
                              /odom /tf /joint_states            RTAB-Map SLAM/loc  -> map->odom, /map
                              /pointcloud /imu                   pointcloud->/scan  (Nav2 + frontier)
                              /cmd_vel -> Sport API   <---------  Nav2 controller -> /cmd_vel
                                                                  frontier_explorer (exploration)
                                                                  mission_control (12 services)
                                                                  zone_wall_follower + YOLOE (inspection)
```

The bridge owns the hardware; this app owns autonomy. To **move** the robot, the bridge must be deployed
with `enable-cmd-vel` (it forwards `/cmd_vel` to the Sport API). This app just publishes `/cmd_vel`.

## What this app adds (vs. the bridge)

- `base_link -> utlidar_lidar / camera_link / camera_link_optical` **static TF** (the bridge gives only
  `odom -> base_link`). Values lifted from the sim URDF — **verify on the real robot** (see SIM-TO-REAL.md).
- `self_filter`: drop the dog's body from `/pointcloud` -> `/utlidar/cloud_filtered`.
- `pointcloud_to_laserscan`: `/utlidar/cloud_filtered` -> `/scan` (Nav2 obstacle layer + frontier).
- **RTAB-Map** (pure-lidar SLAM or localization) -> `map->odom` + `/map`.
- **Nav2** (controller publishes `/cmd_vel`).
- **frontier_explorer**, **mission_control** (12 services), **zone_wall_follower + yoloe_segmenter**.
- optional **USB camera** (`/camera/image_raw` + `/camera/camera_info`) for inspection.

Everything runs with **`use_sim_time:=false`** (the real Go2 has no `/clock`).

## Deploy

The bridge must be running first (see `../go2-ros2-bridge-only`), with `enable-cmd-vel` to allow motion.

```bash
# 1) build + deploy this app (detached). MODE defaults to mapping.
wendy --device <device> run --prefix apps/go2-autonomy --yes --detach --debug

# inspection mode (localizes on a saved map, runs the inspection services):
wendy --device <device> run --prefix apps/go2-autonomy --yes --detach --debug \
  --user-args inspection enable-camera
```

Runtime `--user-args`: `mapping` | `inspection` | `enable-camera` | `enable-octomap` | `continue-map` |
`no-mission-control`. Other knobs are env vars (`MAP_YAML`, `CAMERA_DEVICE`, `GO2_ZONES`, ...) — see the
Dockerfile `ENV` block.

## Drive it (services, domain 30)

```bash
export T=go2_inspection_interfaces/srv/ZoneTask
# MAPPING
wendy --device <device> device ros2 service call /start_exploration $T "{}" --domain 30
wendy --device <device> device ros2 service call /get_status        $T "{}" --domain 30
wendy --device <device> device ros2 service call /save_map          $T "{}" --domain 30
# INSPECTION (after redeploying with --user-args inspection)
wendy --device <device> device ros2 service call /inspect_zone $T "{zone_id: zone_1, read: false}" --domain 30
wendy --device <device> device ros2 service call /get_zone_image $T "{zone_id: zone_1}" --domain 30
```

## Verify the contract

```bash
wendy --device <device> device ros2 topics --domain 30           # expect /scan, /map, /cmd_vel, /odom, /tf
wendy --device <device> device ros2 echo /scan --domain 30 --count 1
wendy --device <device> device ros2 run tf2_ros tf2_echo map base_link --domain 30
```

## Known open items (flagged for on-robot work)

1. **Sensor mounts** (`base_link->utlidar_lidar` etc.) are the sim's modelled values — confirm against the
   real Go2's `/utlidar/cloud_deskewed` frame and L1 placement; adjust the static-TF args in
   `real_bringup.launch.py` if the scan is tilted/offset.
2. **Camera calibration**: set `camera_info_url` to a real C920 calibration so the panorama `ppm` (and any
   future projection) is correct. The inspection sweeper is camera-frame + odometry only, so the camera
   *mount* is not safety-critical, but the *intrinsics* matter for gauge sizing.
3. **YOLOE on arm64**: torch/ultralytics is heavy on the 8 GB Orin and is left commented in the Dockerfile.
   The segmenter degrades gracefully without it (empty detections). Enable when ready.
4. **`/save_map`** expects the map helper scripts under `$GO2_WORKSPACE/maps` (shipped) + a writable
   `/maps`; review paths before relying on on-robot remapping.

See `docs/ARCHITECTURE.md` and `docs/SIM-TO-REAL.md`.
