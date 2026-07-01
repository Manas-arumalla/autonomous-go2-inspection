#!/usr/bin/env bash
# Go2 autonomy stack entrypoint. Joins the bridge's clean ROS 2 domain 30 (same host, host networking),
# then launches the real-robot bringup (RTAB-Map + Nav2 + pointcloud->scan + sensor TF) and the
# mission-control service layer. The go2-ros2-bridge-only app MUST already be running (it owns
# /odom,/tf,/pointcloud,/imu and forwards /cmd_vel -> Sport API; deploy it with enable-cmd-vel to move).
set -euo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
unset ROS_LOCALHOST_ONLY || true

# runtime flags via wendy --user-args (mapping is the default)
for arg in "$@"; do
    case "$arg" in
        mapping|--mapping)         export MODE=mapping ;;
        inspection|--inspection)   export MODE=inspection ;;
        camera|enable-camera|--enable-camera)   export ENABLE_CAMERA=true ;;
        no-camera|--no-camera)     export ENABLE_CAMERA=false ;;
        octomap|enable-octomap|--enable-octomap) export ENABLE_OCTOMAP=true ;;
        continue-map|--continue-map)             export CONTINUE_MAP=true ;;
        no-mission-control|--no-mission-control) export LAUNCH_MISSION_CONTROL=false ;;
    esac
done
export MODE="${MODE:-mapping}"
export ENABLE_CAMERA="${ENABLE_CAMERA:-false}"
export ENABLE_OCTOMAP="${ENABLE_OCTOMAP:-false}"
export CONTINUE_MAP="${CONTINUE_MAP:-false}"
export USE_SIM_TIME="${USE_SIM_TIME:-false}"

# --- CycloneDDS: discover the bridge on domain 30 (both containers are host-networked on this Jetson) ---
export CYCLONEDDS_FILE="/tmp/cyclonedds.xml"
cat > "$CYCLONEDDS_FILE" <<XML
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="${ROS_DOMAIN_ID}">
    <General>
      <AllowMulticast>true</AllowMulticast>
      <EnableMulticastLoopback>true</EnableMulticastLoopback>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
      <Peers><Peer Address="127.0.0.1" /></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
XML
export CYCLONEDDS_URI="file://${CYCLONEDDS_FILE}"

source_setup() { [ -f "$1" ] && { set +u; source "$1"; set -u; }; }
source_setup /opt/ros/jazzy/setup.bash
source_setup /autonomy_ws/install/setup.bash

echo ">>> Go2 autonomy: MODE=${MODE} domain=${ROS_DOMAIN_ID} use_sim_time=${USE_SIM_TIME} camera=${ENABLE_CAMERA}"

# --- wait (best-effort) for the bridge's /odom on domain 30 before starting RTAB-Map ---
echo ">>> Waiting up to 30s for the bridge (/odom on domain ${ROS_DOMAIN_ID})..."
for _ in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -q '^/odom$'; then echo ">>> bridge detected (/odom)."; break; fi
    sleep 1
done

# inspection mode: a static map_server owns /map (Nav2 global static_layer needs it), so RTAB-Map publishes
# its grid on /rtabmap/grid_map. The map MUST exist (built by MODE=mapping + /save_map) or Nav2's global
# costmap stalls with no /map -> default MAP_YAML to the saved map and fail fast if it isn't there yet.
GRID_TOPIC="/map"
if [ "${MODE}" = "inspection" ]; then
    GRID_TOPIC="/rtabmap/grid_map"
    export MAP_YAML="${MAP_YAML:-/maps/facility_inspection_map.yaml}"
    if [ ! -f "${MAP_YAML}" ]; then
        echo ">>> ERROR: MODE=inspection but map '${MAP_YAML}' not found."
        echo ">>>        Run MODE=mapping first, /save_map, then redeploy with --user-args inspection."
        exit 1
    fi
    echo ">>> inspection map: ${MAP_YAML}  (rtabmap localizes off ${RTABMAP_DB:-/maps/rtabmap.db})"
fi
[ -z "${ANTHROPIC_API_KEY:-}" ] && echo ">>> WARN: ANTHROPIC_API_KEY unset -- on-device LLM gauge READING will be SKIPPED (crops + segmentation still run; read them off-device via /get_zone_image)."

PIDS=()
shutdown() { for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done; wait || true; }
trap shutdown INT TERM

echo ">>> Launching real_bringup (${MODE})..."
# Build args as an array; ros2 launch REJECTS an empty value (e.g. `map_yaml:=`), so only pass map_yaml
# when it's actually set (inspection mode). In mapping mode the launch's own default ("") applies.
LAUNCH_ARGS=(
    use_sim_time:=${USE_SIM_TIME}
    mode:=${MODE}
    cloud_topic:=${CLOUD_TOPIC:-/pointcloud}
    odom_topic:=${ODOM_TOPIC:-/odom}
    grid_topic:=${GRID_TOPIC}
    continue_map:=${CONTINUE_MAP}
    enable_camera:=${ENABLE_CAMERA}
    camera_device:=${CAMERA_DEVICE:-/dev/video0}
    camera_info_url:=${CAMERA_INFO_URL:-package://go2_bringup/config/camera_generic_720p.yaml}
    enable_octomap:=${ENABLE_OCTOMAP}
)
[ -n "${MAP_YAML:-}" ] && LAUNCH_ARGS+=( map_yaml:=${MAP_YAML} )
ros2 launch go2_bringup real_bringup.launch.py "${LAUNCH_ARGS[@]}" &
PIDS+=("$!")

if [ "${LAUNCH_MISSION_CONTROL:-true}" = "true" ]; then
    echo ">>> Launching mission_control (12 services)..."
    ros2 launch go2_bringup mission_control.launch.py \
        use_sim_time:=${USE_SIM_TIME} \
        zones_file:="${GO2_ZONES:-/maps/facility_inspection_zones.yaml}" \
        workspace:="${GO2_WORKSPACE:-/autonomy_ws}" &
    PIDS+=("$!")
fi

echo ">>> Go2 autonomy initialized. Waiting..."
set +e
wait -n "${PIDS[@]}"
STATUS="$?"
set -e
shutdown
exit "$STATUS"
