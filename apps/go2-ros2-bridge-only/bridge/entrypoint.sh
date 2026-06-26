#!/usr/bin/env bash
set -euo pipefail

export GO2_IP="${GO2_IP:-192.168.123.161}"
export GO2_ROS_DOMAIN_ID="${GO2_ROS_DOMAIN_ID:-0}"
export BRIDGE_ROS_DOMAIN_ID="${BRIDGE_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-30}}"
export ROS_DOMAIN_ID="${BRIDGE_ROS_DOMAIN_ID}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
unset ROS_LOCALHOST_ONLY

for arg in "$@"; do
    case "$arg" in
        cmd-vel|enable-cmd-vel|--cmd-vel|--enable-cmd-vel)
            export ENABLE_CMD_VEL=true
            ;;
        no-cmd-vel|disable-cmd-vel|--no-cmd-vel|--disable-cmd-vel)
            export ENABLE_CMD_VEL=false
            ;;
        no-foxglove|disable-foxglove|--no-foxglove|--disable-foxglove)
            export LAUNCH_FOXGLOVE=false
            ;;
        usb-camera|enable-usb-camera|--usb-camera|--enable-usb-camera)
            export ENABLE_USB_CAMERA=true
            ;;
        no-restamp|--no-restamp|source-time|--source-time)
            export RESTAMP_WITH_ROS_TIME=false
            ;;
    esac
done

echo ">>> Auto-detecting DDS bind address for GO2_IP (${GO2_IP})..."
read -r IFNAME LOCAL_IP <<EOF
$(python3 - "$GO2_IP" <<'PY'
import os
import socket
import subprocess
import sys

target = sys.argv[1]
local = os.environ.get("GO2_DDS_ADDRESS", "").strip()

if not local:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect((target, 1))
            local = sock.getsockname()[0]
        finally:
            sock.close()
    except OSError:
        local = ""

ifname = ""
if local:
    try:
        out = subprocess.check_output(["ip", "-o", "-4", "addr", "show"], text=True)
        for line in out.splitlines():
            parts = line.split()
            if len(parts) >= 4 and parts[3].split("/", 1)[0] == local:
                ifname = parts[1]
                break
    except Exception:
        pass

print(ifname, local)
PY
)
EOF

if [ -n "${LOCAL_IP:-}" ]; then
    RAW_INTERFACES_XML="      <Interfaces>
        <NetworkInterface address=\"${LOCAL_IP}\" priority=\"default\" multicast=\"default\" />
      </Interfaces>"
    BRIDGE_INTERFACES_XML="      <Interfaces>
        <NetworkInterface address=\"${LOCAL_IP}\" priority=\"default\" multicast=\"default\" />
        <NetworkInterface address=\"127.0.0.1\" priority=\"default\" multicast=\"default\" />
      </Interfaces>"
    echo ">>> Selected DDS bind address: ${LOCAL_IP} (${IFNAME:-unknown interface})"
else
    RAW_INTERFACES_XML=""
    BRIDGE_INTERFACES_XML="      <Interfaces>
        <NetworkInterface address=\"127.0.0.1\" priority=\"default\" multicast=\"default\" />
      </Interfaces>"
    echo ">>> WARNING: no IPv4 route to ${GO2_IP}; CycloneDDS will scan interfaces."
fi

export CYCLONEDDS_FILE="/tmp/cyclonedds.xml"
cat > "$CYCLONEDDS_FILE" <<XML
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="${GO2_ROS_DOMAIN_ID}">
    <General>
${RAW_INTERFACES_XML}
      <AllowMulticast>true</AllowMulticast>
      <EnableMulticastLoopback>true</EnableMulticastLoopback>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
  <Domain Id="${BRIDGE_ROS_DOMAIN_ID}">
    <General>
${BRIDGE_INTERFACES_XML}
      <AllowMulticast>true</AllowMulticast>
      <EnableMulticastLoopback>true</EnableMulticastLoopback>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
      <Peers>
        <Peer Address="127.0.0.1" />
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
XML

export CYCLONEDDS_URI="file://${CYCLONEDDS_FILE}"
echo ">>> Wrote CycloneDDS config to $CYCLONEDDS_URI"
echo ">>> Domains: raw Go2=${GO2_ROS_DOMAIN_ID}, standard bridge=${BRIDGE_ROS_DOMAIN_ID}"

source_setup() {
    local setup_file="$1"
    if [ -f "$setup_file" ]; then
        set +u
        # shellcheck disable=SC1090
        source "$setup_file"
        set -u
    fi
}

source_setup /opt/ros/jazzy/setup.bash
source_setup /unitree_ws/install/setup.bash
source_setup /go2_ws/install/setup.bash

PIDS=()

shutdown() {
    for pid in "${PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    wait || true
}

trap shutdown INT TERM

if [ "${LAUNCH_BRIDGE:-true}" = "true" ]; then
    echo ">>> Starting go2_bridge..."
    ros2 launch go2_bridge bridge.launch.py \
        enable_cmd_vel:=${ENABLE_CMD_VEL:-false} \
        enable_monitor:=${ENABLE_TOPIC_MONITOR:-true} \
        enable_usb_camera:=${ENABLE_USB_CAMERA:-false} \
        odom_source:=${ODOM_SOURCE_TOPIC:-/utlidar/robot_odom} \
        odom_output:=${ODOM_OUTPUT_TOPIC:-/odom} \
        restamp_with_ros_time:=${RESTAMP_WITH_ROS_TIME:-true} \
        lowstate_topic:=${LOWSTATE_TOPIC:-/lowstate} &
    PIDS+=("$!")
fi

if [ "${LAUNCH_FOXGLOVE:-true}" = "true" ]; then
    echo ">>> Starting Foxglove bridge on port ${FOXGLOVE_PORT:-8767}..."
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
        port:=${FOXGLOVE_PORT:-8767} \
        address:=0.0.0.0 &
    PIDS+=("$!")
fi

if [ "${#PIDS[@]}" -eq 0 ]; then
    echo "ERROR: no services enabled. Set LAUNCH_BRIDGE=true and/or LAUNCH_FOXGLOVE=true."
    exit 1
fi

echo ">>> Go2 ROS 2 bridge-only app initialized. Waiting..."
set +e
wait -n "${PIDS[@]}"
STATUS="$?"
set -e
shutdown
exit "$STATUS"
