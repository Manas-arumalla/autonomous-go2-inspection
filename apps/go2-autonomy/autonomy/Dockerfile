# syntax=docker/dockerfile:1
# Go2 autonomy stack (RTAB-Map + Nav2 + frontier + inspection) WendyOS image, arm64 / ROS 2 Jazzy.
# Runs ON TOP of the go2-ros2-bridge-only app (domain 30): subscribes /odom,/tf,/pointcloud,/imu and
# publishes /cmd_vel. NO Gazebo / CHAMP / unitree_ros2 here (the bridge owns the hardware contract).
#
# NOTE (arm64 build gotcha): `wendy run` builds with --platform wendyos/arm64. If the FROM line fails with
# "no match for platform in manifest", either build natively on the Jetson, or pin the base image:
#   FROM --platform=linux/arm64 ros:jazzy-ros-base
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

# --- ROS runtime deps for the autonomy stack ---
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl git iproute2 iputils-ping tini ccache \
      python3-colcon-common-extensions python3-pip python3-opencv \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-rtabmap-ros \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
      ros-jazzy-pointcloud-to-laserscan \
      ros-jazzy-robot-localization \
      ros-jazzy-twist-mux \
      ros-jazzy-octomap-server \
      ros-jazzy-robot-state-publisher ros-jazzy-xacro \
      ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
      ros-jazzy-v4l2-camera ros-jazzy-image-transport-plugins ros-jazzy-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# --- gauge reader (LLM) -- small. The gauge segmenter's YOLOE/torch is HEAVY on arm64 and is OPTIONAL
#     (yoloe_segmenter degrades gracefully if ultralytics/torch is absent). Uncomment to enable on-device
#     detection (expect a much larger/slower image; prefer a Jetson-native torch/dustynv base if you do). ---
RUN pip3 install --no-cache-dir --break-system-packages anthropic
# RUN pip3 install --no-cache-dir --break-system-packages ultralytics

# Bump REBUILD to force `wendy run` to rebuild when only COPYed source changed (wendy skips the build when
# its content hash is unchanged; this ARG changes the Dockerfile so the build actually re-runs). apt/pip
# layers above stay cached; the COPY + colcon layers below re-run and pick up the new source.
ARG REBUILD=2

# --- build our workspace (the copied autonomy packages; no unitree_ros2 needed) ---
# ccache + a BuildKit cache mount persist the compiler object-file cache OUTSIDE this layer, so even when
# the layer re-runs from scratch (source change or REBUILD bump) the C++ compiles hit the cache and finish
# fast. Cache lives on the build host's BuildKit store (build on the Jetson for max reuse / no emulation).
# `ccache -s` prints hit stats. If `wendy run` errors on the --mount line, BuildKit isn't active.
COPY ros2_ws/src /autonomy_ws/src
RUN --mount=type=cache,target=/root/.ccache,sharing=locked \
    source /opt/ros/jazzy/setup.bash \
    && cd /autonomy_ws \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_C_COMPILER_LAUNCHER=ccache \
         -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    && ccache -s \
    && rm -rf build log

# map-persistence helper scripts used by the /save_map service (see entrypoint MODE=mapping)
COPY maps /autonomy_ws/maps

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV ROS_DOMAIN_ID=30 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
    GO2_WORKSPACE=/autonomy_ws \
    GO2_MAPS=/maps \
    RTABMAP_DB=/maps/rtabmap.db \
    GAUGES_ROOT=/data/gauges \
    GO2_ZONES=/maps/facility_inspection_zones.yaml \
    GAUGE_PYTHON=/usr/bin/python3 \
    USE_SIM_TIME=false \
    MODE=mapping \
    CLOUD_TOPIC=/pointcloud \
    ODOM_TOPIC=/odom \
    MAP_YAML="" \
    ENABLE_CAMERA=false \
    CAMERA_DEVICE=/dev/video0 \
    CAMERA_INFO_URL="" \
    ENABLE_OCTOMAP=false \
    LAUNCH_MISSION_CONTROL=true

ENTRYPOINT ["/usr/bin/tini", "--", "/entrypoint.sh"]
