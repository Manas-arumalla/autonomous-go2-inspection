# Go2 ROS 2 bridge-only WendyOS image.
# Base: ROS 2 Jazzy + CycloneDDS + Unitree interface packages + go2_bridge.
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      git \
      iproute2 \
      iputils-ping \
      python3-colcon-common-extensions \
      python3-pip \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-rosidl-generator-dds-idl \
      ros-jazzy-tf2-ros \
      ros-jazzy-tf2-geometry-msgs \
      ros-jazzy-foxglove-bridge \
      ros-jazzy-v4l2-camera \
      ros-jazzy-image-transport-plugins \
      tini \
    && rm -rf /var/lib/apt/lists/*

# Build official Unitree ROS 2 message/interface packages only.
RUN git clone --depth 1 https://github.com/unitreerobotics/unitree_ros2 /tmp/unitree_ros2 \
    && mkdir -p /unitree_ws/src \
    && for d in $(find /tmp/unitree_ros2 -name package.xml -exec dirname {} \;); do \
         name="$(basename "$d")"; \
         case "$name" in \
           unitree_*|common) echo ">>> copying $name"; cp -r "$d" /unitree_ws/src/ ;; \
         esac; \
       done \
    && source /opt/ros/jazzy/setup.bash \
    && cd /unitree_ws \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log /tmp/unitree_ros2

COPY ros2_ws/src /go2_ws/src

RUN source /opt/ros/jazzy/setup.bash \
    && source /unitree_ws/install/setup.bash \
    && cd /go2_ws \
    && colcon build \
    && rm -rf build log

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV ROS_DOMAIN_ID=30 \
    BRIDGE_ROS_DOMAIN_ID=30 \
    GO2_ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    GO2_IP=192.168.123.161 \
    GO2_DDS_ADDRESS="" \
    LAUNCH_BRIDGE=true \
    LAUNCH_FOXGLOVE=true \
    ENABLE_CMD_VEL=true \
    ENABLE_TOPIC_MONITOR=true \
    ENABLE_USB_CAMERA=false \
    RESTAMP_WITH_ROS_TIME=true \
    FOXGLOVE_PORT=8767

EXPOSE 8767

ENTRYPOINT ["/usr/bin/tini", "--", "/entrypoint.sh"]
