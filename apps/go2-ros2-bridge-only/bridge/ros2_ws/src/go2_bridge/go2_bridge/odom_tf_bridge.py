#!/usr/bin/env python3
"""Bridge: /utlidar/robot_odom -> TF (odom->base_link) + /odom.

Subscribes to the Go2 onboard odometry and:
  1. Republishes it on /odom with correct frame IDs for Nav2.
  2. Broadcasts the odom -> base_link TF transform.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from go2_bridge.domains import bridge_domain_id, go2_domain_id, init_context, spin_nodes


class OdomTfBridge(Node):
    """Bridges Go2 odometry to standard /odom + TF."""

    def __init__(self, source_context, output_context) -> None:
        super().__init__('odom_tf_bridge', context=source_context)
        self._out_node = Node('odom_tf_bridge_output', context=output_context)

        # --- Parameters ---
        self.declare_parameter('odom_source_topic', '/utlidar/robot_odom')
        self.declare_parameter('odom_output_topic', '/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('restamp_with_ros_time', True)

        self._odom_source = self.get_parameter('odom_source_topic').value
        self._odom_output = self.get_parameter('odom_output_topic').value
        self._odom_frame = self.get_parameter('odom_frame_id').value
        self._base_frame = self.get_parameter('base_frame_id').value
        self._publish_tf = self.get_parameter('publish_tf').value
        self._restamp = bool(self.get_parameter('restamp_with_ros_time').value)

        # --- QoS: match Go2 DDS publishers (best effort, volatile) ---
        go2_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        standard_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # --- Subscriber ---
        self._sub = self.create_subscription(
            Odometry, self._odom_source, self._odom_cb, go2_qos
        )

        # --- Publisher ---
        self._pub = self._out_node.create_publisher(
            Odometry, self._odom_output, standard_qos
        )

        # --- TF Broadcaster ---
        self._tf_broadcaster: TransformBroadcaster | None = None
        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self._out_node)

        self._msg_count = 0
        self.get_logger().info(
            f'odom_tf_bridge: {self._odom_source} -> '
            f'{self._odom_output} + TF({self._odom_frame}->{self._base_frame}) '
            f'(restamp_with_ros_time={self._restamp})'
        )

    @property
    def output_node(self) -> Node:
        return self._out_node

    def _odom_cb(self, msg: Odometry) -> None:
        self._msg_count += 1
        if self._msg_count == 1:
            self.get_logger().info(
                f'Received first odom message from {self._odom_source}'
            )

        stamp = (
            self._out_node.get_clock().now().to_msg()
            if self._restamp
            else msg.header.stamp
        )

        # Republish with correct frame IDs and a clock domain Nav2 can use.
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame
        out.pose = msg.pose
        out.twist = msg.twist
        self._pub.publish(out)

        # Broadcast TF: odom -> base_link
        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = msg.pose.pose.position.x
            tf.transform.translation.y = msg.pose.pose.position.y
            tf.transform.translation.z = msg.pose.pose.position.z
            tf.transform.rotation = msg.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf)

    def destroy_node(self) -> bool:
        self._out_node.destroy_node()
        return super().destroy_node()


def main(args=None):
    del args
    source_domain = go2_domain_id()
    output_domain = bridge_domain_id()
    source_context = init_context(source_domain)
    output_context = init_context(output_domain)
    node = OdomTfBridge(source_context, output_context)
    node.get_logger().info(
        f'odom_tf_bridge domains: Go2 source={source_domain}, '
        f'standard output={output_domain}'
    )
    try:
        spin_nodes([
            (node, source_context),
            (node.output_node, output_context),
        ])
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown(context=source_context)
        rclpy.try_shutdown(context=output_context)


if __name__ == '__main__':
    main()
