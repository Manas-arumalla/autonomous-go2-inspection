#!/usr/bin/env python3
"""Bridge: /lowstate (unitree_go/msg/LowState) -> /joint_states.

Converts Unitree Go2 motor states into standard sensor_msgs/JointState
messages for robot_state_publisher, RViz, and URDF visualization.

Go2 joint ordering (12 actuated joints, 3 per leg):
  Index 0-2:  FR (Front Right) — hip, thigh, calf
  Index 3-5:  FL (Front Left)  — hip, thigh, calf
  Index 6-8:  RR (Rear Right)  — hip, thigh, calf
  Index 9-11: RL (Rear Left)   — hip, thigh, calf
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from go2_bridge.domains import bridge_domain_id, go2_domain_id, init_context, spin_nodes

# Unitree message — built from unitree_ros2 repo
try:
    from unitree_go.msg import LowState
    _HAS_UNITREE = True
except ImportError:
    _HAS_UNITREE = False

# Standard Go2 joint names matching common URDF conventions
GO2_JOINT_NAMES = [
    'FR_hip_joint',   'FR_thigh_joint',   'FR_calf_joint',    # 0-2
    'FL_hip_joint',   'FL_thigh_joint',   'FL_calf_joint',    # 3-5
    'RR_hip_joint',   'RR_thigh_joint',   'RR_calf_joint',    # 6-8
    'RL_hip_joint',   'RL_thigh_joint',   'RL_calf_joint',    # 9-11
]

NUM_JOINTS = 12


class JointStateBridge(Node):
    """Bridges Go2 LowState motor data to sensor_msgs/JointState."""

    def __init__(self, source_context, output_context) -> None:
        super().__init__('joint_state_bridge', context=source_context)
        self._out_node = Node('joint_state_bridge_output', context=output_context)

        if not _HAS_UNITREE:
            self.get_logger().fatal(
                'unitree_go package not found. Build unitree_ros2 first.'
            )
            raise SystemExit(1)

        # --- Parameters ---
        self.declare_parameter('lowstate_topic', '/lowstate')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('base_frame_id', 'base_link')

        lowstate_topic = self.get_parameter('lowstate_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        self._frame_id = self.get_parameter('base_frame_id').value

        # --- QoS: match Go2 DDS publishers ---
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
            LowState, lowstate_topic, self._lowstate_cb, go2_qos
        )

        # --- Publisher ---
        self._pub = self._out_node.create_publisher(
            JointState, joint_states_topic, standard_qos
        )

        self._msg_count = 0
        self.get_logger().info(
            f'joint_state_bridge: {lowstate_topic} -> {joint_states_topic} '
            f'({NUM_JOINTS} joints)'
        )

    @property
    def output_node(self) -> Node:
        return self._out_node

    def _lowstate_cb(self, msg: LowState) -> None:
        self._msg_count += 1
        if self._msg_count == 1:
            motor_count = len(msg.motor_state) if hasattr(msg, 'motor_state') else 0
            self.get_logger().info(
                f'Received first LowState (motor_state count: {motor_count})'
            )

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = self._frame_id
        js.name = list(GO2_JOINT_NAMES)

        positions = []
        velocities = []
        efforts = []

        try:
            motors = msg.motor_state
            for i in range(NUM_JOINTS):
                if i < len(motors):
                    m = motors[i]
                    positions.append(float(m.q))
                    velocities.append(float(m.dq))
                    efforts.append(float(m.tau_est))
                else:
                    positions.append(0.0)
                    velocities.append(0.0)
                    efforts.append(0.0)
        except (AttributeError, IndexError) as exc:
            if self._msg_count <= 3:
                self.get_logger().warn(
                    f'Error reading motor_state: {exc}. '
                    f'Check unitree_go/msg/LowState field names.'
                )
            return

        js.position = positions
        js.velocity = velocities
        js.effort = efforts
        self._pub.publish(js)

    def destroy_node(self) -> bool:
        self._out_node.destroy_node()
        return super().destroy_node()


def main(args=None):
    del args
    source_domain = go2_domain_id()
    output_domain = bridge_domain_id()
    source_context = init_context(source_domain)
    output_context = init_context(output_domain)
    node = JointStateBridge(source_context, output_context)
    node.get_logger().info(
        f'joint_state_bridge domains: Go2 source={source_domain}, '
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
