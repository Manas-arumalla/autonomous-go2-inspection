#!/usr/bin/env python3
"""Bridge: /cmd_vel (geometry_msgs/Twist) -> /api/sport/request.

Converts standard ROS2 velocity commands into Unitree Sport API requests
so Nav2 and teleop can drive the Go2.

Sport API command IDs:
  1003 = StopMove
  1004 = StandUp
  1005 = StandDown
  1006 = RecoveryStand
  1008 = Move (velocity command)

Velocity clamps (from official go2-rc template):
  vx:   +/-0.6  m/s
  vy:   +/-0.4  m/s
  vyaw: +/-1.0  rad/s

Safety: a watchdog sends StopMove if no cmd_vel arrives within timeout.
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist

from go2_bridge.domains import bridge_domain_id, go2_domain_id, init_context, spin_nodes

# Unitree message — built from unitree_ros2 repo
try:
    from unitree_api.msg import Request
    _HAS_UNITREE_API = True
except ImportError:
    _HAS_UNITREE_API = False

# Sport API command IDs
SPORT_API_ID_STOPMOVE = 1003
SPORT_API_ID_STANDUP = 1004
SPORT_API_ID_MOVE = 1008


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class CmdVelBridge(Node):
    """Bridges /cmd_vel to Unitree Sport API velocity commands."""

    def __init__(self, command_context, go2_context) -> None:
        super().__init__('cmd_vel_bridge', context=command_context)
        self._go2_node = Node('cmd_vel_bridge_go2_output', context=go2_context)

        if not _HAS_UNITREE_API:
            self.get_logger().fatal(
                'unitree_api package not found. Build unitree_ros2 first.'
            )
            raise SystemExit(1)

        # --- Parameters ---
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('sport_request_topic', '/api/sport/request')
        self.declare_parameter('max_vx', 0.6)
        self.declare_parameter('max_vy', 0.4)
        self.declare_parameter('max_vyaw', 1.0)
        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter('watchdog_rate', 10.0)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        sport_topic = self.get_parameter('sport_request_topic').value
        self._max_vx = self.get_parameter('max_vx').value
        self._max_vy = self.get_parameter('max_vy').value
        self._max_vyaw = self.get_parameter('max_vyaw').value
        self._watchdog_timeout = self.get_parameter('watchdog_timeout').value
        watchdog_rate = self.get_parameter('watchdog_rate').value

        # --- Subscriber ---
        self._sub = self.create_subscription(
            Twist, cmd_vel_topic, self._cmd_vel_cb, 10
        )

        # --- Publisher ---
        go2_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self._go2_node.create_publisher(Request, sport_topic, go2_qos)

        # --- Watchdog ---
        self._last_cmd_time = 0.0
        self._was_moving = False
        self._request_id = 0
        self._watchdog_timer = self.create_timer(
            1.0 / watchdog_rate, self._watchdog_cb
        )

        self.get_logger().info(
            f'cmd_vel_bridge: {cmd_vel_topic} -> {sport_topic}\n'
            f'  velocity clamps: vx=+/-{self._max_vx}, '
            f'vy=+/-{self._max_vy}, vyaw=+/-{self._max_vyaw}\n'
            f'  watchdog timeout: {self._watchdog_timeout}s'
        )

    @property
    def go2_node(self) -> Node:
        return self._go2_node

    def _next_id(self) -> int:
        self._request_id += 1
        return self._request_id

    def _make_request(self, api_id: int, parameter: str = '') -> Request:
        """Build a unitree_api/msg/Request."""
        req = Request()
        try:
            req.header.identity.id = self._next_id()
            req.header.identity.api_id = api_id
        except AttributeError:
            # Fallback if the message structure differs
            self.get_logger().warn(
                'Request.header.identity structure not as expected. '
                'Check unitree_api/msg/Request fields.',
                once=True,
            )
        req.parameter = parameter
        return req

    def _cmd_vel_cb(self, msg: Twist) -> None:
        vx = _clamp(msg.linear.x, -self._max_vx, self._max_vx)
        vy = _clamp(msg.linear.y, -self._max_vy, self._max_vy)
        vyaw = _clamp(msg.angular.z, -self._max_vyaw, self._max_vyaw)

        param = json.dumps({'x': vx, 'y': vy, 'z': vyaw})
        req = self._make_request(SPORT_API_ID_MOVE, param)
        self._pub.publish(req)

        self._last_cmd_time = time.monotonic()
        self._was_moving = True

    def _watchdog_cb(self) -> None:
        """Send StopMove if no cmd_vel received within timeout."""
        if not self._was_moving:
            return

        elapsed = time.monotonic() - self._last_cmd_time
        if elapsed > self._watchdog_timeout:
            req = self._make_request(SPORT_API_ID_STOPMOVE)
            self._pub.publish(req)
            self._was_moving = False
            self.get_logger().info(
                f'Watchdog: no cmd_vel for {elapsed:.2f}s, sent StopMove'
            )

    def destroy_node(self) -> bool:
        self._go2_node.destroy_node()
        return super().destroy_node()


def main(args=None):
    del args
    command_domain = bridge_domain_id()
    go2_domain = go2_domain_id()
    command_context = init_context(command_domain)
    go2_context = init_context(go2_domain)
    node = CmdVelBridge(command_context, go2_context)
    node.get_logger().info(
        f'cmd_vel_bridge domains: command source={command_domain}, '
        f'Go2 output={go2_domain}'
    )
    try:
        spin_nodes([
            (node, command_context),
            (node.go2_node, go2_context),
        ])
    except KeyboardInterrupt:
        pass
    finally:
        # Send a final StopMove on shutdown
        try:
            req = node._make_request(SPORT_API_ID_STOPMOVE)
            node._pub.publish(req)
            node.get_logger().info('Shutdown: sent StopMove')
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown(context=command_context)
        rclpy.try_shutdown(context=go2_context)


if __name__ == '__main__':
    main()
