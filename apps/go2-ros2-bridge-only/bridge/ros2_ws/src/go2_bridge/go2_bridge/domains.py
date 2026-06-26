"""ROS domain helpers for the Go2 bridge.

The Go2 publishes its built-in DDS graph on domain 0 with older discovery
metadata. The bridge republishes standard ROS 2 topics on a clean domain so
normal tooling does not need to parse the raw Unitree graph.
"""

from __future__ import annotations

import os
from collections.abc import Iterable
from threading import Thread

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


def _env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return int(value)


def go2_domain_id() -> int:
    return _env_int("GO2_ROS_DOMAIN_ID", 0)


def bridge_domain_id() -> int:
    return _env_int("BRIDGE_ROS_DOMAIN_ID", _env_int("ROS_DOMAIN_ID", 30))


def init_context(domain_id: int) -> Context:
    context = Context()
    rclpy.init(args=None, context=context, domain_id=domain_id)
    return context


def spin_node(node: Node, context: Context) -> None:
    spin_nodes([(node, context)])


def spin_nodes(node_contexts: Iterable[tuple[Node, Context]]) -> None:
    """Spin nodes that may belong to different rclpy contexts."""
    executors: list[SingleThreadedExecutor] = []
    threads: list[Thread] = []

    for node, context in node_contexts:
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executors.append(executor)

    if not executors:
        return

    for executor in executors[1:]:
        thread = Thread(target=executor.spin, daemon=True)
        thread.start()
        threads.append(thread)

    try:
        executors[0].spin()
    finally:
        for executor in executors:
            executor.shutdown()
        for thread in threads:
            thread.join(timeout=2.0)
        for executor in executors:
            for node in list(executor.get_nodes()):
                executor.remove_node(node)
