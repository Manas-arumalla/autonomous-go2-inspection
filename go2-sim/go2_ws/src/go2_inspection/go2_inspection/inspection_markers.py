"""inspection_markers.py — publish detected gauges as RViz markers (visualization only).

Reads the per-zone objects.json the inspection writes (~/gauges/<zone>/objects.json) and republishes every
localized object as a MarkerArray on /inspection/objects: a colour-coded sphere at the 3D world position +
a text label (class, and the gauge value if it was read). Purely additive and read-only — it never touches
the inspection/nav nodes; it just makes the inspection output visible in RViz for demos and presentations.

  ros2 run go2_inspection inspection_markers
"""
import glob
import json
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

GAUGES_ROOT = os.path.expanduser("~/gauges")


def _is_gauge(name):
    n = (name or "").lower()
    return "gauge" in n or "dial" in n


class InspectionMarkers(Node):
    def __init__(self):
        super().__init__("inspection_markers")
        self.frame = self.declare_parameter("frame_id", "map").value
        self.root = os.path.expanduser(self.declare_parameter("gauges_root", GAUGES_ROOT).value)
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL  # late-joining RViz still gets the markers
        self.pub = self.create_publisher(MarkerArray, "/inspection/objects", qos)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f"inspection_markers: /inspection/objects from {self.root}/*/objects.json (frame={self.frame})"
        )

    def _objects(self):
        out = []
        for oj in sorted(glob.glob(os.path.join(self.root, "*", "objects.json"))):
            try:
                d = json.load(open(oj))
            except Exception:
                continue
            for o in d.get("objects", []) or []:
                w = o.get("world")
                if w and len(w) >= 2 and o.get("localized", True):
                    out.append(o)
        return out

    def _tick(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        i = 0
        for o in self._objects():
            w = o["world"]
            confirmed = o.get("read_confirmed")
            if confirmed is True:           # close-approach re-detection confirmed it -> green
                col = ColorRGBA(r=0.10, g=0.90, b=0.25, a=0.95)
            elif confirmed is False:        # detected but the close read couldn't confirm -> amber
                col = ColorRGBA(r=0.95, g=0.75, b=0.10, a=0.95)
            else:                           # detected, not approached -> cyan
                col = ColorRGBA(r=0.20, g=0.80, b=0.95, a=0.95)
            z = float(w[2]) if len(w) >= 3 and w[2] is not None else 0.45
            m = Marker()
            m.header.frame_id = self.frame
            m.ns, m.id, m.type, m.action = "gauge", i, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(w[0]), float(w[1]), z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.22
            m.color = col
            arr.markers.append(m)
            i += 1
            t = Marker()
            t.header.frame_id = self.frame
            t.ns, t.id, t.type, t.action = "label", i, Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = float(w[0]), float(w[1]), z + 0.30
            t.pose.orientation.w = 1.0
            t.scale.z = 0.18
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label = "gauge" if _is_gauge(o.get("class", "")) else (o.get("class", "object")[:18])
            gr = o.get("gauge_reading")
            if isinstance(gr, dict) and gr.get("value") is not None:
                label += f"\n{gr.get('value')} {gr.get('unit', '')}".rstrip()
            t.text = label
            arr.markers.append(t)
            i += 1
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    n = InspectionMarkers()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
