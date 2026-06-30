#!/usr/bin/env python3
"""Grab the live RTAB-Map 2D occupancy grid (/map) and save it to a .npz (grid, res, origin), printing
per-room coverage. Run while the SLAM stack is still up, after the robot has mapped the facility.

A helper that `mission_control_server.save_map()` invokes; it can also be run standalone:
    python3 map_grab.py out.npz
"""
import sys, time, rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np

out = sys.argv[1] if len(sys.argv) > 1 else "map.npz"
ROOMS = {"corridor": (-15, 15, -1.5, 1.5),
         "NW": (-15, -5, 1.5, 10), "NC": (-5, 5, 1.5, 10), "NE": (5, 15, 1.5, 10),
         "SW": (-15, -5, -10, -1.5), "SC": (-5, 5, -10, -1.5), "SE": (5, 15, -10, -1.5)}


class Grab(Node):
    def __init__(self):
        super().__init__("map_grab")
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, "/map", self.cb, qos)
        self.done = False

    def cb(self, m):
        w, h = m.info.width, m.info.height
        res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
        g = np.array(m.data, dtype=np.int16).reshape(h, w)
        np.savez_compressed(out, grid=g, res=res, origin_x=ox, origin_y=oy)
        print(f"saved {out}: {w}x{h} res={res:.3f} origin=({ox:.2f},{oy:.2f}) free={int(np.sum(g==0))} cells")
        for r, (x0, x1, y0, y1) in ROOMS.items():
            i0 = max(0, int((x0 - ox) / res)); i1 = min(w, int((x1 - ox) / res))
            j0 = max(0, int((y0 - oy) / res)); j1 = min(h, int((y1 - oy) / res))
            sub = g[j0:j1, i0:i1]
            f = int(np.sum(sub == 0)) * 100 // max(1, sub.size) if sub.size else 0
            print(f"   {r:<9} free {f:>3}%   {'OK' if f > 15 else '-- unmapped'}")
        self.done = True


rclpy.init(); n = Grab(); t0 = time.time()
while rclpy.ok() and not n.done and time.time() - t0 < 15:
    rclpy.spin_once(n, timeout_sec=0.5)
if not n.done:
    print("ERROR: no /map received in 15s -- is the SLAM stack up?")
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if n.done else 1)
