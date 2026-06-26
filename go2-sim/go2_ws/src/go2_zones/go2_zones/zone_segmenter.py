#!/usr/bin/env python3
"""zone_segmenter -- split an occupancy grid (RTAB-Map /map, or a saved map) into topological ZONES
(rooms), for the autonomous gauge-inspection sweep (Phase 1).

Algorithm: free mask -> distance transform -> threshold to ROOM CORES (open interiors; narrow corridors
& doorways fall below the threshold) -> connected components = seeds -> watershed floods each seed across
the free space, meeting at walls and mid-doorway -> one label per room. Each zone -> {id, center, polygon,
area} in MAP coordinates. Works on any nav_msgs/OccupancyGrid, so it ports to the live rtabmap /map.

Two ways to run:
  ROS node:     ros2 run go2_zones zone_segmenter            # subscribes /map -> /zones markers + zones.yaml
  standalone:   python3 zone_segmenter.py <facility_map.npz> # offline dev/test -> zones.yaml + zones_viz.png
"""
import os, sys, json
import numpy as np
import cv2


def segment_grid(grid, res, ox, oy, core_dist_m=1.6, min_area_m2=4.0):
    """grid: int16 HxW (-1 unknown / 0 free / 100 occupied). Returns (zones, label_markers)."""
    H, W = grid.shape
    free = (grid == 0).astype(np.uint8)
    # NOTE: do NOT morphologically close the free mask -- a kernel >= wall thickness (0.15m = 3 cells)
    # bridges the thin walls and fuses all rooms into one blob. (For noisy real maps, denoise with a
    # method that respects walls, e.g. fill only small unknown holes fully enclosed by free.)
    dist = cv2.distanceTransform(free, cv2.DIST_L2, 5)                          # cells to nearest non-free
    cores = (dist > core_dist_m / res).astype(np.uint8)                        # open room interiors
    ncores, seeds = cv2.connectedComponents(cores)                             # 0=bg, 1..ncores-1 = a core
    markers = np.zeros((H, W), np.int32)
    markers[free == 0] = 1                          # walls/unknown = barrier (background marker)
    markers[seeds > 0] = seeds[seeds > 0] + 1       # room cores -> labels >= 2
    cv2.watershed(cv2.cvtColor(free * 255, cv2.COLOR_GRAY2BGR), markers)        # flood free space from cores
    zones = []
    for lab in range(2, int(markers.max()) + 1):
        m = (markers == lab).astype(np.uint8)
        area = float(m.sum()) * res * res
        if area < min_area_m2:
            continue
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue
        c = max(cnts, key=cv2.contourArea)
        poly = cv2.approxPolyDP(c, 0.12 / res, True).reshape(-1, 2)
        M = cv2.moments(m, binaryImage=True)
        cx, cy = M["m10"] / M["m00"], M["m01"] / M["m00"]
        zones.append({
            "center": [round(ox + cx * res, 3), round(oy + cy * res, 3)],
            "polygon": [[round(ox + px * res, 3), round(oy + py * res, 3)] for px, py in poly],
            "area": round(area, 2),
            "_label": lab,
        })
    zones.sort(key=lambda z: -z["area"])
    for i, z in enumerate(zones):
        z["id"] = f"zone_{i}"
    return zones, markers


def _viz(grid, markers, zones, res, ox, oy, path):
    H, W = grid.shape
    rng = np.random.default_rng(0)
    out = np.full((H, W, 3), 60, np.uint8)
    out[grid == 0] = (230, 230, 230)
    out[grid == 100] = (0, 0, 0)
    for z in zones:
        col = tuple(int(c) for c in rng.integers(60, 255, 3))
        out[markers == z["_label"]] = col
    out[markers == -1] = (0, 0, 0)            # watershed lines
    out = cv2.flip(out, 0)                     # flip so +Y is up, THEN draw labels (upright)
    for z in zones:
        cx = int((z["center"][0] - ox) / res); cy = H - 1 - int((z["center"][1] - oy) / res)
        cv2.circle(out, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(out, z["id"].replace("zone_", "Z"), (cx - 12, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.imwrite(path, out)


def main_standalone(npz_path):
    d = np.load(npz_path)
    grid = d["grid"]; res = float(d["res"]); ox = float(d["origin_x"]); oy = float(d["origin_y"])
    zones, markers = segment_grid(grid, res, ox, oy)
    out_dir = os.path.dirname(os.path.abspath(npz_path))
    clean = [{k: v for k, v in z.items() if k != "_label"} for z in zones]
    with open(os.path.join(out_dir, "zones.yaml"), "w") as f:
        json.dump({"zones": clean}, f, indent=2)
    _viz(grid, markers, zones, res, ox, oy, os.path.join(out_dir, "zones_viz.png"))
    print(f"{len(zones)} zones -> {out_dir}/zones.yaml + zones_viz.png")
    for z in zones:
        print(f"  {z['id']}: center={z['center']} area={z['area']}m2 ({len(z['polygon'])}-pt polygon)")


# ---- ROS 2 node ----
def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from nav_msgs.msg import OccupancyGrid
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point

    class ZoneSegmenter(Node):
        def __init__(self):
            super().__init__("zone_segmenter")
            self.declare_parameter("use_sim_time", True)
            self.declare_parameter("core_dist_m", 1.6)
            self.declare_parameter("out_path", os.path.expanduser("~/zones.yaml"))
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
            self.sub = self.create_subscription(OccupancyGrid, "/map", self.cb, qos)
            self.pub = self.create_publisher(MarkerArray, "/zones", 1)
            self.get_logger().info("zone_segmenter: waiting for /map ...")

        def cb(self, m):
            grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
            zones, _ = segment_grid(grid, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y,
                                    core_dist_m=self.get_parameter("core_dist_m").value)
            clean = [{k: v for k, v in z.items() if k != "_label"} for z in zones]
            with open(self.get_parameter("out_path").value, "w") as f:
                json.dump({"zones": clean}, f, indent=2)
            self.publish_markers(zones)
            self.get_logger().info(f"segmented {len(zones)} zones -> {self.get_parameter('out_path').value}")

        def publish_markers(self, zones):
            arr = MarkerArray()
            dele = Marker(); dele.action = Marker.DELETEALL; arr.markers.append(dele)
            for i, z in enumerate(zones):
                ls = Marker(); ls.header.frame_id = "map"; ls.ns = "zone_poly"; ls.id = i
                ls.type = Marker.LINE_STRIP; ls.action = Marker.ADD; ls.scale.x = 0.06
                ls.color.g = 1.0; ls.color.a = 1.0; ls.pose.orientation.w = 1.0
                for px, py in z["polygon"] + [z["polygon"][0]]:
                    ls.points.append(Point(x=float(px), y=float(py), z=0.05))
                arr.markers.append(ls)
                tx = Marker(); tx.header.frame_id = "map"; tx.ns = "zone_label"; tx.id = i
                tx.type = Marker.TEXT_VIEW_FACING; tx.action = Marker.ADD; tx.scale.z = 0.6
                tx.color.r = 1.0; tx.color.g = 1.0; tx.color.b = 1.0; tx.color.a = 1.0
                tx.pose.position.x = float(z["center"][0]); tx.pose.position.y = float(z["center"][1])
                tx.pose.position.z = 0.3; tx.pose.orientation.w = 1.0; tx.text = z["id"]
                arr.markers.append(tx)
            self.pub.publish(arr)

    rclpy.init(args=args)
    n = ZoneSegmenter()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1].endswith(".npz"):
        main_standalone(sys.argv[1])
    else:
        main()
