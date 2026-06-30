#!/usr/bin/env python3
"""zone_segmenter -- split an occupancy grid (RTAB-Map /map, or a saved map) into topological ZONES
(rooms / coherent open regions) for the autonomous inspection sweep.

Pipeline:
  1. OBSTACLE-ISLAND FILL: free-standing props (drums, pallets, racks, boxes,
     chairs, ...) are OCCUPIED blobs that are NOT part of the building's wall skeleton. Each such island
     used to puncture the free space and split one room into several spurious cores. We reclassify every
     small free-standing occupied island (footprint < island_max_m2, and not spanning > wall_span_m --
     i.e. not a wall run) as FREE before segmenting. The one big connected wall structure is never
     touched. This removes the dominant source of over-fragmentation in cluttered rooms.
  2. distance transform -> threshold to ROOM CORES (open interiors; narrow corridors/doorways fall below
     the threshold) -> connected components = seeds -> watershed floods each seed across free space,
     meeting at walls and mid-doorway -> one label per region.
  3. Each zone -> {id, center, polygon, area, nav_point, label} in MAP coordinates. `nav_point` is the
     deepest ORIGINALLY-free point of the zone (max distance-transform of the un-filled free mask), so a
     navigation goal lands in open space, never on the raw centroid (which can sit on a prop) or on a
     filled island. `label` is a generic quadrant tag (e.g. "NW", "center") for reports/markers only.

NOTE on merging: we deliberately do NOT auto-merge adjacent regions. On open-plan layouts (rooms that
open to a wide corridor) there is no robust LOCAL geometric criterion to tell "intra-room split" from
"room opening onto corridor" -- wall-fraction, region-radius type, neck-clearance ratio, and
neighbour-domination were all tested and each fuses rooms into the corridor (a mixed-room zone, which
breaks inspection semantics). So the invariant we keep is: never emit a mixed-room zone. A room that
ends up split into two coherent halves is only scanned slightly redundantly, never incorrectly. For
worlds with real NARROW doorways the watershed already yields one zone per room. (A wall-flanking
medial-axis door detector is the principled way to also collapse open-plan rooms; left as future work.)

Two ways to run:
  ROS node:     ros2 run go2_zones zone_segmenter            # subscribes /map -> /zones markers + zones.yaml
  standalone:   python3 zone_segmenter.py <facility_map.npz> # offline dev/test -> zones.yaml + zones_viz.png
"""

import os, sys, json
import numpy as np
import cv2


def _island_fill(grid, res, island_max_m2=2.5, wall_span_m=3.0):
    """Return a free mask with small free-standing OCCUPIED islands reclassified as free. The single
    largest connected occupied component (the building wall skeleton) is always kept; any other occupied
    component is kept only if it is large (>= island_max_m2) OR spans > wall_span_m in either axis
    (likely a wall run on a noisy map). Everything else is swallowed into free."""
    free = (grid == 0).astype(np.uint8)
    occ = (grid == 100).astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(occ, 8)
    if n <= 1:
        return free, 0
    wall_lab = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))  # the one big wall structure
    free_clean = free.copy()
    filled = 0
    for l in range(1, n):
        if l == wall_lab:
            continue
        area_m2 = stats[l, cv2.CC_STAT_AREA] * res * res
        span = max(stats[l, cv2.CC_STAT_WIDTH], stats[l, cv2.CC_STAT_HEIGHT]) * res
        if area_m2 < island_max_m2 and span < wall_span_m:
            free_clean[lab == l] = 1  # free-standing prop -> treat as free
            filled += 1
    return free_clean, filled


def _quadrant_label(cx, cy, ox, oy, W, H, res):
    """Generic, world-agnostic location tag from the centroid's third within the map extent."""
    fx = (cx - ox) / (W * res)
    fy = (cy - oy) / (H * res)
    ns = "S" if fy < 1 / 3 else ("N" if fy > 2 / 3 else "")
    we = "W" if fx < 1 / 3 else ("E" if fx > 2 / 3 else "")
    return (ns + we) or "center"


def segment_grid(
    grid, res, ox, oy, core_dist_m=1.6, min_area_m2=6.0, island_max_m2=2.5, wall_span_m=3.0
):
    """grid: int16 HxW (-1 unknown / 0 free / 100 occupied). Returns (zones, label_markers)."""
    H, W = grid.shape
    free_clean, _ = _island_fill(grid, res, island_max_m2, wall_span_m)
    # distance transform of the CLEANED free mask drives cores + watershed (props no longer split rooms);
    # distance transform of the ORIGINAL free mask drives nav_point (so the goal avoids real props too).
    dist = cv2.distanceTransform(free_clean, cv2.DIST_L2, 5)
    dist_orig = cv2.distanceTransform((grid == 0).astype(np.uint8), cv2.DIST_L2, 5)
    cores = (dist > core_dist_m / res).astype(np.uint8)  # open region interiors
    ncores, seeds = cv2.connectedComponents(cores)  # 0=bg, 1..ncores-1 = a core
    markers = np.zeros((H, W), np.int32)
    markers[free_clean == 0] = 1  # walls/unknown = barrier (background marker)
    markers[seeds > 0] = seeds[seeds > 0] + 1  # region cores -> labels >= 2
    cv2.watershed(
        cv2.cvtColor(free_clean * 255, cv2.COLOR_GRAY2BGR), markers
    )  # flood free space from cores
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
        cx_px, cy_px = M["m10"] / M["m00"], M["m01"] / M["m00"]
        # nav_point = deepest ORIGINALLY-free point inside this zone (open space, off walls AND props)
        np_idx = np.unravel_index(np.argmax(dist_orig * m), dist_orig.shape)
        cx_w, cy_w = round(float(ox + cx_px * res), 3), round(float(oy + cy_px * res), 3)
        zones.append(
            {
                "center": [cx_w, cy_w],
                "nav_point": [
                    round(float(ox + np_idx[1] * res), 3),
                    round(float(oy + np_idx[0] * res), 3),
                ],
                "polygon": [[round(ox + px * res, 3), round(oy + py * res, 3)] for px, py in poly],
                "area": round(area, 2),
                "label": _quadrant_label(cx_w, cy_w, ox, oy, W, H, res),
                "_label": lab,
            }
        )
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
    out[markers == -1] = (0, 0, 0)  # watershed lines
    out = cv2.flip(out, 0)  # flip so +Y is up, THEN draw labels (upright)
    for z in zones:
        cx = int((z["center"][0] - ox) / res)
        cy = H - 1 - int((z["center"][1] - oy) / res)
        cv2.circle(out, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(
            out,
            z["id"].replace("zone_", "Z") + ":" + z.get("label", ""),
            (cx - 14, cy - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            2,
        )
        nx = int((z["nav_point"][0] - ox) / res)
        ny = H - 1 - int((z["nav_point"][1] - oy) / res)
        cv2.drawMarker(out, (nx, ny), (255, 0, 255), cv2.MARKER_CROSS, 12, 2)  # nav_point
    cv2.imwrite(path, out)


def main_standalone(npz_path):
    d = np.load(npz_path)
    grid = d["grid"]
    res = float(d["res"])
    ox = float(d["origin_x"])
    oy = float(d["origin_y"])
    zones, markers = segment_grid(grid, res, ox, oy)
    out_dir = os.path.dirname(os.path.abspath(npz_path))
    clean = [{k: v for k, v in z.items() if k != "_label"} for z in zones]
    with open(os.path.join(out_dir, "zones.yaml"), "w") as f:
        json.dump({"zones": clean}, f, indent=2)
    _viz(grid, markers, zones, res, ox, oy, os.path.join(out_dir, "zones_viz.png"))
    print(f"{len(zones)} zones -> {out_dir}/zones.yaml + zones_viz.png")
    for z in zones:
        print(
            f"  {z['id']} [{z['label']}]: center={z['center']} nav_point={z['nav_point']} "
            f"area={z['area']}m2 ({len(z['polygon'])}-pt polygon)"
        )


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
            self.declare_parameter("min_area_m2", 6.0)
            self.declare_parameter("island_max_m2", 2.5)
            self.declare_parameter("out_path", os.path.expanduser("~/zones.yaml"))
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.sub = self.create_subscription(OccupancyGrid, "/map", self.cb, qos)
            self.pub = self.create_publisher(MarkerArray, "/zones", 1)
            self.get_logger().info("zone_segmenter: waiting for /map ...")

        def cb(self, m):
            grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
            zones, _ = segment_grid(
                grid,
                m.info.resolution,
                m.info.origin.position.x,
                m.info.origin.position.y,
                core_dist_m=self.get_parameter("core_dist_m").value,
                min_area_m2=self.get_parameter("min_area_m2").value,
                island_max_m2=self.get_parameter("island_max_m2").value,
            )
            clean = [{k: v for k, v in z.items() if k != "_label"} for z in zones]
            with open(self.get_parameter("out_path").value, "w") as f:
                json.dump({"zones": clean}, f, indent=2)
            self.publish_markers(zones)
            self.get_logger().info(
                f"segmented {len(zones)} zones -> {self.get_parameter('out_path').value}"
            )

        def publish_markers(self, zones):
            arr = MarkerArray()
            dele = Marker()
            dele.action = Marker.DELETEALL
            arr.markers.append(dele)
            for i, z in enumerate(zones):
                ls = Marker()
                ls.header.frame_id = "map"
                ls.ns = "zone_poly"
                ls.id = i
                ls.type = Marker.LINE_STRIP
                ls.action = Marker.ADD
                ls.scale.x = 0.06
                ls.color.g = 1.0
                ls.color.a = 1.0
                ls.pose.orientation.w = 1.0
                for px, py in z["polygon"] + [z["polygon"][0]]:
                    ls.points.append(Point(x=float(px), y=float(py), z=0.05))
                arr.markers.append(ls)
                tx = Marker()
                tx.header.frame_id = "map"
                tx.ns = "zone_label"
                tx.id = i
                tx.type = Marker.TEXT_VIEW_FACING
                tx.action = Marker.ADD
                tx.scale.z = 0.6
                tx.color.r = 1.0
                tx.color.g = 1.0
                tx.color.b = 1.0
                tx.color.a = 1.0
                tx.pose.position.x = float(z["center"][0])
                tx.pose.position.y = float(z["center"][1])
                tx.pose.position.z = 0.3
                tx.pose.orientation.w = 1.0
                tx.text = f"{z['id']} ({z.get('label', '')})"
                arr.markers.append(tx)
            self.pub.publish(arr)

    rclpy.init(args=args)
    n = ZoneSegmenter()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1].endswith(".npz"):
        main_standalone(sys.argv[1])
    else:
        main()
