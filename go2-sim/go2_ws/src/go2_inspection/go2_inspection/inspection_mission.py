#!/usr/bin/env python3
"""inspection_mission -- autonomous object-inspection mission driven entirely by the auto-segmented map.

There are no hard-coded poses, so the mission works in any world.

From HOME, for each candidate zone (from zones.yaml):
  Nav2 -> zone nav_point (deepest free point; falls back to centre) -> zone_inspector (viewpoint sampling
  + 360-degree spin with live YOLOE detection; projects each object to the map via depth, de-duplicates,
  crops, writes a per-zone report + zone_map) -> return HOME -> one facility report + facility map.
Detection and report only (no gauge reading). Everything per-room is discovered, not configured.

  ros2 launch go2_bringup mission.launch.py                 # all candidate zones
  ros2 run go2_inspection inspection_mission --ros-args -p zones:=zone_0   # run a subset of zones
"""

import os, json, math, subprocess, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

from go2_inspection import report_utils

GAUGES_ROOT = os.path.expanduser("~/gauges")
MANIFEST = os.path.join(GAUGES_ROOT, "facility_inspection_manifest.json")
HOME = (0.0, 0.0, 0.0)
# Optional Claude gauge-value reading layer (ADR-016 M4): when read:=true, after zone_inspector detects +
# crops gauges, run gauge_inspector (in the venv that has `anthropic`) to read each gauge's value into the
# per-zone report. Off the locomotion loop; degrades gracefully if the key/venv is absent.
VENV_PY = os.environ.get("GAUGE_PYTHON", os.path.expanduser("~/gauge_venv/bin/python"))
INSPECTOR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gauge_inspector.py")


class Mission(Node):
    def __init__(self):
        super().__init__("inspection_mission")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.zones_file = os.path.expanduser(
            self.declare_parameter(
                "zones_file", "~/.go2_maps/facility_inspection_zones.yaml"
            ).value
        )
        self.map_yaml = self.declare_parameter("map_yaml", report_utils.DEFAULT_MAP_YAML).value
        self.min_area = float(
            self.declare_parameter("min_area", 15.0).value
        )  # skip tiny/noise zones
        self.only = [z for z in self.declare_parameter("zones", "").value.split(",") if z]
        # Additive flags (defaults preserve the full-mission behaviour); the service layer composes
        # navigate-only / single-zone / dock actions.
        self.inspect = bool(self.declare_parameter("inspect", True).value)  # False = navigate only
        self.return_home = bool(self.declare_parameter("return_home", True).value)
        self.goto_home = bool(
            self.declare_parameter("goto_home", False).value
        )  # True = just dock at HOME
        self.read = bool(
            self.declare_parameter("read", False).value
        )  # True = also Claude-read detected gauges per zone (ADR-016 M4)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def goto(self, x, y, yaw=0.0, timeout=200.0, label=""):
        if not self.nav.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("no Nav2 server")
            return False
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        # stamp 0 = use latest available TF (rtabmap map->odom lags sim-now, so a now()-stamped goal can't
        # be transformed into the costmap frame by the controller).
        g.pose.header.stamp = rclpy.time.Time().to_msg()
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.z = math.sin(yaw / 2)
        g.pose.pose.orientation.w = math.cos(yaw / 2)
        self.get_logger().info(f"NAV -> {label} ({x:.1f},{y:.1f})")
        fut = self.nav.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn(f"{label} goal REJECTED")
            return False
        rf = gh.get_result_async()
        t0 = time.time()
        while rclpy.ok() and not rf.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
        if not rf.done():
            gh.cancel_goal_async()
            time.sleep(1.0)
            self.get_logger().warn(f"{label} TIMEOUT")
            return False
        ok = rf.result().status == 4
        self.get_logger().info(
            f"{label} -> {'ARRIVED' if ok else 'status ' + str(rf.result().status)}"
        )
        return ok


def _run(cmd, timeout, label):
    print(f"      $ {label}", flush=True)
    try:
        r = subprocess.run(cmd, timeout=timeout, capture_output=True, text=True)
        if r.returncode != 0:
            print(
                f"      {label} rc={r.returncode}: {(r.stderr or '').strip()[-200:]}", flush=True
            )
        return r.returncode == 0
    except subprocess.TimeoutExpired:
        print(f"      {label} TIMEOUT", flush=True)
        return False


def inspect_zone(zone_id, zones_file, map_yaml):
    """zone_inspector runs YOLOE LIVE during a viewpoint+spin scan and writes crops + detections.json +
    objects.json + zone_map.png + report.{md,csv} itself. Returns the objects.json dict (or None)."""
    zone_dir = os.path.join(GAUGES_ROOT, zone_id)
    cmd = [
        "ros2",
        "run",
        "go2_inspection",
        "zone_inspector",
        "--ros-args",
        "-p",
        "use_sim_time:=true",
        "-p",
        f"zone_id:={zone_id}",
        "-p",
        f"zones_file:={zones_file}",
        "-p",
        f"map_yaml:={map_yaml}",
    ]
    if not _run(cmd, 900, f"inspect {zone_id}"):
        return None
    oj = os.path.join(zone_dir, "objects.json")
    return json.load(open(oj)) if os.path.exists(oj) else None


def _facility_rollup(zones_file, map_yaml, results):
    """Aggregate per-zone objects.json into a facility manifest + facility map + facility report."""
    zones = {z["id"]: z for z in json.load(open(zones_file))["zones"]}
    zones_objects = {}
    zone_polys = {}
    for r in results:
        zid = r["zone"]
        oj = os.path.join(GAUGES_ROOT, zid, "objects.json")
        if os.path.exists(oj):
            objs = json.load(open(oj)).get("objects", [])
            zones_objects[zid] = objs
            zone_polys[zid] = zones.get(zid, {}).get("polygon", [])
    total = sum(len(v) for v in zones_objects.values())
    rooms_ok = sum(1 for v in zones_objects.values() if v)
    json.dump(
        {
            "rooms": [
                {
                    "zone": r["zone"],
                    "label": zones.get(r["zone"], {}).get("label", ""),
                    "nav": r["nav"],
                    "n_objects": len(zones_objects.get(r["zone"], [])),
                }
                for r in results
            ],
            "total_objects": total,
            "zones_with_objects": rooms_ok,
        },
        open(MANIFEST, "w"),
        indent=2,
    )
    try:
        if zones_objects:
            report_utils.plot_facility_map(
                zones_objects, os.path.join(GAUGES_ROOT, "facility_map.png"), zone_polys, map_yaml
            )
        with open(os.path.join(GAUGES_ROOT, "facility_report.md"), "w") as f:
            f.write(
                f"# Facility inspection report\n\n**{total} objects across {rooms_ok} zone(s).**\n\n"
            )
            for zid, objs in zones_objects.items():
                lbl = zones.get(zid, {}).get("label", "")
                f.write(
                    f"- **{zid}** ({lbl}): {len(objs)} object(s) -- "
                    f"{', '.join(sorted({o['class'] for o in objs})) or 'none'}\n"
                )
    except Exception as e:
        print(f"      facility rollup plot failed: {e}", flush=True)
    return total, rooms_ok


def main(args=None):
    rclpy.init(args=args)
    m = Mission()
    if m.goto_home:
        ok = m.goto(*HOME, label="HOME (dock)")
        m.get_logger().info(f"=== HOME dock -> {'ARRIVED' if ok else 'FAILED'} ===")
        m.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return
    zones = json.load(open(m.zones_file))["zones"]
    cands = [
        z for z in zones if (z["id"] in m.only) or (not m.only and z.get("area", 0) >= m.min_area)
    ]
    mode = "inspect" if m.inspect else "navigate-only"
    m.get_logger().info(
        f"=== MISSION START at HOME -- {len(cands)} candidate zones, {mode} "
        f"({'subset ' + ','.join(m.only) if m.only else 'area>=' + str(m.min_area)}) ==="
    )
    results = []
    for z in cands:
        zid = z["id"]
        gx, gy = z.get("nav_point") or z["center"]  # prefer the free-space nav point
        if not m.goto(gx, gy, label=f"zone {zid}"):
            results.append({"zone": zid, "nav": False, "n_objects": 0})
            continue
        if not m.inspect:
            results.append({"zone": zid, "nav": True, "n_objects": 0})
            continue
        oj = inspect_zone(zid, m.zones_file, m.map_yaml)
        n = oj.get("n_objects", 0) if oj else 0
        # gauge-reading layer: if read:=true and a key+venv are present, Claude reads each detected gauge
        # crop into the zone report (objects.json gains a 'gauge_reading' per gauge). Off the spin loop.
        if m.read and oj and os.environ.get("ANTHROPIC_API_KEY") and os.path.exists(VENV_PY):
            _run([VENV_PY, INSPECTOR, os.path.join(GAUGES_ROOT, zid)], 300, f"read gauges {zid}")
        m.get_logger().info(f"   {zid}: {n} objects")
        results.append({"zone": zid, "nav": True, "n_objects": n})
    if m.return_home:
        m.goto(*HOME, label="HOME (return)")
    os.makedirs(GAUGES_ROOT, exist_ok=True)
    if m.inspect:
        total, rooms_ok = _facility_rollup(m.zones_file, m.map_yaml, results)
        m.get_logger().info(
            f"=== MISSION COMPLETE -> {MANIFEST} : {total} objects across {rooms_ok} zones ==="
        )
    m.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
