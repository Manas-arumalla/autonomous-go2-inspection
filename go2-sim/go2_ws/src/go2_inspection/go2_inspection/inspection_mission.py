#!/usr/bin/env python3
"""inspection_mission -- autonomous object-inspection mission driven entirely by the auto-segmented map.

There are no hard-coded poses, so the mission works in any world.

From HOME, for each candidate zone (from zones.yaml):
  Nav2 -> zone nav_point (deepest free point; falls back to centre) -> zone_inspector (viewpoint sampling
  + 360-degree spin with live YOLOE detection; projects each object to the map via depth, de-duplicates,
  crops, writes a per-zone report + zone_map) -> return HOME -> one facility report + facility map.
Detection and report only (no gauge reading). Everything per-room is discovered, not configured.

  # recommended: bring up inspection_nav.launch.py + mission_control.launch.py, then call /run_mission
  #   (or /inspect_zone); this node is what mission_control drives per mission.
  ros2 run go2_inspection inspection_mission --ros-args -p zones:=zone_0   # direct: a subset of zones
"""

import os, json, math, subprocess, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, ComputePathToPose

from go2_inspection import report_utils
from go2_inspection.mission_fsm import MissionFSM, MissionState

GAUGES_ROOT = os.path.expanduser("~/gauges")
MANIFEST = os.path.join(GAUGES_ROOT, "facility_inspection_manifest.json")
EVENTS = os.path.join(GAUGES_ROOT, "mission_events.jsonl")  # structured mission event stream (ADR-016 M7b)
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
        self.read_approach = bool(
            self.declare_parameter("read_approach", False).value
        )  # True = drive close to each detected gauge for a high-res read crop (ADR-017)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.reach = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

    def reachable(self, x, y):
        """Best-effort reachability pre-check via Nav2's global planner (ComputePathToPose) from the robot's
        current pose. Returns False only when the planner is up AND returns no path -> skip a zone fast
        instead of burning the full nav timeout toward it. Planner unavailable/inconclusive -> True (let
        nav try, preserving old behaviour)."""
        if not self.reach.wait_for_server(timeout_sec=3.0):
            return True
        g = ComputePathToPose.Goal()
        g.goal.header.frame_id = "map"
        g.goal.header.stamp = rclpy.time.Time().to_msg()
        g.goal.pose.position.x = float(x)
        g.goal.pose.position.y = float(y)
        g.goal.pose.orientation.w = 1.0
        g.use_start = False
        fut = self.reach.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=8.0)
        try:
            r = rf.result()
            return r.status == 4 and len(r.result.path.poses) >= 2
        except Exception:
            return True

    def goto(self, x, y, yaw=0.0, timeout=200.0, label=""):
        if not self.reachable(x, y):
            self.get_logger().warn(f"{label} UNREACHABLE (Nav2 planner found no path); skip")
            return False
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


def _ev(fsm, fn, *a, **k):
    """Best-effort FSM call: the structured event stream is observability only and must NEVER raise into
    (and break) the mission's control flow. Any FSM/IO error is swallowed."""
    if fsm is None:
        return
    try:
        getattr(fsm, fn)(*a, **k)
    except Exception:
        pass


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


def inspect_zone(zone_id, zones_file, map_yaml, read_approach=False):
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
        "-p",
        f"read_approach:={'true' if read_approach else 'false'}",  # detect-then-approach close reading (ADR-017)
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
    try:
        fsm = MissionFSM(EVENTS)  # structured event stream for this mission (best-effort; see _ev)
    except Exception:
        fsm = None
    if m.goto_home:
        _ev(fsm, "to", MissionState.PLANNING, kind="DOCK", data={"target": list(HOME)})
        ok = m.goto(*HOME, label="HOME (dock)")
        m.get_logger().info(f"=== HOME dock -> {'ARRIVED' if ok else 'FAILED'} ===")
        _ev(fsm, "to", MissionState.DONE, data={"docked": ok})
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
    _ev(fsm, "to", MissionState.PLANNING, data={"zones": [z["id"] for z in cands], "mode": mode})
    results = []
    for z in cands:
        zid = z["id"]
        gx, gy = z.get("nav_point") or z["center"]  # prefer the free-space nav point
        _ev(fsm, "to", MissionState.NAVIGATING, zone=zid, data={"target": [round(gx, 2), round(gy, 2)]})
        if not m.goto(gx, gy, label=f"zone {zid}"):
            _ev(fsm, "emit", "NAV_FAILED", zone=zid)
            results.append({"zone": zid, "nav": False, "n_objects": 0})
            continue
        _ev(fsm, "emit", "ARRIVED", zone=zid)
        if not m.inspect:
            results.append({"zone": zid, "nav": True, "n_objects": 0})
            continue
        _ev(fsm, "to", MissionState.INSPECTING, zone=zid)
        oj = inspect_zone(zid, m.zones_file, m.map_yaml, m.read_approach)
        n = oj.get("n_objects", 0) if oj else 0
        _ev(fsm, "emit", "INSPECT_DONE", zone=zid, data={"n_objects": n})
        # gauge-reading layer: if read:=true and a key+venv are present, Claude reads each detected gauge
        # crop into the zone report (objects.json gains a 'gauge_reading' per gauge). Off the spin loop.
        if m.read and oj and os.environ.get("ANTHROPIC_API_KEY") and os.path.exists(VENV_PY):
            _ev(fsm, "to", MissionState.READING, zone=zid)
            _run([VENV_PY, INSPECTOR, os.path.join(GAUGES_ROOT, zid)], 300, f"read gauges {zid}")
            _ev(fsm, "emit", "READ_DONE", zone=zid)
        m.get_logger().info(f"   {zid}: {n} objects")
        results.append({"zone": zid, "nav": True, "n_objects": n})
    if m.return_home:
        _ev(fsm, "emit", "RETURN_HOME")
        m.goto(*HOME, label="HOME (return)")
    os.makedirs(GAUGES_ROOT, exist_ok=True)
    _ev(fsm, "to", MissionState.ROLLUP)
    if m.inspect:
        total, rooms_ok = _facility_rollup(m.zones_file, m.map_yaml, results)
        m.get_logger().info(
            f"=== MISSION COMPLETE -> {MANIFEST} : {total} objects across {rooms_ok} zones ==="
        )
        _ev(fsm, "to", MissionState.DONE, data={"total_objects": total, "zones_with_objects": rooms_ok})
    else:
        _ev(fsm, "to", MissionState.DONE, data={"navigated": sum(1 for r in results if r.get("nav"))})
    m.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
