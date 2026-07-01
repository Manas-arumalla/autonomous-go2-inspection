#!/usr/bin/env python3
"""inspection_mission -- Phase 5c (GENERAL): autonomous gauge-inspection mission, driven ENTIRELY by the
auto-segmented map. NO hard-coded poses or strafe widths -> works in any world / on the real robot.

From HOME, for each candidate zone (from zones.yaml):
  Nav2 -> zone centre  ->  zone_sweeper(find_wall): rotate + camera-detect the gauge wall -> approach ->
  square-up (/scan) -> strafe with extent DERIVED from the zone polygon -> back-off  ->  panorama_segmenter
  (FastSAM crops)  ->  (optional) gauge_inspector (Anthropic API, if ANTHROPIC_API_KEY)  ->  return HOME  ->  one
  facility report. Everything per-room is DISCOVERED, not configured.

  ros2 launch go2_bringup mission.launch.py                 # all candidate zones
  ros2 run go2_inspection inspection_mission --ros-args -p zones:=zone_0   # a subset (testing)
"""
import os, json, math, subprocess, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

# env-driven for the container (persisted /data volume; the LLM reader runs in whatever python has the
# anthropic SDK -- on the Go2 that's the system python, so VENV_PY defaults to it).
GAUGES_ROOT = os.path.expanduser(os.environ.get("GAUGES_ROOT", "~/gauges"))
VENV_PY = os.environ.get("GAUGE_PYTHON", os.path.expanduser("~/gauge_venv/bin/python"))
INSPECTOR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gauge_inspector.py")
HOME = (0.0, 0.0, 0.0)


class Mission(Node):
    def __init__(self):
        super().__init__("inspection_mission")
        # use_sim_time comes from the -p flag (FALSE on the real Go2 -- no /clock); forward it to the
        # sweeper/segmenter children. (The SIM passes use_sim_time:=true.)
        self.ust = "true" if bool(self.get_parameter("use_sim_time").value) else "false"
        self.zones_file = os.path.expanduser(self.declare_parameter(
            "zones_file", os.environ.get("GO2_ZONES", "/maps/facility_inspection_zones.yaml")).value)
        self.min_area = float(self.declare_parameter("min_area", 15.0).value)   # skip tiny/noise zones
        self.only = [z for z in self.declare_parameter("zones", "").value.split(",") if z]
        # Additive flags (defaults preserve the original full-mission behaviour); the service layer
        # (mission_control_server) sets these to compose navigate-only / single-zone / dock actions.
        self.inspect = bool(self.declare_parameter("inspect", True).value)         # False = navigate only
        self.return_home = bool(self.declare_parameter("return_home", True).value)
        self.goto_home = bool(self.declare_parameter("goto_home", False).value)    # True = just dock at HOME
        self.read = bool(self.declare_parameter("read", True).value)               # AND gated on ANTHROPIC_API_KEY
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def goto(self, x, y, yaw=0.0, timeout=200.0, label=""):
        if not self.nav.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("no Nav2 server"); return False
        g = NavigateToPose.Goal(); g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(x); g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.z = math.sin(yaw / 2); g.pose.pose.orientation.w = math.cos(yaw / 2)
        self.get_logger().info(f"NAV -> {label} ({x:.1f},{y:.1f})")
        fut = self.nav.send_goal_async(g); rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn(f"{label} goal REJECTED"); return False
        rf = gh.get_result_async(); t0 = time.time()
        while rclpy.ok() and not rf.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
        if not rf.done():
            gh.cancel_goal_async(); time.sleep(1.0); self.get_logger().warn(f"{label} TIMEOUT"); return False
        ok = rf.result().status == 4
        self.get_logger().info(f"{label} -> {'ARRIVED' if ok else 'status ' + str(rf.result().status)}")
        return ok


def _run(cmd, timeout, label):
    print(f"      $ {label}", flush=True)
    try:
        r = subprocess.run(cmd, timeout=timeout, capture_output=True, text=True)
        if r.returncode != 0:
            print(f"      {label} rc={r.returncode}: {(r.stderr or '').strip()[-200:]}", flush=True)
        return r.returncode == 0
    except subprocess.TimeoutExpired:
        print(f"      {label} TIMEOUT", flush=True)
        return False


def inspect_zone(zone_id, zones_file, do_read=True, ust="false"):
    """MAP-DRIVEN wall follow (one panorama per wall, always facing it) -> YOLOE segment each panorama
    -> (optional) LLM read. Set INSPECT_LEGACY=1 to fall back to the perception-driven single-wall
    zone_sweeper + FastSAM. Both produce ~/gauges/<zone>/gauges.json in the same schema. ust = use_sim_time
    for the child nodes (false on the real Go2)."""
    zone_dir = os.path.join(GAUGES_ROOT, zone_id)
    if os.environ.get("INSPECT_LEGACY") == "1":
        sweeper = ["ros2", "run", "go2_inspection", "zone_sweeper", "--ros-args",
                   "-p", f"use_sim_time:={ust}", "-p", "skip_nav:=true", "-p", "find_wall:=true",
                   "-p", f"zone_id:={zone_id}", "-p", f"zones_file:={zones_file}", "-p", "half_width:=0.0"]
        segmenter = ["ros2", "run", "go2_inspection", "panorama_segmenter", zone_dir]
    else:
        # zone_wall_follower now runs YOLOE LIVE during the scan and writes crops + detections.json +
        # gauges.json itself; no separate post-pass (yoloe_segmenter would find no panoramas and clobber
        # gauges.json with an empty result).
        sweeper = ["ros2", "run", "go2_inspection", "zone_wall_follower", "--ros-args",
                   "-p", f"use_sim_time:={ust}",
                   "-p", f"zone_id:={zone_id}", "-p", f"zones_file:={zones_file}"]
        segmenter = None
    if not _run(sweeper, 600, f"wall-follow {zone_id}"):
        return None
    if segmenter:
        _run(segmenter, 240, f"segment {zone_id}")
    if do_read and os.environ.get("ANTHROPIC_API_KEY") and os.path.exists(VENV_PY):
        _run([VENV_PY, INSPECTOR, zone_dir], 240, f"read {zone_id}")
    gj = os.path.join(zone_dir, "gauges.json")
    return json.load(open(gj)) if os.path.exists(gj) else None


def main(args=None):
    rclpy.init(args=args)
    m = Mission()
    # goto_home mode: just dock at HOME and exit (service: /navigate_home).
    if m.goto_home:
        ok = m.goto(*HOME, label="HOME (dock)")
        m.get_logger().info(f"=== HOME dock -> {'ARRIVED' if ok else 'FAILED'} ===")
        m.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return
    zones = json.load(open(m.zones_file))["zones"]
    cands = [z for z in zones if (z["id"] in m.only) or (not m.only and z.get("area", 0) >= m.min_area)]
    mode = "inspect" if m.inspect else "navigate-only"
    m.get_logger().info(f"=== MISSION START at HOME -- {len(cands)} candidate zones, {mode} "
                        f"({'subset ' + ','.join(m.only) if m.only else 'area>=' + str(m.min_area)}) ===")
    results = []
    for z in cands:
        zid = z["id"]; cx, cy = z["center"]
        if not m.goto(cx, cy, label=f"zone {zid}"):
            results.append({"zone": zid, "nav": False, "n_gauges": 0}); continue
        if not m.inspect:
            results.append({"zone": zid, "nav": True, "n_gauges": 0}); continue
        gj = inspect_zone(zid, m.zones_file, m.read, ust=m.ust)
        n = gj.get("n_gauges", 0) if gj else 0
        m.get_logger().info(f"   {zid}: {n} gauges")
        results.append({"zone": zid, "nav": True, "n_gauges": n,
                        "gauges": gj.get("gauges", []) if gj else []})
    if m.return_home:
        m.goto(*HOME, label="HOME (return)")
    os.makedirs(GAUGES_ROOT, exist_ok=True)
    out = os.path.join(GAUGES_ROOT, "facility_inspection_manifest.json")
    total = sum(r.get("n_gauges", 0) for r in results)
    rooms_ok = sum(1 for r in results if r.get("n_gauges"))
    json.dump({"rooms": results, "total_gauges": total, "rooms_with_gauges": rooms_ok},
              open(out, "w"), indent=2)
    m.get_logger().info(f"=== MISSION COMPLETE -> {out} : {total} gauges across {rooms_ok} rooms ===")
    m.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
