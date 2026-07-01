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
# Optional API-based gauge-value reading layer (ADR-016 M4): when read:=true, after zone_inspector detects +
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
        )  # True = also model-read detected gauges per zone (ADR-016 M4)
        self.read_approach = bool(
            self.declare_parameter("read_approach", False).value
        )  # True = drive close to each detected gauge for a high-res read crop (ADR-017)
        # How long to wait, at mission start, for localization (map->base_link) before giving up. The
        # RTAB-Map DB load + relocalization can take a while on a loaded machine; without this gate the
        # mission would mistake "not localized yet" for "every zone unreachable" and skip everything.
        self.localize_timeout = float(self.declare_parameter("localize_timeout", 90.0).value)
        # Per-zone navigation timeout (wall-clock). The reachable zones all arrive well within this; a
        # TRUE doorway wedge (DWB can't maneuver in from a bad approach angle) never resolves no matter
        # how long we wait, so the timeout's job is just to fail reasonably fast and hand off to the
        # re-stage-via-HOME retry (which IS the wedge fix). 180 s balances "enough for a slow-but-
        # progressing traverse under 0.75x sim RTF" against "don't burn 5 min on a wedge".
        self.nav_timeout = float(self.declare_parameter("nav_timeout", 180.0).value)
        # Fast wedge detection: a doorway wedge never resolves, so don't burn the full nav_timeout waiting
        # for it. If the robot's BEST distance-to-goal stops improving by wedge_progress for wedge_timeout
        # seconds while still wedge_min_dist away, cancel early and hand off to the retry-via-HOME path.
        # Tracking the BEST (monotone-min) distance makes this robust to legitimate detours (going around
        # an obstacle temporarily increases raw distance but the min keeps falling).
        self.wedge_timeout = float(self.declare_parameter("wedge_timeout", 30.0).value)
        self.wedge_progress = float(self.declare_parameter("wedge_progress", 0.25).value)
        self.wedge_min_dist = float(self.declare_parameter("wedge_min_dist", 0.7).value)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.reach = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self._tf_buf = None  # lazily created TransformListener buffer (localization gate)

    def wait_for_localization(self, timeout=None):
        """Block until the robot is localized in the map frame (map->base_link TF resolvable) so the global
        planner can actually plan. Returns True once localized, False on timeout. CRITICAL: this lets the
        mission distinguish 'localization not ready' (abort with a clear message) from 'zone unreachable'
        (skip that zone) -- conflating them silently skipped the entire mission (the reachability check
        aborts identically in both cases). map->base_link = map->odom (RTAB-Map localization) chained with
        odom->base_link (the always-present sim odometry), so it is the single ground-truth that the robot
        is placed in the map."""
        from tf2_ros import Buffer, TransformListener

        if timeout is None:
            timeout = self.localize_timeout
        if self._tf_buf is None:
            self._tf_buf = Buffer()
            TransformListener(self._tf_buf, self)
        t0 = time.time()
        last_log = 0.0
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                self._tf_buf.lookup_transform("map", "base_link", rclpy.time.Time())
                self.get_logger().info(
                    f"localization READY (map->base_link present after {time.time() - t0:.0f}s)"
                )
                # let the global costmap finish populating from the map before the first plan request
                t1 = time.time()
                while rclpy.ok() and time.time() - t1 < 3.0:
                    rclpy.spin_once(self, timeout_sec=0.2)
                return True
            except Exception:
                pass
            if time.time() - last_log > 10.0:
                self.get_logger().info(
                    f"waiting for localization (map->base_link) ... {int(time.time() - t0)}/{int(timeout)}s"
                )
                last_log = time.time()
        return False

    def is_localized(self):
        """Quick check: is map->base_link currently resolvable? Used to tell 'localization dropped' apart
        from 'zone truly unreachable' when a plan fails mid-mission."""
        from tf2_ros import Buffer, TransformListener

        if self._tf_buf is None:
            self._tf_buf = Buffer()
            TransformListener(self._tf_buf, self)
            t0 = time.time()
            while rclpy.ok() and time.time() - t0 < 1.0:
                rclpy.spin_once(self, timeout_sec=0.1)
        try:
            self._tf_buf.lookup_transform("map", "base_link", rclpy.time.Time())
            return True
        except Exception:
            return False

    def _robot_xy(self):
        """Current robot (x,y) in the map frame, or None. Used for wedge detection during a nav goal."""
        from tf2_ros import Buffer, TransformListener

        if self._tf_buf is None:
            self._tf_buf = Buffer()
            TransformListener(self._tf_buf, self)
        try:
            t = self._tf_buf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
            return (t.translation.x, t.translation.y)
        except Exception:
            return None

    def reachable(self, x, y):
        """Best-effort reachability pre-check via Nav2's global planner (ComputePathToPose) from the robot's
        current pose. Returns False only when the planner is up AND returns no path -> skip a zone fast
        instead of burning the full nav timeout toward it. Planner unavailable/inconclusive -> True (let
        nav try, preserving old behaviour). Retries once on a 'no path' result to ride out a transient
        global-costmap update (a zone is only declared unreachable if it fails twice). The mission's
        wait_for_localization() gate already guarantees we are localized before this runs, so a 'no path'
        here means a genuine planning failure, not a missing map->base_link."""
        if not self.reach.wait_for_server(timeout_sec=3.0):
            return True
        for attempt in range(2):
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
            if gh is not None and gh.accepted:
                rf = gh.get_result_async()
                rclpy.spin_until_future_complete(self, rf, timeout_sec=8.0)
                try:
                    r = rf.result()
                    if r.status == 4 and len(r.result.path.poses) >= 2:
                        return True
                except Exception:
                    return True  # inconclusive -> let nav try
            if attempt == 0:
                # transient: brief settle, then retry once before declaring unreachable
                t1 = time.time()
                while rclpy.ok() and time.time() - t1 < 2.0:
                    rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def goto(self, x, y, yaw=0.0, timeout=None, label=""):
        if timeout is None:
            timeout = self.nav_timeout
        if not self.reachable(x, y):
            # A 'no path' means either the zone is truly unreachable OR localization dropped mid-mission
            # (RTAB-Map can lose the pose when CPU starvation makes odom->base_link TF lag behind the
            # scan timestamps). Only the latter is recoverable -- distinguish them so a transient
            # localization blip doesn't get every remaining zone wrongly skipped as 'unreachable'.
            if self.is_localized():
                self.get_logger().warn(f"{label} UNREACHABLE (localized, but Nav2 found no path); skip")
                return False
            self.get_logger().warn(f"{label}: localization lost mid-mission -- waiting to recover ...")
            if not (
                self.wait_for_localization(timeout=min(self.localize_timeout, 60.0))
                and self.reachable(x, y)
            ):
                self.get_logger().warn(f"{label} UNREACHABLE (localization did not recover); skip")
                return False
            self.get_logger().info(f"{label}: localization recovered -- proceeding")
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
        best_d = float("inf")
        best_t = t0
        while rclpy.ok() and not rf.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            # Fast wedge bail-out: track the BEST distance-to-goal; if it stops improving by wedge_progress
            # for wedge_timeout while still wedge_min_dist away, the robot is wedged (a doorway DWB can't
            # work through) -> cancel now so the caller's retry-via-HOME fires in ~30 s, not ~180 s.
            p = self._robot_xy()
            if p is not None:
                d = math.hypot(p[0] - x, p[1] - y)
                if d < best_d - self.wedge_progress:
                    best_d, best_t = d, time.time()
                elif (
                    time.time() - best_t > self.wedge_timeout and d > self.wedge_min_dist
                ):
                    gh.cancel_goal_async()
                    time.sleep(1.0)
                    self.get_logger().warn(
                        f"{label} WEDGED (no progress for {self.wedge_timeout:.0f}s, {d:.1f}m from goal)"
                    )
                    return False
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
    # Localization gate: nothing can navigate until RTAB-Map has placed the robot in the map frame.
    # Without this, a slow/failed localization makes every ComputePathToPose abort, which the mission
    # would otherwise read as "every zone unreachable" and silently complete with 0 objects.
    if not m.wait_for_localization():
        m.get_logger().error(
            "ABORT: robot never localized (no map->base_link TF within "
            f"{m.localize_timeout:.0f}s). The zones are NOT unreachable -- RTAB-Map localization is not "
            "up yet (check the rtabmap node / give it more time / set localize_timeout higher). "
            "Not running the mission."
        )
        _ev(fsm, "to", MissionState.DONE, data={"aborted": "no_localization"})
        m.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return
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
            # A direct room-to-room traverse can wedge the controller at a doorway it can't maneuver
            # through from that approach angle (DWB is forward-only). Every room IS reliably reachable
            # from the hub, so re-stage at HOME (the hub) and retry once -- a sensible "return to the
            # corridor, then enter the room" patrol pattern.
            m.get_logger().info(f"zone {zid}: direct nav failed -- re-staging via HOME (hub) and retrying")
            _ev(fsm, "emit", "NAV_RETRY_VIA_HOME", zone=zid)
            m.goto(*HOME, label="HOME (re-stage)")
            if not m.goto(gx, gy, label=f"zone {zid} (retry)"):
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
        # gauge-reading layer: if read:=true and a key+venv are present, the model reads each detected gauge
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
