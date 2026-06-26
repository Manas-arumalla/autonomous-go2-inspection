#!/usr/bin/env python3
"""mission_control_server -- a thin ROS2 SERVICE layer over the Go2 inspection stack.

Turns the multi-terminal manual stack (frontier_explorer, inspection_mission, zone_sweeper, map
saving, ...) into single-call service TRIGGERS. Each service maps 1:1 to a future WendyOS/MCP tool --
the MCP server (later, a separate process) just opens rclpy clients to these. NOTHING here is
MCP/Wendy-specific; this is pure ROS2.

DESIGN -- subprocess orchestrator (the proven pattern already used inside inspection_mission):
heavy capabilities run the EXISTING, validated nodes as child processes with their own clean rclpy
context, so this node never fights rclpy single-context/threading issues and never modifies the
working nodes. The server only holds: the frontier child handle, a robot-busy lock (one motion at a
time), a cached /map (status/coverage), and the last result. A MultiThreadedExecutor +
ReentrantCallbackGroup keep /get_status and /stop_exploration responsive while a long inspect runs.

Every service uses ONE uniform srv (go2_inspection_interfaces/ZoneTask): {zone_id, read} ->
{success, message, result_json}. zone_id/read are ignored where N/A.

SERVICE CATALOG (all still runnable standalone as today):
  MAPPING      /start_exploration  /stop_exploration  /save_map
  NAVIGATION   /navigate_to_zone   /navigate_home
  INSPECTION   /inspect_zone       /run_mission
  DATA         /list_zones  /get_zone_image  /get_zone_gauges  /get_report  /get_status

The server assumes the BASE stack is already up (sim/robot + SLAM-or-localization + Nav2):
  - mapping mode    (start/stop_exploration, save_map):  rtabmap SLAM + Nav2 stack.
  - inspection mode (navigate/inspect/run_mission):      localization + map_server + Nav2 stack.
Run it as one extra node beside whatever base stack is up:
  ros2 launch go2_bringup mission_control.launch.py
"""
import os, glob, json, signal, subprocess, threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from go2_inspection_interfaces.srv import ZoneTask

GAUGES_ROOT = os.path.expanduser("~/gauges")
MANIFEST = os.path.join(GAUGES_ROOT, "facility_inspection_manifest.json")
DEFAULT_WS = "/home/manas-reddy/Downloads/EE26 Hackathon/go2-sim"
DEFAULT_ZONES = "~/.go2_maps/facility_inspection_zones.yaml"


def _env():
    """Child env with the standing DDS fix so subprocesses can reach the running stack."""
    e = os.environ.copy()
    e["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    return e


class MissionControl(Node):
    def __init__(self):
        super().__init__("mission_control")
        self.zones_file = os.path.expanduser(self.declare_parameter("zones_file", DEFAULT_ZONES).value)
        self.ws = self.declare_parameter("workspace", DEFAULT_WS).value
        self.map_name = self.declare_parameter("map_name", "facility_inspection").value
        # state
        self._frontier = None            # Popen handle (continuous task)
        self._busy = threading.Lock()    # one robot motion at a time
        self._action = "idle"
        self._last = {}
        self._grid = None                # cached /map for status/coverage
        cb = ReentrantCallbackGroup()
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, qos, callback_group=cb)
        svc = [
            ("start_exploration", self.start_exploration), ("stop_exploration", self.stop_exploration),
            ("save_map", self.save_map),
            ("navigate_to_zone", self.navigate_to_zone), ("navigate_home", self.navigate_home),
            ("inspect_zone", self.inspect_zone), ("run_mission", self.run_mission),
            ("list_zones", self.list_zones), ("get_zone_image", self.get_zone_image),
            ("get_zone_gauges", self.get_zone_gauges), ("get_report", self.get_report),
            ("get_status", self.get_status),
        ]
        for name, fn in svc:
            self.create_service(ZoneTask, name, fn, callback_group=cb)
        self.get_logger().info(f"mission_control up: {len(svc)} services. zones_file={self.zones_file}")

    # ---------- helpers ----------
    def _map_cb(self, m):
        self._grid = m

    def _zones(self):
        with open(self.zones_file) as f:
            return json.load(f)["zones"]

    def _frontier_running(self):
        if self._frontier is None:
            return False
        if self._frontier.poll() is None:
            return True
        # frontier child exited on its own (exploration COMPLETE / crash) -> reap ONCE and free the leaked
        # robot busy-lock, so navigate/inspect/run_mission auto-recover and status reads idle (was: a
        # self-exited explorer wedged every motion service with a misleading 'robot busy' until /stop).
        self.get_logger().warn("exploration child exited; releasing robot lock")
        self._frontier = None
        self._release()
        return False

    def _guard(self, action):
        """Acquire the robot for a motion task. Returns (ok, msg)."""
        if self._frontier_running():
            return False, "exploration is running -- call /stop_exploration first"
        if not self._busy.acquire(blocking=False):
            return False, f"robot busy with '{self._action}'"
        self._action = action
        return True, ""

    def _release(self):
        self._action = "idle"
        try:
            self._busy.release()
        except RuntimeError:
            pass

    def _mission(self, zone_id="", inspect=True, return_home=True, goto_home=False, read=False, timeout=900):
        """Run the proven inspection_mission node as a subprocess; return (ok, manifest_dict)."""
        cmd = ["ros2", "run", "go2_inspection", "inspection_mission", "--ros-args",
               "-p", "use_sim_time:=true",
               "-p", f"zones_file:={self.zones_file}",
               "-p", f"zones:={zone_id}",
               "-p", f"inspect:={'true' if inspect else 'false'}",
               "-p", f"return_home:={'true' if return_home else 'false'}",
               "-p", f"goto_home:={'true' if goto_home else 'false'}",
               "-p", f"read:={'true' if read else 'false'}"]
        ok = False
        try:
            r = subprocess.run(cmd, env=_env(), capture_output=True, text=True, timeout=timeout)
            ok = r.returncode == 0
            if not ok:
                self.get_logger().warn(f"inspection_mission rc={r.returncode}: {(r.stderr or '')[-300:]}")
        except subprocess.TimeoutExpired:
            self.get_logger().error("inspection_mission TIMEOUT")
        man = {}
        if os.path.exists(MANIFEST):
            try:
                man = json.load(open(MANIFEST))
            except Exception:
                pass
        return ok, man

    def _zone_result(self, man, zid):
        base = next((dict(r) for r in man.get("rooms", []) if r.get("zone") == zid),
                    {"zone": zid, "n_gauges": 0})
        gj = os.path.join(GAUGES_ROOT, zid, "gauges.json")
        if os.path.exists(gj):
            try:
                base["gauges_detail"] = json.load(open(gj))
            except Exception:
                pass
        return base

    @staticmethod
    def _ok(res, success, message, payload=None):
        res.success = success
        res.message = message
        res.result_json = json.dumps(payload if payload is not None else {})
        return res

    # ---------- MAPPING ----------
    def start_exploration(self, req, res):
        if self._frontier_running():
            return self._ok(res, True, f"exploration already running (pid {self._frontier.pid})")
        if not self._busy.acquire(blocking=False):
            return self._ok(res, False, f"robot busy with '{self._action}'")
        try:
            self._frontier = subprocess.Popen(
                ["ros2", "run", "go2_exploration", "frontier_explorer", "--ros-args",
                 "-p", "use_sim_time:=true", "-p", "autostart:=true", "-p", "robot_base_frame:=base_link"],
                env=_env())
            self._action = "exploration"
            return self._ok(res, True, f"exploration started (pid {self._frontier.pid})")
        except Exception as e:
            self._busy.release()
            return self._ok(res, False, f"failed to start exploration: {e}")

    def stop_exploration(self, req, res):
        if self._frontier_running():
            self._frontier.send_signal(signal.SIGINT)
            try:
                self._frontier.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self._frontier.kill()
            msg = "exploration stopped"
        else:
            msg = "exploration not running"
        self._frontier = None
        self._release()
        return self._ok(res, True, msg)

    def save_map(self, req, res):
        npz = os.path.join(self.ws, "maps", f"{self.map_name}_map.npz")
        seg = os.path.join(self.ws, "go2_ws", "src", "go2_zones", "go2_zones", "zone_segmenter.py")
        steps = [(["python3", os.path.join(self.ws, "maps", "map_grab.py"), npz], 30, "grab"),
                 (["python3", seg, npz], 60, "zones"),
                 (["python3", os.path.join(self.ws, "maps", "npz_to_map.py"), npz], 30, "pgm")]
        lines = []
        for cmd, to, label in steps:
            try:
                r = subprocess.run(cmd, env=_env(), capture_output=True, text=True, timeout=to)
                if r.stdout.strip():
                    lines.append(r.stdout.strip().splitlines()[-1])
                if r.returncode != 0:
                    return self._ok(res, False, f"save step '{label}' failed: {(r.stderr or '')[-200:]}")
            except subprocess.TimeoutExpired:
                return self._ok(res, False, f"save step '{label}' TIMEOUT")
        # zone_segmenter writes a generic 'zones.yaml' beside the npz -> name it per-map (matches the
        # npz/pgm/db prefix) so multiple maps (maze vs facility) don't clobber each other.
        generic = os.path.join(self.ws, "maps", "zones.yaml")
        zones_out = os.path.join(self.ws, "maps", f"{self.map_name}_zones.yaml")
        if os.path.exists(generic):
            os.replace(generic, zones_out)
        db = os.path.expanduser("~/.ros/rtabmap.db")
        db_out = os.path.join(self.ws, "maps", f"{self.map_name}.db")
        if os.path.exists(db):
            subprocess.run(["cp", db, db_out])
        return self._ok(res, True,
                        f"map saved -> {self.map_name}_map.npz + _zones.yaml + _map.pgm/.yaml + .db",
                        {"npz": npz, "zones": zones_out, "db": db_out, "notes": lines})

    # ---------- NAVIGATION ----------
    def navigate_to_zone(self, req, res):
        zid = req.zone_id.strip()
        if not zid:
            return self._ok(res, False, "zone_id required")
        ok, msg = self._guard(f"navigate_to_zone:{zid}")
        if not ok:
            return self._ok(res, False, msg)
        try:
            okm, man = self._mission(zone_id=zid, inspect=False, return_home=False, timeout=300)
            return self._ok(res, okm, f"navigated to {zid}" if okm else f"navigation to {zid} failed/timeout",
                            self._zone_result(man, zid))
        finally:
            self._release()

    def navigate_home(self, req, res):
        ok, msg = self._guard("navigate_home")
        if not ok:
            return self._ok(res, False, msg)
        try:
            okm, _ = self._mission(goto_home=True, inspect=False, return_home=False, timeout=300)
            return self._ok(res, okm, "returned HOME" if okm else "return HOME failed/timeout")
        finally:
            self._release()

    # ---------- INSPECTION ----------
    def inspect_zone(self, req, res):
        zid = req.zone_id.strip()
        if not zid:
            return self._ok(res, False, "zone_id required")
        ok, msg = self._guard(f"inspect_zone:{zid}")
        if not ok:
            return self._ok(res, False, msg)
        try:
            okm, man = self._mission(zone_id=zid, inspect=True, return_home=False, read=req.read, timeout=600)
            zr = self._zone_result(man, zid)
            self._last = {"action": "inspect_zone", "zone": zid, "result": zr}
            return self._ok(res, okm, f"{zid}: {zr.get('n_gauges', 0)} gauges" if okm else f"inspect {zid} failed", zr)
        finally:
            self._release()

    def run_mission(self, req, res):
        zones = req.zone_id.strip()
        zones = "" if zones in ("", "all", "*") else zones
        ok, msg = self._guard("run_mission")
        if not ok:
            return self._ok(res, False, msg)
        try:
            okm, man = self._mission(zone_id=zones, inspect=True, return_home=True, read=req.read, timeout=1800)
            self._last = {"action": "run_mission", "result": man}
            msg = (f"mission: {man.get('total_gauges', 0)} gauges across {man.get('rooms_with_gauges', 0)} rooms"
                   if man else "mission failed (no manifest)")
            return self._ok(res, okm, msg, man)
        finally:
            self._release()

    # ---------- DATA ----------
    def list_zones(self, req, res):
        try:
            data = [{"id": z["id"], "center": z.get("center"), "area": z.get("area")} for z in self._zones()]
            return self._ok(res, True, f"{len(data)} zones", {"zones": data, "zones_file": self.zones_file})
        except Exception as e:
            return self._ok(res, False, f"cannot read zones: {e}")

    def get_zone_image(self, req, res):
        zid = req.zone_id.strip()
        d = os.path.join(GAUGES_ROOT, zid)
        if not zid or not os.path.isdir(d):
            return self._ok(res, False, f"no data for zone '{zid}'")
        pano = os.path.join(d, "panorama.png")
        sheet = os.path.join(d, "gauges_contact_sheet.png")
        panoramas = sorted(glob.glob(os.path.join(d, "panorama_*.png")))   # one per swept wall
        crops = sorted(glob.glob(os.path.join(d, "gauges", "*.png")))
        det_path = os.path.join(d, "detections.json")
        detections = None
        if os.path.exists(det_path):
            try:
                detections = json.load(open(det_path))
            except Exception:
                pass
        payload = {"zone": zid,
                   "panorama": pano if os.path.exists(pano) else None,
                   "panoramas": panoramas,                                 # per-wall-segment panoramas
                   "contact_sheet": sheet if os.path.exists(sheet) else None,
                   "crops": crops,
                   "detections": detections}
        return self._ok(res, True, f"{len(panoramas)} panoramas, {len(crops)} crops", payload)

    def get_zone_gauges(self, req, res):
        zid = req.zone_id.strip()
        d = os.path.join(GAUGES_ROOT, zid)
        out = {"zone": zid}
        gj, csv = os.path.join(d, "gauges.json"), os.path.join(d, "inspection_report.csv")
        if os.path.exists(gj):
            try:
                out["gauges"] = json.load(open(gj))
            except Exception:
                pass
        if os.path.exists(csv):
            out["report_csv"] = open(csv).read()
        ok = "gauges" in out
        n = out.get("gauges", {}).get("n_gauges", 0)
        return self._ok(res, ok, f"{n} gauges" if ok else f"no gauges.json for '{zid}'", out)

    def get_report(self, req, res):
        if not os.path.exists(MANIFEST):
            return self._ok(res, False, "no mission report yet")
        man = json.load(open(MANIFEST))
        return self._ok(res, True,
                        f"{man.get('total_gauges', 0)} gauges across {man.get('rooms_with_gauges', 0)} rooms", man)

    def get_status(self, req, res):
        known = free = total = 0
        if self._grid is not None:
            g = np.asarray(self._grid.data, dtype=np.int16)
            total, free, known = g.size, int((g == 0).sum()), int((g != -1).sum())
        running = self._frontier_running()
        unknown, pct = total - known, (round(100 * known / total, 1) if total else 0.0)
        st = {"frontier_running": running,
              "busy": self._action,
              "map_seen": self._grid is not None,
              "map_known_pct": pct,
              "map_free_cells": free,          # strictly cells==0 (rtabmap free); differs from frontier's 0..25
              "map_unknown_cells": unknown,    # unmapped cells in the grid bbox -- stays >0 (cells beyond walls),
                                               # so map_known_pct plateaus well under 100% even on a COMPLETE map
              "last": self._last,
              "zones_file": self.zones_file}
        return self._ok(res, True,
                        f"frontier={running} busy={st['busy']} known={pct}% (unknown={unknown})", st)


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node._frontier_running():
            node._frontier.send_signal(signal.SIGINT)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
