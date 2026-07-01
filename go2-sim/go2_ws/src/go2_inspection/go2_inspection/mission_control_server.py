#!/usr/bin/env python3
"""mission_control_server -- a thin ROS2 SERVICE layer over the Go2 inspection stack.

Turns the multi-terminal manual stack (frontier_explorer, inspection_mission, zone_inspector, map
saving, ...) into single-call service TRIGGERS. Each service maps 1:1 to a future WendyOS/MCP tool --
the MCP server (later, a separate process) just opens rclpy clients to these. NOTHING here is
MCP/Wendy-specific; this is pure ROS2.

DESIGN -- subprocess orchestrator (the proven pattern already used inside inspection_mission):
heavy capabilities run the EXISTING, validated nodes as child processes with their own clean rclpy
context, so this node never fights rclpy single-context/threading issues and never modifies the
working nodes. The server only holds: the frontier child handle, a robot-busy lock (one motion at a
time), a cached /map (status/coverage), and the last result. A MultiThreadedExecutor +
ReentrantCallbackGroup keep /get_status and /stop_exploration responsive while a long inspect runs.

Every service uses ONE uniform srv (go2_inspection_interfaces/ZoneTask): {zone_id} ->
{success, message, result_json}. zone_id is ignored where N/A.

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

import os, glob, json, signal, subprocess, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from go2_inspection_interfaces.srv import ZoneTask
from go2_inspection.mission_fsm import read_events

GAUGES_ROOT = os.path.expanduser("~/gauges")
MANIFEST = os.path.join(GAUGES_ROOT, "facility_inspection_manifest.json")
EVENTS = os.path.join(GAUGES_ROOT, "mission_events.jsonl")  # mission FSM event stream (ADR-016 M7b)
# This package's directory, which ships the map_grab / npz_to_map save-map helpers.
PKG_DIR = os.path.dirname(os.path.realpath(__file__))
DEFAULT_WS = os.environ.get(
    "GO2_WS",
    os.path.abspath(os.path.join(PKG_DIR, "../../../..")),
)

DEFAULT_ZONES = "~/.go2_maps/facility_inspection_zones.yaml"


def _env():
    """Child env with the standing DDS fix so subprocesses can reach the running stack."""
    e = os.environ.copy()
    e["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    return e


class MissionControl(Node):
    def __init__(self):
        super().__init__("mission_control")
        self.zones_file = os.path.expanduser(
            self.declare_parameter("zones_file", DEFAULT_ZONES).value
        )
        self.ws = self.declare_parameter("workspace", DEFAULT_WS).value
        self.map_name = self.declare_parameter("map_name", "facility_inspection").value
        # state
        self._frontier = None  # Popen handle (continuous task)
        self._busy = threading.Lock()  # one robot motion at a time
        self._busy_owner = None  # token of whoever holds _busy -- ONLY that owner may release it
        self._state_lock = (
            threading.Lock()
        )  # guards _busy(_owner)/_action/_frontier for small sections
        self._action = "idle"
        self._last = {}
        self._grid = None  # cached /map for status/coverage
        self._mission_proc = (
            None  # Popen of the currently-running async motion (own session group)
        )
        self._task = None  # JSON-able lifecycle of that motion (id/action/zone/status/result)
        self._task_seq = 0
        self.cmd_pub = self.create_publisher(
            Twist, "/cmd_vel", 10
        )  # zero-velocity safety stop on cancel
        cb = ReentrantCallbackGroup()
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, qos, callback_group=cb)
        svc = [
            ("start_exploration", self.start_exploration),
            ("stop_exploration", self.stop_exploration),
            ("save_map", self.save_map),
            ("navigate_to_zone", self.navigate_to_zone),
            ("navigate_home", self.navigate_home),
            ("inspect_zone", self.inspect_zone),
            ("run_mission", self.run_mission),
            ("cancel_task", self.cancel_task),
            ("list_zones", self.list_zones),
            ("get_zone_image", self.get_zone_image),
            ("get_zone_gauges", self.get_zone_gauges),
            ("get_report", self.get_report),
            ("get_status", self.get_status),
            ("get_events", self.get_events),
        ]
        for name, fn in svc:
            self.create_service(ZoneTask, name, fn, callback_group=cb)
        self.get_logger().info(
            f"mission_control up: {len(svc)} services. zones_file={self.zones_file}"
        )

    # ---------- helpers ----------
    def _map_cb(self, m):
        self._grid = m

    def _zones(self):
        with open(self.zones_file) as f:
            return json.load(f)["zones"]

    def _resolve_zone(self, zid):
        """(zid, None) if zid is a known zone, else (None, message listing the known zones) so the LLM can
        recover from a mis-parsed/hallucinated zone instead of launching an empty mission."""
        try:
            known = [z["id"] for z in self._zones()]
        except Exception as e:
            return None, f"cannot read zones file: {e}"
        if zid not in known:
            return None, f"unknown zone '{zid}'; known zones: {known}"
        return zid, None

    def _frontier_alive(self):
        """READ-ONLY: is the exploration child still running? No side effects (safe for get_status)."""
        return self._frontier is not None and self._frontier.poll() is None

    def _reap_frontier(self):
        """If the exploration child exited on its own (COMPLETE/crash), clear it and free the EXPLORER's
        lock so motions auto-recover. MUST be called holding _state_lock. Only ever frees a lock that the
        explorer owns -- it can never steal a running motion's lock."""
        if self._frontier is not None and self._frontier.poll() is not None:
            self.get_logger().warn("exploration child exited; releasing robot lock")
            self._frontier = None
            if self._busy_owner == "exploration" and self._busy.locked():
                self._busy_owner = None
                self._action = "idle"
                self._busy.release()

    def _guard(self, action):
        """Acquire the robot for a motion task. Returns (ok, msg, owner); pass owner back to _release."""
        owner = threading.get_ident()  # a motion acquires + releases on the SAME callback thread
        with self._state_lock:
            self._reap_frontier()
            if self._frontier_alive():
                return False, "exploration is running -- call /stop_exploration first", None
            if not self._busy.acquire(blocking=False):
                return False, f"robot busy with '{self._action}'", None
            self._busy_owner = owner
            self._action = action
        return True, "", owner

    def _release(self, owner):
        """Free the robot lock ONLY if `owner` actually holds it. A caller that doesn't own the lock (e.g.
        stop_exploration while an inspect is running) is a no-op -- this is what prevents a stolen lock from
        admitting a second concurrent motion."""
        with self._state_lock:
            if self._busy.locked() and self._busy_owner == owner:
                self._busy_owner = None
                self._action = "idle"
                self._busy.release()

    def _mission_cmd(self, zone_id="", inspect=True, return_home=True, goto_home=False, read=False):
        return [
            "ros2",
            "run",
            "go2_inspection",
            "inspection_mission",
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            f"zones_file:={self.zones_file}",
            "-p",
            f"zones:={zone_id}",
            "-p",
            f"inspect:={'true' if inspect else 'false'}",
            "-p",
            f"return_home:={'true' if return_home else 'false'}",
            "-p",
            f"goto_home:={'true' if goto_home else 'false'}",
            "-p",
            f"read:={'true' if read else 'false'}",  # ADR-016 M4: also model-read detected gauges
        ]

    @staticmethod
    def _trim_objects(objs):
        """Compact per-object payload for tool output (full detail stays in ~/gauges/<zone>/objects.json):
        drops 15-decimal coords / raw crops, keeps what an LLM reasons over."""
        out = []
        for o in objs or []:
            w = o.get("world")
            out.append(
                {
                    "id": o.get("id"),
                    "class": o.get("class"),
                    "confidence": round(float(o.get("confidence", 0.0)), 2),
                    "localized": o.get("localized", False),
                    "n_observations": o.get("n_observations", 1),
                    "world": [round(w[0], 2), round(w[1], 2)] if w and w[0] is not None else None,
                }
            )
        return out

    def _zone_result(self, man, zid):
        base = next(
            (dict(r) for r in man.get("rooms", []) if r.get("zone") == zid),
            {"zone": zid, "n_objects": 0},
        )
        oj = os.path.join(GAUGES_ROOT, zid, "objects.json")
        if os.path.exists(oj):
            try:
                full = json.load(open(oj))
                base["n_objects"] = full.get("n_objects", base.get("n_objects", 0))
                base["objects"] = self._trim_objects(full.get("objects", []))
            except Exception:
                pass
        return base

    # ---------- safe subprocess-tree control (respects the 'never kill a broad group' rule) ----------
    def _kill_proc_tree(self, proc, grace=6.0):
        """Terminate a mission child AND its grandchildren (ros2 run -> node -> zone_inspector). The child is
        spawned with start_new_session=True so it is its OWN session/process-group leader (pgid == pid),
        ISOLATED from this server. We signal ONLY that group, with hard guards making it impossible to ever
        hit pgid 0/1 or THIS server's own group. If the child is somehow not an isolated leader we fall back
        to terminating the single PID -- we NEVER killpg a group we don't positively own."""
        if proc is None or proc.poll() is not None:
            return
        pgid = getattr(
            proc, "_pgid", None
        )  # captured at spawn for mission procs (never getpgid a reaped pid)
        if pgid is None:  # edge proc without a stored group -> query its ACTUAL group
            try:
                pgid = os.getpgid(proc.pid)
            except (ProcessLookupError, OSError):
                return
        try:
            own = os.getpgid(0)
        except OSError:
            return
        if not (pgid == proc.pid and pgid not in (0, 1) and pgid != own):
            try:
                proc.terminate()  # not a positively-owned isolated group -> single PID only
            except Exception:
                pass
            return
        if (
            proc.poll() is not None
        ):  # reaped meanwhile -> don't signal a possibly-reused pid's group
            return
        try:
            os.killpg(pgid, signal.SIGINT)
        except (ProcessLookupError, OSError):
            return
        # reaping is the monitor thread's job (its proc.wait); here we only poll for death, then escalate
        deadline = time.monotonic() + grace
        while proc.poll() is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if proc.poll() is None:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass

    def _stop_robot(self):
        """Publish zero-velocity Twists so the robot halts after a killed motion (the dead child's last
        /cmd_vel can otherwise latch on the gait bridge)."""
        z = Twist()
        for _ in range(3):
            self.cmd_pub.publish(z)

    # ---------- async motion: start now, monitor in the background, release the lock on finish ----------
    def _start_mission(
        self,
        res,
        action,
        zone_id="",
        inspect=True,
        return_home=False,
        goto_home=False,
        read=False,
        timeout=900,
    ):
        """Start inspection_mission as an ISOLATED-session subprocess and return IMMEDIATELY with a task id;
        a daemon monitor thread reaps it and frees the robot lock. Non-blocking, so the LLM tool call returns
        at once -- poll get_status for progress/result, cancel_task to abort."""
        ok, msg, owner = self._guard(action)
        if not ok:
            return self._ok(res, False, msg)
        # Record the task as 'running' BEFORE spawning so a cancel arriving during startup is not lost
        # (cancel sets cancel_requested even before the proc exists; we honor it right after Popen).
        self._task_seq += 1
        tid = f"{action.split(':')[0]}-{self._task_seq}"
        with self._state_lock:
            self._mission_proc = None
            self._task = {
                "id": tid,
                "action": action,
                "zone": zone_id,
                "status": "running",
                "t0": time.monotonic(),
                "cancel_requested": False,
            }
        try:
            proc = subprocess.Popen(
                self._mission_cmd(zone_id, inspect, return_home, goto_home, read),
                env=_env(),
                start_new_session=True,
            )
        except Exception as e:
            with self._state_lock:
                self._task = None
            self._release(owner)
            return self._ok(res, False, f"failed to start {action}: {e}")
        proc._pgid = (
            proc.pid
        )  # start_new_session => pgid==pid; capture now (never getpgid a reaped pid)
        with self._state_lock:
            self._mission_proc = proc
            cancel_now = bool(self._task and self._task.get("cancel_requested"))
        if cancel_now:  # a cancel landed during startup -> kill the just-spawned tree
            self._kill_proc_tree(proc)
            self._stop_robot()
        try:
            threading.Thread(
                target=self._monitor_mission,
                args=(owner, proc, action, zone_id, goto_home, timeout),
                daemon=True,
            ).start()
        except Exception as e:  # the monitor is the ONLY lock-releaser -> never leak it
            self._kill_proc_tree(proc)
            self._stop_robot()
            with self._state_lock:
                self._mission_proc = None
                self._task = None
            self._release(owner)
            return self._ok(res, False, f"failed to start monitor for {action}: {e}")
        return self._ok(
            res,
            True,
            f"{action} started -- poll get_status for progress, cancel_task to abort",
            {"task_id": tid, "action": action, "zone": zone_id},
        )

    def _monitor_mission(self, owner, proc, action, zid, goto_home, timeout):
        # The lock release is in `finally` so ANY error in here can never strand the robot lock (held forever
        # would wedge every future motion). status defaults to 'failed' until we compute the real outcome.
        status, result, rc = "failed", None, None
        try:
            timed_out = False
            try:
                proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                timed_out = True
                self.get_logger().error(
                    f"{action} TIMEOUT after {timeout:.0f}s -- killing subprocess tree"
                )
                self._kill_proc_tree(proc)
                self._stop_robot()
            rc = proc.poll()
            man = {}
            if os.path.exists(MANIFEST):
                try:
                    man = json.load(open(MANIFEST))
                except Exception:
                    pass
            if not isinstance(man, dict):  # a corrupt/non-dict manifest -> empty, not a crash
                man = {}
            with self._state_lock:
                cancelled = bool(self._task and self._task.get("cancel_requested"))
            if action.startswith("inspect_zone"):
                result = self._zone_result(man, zid)
            elif action == "run_mission":
                result = man
            elif goto_home:
                result = {"home": rc == 0}
            else:
                result = {"zone": zid, "navigated": rc == 0}
            status = (
                "cancelled"
                if cancelled
                else "succeeded"
                if rc == 0
                else "timeout"
                if timed_out
                else "failed"
            )
        except Exception as e:
            self.get_logger().error(f"{action} monitor error: {e}")
        finally:
            with self._state_lock:
                if self._mission_proc is proc:
                    if self._task:
                        self._task.update(
                            {
                                "status": status,
                                "result": result,
                                "elapsed_sec": round(time.monotonic() - self._task["t0"], 1),
                            }
                        )
                    self._mission_proc = None
                self._last = {"action": action, "zone": zid, "status": status}
            self._release(owner)
            self.get_logger().info(f"{action} -> {status} (rc={rc})")

    @staticmethod
    def _ok(res, success, message, payload=None):
        res.success = success
        res.message = message
        res.result_json = json.dumps(payload if payload is not None else {})
        return res

    # ---------- MAPPING ----------
    def start_exploration(self, req, res):
        with self._state_lock:
            self._reap_frontier()
            if self._frontier_alive():
                return self._ok(
                    res, True, f"exploration already running (pid {self._frontier.pid})"
                )
            if not self._busy.acquire(blocking=False):
                return self._ok(res, False, f"robot busy with '{self._action}'")
            self._busy_owner = "exploration"
            self._action = "exploration"
        try:
            self._frontier = subprocess.Popen(  # Popen OUTSIDE the state lock
                [
                    "ros2",
                    "run",
                    "go2_exploration",
                    "frontier_explorer",
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                    "-p",
                    "autostart:=true",
                    "-p",
                    "robot_base_frame:=base_link",
                ],
                env=_env(),
            )
            return self._ok(res, True, f"exploration started (pid {self._frontier.pid})")
        except Exception as e:
            self._release("exploration")
            return self._ok(res, False, f"failed to start exploration: {e}")

    def stop_exploration(self, req, res):
        with self._state_lock:
            proc = self._frontier if self._frontier_alive() else None
        if proc is not None:
            proc.send_signal(
                signal.SIGINT
            )  # SIGINT to the specific frontier child PID (not a group)
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.kill()  # SIGKILL the specific PID we spawned
            with self._state_lock:
                self._frontier = None
                if self._busy_owner == "exploration" and self._busy.locked():
                    self._busy_owner = None
                    self._action = "idle"
                    self._busy.release()
            return self._ok(res, True, "exploration stopped")
        # NOT running -> clear a dead handle but DON'T touch _busy (a navigate/inspect/mission may hold it)
        with self._state_lock:
            self._frontier = None
        return self._ok(res, True, "exploration not running")

    def save_map(self, req, res):
        if self._frontier_alive():
            return self._ok(
                res,
                False,
                "exploration is still running -- call stop_exploration before save_map "
                "(saving mid-mapping snapshots a moving/locked map)",
            )
        npz = os.path.join(self.ws, "maps", f"{self.map_name}_map.npz")
        seg = os.path.join(self.ws, "go2_ws", "src", "go2_zones", "go2_zones", "zone_segmenter.py")
        steps = [
            (["python3", os.path.join(PKG_DIR, "map_grab.py"), npz], 30, "grab"),
            (["python3", seg, npz], 60, "zones"),
            (["python3", os.path.join(PKG_DIR, "npz_to_map.py"), npz], 30, "pgm"),
        ]
        lines = []
        for cmd, to, label in steps:
            try:
                r = subprocess.run(cmd, env=_env(), capture_output=True, text=True, timeout=to)
                if r.stdout.strip():
                    lines.append(r.stdout.strip().splitlines()[-1])
                if r.returncode != 0:
                    return self._ok(
                        res, False, f"save step '{label}' failed: {(r.stderr or '')[-200:]}"
                    )
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
            cp = subprocess.run(
                ["cp", db, db_out], env=_env(), capture_output=True, text=True, timeout=30
            )
            if cp.returncode != 0:
                lines.append(f"(db copy failed: {(cp.stderr or '').strip()[-120:]})")
        return self._ok(
            res,
            True,
            f"map saved -> {self.map_name}_map.npz + _zones.yaml + _map.pgm/.yaml + .db",
            {"npz": npz, "zones": zones_out, "db": db_out, "notes": lines},
        )

    # ---------- NAVIGATION (async: start + poll get_status) ----------
    def navigate_to_zone(self, req, res):
        zid = req.zone_id.strip()
        if not zid:
            return self._ok(res, False, "zone_id required")
        zid, err = self._resolve_zone(zid)
        if err:
            return self._ok(res, False, err)
        return self._start_mission(
            res,
            f"navigate_to_zone:{zid}",
            zone_id=zid,
            inspect=False,
            return_home=False,
            timeout=300,
        )

    def navigate_home(self, req, res):
        return self._start_mission(
            res, "navigate_home", goto_home=True, inspect=False, return_home=False, timeout=300
        )

    # ---------- INSPECTION (async: start + poll get_status) ----------
    def inspect_zone(self, req, res):
        zid = req.zone_id.strip()
        if not zid:
            return self._ok(res, False, "zone_id required")
        zid, err = self._resolve_zone(zid)
        if err:
            return self._ok(res, False, err)
        return self._start_mission(
            res, f"inspect_zone:{zid}", zone_id=zid, inspect=True, return_home=False,
            read=bool(getattr(req, "read", False)), timeout=900,
        )

    def run_mission(self, req, res):
        zones = req.zone_id.strip()
        zones = "" if zones in ("", "all", "*") else zones
        if zones:  # a subset -> validate each id so an all-unknown list
            req_ids = [
                z for z in zones.split(",") if z
            ]  # doesn't 'succeed' having visited nothing
            try:
                known = {z["id"] for z in self._zones()}
            except Exception as e:
                return self._ok(res, False, f"cannot read zones file: {e}")
            bad = [z for z in req_ids if z not in known]
            if bad:
                return self._ok(res, False, f"unknown zone(s) {bad}; known zones: {sorted(known)}")
            zones = ",".join(req_ids)
        return self._start_mission(
            res, "run_mission", zone_id=zones, inspect=True, return_home=True,
            read=bool(getattr(req, "read", False)), timeout=3600,
        )

    def cancel_task(self, req, res):
        """Abort whatever the robot is doing -- a running navigate/inspect/run_mission AND/OR exploration --
        and stop the robot. The motion's monitor thread marks the task 'cancelled' and frees the robot lock."""
        with self._state_lock:
            proc = self._mission_proc
            if self._task and self._task.get("status") == "running":
                self._task["cancel_requested"] = True
            frontier = self._frontier if self._frontier_alive() else None
        did = []
        if proc is not None:
            self._kill_proc_tree(
                proc
            )  # scoped killpg of the child's OWN session (see _kill_proc_tree)
            did.append("motion")
        if frontier is not None:
            frontier.send_signal(signal.SIGINT)  # SIGINT the specific frontier PID
            try:
                frontier.wait(timeout=8)
            except subprocess.TimeoutExpired:
                frontier.kill()
            with self._state_lock:
                self._frontier = None
                if self._busy_owner == "exploration" and self._busy.locked():
                    self._busy_owner = None
                    self._action = "idle"
                    self._busy.release()
            did.append("exploration")
        self._stop_robot()
        if not did:
            return self._ok(res, True, "nothing to cancel (robot idle)")
        return self._ok(res, True, f"cancelled {', '.join(did)}; robot stopping (poll get_status)")

    # ---------- DATA ----------
    def list_zones(self, req, res):
        try:
            data = [
                {"id": z["id"], "center": z.get("center"), "area": z.get("area")}
                for z in self._zones()
            ]
            return self._ok(
                res, True, f"{len(data)} zones", {"zones": data, "zones_file": self.zones_file}
            )
        except Exception as e:
            return self._ok(res, False, f"cannot read zones: {e}")

    def get_zone_image(self, req, res):
        zid = req.zone_id.strip()
        d = os.path.join(GAUGES_ROOT, zid)
        if not zid or not os.path.isdir(d):
            return self._ok(res, False, f"no data for zone '{zid}'")
        sheet = os.path.join(d, "objects_contact_sheet.png")
        zmap = os.path.join(d, "zone_map.png")
        crops = sorted(glob.glob(os.path.join(d, "crops", "*.png")))
        # IMAGE PATHS ONLY -- the object DATA lives in get_zone_gauges; embedding it here too just duplicated it.
        payload = {
            "zone": zid,
            "zone_map": zmap if os.path.exists(zmap) else None,
            "contact_sheet": sheet if os.path.exists(sheet) else None,
            "crops": crops,
        }
        return self._ok(res, True, f"{len(crops)} crops", payload)

    def get_zone_gauges(self, req, res):
        """Detected objects for a zone (service name kept stable for the MCP layer; returns the TRIMMED
        objects list -- full detail incl. raw crops/CSV stays in ~/gauges/<zone>/)."""
        zid = req.zone_id.strip()
        oj = os.path.join(GAUGES_ROOT, zid, "objects.json")
        if not os.path.exists(oj):
            return self._ok(res, False, f"no objects.json for '{zid}' (inspect the zone first)")
        try:
            full = json.load(open(oj))
        except Exception as e:
            return self._ok(res, False, f"cannot read objects for '{zid}': {e}")
        n = full.get("n_objects", 0)
        out = {"zone": zid, "n_objects": n, "objects": self._trim_objects(full.get("objects", []))}
        return self._ok(res, True, f"{n} objects", out)

    def get_report(self, req, res):
        if not os.path.exists(MANIFEST):
            return self._ok(res, False, "no mission report yet")
        try:
            man = json.load(open(MANIFEST))
        except Exception as e:
            return self._ok(res, False, f"cannot read report: {e}")
        if not isinstance(man, dict):
            return self._ok(res, False, "report file is corrupt")
        return self._ok(
            res,
            True,
            f"{man.get('total_objects', 0)} objects across {man.get('zones_with_objects', 0)} zones",
            man,
        )

    def get_status(self, req, res):
        # READ-ONLY snapshot under the state lock -- no reaping / lock release here. This is the most-polled
        # tool, and a side-effecting status read must never free the motion lock by accident.
        with self._state_lock:
            running = self._frontier_alive()
            action = self._action
            last = dict(self._last)
            grid = self._grid
            task = dict(self._task) if self._task else None
        if task is not None:
            t0 = task.pop("t0", None)  # internal monotonic stamp -- never expose it
            if t0 is not None and task.get("status") == "running":
                task["elapsed_sec"] = round(time.monotonic() - t0, 1)
        known = free = total = 0
        if grid is not None:
            g = np.asarray(grid.data, dtype=np.int16)
            total, free, known = g.size, int((g == 0).sum()), int((g != -1).sum())
        unknown, pct = total - known, (round(100 * known / total, 1) if total else 0.0)
        st = {
            "frontier_running": running,
            "busy": action,
            "task": task,  # current/last async motion: {id, action, zone, status,
            #   elapsed_sec, result} -- poll until status != 'running'
            "map_seen": grid is not None,
            "map_known_pct": pct,
            "map_free_cells": free,  # strictly cells==0 (rtabmap free); differs from frontier's 0..25
            "map_unknown_cells": unknown,  # unmapped cells in the grid bbox -- stays >0 (cells beyond walls),
            # so map_known_pct plateaus well under 100% even on a COMPLETE map
            "last": last,
            "zones_file": self.zones_file,
        }
        # Surface the driving mission's live phase from the FSM event stream (ADR-016 M7b). Read-only +
        # defensive: a missing/garbled stream just omits the phase, never affects the status response.
        try:
            ev = read_events(EVENTS, last=1)
        except Exception:
            ev = []
        if ev:
            e = ev[0]
            st["mission_phase"] = e.get("state")
            st["mission_event"] = {
                k: e.get(k) for k in ("seq", "kind", "zone", "data") if e.get(k) is not None
            }
        tstat = f" task={task['status']}" if task else ""
        phase = f" phase={st['mission_phase']}" if ev else ""
        return self._ok(
            res,
            True,
            f"frontier={running} busy={action}{tstat}{phase} known={pct}% (unknown={unknown})",
            st,
        )

    def get_events(self, req, res):
        """Full mission FSM event history (the structured progress stream the dashboard/MCP can replay):
        [{seq, t, state, kind, zone?, data?}]. zone_id is ignored. Read-only; empty if no mission has run."""
        try:
            events = read_events(EVENTS)
        except Exception:
            events = []
        phase = events[-1]["state"] if events else None
        return self._ok(
            res, True, f"{len(events)} events; phase={phase}", {"events": events, "phase": phase}
        )


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
        if node._frontier_alive():
            node._frontier.send_signal(signal.SIGINT)
        if (
            node._mission_proc is not None
        ):  # don't leave a mission subprocess tree driving the robot
            node._kill_proc_tree(node._mission_proc)
        node._stop_robot()  # zero /cmd_vel so the robot doesn't coast on a latched cmd
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
