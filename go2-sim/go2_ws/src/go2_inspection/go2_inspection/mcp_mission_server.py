#!/usr/bin/env python3
"""mcp_mission_server -- a FastMCP (stdio) server that exposes the SIM mission-control ROS2 services as
MCP tools, so you can drive the Go2 inspection sim in natural language.

It is a thin bridge: one MCP tool per mission_control_server service (all the custom
go2_inspection_interfaces/srv/ZoneTask {zone_id} -> {success, message, result_json}). A single rclpy
node holds a client to each service and spins in a background thread; each (synchronous) MCP tool calls
its service and blocks for the response. FastMCP runs sync tools in a worker thread, so blocking here does
not stall the MCP stdio loop.

SCOPE: SIMULATION ONLY (talks to the sim's mission_control on the default ROS domain). Nothing WendyOS.

Run via run_mcp_sim.sh (which sources ROS + the sim workspace + the sim's DDS env, then runs this server
with the python3 on PATH, which must have fastmcp). Register it with your MCP client (stdio) pointing at
this wrapper script, e.g.:
    <mcp-client> add go2-sim -- "/abs/path/to/go2-sim/go2_ws/src/run_mcp_sim.sh"

The sim base stack must be running for the tools to do anything:
  - mapping/exploration:  ros2 launch go2_bringup rtabmap_slam.launch.py world:=maze.sdf headless:=false
                          (+ nav2.launch.py so the frontier explorer can drive)
  - inspection:           ros2 launch go2_bringup inspection_nav.launch.py world:=maze.sdf map_yaml:=...
  - the service layer:     ros2 launch go2_bringup mission_control.launch.py [zones_file:=... map_name:=...]
If mission_control is not up, every tool returns a clear "service not available" message (no crash).
"""

import os, re, json, threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from go2_inspection_interfaces.srv import ZoneTask
from fastmcp import FastMCP

mcp = FastMCP("go2-sim-inspection")

SERVICES = [
    "start_exploration",
    "stop_exploration",
    "save_map",
    "navigate_to_zone",
    "navigate_home",
    "inspect_zone",
    "run_mission",
    "cancel_task",
    "list_zones",
    "get_zone_image",
    "get_zone_gauges",
    "get_report",
    "get_status",
]

_node = None
_node_lock = threading.Lock()


def _ensure_node():
    """Lazily start rclpy + the bridge node + a background spinner (idempotent, thread-safe)."""
    global _node
    with _node_lock:
        if _node is None:
            if not rclpy.ok():
                rclpy.init()
            n = rclpy.create_node("mcp_mission_client")
            n._svc = {s: n.create_client(ZoneTask, s) for s in SERVICES}
            ex = MultiThreadedExecutor()
            ex.add_node(n)
            threading.Thread(target=ex.spin, daemon=True).start()
            _node = n
    return _node


def _norm_zone(z):
    """Normalize NL zone references: 'zone_3'/'zone 3'/'zone-3'/'room 3'/'3' -> 'zone_3';
    ''/'all'/'all zones'/'everything'/'whole facility' -> 'all'. Anything else is returned unchanged so the
    server can validate it and reply with the list of known zones (lets the LLM recover)."""
    z = (z or "").strip().lower()
    if z in (
        "",
        "all",
        "*",
        "everything",
        "every zone",
        "all zones",
        "all the zones",
        "whole facility",
    ):
        return "all"
    m = re.match(r"^(?:zone|room)?[ _\-]*(\d+)\b", z)
    if m:
        return f"zone_{m.group(1)}"
    return z


def _call(name, zone_id="", read=False, timeout=600.0):
    """Call one ZoneTask service and block for the response. Returns a JSON-able dict."""
    node = _ensure_node()
    cli = node._svc.get(name)
    if cli is None:
        return {"success": False, "message": f"unknown service '{name}'"}
    if not cli.wait_for_service(timeout_sec=5.0):
        return {
            "success": False,
            "message": (
                f"/{name} is not available. Start the sim + the service layer first: "
                f"`ros2 launch go2_bringup mission_control.launch.py` (and the sim base stack). "
                f"Also make sure the MCP server's DDS env matches the sim "
                f"(ROS_DOMAIN_ID / FASTDDS_BUILTIN_TRANSPORTS / ROS_LOCALHOST_ONLY)."
            ),
        }
    req = ZoneTask.Request()
    req.zone_id = zone_id or ""
    if hasattr(req, "read"):                # gauge-reading layer (ADR-016 M4); srv superset field
        req.read = bool(read)
    fut = cli.call_async(req)
    done = threading.Event()
    fut.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout):
        return {
            "success": False,
            "message": f"/{name} has not responded after {timeout:.0f}s -- it may still be running. "
            f"Call get_status to check.",
        }
    try:
        resp = fut.result()
    except Exception as e:
        return {"success": False, "message": f"/{name} call failed: {type(e).__name__}: {e}"}
    out = {"success": bool(resp.success), "message": resp.message}
    if resp.result_json:
        try:
            out["result"] = json.loads(resp.result_json)
        except Exception:
            out["result"] = resp.result_json
    return out


# ---------------------------------------------------------------- MAPPING / EXPLORATION
@mcp.tool
def start_exploration() -> dict:
    """Begin AUTONOMOUS frontier exploration: the Go2 drives itself around the unknown area, building the
    map with RTAB-Map SLAM. NON-BLOCKING -- returns immediately. Use get_status to watch coverage, then
    stop_exploration when the map looks complete, then save_map. Requires the mapping stack
    (rtabmap_slam.launch.py + nav2.launch.py) running."""
    return _call("start_exploration", timeout=30)


@mcp.tool
def stop_exploration() -> dict:
    """Stop the autonomous frontier exploration started by start_exploration (frees the robot for other
    actions)."""
    return _call("stop_exploration", timeout=30)


@mcp.tool
def save_map() -> dict:
    """Save the map just built by exploration: occupancy grid (npz/pgm/yaml), the auto-segmented zones
    file, and the RTAB-Map database (named by the launch's map_name). Call after exploration is complete.
    Takes up to ~2 minutes."""
    return _call(
        "save_map", timeout=180
    )  # margin over the server's 30+60+30+30s sequential worst case


# ---------------------------------------------------------------- NAVIGATION
@mcp.tool
def navigate_to_zone(zone: str) -> dict:
    """START driving the robot to a zone/room with Nav2 (DRIVES only -- does not inspect; use inspect_zone
    for that). Returns IMMEDIATELY with a task_id; the drive runs in the background (~minutes). Poll
    get_status until status.task.status != 'running', or call cancel_task to abort. Accepts 'zone_3', '3',
    'zone 3'."""
    return _call("navigate_to_zone", zone_id=_norm_zone(zone), timeout=30)


@mcp.tool
def navigate_home() -> dict:
    """START driving the robot back to HOME (map origin, 0,0). Returns immediately; poll get_status; call
    cancel_task to abort."""
    return _call("navigate_home", timeout=30)


# ---------------------------------------------------------------- INSPECTION
@mcp.tool
def inspect_zone(zone: str, read_gauges: bool = False) -> dict:
    """START inspecting ONE zone (drive there, sample viewpoints + slow 360-degree spin with LIVE open-vocab
    YOLOE object detection, project each object onto the map, de-duplicate, crop). Returns IMMEDIATELY with a
    task_id -- the scan runs in the BACKGROUND (~10-15 min). Poll get_status until status.task.status !=
    'running' (then status.task.result holds the objects), or call cancel_task to abort. Afterwards use
    get_zone_objects / get_zone_image to review. Accepts 'zone_3', '3', 'zone 3'. Set read_gauges=true to
    ALSO have the model read any detected analog gauges into values (needs ANTHROPIC_API_KEY in the sim env)."""
    return _call("inspect_zone", zone_id=_norm_zone(zone), read=read_gauges, timeout=30)


@mcp.tool
def run_mission(zones: str = "all", read_gauges: bool = False) -> dict:
    """START the FULL autonomous inspection mission: from HOME, visit + inspect every candidate zone (or a
    subset), return HOME, build one facility report + map. Returns IMMEDIATELY with a task_id; the mission
    runs in the BACKGROUND (can be ~30-60 min). Poll get_status (status.task.status / .result) or get_report;
    call cancel_task to abort. 'zones'='all' for every zone, or a comma list like 'zone_0,zone_3' (or '0,3').
    Set read_gauges=true to ALSO model-read detected analog gauges into values (needs ANTHROPIC_API_KEY)."""
    z = zones.strip()
    if z.lower() in ("", "all", "*"):
        zone_id = ""  # mission_control treats '' as 'all candidate zones'
    else:
        zone_id = ",".join(_norm_zone(x) for x in z.split(",") if x.strip())
    return _call("run_mission", zone_id=zone_id, read=read_gauges, timeout=30)


@mcp.tool
def cancel_task() -> dict:
    """STOP the robot: abort any running navigate/inspect/run_mission (and exploration) and halt motion. Call
    this when the operator says stop/abort/cancel/come back, or to clear a stuck action. Safe to call when idle."""
    return _call("cancel_task", timeout=30)


# ---------------------------------------------------------------- DATA / STATUS
@mcp.tool
def list_zones() -> dict:
    """List the inspectable zones (id, centre, area) from the loaded zones file."""
    return _call("list_zones", timeout=15)


@mcp.tool
def get_zone_objects(zone: str) -> dict:
    """Get the OBJECTS detected in a zone (id, class, confidence, map position, observation count). This is
    open-vocab object DETECTION, not gauge/instrument readings. Call after inspect_zone has finished (poll
    get_status). Accepts 'zone_3', '3', 'zone 3'."""
    return _call("get_zone_gauges", zone_id=_norm_zone(zone), timeout=15)


@mcp.tool
def get_zone_gauges(zone: str) -> dict:
    """DEPRECATED alias of get_zone_objects (the robot detects generic objects, not gauges). Prefer
    get_zone_objects."""
    return _call("get_zone_gauges", zone_id=_norm_zone(zone), timeout=15)


@mcp.tool
def get_report() -> dict:
    """Get the overall facility inspection report/manifest: total objects, zones inspected, per-zone
    results."""
    return _call("get_report", timeout=15)


@mcp.tool
def get_status() -> dict:
    """Robot/mission status -- the tool to POLL while a navigate/inspect/run_mission runs. Returns:
    frontier_running, busy (current action), task {id, action, zone, status: running|succeeded|failed|
    cancelled|timeout, elapsed_sec, result}, map coverage %, and the last action. Read-only and safe to call
    anytime, including in a poll loop."""
    return _call("get_status", timeout=15)


@mcp.tool
def get_zone_image(zone: str):
    """Show the inspection imagery for a zone: returns the annotated zone map (objects plotted on the
    world map), the contact sheet, and each detected-object CROP image inline (so you can SEE what the
    robot found) plus the file paths. Accepts 'zone_3', '3', or 'zone 3'."""
    from fastmcp.utilities.types import Image

    r = _call("get_zone_image", zone_id=_norm_zone(zone), timeout=20)
    content = [json.dumps({"success": r.get("success"), "message": r.get("message")})]
    res = r.get("result") or {}
    crops = res.get("crops") or []
    zmap = res.get("zone_map")
    sheet = res.get("contact_sheet")
    imgs = [p for p in ([zmap, sheet] + list(crops)) if isinstance(p, str) and os.path.isfile(p)]
    content.append(
        f"{len(crops)} crops for {_norm_zone(zone)}"
        + (" (no images on disk yet -- inspect the zone first)" if not imgs else "")
    )
    for p in imgs:
        content.append(f"file: {p}")
        try:
            content.append(Image(path=p))
        except Exception:
            pass
    return content


def main():
    _ensure_node()
    mcp.run()  # stdio transport (the MCP client spawns this)


if __name__ == "__main__":
    main()
