# Mission-Control Service Layer — the trigger surface

The multi-terminal stack is wrapped in **one ROS2 service node** so each capability is a single
call. This is the foundation the MCP tools sit on: **every service maps 1:1 to an MCP tool**, and the
MCP server (separate process) is just a rclpy client to these. Nothing here is MCP-specific — it's pure
ROS2 and runs standalone.

## Design (why it's safe)
`mission_control_server` is a **subprocess orchestrator** — the same pattern `inspection_mission`
already uses for the sweeper/segmenter. Heavy capabilities run the **existing, validated nodes**
(`inspection_mission`, `frontier_explorer`, the save scripts) as child processes with their own clean
rclpy context. So the server **never modifies the working nodes** and never fights rclpy
single-context/threading issues. It only holds: the frontier child handle, a **robot-busy lock** (one
motion at a time), a cached `/map` (for status/coverage), and the last result. A `MultiThreadedExecutor`
keeps `/get_status` and `/stop_exploration` responsive while a long inspect/mission runs.

All services use **one uniform srv** `go2_inspection_interfaces/srv/ZoneTask`:
`{string zone_id, bool read}` → `{bool success, string message, string result_json}`
(`zone_id`/`read` are ignored where N/A; `result_json` carries structured payloads).

## Start it (beside whatever base stack is up — it does NOT bring up the sim)
```bash
ros2 launch go2_bringup mission_control.launch.py
```
It needs the **base stack** already running:
- **mapping mode** (for `start/stop_exploration`, `save_map`): the rtabmap **SLAM** + Nav2 stack.
- **inspection mode** (for `navigate/inspect/run_mission`): the **localization + map_server** + Nav2 stack
  (`inspection_nav.launch.py`).

## Service catalog

| Group | Service | zone_id | read | Returns (`result_json`) |
|---|---|---|---|---|
| **Mapping** | `/start_exploration` | – | – | starts `frontier_explorer` |
| | `/stop_exploration` | – | – | stops it |
| | `/save_map` | – | – | `{npz, db, notes}` (grid+zones+pgm/yaml+db) |
| **Navigation** | `/navigate_to_zone` | ✔ | – | zone nav result |
| | `/navigate_home` | – | – | dock at HOME (0,0) |
| **Inspection** | `/inspect_zone` | ✔ | ✔ | one zone: approach→sweep→segment→(read) → `{n_gauges, gauges_detail}` |
| | `/run_mission` | csv/`all` | ✔ | whole facility → manifest `{rooms, total_gauges,…}` |
| **Data** | `/list_zones` | – | – | `{zones:[{id,center,area}]}` |
| | `/get_zone_image` | ✔ | – | `{panorama, contact_sheet, crops[]}` paths |
| | `/get_zone_gauges` | ✔ | – | `{gauges, report_csv}` |
| | `/get_report` | – | – | last mission manifest |
| | `/get_status` | – | – | `{frontier_running, busy, map_known_pct, last}` |

## Examples
```bash
T='go2_inspection_interfaces/srv/ZoneTask'
# mapping
ros2 service call /start_exploration $T "{}"
ros2 service call /get_status       $T "{}"          # watch map_known_pct climb
ros2 service call /save_map         $T "{}"
ros2 service call /stop_exploration $T "{}"
# inspection
ros2 service call /list_zones       $T "{}"
ros2 service call /navigate_to_zone $T "{zone_id: zone_3}"
ros2 service call /inspect_zone     $T "{zone_id: zone_3, read: false}"   # read:true needs ANTHROPIC_API_KEY
ros2 service call /run_mission      $T "{zone_id: all, read: false}"
ros2 service call /get_zone_image   $T "{zone_id: zone_3}"
ros2 service call /navigate_home    $T "{}"
```

## How it composes the existing nodes
`inspection_mission` gained additive flags (defaults unchanged): `inspect` (False = navigate only),
`return_home`, `goto_home` (just dock), `read`. The server drives them:
- `navigate_to_zone` → `inspection_mission zones:=<id> inspect:=false return_home:=false`
- `inspect_zone`     → `inspection_mission zones:=<id> inspect:=true  return_home:=false read:=<>`
- `run_mission`      → `inspection_mission zones:=<csv> inspect:=true return_home:=true  read:=<>`
- `navigate_home`    → `inspection_mission goto_home:=true`
- `save_map`         → `map_grab.py → zone_segmenter.py → npz_to_map.py → cp rtabmap.db`
- `start_exploration`→ `frontier_explorer` (tracked child; `stop_exploration` SIGINTs it)

## Notes / guardrails
- **One motion at a time:** nav/inspect/mission take a busy-lock; a call during another returns
  `success:false, "robot busy with '<action>'"`. Exploration must be stopped before inspecting.
- **Standing DDS fix:** the server sets `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` on every child.
- **Reading is opt-in:** `read:true` AND `ANTHROPIC_API_KEY` in the server's env.
- **On the roadmap:** the MCP server exposing these as MCP tools (`get_zone_image` returning the actual
  image bytes, etc.).
