# go2_inspection — autonomous gauge inspection

The inspection package: it drives the robot through each room, detects and 3D-localizes every gauge, and
produces a per-room and facility report. It builds on top of the RTAB-Map SLAM + Nav2 + frontier-mapping
stack and does not modify it.

## Pipeline

`zone_inspector` is the inspection engine. For each room it:

1. samples safe interior viewpoints from the room polygon;
2. navigates to each viewpoint with Nav2 (with a reachability pre-check);
3. runs a 360° in-place spin with **live YOLOE** open-vocabulary detection;
4. projects each detection to a 3D map position through the depth camera, de-duplicates across the room,
   applies a persistence filter and an observation-aware consolidation, and crops the best view of each
   instrument.

An optional **detect-then-approach** mode drives close to each detected gauge for a high-resolution,
resolution-budgeted read. `inspection_mission` runs the full HOME → rooms → report → HOME loop, and
`benchmark.py` scores detection precision/recall and localization error against world ground truth.

## Reading and control (optional)

- `gauge_inspector` reads each gauge crop (type · unit · value · risk) into the report via the Anthropic
  API, and scores the readings against ground truth when available. Runtime: a small venv with `anthropic`.
- `mission_control_server` exposes the stack as ROS 2 service triggers with a structured event stream;
  `mcp_mission_server` / `mcp_gauge_server` provide an optional MCP surface for natural-language control.

Create the reading venv once:

```bash
uv venv ~/gauge_venv && uv pip install --python ~/gauge_venv/bin/python fastmcp anthropic opencv-python-headless
```

## Output (`~/gauges/<zone>/`)

Per zone: `objects.json` (localized detections), `detections.json` (raw observations), `crops/`, an
annotated `zone_map.png`, and `report.{md,csv}`. The mission aggregates these into a facility manifest,
map, and report. Reading columns: **ID, Zone, Type, Reading, Unit, SI_Unit, Range, Risk, Confidence**.

See the workspace [`RUN-SIM.md`](../../../RUN-SIM.md) for the full run sequence and the
[`docs/`](../../../docs/) ADRs for the pipeline design.
