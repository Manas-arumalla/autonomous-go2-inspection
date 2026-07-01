#!/usr/bin/env python3
"""mcp_gauge_server -- Phase 4 (MCP transport): a FastMCP server that yields the segmented
gauge crops to an MCP client, which reads them and writes the report.

This is the "MCP server yielding images" transport path. The robust, scriptable ON-DEVICE path is
gauge_inspector.py (direct Anthropic API); both use the SAME reasoning-first reading instructions
(kept in sync below). Run it as a generic MCP stdio server and register it with your MCP client:

    GAUGES_ROOT=~/gauges  ~/gauge_venv/bin/python mcp_gauge_server.py          # stdio
    # your MCP client config -> mcpServers: { "go2-gauges": { "command": ".../python",
    #   "args": [".../mcp_gauge_server.py"], "env": {"GAUGES_ROOT": "/home/.../gauges"} } }

Tools:
  list_inspection_zones()            -> [{zone, n_gauges}]
  get_zone_gauges(zone)              -> reading instructions + each gauge crop (Image) + its id/path
  get_gauge_image(zone, gauge_id)    -> one crop (Image)            (per-gauge fallback)
  save_inspection_report(zone, rows) -> writes inspection_report.csv from the model's readings

MCP-image-bug mitigation (image RETURN is flaky in Desktop/Code): every crop is also given by absolute
FILE PATH in text, and get_zone_gauge_paths() returns paths only -- so the client can fall back to
reading the files directly if inline images fail. Crops are small PNGs already.
"""
import os, json, glob, csv

from fastmcp import FastMCP
from fastmcp.utilities.types import Image

GAUGES_ROOT = os.path.expanduser(os.environ.get("GAUGES_ROOT", "~/gauges"))
mcp = FastMCP("go2-gauge-inspector")

# Reasoning-first instructions -- MUST stay in sync with gauge_inspector.PROMPT.
READING_INSTRUCTIONS = (
    "You are reading industrial ANALOG dial gauges from a robot inspection sweep. For EACH gauge image, "
    "reason in this order before recording it:\n"
    "1. TYPE from the printed label (PRESSURE/VOLTAGE/TEMPERATURE/CURRENT/...).\n"
    "2. UNIT printed on the dial (psi/bar/V/degC/A) + the SI unit of that quantity (Pa/K/V/A).\n"
    "3. RANGE = smallest..largest numbered tick.\n"
    "4. VALUE: find the needle TIP, the two numbered ticks it lies between, interpolate.\n"
    "5. RISK: needle inside a RED danger arc -> CRITICAL; just below / top ~10% -> WARNING; else OK.\n"
    "6. CONFIDENCE 0-1 (lower if blurry/angled/glared).\n"
    "Then call save_inspection_report(zone, rows) where each row is "
    "{id, type, reading, unit, si_unit, range, risk, confidence}. Skip anything that is NOT a gauge."
)


def _zone_dir(zone):
    return os.path.join(GAUGES_ROOT, zone)


@mcp.tool
def list_inspection_zones() -> list:
    """List swept zones that have segmented gauges ready to read."""
    out = []
    for d in sorted(glob.glob(os.path.join(GAUGES_ROOT, "*"))):
        gj = os.path.join(d, "gauges.json")
        if os.path.isfile(gj):
            m = json.load(open(gj))
            out.append({"zone": m.get("zone", os.path.basename(d)), "n_gauges": m.get("n_gauges", 0)})
    return out


@mcp.tool
def get_zone_gauges(zone: str):
    """Return the reading instructions followed by every gauge crop (image + id + file path) for a zone.
    The model should read each image and then call save_inspection_report. (No return annotation: this
    tool yields mixed text+image CONTENT blocks, not a structured object.)"""
    d = _zone_dir(zone)
    m = json.load(open(os.path.join(d, "gauges.json")))
    content = [READING_INSTRUCTIONS, f"Zone '{zone}' has {m['n_gauges']} gauges:"]
    for g in m["gauges"]:
        p = os.path.join(d, g["file"])
        content.append(f"--- {g['id']}  (lateral {g['lateral']} m)  file: {p} ---")
        content.append(Image(path=p))
    return content


@mcp.tool
def get_zone_gauge_paths(zone: str) -> list:
    """Fallback when inline images fail in the client: the absolute path of every gauge crop."""
    d = _zone_dir(zone)
    m = json.load(open(os.path.join(d, "gauges.json")))
    return [{"id": g["id"], "lateral": g["lateral"], "path": os.path.join(d, g["file"])}
            for g in m["gauges"]]


@mcp.tool
def get_gauge_image(zone: str, gauge_id: str) -> Image:
    """Return a single gauge crop by id (per-gauge fallback)."""
    d = _zone_dir(zone)
    m = json.load(open(os.path.join(d, "gauges.json")))
    g = next(x for x in m["gauges"] if x["id"] == gauge_id)
    return Image(path=os.path.join(d, g["file"]))


@mcp.tool
def save_inspection_report(zone: str, rows: list) -> str:
    """Write inspection_report.csv from the readings the model produced.
    Each row: {id, type, reading, unit, si_unit, range, risk, confidence}."""
    d = _zone_dir(zone)
    out = os.path.join(d, "inspection_report.csv")
    cols = ["ID", "Zone", "Type", "Reading", "Unit", "SI_Unit", "Range", "Risk", "Confidence"]
    with open(out, "w", newline="") as f:
        w = csv.writer(f); w.writerow(cols)
        for r in rows:
            w.writerow([r.get("id"), zone, r.get("type"), r.get("reading"), r.get("unit"),
                        r.get("si_unit"), r.get("range"), r.get("risk"), r.get("confidence")])
    return f"wrote {out} ({len(rows)} gauges)"


if __name__ == "__main__":
    mcp.run()
