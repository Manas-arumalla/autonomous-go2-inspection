#!/usr/bin/env python3
"""gauge_inspector -- Phase 4 of the autonomous gauge inspection: read each cropped gauge via the
Anthropic API and write the inspection-report CSV.

INPUT  (from panorama_segmenter, Phase 3):  ~/gauges/<zone>/gauges.json + gauges/gauge_NN.png
OUTPUT: ~/gauges/<zone>/inspection_report.csv  (+ readings.json with the full reasoning)
        columns: ID, Zone, Type, Reading, Unit, SI_Unit, Range, Risk, Confidence

WHY a REASONING-FIRST prompt (not "what's the value?"): VLMs are unreliable at reading an
analog VALUE in one shot (MeasureBench: unit ~95% right but value ~15-31%). The crop is clean + full
(Phase 3), and the prompt forces the model to reason in the order a human does -- find the numbered ticks,
the unit, which two ticks the NEEDLE TIP lies between, THEN interpolate -- and to judge risk from the RED
danger arc it can see. It also collects a confidence so a low-confidence reading can be flagged for
review. No ground-truth is given to the model (it reads the range off the dial itself) -> ports to the
real robot.

This is the robust, scriptable, ON-DEVICE/real-time path (one API call per gauge, POST-sweep, off the
locomotion loop). An alternate MCP transport (an MCP client via MCP) is mcp_gauge_server.py,
which reuses READ_TOOL + the prompt below. claude-opus-4-8 for accuracy (post-sweep, not in-loop);
claude-haiku-4-5 if latency matters more than a few % accuracy.
"""
import os, json, base64, csv, argparse

SYSTEM = (
    "You are an industrial inspection vision expert reading ANALOG dial gauges from a robot's camera. "
    "Accuracy matters: a misread gauge can hide a dangerous condition. Reason step by step before you "
    "commit to a value, and never guess a precise number you cannot justify from the visible ticks."
)

# Reasoning-first instructions. The model must work through these BEFORE filling the structured tool.
PROMPT = (
    "Read this analog gauge. Work through it IN THIS ORDER, then call report_gauge:\n"
    "1. TYPE: read the printed label (PRESSURE / VOLTAGE / TEMPERATURE / CURRENT / FLOW / other).\n"
    "2. UNIT: read the printed unit (psi, bar, kPa, V, degC, A, ...). Give the SI unit of that quantity "
    "(pressure->Pa, temperature->K, voltage->V, current->A).\n"
    "3. SCALE: read the smallest and largest NUMBERED ticks = the range. Note the value of one minor tick.\n"
    "4. NEEDLE: find the needle TIP. Identify the two numbered ticks it lies between and interpolate its "
    "value. Sanity-check it is within the range.\n"
    "5. RISK: look for a RED/coloured danger arc. If the needle is inside it -> CRITICAL; if just below it "
    "or in the top ~10% of range -> WARNING; otherwise OK. Say why.\n"
    "6. CONFIDENCE: 0-1, lower if the dial is blurry, angled, glared, or the needle is between ticks.\n"
    "If this is NOT a gauge (a vent, sign, fixture, etc.), set type to 'NOT_A_GAUGE' and confidence high."
)

READ_TOOL = {
    "name": "report_gauge",
    "description": "Report the structured reading of one analog gauge after reasoning through it.",
    "input_schema": {
        "type": "object",
        "properties": {
            "reasoning": {"type": "string", "description": "Your step 1-6 reasoning, briefly."},
            "type": {"type": "string", "description": "PRESSURE/VOLTAGE/TEMPERATURE/CURRENT/FLOW/NOT_A_GAUGE/other"},
            "unit": {"type": "string", "description": "the unit printed on the dial, e.g. psi, bar, V, degC, A"},
            "si_unit": {"type": "string", "description": "SI unit of the quantity: Pa, K, V, A"},
            "reading": {"type": "number", "description": "needle value in the dial's printed unit"},
            "range_min": {"type": "number"},
            "range_max": {"type": "number"},
            "risk": {"type": "string", "enum": ["OK", "WARNING", "CRITICAL", "UNKNOWN"]},
            "risk_reason": {"type": "string"},
            "confidence": {"type": "number", "description": "0..1"},
        },
        "required": ["type", "unit", "si_unit", "reading", "range_min", "range_max",
                     "risk", "confidence", "reasoning"],
    },
}


def read_one(client, crop_path, model):
    """One API call -> structured reading dict (raises on API error)."""
    with open(crop_path, "rb") as f:
        b64 = base64.standard_b64encode(f.read()).decode()
    msg = client.messages.create(
        model=model, max_tokens=1024, system=SYSTEM,
        tools=[READ_TOOL], tool_choice={"type": "tool", "name": "report_gauge"},
        messages=[{"role": "user", "content": [
            {"type": "image", "source": {"type": "base64", "media_type": "image/png", "data": b64}},
            {"type": "text", "text": PROMPT},
        ]}],
    )
    for block in msg.content:
        if block.type == "tool_use" and block.name == "report_gauge":
            return block.input
    raise RuntimeError("model did not call report_gauge")


def rows_to_csv(zone, readings, out_csv):
    cols = ["ID", "Zone", "Type", "Reading", "Unit", "SI_Unit", "Range", "Risk", "Confidence"]
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f); w.writerow(cols)
        for g, r in readings:
            w.writerow([g["id"], zone, r.get("type", "?"), r.get("reading", ""),
                        r.get("unit", ""), r.get("si_unit", ""),
                        f"{r.get('range_min','?')}-{r.get('range_max','?')} {r.get('unit','')}".strip(),
                        r.get("risk", "UNKNOWN"), round(float(r.get("confidence", 0)), 2)])
    return out_csv


def score(readings, gt_path):
    """Score readings vs a ground-truth json (matched by left->right lateral order)."""
    gt = json.load(open(gt_path))
    ordered = sorted(readings, key=lambda gr: gr[0].get("lateral", 0.0))   # tolerant: new segmenter or old
    print(f"\n  {'ID':<12}{'type ok':<9}{'unit ok':<9}{'read':>8}{'true':>8}{'err%':>7}")
    n_type = n_unit = n_val = 0
    for (g, r), t in zip(ordered, gt):
        type_ok = r.get("type", "").upper() == t["type"].upper()
        unit_ok = r.get("unit", "").lower().replace("°", "") == t["unit"].lower().replace("°", "")
        rd, tv = float(r.get("reading", 0)), float(t["true_reading"])
        span = float(t["range_max"]) - float(t["range_min"]) or 1.0
        err = abs(rd - tv) / span * 100.0
        val_ok = err <= 5.0
        n_type += type_ok; n_unit += unit_ok; n_val += val_ok
        print(f"  {g['id']:<12}{str(type_ok):<9}{str(unit_ok):<9}{rd:>8.1f}{tv:>8.1f}{err:>6.1f}%")
    n = len(gt)
    print(f"\n  type {n_type}/{n}  unit {n_unit}/{n}  value(<=5% span) {n_val}/{n}")


def inspect(zone_dir, model="claude-opus-4-8", gt_path=None):
    import anthropic
    zone_dir = os.path.expanduser(zone_dir)
    meta = json.load(open(os.path.join(zone_dir, "gauges.json")))
    zone = meta["zone"]
    client = anthropic.Anthropic()
    readings = []
    for g in meta["gauges"]:
        r = read_one(client, os.path.join(zone_dir, g["file"]), model)
        readings.append((g, r))
        print(f"  {g['id']}: {r['type']} {r['reading']}{r['unit']} "
              f"[{r['range_min']}-{r['range_max']}] {r['risk']} conf={r['confidence']}")
    out_csv = os.path.join(zone_dir, "inspection_report.csv")
    rows_to_csv(zone, readings, out_csv)
    json.dump([{"id": g["id"], **r} for g, r in readings],
              open(os.path.join(zone_dir, "readings.json"), "w"), indent=2)
    print(f"\nwrote {out_csv}")
    if gt_path and os.path.exists(os.path.expanduser(gt_path)):
        score(readings, os.path.expanduser(gt_path))
    return readings


def main():
    ap = argparse.ArgumentParser(description="Reasoning-first gauge reading -> CSV (Phase 4).")
    ap.add_argument("zone_dir", nargs="?", default="~/gauges/zone_1")
    ap.add_argument("--model", default="claude-opus-4-8")
    ap.add_argument("--groundtruth", default=None, help="optional gauges_groundtruth.json to score against")
    a = ap.parse_args()
    inspect(a.zone_dir, model=a.model, gt_path=a.groundtruth)


if __name__ == "__main__":
    main()
