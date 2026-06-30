"""benchmark.py — score an inspection run against world ground truth (ROS-free, CI-testable).

Detection quality is the dimension neither the `-main` engine nor our old wall-follower ever measured.
Given (a) ground-truth object world-positions parsed straight from the Gazebo world SDF and (b) the
per-zone `objects.json` the inspection writes, this computes **precision / recall / F1**, the **mean
localization error** of matched objects, and **coverage** (which ground-truth objects fell in a zone we
actually inspected). Matching is greedy nearest-neighbour, per canonical class, within `match_radius`.

Run:  python3 benchmark.py <world.sdf> <gauges_root=~/gauges> [match_radius_m=1.0]

Pure functions (`parse_world_gt`, `load_detected`, `score_detections`, `canon_class`) are unit-tested in
test/test_benchmark.py and gated by CI — so the scoring can't silently regress even though a full
detection run needs YOLOE weights + the CLIP backend at runtime.
"""
import glob
import json
import math
import os
import re
import sys

# Gazebo model-name prefix -> canonical object class. Extend as worlds gain object types (the
# inspection_arena adds fire/fumes/extinguisher/exit/person/cones/crates).
GT_PREFIX_CLASS = {
    "gauge": "gauge",
    "dial": "gauge",
    "fire": "fire",
    "fumes": "fumes",
    "extinguisher": "extinguisher",
    "exit": "exit",
    "person": "person",
    "human": "person",
    "cone": "cone",
    "crate": "crate",
}


def canon_class(name):
    """Canonicalize a detected/ground-truth class label so YOLOE prompt strings and SDF model names
    compare equal (e.g. 'white round analog gauge with a needle' and 'gauge_01' both -> 'gauge')."""
    n = (name or "").lower()
    if "gauge" in n or "dial" in n:
        return "gauge"
    for key, cls in GT_PREFIX_CLASS.items():
        if n.startswith(key) or key in n:
            return cls
    return n.strip()


def parse_world_gt(sdf_text_or_path):
    """Ground-truth objects from a Gazebo world SDF (accepts a path or the raw XML).
    Returns [{'class','x','y','name'}] for every model whose name maps to a known object class —
    walls/ground/robot are skipped, so only inspectable props count as ground truth."""
    text = sdf_text_or_path
    if "\n" not in sdf_text_or_path and os.path.exists(os.path.expanduser(sdf_text_or_path)):
        text = open(os.path.expanduser(sdf_text_or_path)).read()
    out = []
    for m in re.finditer(r'<model\s+name="([^"]+)"[^>]*>(.*?)</model>', text, re.S):
        name, body = m.group(1), m.group(2)
        cls = None
        for key, c in GT_PREFIX_CLASS.items():
            if name.lower().startswith(key):
                cls = c
                break
        if cls is None:
            continue
        pose = re.search(r"<pose>\s*([-\d.eE]+)\s+([-\d.eE]+)", body)
        if not pose:
            continue
        out.append({"class": cls, "x": float(pose.group(1)), "y": float(pose.group(2)), "name": name})
    return out


def load_detected(gauges_root="~/gauges"):
    """Localized detected objects across every <zone>/objects.json under gauges_root."""
    out = []
    root = os.path.expanduser(gauges_root)
    for oj in sorted(glob.glob(os.path.join(root, "*", "objects.json"))):
        try:
            d = json.load(open(oj))
        except Exception:
            continue
        zone = d.get("zone", os.path.basename(os.path.dirname(oj)))
        for o in d.get("objects", []) or []:
            w = o.get("world")
            if not w or len(w) < 2 or not o.get("localized", True):
                continue
            out.append(
                {"class": canon_class(o.get("class")), "x": float(w[0]), "y": float(w[1]), "zone": zone}
            )
    return out


def score_detections(ground_truth, detected, match_radius=1.0):
    """Greedy per-class nearest-neighbour matching of detected objects to ground truth.
    A detection matches the nearest still-unmatched GT object of the same canonical class within
    `match_radius` (metres); unmatched detections are false positives, unmatched GT are false negatives.
    Returns precision/recall/F1, TP/FP/FN counts, mean localization error, and per-class recall."""
    gt = [dict(g, _used=False) for g in ground_truth]
    tp = 0
    fp = 0
    matched_err = []
    for d in detected:
        dc = canon_class(d["class"])
        best = None
        best_dist = float("inf")
        for g in gt:
            if g["_used"] or canon_class(g["class"]) != dc:
                continue
            dist = math.hypot(g["x"] - d["x"], g["y"] - d["y"])
            if dist < best_dist:
                best_dist = dist
                best = g
        if best is not None and best_dist <= match_radius:
            best["_used"] = True
            tp += 1
            matched_err.append(best_dist)
        else:
            fp += 1
    fn = sum(1 for g in gt if not g["_used"])
    prec = tp / (tp + fp) if (tp + fp) else 0.0
    rec = tp / (tp + fn) if (tp + fn) else 0.0
    f1 = 2 * prec * rec / (prec + rec) if (prec + rec) else 0.0
    # per-class recall
    per_class = {}
    for g in gt:
        c = canon_class(g["class"])
        pc = per_class.setdefault(c, {"gt": 0, "found": 0})
        pc["gt"] += 1
        pc["found"] += 1 if g["_used"] else 0
    return {
        "precision": round(prec, 3),
        "recall": round(rec, 3),
        "f1": round(f1, 3),
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "n_ground_truth": len(gt),
        "n_detected": len(detected),
        "mean_loc_error_m": round(sum(matched_err) / len(matched_err), 3) if matched_err else None,
        "match_radius_m": match_radius,
        "per_class": {c: {**v, "recall": round(v["found"] / v["gt"], 3)} for c, v in per_class.items()},
    }


def format_report(world, gauges_root, result, gt):
    lines = [
        "# Inspection benchmark vs ground truth",
        f"world: `{world}`  ·  detections: `{gauges_root}`",
        "",
        f"- ground-truth objects: **{result['n_ground_truth']}**  ·  detected (localized): "
        f"**{result['n_detected']}**",
        f"- precision **{result['precision']}**  ·  recall **{result['recall']}**  ·  F1 "
        f"**{result['f1']}**   (TP {result['tp']} / FP {result['fp']} / FN {result['fn']})",
        f"- mean localization error: **{result['mean_loc_error_m']} m** (matched only, radius "
        f"{result['match_radius_m']} m)",
        "",
        "| class | ground truth | found | recall |",
        "|---|---|---|---|",
    ]
    for c, v in sorted(result["per_class"].items()):
        lines.append(f"| {c} | {v['gt']} | {v['found']} | {v['recall']} |")
    lines += ["", "Ground-truth objects (world xy):"]
    for g in gt:
        lines.append(f"- {g['name']}: {g['class']} @ ({g['x']:.2f}, {g['y']:.2f})")
    return "\n".join(lines)


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    world = argv[0]
    gauges_root = argv[1] if len(argv) > 1 else "~/gauges"
    radius = float(argv[2]) if len(argv) > 2 else 1.0
    gt = parse_world_gt(world)
    det = load_detected(gauges_root)
    result = score_detections(gt, det, radius)
    report = format_report(world, gauges_root, result, gt)
    print(report)
    out = os.path.join(os.path.expanduser(gauges_root), "benchmark.md")
    try:
        open(out, "w").write(report + "\n")
        json.dump(result, open(os.path.splitext(out)[0] + ".json", "w"), indent=2)
        print(f"\nwrote {out}")
    except Exception as e:  # noqa: BLE001
        print(f"(could not write report: {e})")
    return 0


def _cli():  # console_scripts entry: `ros2 run go2_inspection benchmark <world.sdf> <gauges_root> [r]`
    raise SystemExit(main(sys.argv[1:]))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
