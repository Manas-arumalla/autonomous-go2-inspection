#!/usr/bin/env python3
"""report_utils -- map plotting + report writing for the inspection pipeline.

Shared by zone_inspector (per-zone outputs) and inspection_mission (facility rollup). Pure helpers:
load the saved nav2 map (.pgm + .yaml), convert world<->pixel, draw detected objects on the map, and
emit human-readable report.md / report.csv. No ROS deps so it works in either process.
"""

import os, csv
import numpy as np
import cv2

from go2_inspection.detect_utils import color_for

DEFAULT_MAP_YAML = os.path.expanduser("~/.go2_maps/facility_inspection_map.yaml")


def _parse_map_yaml(path):
    """Minimal parser for the nav2 map yaml (image/resolution/origin) -> dict, avoids a yaml dependency."""
    import json

    d = {}
    for line in open(path):
        line = line.split("#", 1)[0].strip()
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        d[k.strip()] = v.strip()
    res = float(d["resolution"])
    origin = (
        json.loads(d["origin"])
        if d["origin"].startswith("[")
        else [float(x) for x in d["origin"].split()]
    )
    img_name = d["image"]
    return img_name, res, float(origin[0]), float(origin[1])


def load_map(map_yaml=None):
    """Return (bgr_image, res, ox, oy, H, W). bgr_image has row 0 = top (max y), per pgm convention."""
    map_yaml = os.path.expanduser(map_yaml or DEFAULT_MAP_YAML)
    img_name, res, ox, oy = _parse_map_yaml(map_yaml)
    pgm = os.path.join(os.path.dirname(map_yaml), img_name)
    gray = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise FileNotFoundError(f"map image not found: {pgm}")
    bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    H, W = gray.shape
    return bgr, res, ox, oy, H, W


def load_occupancy(map_yaml=None):
    """Return (gray, res, ox, oy, H, W) for the saved nav2 map grayscale image (row 0 = top = max y).
    pgm convention (map_saver, negate 0): free=254, occupied=0, unknown=205. Used by zone_inspector to
    sanity-check that a detected object's projected position lies near a mapped obstacle/unknown cell."""
    map_yaml = os.path.expanduser(map_yaml or DEFAULT_MAP_YAML)
    img_name, res, ox, oy = _parse_map_yaml(map_yaml)
    pgm = os.path.join(os.path.dirname(map_yaml), img_name)
    gray = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise FileNotFoundError(f"map image not found: {pgm}")
    H, W = gray.shape
    return gray, res, ox, oy, H, W


def world_to_px(x, y, res, ox, oy, H):
    """World (m) -> pixel (col, row) in the pgm image (row 0 = top = max y)."""
    px = int(round((x - ox) / res))
    py = int(round(H - 1 - (y - oy) / res))
    return px, py


def _draw_objects(img, objects, res, ox, oy, H, label=True):
    for o in objects:
        w = o.get("world")
        if not w or w[0] is None:
            continue
        px, py = world_to_px(w[0], w[1], res, ox, oy, H)
        col = color_for(o["class"])
        cv2.circle(img, (px, py), 6, col, -1)
        cv2.circle(img, (px, py), 6, (30, 30, 30), 1)
        if label:
            cv2.putText(
                img,
                o["class"],
                (px + 7, py + 3),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                col,
                1,
                cv2.LINE_AA,
            )


def _draw_polygon(img, polygon, res, ox, oy, H, col=(255, 120, 0)):
    if not polygon:
        return
    pts = np.array([world_to_px(x, y, res, ox, oy, H) for x, y in polygon], np.int32)
    cv2.polylines(img, [pts], True, col, 2)


def plot_zone_map(zone_id, polygon, viewpoints, objects, out_path, map_yaml=None):
    """Annotated map for ONE zone: zone polygon (orange), viewpoints (magenta x), objects (class-colored)."""
    img, res, ox, oy, H, W = load_map(map_yaml)
    _draw_polygon(img, polygon, res, ox, oy, H)
    for x, y in viewpoints or []:
        px, py = world_to_px(x, y, res, ox, oy, H)
        cv2.drawMarker(img, (px, py), (255, 0, 255), cv2.MARKER_CROSS, 12, 2)
    _draw_objects(img, objects, res, ox, oy, H)
    cv2.putText(
        img,
        f"{zone_id}: {len(objects)} objects",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 200),
        2,
        cv2.LINE_AA,
    )
    cv2.imwrite(out_path, img)
    return out_path


def plot_facility_map(zones_objects, out_path, zone_polys=None, map_yaml=None):
    """Facility rollup: every zone's polygon + all objects on the full map. zones_objects: {zone: [objs]}."""
    img, res, ox, oy, H, W = load_map(map_yaml)
    total = 0
    for zid, objs in zones_objects.items():
        if zone_polys and zid in zone_polys:
            _draw_polygon(img, zone_polys[zid], res, ox, oy, H)
        _draw_objects(img, objs, res, ox, oy, H, label=False)
        total += len(objs)
    cv2.putText(
        img,
        f"facility: {total} objects across {len(zones_objects)} zones",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 200),
        2,
        cv2.LINE_AA,
    )
    cv2.imwrite(out_path, img)
    return out_path


def write_zone_report(zone_id, label, objects, out_dir):
    """Write report.md + report.csv for one zone. objects: dicts with id/class/confidence/world/n_observations."""
    md = os.path.join(out_dir, "report.md")
    csvp = os.path.join(out_dir, "report.csv")
    with open(md, "w") as f:
        f.write(f"# Inspection report -- {zone_id} ({label})\n\n")
        f.write(f"**{len(objects)} unique object(s) detected.**\n\n")
        f.write("| # | class | confidence | world (x, y, z) | observations | localized | crop |\n")
        f.write("|---|-------|-----------|------------------|--------------|-----------|------|\n")
        for i, o in enumerate(objects):
            w = o.get("world")
            ws = f"({w[0]:.2f}, {w[1]:.2f}, {w[2]:.2f})" if w and w[0] is not None else "n/a"
            f.write(
                f"| {i} | {o['class']} | {o['confidence']:.2f} | {ws} | "
                f"{o.get('n_observations', 1)} | {o.get('localized', False)} | {o.get('crop', '')} |\n"
            )
    with open(csvp, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(
            [
                "id",
                "zone",
                "class",
                "confidence",
                "x",
                "y",
                "z",
                "n_observations",
                "localized",
                "crop",
            ]
        )
        for o in objects:
            w = o.get("world") or [None, None, None]
            wr.writerow(
                [
                    o.get("id"),
                    zone_id,
                    o["class"],
                    round(o["confidence"], 3),
                    w[0],
                    w[1],
                    w[2],
                    o.get("n_observations", 1),
                    o.get("localized", False),
                    o.get("crop", ""),
                ]
            )
    return md, csvp
