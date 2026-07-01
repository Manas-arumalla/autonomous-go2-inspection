"""inspect_planner.py — resolution-driven inspection-pose planning (ROS-free, CI-testable).

The 'approach' half of **detect-then-approach** (ADR-017). A 360° spin from an interior viewpoint can
DETECT a wall-mounted gauge at range, but READING the dial needs a close, fronto-parallel, high-res view —
and the max distance at which a dial is still readable is a hard function of the camera and a pixel budget.
This module, given a detected asset's world position + the occupancy map + the camera model, computes a
**close, reachable, fronto-parallel pose to read it from**. The standoff distance is *derived from the
resolution requirement* (not fixed), which is exactly what makes reading work at any room size — the same
principle Spot / ANYmal use (navigate-close + frame the asset) rather than reading from a wide spin.

All functions are pure geometry over numpy occupancy grids, unit-tested in test/test_inspect_planner.py
and gated by CI. Occupancy convention matches report_utils.load_occupancy: free=254, occupied=0,
unknown=205, origin (ox,oy) at the grid's bottom-left, row 0 = top.
"""
import math

import numpy as np


def standoff_distance(fx_px, asset_size_m, target_px, dmin=0.45, dmax=1.2):
    """Distance at which an asset of physical size `asset_size_m` (m) spans `target_px` pixels for a camera
    of focal length `fx_px` (pixels):  d = fx * size / px.  Clamped to [dmin, dmax] — closer risks FOV
    clipping + Nav2 inflation, farther is unreadable. THIS is the knob that makes reading scale-invariant:
    bigger room ⇒ farther detection, but the read pose is always pulled back to this readable distance."""
    if fx_px <= 0 or target_px <= 0 or asset_size_m <= 0:
        return dmin
    return max(dmin, min(dmax, fx_px * asset_size_m / target_px))


def _world_to_px(x, y, res, ox, oy, H):
    return int((x - ox) / res), int(H - 1 - (y - oy) / res)


def _px_to_world(cx, cy, res, ox, oy, H):
    return ox + (cx + 0.5) * res, oy + (H - 1 - cy + 0.5) * res


def wall_normal(occ, world_xy, res, ox, oy, win_m=0.6, free_at_least=250):
    """Outward wall normal at a wall-mounted asset. Samples occupancy in a `win_m` window around the asset;
    the normal points from the centroid of nearby NON-FREE (wall) cells toward free space → a unit
    (nx, ny). Falls back to pointing toward the map's free centroid when no wall is nearby (free-standing
    asset). `free_at_least`: cells with value >= this are free (default matches load_occupancy's 254)."""
    H, W = occ.shape
    cx, cy = _world_to_px(world_xy[0], world_xy[1], res, ox, oy, H)
    r = max(1, int(win_m / res))
    x0, x1 = max(0, cx - r), min(W, cx + r + 1)
    y0, y1 = max(0, cy - r), min(H, cy + r + 1)
    sub = occ[y0:y1, x0:x1]
    ys, xs = np.where(sub < free_at_least)  # wall / unknown cells in the window
    if len(xs) == 0:
        fy, fx_ = np.where(occ >= free_at_least)  # fallback: toward overall free centroid
        if len(fx_) == 0:
            return (1.0, 0.0)
        tx, ty = _px_to_world(float(fx_.mean()), float(fy.mean()), res, ox, oy, H)
        v = (tx - world_xy[0], ty - world_xy[1])
    else:
        wxw, wyw = _px_to_world(xs.mean() + x0, ys.mean() + y0, res, ox, oy, H)
        v = (world_xy[0] - wxw, world_xy[1] - wyw)  # away from the wall centroid
    n = math.hypot(v[0], v[1])
    return (v[0] / n, v[1] / n) if n > 1e-6 else (1.0, 0.0)


def inspection_pose(asset_xy, normal, d):
    """Standoff pose (x, y, yaw) at distance d along `normal` from the asset, yaw facing back at it."""
    px = asset_xy[0] + normal[0] * d
    py = asset_xy[1] + normal[1] * d
    yaw = math.atan2(asset_xy[1] - py, asset_xy[0] - px)
    return (px, py, yaw)


def plan_reading_poses(asset_xy, normal, d, is_free, arc_deg=60.0, step_deg=15.0, max_poses=4):
    """RANKED list of reachable reading poses: the wall-normal standoff first, then the nearest-angle
    alternates (rotating the standoff direction around the asset within ±arc_deg). Used for next-best-view
    re-approach — if the first pose yields a poor read (glare/occlusion/too small), the engine tries the
    next. Returns up to `max_poses` (x, y, yaw); [] if nothing in the arc is free. `is_free(x, y) -> bool`
    tests the costmap/occupancy (free + footprint clearance)."""
    base = math.atan2(normal[1], normal[0])
    offsets = [0.0]
    a = step_deg
    while a <= arc_deg + 1e-6:
        offsets += [a, -a]
        a += step_deg
    out = []
    for off in offsets:
        ang = base + math.radians(off)
        n = (math.cos(ang), math.sin(ang))
        x, y, yaw = inspection_pose(asset_xy, n, d)
        if is_free(x, y):
            out.append((x, y, yaw))
            if len(out) >= max_poses:
                break
    return out


def plan_reading_pose(asset_xy, normal, d, is_free, arc_deg=60.0, step_deg=15.0):
    """The single best reachable reading pose (the wall-normal standoff, or nearest free angle). Returns
    (x, y, yaw) or None. Thin wrapper over plan_reading_poses for callers that want just the first."""
    poses = plan_reading_poses(asset_xy, normal, d, is_free, arc_deg, step_deg, max_poses=1)
    return poses[0] if poses else None


def read_quality(gauge_px, frame_w, sharpness_val, target_px, sharp_min=20.0):
    """Score a close read crop and decide if it's good enough to stop re-approaching. `gauge_px` = detected
    gauge width in px (0 if not detected), `frame_w` = image width, `sharpness_val` = variance-of-Laplacian.
    Returns (score, ok): score ranks attempts (bigger, sharper = better) so the best crop is kept; ok is
    True when the gauge is detected at >= target_px and the frame is sharp enough (stop early)."""
    if gauge_px <= 0 or frame_w <= 0:
        return (0.0, False)
    size_ratio = gauge_px / float(target_px)
    score = gauge_px * (1.0 + min(sharpness_val, 500.0) / 500.0)  # size dominates, sharpness breaks ties
    ok = size_ratio >= 0.85 and sharpness_val >= sharp_min
    return (score, ok)


def make_is_free(plausible_mask, res, ox, oy):
    """Build an is_free(x, y) closure from a dilated obstacle mask (0 = free+clearance, nonzero = obstacle
    within the clearance radius) — i.e. zone_inspector's self._plausible. Out-of-bounds = not free."""
    H, W = plausible_mask.shape

    def is_free(x, y):
        cx, cy = _world_to_px(x, y, res, ox, oy, H)
        if not (0 <= cx < W and 0 <= cy < H):
            return False
        return bool(plausible_mask[cy, cx] == 0)

    return is_free


def sharpness(gray):
    """Variance-of-Laplacian focus measure (higher = sharper) — used to pick the sharpest frame of an
    approach capture burst so a motion-blurred frame never reaches the reader."""
    import cv2

    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def weak_duplicate_map(objects, radius, obs_frac):
    """Map {weak_index: strong_index} of localization-noise duplicates to consolidate. A detection is a
    WEAK duplicate of a stronger same-class one when it sits within `radius` AND was seen in <= obs_frac of
    the stronger's frames. (The looser depth gate that fixes recall can localize a few sparse-depth frames
    of one gauge ~1 m off — a weak outlier — so a single object splits into a strong detection plus a weak
    ghost.) Two comparably-well-seen DISTINCT objects (observation ratio > obs_frac) are never paired, so
    recall is preserved; the radius is the primary same-object prior (facility gauges are metres apart).
    Pure: objects = [{"world": [x, y, ...], "class": str, "n_observations": int, "localized": bool}, ...].
    A strong object is never itself absorbed (strongest-first), so the map has no chains."""
    idx = [
        i
        for i, o in enumerate(objects)
        if o.get("localized") and o.get("world") and o["world"][0] is not None
    ]
    idx.sort(key=lambda i: objects[i]["n_observations"], reverse=True)  # strongest first
    absorbed = {}
    for si in idx:
        if si in absorbed:
            continue
        s = objects[si]
        for wi in idx:
            if wi == si or wi in absorbed:
                continue
            w = objects[wi]
            if w.get("class") != s.get("class"):
                continue
            if w["n_observations"] > obs_frac * s["n_observations"]:
                continue  # comparably well-seen -> a distinct object, keep it
            if math.hypot(w["world"][0] - s["world"][0], w["world"][1] - s["world"][1]) <= radius:
                absorbed[wi] = si
    return absorbed
