"""Unit tests for the door-aware zone segmenter (pure numpy/cv2 — no ROS).

Builds a synthetic occupancy grid and asserts the watershed segmentation behaves: two rooms separated by a
wall with a doorway -> two zones; a free-standing obstacle island is reclaimed; outputs carry the fields the
inspection engine consumes (id / center / polygon / area / nav_point / label).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))  # make `go2_zones` importable standalone

import numpy as np  # noqa: E402

from go2_zones.zone_segmenter import segment_grid, _island_fill, _quadrant_label  # noqa: E402

RES = 0.1  # m/px


def _two_room_grid():
    """A 10x10 m building split into two rooms by a wall with a ~1 m doorway.
    Grid convention: -1 unknown (outside), 0 free, 100 occupied (walls)."""
    H = W = 120
    g = np.full((H, W), -1, np.int16)
    g[10:110, 10:110] = 0  # free interior (100x100 px = 10x10 m)
    g[10:110, 10] = 100
    g[10:110, 109] = 100  # left / right perimeter walls
    g[10, 10:110] = 100
    g[109, 10:110] = 100  # top / bottom perimeter walls
    g[10:110, 60] = 100  # dividing wall down the middle
    g[55:65, 60] = 0  # a ~1 m doorway through it
    return g


def test_segments_two_rooms():
    zones, _ = segment_grid(_two_room_grid(), RES, 0.0, 0.0)
    assert len(zones) == 2, f"expected 2 rooms, got {len(zones)}"
    for z in zones:
        assert z["area"] > 6.0  # each ~ 4.8 x 9.8 m
        assert len(z["polygon"]) >= 3
        assert z["id"].startswith("zone_")
        for key in ("center", "nav_point", "label", "area"):
            assert key in z
        # the nav_point must land on an ORIGINALLY-free cell (a reachable goal, not a wall)
        nx, ny = z["nav_point"]
        px, py = int(round(nx / RES)), int(round(ny / RES))
        assert _two_room_grid()[py, px] == 0, "nav_point must be in free space"


def test_zones_sorted_largest_first_with_stable_ids():
    zones, _ = segment_grid(_two_room_grid(), RES, 0.0, 0.0)
    areas = [z["area"] for z in zones]
    assert areas == sorted(areas, reverse=True)
    assert [z["id"] for z in zones] == [f"zone_{i}" for i in range(len(zones))]


def test_island_fill_reclaims_freestanding_obstacle():
    g = _two_room_grid()
    g[40:46, 30:36] = 100  # a small free-standing obstacle island in the left room
    free_clean, _ = _island_fill(g, RES)
    assert free_clean[42, 32] == 1, "a free-standing obstacle island should be reclaimed as free"


def test_quadrant_label_is_a_nonempty_tag():
    lab = _quadrant_label(2.0, 9.0, 0.0, 0.0, 120, 120, RES)
    assert isinstance(lab, str) and lab
