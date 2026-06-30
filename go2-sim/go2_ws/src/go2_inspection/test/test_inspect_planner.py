"""Unit tests for the resolution-driven inspection-pose planner (pure geometry — no ROS).

Covers the standoff-distance formula + clamping, wall-normal estimation from occupancy, the standoff pose,
and the reachability arc-search that avoids stranding when the ideal pose is blocked.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))  # make `go2_inspection` importable

from go2_inspection.inspect_planner import (  # noqa: E402
    standoff_distance,
    wall_normal,
    inspection_pose,
    plan_reading_pose,
    make_is_free,
)


def test_standoff_distance_formula_and_clamp():
    # fx=381, 0.26 m gauge needing 120 px -> ~0.825 m (in range)
    assert abs(standoff_distance(381, 0.26, 120) - 0.8255) < 0.01
    # asking for too many px -> too close -> clamped up to dmin
    assert standoff_distance(381, 0.26, 1000, dmin=0.45) == 0.45
    # asking for few px -> very far -> clamped down to dmax
    assert standoff_distance(381, 0.26, 10, dmax=1.2) == 1.2
    assert standoff_distance(0, 0.26, 120) == 0.45  # degenerate fx -> dmin


def test_wall_normal_points_away_from_wall():
    occ = np.full((40, 40), 254, np.int16)  # all free
    occ[:, 0:3] = 0  # a wall along the left edge (world x ~ 0..0.25)
    n = wall_normal(occ, (0.55, 2.05), res=0.1, ox=0.0, oy=0.0, win_m=0.6)
    assert n[0] > 0.85 and abs(n[1]) < 0.4  # points +x (away from the -x wall), into free space


def test_inspection_pose_distance_and_facing():
    x, y, yaw = inspection_pose((5.0, 2.0), (1.0, 0.0), 0.8)
    assert abs(x - 5.8) < 1e-6 and abs(y - 2.0) < 1e-6
    assert abs(abs(yaw) - math.pi) < 1e-6  # standing at +x, facing back (-x) -> yaw = pi


def test_plan_reading_pose_ideal_free():
    pose = plan_reading_pose((5.0, 2.0), (1.0, 0.0), 0.8, is_free=lambda x, y: True)
    assert pose is not None and abs(pose[0] - 5.8) < 1e-6


def test_plan_reading_pose_arc_fallback_when_ideal_blocked():
    # ideal pose (5.8, 2.0) blocked; only poses with y noticeably different are free
    def is_free(x, y):
        return abs(y - 2.0) > 0.15  # the straight-ahead standoff (y=2.0) is blocked
    pose = plan_reading_pose((5.0, 2.0), (1.0, 0.0), 0.8, is_free, arc_deg=60, step_deg=15)
    assert pose is not None and abs(pose[1] - 2.0) > 0.1  # found an angled, free pose


def test_plan_reading_pose_none_when_all_blocked():
    assert plan_reading_pose((5.0, 2.0), (1.0, 0.0), 0.8, is_free=lambda x, y: False) is None


def test_make_is_free_reads_mask():
    mask = np.zeros((20, 20), np.uint8)  # all free
    mask[10, 5] = 1  # one blocked cell
    is_free = make_is_free(mask, res=0.1, ox=0.0, oy=0.0)
    # cell (cx=5, cy=10) blocked: x in [0.5,0.6) -> 0.55 ; cy=10 needs y in (0.8,0.9] -> 0.85
    assert is_free(0.55, 0.85) is False
    assert is_free(0.25, 0.85) is True  # a free cell, same row
    assert is_free(-5.0, 0.0) is False  # out of bounds
