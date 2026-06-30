"""Unit tests for the ground-truth benchmark scoring (pure — no ROS, no torch).

Verifies the detection/localization metrics that make the converged stack measurable: SDF ground-truth
parsing, canonical-class matching, and precision/recall/F1 + localization error under perfect, missed,
and false-positive conditions.
"""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))  # make `go2_inspection` importable

from go2_inspection.benchmark import (  # noqa: E402
    canon_class,
    parse_world_gt,
    load_detected,
    score_detections,
)

# a tiny world: two gauges + one wall/ground that must NOT count as ground truth
_SDF = """
<sdf version="1.9">
  <world name="t">
    <model name="ground_plane"><pose>0 0 0 0 0 0</pose></model>
    <model name="maze"><pose>0 4 0.8 0 0 0</pose></model>
    <model name="gauge_01"><static>true</static><pose>5.85 2.0 0.30 0 0 1.5708</pose></model>
    <model name="gauge_02"><static>true</static><pose>-3.0 -3.85 0.30 0 0 0</pose></model>
  </world>
</sdf>
"""


def test_canon_class_groups_gauges():
    assert canon_class("white round analog gauge with a needle") == "gauge"
    assert canon_class("circular pressure dial meter") == "gauge"
    assert canon_class("gauge_01") == "gauge"
    assert canon_class("human person") == "person"
    assert canon_class("wooden crate") == "crate"


def test_parse_world_gt_skips_structure():
    gt = parse_world_gt(_SDF)
    names = sorted(g["name"] for g in gt)
    assert names == ["gauge_01", "gauge_02"], "only object models are ground truth (no ground/maze)"
    g01 = next(g for g in gt if g["name"] == "gauge_01")
    assert g01["class"] == "gauge" and abs(g01["x"] - 5.85) < 1e-6 and abs(g01["y"] - 2.0) < 1e-6


def test_score_perfect_detection():
    gt = parse_world_gt(_SDF)
    det = [{"class": "analog gauge", "x": 5.80, "y": 2.05}, {"class": "gauge", "x": -3.05, "y": -3.80}]
    r = score_detections(gt, det, match_radius=1.0)
    assert r["tp"] == 2 and r["fp"] == 0 and r["fn"] == 0
    assert r["precision"] == 1.0 and r["recall"] == 1.0 and r["f1"] == 1.0
    assert r["mean_loc_error_m"] is not None and r["mean_loc_error_m"] < 0.1


def test_score_missed_object_is_false_negative():
    gt = parse_world_gt(_SDF)
    det = [{"class": "gauge", "x": 5.85, "y": 2.0}]  # only one of two found
    r = score_detections(gt, det, match_radius=1.0)
    assert r["tp"] == 1 and r["fn"] == 1 and r["recall"] == 0.5
    assert r["per_class"]["gauge"] == {"gt": 2, "found": 1, "recall": 0.5}


def test_score_false_positive_out_of_radius():
    gt = parse_world_gt(_SDF)
    det = [{"class": "gauge", "x": 5.85, "y": 2.0}, {"class": "gauge", "x": 0.0, "y": 0.0}]
    r = score_detections(gt, det, match_radius=1.0)  # second detection matches nothing within 1 m
    assert r["tp"] == 1 and r["fp"] == 1 and r["precision"] == 0.5


def test_load_detected_reads_zone_files(tmp_path):
    zdir = tmp_path / "zone_0"
    zdir.mkdir()
    json.dump(
        {
            "zone": "zone_0",
            "objects": [
                {"class": "analog gauge", "world": [5.85, 2.0, 0.3], "localized": True},
                {"class": "blob", "world": [1.0, 1.0], "localized": False},  # unlocalized -> skipped
            ],
        },
        open(zdir / "objects.json", "w"),
    )
    det = load_detected(str(tmp_path))
    assert len(det) == 1 and det[0]["class"] == "gauge" and det[0]["zone"] == "zone_0"
