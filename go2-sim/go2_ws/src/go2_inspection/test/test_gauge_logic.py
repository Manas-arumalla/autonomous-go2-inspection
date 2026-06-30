"""Unit tests for the gauge-reading layer's pure logic (no ROS, no Anthropic SDK).

`gauge_inspector` defers the `anthropic` import to call time, so the gauge classifier is importable and
testable on its own.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from go2_inspection.gauge_inspector import _is_gauge, score_readings  # noqa: E402


def test_gauge_inspector_filters_gauges():
    assert _is_gauge("analog gauge")
    assert _is_gauge("pressure dial meter")
    assert not _is_gauge("wooden desk")
    assert not _is_gauge(None)


def test_score_readings_type_unit_value():
    # readings: (gauge_meta, model_reading); gt matched left->right by `lateral`
    readings = [
        ({"id": "g_left", "lateral": -1.0}, {"type": "pressure", "unit": "bar", "reading": 4.0}),
        ({"id": "g_right", "lateral": 1.0}, {"type": "temperature", "unit": "°C", "reading": 50.0}),
    ]
    gt = [
        {"type": "PRESSURE", "unit": "bar", "true_reading": 4.0, "range_min": 0, "range_max": 10},
        {"type": "TEMPERATURE", "unit": "C", "true_reading": 60.0, "range_min": 0, "range_max": 100},
    ]
    r = score_readings(readings, gt)
    assert r["n"] == 2
    assert r["type_ok"] == 2  # case-insensitive type match
    assert r["unit_ok"] == 2  # "°C" normalizes to "c" == "C"
    assert r["value_ok"] == 1  # g_left exact; g_right err=|50-60|/100=10% > 5% span
    assert r["rows"][0]["value_ok"] is True and r["rows"][0]["err_pct"] == 0.0
    assert r["rows"][1]["value_ok"] is False and r["rows"][1]["err_pct"] == 10.0


def test_score_readings_orders_by_lateral():
    # readings given right-then-left must be re-ordered to match gt left->right
    readings = [
        ({"id": "right", "lateral": 2.0}, {"type": "x", "unit": "u", "reading": 9.0}),
        ({"id": "left", "lateral": -2.0}, {"type": "x", "unit": "u", "reading": 1.0}),
    ]
    gt = [
        {"type": "x", "unit": "u", "true_reading": 1.0, "range_min": 0, "range_max": 10},
        {"type": "x", "unit": "u", "true_reading": 9.0, "range_min": 0, "range_max": 10},
    ]
    r = score_readings(readings, gt)
    assert r["value_ok"] == 2  # correctly paired after lateral sort
    assert [row["id"] for row in r["rows"]] == ["left", "right"]
