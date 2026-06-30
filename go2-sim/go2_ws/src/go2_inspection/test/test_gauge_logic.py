"""Unit tests for the gauge-reading layer's pure logic (no ROS, no Anthropic SDK).

`gauge_inspector` defers the `anthropic` import to call time, so the gauge classifier is importable and
testable on its own.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from go2_inspection.gauge_inspector import _is_gauge  # noqa: E402


def test_gauge_inspector_filters_gauges():
    assert _is_gauge("analog gauge")
    assert _is_gauge("pressure dial meter")
    assert not _is_gauge("wooden desk")
    assert not _is_gauge(None)
