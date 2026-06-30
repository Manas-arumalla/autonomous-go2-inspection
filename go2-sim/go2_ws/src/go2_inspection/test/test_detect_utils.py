"""Unit tests for the open-vocab detection helpers (pure — no ROS, no torch).

Covers the gauge-class addition (ADR-016 M4): the vocabulary must contain gauge prompts, `is_gauge` must
classify them, and `color_for` must group classes into the right semantic colours.
"""
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))  # make `go2_inspection` importable

from go2_inspection import detect_utils  # noqa: E402
from go2_inspection.detect_utils import is_gauge, color_for, PROMPTS  # noqa: E402


def test_is_gauge_classifies():
    assert is_gauge("white round analog gauge with a needle")
    assert is_gauge("circular pressure dial meter")
    assert not is_gauge("black office chair")
    assert not is_gauge("red fire extinguisher")
    assert not is_gauge("")


def test_vocabulary_includes_gauges():
    assert sum(1 for p in PROMPTS if is_gauge(p)) >= 2, "gauge classes must be present in the vocabulary"


def test_color_for_semantic_groups():
    assert color_for("analog gauge") == (206, 194, 54)          # gauges -> cyan
    assert color_for("human person") == (0, 140, 255)            # people -> orange
    assert color_for("red fire extinguisher") == (0, 220, 220)   # safety equipment -> yellow
    assert color_for("orange flame fire") == (40, 40, 220)       # hazard -> red
    assert color_for("black crate") == (60, 200, 60)             # default -> green


# --- bounded text-PE loader (ADR-016 runtime fix) -------------------------------------------------
# `model.get_text_pe()` downloads the CLIP backend with no internal timeout; offline it would hang the
# zone_inspector node in __init__ forever (robot sits after arriving at a zone). `_get_text_pe_bounded`
# must instead RAISE so the node degrades gracefully (navigate + spin, no detection).


def test_text_pe_bounded_times_out(monkeypatch):
    class _SlowModel:                       # simulates a CLIP backend that never returns in time
        def get_text_pe(self, names):
            time.sleep(5.0)
            return "pe"

    monkeypatch.setenv("YOLOE_PE_TIMEOUT", "0.3")
    with pytest.raises(TimeoutError):
        detect_utils._get_text_pe_bounded(_SlowModel(), ["a", "b"])


def test_text_pe_bounded_passes_through():
    class _FastModel:
        def get_text_pe(self, names):
            return ("pe", tuple(names))

    assert detect_utils._get_text_pe_bounded(_FastModel(), ["x"]) == ("pe", ("x",))


def test_text_pe_bounded_propagates_error():
    class _BadModel:
        def get_text_pe(self, names):
            raise RuntimeError("no CLIP backend")

    with pytest.raises(RuntimeError):
        detect_utils._get_text_pe_bounded(_BadModel(), ["x"])
