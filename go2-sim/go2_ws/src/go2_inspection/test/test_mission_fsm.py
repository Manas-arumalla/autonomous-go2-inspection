"""Unit tests for the mission state machine + event stream (pure — no ROS).

Verifies the lifecycle transitions, that illegal transitions are rejected, the structured event stream
(in-memory + JSONL), and abort-from-any-active-state.
"""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))  # make `go2_inspection` importable

from go2_inspection.mission_fsm import MissionFSM, MissionState, read_events  # noqa: E402


def _clock():
    _clock.t += 1.0
    return _clock.t


_clock.t = 0.0


def test_full_lifecycle_two_zones():
    f = MissionFSM(clock=_clock)
    assert f.state == MissionState.IDLE
    f.to(MissionState.PLANNING, data={"zones": ["zone_0", "zone_1"]})
    for z in ("zone_0", "zone_1"):
        f.to(MissionState.NAVIGATING, zone=z)
        f.to(MissionState.INSPECTING, zone=z)
        f.to(MissionState.READING, zone=z)
    f.to(MissionState.ROLLUP)
    f.to(MissionState.DONE, data={"total_objects": 2})
    assert f.state == MissionState.DONE
    kinds = [e["kind"] for e in f.events]
    assert kinds[0] == "PLANNING" and kinds[-1] == "DONE"
    assert [e["seq"] for e in f.events] == list(range(1, len(f.events) + 1))  # monotonic seq


def test_illegal_transition_raises():
    f = MissionFSM(clock=_clock)
    with pytest.raises(ValueError):
        f.to(MissionState.DONE)  # IDLE -> DONE is not allowed
    f.to(MissionState.PLANNING)
    with pytest.raises(ValueError):
        f.to(MissionState.READING)  # PLANNING -> READING is not allowed


def test_abort_from_active_state():
    f = MissionFSM(clock=_clock)
    f.to(MissionState.PLANNING)
    f.to(MissionState.NAVIGATING, zone="zone_0")
    ev = f.abort(reason="cancelled")
    assert f.state == MissionState.ABORTED and ev["kind"] == "ABORTED"
    assert f.abort() is None  # already terminal -> no-op


def test_jsonl_stream_roundtrip(tmp_path):
    p = tmp_path / "mission_events.jsonl"
    f = MissionFSM(path=str(p), clock=_clock)
    f.to(MissionState.PLANNING, data={"zones": ["z0"]})
    f.to(MissionState.NAVIGATING, zone="z0", data={"target": [2.9, 2.0]})
    lines = [json.loads(x) for x in open(p)]
    assert len(lines) == 2 and lines[1]["zone"] == "z0" and lines[1]["data"]["target"] == [2.9, 2.0]
    assert read_events(str(p), last=1)[0]["kind"] == "NAVIGATING"
