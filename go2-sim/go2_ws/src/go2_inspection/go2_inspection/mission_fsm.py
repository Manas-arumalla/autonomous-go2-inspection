"""mission_fsm.py — explicit mission state machine + structured event stream (ROS-free, CI-testable).

The converged orchestration (from `-main`) returns a task_id and runs the mission asynchronously, but the
mission *itself* is a sequence of `print()`s with no explicit state model — you can't ask "what phase is
it in?", replay what happened, or feed progress to a dashboard. This adds a small, validated state machine
over the mission lifecycle plus a structured JSONL **event stream** that `get_status` / MCP / a dashboard /
the benchmark can consume.

Lifecycle:
    IDLE -> PLANNING -> NAVIGATING -> INSPECTING -> (READING) -> [next zone: NAVIGATING] ... -> ROLLUP -> DONE
ABORTED is reachable from any active phase. Each transition appends an event
    {seq, t, state, kind, zone?, data?}
to an in-memory list and (optionally) a JSONL file — one line per event, append-only, newest last.

The state machine is strict (illegal transitions raise) so it's unit-testable; callers that only want the
event stream wrap calls defensively so observability can never break the mission's control flow.
"""
import json
import os
import time


class MissionState:
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"
    INSPECTING = "INSPECTING"
    READING = "READING"
    ROLLUP = "ROLLUP"
    DONE = "DONE"
    ABORTED = "ABORTED"


ACTIVE = frozenset(
    {
        MissionState.PLANNING,
        MissionState.NAVIGATING,
        MissionState.INSPECTING,
        MissionState.READING,
        MissionState.ROLLUP,
    }
)

# allowed forward transitions (ABORTED is added from every active state below)
_TRANSITIONS = {
    MissionState.IDLE: {MissionState.PLANNING},
    MissionState.PLANNING: {MissionState.NAVIGATING, MissionState.ROLLUP, MissionState.DONE},
    MissionState.NAVIGATING: {MissionState.INSPECTING, MissionState.NAVIGATING, MissionState.ROLLUP},
    MissionState.INSPECTING: {MissionState.READING, MissionState.NAVIGATING, MissionState.ROLLUP},
    MissionState.READING: {MissionState.NAVIGATING, MissionState.ROLLUP},
    MissionState.ROLLUP: {MissionState.DONE},
    MissionState.DONE: set(),
    MissionState.ABORTED: set(),
}
for _s in list(ACTIVE):
    _TRANSITIONS[_s] = set(_TRANSITIONS[_s]) | {MissionState.ABORTED}


class MissionFSM:
    """Strict mission state machine with an append-only structured event stream."""

    def __init__(self, path=None, clock=time.time):
        self.state = MissionState.IDLE
        self.seq = 0
        self.events = []
        self._clock = clock
        self.path = os.path.expanduser(path) if path else None
        if self.path:
            try:
                os.makedirs(os.path.dirname(self.path), exist_ok=True)
                open(self.path, "w").close()  # one fresh stream per mission
            except Exception:
                self.path = None

    def can(self, to_state):
        return to_state == self.state or to_state in _TRANSITIONS.get(self.state, set())

    def to(self, to_state, kind=None, zone=None, data=None):
        """Transition to `to_state` (raises ValueError if illegal) and emit an event."""
        if to_state != self.state and to_state not in _TRANSITIONS.get(self.state, set()):
            raise ValueError(f"illegal mission transition {self.state} -> {to_state}")
        self.state = to_state
        return self.emit(kind or to_state, zone=zone, data=data)

    def emit(self, kind, zone=None, data=None):
        """Append an event in the current state without changing state (e.g. a progress note)."""
        self.seq += 1
        ev = {"seq": self.seq, "t": round(self._clock(), 3), "state": self.state, "kind": kind}
        if zone is not None:
            ev["zone"] = zone
        if data is not None:
            ev["data"] = data
        self.events.append(ev)
        if self.path:
            try:
                with open(self.path, "a") as f:
                    f.write(json.dumps(ev) + "\n")
            except Exception:
                pass
        return ev

    def abort(self, reason=None):
        """Force-abort from any active state (no-op if already terminal)."""
        if self.state in (MissionState.DONE, MissionState.ABORTED):
            return None
        self.state = MissionState.ABORTED
        return self.emit("ABORTED", data={"reason": reason} if reason else None)


def read_events(path, last=None):
    """Read an event stream back (helper for get_status / dashboards). Returns a list of event dicts."""
    p = os.path.expanduser(path)
    if not os.path.exists(p):
        return []
    out = []
    for line in open(p):
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except Exception:
            continue
    return out[-last:] if last else out
