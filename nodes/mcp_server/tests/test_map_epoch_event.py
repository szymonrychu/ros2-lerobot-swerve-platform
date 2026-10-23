"""Tests for the map_epoch_reset robot event: FrontierTracker epoch changes explained by /robot_events causes."""

import json
import logging
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from mcp_server.config import FrontierSettings, McpServerConfig, MonitorSettings
from mcp_server.frontiers import FrontierTracker, MapGrid
from mcp_server.models import RobotEvent
from mcp_server.monitor import RobotMonitor
from mcp_server.tools import build_mcp_server

from .test_frontier_tools import FrontierRobot, room
from .test_tools import TOKEN, call

RES = 0.05
CAUSE_WINDOW_S = 30.0
EVENT_TYPE = "map_epoch_reset"


class Clock:
    """Settable monotonic clock for the monitor."""

    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t


def small_grid(version: tuple = (2,), size: int = 12) -> MapGrid:
    """A closed little room: far fewer known cells than room() so the tracker starts a new epoch."""
    data = np.full((80, 80), -1, dtype=np.int16)
    data[10 : 10 + size, 10 : 10 + size] = 100
    data[11 : 9 + size, 11 : 9 + size] = 0
    return MapGrid.from_array(data, RES, 0.0, 0.0, 0.0, version)


def external(type_: str, source: str, message: str, data: dict[str, Any], severity: str = "critical") -> str:
    """A /robot_events payload as published by web_ui or map_guard."""
    return json.dumps(
        {"seq": 1, "ts": 1.0, "type": type_, "severity": severity, "source": source, "message": message, "data": data}
    )


def wired() -> tuple[RobotMonitor, FrontierTracker, list[RobotEvent], Clock]:
    """Monitor and tracker joined the way build_mcp_server joins them, with a collecting sink."""
    clock = Clock()
    monitor = RobotMonitor(
        MonitorSettings(), None, clock=clock, cpu_temp_reader=lambda: 48.0, throttled_reader=lambda: 0
    )
    events: list[RobotEvent] = []
    monitor.set_sink(events.append)
    tracker = FrontierTracker(FrontierSettings(), footprint_width_m=0.2)
    tracker.on_epoch_reset = monitor.on_map_epoch_reset
    return monitor, tracker, events, clock


def epoch_events(events: list[RobotEvent]) -> list[RobotEvent]:
    return [e for e in events if e.type == EVENT_TYPE]


def test_epoch_change_publishes_event(caplog: pytest.LogCaptureFixture) -> None:
    monitor, tracker, events, _ = wired()
    big = room()
    tracker.update(big)
    assert epoch_events(events) == []  # epoch 1 is not a reset
    with caplog.at_level(logging.WARNING, logger="mcp_server.monitor"):
        snap = tracker.update(small_grid())
    [event] = epoch_events(events)
    assert snap.epoch == 2
    assert event.source == "mcp_server" and event.severity == "warning"
    data = event.data
    assert (data["epoch_before"], data["epoch_after"]) == (1, 2)
    assert data["known_before"] == big.known_cells and data["known_after"] == snap.grid.known_cells
    assert data["resolution_before"] == RES and data["resolution_after"] == RES
    assert {"epoch_before", "epoch_after", "known_before", "known_after", "cause"} <= set(data)
    assert any("map_epoch_reset" in r.getMessage() and "known_before" in r.getMessage() for r in caplog.records)
    # shows in get_body_state's recent events and the digest
    assert any(e["type"] == EVENT_TYPE for e in monitor.digest("test-session")[0])
    assert any(e.type == EVENT_TYPE for e in monitor.body_state().recent_events)
    # a repeat of the same snapshot version is not a second event; a further collapse is debounced
    tracker.update(small_grid())
    tracker.update(small_grid(version=(3,), size=5))
    assert len(epoch_events(events)) == 1


def test_critical_while_base_motion_runs() -> None:
    monitor, tracker, events, _ = wired()
    tracker.update(room())
    with monitor.base_motion():
        tracker.update(small_grid())
    assert epoch_events(events)[0].severity == "critical"


def test_critical_while_a_job_runs() -> None:
    monitor, tracker, events, _ = wired()
    monitor.job_running = lambda: True
    tracker.update(room())
    tracker.update(small_grid())
    assert epoch_events(events)[0].severity == "critical"


def test_cause_from_recent_map_reset_event() -> None:
    monitor, tracker, events, clock = wired()
    tracker.update(room())
    monitor.on_robot_event(external("map_guard_reset", "map_guard", "old reset", {"requester": "x"}))
    clock.t += 10.0
    monitor.on_robot_event(
        external("map_reset", "web_ui", "map reset on tab 'main' requested by 10.0.0.7: ok", {"requester": "10.0.0.7"})
    )
    clock.t += 5.0
    tracker.update(small_grid())
    [event] = epoch_events(events)
    cause = event.data["cause"]
    assert cause["type"] == "map_reset" and cause["source"] == "web_ui"
    assert cause["requester"] == "10.0.0.7"
    assert "requested by 10.0.0.7" in cause["message"]
    assert tracker.epoch_cause == cause and tracker.epoch_changed_at is not None


def test_cause_from_map_guard_reset_event() -> None:
    monitor, tracker, events, _ = wired()
    tracker.update(room())
    monitor.on_robot_event(external("map_guard_reset", "map_guard", "disturbed too long", {"backup": "/b"}))
    tracker.update(small_grid())
    cause = epoch_events(events)[0].data["cause"]
    assert cause["type"] == "map_guard_reset" and cause["source"] == "map_guard"
    assert cause["message"] == "disturbed too long"


def test_unexplained_cause_with_counts() -> None:
    monitor, tracker, events, clock = wired()
    big = room()
    tracker.update(big)
    monitor.on_robot_event(external("map_reset", "web_ui", "stale", {"requester": "x"}))
    clock.t += CAUSE_WINDOW_S + 1.0  # outside the window
    snap = tracker.update(small_grid())
    cause = epoch_events(events)[0].data["cause"]
    assert cause["type"] == "unexplained"
    assert cause["known_before"] == big.known_cells and cause["known_after"] == snap.grid.known_cells
    assert "no map_reset or map_guard_reset event" in cause["message"]
    # unrelated events are not causes
    monitor2, tracker2, events2, _ = wired()
    tracker2.update(room())
    monitor2.on_robot_event(external("collision_stop", "front", "stop", {}))
    tracker2.update(small_grid())
    assert epoch_events(events2)[0].data["cause"]["type"] == "unexplained"


def test_malformed_external_event_is_ignored() -> None:
    monitor, _, _, _ = wired()
    monitor.on_robot_event("not json")
    monitor.on_robot_event(json.dumps([1, 2]))


def test_exploration_status_names_epoch_cause(tmp_path: Path) -> None:
    robot = FrontierRobot(tmp_path)
    monitor = RobotMonitor(MonitorSettings(), None, cpu_temp_reader=lambda: 48.0, throttled_reader=lambda: 0)
    server = build_mcp_server(robot, McpServerConfig(), TOKEN, None, monitor)
    first = call(server, "list_frontiers", {}).structured_content
    assert first["epoch"] == 1 and first["epoch_changed_at"] is None and first["epoch_cause"] is None
    robot.pois.append(
        {
            "id": "s1",
            "name": "n",
            "kind": "point",
            "x": 1.5,
            "y": 2.0,
            "status": "open",
            "created_by": "agent",
            "note": "",
        }
    )
    monitor.on_robot_event(external("map_reset", "web_ui", "reset by operator", {"requester": "10.0.0.7"}))
    robot.map = small_grid()
    robot.pose = robot.pose.model_copy(update={"x": 0.8, "y": 0.8})
    listed = call(server, "list_frontiers", {}).structured_content
    assert listed["epoch"] == 2 and listed["epoch_changed_at"] is not None
    assert listed["epoch_cause"]["type"] == "map_reset" and listed["epoch_cause"]["requester"] == "10.0.0.7"
    status = call(server, "get_exploration_status", {}).structured_content
    assert status["epoch"] == 2
    assert status["epoch_changed_at"] == listed["epoch_changed_at"]
    assert status["epoch_cause"] == listed["epoch_cause"]
    assert any(e.type == EVENT_TYPE for e in monitor.body_state().recent_events)
