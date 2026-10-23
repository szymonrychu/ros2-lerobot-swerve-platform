"""Tilt events carry the evidence to judge them: IMU roll/pitch, time over the limit, base speed, ground plane, map_guard."""

import json
from typing import Any

import pytest

from mcp_server.config import MonitorSettings
from mcp_server.models import GroundState
from mcp_server.monitor import RobotMonitor

from .test_ground_summary import example
from .test_tilt_guard import Clock, make, state

GROUND_PITCH_DEG = -18.0
GROUND_ROLL_DEG = 1.5
EVIDENCE_KEYS = (
    "imu_roll_deg",
    "imu_pitch_deg",
    "over_limit_s",
    "base_speed_mps",
    "ground_pitch_deg",
    "ground_roll_deg",
    "ground_age_s",
    "map_guard_state",
)


def ground(pitch: float | None = GROUND_PITCH_DEG, roll: float | None = GROUND_ROLL_DEG) -> GroundState:
    """The contract example with the plane slope replaced."""
    raw = example()
    raw["slope_pitch_deg"] = pitch
    raw["slope_roll_deg"] = roll
    return GroundState.model_validate(raw)


def guard_event(mon: RobotMonitor, kind: str) -> dict[str, Any]:
    """The last event of a type, as data."""
    return next(e for e in reversed(mon.events) if e.type == kind).data


def map_guard(kind: str) -> str:
    """A map_guard /robot_events payload."""
    return json.dumps({"type": kind, "source": "map_guard", "severity": "warning", "message": "x", "data": {}})


def test_tilt_warning_carries_imu_and_ground_pitch() -> None:
    mon, clock, seen, _ = make()
    mon.on_odom(0.1, 0.0, 0.0)
    mon.on_ground_state(ground())
    mon.on_robot_event(map_guard("map_guard_disturbed"))
    clock.t += 0.5
    mon.on_tilt_guard(state("warning", roll=2.1, pitch=-19.4))
    data = seen[-1].data
    assert seen[-1].type == "tilt_warning"
    assert data["imu_roll_deg"] == pytest.approx(2.1) and data["imu_pitch_deg"] == pytest.approx(-19.4)
    assert data["over_limit_s"] == 0.0
    assert data["base_speed_mps"] == pytest.approx(0.1)
    assert data["ground_pitch_deg"] == pytest.approx(GROUND_PITCH_DEG)
    assert data["ground_roll_deg"] == pytest.approx(GROUND_ROLL_DEG)
    assert data["ground_age_s"] == pytest.approx(0.5)
    assert data["map_guard_state"] == "disturbed"


def test_tilt_error_carries_the_same_evidence() -> None:
    mon, clock, seen, _ = make()
    mon.on_tilt_guard(state("warning", pitch=-14.0))
    clock.t += 2.0
    mon.on_tilt_guard(state("error", pitch=-25.0))
    data = seen[-1].data
    assert seen[-1].type == "tilt_error"
    assert data["imu_pitch_deg"] == pytest.approx(-25.0) and data["over_limit_s"] == pytest.approx(2.0)


def test_ground_pitch_null_when_stale() -> None:
    mon, clock, seen, _ = make()
    mon.on_ground_state(ground())
    clock.t += 1.5
    mon.on_tilt_guard(state("warning", pitch=-19.4))
    data = seen[-1].data
    assert data["ground_pitch_deg"] is None and data["ground_roll_deg"] is None
    assert data["ground_age_s"] == pytest.approx(1.5)


def test_missing_data_is_null_not_invented() -> None:
    mon, _, seen, _ = make()
    mon.on_tilt_guard(state("warning", pitch=-19.4))
    data = seen[-1].data
    for key in ("base_speed_mps", "ground_pitch_deg", "ground_roll_deg", "ground_age_s", "map_guard_state"):
        assert key in data and data[key] is None
    assert all(key in data for key in EVIDENCE_KEYS)


def test_ground_without_plane_gives_null_slopes_but_an_age() -> None:
    mon, _, seen, _ = make()
    mon.on_ground_state(ground(pitch=None, roll=None))
    mon.on_tilt_guard(state("warning", pitch=-19.4))
    data = seen[-1].data
    assert data["ground_pitch_deg"] is None and data["ground_age_s"] == 0.0


def test_stale_odometry_gives_null_speed() -> None:
    mon, clock, seen, _ = make()
    mon.on_odom(0.3, 0.4, 0.0)
    clock.t += 5.0
    mon.on_tilt_guard(state("warning", pitch=-19.4))
    assert seen[-1].data["base_speed_mps"] is None


def test_tilt_clear_carries_over_limit_time() -> None:
    mon, clock, seen, _ = make()
    mon.on_odom(0.3, 0.4, 0.0)
    mon.on_tilt_guard(state("warning", roll=2.1, pitch=-19.4))
    clock.t += 6.0
    mon.on_odom(0.0, 0.0, 0.0)
    mon.on_robot_event(map_guard("map_guard_resumed"))
    mon.on_tilt_guard(state("ok", roll=0.4, pitch=1.5))
    clear = seen[-1]
    assert clear.type == "tilt_clear"
    assert clear.data["over_limit_s"] == pytest.approx(6.0)
    assert clear.data["imu_roll_deg"] == pytest.approx(0.4) and clear.data["imu_pitch_deg"] == pytest.approx(1.5)
    assert clear.data["base_speed_mps"] == 0.0
    assert clear.data["map_guard_state"] == "ok"


def test_over_limit_restarts_after_a_clear() -> None:
    mon, clock, seen, _ = make()
    mon.on_tilt_guard(state("warning", pitch=-19.4))
    clock.t += 4.0
    mon.on_tilt_guard(state("ok"))
    clock.t += 10.0
    mon.on_tilt_guard(state("warning", pitch=-15.0))
    assert seen[-1].data["over_limit_s"] == 0.0


def test_event_log_line_carries_the_evidence(caplog: pytest.LogCaptureFixture) -> None:
    mon, _, _, _ = make()
    with caplog.at_level("WARNING", logger="mcp_server.monitor"):
        mon.on_tilt_guard(state("warning", roll=2.1, pitch=-19.4))
    line = next(r.getMessage() for r in caplog.records if "tilt_warning" in r.getMessage())
    assert "imu_pitch_deg=-19.4" in line and "map_guard_state=None" in line


def test_unrelated_events_do_not_change_map_guard_state() -> None:
    mon = RobotMonitor(MonitorSettings(), None, clock=Clock(), cpu_temp_reader=lambda: 1.0, throttled_reader=lambda: 0)
    mon.on_robot_event(map_guard("map_guard_disturbed"))
    mon.on_robot_event(json.dumps({"type": "bump", "source": "x", "data": {}}))
    mon.on_robot_event("not json")
    assert mon.map_guard_state == "disturbed"
