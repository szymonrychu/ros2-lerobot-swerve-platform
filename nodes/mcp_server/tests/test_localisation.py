"""Localisation health from map_guard_cpp's /map_guard/state: parsing, the reliable verdict, stale and missing data,
the monitor store, the tilt evidence and the ros_iface wiring."""

import json
from types import ModuleType
from typing import Any
from unittest.mock import MagicMock

import pytest

from mcp_server.config import McpServerConfig, MonitorSettings
from mcp_server.localisation import (
    UNRELIABLE_STATES,
    MapGuardStatus,
    localisation_health,
    parse_map_guard_state,
)
from mcp_server.models import ArmState, LocalisationHealth

from .test_tilt_guard import make, state

STALE_S = 3.0


def guard_state(**overrides: Any) -> dict[str, Any]:
    """A healthy /map_guard/state payload with fields replaced."""
    raw: dict[str, Any] = {
        "state": "ok",
        "reasons": [],
        "agreement_ratio": 0.92,
        "ratio_age_s": 0.3,
        "unscored_scans": 0,
        "median_ratio": 0.9,
        "healthy_ratio": 0.6,
        "dry_run": False,
        "held": False,
        "stamp": 1_760_000_000.0,
    }
    raw.update(overrides)
    return raw


def status(**overrides: Any) -> MapGuardStatus:
    """A parsed guard state."""
    parsed = parse_map_guard_state(json.dumps(guard_state(**overrides)))
    assert parsed is not None
    return parsed


def test_parse_reads_every_field() -> None:
    parsed = status(state="disturbed", reasons=["carried", "rotated"], agreement_ratio=0.03)
    assert parsed.state == "disturbed" and parsed.reasons == ["carried", "rotated"]
    assert parsed.agreement_ratio == 0.03 and parsed.median_ratio == 0.9 and parsed.healthy_ratio == 0.6
    assert parsed.dry_run is False and parsed.held is False and parsed.ratio_age_s == 0.3
    assert parsed.unscored_scans == 0


@pytest.mark.parametrize(
    "payload",
    [
        "not json",
        "[1]",
        json.dumps(guard_state(state="lost")),
        json.dumps(guard_state(agreement_ratio=1.5)),
        json.dumps({k: v for k, v in guard_state().items() if k != "healthy_ratio"}),
        json.dumps({k: v for k, v in guard_state().items() if k != "unscored_scans"}),
        json.dumps(guard_state(unscored_scans=-1)),
    ],
)
def test_parse_rejects_anything_off_contract(payload: str) -> None:
    assert parse_map_guard_state(payload) is None


def test_fresh_healthy_state_is_reliable_without_notes() -> None:
    health, notes = localisation_health(status(), 0.5, STALE_S)
    assert health is not None and health.reliable is True and notes == []
    assert health.state == "ok" and health.agreement_ratio == 0.92 and health.median_ratio == 0.9
    assert health.healthy_ratio == 0.6 and health.dry_run is False and health.age_s == 0.5
    assert health.ratio_age_s == pytest.approx(0.8) and health.unscored_scans == 0
    assert health.note is None


def test_missing_state_is_null_with_a_note() -> None:
    health, notes = localisation_health(None, None, STALE_S)
    assert health is None
    assert len(notes) == 1 and "no /map_guard/state" in notes[0] and "unknown" in notes[0]


def test_stale_state_is_null_with_a_note() -> None:
    health, notes = localisation_health(status(), STALE_S + 0.1, STALE_S)
    assert health is None
    assert len(notes) == 1 and "stale" in notes[0] and "3.1 s" in notes[0]


@pytest.mark.parametrize("guard", sorted(UNRELIABLE_STATES))
def test_unsettled_states_are_unreliable(guard: str) -> None:
    health, notes = localisation_health(status(state=guard, reasons=["carried"]), 0.2, STALE_S)
    assert health is not None and health.reliable is False
    assert health.note is not None and guard in health.note
    assert notes == [health.note]


def test_kidnap_ratio_is_unreliable_and_names_relocalise() -> None:
    """The incident: dry run resumed to ok at agreement 0.03."""
    health, notes = localisation_health(
        status(state="ok", agreement_ratio=0.03, median_ratio=0.04, dry_run=True), 0.2, STALE_S
    )
    assert health is not None and health.reliable is False
    assert health.note is not None
    for phrase in ("0.04", "0.6", "relocalise", "navigate_to_pose", "move_relative", "ask a person"):
        assert phrase in health.note, phrase
    assert notes == [health.note]


def test_relocalising_tells_the_agent_to_wait() -> None:
    health, _ = localisation_health(status(state="relocalising", reasons=["mismatch"]), 0.2, STALE_S)
    assert health is not None and health.note is not None and "wait" in health.note


@pytest.mark.parametrize(("latest", "median"), [(0.4, 0.8), (0.8, 0.4)])
def test_both_the_latest_sample_and_the_median_must_be_healthy(latest: float, median: float) -> None:
    health, notes = localisation_health(status(agreement_ratio=latest, median_ratio=median), 0.2, STALE_S)
    assert health is not None and health.reliable is False
    assert health.note is not None and "0.40" in health.note and "relocalise" in health.note
    assert notes == [health.note]


def test_latest_sample_alone_is_judged_without_a_median() -> None:
    fallback, _ = localisation_health(status(agreement_ratio=0.8, median_ratio=None), 0.2, STALE_S)
    assert fallback is not None and fallback.reliable is True


def assert_unknown_without_relocalise_advice(health: LocalisationHealth | None, notes: list[str]) -> str:
    """reliable is null (unknown), the note says it is no evidence of a wrong pose and advises no relocalise."""
    assert health is not None and health.reliable is None
    assert health.note is not None and "not evidence of a wrong pose" in health.note
    assert "call relocalise" not in health.note and "ask a person" not in health.note
    assert notes == [health.note]
    return health.note


def test_no_ratio_in_ok_is_unknown_not_invented() -> None:
    health, notes = localisation_health(
        status(agreement_ratio=None, ratio_age_s=None, median_ratio=None, unscored_scans=0), 0.2, STALE_S
    )
    note = assert_unknown_without_relocalise_advice(health, notes)
    assert health is not None and health.agreement_ratio is None and health.ratio_age_s is None
    assert "no scan scored" in note


def test_unscorable_open_space_is_unknown_not_bad() -> None:
    """Exploring: the robot faces unmapped or open space, map_guard cannot score agreement, the pose is fine."""
    health, notes = localisation_health(status(ratio_age_s=STALE_S + 5.0, unscored_scans=12), 0.2, STALE_S)
    note = assert_unknown_without_relocalise_advice(health, notes)
    assert "map agreement not scorable here (unmapped/open space), not evidence of a wrong pose" in note
    assert health is not None and health.unscored_scans == 12


def test_old_low_ratio_in_ok_is_still_unknown() -> None:
    health, notes = localisation_health(
        status(agreement_ratio=0.1, median_ratio=0.1, ratio_age_s=STALE_S + 5.0, unscored_scans=3), 0.2, STALE_S
    )
    assert_unknown_without_relocalise_advice(health, notes)


def test_ratio_age_counts_the_message_age() -> None:
    """A 2 s old sample in a 1.5 s old message is 3.5 s old: past the 3 s limit."""
    health, notes = localisation_health(status(ratio_age_s=2.0, unscored_scans=4), 1.5, STALE_S)
    assert_unknown_without_relocalise_advice(health, notes)
    assert health is not None and health.ratio_age_s == pytest.approx(3.5)
    fresh, _ = localisation_health(status(ratio_age_s=1.0), 1.5, STALE_S)
    assert fresh is not None and fresh.reliable is True


def test_held_is_unreliable() -> None:
    health, _ = localisation_health(status(held=True), 0.2, STALE_S)
    assert health is not None and health.reliable is False
    assert health.note is not None and "not localised" in health.note


def test_ratio_exactly_healthy_is_reliable() -> None:
    health, _ = localisation_health(status(agreement_ratio=0.6, median_ratio=0.6), 0.2, STALE_S)
    assert health is not None and health.reliable is True


# --- monitor store ---------------------------------------------------------------------------------------------------


def test_monitor_localisation_follows_the_topic_and_ages() -> None:
    mon, clock, _, _ = make()
    health, notes = mon.localisation()
    assert health is None and notes
    mon.on_map_guard_state(json.dumps(guard_state(stamp=clock.wall())))
    clock.t += 1.0
    health, notes = mon.localisation()
    assert health is not None and health.reliable is True and health.age_s == pytest.approx(1.0) and notes == []
    clock.t += MonitorSettings().map_guard_state_stale_s
    health, notes = mon.localisation()
    assert health is None and "stale" in notes[0]


def test_monitor_age_is_the_older_of_receipt_and_stamp() -> None:
    """A latched stale sample delivered late (old stamp, same host clock) is not treated as fresh."""
    mon, clock, _, _ = make()
    mon.on_map_guard_state(json.dumps(guard_state(stamp=clock.wall() - 10.0)))
    health, notes = mon.localisation()
    assert health is None and "stale" in notes[0] and "10.0 s" in notes[0]
    mon.on_map_guard_state(json.dumps(guard_state(stamp=clock.wall() - 1.0)))
    health, _ = mon.localisation()
    assert health is not None and health.age_s == pytest.approx(1.0)


def test_monitor_tracks_the_last_reliable_wall_time() -> None:
    mon, clock, _, _ = make()
    assert mon.last_reliable_wall() is None
    good = clock.wall()
    mon.on_map_guard_state(json.dumps(guard_state(stamp=good, ratio_age_s=0.3)))
    assert mon.last_reliable_wall() == pytest.approx(good - 0.3)
    clock.t += 5.0
    mon.on_map_guard_state(json.dumps(guard_state(state="disturbed", reasons=["carried"], stamp=clock.wall())))
    clock.t += 5.0
    mon.on_map_guard_state(json.dumps(guard_state(agreement_ratio=0.05, median_ratio=0.05, stamp=clock.wall())))
    clock.t += 5.0
    mon.on_map_guard_state(json.dumps(guard_state(ratio_age_s=30.0, unscored_scans=9, stamp=clock.wall())))
    assert mon.last_reliable_wall() == pytest.approx(good - 0.3)  # unreliable and unknown states do not move it
    mon.on_map_guard_state(json.dumps(guard_state(stamp=clock.wall() - 60.0)))
    assert mon.last_reliable_wall() == pytest.approx(good - 0.3)  # a stale message is not evidence


def test_monitor_drops_an_unreadable_state() -> None:
    mon, _, _, _ = make()
    mon.on_map_guard_state(json.dumps(guard_state()))
    mon.on_map_guard_state("garbage")
    health, _ = mon.localisation()
    assert health is not None and health.state == "ok"


def map_guard_event(kind: str) -> str:
    """A map_guard /robot_events payload."""
    return json.dumps({"type": kind, "source": "map_guard", "severity": "warning", "message": "x", "data": {}})


def test_tilt_evidence_prefers_a_fresh_topic_state_over_events() -> None:
    mon, clock, seen, _ = make()
    mon.on_robot_event(map_guard_event("map_guard_resumed"))
    mon.on_map_guard_state(json.dumps(guard_state(state="settling", reasons=["lifted"])))
    clock.t += 0.5
    mon.on_tilt_guard(state("warning", roll=2.0, pitch=-19.0))
    assert seen[-1].data["map_guard_state"] == "settling"


def test_tilt_evidence_falls_back_to_events_when_the_topic_is_stale() -> None:
    mon, clock, seen, _ = make()
    mon.on_map_guard_state(json.dumps(guard_state(state="settling")))
    clock.t += MonitorSettings().map_guard_state_stale_s + 1.0
    mon.on_robot_event(map_guard_event("map_guard_disturbed"))
    mon.on_tilt_guard(state("warning", roll=2.0, pitch=-19.0))
    assert seen[-1].data["map_guard_state"] == "disturbed"


def test_config_defaults() -> None:
    cfg = McpServerConfig()
    assert cfg.topics.map_guard_state == "/map_guard/state"
    assert cfg.monitor.map_guard_state_stale_s == 3.0
    with pytest.raises(ValueError):
        MonitorSettings(map_guard_state_stale_s=0.0)


# --- ros_iface wiring ------------------------------------------------------------------------------------------------


def test_ros_iface_subscribes_the_latched_state_into_the_monitor(ros_iface: ModuleType) -> None:
    robot = ros_iface.RosRobot(McpServerConfig())
    robot.monitor = MagicMock()
    for call in robot.node.create_subscription.call_args_list:
        if call.args[1] == "/map_guard/state":
            qos = call.args[3]
            assert qos is ros_iface.MAP_GUARD_STATE_QOS
            call.args[2](MagicMock(data="payload"))
            robot.monitor.on_map_guard_state.assert_called_once_with("payload")
            return
    raise AssertionError("no subscription on /map_guard/state")


def test_robot_state_carries_the_localisation(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = ros_iface.RosRobot(McpServerConfig())
    mon, clock, _, _ = make()
    robot.monitor = mon
    monkeypatch.setattr(robot, "lookup_pose", lambda *_: None)
    robot.arm = MagicMock()
    robot.arm.state.return_value = ArmState(control_held=False, home_stored=False)
    robot.head = MagicMock()
    robot.head.angles_deg.return_value = None
    mon.on_map_guard_state(json.dumps(guard_state(agreement_ratio=0.03, median_ratio=0.03)))
    snapshot = robot.robot_state()
    assert snapshot.localisation is not None and snapshot.localisation.reliable is False
    assert any("relocalise" in note for note in snapshot.notes)
    clock.t += 10.0
    snapshot = robot.robot_state()
    assert snapshot.localisation is None and any("/map_guard/state" in note for note in snapshot.notes)
