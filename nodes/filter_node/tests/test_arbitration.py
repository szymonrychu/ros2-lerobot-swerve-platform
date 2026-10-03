"""Tests for filter_node command source arbitration (filter_node.arbitration, rclpy-free)."""

from __future__ import annotations

from filter_node.arbitration import (
    SOURCE_AUTONOMY,
    SOURCE_LEADER,
    SOURCE_NONE,
    SOURCE_WEB_UI,
    ActiveSourceReporter,
    SourceArbiter,
)

FOLLOWER = {"shoulder_pan": 0.5, "shoulder_lift": 0.3}
LEADER_CLOSE = {"shoulder_pan": 0.55, "shoulder_lift": 0.32}  # within 0.15 rad
LEADER_FAR = {"shoulder_pan": 0.55, "shoulder_lift": 1.0}  # shoulder_lift too far


def make_arbiter(proximity: bool = True) -> SourceArbiter:
    """Build an arbiter with the default timeout/threshold and optional follower feedback.

    Args:
        proximity (bool): Whether follower feedback (proximity check) is configured.

    Returns:
        SourceArbiter: Fresh arbiter in the leader state.
    """
    arb = SourceArbiter(
        web_ui_timeout_s=0.5,
        takeover_threshold_rad=0.15,
        proximity_check_enabled=proximity,
    )
    if proximity:
        arb.update_follower_positions(FOLLOWER)
    return arb


# --- existing leader / web_ui behaviour (autonomy unused) -------------------


def test_initial_source_is_leader_and_leader_accepted() -> None:
    arb = make_arbiter(proximity=False)
    assert arb.active_source == SOURCE_LEADER
    assert arb.on_leader_input({"shoulder_pan": 0.5}, now=10.0).accepted
    assert arb.should_publish_filtered(now=10.0)


def test_web_ui_input_sets_source_and_is_forwarded() -> None:
    arb = make_arbiter(proximity=False)
    assert arb.on_web_ui_command(now=10.0)
    assert arb.active_source == SOURCE_WEB_UI


def test_leader_rejected_during_web_ui_active_window() -> None:
    arb = make_arbiter(proximity=False)
    arb.on_web_ui_command(now=10.0)
    assert not arb.on_leader_input({"shoulder_pan": 0.5}, now=10.1).accepted
    assert arb.active_source == SOURCE_WEB_UI
    assert not arb.should_publish_filtered(now=10.1)


def test_leader_accepted_after_web_ui_timeout() -> None:
    arb = make_arbiter(proximity=False)
    arb.on_web_ui_command(now=10.0)
    assert arb.should_publish_filtered(now=10.6)
    decision = arb.on_leader_input({"shoulder_pan": 0.5}, now=10.6)
    assert decision.accepted
    assert not decision.resumed_after_release
    assert arb.active_source == SOURCE_LEADER


def test_takeover_accepted_when_all_joints_within_threshold() -> None:
    arb = make_arbiter()
    arb.on_web_ui_command(now=10.0)
    assert arb.on_leader_input(LEADER_CLOSE, now=10.1).accepted
    assert arb.active_source == SOURCE_LEADER


def test_takeover_rejected_when_any_joint_exceeds_threshold() -> None:
    arb = make_arbiter()
    arb.on_web_ui_command(now=10.0)
    assert not arb.on_leader_input(LEADER_FAR, now=10.1).accepted
    assert arb.active_source == SOURCE_WEB_UI


def test_takeover_rejected_when_follower_positions_empty() -> None:
    arb = SourceArbiter(web_ui_timeout_s=0.5, takeover_threshold_rad=0.15, proximity_check_enabled=True)
    arb.on_web_ui_command(now=10.0)
    assert not arb.on_leader_input({"shoulder_pan": 0.5}, now=10.1).accepted


def test_takeover_rejected_when_follower_joint_missing() -> None:
    arb = make_arbiter()
    arb.on_web_ui_command(now=10.0)
    assert not arb.on_leader_input({"shoulder_pan": 0.5, "gripper": 0.1}, now=10.1).accepted


# --- autonomy lease ----------------------------------------------------------


def test_autonomy_preempts_leader() -> None:
    arb = make_arbiter()
    assert arb.on_autonomy_command()
    assert arb.active_source == SOURCE_AUTONOMY
    assert arb.autonomy_held
    assert not arb.on_leader_input(LEADER_CLOSE, now=10.0).accepted
    assert not arb.should_publish_filtered(now=10.0)


def test_autonomy_preempts_web_ui() -> None:
    arb = make_arbiter()
    arb.on_web_ui_command(now=10.0)
    assert arb.on_autonomy_command()
    assert arb.active_source == SOURCE_AUTONOMY
    assert not arb.on_web_ui_command(now=10.1)
    assert arb.active_source == SOURCE_AUTONOMY


def test_autonomy_is_sticky_without_messages() -> None:
    arb = make_arbiter()
    arb.on_autonomy_command()
    # An hour with no autonomy messages: still held, nothing else may publish.
    assert arb.autonomy_held
    assert not arb.should_publish_filtered(now=3600.0)
    assert not arb.on_leader_input(LEADER_CLOSE, now=3600.0).accepted
    assert not arb.on_web_ui_command(now=3600.0)
    assert arb.active_source == SOURCE_AUTONOMY


def test_web_ui_and_leader_ignored_while_held() -> None:
    arb = make_arbiter(proximity=False)
    arb.on_autonomy_command()
    for t in (10.0, 10.6, 20.0):
        assert not arb.on_web_ui_command(now=t)
        assert not arb.on_leader_input({"shoulder_pan": 0.5}, now=t).accepted
        assert arb.active_source == SOURCE_AUTONOMY


def test_release_false_is_ignored() -> None:
    arb = make_arbiter()
    arb.on_autonomy_command()
    assert not arb.on_autonomy_release(False)
    assert arb.autonomy_held
    assert arb.active_source == SOURCE_AUTONOMY


def test_release_without_lease_is_noop() -> None:
    arb = make_arbiter()
    assert not arb.on_autonomy_release(True)
    assert arb.active_source == SOURCE_LEADER
    assert arb.on_leader_input(LEADER_FAR, now=10.0).accepted


def test_release_ends_lease_and_nobody_publishes_filtered() -> None:
    arb = make_arbiter()
    arb.on_autonomy_command()
    assert arb.on_autonomy_release(True)
    assert not arb.autonomy_held
    assert arb.active_source == SOURCE_NONE
    # Stale leader Kalman state must not be published (it would snap the follower).
    assert not arb.should_publish_filtered(now=100.0)


def test_release_then_web_ui_works_immediately() -> None:
    arb = make_arbiter()
    arb.on_autonomy_command()
    arb.on_autonomy_release(True)
    assert arb.on_web_ui_command(now=10.0)
    assert arb.active_source == SOURCE_WEB_UI


def test_release_then_leader_only_after_proximity() -> None:
    arb = make_arbiter()
    arb.on_autonomy_command()
    arb.on_autonomy_release(True)
    # Far leader is ignored no matter how long we wait (no timeout path after release).
    assert not arb.on_leader_input(LEADER_FAR, now=10.0).accepted
    assert not arb.on_leader_input(LEADER_FAR, now=1000.0).accepted
    assert arb.active_source == SOURCE_NONE
    decision = arb.on_leader_input(LEADER_CLOSE, now=1000.1)
    assert decision.accepted
    assert decision.resumed_after_release
    assert arb.active_source == SOURCE_LEADER
    assert arb.should_publish_filtered(now=1000.1)
    # Subsequent leader messages are plain accepts.
    assert not arb.on_leader_input(LEADER_FAR, now=1000.2).resumed_after_release


def test_release_then_leader_blocked_without_follower_feedback() -> None:
    arb = make_arbiter(proximity=False)
    arb.on_autonomy_command()
    arb.on_autonomy_release(True)
    assert not arb.on_leader_input({"shoulder_pan": 0.5}, now=10.0).accepted
    assert arb.active_source == SOURCE_NONE


def test_autonomy_can_retake_lease_after_release() -> None:
    arb = make_arbiter()
    arb.on_autonomy_command()
    arb.on_autonomy_release(True)
    arb.on_web_ui_command(now=10.0)
    assert arb.on_autonomy_command()
    assert arb.active_source == SOURCE_AUTONOMY
    assert not arb.on_web_ui_command(now=10.1)


def test_follower_positions_update_merges() -> None:
    arb = SourceArbiter(web_ui_timeout_s=0.5, takeover_threshold_rad=0.15, proximity_check_enabled=True)
    arb.update_follower_positions({"shoulder_pan": 0.5})
    arb.update_follower_positions({"shoulder_lift": 0.3})
    arb.on_web_ui_command(now=10.0)
    assert arb.on_leader_input(LEADER_CLOSE, now=10.1).accepted


# --- active source reporting ---------------------------------------------------


def test_reporter_publishes_first_value_immediately() -> None:
    rep = ActiveSourceReporter(period_s=1.0)
    assert rep.due(SOURCE_LEADER, now=0.0)


def test_reporter_publishes_on_change() -> None:
    rep = ActiveSourceReporter(period_s=1.0)
    rep.due(SOURCE_LEADER, now=0.0)
    assert not rep.due(SOURCE_LEADER, now=0.1)
    assert rep.due(SOURCE_AUTONOMY, now=0.2)
    assert not rep.due(SOURCE_AUTONOMY, now=0.3)
    assert rep.due(SOURCE_NONE, now=0.4)


def test_reporter_publishes_periodically() -> None:
    rep = ActiveSourceReporter(period_s=1.0)
    rep.due(SOURCE_LEADER, now=0.0)
    assert not rep.due(SOURCE_LEADER, now=0.99)
    assert rep.due(SOURCE_LEADER, now=1.0)
    assert not rep.due(SOURCE_LEADER, now=1.5)
    assert rep.due(SOURCE_LEADER, now=2.0)


def test_active_source_transitions_full_cycle() -> None:
    arb = make_arbiter()
    rep = ActiveSourceReporter(period_s=1.0)
    seen: list[str] = []

    def observe(now: float) -> None:
        if rep.due(arb.active_source, now):
            seen.append(arb.active_source)

    observe(0.0)
    arb.on_web_ui_command(now=0.1)
    observe(0.1)
    arb.on_autonomy_command()
    observe(0.2)
    arb.on_autonomy_release(True)
    observe(0.3)
    arb.on_leader_input(LEADER_CLOSE, now=0.4)
    observe(0.4)
    assert seen == [
        SOURCE_LEADER,
        SOURCE_WEB_UI,
        SOURCE_AUTONOMY,
        SOURCE_NONE,
        SOURCE_LEADER,
    ]
