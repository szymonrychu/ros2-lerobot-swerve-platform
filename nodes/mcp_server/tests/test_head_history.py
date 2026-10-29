"""Head joint history: pan/tilt interpolated at an image stamp, refused when the stamp is not covered or the head moved."""

import math

import pytest

from mcp_server.config import HeadSettings, McpServerConfig
from mcp_server.head import AppliedHeadKinematics, HeadController, HeadHistory, HeadHistoryError
from mcp_server.head_kinematics import DEFAULT_HEAD_URDF
from tests.fakes import FakeHeadBackend

RATE_HZ = 50.0
T0 = 1000.0


def settings(**overrides: float) -> HeadSettings:
    return HeadSettings(**overrides)


def history(**overrides: float) -> HeadHistory:
    return HeadHistory(settings(**overrides))


def feed(hist: HeadHistory, start: float, stop: float, pan, tilt=lambda t: 0.5) -> None:
    """Add samples at RATE_HZ from start to stop (inclusive) with pan(t) and tilt(t) in rad."""
    n = round((stop - start) * RATE_HZ)
    for i in range(n + 1):
        t = start + i / RATE_HZ
        hist.add(t, {"head_pan": pan(t), "head_tilt": tilt(t)})


def test_static_head_pose_at_a_sample_stamp() -> None:
    hist = history()
    feed(hist, T0, T0 + 2.0, lambda t: 0.3)
    pose = hist.pose_at(T0 + 1.0)
    assert (pose.pan, pose.tilt) == pytest.approx((0.3, 0.5))
    assert pose.moving is False and pose.window_motion_rad == pytest.approx(0.0, abs=1e-12)
    assert pose.stamp_s == T0 + 1.0


def test_interpolates_between_two_samples() -> None:
    hist = history(max_motion_rad=1.0)  # the ramp is not what this test is about
    hist.add(T0, {"head_pan": 0.0, "head_tilt": 0.2})
    hist.add(T0 + 0.02, {"head_pan": 0.01, "head_tilt": 0.4})
    pose = hist.pose_at(T0 + 0.005)
    assert pose.pan == pytest.approx(0.0025) and pose.tilt == pytest.approx(0.25)
    assert pose.nearest_gap_s == pytest.approx(0.005)


def test_moving_head_is_flagged_within_the_stamp_window() -> None:
    hist = history()
    feed(hist, T0, T0 + 2.0, lambda t: 0.5 * (t - T0))  # 0.5 rad/s: 0.05 rad over the +-0.1 s window
    pose = hist.pose_at(T0 + 1.0)
    assert pose.moving is True
    assert pose.window_motion_rad == pytest.approx(0.1, abs=0.01)
    assert pose.pan == pytest.approx(0.5)


def test_head_that_stopped_before_the_window_is_not_moving() -> None:
    hist = history()
    feed(hist, T0, T0 + 1.0, lambda t: 0.5 * min(t - T0, 0.5))  # stops at T0 + 0.5
    feed(hist, T0 + 1.0 + 1 / RATE_HZ, T0 + 2.0, lambda t: 0.25)
    assert hist.pose_at(T0 + 1.5).moving is False
    assert hist.pose_at(T0 + 0.45).moving is True


def test_motion_threshold_is_configurable() -> None:
    hist = history(max_motion_rad=0.5)
    feed(hist, T0, T0 + 2.0, lambda t: 0.5 * (t - T0))
    assert hist.pose_at(T0 + 1.0).moving is False


def test_empty_history_is_refused_naming_the_topic() -> None:
    with pytest.raises(HeadHistoryError, match="no .*head/joint_states"):
        history().pose_at(T0)


def test_stamp_before_the_oldest_sample_is_refused() -> None:
    hist = history()
    feed(hist, T0, T0 + 1.0, lambda t: 0.1)
    with pytest.raises(HeadHistoryError, match="older than the oldest"):
        hist.pose_at(T0 - 0.5)


def test_stamp_after_the_newest_sample_is_refused_beyond_the_gap() -> None:
    hist = history()
    feed(hist, T0, T0 + 1.0, lambda t: 0.1)
    with pytest.raises(HeadHistoryError, match="newest"):
        hist.pose_at(T0 + 1.0 + 0.5)


def test_stamp_just_after_the_newest_sample_holds_a_static_head() -> None:
    hist = history()
    feed(hist, T0, T0 + 1.0, lambda t: 0.1)
    pose = hist.pose_at(T0 + 1.0 + 0.5 / RATE_HZ)
    assert pose.pan == pytest.approx(0.1) and pose.moving is False


def test_stamp_after_the_newest_sample_with_a_moving_head_is_flagged() -> None:
    hist = history()
    feed(hist, T0, T0 + 1.0, lambda t: 1.0 * (t - T0))
    pose = hist.pose_at(T0 + 1.0 + 0.5 / RATE_HZ)
    assert pose.moving is True


def test_gap_between_samples_larger_than_the_limit_is_refused() -> None:
    hist = history()
    hist.add(T0, {"head_pan": 0.0, "head_tilt": 0.0})
    hist.add(T0 + 1.0, {"head_pan": 0.0, "head_tilt": 0.0})
    with pytest.raises(HeadHistoryError, match="gap"):
        hist.pose_at(T0 + 0.5)


def test_old_samples_are_dropped_after_history_s() -> None:
    hist = history(history_s=2.0)
    feed(hist, T0, T0 + 10.0, lambda t: 0.1)
    assert hist.oldest_stamp() >= T0 + 8.0 - 1 / RATE_HZ
    assert len(hist) <= 2.0 * RATE_HZ + 2
    with pytest.raises(HeadHistoryError, match="older than the oldest"):
        hist.pose_at(T0 + 1.0)


def test_out_of_order_sample_is_inserted_in_order() -> None:
    hist = history(max_motion_rad=1.0)
    hist.add(T0, {"head_pan": 0.0, "head_tilt": 0.0})
    hist.add(T0 + 0.04, {"head_pan": 0.04, "head_tilt": 0.0})
    hist.add(T0 + 0.02, {"head_pan": 0.02, "head_tilt": 0.0})
    assert hist.pose_at(T0 + 0.03).pan == pytest.approx(0.03)


def test_sample_missing_a_head_joint_is_ignored() -> None:
    hist = history()
    hist.add(T0, {"head_pan": 0.0})
    assert len(hist) == 0


def test_non_finite_sample_is_ignored() -> None:
    hist = history()
    hist.add(T0, {"head_pan": math.nan, "head_tilt": 0.0})
    assert len(hist) == 0


def test_controller_pose_at_uses_its_own_history() -> None:
    cfg = McpServerConfig()
    head = HeadController(FakeHeadBackend(), AppliedHeadKinematics(DEFAULT_HEAD_URDF, None), cfg.head)
    feed(head.history, T0, T0 + 1.0, lambda t: -0.2)
    pose = head.pose_at(T0 + 0.5)
    assert pose.pan == pytest.approx(-0.2) and not pose.moving


def test_recent_motion_of_the_newest_samples() -> None:
    hist = history()
    assert hist.recent_motion() is None  # no samples
    hist.add(T0, {"head_pan": 0.0, "head_tilt": 0.5})
    assert hist.recent_motion() is None  # one sample cannot tell
    feed(hist, T0 + 0.02, T0 + 4.0, lambda t: 0.3)  # the first sample (pan 0) is long outside the window
    assert hist.recent_motion() == pytest.approx(0.0, abs=1e-12)
    feed(hist, T0 + 4.02, T0 + 5.0, lambda t: 0.3 + 0.5 * (t - T0 - 4.0))
    assert hist.recent_motion() == pytest.approx(0.05, abs=0.01)  # 0.5 rad/s over the 0.1 s window


def test_controller_recent_motion_rad_delegates_to_the_history() -> None:
    head = HeadController(FakeHeadBackend(), AppliedHeadKinematics(DEFAULT_HEAD_URDF, None), McpServerConfig().head)
    feed(head.history, T0, T0 + 1.0, lambda t: 0.5 * (t - T0))
    assert head.recent_motion_rad() == pytest.approx(0.05, abs=0.01)


def test_defaults() -> None:
    s = McpServerConfig().head
    assert (s.history_s, s.stamp_window_s, s.max_motion_rad, s.max_sample_gap_s) == (10.0, 0.1, 0.002, 0.25)
