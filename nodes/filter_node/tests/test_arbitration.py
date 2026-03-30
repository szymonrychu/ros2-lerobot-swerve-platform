"""Tests for filter_node source arbitration and leader takeover logic."""

from __future__ import annotations


class SourceArbiter:
    """Standalone arbitration logic extracted for testing (mirrors node.py logic)."""

    def __init__(
        self,
        web_ui_input_topic: str = "",
        web_ui_timeout_s: float = 0.5,
        follower_feedback_topic: str = "",
        takeover_threshold_rad: float = 0.15,
    ) -> None:
        self.web_ui_input_topic = web_ui_input_topic
        self.web_ui_timeout_s = web_ui_timeout_s
        self.follower_feedback_topic = follower_feedback_topic
        self.takeover_threshold_rad = takeover_threshold_rad
        self._active_source = "leader"
        self._last_web_ui_time = 0.0
        self._follower_positions: dict[str, float] = {}

    @property
    def active_source(self) -> str:
        return self._active_source

    def on_web_ui_input(self, now: float) -> None:
        self._active_source = "web_ui"
        self._last_web_ui_time = now

    def update_follower_positions(self, positions: dict[str, float]) -> None:
        self._follower_positions.update(positions)

    def should_accept_leader(self, leader_positions: dict[str, float], now: float) -> bool:
        """Returns True if leader input should be accepted; also transitions source.

        Args:
            leader_positions: Mapping of joint name to leader position (radians).
            now: Current monotonic timestamp (seconds).

        Returns:
            bool: True if leader input should be accepted and forwarded.
        """
        if not self.web_ui_input_topic or self._active_source != "web_ui":
            return True
        elapsed = now - self._last_web_ui_time
        if elapsed >= self.web_ui_timeout_s:
            self._active_source = "leader"
            return True
        # Web UI active: check proximity
        if not self.follower_feedback_topic or not self._follower_positions:
            return False
        for name, leader_pos in leader_positions.items():
            follower_pos = self._follower_positions.get(name)
            if follower_pos is None or abs(leader_pos - follower_pos) > self.takeover_threshold_rad:
                return False
        self._active_source = "leader"
        return True


def test_no_web_ui_topic_always_accepts_leader() -> None:
    arb = SourceArbiter()
    assert arb.should_accept_leader({"shoulder_pan": 0.5}, now=10.0)
    assert arb.active_source == "leader"


def test_web_ui_input_sets_source() -> None:
    arb = SourceArbiter(web_ui_input_topic="/filter/web_ui_joint_commands")
    arb.on_web_ui_input(now=10.0)
    assert arb.active_source == "web_ui"


def test_leader_rejected_during_web_ui_active_window() -> None:
    arb = SourceArbiter(
        web_ui_input_topic="/filter/web_ui_joint_commands",
        web_ui_timeout_s=0.5,
    )
    arb.on_web_ui_input(now=10.0)
    # 0.1s later — still within timeout, no feedback topic configured
    assert not arb.should_accept_leader({"shoulder_pan": 0.5}, now=10.1)
    assert arb.active_source == "web_ui"


def test_leader_accepted_after_web_ui_timeout() -> None:
    arb = SourceArbiter(
        web_ui_input_topic="/filter/web_ui_joint_commands",
        web_ui_timeout_s=0.5,
    )
    arb.on_web_ui_input(now=10.0)
    # 0.6s later — timeout expired
    assert arb.should_accept_leader({"shoulder_pan": 0.5}, now=10.6)
    assert arb.active_source == "leader"


def test_takeover_accepted_when_all_joints_within_threshold() -> None:
    arb = SourceArbiter(
        web_ui_input_topic="/filter/web_ui_joint_commands",
        web_ui_timeout_s=0.5,
        follower_feedback_topic="/follower/joint_states",
        takeover_threshold_rad=0.15,
    )
    arb.on_web_ui_input(now=10.0)
    arb.update_follower_positions({"shoulder_pan": 0.5, "shoulder_lift": 0.3})
    leader = {"shoulder_pan": 0.55, "shoulder_lift": 0.32}  # within 0.15 rad
    assert arb.should_accept_leader(leader, now=10.1)
    assert arb.active_source == "leader"


def test_takeover_rejected_when_any_joint_exceeds_threshold() -> None:
    arb = SourceArbiter(
        web_ui_input_topic="/filter/web_ui_joint_commands",
        web_ui_timeout_s=0.5,
        follower_feedback_topic="/follower/joint_states",
        takeover_threshold_rad=0.15,
    )
    arb.on_web_ui_input(now=10.0)
    arb.update_follower_positions({"shoulder_pan": 0.5, "shoulder_lift": 0.3})
    leader = {"shoulder_pan": 0.55, "shoulder_lift": 1.0}  # shoulder_lift too far
    assert not arb.should_accept_leader(leader, now=10.1)
    assert arb.active_source == "web_ui"


def test_takeover_rejected_when_follower_positions_empty() -> None:
    arb = SourceArbiter(
        web_ui_input_topic="/filter/web_ui_joint_commands",
        web_ui_timeout_s=0.5,
        follower_feedback_topic="/follower/joint_states",
    )
    arb.on_web_ui_input(now=10.0)
    # No follower positions received yet
    assert not arb.should_accept_leader({"shoulder_pan": 0.5}, now=10.1)


def test_takeover_rejected_without_feedback_topic() -> None:
    arb = SourceArbiter(
        web_ui_input_topic="/filter/web_ui_joint_commands",
        web_ui_timeout_s=0.5,
        follower_feedback_topic="",  # no feedback configured
    )
    arb.on_web_ui_input(now=10.0)
    assert not arb.should_accept_leader({"shoulder_pan": 0.5}, now=10.1)
