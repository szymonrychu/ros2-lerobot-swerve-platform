"""Command source arbitration for the filter node (pure Python, no rclpy).

Priority is autonomy > web_ui > leader:

- **autonomy**: the first autonomy command takes a sticky lease (no timeout). While the lease is held,
  web UI and leader input are ignored. Only an explicit release (Bool ``true``) ends it.
- After a release the active source is ``none``: web UI commands take over immediately; the leader resumes
  only when every joint is within ``takeover_threshold_rad`` of the follower feedback, so the follower never
  snaps to the leader pose. Without follower feedback the leader cannot resume after a release.
- **web_ui**: active for ``web_ui_timeout_s`` after its last command; meanwhile the leader takes over only
  via the proximity rule. After the timeout the leader resumes (legacy behaviour).
"""

from typing import NamedTuple

SOURCE_LEADER = "leader"
SOURCE_WEB_UI = "web_ui"
SOURCE_AUTONOMY = "autonomy"
SOURCE_NONE = "none"
ACTIVE_SOURCE_PERIOD_S = 1.0


class LeaderDecision(NamedTuple):
    """Outcome of a leader input message.

    Attributes:
        accepted (bool): True when the leader message should update the filter.
        resumed_after_release (bool): True when this message handed control back to the leader after an
            autonomy release; the caller should reset filter state so stale estimates are not published.
    """

    accepted: bool
    resumed_after_release: bool = False


def joints_within_threshold(
    leader_positions: dict[str, float],
    follower_positions: dict[str, float],
    threshold_rad: float,
) -> bool:
    """Check that every leader joint is within threshold of the follower feedback.

    Args:
        leader_positions (dict[str, float]): Leader joint name -> position (rad).
        follower_positions (dict[str, float]): Follower joint name -> position (rad).
        threshold_rad (float): Maximum allowed absolute difference (rad).

    Returns:
        bool: True if all leader joints have follower feedback within threshold_rad.
    """
    return all(
        abs(pos - follower_positions.get(name, float("inf"))) <= threshold_rad for name, pos in leader_positions.items()
    )


class SourceArbiter:
    """Decides which command source (autonomy, web_ui, leader) drives the follower."""

    def __init__(
        self,
        web_ui_timeout_s: float = 0.5,
        takeover_threshold_rad: float = 0.15,
        proximity_check_enabled: bool = False,
    ) -> None:
        """Create an arbiter starting in the leader state.

        Args:
            web_ui_timeout_s (float): Seconds after the last web UI command before it counts as idle.
            takeover_threshold_rad (float): Max per-joint leader/follower difference for a leader takeover.
            proximity_check_enabled (bool): True when follower feedback is configured; without it the
                proximity rule can never pass.
        """
        self.web_ui_timeout_s = web_ui_timeout_s
        self.takeover_threshold_rad = takeover_threshold_rad
        self.proximity_check_enabled = proximity_check_enabled
        self.active_source: str = SOURCE_LEADER
        self.last_web_ui_time = 0.0
        self.follower_positions: dict[str, float] = {}

    @property
    def autonomy_held(self) -> bool:
        """Whether the autonomy lease is currently held.

        Returns:
            bool: True while autonomy owns the follower.
        """
        return self.active_source == SOURCE_AUTONOMY

    def update_follower_positions(self, positions: dict[str, float]) -> None:
        """Merge the latest follower feedback positions.

        Args:
            positions (dict[str, float]): Joint name -> follower position (rad).
        """
        self.follower_positions.update(positions)

    def leader_close_to_follower(self, leader_positions: dict[str, float]) -> bool:
        """Apply the proximity rule.

        Args:
            leader_positions (dict[str, float]): Leader joint name -> position (rad).

        Returns:
            bool: True if follower feedback exists and all leader joints are within threshold.
        """
        if not self.proximity_check_enabled or not self.follower_positions:
            return False
        return joints_within_threshold(leader_positions, self.follower_positions, self.takeover_threshold_rad)

    def on_autonomy_command(self) -> bool:
        """Handle an autonomy command: take (or keep) the sticky lease.

        Returns:
            bool: Always True; autonomy commands are always forwarded.
        """
        self.active_source = SOURCE_AUTONOMY
        return True

    def on_autonomy_release(self, release: bool) -> bool:
        """Handle a release message.

        Args:
            release (bool): Bool payload; only True releases the lease.

        Returns:
            bool: True if the lease was held and has now ended.
        """
        if not release or not self.autonomy_held:
            return False
        self.active_source = SOURCE_NONE
        return True

    def on_web_ui_command(self, now: float) -> bool:
        """Handle a web UI command.

        Args:
            now (float): Monotonic time (s).

        Returns:
            bool: True if the command should be forwarded (False while autonomy holds the lease).
        """
        if self.autonomy_held:
            return False
        self.active_source = SOURCE_WEB_UI
        self.last_web_ui_time = now
        return True

    def on_leader_input(self, leader_positions: dict[str, float], now: float) -> LeaderDecision:
        """Handle a leader input message and transition the source when the leader takes over.

        Args:
            leader_positions (dict[str, float]): Leader joint name -> position (rad).
            now (float): Monotonic time (s).

        Returns:
            LeaderDecision: Whether to accept the message and whether it resumed control after a release.
        """
        if self.active_source == SOURCE_LEADER:
            return LeaderDecision(accepted=True)
        if self.autonomy_held:
            return LeaderDecision(accepted=False)
        if self.active_source == SOURCE_NONE:
            if not self.leader_close_to_follower(leader_positions):
                return LeaderDecision(accepted=False)
            self.active_source = SOURCE_LEADER
            return LeaderDecision(accepted=True, resumed_after_release=True)
        # web_ui
        if now - self.last_web_ui_time >= self.web_ui_timeout_s or self.leader_close_to_follower(leader_positions):
            self.active_source = SOURCE_LEADER
            return LeaderDecision(accepted=True)
        return LeaderDecision(accepted=False)

    def should_publish_filtered(self, now: float) -> bool:
        """Whether the control loop may publish the filtered (leader) output.

        Args:
            now (float): Monotonic time (s).

        Returns:
            bool: False while autonomy holds the lease, after a release until the leader resumes, and while
            the web UI is active; True otherwise.
        """
        if self.active_source == SOURCE_WEB_UI:
            return now - self.last_web_ui_time >= self.web_ui_timeout_s
        return self.active_source == SOURCE_LEADER


class ActiveSourceReporter:
    """Decides when to publish the active source: on change and every period_s."""

    def __init__(self, period_s: float = ACTIVE_SOURCE_PERIOD_S) -> None:
        """Create a reporter that has not published yet.

        Args:
            period_s (float): Republish period (s) when the source does not change.
        """
        self.period_s = period_s
        self.last_source: str | None = None
        self.last_time = 0.0

    def due(self, source: str, now: float) -> bool:
        """Check whether source should be published now; records it as published if so.

        Args:
            source (str): Current active source.
            now (float): Monotonic time (s).

        Returns:
            bool: True if the source changed or period_s elapsed since the last publish.
        """
        if source != self.last_source or now - self.last_time >= self.period_s:
            self.last_source = source
            self.last_time = now
            return True
        return False
