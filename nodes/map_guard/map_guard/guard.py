"""Map guard state machine.

    OK --(lifted / carried / rotated)--> DISTURBED (pause new measurements, notify)
    OK --(mismatch)--> EVALUATING (pause, notify; no settling: nobody is handling the robot)
    DISTURBED --(no detector fires)--> SETTLING --(settle_s quiet)--> EVALUATING
    SETTLING / EVALUATING --(any physical detector)--> DISTURBED
    EVALUATING --(evaluate_s; median ratio >= healthy_ratio, or no agreement sample at all)--> RESUMED (unpause) -> OK
    EVALUATING --(evaluate_s; median ratio < healthy_ratio)--> RELOCALISING (stay paused, ask map_manager to
        relocalise against the live map from the last pose before the move)
    RELOCALISING --(result localised)--> RESUMED (map and POIs kept)
    RELOCALISING --(result not localised / ambiguous / failed, or relocalise_timeout_s without an answer)-->
        RESET (serialize backup, prune, reset, unpause, publish the reset topic; map_manager has already cleared the
        live POIs) -> OK, with cooldown_s during which a mismatch cannot trigger

The slam_toolbox pause service is a toggle: the guard reads the paused_new_measurements parameter (falling back to
its own tracked state when the parameter cannot be read), toggles only when the state differs from the wanted one
and re-reads it afterwards. In dry run no service is ever called and the reset topic is never published; the state
machine, the logs and the metrics run unchanged.
"""

import statistics
from collections.abc import Callable, Sequence
from datetime import UTC, datetime
from enum import StrEnum
from pathlib import Path
from typing import Any, Protocol

from . import metrics
from .agent import EVENT_DISTURBED, EVENT_RESET, EVENT_RESUMED
from .agreement import MismatchMonitor
from .backup import backup_name, prune_backups
from .config import MapGuardConfig
from .detectors import REASON_CARRIED, REASON_LIFTED, REASON_MISMATCH, REASON_ROTATED, DisturbanceReport


class State(StrEnum):
    """Guard states (RESUMED and RESET last one tick; RELOCALISING lasts until the answer or the timeout)."""

    OK = "ok"
    DISTURBED = "disturbed"
    SETTLING = "settling"
    EVALUATING = "evaluating"
    RELOCALISING = "relocalising"
    RESUMED = "resumed"
    RESET = "reset"


class SlamPort(Protocol):
    """Blocking slam_toolbox calls (the node implements them with service clients)."""

    def query_paused(self) -> bool | None:
        """Read paused_new_measurements; None when it cannot be read."""

    def toggle_pause(self) -> bool:
        """Call the Pause toggle; True when the service answered with status true."""

    def serialize_map(self, filename: str) -> bool:
        """Serialize the pose graph and map to filename (no extension); True on RESULT_SUCCESS."""

    def reset(self) -> bool:
        """Reset the map and pose graph; True on RESULT_SUCCESS."""


class Notifier(Protocol):
    """Agent event sink (map_guard.agent.EventNotifier)."""

    def notify(
        self, now: float, kind: str, reason: str, ratio: float | None, data: dict[str, Any] | None = None
    ) -> Any:
        """Send an event if the agent is busy."""


def fmt(value: float | None) -> str:
    """Format a number for the logs.

    Args:
        value (float | None): Number or None.

    Returns:
        str: Two decimals, or "n/a".
    """
    return "n/a" if value is None else f"{value:.2f}"


class MapGuard:
    """Drives pause / resume / reset of slam_toolbox from the detector reports and agreement samples."""

    def __init__(
        self,
        config: MapGuardConfig,
        slam: SlamPort,
        notifier: Notifier,
        publish_reset: Callable[[], None],
        request_relocalise: Callable[[str, float], None],
        warn: Callable[[str], None],
        utc_now: Callable[[], datetime] = lambda: datetime.now(UTC),
    ) -> None:
        """Create the guard in state OK.

        Args:
            config (MapGuardConfig): Settings.
            slam (SlamPort): slam_toolbox calls (never used in dry run).
            notifier (Notifier): Agent events.
            publish_reset (Callable[[], None]): Publishes std_msgs/Empty on the reset topic (never in dry run).
            request_relocalise (Callable[[str, float], None]): Publishes the relocalise command to map_manager with
                (request_id, since: epoch seconds when the episode started); never called in dry run.
            warn (Callable[[str], None]): Warning logger.
            utc_now (Callable[[], datetime]): Wall clock for backup names.
        """
        self.config = config
        self.slam = slam
        self.notifier = notifier
        self.publish_reset = publish_reset
        self.request_relocalise = request_relocalise
        self.warn = warn
        self.utc_now = utc_now
        self.enabled = frozenset(config.reasons)
        self.monitor = MismatchMonitor(
            config.mismatch_ratio,
            config.healthy_ratio,
            config.mismatch_s,
            config.min_map_cells_known,
            config.median_window,
        )
        self.state = State.OK
        self.state_since = 0.0
        self.paused: bool | None = None
        self.episode_reasons: set[str] = set()
        self.evaluation: list[float] = []
        self.cooldown_until: float | None = None
        self.episode_started = 0.0
        self.last_now = 0.0
        self.request_seq = 0
        self.pending_request: str | None = None
        self.relocalise_deadline: float | None = None
        self.relocalise_median: float | None = None
        metrics.DRY_RUN.set(1 if config.dry_run else 0)
        self.set_state_gauge()

    @property
    def dry(self) -> str:
        """Log prefix: "[dry run] " in dry run."""
        return "[dry run] " if self.config.dry_run else ""

    @property
    def ratio(self) -> float | None:
        """Latest rolling median agreement."""
        return self.monitor.median

    def set_state_gauge(self) -> None:
        """Export 1 for the current state and 0 for the others."""
        for state in State:
            metrics.STATE.labels(state.value).set(1 if state is self.state else 0)

    def transition(self, now: float, state: State, reason: str, detail: str = "") -> None:
        """Enter state and log the transition at warn with the numbers.

        Args:
            now (float): Current time, s.
            state (State): New state.
            reason (str): Reason(s) of the episode.
            detail (str): Extra text for the log line.
        """
        self.warn(
            f"{self.dry}map_guard {self.state.value} -> {state.value}: reason {reason}, ratio {fmt(self.ratio)}"
            + (f", {detail}" if detail else "")
        )
        self.state = state
        self.state_since = now
        self.set_state_gauge()

    def tick(self, now: float, report: DisturbanceReport, samples: Sequence[tuple[float, int]]) -> None:
        """Advance the state machine.

        Args:
            now (float): Monotonic time, s.
            report (DisturbanceReport): Detector report at now.
            samples (Sequence[tuple[float, int]]): New (ratio, known_cells) agreement samples since the last tick.
        """
        self.last_now = now
        if report.tilt_deg is not None:
            metrics.TILT_DEG.set(report.tilt_deg)
        if report.unexplained_speed_mps is not None:
            metrics.UNEXPLAINED_SPEED.set(report.unexplained_speed_mps)
        mismatch = False
        for ratio, known_cells in samples:
            metrics.AGREEMENT_RATIO.set(ratio)
            mismatch = self.monitor.update(now, ratio, known_cells) or mismatch
            if self.state is State.EVALUATING:
                self.evaluation.append(ratio)
        in_cooldown = self.cooldown_until is not None and now < self.cooldown_until
        mismatch = mismatch and REASON_MISMATCH in self.enabled and not in_cooldown
        physical = set(report.reasons & self.enabled) - {REASON_MISMATCH}

        if self.state in (State.RESUMED, State.RESET):
            self.transition(now, State.OK, self.reason_text())
            self.episode_reasons = set()
        if self.state is State.OK:
            if physical:
                self.start_episode(now, physical, State.DISTURBED)
            elif mismatch:
                self.start_episode(now, {REASON_MISMATCH}, State.EVALUATING)
            return
        if physical:
            self.add_reasons(now, physical)
            if self.state is not State.DISTURBED:
                self.transition(now, State.DISTURBED, self.reason_text())
            return
        if self.state is State.DISTURBED:
            self.transition(now, State.SETTLING, self.reason_text())
        elif self.state is State.SETTLING and now - self.state_since >= self.config.settle_s - 1e-9:
            self.evaluation = []
            self.transition(now, State.EVALUATING, self.reason_text())
        elif self.state is State.EVALUATING and now - self.state_since >= self.config.evaluate_s - 1e-9:
            self.decide(now)
        elif (
            self.state is State.RELOCALISING
            and self.relocalise_deadline is not None
            and now >= self.relocalise_deadline - 1e-9
        ):
            self.warn(
                f"map_guard: no relocalise answer for {self.config.relocalise_timeout_s} s; treated as unrecoverable"
            )
            self.finish_relocalise(now, recovered=False, discarded_pois=0)

    def reason_text(self) -> str:
        """Reasons of the current episode, comma separated."""
        return ",".join(sorted(self.episode_reasons)) or "none"

    def primary_reason(self) -> str:
        """The reason reported to the agent: lifted/carried first (they stop the agent), then the others."""
        for reason in (REASON_LIFTED, REASON_CARRIED, REASON_ROTATED, REASON_MISMATCH):
            if reason in self.episode_reasons:
                return reason
        return REASON_MISMATCH

    def start_episode(self, now: float, reasons: set[str], state: State) -> None:
        """Leave OK: pause new measurements, count and report the triggers.

        Args:
            now (float): Current time, s.
            reasons (set[str]): Firing reasons.
            state (State): DISTURBED, or EVALUATING for a mismatch.
        """
        self.episode_reasons = set()
        self.evaluation = []
        self.episode_started = now
        self.add_reasons(now, reasons, log=False)
        self.transition(now, state, self.reason_text())
        metrics.PAUSES.inc()
        self.set_paused(True)

    def add_reasons(self, now: float, reasons: set[str], log: bool = True) -> None:
        """Count and report reasons new to the episode.

        Args:
            now (float): Current time, s.
            reasons (set[str]): Firing reasons.
            log (bool): Log the new reasons (False when a transition logs them).
        """
        for reason in sorted(reasons - self.episode_reasons):
            self.episode_reasons.add(reason)
            metrics.TRIGGERS.labels(reason).inc()
            if log:
                self.warn(f"{self.dry}map_guard {self.state.value}: also {reason}, ratio {fmt(self.ratio)}")
            self.notifier.notify(now, EVENT_DISTURBED, reason, self.ratio)

    def decide(self, now: float) -> None:
        """End of the evaluation window: resume, or ask map_manager to relocalise.

        Args:
            now (float): Current time, s.
        """
        reason = self.primary_reason()
        if not self.evaluation:
            self.transition(now, State.RESUMED, self.reason_text(), "no agreement sample (no map or too few points)")
            self.resume(now, reason, None)
            return
        median = statistics.median(self.evaluation)
        detail = f"evaluation median {median:.2f} over {len(self.evaluation)} samples (healthy >= {self.config.healthy_ratio})"
        if median >= self.config.healthy_ratio:
            self.transition(now, State.RESUMED, self.reason_text(), detail)
            self.resume(now, reason, median)
        else:
            self.transition(now, State.RELOCALISING, self.reason_text(), detail)
            self.start_relocalise(now, median)

    def start_relocalise(self, now: float, median: float) -> None:
        """Ask map_manager to relocalise (the guard stays paused), or log it in dry run and count it as recovered.

        Args:
            now (float): Current time, s.
            median (float): Poor evaluation median.
        """
        self.relocalise_median = median
        self.pending_request = None
        self.relocalise_deadline = None
        if self.config.dry_run:
            self.warn(
                f"{self.dry}map_guard would request relocalise on {self.config.command_topic}; counted as recovered"
            )
            self.transition(now, State.RESUMED, self.reason_text(), "dry run: relocalise assumed to succeed")
            self.resume(now, self.primary_reason(), median)
            return
        self.request_seq += 1
        self.pending_request = f"map_guard-{self.request_seq}"
        self.relocalise_deadline = now + self.config.relocalise_timeout_s
        since = self.utc_now().timestamp() - (now - self.episode_started)
        self.request_relocalise(self.pending_request, since)

    def on_relocalise_result(self, request_id: str, ok: bool, state: str, discarded_pois: int) -> None:
        """Handle map_manager's answer to the pending relocalise request.

        Args:
            request_id (str): Request id echoed by map_manager; answers to other ids are ignored.
            ok (bool): The result's ok flag.
            state (str): Localisation state ("localised", "not_localised", "ambiguous", ...).
            discarded_pois (int): POIs map_manager cleared from the live set (0 when none).
        """
        if self.state is not State.RELOCALISING or request_id != self.pending_request:
            return
        self.finish_relocalise(self.last_now, ok and state == "localised", discarded_pois)

    def finish_relocalise(self, now: float, recovered: bool, discarded_pois: int) -> None:
        """Leave RELOCALISING: resume when the pose is recovered, otherwise run the live map reset.

        Args:
            now (float): Current time, s.
            recovered (bool): The robot found itself in the live map.
            discarded_pois (int): POIs map_manager cleared on failure.
        """
        reason, median = self.primary_reason(), self.relocalise_median
        self.pending_request = None
        self.relocalise_deadline = None
        if recovered:
            self.transition(now, State.RESUMED, self.reason_text(), "relocalised against the live map")
            self.resume(now, reason, median)
        else:
            self.transition(now, State.RESET, self.reason_text(), f"unrecoverable, {discarded_pois} POIs discarded")
            self.reset_map(now, reason, median if median is not None else 0.0, discarded_pois)

    def resume(self, now: float, reason: str, median: float | None) -> None:
        """Unpause and report.

        Args:
            now (float): Current time, s.
            reason (str): Primary reason.
            median (float | None): Evaluation median.
        """
        self.set_paused(False)
        self.monitor.reset_hold()
        metrics.RESUMES.inc()
        self.notifier.notify(now, EVENT_RESUMED, reason, median)

    def reset_map(self, now: float, reason: str, median: float, discarded_pois: int = 0) -> None:
        """Back up, reset slam_toolbox, unpause, publish the reset topic, start the cooldown (never reached in dry run).

        A failed backup skips the reset (the map is never dropped without a copy); the cooldown applies either way.

        Args:
            now (float): Current time, s.
            reason (str): Primary reason.
            median (float): Evaluation median.
            discarded_pois (int): POIs map_manager cleared before the reset (reported in the event).
        """
        cfg = self.config
        target = str(Path(cfg.backup_dir) / backup_name(cfg.backup_prefix, self.utc_now()))
        metrics.RESETS.labels("true" if cfg.dry_run else "false").inc()
        self.cooldown_until = now + cfg.cooldown_s
        self.monitor.reset()
        if not self.slam.serialize_map(target):
            self.warn(f"map_guard: serialize_map to {target} failed; map NOT reset")
            self.set_paused(False)
            return
        for path in prune_backups(Path(cfg.backup_dir), cfg.backup_prefix, cfg.keep_backups):
            self.warn(f"map_guard: deleted old backup {path}")
        if not self.slam.reset():
            self.warn("map_guard: slam_toolbox reset failed")
            self.set_paused(False)
            return
        self.set_paused(False)
        self.publish_reset()
        self.notifier.notify(now, EVENT_RESET, reason, median, {"backup": target, "discarded_pois": discarded_pois})

    def set_paused(self, desired: bool) -> None:
        """Bring slam_toolbox's new-measurement pause to desired through the toggle service.

        Args:
            desired (bool): True to pause, False to resume.
        """
        if self.config.dry_run:
            self.warn(f"{self.dry}map_guard would {'pause' if desired else 'unpause'} new measurements")
            self.paused = desired
            return
        actual = self.slam.query_paused()
        current = actual if actual is not None else self.paused
        if current == desired:
            self.paused = desired
            return
        if not self.slam.toggle_pause():
            self.warn(f"map_guard: pause toggle failed; new measurements stay {'paused' if current else 'active'}")
            self.paused = current
            return
        verified = self.slam.query_paused()
        if verified is not None and verified != desired:
            self.warn(
                f"map_guard: pause toggle answered but new measurements are still {'paused' if verified else 'active'}"
            )
            self.paused = verified
            return
        self.paused = desired
