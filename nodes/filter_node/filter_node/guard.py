"""Arm self-collision guard: every follower command is checked against the robot body before filter_node publishes it.

The guard keeps the arm's last known pose: the last command it published, replaced by every follower feedback sample
(seed), whichever came last. A request is the straight joint-space segment from that pose to the requested one, sampled
at most max_step_rad per joint and checked by the shared SelfCollisionChecker (arm_collision) with the head posed where
/head/joint_states puts it (the full head envelope without a fresh head state). A start already inside the hard margin
may move out but not deeper (allow_escape). A leader command's gripper is in leader radians: the follower bridge maps
it onto the follower range (leader_gripper), so it is checked mapped and a clamp goes out mapped back. With no
violation the request is published as is; otherwise the furthest safe sample before the first violating one, or
nothing when that sample is the start (the arm holds).
"""

import json
import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any, Protocol

from arm_collision.head import HeadPose
from arm_collision.ik import ARM_CHAIN_JOINTS
from feetech_mapping.command_mapping import RADIANS_PER_STEP, STEP_CENTER, map_position_to_steps, steps_to_radians

from .arbitration import SOURCE_LEADER
from .config import LeaderGripperSettings

# Obstacle label of a refusal whose arm pose is unknown (a partial request before any command or feedback).
UNKNOWN_POSE = "unknown_pose"
# A blocked guard state is republished this often (s) while commands keep arriving blocked.
GUARD_STATE_PERIOD_S = 1.0
# A blocked guard state with no new blocked command for this long (s) reports clear again.
GUARD_STATE_HOLD_S = 2.0
# Error text of a request with a NaN or infinite position (refused like a failed check).
NON_FINITE_ERROR = "non-finite position in the request"


class BodyCheck(Protocol):
    """What the guard needs of SelfCollisionChecker: a checker posed at the head pose and its path check."""

    def posed(self, head: HeadPose | None) -> "BodyCheck":
        """Checker with the head at a measured pose (None: the full envelope)."""
        ...

    def path(self, samples: list[dict[str, float]], allow_escape: bool = True) -> Any:
        """BodyReport of the samples (samples[0] = start)."""
        ...


@dataclass(frozen=True)
class GuardResult:
    """Verdict on one request.

    Attributes:
        positions: Positions to publish, in the request's joint order (the request or its clamp); None publishes
            nothing.
        blocked: True when the request broke the hard margin (clamped or refused).
        link: Arm part that breaks the margin ('' when not blocked).
        obstacle: Body obstacle it breaks it against (UNKNOWN_POSE for an unknown pose; '' when not blocked).
        clearance_m: Clearance of the violating sample (m; inf when not blocked).
        check_seconds: Wall time of the check (s).
        error: Checker exception text when the check failed (positions None, not blocked); '' otherwise.
    """

    positions: list[float] | None
    blocked: bool = False
    link: str = ""
    obstacle: str = ""
    clearance_m: float = math.inf
    check_seconds: float = 0.0
    error: str = ""


def leader_to_follower(leader_rad: float, mapping: LeaderGripperSettings) -> float:
    """Follower radians the bridge drives a leader gripper command to (its own map_position_to_steps).

    Args:
        leader_rad (float): Leader gripper position (rad).
        mapping (LeaderGripperSettings): The bridge's gripper range mapping.

    Returns:
        float: Follower gripper position (rad, as /follower/joint_states reports it).
    """
    steps = map_position_to_steps(
        leader_rad,
        mapping.source_min_steps,
        mapping.source_max_steps,
        mapping.command_min_steps,
        mapping.command_max_steps,
        source_inverted=mapping.source_inverted,
    )
    return steps_to_radians(steps, mapping.inverted)


def follower_to_leader(follower_rad: float, mapping: LeaderGripperSettings) -> float:
    """Leader radians the bridge maps onto a follower gripper position (the inverse of leader_to_follower, clamped to
    the follower command range).

    Args:
        follower_rad (float): Follower gripper position (rad).
        mapping (LeaderGripperSettings): The bridge's gripper range mapping.

    Returns:
        float: Leader gripper position (rad).
    """
    steps = (-follower_rad if mapping.inverted else follower_rad) / RADIANS_PER_STEP + STEP_CENTER
    span = mapping.command_max_steps - mapping.command_min_steps
    progress = min(1.0, max(0.0, (steps - mapping.command_min_steps) / span))
    if mapping.source_inverted:
        progress = 1.0 - progress
    leader_steps = mapping.source_min_steps + progress * (mapping.source_max_steps - mapping.source_min_steps)
    return (leader_steps - STEP_CENTER) * RADIANS_PER_STEP


def segment(start: dict[str, float], end: dict[str, float], max_step_rad: float) -> list[dict[str, float]]:
    """Joint-space samples from start to end (both included), at most max_step_rad apart on every joint.

    Args:
        start (dict[str, float]): Start pose (joint -> rad); same joints as end.
        end (dict[str, float]): End pose (joint -> rad).
        max_step_rad (float): Largest per-joint step between samples (rad, > 0).

    Returns:
        list[dict[str, float]]: Samples, start first and end last (just [start, end] for a step within the limit).
    """
    span = max((abs(end[n] - start[n]) for n in end), default=0.0)
    steps = max(1, math.ceil(span / max_step_rad))
    return [
        *({n: start[n] + (end[n] - start[n]) * k / steps for n in end} for k in range(steps)),
        dict(end),
    ]


class ArmGuard:
    """Self-collision guard of the follower arm commands (see the module docstring)."""

    def __init__(
        self,
        checker: BodyCheck,
        max_step_rad: float,
        enabled: bool,
        head_source: Callable[[], HeadPose | None] | None = None,
        leader_gripper: LeaderGripperSettings | None = None,
    ) -> None:
        """Bind the guard to a body checker.

        Args:
            checker (BodyCheck): SelfCollisionChecker of the robot body and the arm calibration.
            max_step_rad (float): Largest per-joint step between checked samples (rad, > 0).
            enabled (bool): False publishes every request unchecked.
            head_source (Callable[[], HeadPose | None] | None): Head pose of a fresh /head/joint_states sample, None
                when missing or stale (the full head envelope); None = never a head pose.
            leader_gripper (LeaderGripperSettings | None): The follower bridge's gripper range mapping of leader
                commands; None checks leader commands as sent.
        """
        self.checker = checker
        self.max_step_rad = max_step_rad
        self.enabled = enabled
        self.head_source = head_source or (lambda: None)
        self.leader_gripper = leader_gripper
        self.pose: dict[str, float] = {}  # the arm's last known pose: last published command or follower feedback
        self.errors = 0

    def seed(self, names: Sequence[str], positions: Sequence[float]) -> None:
        """Take a follower feedback sample as the arm's pose (the start of the next checked segment).

        Args:
            names (Sequence[str]): Joint names.
            positions (Sequence[float]): Measured positions (rad), aligned with names.
        """
        self.pose.update({n: float(p) for n, p in zip(names, positions, strict=False)})

    def admit(self, source: str, names: Sequence[str], positions: Sequence[float]) -> GuardResult:
        """Check a request from the arm's last known pose and say what to publish.

        Args:
            source (str): Command source (leader, web_ui, autonomy): a leader gripper is mapped (leader_gripper).
            names (Sequence[str]): Requested joint names.
            positions (Sequence[float]): Requested positions (rad, measured space), aligned with names.

        Returns:
            GuardResult: The request, its clamp or None to publish, with the violation and the check time.
        """
        request = {n: float(p) for n, p in zip(names, positions, strict=False)}
        if not self.enabled:
            return GuardResult(list(request.values()))
        began = time.perf_counter()
        try:
            if not all(math.isfinite(p) for p in request.values()):
                raise ValueError(NON_FINITE_ERROR)
            gripper = self.leader_gripper if source == SOURCE_LEADER else None
            checked = dict(request)  # the request in follower radians (as the bridge will drive it)
            if gripper is not None and gripper.joint in checked:
                checked[gripper.joint] = leader_to_follower(checked[gripper.joint], gripper)
            end = {**self.pose, **checked}
            if any(j not in end for j in ARM_CHAIN_JOINTS):
                return GuardResult(None, blocked=True, obstacle=UNKNOWN_POSE, check_seconds=time.perf_counter() - began)
            has_start = all(j in self.pose for j in end)
            samples = segment({n: self.pose[n] for n in end}, end, self.max_step_rad) if has_start else [end]
            report = self.checker.posed(self.head_source()).path(samples, allow_escape=has_start)
        except Exception as exc:  # fail closed: any failure of the check publishes nothing
            self.errors += 1
            return GuardResult(None, check_seconds=time.perf_counter() - began, error=f"{type(exc).__name__}: {exc}")
        elapsed = time.perf_counter() - began
        if report.violation is None:
            self.pose = end
            return GuardResult(list(request.values()), check_seconds=elapsed)
        clamp = samples[report.violation - 1] if report.violation > 1 else None
        published = None
        if clamp is not None:
            self.pose = clamp
            published = dict(clamp)
            if gripper is not None and gripper.joint in request:
                published[gripper.joint] = follower_to_leader(clamp[gripper.joint], gripper)
        return GuardResult(
            [published[n] for n in request] if published is not None else None,
            blocked=True,
            link=report.violation_part,
            obstacle=report.violation_obstacle,
            clearance_m=report.violation_clearance,
            check_seconds=elapsed,
        )


def guard_state_payload(source: str, result: GuardResult) -> str:
    """JSON text of the guard state topic for one verdict.

    Args:
        source (str): Command source of the verdict (reported only while blocked, '' otherwise).
        result (GuardResult): The verdict.

    Returns:
        str: JSON object with blocked, source, link, obstacle and clearance_m (null when not blocked).
    """
    return json.dumps(
        {
            "blocked": result.blocked,
            "source": source if result.blocked else "",
            "link": result.link,
            "obstacle": result.obstacle,
            "clearance_m": result.clearance_m if result.blocked and math.isfinite(result.clearance_m) else None,
        }
    )


class GuardStateReporter:
    """Decides when to publish the guard state: on change and every GUARD_STATE_PERIOD_S while blocked."""

    def __init__(self) -> None:
        """Start clear, nothing published."""
        self.payload = guard_state_payload("", GuardResult(None))
        self.blocked = False
        self.blocked_at = -math.inf
        self.published: str | None = None
        self.published_at = -math.inf

    def update(self, source: str, result: GuardResult, now: float) -> None:
        """Take one verdict. A failed check says nothing about the arm and leaves the state alone.

        Args:
            source (str): Command source of the verdict.
            result (GuardResult): The verdict.
            now (float): Monotonic time (s).
        """
        if result.error:
            return
        self.payload = guard_state_payload(source, result)
        self.blocked = result.blocked
        if result.blocked:
            self.blocked_at = now

    def poll(self, now: float) -> str | None:
        """The payload to publish now, or None.

        A blocked state without a blocked command for GUARD_STATE_HOLD_S turns clear first.

        Args:
            now (float): Monotonic time (s).

        Returns:
            str | None: JSON payload when the state changed or the blocked period elapsed; None otherwise. The first
                clear state is not published (nothing was ever blocked).
        """
        if self.blocked and now - self.blocked_at > GUARD_STATE_HOLD_S:
            self.payload = guard_state_payload("", GuardResult(None))
            self.blocked = False
        changed = self.payload != self.published and (self.published is not None or self.blocked)
        repeat = self.blocked and now - self.published_at >= GUARD_STATE_PERIOD_S
        if not (changed or repeat):
            return None
        self.published = self.payload
        self.published_at = now
        return self.payload
