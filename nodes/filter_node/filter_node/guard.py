"""Arm self-collision guard: every follower command is checked against the robot body before filter_node publishes it.

The guard keeps the arm's last known pose: the last command it published, replaced by every follower feedback sample
(seed), whichever came last. A request is the straight joint-space segment from that pose to the requested one, sampled
at most max_step_rad per joint and checked by the shared SelfCollisionChecker (arm_collision) with the head posed where
/head/joint_states puts it (the full head envelope without a fresh head state). A start already inside the hard margin
may move out but not deeper (allow_escape). With no violation the request is published as is; otherwise the furthest
safe sample before the first violating one, or nothing when that sample is the start (the arm holds).
"""

import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any, Protocol

from arm_collision.head import HeadPose
from arm_collision.ik import ARM_CHAIN_JOINTS

# Obstacle label of a refusal whose arm pose is unknown (a partial request before any command or feedback).
UNKNOWN_POSE = "unknown_pose"


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
    ) -> None:
        """Bind the guard to a body checker.

        Args:
            checker (BodyCheck): SelfCollisionChecker of the robot body and the arm calibration.
            max_step_rad (float): Largest per-joint step between checked samples (rad, > 0).
            enabled (bool): False publishes every request unchecked.
            head_source (Callable[[], HeadPose | None] | None): Head pose of a fresh /head/joint_states sample, None
                when missing or stale (the full head envelope); None = never a head pose.
        """
        self.checker = checker
        self.max_step_rad = max_step_rad
        self.enabled = enabled
        self.head_source = head_source or (lambda: None)
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
            source (str): Command source (leader, web_ui, autonomy), for the caller's log and metrics.
            names (Sequence[str]): Requested joint names.
            positions (Sequence[float]): Requested positions (rad, measured space), aligned with names.

        Returns:
            GuardResult: The request, its clamp or None to publish, with the violation and the check time.
        """
        del source  # the verdict does not depend on the source; the caller labels its log and metrics with it
        request = {n: float(p) for n, p in zip(names, positions, strict=False)}
        if not self.enabled:
            return GuardResult(list(request.values()))
        began = time.perf_counter()
        end = {**self.pose, **request}
        if any(j not in end for j in ARM_CHAIN_JOINTS):
            return GuardResult(None, blocked=True, obstacle=UNKNOWN_POSE, check_seconds=time.perf_counter() - began)
        has_start = all(j in self.pose for j in end)
        samples = segment({n: self.pose[n] for n in end}, end, self.max_step_rad) if has_start else [end]
        try:
            report = self.checker.posed(self.head_source()).path(samples, allow_escape=has_start)
        except Exception as exc:  # fail closed: any checker failure publishes nothing
            self.errors += 1
            return GuardResult(None, check_seconds=time.perf_counter() - began, error=f"{type(exc).__name__}: {exc}")
        elapsed = time.perf_counter() - began
        if report.violation is None:
            self.pose = end
            return GuardResult(list(request.values()), check_seconds=elapsed)
        clamp = samples[report.violation - 1] if report.violation > 1 else None
        if clamp is not None:
            self.pose = clamp
        return GuardResult(
            [clamp[n] for n in request] if clamp is not None else None,
            blocked=True,
            link=report.violation_part,
            obstacle=report.violation_obstacle,
            clearance_m=report.violation_clearance,
            check_seconds=elapsed,
        )
