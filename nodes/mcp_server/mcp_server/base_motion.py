"""Timed base velocity streaming (drive tool) and the stop sequence: clamp, publish at a fixed rate, always finish
with a zero twist."""

import math
from collections.abc import Callable

from pydantic import BaseModel

from .geometry import clamp_twist
from .models import StopResult


class DriveError(ValueError):
    """Raised for an invalid drive request."""


class DriveOutcome(BaseModel):
    """Result of a timed drive."""

    commanded: tuple[float, float, float]
    clamped: bool
    aborted: bool
    published: int


def run_drive(
    publish: Callable[[float, float, float], None],
    now: Callable[[], float],
    sleep: Callable[[float], None],
    should_abort: Callable[[], bool],
    vx: float,
    vy: float,
    wz: float,
    duration_s: float,
    rate_hz: float,
    max_linear: float,
    max_angular: float,
    max_duration: float,
) -> DriveOutcome:
    """Publish a clamped twist at rate_hz for duration_s, then a zero twist (also after an abort).

    Args:
        publish (Callable[[float, float, float], None]): Sends one twist (vx, vy, wz).
        now (Callable[[], float]): Monotonic clock (s).
        sleep (Callable[[float], None]): Sleeps the given seconds.
        should_abort (Callable[[], bool]): Polled each cycle; True stops early (stop tool).
        vx (float): Forward velocity (m/s).
        vy (float): Lateral velocity (m/s).
        wz (float): Yaw rate (rad/s).
        duration_s (float): Drive time, in (0, max_duration].
        rate_hz (float): Publish rate.
        max_linear (float): Linear clamp (m/s).
        max_angular (float): Angular clamp (rad/s).
        max_duration (float): Longest allowed duration (s).

    Returns:
        DriveOutcome: Commanded (clamped) twist, whether it was clamped/aborted, and messages published.
    """
    if not math.isfinite(duration_s) or duration_s <= 0.0 or duration_s > max_duration:
        raise DriveError(f"duration_s must be in (0, {max_duration}] s, got {duration_s}")
    try:
        cmd = clamp_twist(vx, vy, wz, max_linear, max_angular)
    except ValueError as exc:
        raise DriveError(str(exc)) from exc
    period = 1.0 / rate_hz
    start = now()
    end = start + duration_s
    next_tick = start
    published = 0
    aborted = False
    try:
        while now() < end - 1e-9:
            if should_abort():
                aborted = True
                break
            publish(*cmd)
            published += 1
            next_tick += period
            sleep(max(0.0, next_tick - now()))
    finally:
        publish(0.0, 0.0, 0.0)
        published += 1
    return DriveOutcome(commanded=cmd, clamped=cmd != (vx, vy, wz), aborted=aborted, published=published)


def run_stop(
    signal_base_stop: Callable[[], None],
    publish: Callable[[float, float, float], None],
    cancel_all_goals: Callable[[], bool],
    stop_arm: Callable[[], tuple[bool, str]],
) -> StopResult:
    """Stop tool sequence: every part is attempted regardless of the others; the base is always zeroed.

    Args:
        signal_base_stop (Callable[[], None]): Tells running navigate/drive calls to abort.
        publish (Callable[[float, float, float], None]): Sends one twist (vx, vy, wz).
        cancel_all_goals (Callable[[], bool]): Cancels every Nav2 goal; True when the cancel service answered.
        stop_arm (Callable[[], tuple[bool, str]]): Arm part (ArmController.stop_hold): held flag and note.

    Returns:
        StopResult: What succeeded.
    """
    signal_base_stop()
    publish(0.0, 0.0, 0.0)
    arm_held, arm_note = stop_arm()
    cancelled = cancel_all_goals()
    publish(0.0, 0.0, 0.0)
    notes = [] if cancelled else ["Nav2 cancel service unavailable"]
    if arm_note:
        notes.append(arm_note)
    return StopResult(nav_goals_cancelled=cancelled, base_zeroed=True, arm_held=arm_held, message="; ".join(notes))
