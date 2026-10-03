"""Timed base velocity streaming (drive tool): clamp, publish at a fixed rate, always finish with a zero twist."""

import math
from collections.abc import Callable

from pydantic import BaseModel

from .geometry import clamp_twist


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
