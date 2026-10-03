"""Tests for mcp_server.base_motion.run_drive (timed, clamped velocity streaming)."""

import pytest

from mcp_server.base_motion import DriveError, run_drive


class Clock:
    def __init__(self) -> None:
        self.t = 0.0
        self.sent: list[tuple[float, float, float]] = []
        self.abort_at: float | None = None

    def publish(self, vx: float, vy: float, wz: float) -> None:
        self.sent.append((vx, vy, wz))

    def now(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        self.t += dt

    def abort(self) -> bool:
        return self.abort_at is not None and self.t >= self.abort_at


def drive(c: Clock, vx: float, vy: float, wz: float, duration: float):
    return run_drive(c.publish, c.now, c.sleep, c.abort, vx, vy, wz, duration, 20.0, 0.25, 0.5, 2.0)


def test_drive_streams_at_rate_then_zero() -> None:
    c = Clock()
    out = drive(c, 0.1, 0.0, 0.2, 1.0)
    assert c.sent[-1] == (0.0, 0.0, 0.0)
    assert len(c.sent) - 1 == pytest.approx(20, abs=1)
    assert all(s == (0.1, 0.0, 0.2) for s in c.sent[:-1])
    assert not out.aborted
    assert out.commanded == (0.1, 0.0, 0.2)


def test_drive_clamps_velocity() -> None:
    c = Clock()
    out = drive(c, 1.0, -1.0, 3.0, 0.2)
    assert out.commanded == (0.25, -0.25, 0.5)
    assert out.clamped


def test_drive_rejects_long_or_non_positive_duration() -> None:
    with pytest.raises(DriveError):
        drive(Clock(), 0.1, 0.0, 0.0, 2.5)
    with pytest.raises(DriveError):
        drive(Clock(), 0.1, 0.0, 0.0, 0.0)


def test_drive_abort_stops_early_with_zero() -> None:
    c = Clock()
    c.abort_at = 0.3
    out = drive(c, 0.1, 0.0, 0.0, 2.0)
    assert out.aborted
    assert c.sent[-1] == (0.0, 0.0, 0.0)
    assert len(c.sent) < 10
