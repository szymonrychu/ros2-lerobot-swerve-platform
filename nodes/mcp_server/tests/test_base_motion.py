"""Tests for mcp_server.base_motion.run_drive (timed, clamped velocity streaming)."""

import math

import pytest

from mcp_server.base_motion import DriveError, SpinOutcome, run_drive, run_spin


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


# --- run_spin: continuous in-place rotation with marks (look_around spin mode) --------------------------------------


class SpinRig(Clock):
    """Clock whose yaw integrates the last published yaw rate (a perfect base)."""

    def __init__(self) -> None:
        super().__init__()
        self.yaw = 3.0  # wraps through +-pi during a turn
        self.pose_lost_at: float | None = None
        self.event_at: float | None = None
        self.marks: list[tuple[int, float]] = []
        self.refuse_mark: int | None = None

    def sleep(self, dt: float) -> None:
        wz = self.sent[-1][2] if self.sent else 0.0
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))
        super().sleep(dt)

    def current_yaw(self) -> float | None:
        if self.pose_lost_at is not None and self.t >= self.pose_lost_at:
            return None
        return self.yaw

    def interrupt(self) -> str | None:
        return "collision_stop" if self.event_at is not None and self.t >= self.event_at else None

    def on_mark(self, index: int) -> bool:
        self.marks.append((index, self.t))
        return index != self.refuse_mark


def spin(rig: SpinRig, angle: float, marks: list[float], wz: float = 0.4, timeout: float = 30.0) -> SpinOutcome:
    return run_spin(
        rig.publish,
        rig.now,
        rig.sleep,
        rig.abort,
        rig.interrupt,
        rig.current_yaw,
        wz,
        angle,
        marks,
        rig.on_mark,
        20.0,
        0.5,
        timeout,
    )


def test_spin_turns_the_angle_calls_every_mark_and_ends_with_zero() -> None:
    rig = SpinRig()
    out = spin(rig, 3 * math.pi / 2, [math.pi / 2, math.pi, 3 * math.pi / 2])
    assert out.status == "completed"
    assert out.rotated_rad == pytest.approx(3 * math.pi / 2, abs=0.05)
    assert [i for i, _ in rig.marks] == [0, 1, 2]
    # Marks fire in order at about 90 degree intervals of rotation (0.4 rad/s -> ~3.9 s apart).
    gaps = [b - a for (_, a), (_, b) in zip(rig.marks, rig.marks[1:], strict=False)]
    assert all(g == pytest.approx(math.pi / 2 / 0.4, abs=0.15) for g in gaps)
    assert rig.sent[-1] == (0.0, 0.0, 0.0)
    assert all(vx == 0.0 and vy == 0.0 for vx, vy, _ in rig.sent)


def test_spin_clamps_the_yaw_rate() -> None:
    rig = SpinRig()
    out = spin(rig, 0.5, [], wz=2.0)
    assert out.commanded_wz == 0.5 and max(abs(w) for *_, w in rig.sent) == 0.5


def test_spin_stops_on_stop_request() -> None:
    rig = SpinRig()
    rig.abort_at = 2.0
    out = spin(rig, 2 * math.pi, [math.pi])
    assert out.status == "stopped" and rig.sent[-1] == (0.0, 0.0, 0.0) and rig.marks == []


def test_spin_ends_on_a_critical_event() -> None:
    rig = SpinRig()
    rig.event_at = 1.0
    out = spin(rig, 2 * math.pi, [math.pi])
    assert out.status == "interrupted" and out.interrupted_by == "collision_stop"
    assert rig.sent[-1] == (0.0, 0.0, 0.0)


def test_spin_ends_when_a_mark_refuses() -> None:
    rig = SpinRig()
    rig.refuse_mark = 0
    out = spin(rig, 2 * math.pi, [math.pi / 2, math.pi])
    assert out.status == "aborted" and [i for i, _ in rig.marks] == [0]
    assert out.rotated_rad < math.pi
    assert rig.sent[-1] == (0.0, 0.0, 0.0)


def test_spin_fails_without_a_pose_and_times_out() -> None:
    rig = SpinRig()
    rig.pose_lost_at = 1.0
    out = spin(rig, 2 * math.pi, [])
    assert out.status == "failed" and "pose" in out.message
    stuck = SpinRig()
    stuck.sleep = lambda dt: Clock.sleep(stuck, dt)  # the base does not turn
    out = spin(stuck, 1.0, [], timeout=3.0)
    assert out.status == "timeout" and stuck.sent[-1] == (0.0, 0.0, 0.0)


def test_spin_rejects_bad_arguments() -> None:
    with pytest.raises(DriveError):
        spin(SpinRig(), 0.0, [])
    with pytest.raises(DriveError):
        spin(SpinRig(), 1.0, [], wz=0.0)
