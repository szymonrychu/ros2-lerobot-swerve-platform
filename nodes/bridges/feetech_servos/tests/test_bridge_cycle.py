"""Tests for the rclpy-free bridge cycle: callback draining, velocity watchdog and NaN-aware command handling."""

import math
from collections import deque
from dataclasses import dataclass, field

import pytest

from feetech_servos.bridge_cycle import BridgeCycle, drain_callbacks, remaining_sleep_s
from feetech_servos.config import JointEntry, JointGroup
from feetech_servos.registers import STS_GOAL_POSITION_L, STS_GOAL_SPEED_L, get_register_entry_by_name

NAN = float("nan")
TIMEOUT_S = 0.3
QOS_DEPTH = 10
CONTROLLER_HZ = 107.5  # swerve_drive_controller publishes a steer + drive pair per cycle (~215 msg/s)
STEER = [
    JointEntry("fl_steer", 33),
    JointEntry("fr_steer", 34),
    JointEntry("rl_steer", 37),
    JointEntry("rr_steer", 38),
]
DRIVE = [
    JointEntry("fl_drive", 32, mode="velocity"),
    JointEntry("fr_drive", 35, mode="velocity"),
    JointEntry("rl_drive", 39, mode="velocity"),
    JointEntry("rr_drive", 36, mode="velocity"),
]
SWERVE = JointGroup("swerve_drive", STEER + DRIVE)
ARM = JointGroup("follower", [JointEntry("gripper", 6), JointEntry("wrist_roll", 5)])
DRIVE_IDS = {j.id for j in DRIVE}


@dataclass
class FakeClock:
    """Settable monotonic clock."""

    now: float = 0.0

    def __call__(self) -> float:
        """Return the current simulated time in seconds."""
        return self.now


@dataclass
class FakeServo:
    """Records register writes like the ST3215 API; writes fail while fail_writes is True."""

    clock: FakeClock
    writes: list[tuple[float, int, int, int]] = field(default_factory=list)  # (time, servo_id, address, value)
    fail_writes: bool = False

    def write1ByteTxRx(self, sts_id: int, address: int, value: int) -> tuple[int, int]:  # noqa: N802 - st3215 API
        """Record a 1-byte write."""
        return self.write2ByteTxRx(sts_id, address, value)

    def write2ByteTxRx(self, sts_id: int, address: int, value: int) -> tuple[int, int]:  # noqa: N802 - st3215 API
        """Record a 2-byte write and return (comm_result, error)."""
        if self.fail_writes:
            return (-1001, 0)
        self.writes.append((self.clock.now, sts_id, address, value))
        return (0, 0)

    def speed_writes(self, servo_id: int) -> list[tuple[float, int]]:
        """Return (time, raw goal_speed) writes for one servo."""
        return [(t, v) for t, sid, addr, v in self.writes if sid == servo_id and addr == STS_GOAL_SPEED_L]

    def position_writes(self, servo_id: int) -> list[int]:
        """Return raw goal_position writes for one servo."""
        return [v for _t, sid, addr, v in self.writes if sid == servo_id and addr == STS_GOAL_POSITION_L]


def make_cycle(
    joints: list[JointEntry] | None = None,
) -> tuple[BridgeCycle, FakeServo, FakeClock]:
    """Build a BridgeCycle over a fake servo bus with the swerve wheels in velocity mode."""
    clock = FakeClock()
    servo = FakeServo(clock)
    velocity_joints = [j for j in (joints or DRIVE) if j.mode == "velocity"]
    cycle = BridgeCycle(
        servo=servo,
        goal_entry=get_register_entry_by_name("goal_position"),
        speed_goal_entry=get_register_entry_by_name("goal_speed"),
        velocity_joints=velocity_joints,
        command_limits={},
        last_written={},
        velocity_command_timeout_s=TIMEOUT_S,
        clock=clock,
    )
    return cycle, servo, clock


def steer_msg(angle: float) -> tuple[list[str], list[float], list[float]]:
    """Steer-only command as published by swerve_drive_controller."""
    return ([j.name for j in STEER], [angle] * len(STEER), [])


def drive_msg(velocity: float, joints: list[JointEntry] = DRIVE) -> tuple[list[str], list[float], list[float]]:
    """Drive-only command as published by swerve_drive_controller."""
    return ([j.name for j in joints], [], [velocity] * len(joints))


def simulate(
    cycle: BridgeCycle,
    clock: FakeClock,
    duration_s: float,
    loop_hz: float,
    drive_velocity: float = 1.0,
    drive_until_s: float | None = None,
) -> None:
    """Run the bridge loop against a controller publishing steer/drive pairs faster than the loop.

    Messages land in a KEEP_LAST(QOS_DEPTH) queue (oldest dropped on overflow), like the rclpy subscription.
    Each loop iteration drains callbacks via drain_callbacks, then runs the velocity watchdog.
    """
    queue: deque[tuple[list[str], list[float], list[float]]] = deque(maxlen=QOS_DEPTH)
    publish_period = 1.0 / CONTROLLER_HZ
    next_publish = 0.0

    def spin_ready() -> bool:
        if not queue:
            return False
        cycle.handle_command(SWERVE, *queue.popleft())
        return True

    tick = 0
    while tick / loop_hz <= duration_s:
        now = tick / loop_hz
        while next_publish <= now:
            queue.append(steer_msg(0.2))
            if drive_until_s is None or next_publish < drive_until_s:
                queue.append(drive_msg(drive_velocity))
            next_publish += publish_period
        clock.now = now
        drain_callbacks(spin_ready)
        cycle.stop_expired()
        tick += 1


# --- regression: wheels must not be stopped while drive commands keep arriving ---


@pytest.mark.parametrize("loop_hz", [85.0, 50.0, 10.0])
def test_goal_speed_never_zero_while_drive_commands_arrive(loop_hz: float) -> None:
    cycle, servo, clock = make_cycle()
    simulate(cycle, clock, duration_s=3.0, loop_hz=loop_hz)
    for sid in DRIVE_IDS:
        writes = servo.speed_writes(sid)
        assert writes, f"servo {sid} never received a drive command"
        zero_writes = [t for t, value in writes if value == 0]
        assert zero_writes == [], f"servo {sid} goal_speed written 0 at t={zero_writes[:3]} during constant drive"


def test_watchdog_stops_wheels_when_drive_commands_cease_but_steer_continues() -> None:
    cycle, servo, clock = make_cycle()
    simulate(cycle, clock, duration_s=1.5, loop_hz=85.0, drive_until_s=0.5)
    for sid in DRIVE_IDS:
        zero_times = [t for t, value in servo.speed_writes(sid) if value == 0]
        assert len(zero_times) == 1
        assert 0.5 + TIMEOUT_S <= zero_times[0] <= 0.5 + TIMEOUT_S + 2.0 / 85.0


def test_watchdog_is_per_wheel() -> None:
    cycle, servo, clock = make_cycle()
    cycle.handle_command(SWERVE, *drive_msg(1.0))
    for step in range(1, 40):
        clock.now = step * 0.02
        cycle.handle_command(SWERVE, *drive_msg(1.0, joints=[DRIVE[0]]))
        cycle.stop_expired()
    assert [v for _t, v in servo.speed_writes(32)].count(0) == 0
    for sid in DRIVE_IDS - {32}:
        assert servo.speed_writes(sid)[-1][1] == 0


def test_received_drive_command_keeps_watchdog_fed_even_if_write_fails() -> None:
    cycle, servo, clock = make_cycle([DRIVE[0]])
    cycle.handle_command(SWERVE, *drive_msg(1.0, joints=[DRIVE[0]]))
    servo.fail_writes = True
    for step in range(1, 30):  # new commanded value keeps failing to write, but commands keep arriving
        clock.now = step * 0.02
        cycle.handle_command(SWERVE, *drive_msg(2.0, joints=[DRIVE[0]]))
        assert cycle.stop_expired() == []
    servo.fail_writes = False
    clock.now += TIMEOUT_S + 0.01  # commands stop: the wheel (still at the last applied speed) is stopped
    assert cycle.stop_expired() == [32]
    assert servo.speed_writes(32)[-1][1] == 0


def test_failed_zero_command_is_retried_by_watchdog_after_commands_stop() -> None:
    cycle, servo, clock = make_cycle([DRIVE[0]])
    cycle.handle_command(SWERVE, *drive_msg(1.0, joints=[DRIVE[0]]))
    servo.fail_writes = True
    clock.now = 0.05
    cycle.handle_command(SWERVE, *drive_msg(0.0, joints=[DRIVE[0]]))  # stop request lost on the bus
    servo.fail_writes = False
    clock.now = 0.05 + TIMEOUT_S + 0.01
    assert cycle.stop_expired() == [32]
    assert servo.speed_writes(32)[-1][1] == 0


# --- drain_callbacks / remaining_sleep_s ---


def test_drain_callbacks_processes_everything_pending() -> None:
    pending = deque(range(7))

    def spin_ready() -> bool:
        if not pending:
            return False
        pending.popleft()
        return True

    assert drain_callbacks(spin_ready) == 7
    assert not pending


def test_drain_callbacks_is_bounded() -> None:
    calls = []

    def always_ready() -> bool:
        calls.append(1)
        return True

    assert drain_callbacks(always_ready, max_callbacks=5) == 5
    assert len(calls) == 5


def test_drain_callbacks_stops_when_nothing_ready() -> None:
    calls = []

    def never_ready() -> bool:
        calls.append(1)
        return False

    assert drain_callbacks(never_ready) == 0
    assert len(calls) == 1


def test_remaining_sleep_s() -> None:
    assert remaining_sleep_s(0.02, 0.005) == pytest.approx(0.015)
    assert remaining_sleep_s(0.02, 0.05) == 0.0


# --- combined command format and NaN handling ---


def test_combined_message_with_nan_placeholders_drives_steer_and_wheels() -> None:
    cycle, servo, _clock = make_cycle()
    names = [j.name for j in STEER + DRIVE]
    positions = [0.5] * len(STEER) + [NAN] * len(DRIVE)
    velocities = [NAN] * len(STEER) + [1.0] * len(DRIVE)
    cycle.handle_command(SWERVE, names, positions, velocities)
    for joint in STEER:
        assert servo.position_writes(joint.id) == [2048 + round(0.5 * 4096 / (2 * math.pi))]
        assert servo.speed_writes(joint.id) == []
    for joint in DRIVE:
        assert [v for _t, v in servo.speed_writes(joint.id)] == [round(4096 / (2 * math.pi))]
        assert servo.position_writes(joint.id) == []


@pytest.mark.parametrize("bad", [NAN, float("inf"), float("-inf")])
def test_non_finite_velocity_is_ignored_and_does_not_feed_watchdog(bad: float) -> None:
    cycle, servo, clock = make_cycle([DRIVE[0]])
    cycle.handle_command(SWERVE, *drive_msg(1.0, joints=[DRIVE[0]]))
    writes_before = list(servo.writes)
    for step in range(1, 20):
        clock.now = step * 0.02
        cycle.handle_command(SWERVE, ["fl_drive"], [], [bad])
    assert servo.writes == writes_before
    assert cycle.stop_expired() == [32]  # NaN is not a command: the wheel goes stale and is stopped


@pytest.mark.parametrize("bad", [NAN, float("inf"), float("-inf")])
def test_non_finite_position_is_ignored(bad: float) -> None:
    cycle, servo, _clock = make_cycle(ARM.joints)
    cycle.handle_command(ARM, ["gripper", "wrist_roll"], [bad, 0.0], [])
    assert servo.position_writes(6) == []
    assert servo.position_writes(5) == [2048]


def test_arm_position_only_message_still_works() -> None:
    cycle, servo, _clock = make_cycle(ARM.joints)
    cycle.handle_command(ARM, ["gripper", "wrist_roll"], [0.1, -0.1], [])
    assert servo.position_writes(6) == [2048 + round(0.1 * 4096 / (2 * math.pi))]
    assert servo.position_writes(5) == [2048 - round(0.1 * 4096 / (2 * math.pi))]


def test_separate_steer_and_drive_messages_still_work() -> None:
    cycle, servo, _clock = make_cycle()
    cycle.handle_command(SWERVE, *steer_msg(0.0))
    cycle.handle_command(SWERVE, *drive_msg(-1.0))
    for joint in STEER:
        assert servo.position_writes(joint.id) == [2048]
    for joint in DRIVE:
        assert [v for _t, v in servo.speed_writes(joint.id)] == [0x8000 | round(4096 / (2 * math.pi))]
