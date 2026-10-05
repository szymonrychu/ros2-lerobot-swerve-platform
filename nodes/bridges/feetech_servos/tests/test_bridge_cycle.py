"""Tests for the rclpy-free bridge cycle: callback draining, velocity watchdog and NaN-aware command handling."""

import math
from collections import deque
from dataclasses import dataclass, field

import pytest

from feetech_servos.bridge_cycle import BridgeCycle, drain_callbacks, remaining_sleep_s
from feetech_servos.command_mapping import map_position_to_steps
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
    Each loop iteration drains callbacks via drain_callbacks, writes the coalesced targets once, then runs the
    velocity watchdog (same order as bridge.py).
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
        cycle.write_pending_commands()
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
    cycle.write_pending_commands()
    for step in range(1, 40):
        clock.now = step * 0.02
        cycle.handle_command(SWERVE, *drive_msg(1.0, joints=[DRIVE[0]]))
        cycle.write_pending_commands()
        cycle.stop_expired()
    assert [v for _t, v in servo.speed_writes(32)].count(0) == 0
    for sid in DRIVE_IDS - {32}:
        assert servo.speed_writes(sid)[-1][1] == 0


def test_received_drive_command_keeps_watchdog_fed_even_if_write_fails() -> None:
    cycle, servo, clock = make_cycle([DRIVE[0]])
    cycle.handle_command(SWERVE, *drive_msg(1.0, joints=[DRIVE[0]]))
    cycle.write_pending_commands()
    servo.fail_writes = True
    for step in range(1, 30):  # new commanded value keeps failing to write, but commands keep arriving
        clock.now = step * 0.02
        cycle.handle_command(SWERVE, *drive_msg(2.0, joints=[DRIVE[0]]))
        cycle.write_pending_commands()
        assert cycle.stop_expired() == []
    servo.fail_writes = False
    clock.now += TIMEOUT_S + 0.01  # commands stop: the wheel (still at the last applied speed) is stopped
    assert cycle.stop_expired() == [32]
    assert servo.speed_writes(32)[-1][1] == 0


def test_failed_zero_command_is_retried_by_watchdog_after_commands_stop() -> None:
    cycle, servo, clock = make_cycle([DRIVE[0]])
    cycle.handle_command(SWERVE, *drive_msg(1.0, joints=[DRIVE[0]]))
    cycle.write_pending_commands()
    servo.fail_writes = True
    clock.now = 0.05
    cycle.handle_command(SWERVE, *drive_msg(0.0, joints=[DRIVE[0]]))  # stop request lost on the bus
    cycle.write_pending_commands()
    servo.fail_writes = False
    clock.now = 0.05 + TIMEOUT_S + 0.01
    assert cycle.stop_expired() == [32]
    assert servo.speed_writes(32)[-1][1] == 0


# --- coalescing: callbacks only record targets, one write per joint per cycle ---


def test_command_callback_never_writes_to_the_bus() -> None:
    cycle, servo, _clock = make_cycle(ARM.joints + DRIVE)
    cycle.handle_command(ARM, ["gripper", "wrist_roll"], [0.1, -0.1], [])
    cycle.handle_command(SWERVE, *drive_msg(1.0))
    assert servo.writes == []


def test_many_queued_messages_for_one_joint_write_once_with_newest_value() -> None:
    cycle, servo, _clock = make_cycle(ARM.joints + DRIVE)
    for value in (0.1, 0.2, 0.3, 0.4, 0.5):
        cycle.handle_command(ARM, ["gripper"], [value], [])
        cycle.handle_command(SWERVE, *drive_msg(value * 4, joints=[DRIVE[0]]))
    cycle.write_pending_commands()
    assert servo.position_writes(6) == [2048 + round(0.5 * 4096 / (2 * math.pi))]
    assert [v for _t, v in servo.speed_writes(32)] == [round(2.0 * 4096 / (2 * math.pi))]
    assert len(servo.writes) == 2


def test_stale_superseded_target_is_never_written() -> None:
    cycle, servo, _clock = make_cycle(ARM.joints + DRIVE)
    stale_steps = 2048 + round(0.3 * 4096 / (2 * math.pi))
    stale_speed = round(3.0 * 4096 / (2 * math.pi))
    cycle.handle_command(ARM, ["gripper"], [0.3], [])
    cycle.handle_command(SWERVE, *drive_msg(3.0, joints=[DRIVE[0]]))
    cycle.handle_command(ARM, ["gripper"], [0.0], [])
    cycle.handle_command(SWERVE, *drive_msg(0.5, joints=[DRIVE[0]]))
    cycle.write_pending_commands()
    cycle.write_pending_commands()  # a second cycle with no new message writes nothing
    assert stale_steps not in servo.position_writes(6)
    assert stale_speed not in [v for _t, v in servo.speed_writes(32)]
    assert servo.position_writes(6) == [2048]
    assert len(servo.writes) == 2


def test_non_finite_entry_does_not_replace_pending_finite_target() -> None:
    cycle, servo, _clock = make_cycle(ARM.joints + DRIVE)
    cycle.handle_command(ARM, ["gripper"], [0.1], [])
    cycle.handle_command(ARM, ["gripper"], [NAN], [])
    cycle.handle_command(SWERVE, *drive_msg(1.0, joints=[DRIVE[0]]))
    cycle.handle_command(SWERVE, ["fl_drive"], [], [NAN])
    cycle.write_pending_commands()
    assert servo.position_writes(6) == [2048 + round(0.1 * 4096 / (2 * math.pi))]
    assert [v for _t, v in servo.speed_writes(32)] == [round(4096 / (2 * math.pi))]


def test_unchanged_target_is_not_rewritten() -> None:
    cycle, servo, _clock = make_cycle(ARM.joints)
    cycle.handle_command(ARM, ["gripper"], [0.1], [])
    cycle.write_pending_commands()
    cycle.handle_command(ARM, ["gripper"], [0.1], [])
    cycle.write_pending_commands()
    assert len(servo.position_writes(6)) == 1


def test_watchdog_is_fed_by_receive_time_not_write_time() -> None:
    cycle, servo, clock = make_cycle([DRIVE[0]])
    cycle.handle_command(SWERVE, *drive_msg(1.0, joints=[DRIVE[0]]))  # received at t=0
    clock.now = 0.25  # the loop only gets to write it late
    cycle.write_pending_commands()
    assert cycle.last_velocity_commands[32][0] == 0.0
    clock.now = TIMEOUT_S + 0.01  # 0.31 s after receipt, only 0.06 s after the write
    assert cycle.stop_expired() == [32]
    assert servo.speed_writes(32)[-1][1] == 0


def test_stale_pending_drive_target_does_not_restart_wheel_after_watchdog_stop() -> None:
    cycle, servo, clock = make_cycle([DRIVE[0]])
    cycle.handle_command(SWERVE, *drive_msg(1.0, joints=[DRIVE[0]]))
    cycle.write_pending_commands()
    clock.now = TIMEOUT_S + 0.01
    assert cycle.stop_expired() == [32]
    for step in range(1, 5):
        clock.now = TIMEOUT_S + 0.01 + step * 0.02
        cycle.write_pending_commands()
        assert cycle.stop_expired() == []
    assert [v for _t, v in servo.speed_writes(32)] == [round(4096 / (2 * math.pi)), 0]


def test_many_messages_after_hiccup_write_each_joint_at_most_once() -> None:
    cycle, servo, clock = make_cycle()
    queue = deque()
    for i in range(32):  # a burst well beyond one cycle's worth of steer/drive pairs
        queue.append(steer_msg(0.01 * i))
        queue.append(drive_msg(0.1 * (i + 1)))

    def spin_ready() -> bool:
        if not queue:
            return False
        cycle.handle_command(SWERVE, *queue.popleft())
        return True

    drain_callbacks(spin_ready)
    cycle.write_pending_commands()
    assert len(servo.writes) == len(STEER) + len(DRIVE)
    for joint in STEER:
        assert servo.position_writes(joint.id) == [2048 + round(0.31 * 4096 / (2 * math.pi))]
    for joint in DRIVE:
        assert [v for _t, v in servo.speed_writes(joint.id)] == [round(3.2 * 4096 / (2 * math.pi))]


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
    cycle.write_pending_commands()
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
    cycle.write_pending_commands()
    writes_before = list(servo.writes)
    for step in range(1, 20):
        clock.now = step * 0.02
        cycle.handle_command(SWERVE, ["fl_drive"], [], [bad])
        cycle.write_pending_commands()
    assert servo.writes == writes_before
    assert cycle.stop_expired() == [32]  # NaN is not a command: the wheel goes stale and is stopped


@pytest.mark.parametrize("bad", [NAN, float("inf"), float("-inf")])
def test_non_finite_position_is_ignored(bad: float) -> None:
    cycle, servo, _clock = make_cycle(ARM.joints)
    cycle.handle_command(ARM, ["gripper", "wrist_roll"], [bad, 0.0], [])
    cycle.write_pending_commands()
    assert servo.position_writes(6) == []
    assert servo.position_writes(5) == [2048]


def test_arm_position_only_message_still_works() -> None:
    cycle, servo, _clock = make_cycle(ARM.joints)
    cycle.handle_command(ARM, ["gripper", "wrist_roll"], [0.1, -0.1], [])
    cycle.write_pending_commands()
    assert servo.position_writes(6) == [2048 + round(0.1 * 4096 / (2 * math.pi))]
    assert servo.position_writes(5) == [2048 - round(0.1 * 4096 / (2 * math.pi))]


def test_separate_steer_and_drive_messages_still_work() -> None:
    cycle, servo, _clock = make_cycle()
    cycle.handle_command(SWERVE, *steer_msg(0.0))
    cycle.handle_command(SWERVE, *drive_msg(-1.0))
    cycle.write_pending_commands()
    for joint in STEER:
        assert servo.position_writes(joint.id) == [2048]
    for joint in DRIVE:
        assert [v for _t, v in servo.speed_writes(joint.id)] == [0x8000 | round(4096 / (2 * math.pi))]


# --- gripper source range mapping applies only to leader commands (direct sources carry follower radians) ---

# lerobot_follower gripper: leader range 2045..3289 mapped onto the follower command range (servo angle limits).
GRIPPER_MAPPED = JointEntry("gripper", 6, source_min_steps=2045, source_max_steps=3289)
MAPPED_ARM = JointGroup("follower", [GRIPPER_MAPPED, JointEntry("wrist_roll", 5)])
GRIPPER_LIMITS = (1934, 3186)
DIRECT_SOURCES = ("web_ui", "autonomy")
LEADER_SWEEP_RAD = [-0.5, -0.172, -0.0046, 0.0, 0.1, 0.5, 1.0, 1.5, 1.909, 2.5]


def make_mapped_cycle(direct_sources: tuple[str, ...] = DIRECT_SOURCES) -> tuple[BridgeCycle, FakeServo]:
    """Arm cycle with the gripper source range mapping and its command range (servo limits)."""
    clock = FakeClock()
    servo = FakeServo(clock)
    cycle = BridgeCycle(
        servo=servo,
        goal_entry=get_register_entry_by_name("goal_position"),
        speed_goal_entry=get_register_entry_by_name("goal_speed"),
        velocity_joints=[],
        command_limits={6: GRIPPER_LIMITS},
        last_written={},
        velocity_command_timeout_s=TIMEOUT_S,
        clock=clock,
        direct_command_sources=direct_sources,
    )
    return cycle, servo


def gripper_steps_for(cycle: BridgeCycle, servo: FakeServo, position: float, source: str) -> int:
    """goal_position written for one gripper command from a source."""
    cycle.last_written.clear()
    cycle.handle_command(MAPPED_ARM, ["gripper"], [position], [], source=source)
    cycle.write_pending_commands()
    return servo.position_writes(6)[-1]


@pytest.mark.parametrize("source", ["web_ui", "autonomy"])
def test_direct_source_gripper_target_reaches_servo_as_follower_position(source: str) -> None:
    """Regression: move_arm_joints {gripper: 0.0} was remapped as leader progress and clamped near closed."""
    cycle, servo = make_mapped_cycle()
    assert gripper_steps_for(cycle, servo, 0.0, source) == 2048
    assert gripper_steps_for(cycle, servo, -0.1, source) == 2048 - round(0.1 * 4096 / (2 * math.pi))
    assert gripper_steps_for(cycle, servo, 1.5, source) == 2048 + round(1.5 * 4096 / (2 * math.pi))


def test_direct_source_gripper_target_is_clamped_to_command_range() -> None:
    cycle, servo = make_mapped_cycle()
    assert gripper_steps_for(cycle, servo, -0.5, "autonomy") == GRIPPER_LIMITS[0]
    assert gripper_steps_for(cycle, servo, 2.5, "autonomy") == GRIPPER_LIMITS[1]


@pytest.mark.parametrize("source", ["leader", ""])
def test_leader_and_untagged_gripper_steps_are_unchanged(source: str) -> None:
    """Leader (and untagged legacy) commands keep the exact source range mapping."""
    cycle, servo = make_mapped_cycle()
    for position in LEADER_SWEEP_RAD:
        expected = map_position_to_steps(position, 2045, 3289, *GRIPPER_LIMITS)
        assert gripper_steps_for(cycle, servo, position, source) == expected, position


def test_leader_gripper_steps_match_cycle_without_direct_sources() -> None:
    """Configuring direct sources does not change a single step of the leader -> follower gripper output."""
    with_direct, servo_a = make_mapped_cycle()
    legacy, servo_b = make_mapped_cycle(direct_sources=())
    for position in LEADER_SWEEP_RAD:
        assert gripper_steps_for(with_direct, servo_a, position, "leader") == gripper_steps_for(
            legacy, servo_b, position, "leader"
        )


def test_without_direct_sources_every_source_is_mapped() -> None:
    """Default (no direct_command_sources): legacy behaviour, the mapping applies to every message."""
    cycle, servo = make_mapped_cycle(direct_sources=())
    assert gripper_steps_for(cycle, servo, 0.0, "autonomy") == map_position_to_steps(0.0, 2045, 3289, *GRIPPER_LIMITS)


def test_direct_source_does_not_change_joints_without_mapping() -> None:
    cycle, servo = make_mapped_cycle()
    cycle.handle_command(MAPPED_ARM, ["wrist_roll"], [0.1], [], source="autonomy")
    cycle.write_pending_commands()
    assert servo.position_writes(5) == [2048 + round(0.1 * 4096 / (2 * math.pi))]
