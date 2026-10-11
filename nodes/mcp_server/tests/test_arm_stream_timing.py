"""Stream timing: ticks on a deadline, setpoints sampled from the plan at the elapsed time (jitter, overruns)."""

import random
from pathlib import Path

import pytest

from mcp_server.arm import ArmController, JointSample, setpoint_band
from mcp_server.config import McpServerConfig
from mcp_server.ik import ArmKinematics, load_joint_limits
from mcp_server.trajectory import plan_sample, plan_trajectory, quintic

from .fakes import FakeArmBackend

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
PERIOD = 1.0 / CONFIG.limits.arm_rate_hz
PAN = "shoulder_pan"
PAN_TRAVEL = 1.0
# Late wake-up of a jittery sleep: the robot measured 40-60 ms ticks for the planned 40 ms (2026-10-11).
OVERSLEEP_MAX_S = 0.02
# One tick whose work (a slow joint sample) overruns by this many periods.
OVERRUN_PERIODS = 2.5
OVERRUN_TICK = 10
# Linear interpolation of the 25 Hz quintic samples deviates from the analytic quintic by far less than this.
PLAN_TOLERANCE_RAD = 2e-3


def make(tmp_path: Path, body_check: bool = False) -> tuple[ArmController, FakeArmBackend]:
    """Controller on a fake follower that follows every command at once.

    Args:
        tmp_path (Path): Test directory (home file).
        body_check (bool): Keep the robot body check (its soft margin slows the PAN move from the zero pose).

    Returns:
        tuple[ArmController, FakeArmBackend]: Controller and backend.
    """
    be = FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "arm" / "home.yaml"
    cfg.body_check.enabled = body_check
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg), be


def planned_steps() -> int:
    """Setpoints of the full-speed PAN_TRAVEL move from the zero pose.

    Returns:
        int: Number of planned setpoints (the plan lasts that many periods).
    """
    start = {PAN: 0.0}
    return len(plan_trajectory(start, {PAN: PAN_TRAVEL}, CONFIG.limits.arm_max_joint_velocity_rps, 1.0 / PERIOD))


def trajectory_commands(be: FakeArmBackend, first: int) -> list[tuple[float, float]]:
    """(publish time, pan setpoint) of the trajectory commands, from index first up to the first goal command.

    Args:
        be (FakeArmBackend): Backend after the motion.
        first (int): Index of the first trajectory command.

    Returns:
        list[tuple[float, float]]: Commands of the trajectory, the goal last.
    """
    out: list[tuple[float, float]] = []
    for stamp, command in zip(be.command_times[first:], be.commands[first:], strict=True):
        out.append((stamp, command[PAN]))
        if command[PAN] == pytest.approx(PAN_TRAVEL, abs=1e-12):
            break
    return out


def assert_on_plan(commands: list[tuple[float, float]], steps: int) -> None:
    """Every setpoint equals the analytic quintic at its publish time (the first is the plan at one period).

    Args:
        commands (list[tuple[float, float]]): (publish time, pan setpoint) of the trajectory.
        steps (int): Planned setpoints.
    """
    t0 = commands[0][0]
    for stamp, value in commands:
        progress = min((stamp - t0) / PERIOD + 1.0, steps) / steps
        assert value == pytest.approx(PAN_TRAVEL * quintic(progress), abs=PLAN_TOLERANCE_RAD), stamp - t0


def test_jittery_ticks_finish_at_the_planned_time_on_the_plan(tmp_path: Path) -> None:
    """40-60 ms ticks (late wake-ups) no longer stretch the motion: it ends at its planned time, setpoints = plan(t)."""
    arm, be = make(tmp_path)
    arm.acquire()
    rng = random.Random(7)

    def oversleep(backend: FakeArmBackend) -> None:
        backend.t += rng.uniform(0.0, OVERSLEEP_MAX_S)

    be.on_sleep = oversleep
    first = len(be.commands)
    res = arm.move_joints({PAN: PAN_TRAVEL})
    assert res.status == "converged", res.message
    steps = planned_steps()
    assert res.trajectory_s == pytest.approx(steps * PERIOD, abs=PERIOD)
    commands = trajectory_commands(be, first)
    gaps = [b[0] - a[0] for a, b in zip(commands, commands[1:], strict=False)]
    assert max(gaps) > PERIOD + 0.005  # the ticks really were jittery
    assert_on_plan(commands, steps)


def test_an_overrun_skips_ahead_in_one_larger_step(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A tick overrunning by 2.5 periods publishes the plan at the actual time once; no catch-up burst follows."""
    arm, be = make(tmp_path)
    arm.acquire()
    first = len(be.commands)
    plain = be.joint_sample
    overrun_done = False

    def slow_sample() -> JointSample | None:
        nonlocal overrun_done
        if not overrun_done and len(be.commands) == first + OVERRUN_TICK:
            overrun_done = True
            be.t += OVERRUN_PERIODS * PERIOD
        return plain()

    monkeypatch.setattr(be, "joint_sample", slow_sample)
    res = arm.move_joints({PAN: PAN_TRAVEL})
    assert res.status == "converged", res.message
    assert overrun_done
    steps = planned_steps()
    assert res.trajectory_s == pytest.approx(steps * PERIOD, abs=PERIOD)
    commands = trajectory_commands(be, first)
    assert_on_plan(commands, steps)
    gaps = [b[0] - a[0] for a, b in zip(commands, commands[1:], strict=False)]
    assert min(gaps) >= PERIOD - 1e-9  # no burst of setpoints after the overrun
    assert sum(gap > 1.5 * PERIOD for gap in gaps) == 1
    deltas = [b[1] - a[1] for a, b in zip(commands, commands[1:], strict=False)]
    jump = gaps.index(max(gaps))
    assert deltas[jump] > 2.5 * max(deltas[jump - 1], deltas[jump + 1])


def test_jittery_ticks_keep_the_slow_zone_stretch(tmp_path: Path) -> None:
    """The body soft margin stretches the planned time; jittery ticks finish at that stretched time, not later."""
    arm, be = make(tmp_path, body_check=True)
    arm.acquire()
    steady = arm.move_joints({PAN: PAN_TRAVEL})
    assert steady.self_collision is not None and steady.ticks is not None
    stretched_s = steady.ticks * PERIOD
    assert stretched_s > planned_steps() * PERIOD + 1.0
    arm, be = make(tmp_path, body_check=True)
    arm.acquire()
    rng = random.Random(11)

    def oversleep(backend: FakeArmBackend) -> None:
        backend.t += rng.uniform(0.0, OVERSLEEP_MAX_S)

    be.on_sleep = oversleep
    res = arm.move_joints({PAN: PAN_TRAVEL})
    assert res.status == "converged", res.message
    assert res.self_collision is not None
    assert res.trajectory_s == pytest.approx(stretched_s, abs=PERIOD)


def test_plan_sample_interpolates_between_the_start_and_the_points() -> None:
    start = {PAN: 0.0}
    points = [{PAN: 0.1}, {PAN: 0.3}, {PAN: 0.4}]
    assert plan_sample(start, points, 0.0) == pytest.approx(start)
    assert plan_sample(start, points, 1.0) == pytest.approx({PAN: 0.1})
    assert plan_sample(start, points, 1.5) == pytest.approx({PAN: 0.2})
    assert plan_sample(start, points, 3.0) == pytest.approx({PAN: 0.4})
    assert plan_sample(start, points, 7.2) == pytest.approx({PAN: 0.4})  # clamped to the end
    assert plan_sample(start, points, -1.0) == pytest.approx(start)
    assert plan_sample(start, [], 2.0) == pytest.approx(start)


def test_setpoint_band_spans_the_setpoints_since_the_window_start() -> None:
    history = [(1.0, {PAN: 0.0}), (1.04, {PAN: 0.1}), (1.08, {PAN: 0.2}), (1.12, {PAN: 0.15})]
    assert setpoint_band(history, 1.05) == {PAN: (0.1, 0.2)}  # the setpoint current at 1.05 counts
    assert setpoint_band(history, 1.08) == {PAN: (0.15, 0.2)}
    assert setpoint_band(history, 9.0) == {PAN: (0.15, 0.15)}
    assert setpoint_band(history, 0.5) == {PAN: (0.0, 0.2)}  # before the history: from its oldest setpoint
    assert setpoint_band([], 1.0) == {}
