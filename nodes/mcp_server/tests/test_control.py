"""Per-subsystem control leases (control.py): take/release/get_control_state, the central lease check, the
per-step enqueue check, the arm home reset, the steering-neutral base reset and the vitals digest (no ROS)."""

from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server.config import McpServerConfig
from mcp_server.control import ControlManager, steering_neutral, wait_steering_neutral
from mcp_server.monitor import RobotMonitor
from mcp_server.tools import TOOL_SUBSYSTEM, build_mcp_server

from .test_battery_gate import LOW_V, make_guard
from .test_motion_queue import WAIT, GateRobot, make_queue, steps
from .test_tilt_guard import state
from .test_tools import TOKEN, FakeRobot, call

STEERS = ("fl_steer", "fr_steer", "rl_steer", "rr_steer")
ARM_REFUSAL = "move_arm_joints refused: arm control not held - call take_control(subsystem='arm')"
BASE_REFUSAL = "navigate_to_pose refused: base control not held - call take_control(subsystem='base')"


class FakeClock:
    """Monotonic clock advanced by sleep."""

    def __init__(self) -> None:
        self.t = 50.0
        self.sleeps = 0

    def __call__(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        self.t += seconds
        self.sleeps += 1


class SteeringSource:
    """Steer joint feed that reaches neutral after `settle_after` reads (never when None)."""

    def __init__(self, start: float, settle_after: int | None) -> None:
        self.start = start
        self.settle_after = settle_after
        self.reads = 0

    def __call__(self) -> dict[str, float] | None:
        self.reads += 1
        neutral = self.settle_after is not None and self.reads > self.settle_after
        return {j: 0.0 if neutral else self.start for j in STEERS}


def manager(robot: FakeRobot, guard: Any = None, busy: bool = False, clock: FakeClock | None = None) -> ControlManager:
    clock = clock or FakeClock()
    return ControlManager(
        robot.arm, robot, robot.arm.cfg, guard, queue_busy=lambda: busy, clock=clock, sleep=clock.sleep
    )


def store_home_then_move_away(robot: FakeRobot) -> None:
    robot.arm.set_home()
    robot.arm_backend.positions["elbow_flex"] = 0.4


# --- steering waiter ---------------------------------------------------------------------------------------------


def test_steering_neutral_judges_every_steer_joint() -> None:
    assert steering_neutral({j: 0.04 for j in STEERS}, STEERS, 0.05) is True
    assert steering_neutral({**{j: 0.0 for j in STEERS}, "rr_steer": -0.06}, STEERS, 0.05) is False
    assert steering_neutral({"fl_steer": 0.0}, STEERS, 0.05) is None
    assert steering_neutral(None, STEERS, 0.05) is None


def test_steering_waiter_succeeds_once_the_wheels_recenter() -> None:
    clock = FakeClock()
    source = SteeringSource(0.4, settle_after=3)
    ok, why = wait_steering_neutral(source, STEERS, 0.05, 6.0, clock, clock.sleep, 0.1)
    assert ok is True and why == ""
    assert source.reads == 4
    assert clock.t == pytest.approx(50.3)


def test_steering_waiter_times_out_naming_the_worst_joint() -> None:
    clock = FakeClock()
    ok, why = wait_steering_neutral(SteeringSource(0.4, settle_after=None), STEERS, 0.05, 6.0, clock, clock.sleep, 0.1)
    assert ok is False
    assert "not neutral within 6 s" in why and "0.40 rad" in why
    assert clock.t == pytest.approx(56.0, abs=0.11)


def test_steering_waiter_reports_missing_joint_states() -> None:
    clock = FakeClock()
    ok, why = wait_steering_neutral(lambda: None, STEERS, 0.05, 1.0, clock, clock.sleep, 0.1)
    assert ok is False and "no fresh" in why


# --- take / release ------------------------------------------------------------------------------------------------


def test_take_control_is_idempotent_and_keeps_held_since(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    ctl = manager(robot)
    first = ctl.take("arm")
    commands = len(robot.arm_backend.commands)
    robot.arm_backend.t += 5.0
    again = ctl.take("arm")
    assert first.arm.held and again.arm.held
    assert again.arm.held_since == pytest.approx(first.arm.held_since, abs=1e-6)
    assert len(robot.arm_backend.commands) == commands  # a no-op: the hold target is not re-published
    base = ctl.take("base")
    assert base.base.held and base.base.held_since is not None
    assert ctl.take("base").base.held_since == base.base.held_since


def test_release_arm_runs_home_then_publishes_the_release(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    store_home_then_move_away(robot)
    ctl = manager(robot)
    ctl.take("arm")
    robot.arm_backend.events.clear()
    result = ctl.release("arm", keep_position=False)
    assert result.released == ["arm"]
    assert result.reset == {"arm": "homed"}
    kinds = [kind for kind, _ in robot.arm_backend.events]
    assert kinds[-1] == "release" and kinds.count("release") == 1
    assert "command" in kinds[:-1]
    assert robot.arm_backend.positions["elbow_flex"] == pytest.approx(0.0, abs=0.01)
    assert robot.arm.control_held is False
    assert result.state.arm.held is False and result.state.arm.at_home is True


def test_release_arm_keep_position_only_releases(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    store_home_then_move_away(robot)
    ctl = manager(robot)
    ctl.take("arm")
    robot.arm_backend.events.clear()
    result = ctl.release("arm", keep_position=True)
    assert result.reset == {"arm": "kept"}
    assert robot.arm_backend.events == [("release", None)]
    assert robot.arm_backend.positions["elbow_flex"] == pytest.approx(0.4)


def test_release_arm_without_home_pose_still_releases(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    ctl = manager(robot)
    ctl.take("arm")
    result = ctl.release("arm", keep_position=False)
    assert result.reset == {"arm": "failed: no home pose stored"}
    assert result.released == ["arm"]
    assert robot.arm.control_held is False and robot.arm_backend.releases == 1


def test_release_arm_reports_a_failed_home_and_still_releases(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    store_home_then_move_away(robot)
    ctl = manager(robot)
    ctl.take("arm")
    robot.arm_backend.follow = False  # the joints never reach home: the home motion times out
    result = ctl.release("arm", keep_position=False)
    assert result.reset["arm"].startswith("failed: ")
    assert robot.arm.control_held is False and robot.arm_backend.releases == 1


def test_battery_cutoff_skips_the_arm_reset(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    store_home_then_move_away(robot)
    ctl = manager(robot, guard=make_guard(LOW_V))
    ctl.take("arm")
    robot.arm_backend.events.clear()
    result = ctl.release("arm", keep_position=False)
    assert result.reset == {"arm": "skipped: battery_cutoff"}
    assert robot.arm_backend.events == [("release", None)]


def test_release_base_zeroes_and_verifies_neutral_steering(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    ctl = manager(robot)
    ctl.take("base")
    result = ctl.release("base", keep_position=False)
    assert result.reset == {"base": "neutral"}
    assert robot.base_resets == 1
    assert result.state.base.held is False and result.state.base.steering_neutral is True


def test_release_base_timeout_reports_failed_and_releases(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    robot.steer = {j: 0.3 for j in STEERS}
    clock = FakeClock()
    ctl = manager(robot, clock=clock)
    ctl.take("base")
    result = ctl.release("base", keep_position=False)
    assert result.reset["base"].startswith("failed: ") and "not neutral" in result.reset["base"]
    assert result.released == ["base"] and result.state.base.held is False
    assert clock.t - 50.0 == pytest.approx(robot.arm.cfg.control.base_reset_timeout_s, abs=0.2)


def test_release_base_keep_position_only_clears_the_flag(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    ctl = manager(robot)
    ctl.take("base")
    result = ctl.release("base", keep_position=True)
    assert result.reset == {"base": "kept"} and robot.base_resets == 0
    assert result.state.base.held is False


def test_release_all_resets_both_and_skips_what_is_not_held(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    store_home_then_move_away(robot)
    ctl = manager(robot)
    ctl.take("base")
    result = ctl.release("all", keep_position=False)
    assert result.released == ["base"]
    assert result.reset == {"base": "neutral", "arm": "skipped: not held"}
    assert robot.arm_backend.releases == 0


def test_release_refused_while_the_queue_is_busy(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    ctl = manager(robot, busy=True)
    ctl.take("base")
    with pytest.raises(ToolError, match="cancel_motions or wait_for_event first"):
        ctl.release("base", keep_position=False)
    assert ctl.state().base.held is True


# --- at_home --------------------------------------------------------------------------------------------------------


def test_at_home_tolerance_edges_with_the_gripper_excluded(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    assert robot.arm.at_home() is None  # no home pose stored
    robot.arm.set_home()
    limits = robot.arm.cfg.limits
    tol = limits.settle_tolerance_for("shoulder_lift")
    robot.arm_backend.positions["shoulder_lift"] = tol * 0.99
    assert robot.arm.at_home() is True
    robot.arm_backend.positions["shoulder_lift"] = tol * 1.01
    assert robot.arm.at_home() is False
    robot.arm_backend.positions["shoulder_lift"] = 0.0
    robot.arm_backend.positions["gripper"] = 1.2  # holding an object: still home
    assert robot.arm.at_home() is True
    robot.arm_backend.no_samples = True
    assert robot.arm.at_home() is None


# --- the central check (server) -------------------------------------------------------------------------------------


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    return FakeRobot(tmp_path)


@pytest.fixture
def server(robot: FakeRobot) -> Any:
    return build_mcp_server(robot, McpServerConfig(), TOKEN)


def test_tool_subsystem_table() -> None:
    assert {n for n, s in TOOL_SUBSYSTEM.items() if s == "arm"} == {
        "move_arm_joints",
        "move_arm_cartesian",
        "set_gripper",
        "arm_home",
        "grasp_object",
        "release_object",
    }
    assert {n for n, s in TOOL_SUBSYSTEM.items() if s == "base"} == {
        "navigate_to_pose",
        "move_relative",
        "drive",
        "look_around",
    }
    for free in ("stop", "cancel_motions", "take_control", "release_control", "get_control_state", "enqueue_motions"):
        assert free not in TOOL_SUBSYSTEM


def test_arm_tool_without_lease_refused_before_any_publish(server: Any, robot: FakeRobot) -> None:
    with pytest.raises(ToolError) as info:
        call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}})
    assert ARM_REFUSAL in str(info.value)
    assert robot.arm_backend.commands == [] and robot.arm.control_held is False


def test_base_tool_without_lease_refused_before_any_publish(server: Any, robot: FakeRobot) -> None:
    with pytest.raises(ToolError) as info:
        call(server, "navigate_to_pose", {"x": 1.0, "y": 0.0})
    assert BASE_REFUSAL in str(info.value)
    assert robot.calls == []


def test_tools_work_with_their_lease(server: Any, robot: FakeRobot) -> None:
    state = call(server, "take_control", {"subsystem": "base"}).structured_content
    assert state["base"]["held"] is True and state["arm"]["held"] is False
    call(server, "drive", {"vx": 0.1})
    assert robot.calls[-1][0] == "drive"
    with pytest.raises(ToolError, match="arm control not held"):
        call(server, "set_gripper", {"open_fraction": 0.5})
    call(server, "take_control", {"subsystem": "arm"})
    res = call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}, "speed_scale": 0.5})
    assert res.structured_content["status"] == "converged"


def test_lost_arm_names_who_took_it(server: Any, robot: FakeRobot) -> None:
    call(server, "take_control", {"subsystem": "arm"})
    robot.arm.set_home()
    ok, _ = robot.arm.home_service_call()  # web UI Home button
    assert ok and robot.arm.control_held is False
    with pytest.raises(ToolError) as info:
        call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}})
    text = str(info.value)
    assert "arm control not held (released by the /arm/home service" in text
    assert "take_control(subsystem='arm')" in text


def test_arm_home_service_still_releases(robot: FakeRobot) -> None:
    ctl = manager(robot)
    store_home_then_move_away(robot)
    ctl.take("arm")
    ok, _ = robot.arm.home_service_call()
    assert ok is True
    assert robot.arm.control_held is False and robot.arm_backend.releases == 1


def test_arm_home_tool_requires_and_keeps_the_lease(server: Any, robot: FakeRobot) -> None:
    robot.arm.set_home()
    with pytest.raises(ToolError, match="arm_home refused: arm control not held"):
        call(server, "arm_home", {})
    call(server, "take_control", {"subsystem": "arm"})
    assert call(server, "arm_home", {}).structured_content["status"] == "converged"
    assert robot.arm.control_held is True and robot.arm_backend.releases == 0


def test_enqueue_mixing_arm_and_base_with_only_base_held_is_refused_naming_arm(server: Any, robot: FakeRobot) -> None:
    call(server, "take_control", {"subsystem": "base"})
    mixed = [{"kind": "base_relative", "dx": 0.2}, {"kind": "gripper", "open_fraction": 0.5}]
    with pytest.raises(ToolError) as info:
        call(server, "enqueue_motions", {"steps": mixed})
    text = str(info.value)
    assert "enqueue_motions refused: arm control not held" in text and "base control" not in text
    assert "take_control(subsystem='arm')" in text
    assert call(server, "get_motion_status", {}).structured_content["queue"] == []


def test_enqueue_without_lease_steps_needs_no_lease(server: Any) -> None:
    res = call(server, "enqueue_motions", {"steps": [{"kind": "wait_s", "seconds": 0.01}]})
    assert res.structured_content["job_ids"]


def test_lease_refusal_comes_before_the_tilt_refusal(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    config = McpServerConfig()
    monitor = RobotMonitor(config.monitor, cpu_temp_reader=lambda: 50.0, throttled_reader=lambda: None)
    monitor.on_tilt_guard(state("warning", roll=9.0))
    server = build_mcp_server(robot, config, TOKEN, monitor=monitor)
    with pytest.raises(ToolError) as info:
        call(server, "navigate_to_pose", {"x": 1.0, "y": 0.0})
    assert "base control not held" in str(info.value).splitlines()[0]
    assert "tilt guard warning" not in str(info.value).splitlines()[0]
    with pytest.raises(ToolError) as info:
        call(server, "enqueue_motions", {"steps": [{"kind": "base_relative", "dx": 0.2}]})
    assert "base control not held" in str(info.value).splitlines()[0]
    call(server, "take_control", {"subsystem": "base"})
    with pytest.raises(ToolError) as info:
        call(server, "navigate_to_pose", {"x": 1.0, "y": 0.0})
    assert "tilt guard" in str(info.value).splitlines()[0]


def test_take_control_allowed_under_battery_cutoff(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    server = build_mcp_server(robot, McpServerConfig(), TOKEN, make_guard(LOW_V))
    state = call(server, "take_control", {"subsystem": "arm"}).structured_content
    assert state["arm"]["held"] is True
    with pytest.raises(ToolError, match="battery below cut-off"):
        call(server, "set_gripper", {"open_fraction": 0.5})
    result = call(server, "release_control", {"subsystem": "arm"}).structured_content
    assert result["reset"]["arm"] == "skipped: battery_cutoff" and result["released"] == ["arm"]


def test_get_control_state_and_release_tool(server: Any, robot: FakeRobot) -> None:
    robot.arm.set_home()
    state = call(server, "get_control_state", {}).structured_content
    assert state["arm"] == {
        "held": False,
        "held_since": None,
        "at_home": True,
        "home_stored": True,
        "active_source": "autonomy",
        "lost": None,
    }
    assert state["base"] == {"held": False, "held_since": None, "steering_neutral": True}
    call(server, "take_control", {"subsystem": "arm"})
    call(server, "take_control", {"subsystem": "base"})
    result = call(server, "release_control", {"subsystem": "all"}).structured_content
    assert sorted(result["released"]) == ["arm", "base"]
    assert result["reset"] == {"base": "neutral", "arm": "homed"}
    assert result["state"]["arm"]["held"] is False and result["state"]["base"]["held"] is False


def test_vitals_digest_carries_the_lease_state(server: Any, robot: FakeRobot) -> None:
    robot.arm.set_home()
    vitals = call(server, "get_control_state", {}).structured_content["vitals"]
    assert "ctl arm=-@home base=-" in vitals
    call(server, "take_control", {"subsystem": "arm"})
    robot.arm_backend.positions["elbow_flex"] = 0.5
    vitals = call(server, "take_control", {"subsystem": "base"}).structured_content["vitals"]
    assert "ctl arm=H@away base=H" in vitals


def test_any_lease_need_is_satisfied_by_either_lease(robot: FakeRobot) -> None:
    ctl = manager(robot)
    assert "arm or base control not held" in (ctl.refusal("head_look", "any") or "")
    ctl.take("base")
    assert ctl.refusal("head_look", "any") is None


def test_queued_arm_step_refused_at_dispatch_after_the_lease_is_lost(tmp_path: Path) -> None:
    robot = GateRobot(tmp_path)
    robot.gate.set()
    ctl = manager(robot)
    queue = make_queue(robot, lease_gate=ctl.step_refusal)
    queue.enqueue(steps({"kind": "gripper", "open_fraction": 0.5}))
    result = queue.wait(WAIT, "queue_empty")
    failed = [e for e in result.events if e.type == "step_failed"]
    assert failed and "arm control not held" in failed[0].message


def test_release_control_counts_as_a_blocking_motion_while_it_parks(server: Any, robot: FakeRobot) -> None:
    seen: list[str | None] = []
    original = server.control.release

    def spy(*args: Any) -> Any:
        seen.append(server.blocking_tool())
        return original(*args)

    server.control.release = spy
    call(server, "release_control", {"subsystem": "all"})
    assert seen == ["release_control"]
    assert server.blocking_tool() is None
