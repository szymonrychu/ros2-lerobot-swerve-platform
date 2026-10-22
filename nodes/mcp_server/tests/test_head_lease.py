"""Head on lease release (control.py, D10/U6): a release that leaves the agent with no lease homes the head; a
hand-over to the other lease, keep_position, battery cut-off, a release of nothing and an involuntary lease loss do
not move it; a stuck head is reported but never blocks the arm/base release (no ROS)."""

from pathlib import Path
from typing import Any

import pytest

from mcp_server.config import McpServerConfig
from mcp_server.control import ControlManager
from mcp_server.head import AppliedHeadKinematics, HeadController
from mcp_server.tools import build_mcp_server

from .fakes import FakeHeadBackend
from .test_battery_gate import LOW_V, make_guard
from .test_control import FakeClock, store_home_then_move_away
from .test_tools import TOKEN, FakeRobot, call

PAN, TILT = 0.4, -0.2  # away from home (home is pan 0, tilt 0.5236 by default)


def look_away(robot: FakeRobot) -> None:
    """Put the head somewhere other than home and forget that command."""
    robot.head_backend.positions.update({"head_pan": PAN, "head_tilt": TILT})
    robot.head_backend.target.update({"head_pan": PAN, "head_tilt": TILT})
    robot.head_backend.commands.clear()


def manager(robot: FakeRobot, guard: Any = None) -> ControlManager:
    clock = FakeClock()
    return ControlManager(robot.arm, robot, robot.arm.cfg, guard, clock=clock, sleep=clock.sleep, head=robot.head)


def home_command(robot: FakeRobot) -> dict[str, float]:
    cfg = robot.arm.cfg.head
    return dict(zip(cfg.joint_names, (cfg.home_pan_rad, cfg.home_tilt_rad), strict=True))


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    robot = FakeRobot(tmp_path)
    store_home_then_move_away(robot)
    look_away(robot)
    return robot


def test_release_all_homes_the_head_and_reports_homed(robot: FakeRobot) -> None:
    ctl = manager(robot)
    ctl.take("arm")
    ctl.take("base")
    result = ctl.release("all")
    assert result.reset == {"base": "neutral", "arm": "homed", "head": "homed"}
    assert robot.head_backend.commands == [home_command(robot)]
    assert list(result.reset) == ["base", "arm", "head"]  # the head last, after the arm and base resets


def test_release_base_with_the_arm_held_keeps_the_head(robot: FakeRobot) -> None:
    ctl = manager(robot)
    ctl.take("arm")
    ctl.take("base")
    result = ctl.release("base")
    assert result.reset == {"base": "neutral", "head": "kept: arm lease held"}
    assert robot.head_backend.commands == []


def test_release_arm_with_the_base_held_keeps_the_head(robot: FakeRobot) -> None:
    ctl = manager(robot)
    ctl.take("arm")
    ctl.take("base")
    result = ctl.release("arm")
    assert result.reset == {"arm": "homed", "head": "kept: base lease held"}
    assert robot.head_backend.commands == []


def test_release_arm_then_base_homes_only_at_the_second_call(robot: FakeRobot) -> None:
    ctl = manager(robot)
    ctl.take("arm")
    ctl.take("base")
    assert ctl.release("arm").reset["head"] == "kept: base lease held"
    assert robot.head_backend.commands == []
    assert ctl.release("base").reset == {"base": "neutral", "head": "homed"}
    assert robot.head_backend.commands == [home_command(robot)]


def test_keep_position_keeps_the_head(robot: FakeRobot) -> None:
    ctl = manager(robot)
    ctl.take("arm")
    ctl.take("base")
    result = ctl.release("all", keep_position=True)
    assert result.reset == {"base": "kept", "arm": "kept", "head": "kept"}
    assert robot.head_backend.commands == []


def test_battery_cutoff_skips_the_head_home(robot: FakeRobot) -> None:
    ctl = manager(robot, guard=make_guard(LOW_V))
    ctl.take("arm")
    result = ctl.release("arm")
    assert result.reset == {"arm": "skipped: battery_cutoff", "head": "skipped: battery_cutoff"}
    assert result.released == ["arm"]
    assert robot.head_backend.commands == []


def test_stuck_head_reports_failed_and_both_leases_are_released(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    store_home_then_move_away(robot)
    cfg = robot.arm.cfg.head
    robot.head_backend = FakeHeadBackend({"head_pan": PAN, "head_tilt": TILT}, speed=0.0)  # follower never moves
    robot.head = HeadController(robot.head_backend, AppliedHeadKinematics(cfg.urdf_path, None), cfg)
    ctl = manager(robot)
    ctl.take("arm")
    ctl.take("base")
    result = ctl.release("all")
    assert result.reset["head"] == "failed: timeout"
    assert sorted(result.released) == ["arm", "base"]
    assert result.reset["arm"] == "homed" and result.reset["base"] == "neutral"
    assert result.state.arm.held is False and result.state.base.held is False
    assert robot.arm.control_held is False
    assert robot.head_backend.commands == [home_command(robot)]


def test_release_of_nothing_publishes_nothing(robot: FakeRobot) -> None:
    ctl = manager(robot)
    for target in ("all", "arm", "base"):
        result = ctl.release(target)
        assert "head" not in result.reset and result.released == []
    assert robot.head_backend.commands == []


def test_involuntary_arm_lease_loss_publishes_nothing(robot: FakeRobot) -> None:
    ctl = manager(robot)
    ctl.take("arm")
    robot.arm.release()  # the lease ends outside release_control (e.g. the /arm/home service, a web UI take)
    robot.arm.acquire()  # and a new generation starts: not the agent's lease
    assert ctl.arm_held is False
    ctl.state()
    result = ctl.release("arm")
    assert result.reset["arm"].startswith("skipped: not held") and "head" not in result.reset
    assert robot.head_backend.commands == []


def test_stop_does_not_touch_the_head(robot: FakeRobot) -> None:
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    call(server, "take_control", {"subsystem": "base"})
    call(server, "stop", {})
    assert robot.head_backend.commands == []


def test_release_control_tool_homes_the_head(robot: FakeRobot) -> None:
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    call(server, "take_control", {"subsystem": "base"})
    result = call(server, "release_control", {"subsystem": "all"}).structured_content
    assert result["reset"] == {"base": "neutral", "arm": "skipped: not held", "head": "homed"}
    assert robot.head_backend.commands == [home_command(robot)]


def test_manager_without_a_head_reports_no_head_entry(robot: FakeRobot) -> None:
    clock = FakeClock()
    ctl = ControlManager(robot.arm, robot, robot.arm.cfg, None, clock=clock, sleep=clock.sleep)
    ctl.take("base")
    assert ctl.release("base").reset == {"base": "neutral"}
    assert robot.head_backend.commands == []
