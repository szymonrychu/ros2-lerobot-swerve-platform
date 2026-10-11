"""Control leases and the unified carrying signal (control.py, arm.py, carry.py): the lease generation that keeps a
human override from being reversed (H1), the agent lease separate from implicit holders (H2), the grip torque kept by
a keep_position release while carrying (H3), unverified holds counting as carrying (H4), release refused while an arm
motion runs (M1), the base lease cleared before its reset (M3), release_control(all) while carrying (M4), a zero
carried radius (M6), the carrying state in get_control_state after take_control (live L2), its persistence across a
restart (H7), the stop during a park (L3), the lost_note after
a filter_node restart (L2), the home pose cache (L1) and own Nav2 goals (M2). No ROS."""

import threading
from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server import arm as arm_module
from mcp_server.arm import ArmController
from mcp_server.carry import CARRY_FILE_NAME
from mcp_server.config import McpServerConfig
from mcp_server.control import NavGoalRegistry
from mcp_server.grasp import ObjectSpec, grasp_params
from mcp_server.grasp_tools import GraspExecutor, GraspService
from mcp_server.ik import ArmKinematics, load_joint_limits
from mcp_server.tools import build_mcp_server

from .test_control import FakeClock, manager, store_home_then_move_away
from .test_grasp_tools import OBJECT, plush_in_jaws
from .test_motion_queue import WAIT
from .test_tools import TOKEN, FakeRobot, call

CARRIED = 0.08
STOP_JAW = 0.29  # an object between the jaws: 0.455 rad short of closed, outside the empty band
HOLD_LOAD = 200.0  # closing load on the object (soft target_load 150: the close stops there)


def object_between_jaws(robot: FakeRobot) -> None:
    """The jaw cannot close beyond STOP_JAW and then feels HOLD_LOAD (a healthy hold: no re-tighten)."""
    be = robot.arm_backend
    be.positions["gripper"] = 0.8

    def blocked(b: Any) -> None:
        b.positions["gripper"] = max(b.positions["gripper"], STOP_JAW)
        b.efforts["gripper"] = HOLD_LOAD if b.positions["gripper"] <= STOP_JAW + 1e-9 else 0.0

    be.on_sleep = blocked


def empty_jaws(robot: FakeRobot) -> None:
    """Nothing in the jaws any more: no load, the jaw follows every command."""
    robot.arm_backend.on_sleep = None
    robot.arm_backend.efforts["gripper"] = 0.0


def torque_writes(robot: FakeRobot) -> list[int]:
    return [v for j, r, v in robot.arm_backend.register_writes if j == "gripper" and r == "torque_limit"]


def soft_grasp(robot: FakeRobot) -> None:
    object_between_jaws(robot)
    res = robot.arm.set_gripper(close_until_effort=True, grip_profile="soft")
    assert res.status == "grasped", res.message


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    return FakeRobot(tmp_path)


@pytest.fixture
def server(robot: FakeRobot) -> Any:
    return build_mcp_server(robot, McpServerConfig(), TOKEN)


# --- H1: the lease generation ---------------------------------------------------------------------------------------


def test_grasp_object_refused_when_lease_released_during_planning(
    server: Any, robot: FakeRobot, monkeypatch: pytest.MonkeyPatch
) -> None:
    robot.arm.set_home()
    call(server, "take_control", {"subsystem": "arm"})
    plan = GraspExecutor.plan

    def plan_then_web_home(self: GraspExecutor, *args: Any, **kwargs: Any) -> Any:
        result = plan(self, *args, **kwargs)
        ok, _ = robot.arm.home_service_call()  # the web UI Home button while grasp_object plans
        assert ok
        return result

    monkeypatch.setattr(GraspExecutor, "plan", plan_then_web_home)
    result = call(server, "grasp_object", {"object": OBJECT, "strategy": "top_down"}).structured_content
    assert result["outcome"] == "aborted"
    text = " ".join(result["reasons"])
    assert "arm control not held (released by the /arm/home service" in text and "take_control" in text
    assert robot.arm.control_held is False  # not silently re-taken by the grasp or its abort hold
    assert robot.arm_backend.releases == 1
    with pytest.raises(ToolError, match="arm control not held"):
        call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}})


def test_queued_grasp_step_refused_when_lease_lost_after_dispatch(
    server: Any, robot: FakeRobot, monkeypatch: pytest.MonkeyPatch
) -> None:
    call(server, "take_control", {"subsystem": "arm"})
    plan = GraspExecutor.plan

    def plan_then_override(self: GraspExecutor, *args: Any, **kwargs: Any) -> Any:
        result = plan(self, *args, **kwargs)
        robot.arm.release()  # a human ended the lease while the queued grasp planned...
        robot.arm.acquire()  # ...and another caller (web UI grasp) took the arm implicitly
        return result

    monkeypatch.setattr(GraspExecutor, "plan", plan_then_override)
    call(server, "enqueue_motions", {"steps": [{"kind": "grasp", "object": OBJECT, "strategy": "top_down"}]})
    events = call(server, "wait_for_event", {"until": "queue_empty", "timeout_s": WAIT}).structured_content["events"]
    failed = [e for e in events if e["type"] == "step_failed"]
    assert failed and "arm control not held" in failed[0]["message"], events


def test_implicit_acquire_still_works_without_an_expected_lease(robot: FakeRobot) -> None:
    """GraspService and /arm/home pass no lease generation: the first motion takes the arm as before."""
    res = robot.arm.move_joints({"elbow_flex": 0.2})
    assert res.status == "converged" and robot.arm.control_held


def test_expected_lease_generation_mismatch_refuses_a_motion(robot: FakeRobot) -> None:
    robot.arm.acquire()
    stale = robot.arm.lease_generation
    robot.arm.release()
    robot.arm.acquire()
    token = arm_module.EXPECTED_LEASE.set(stale)
    try:
        with pytest.raises(arm_module.ArmError, match="arm control not held"):
            robot.arm.move_joints({"elbow_flex": 0.2})
    finally:
        arm_module.EXPECTED_LEASE.reset(token)


# --- H2: the agent lease is not ArmController._held ----------------------------------------------------------------


def test_web_ui_grasp_does_not_grant_agent_arm_lease(server: Any, robot: FakeRobot) -> None:
    plush_in_jaws(robot.arm_backend)
    done = GraspService(robot.arm, robot.arm.cfg).handle(
        {"action": "execute", "object": OBJECT, "strategy": "top_down"}
    )
    assert done["ok"] is True and robot.arm.control_held is True  # the web UI grasp holds the arm implicitly
    with pytest.raises(ToolError, match="arm control not held"):
        call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}})
    state = call(server, "get_control_state", {}).structured_content
    assert state["arm"]["held"] is False and state["arm"]["held_since"] is None
    assert state["arm"]["held_by"] == "mcp_server"
    taken = call(server, "take_control", {"subsystem": "arm"}).structured_content
    assert taken["arm"]["held"] is True and taken["arm"]["held_by"] == "agent"


def test_stop_hold_during_foreign_motion_does_not_grant_lease(robot: FakeRobot) -> None:
    ctl = manager(robot)
    with robot.arm.exclusive_motion():  # a web UI motion runs (no agent lease)
        held, _ = robot.arm.stop_hold()
    assert held is True and robot.arm.control_held is True
    assert ctl.held("arm") is False
    assert ctl.state().arm.held is False and ctl.state().arm.held_by == "mcp_server"
    assert ctl.release("arm").reset == {"arm": "skipped: not held"}


def test_agent_lease_ends_when_the_arm_is_taken_over(robot: FakeRobot) -> None:
    ctl = manager(robot)
    ctl.take("arm")
    assert ctl.held("arm") and ctl.state().arm.held_by == "agent"
    robot.arm.release()
    robot.arm.acquire()  # re-taken implicitly by someone else: a new lease generation
    assert ctl.held("arm") is False


# --- H3: grip torque on a keep_position release while carrying -----------------------------------------------------


def test_keep_position_release_while_carrying_keeps_grip_torque(robot: FakeRobot) -> None:
    robot.arm.set_home()
    ctl = manager(robot)
    ctl.take("arm")
    soft_grasp(robot)
    soft = robot.arm.cfg.grip_profiles.presets["soft"].torque_limit
    assert torque_writes(robot)[-1] == soft
    writes = len(torque_writes(robot))
    result = ctl.release("arm", keep_position=True, carried_object_radius_m=CARRIED)
    assert result.reset == {"arm": "kept"}
    assert len(torque_writes(robot)) == writes  # no torque_limit write: the soft grip keeps holding
    ctl.take("arm")
    empty_jaws(robot)
    robot.arm.set_gripper(open_fraction=0.8)  # release_object's open
    assert ctl.release("arm").reset == {"arm": "homed"}
    assert torque_writes(robot)[-1] == robot.arm.cfg.grip_profiles.default.torque_limit


def test_keep_position_release_without_carrying_restores_the_default_torque(robot: FakeRobot) -> None:
    ctl = manager(robot)
    ctl.take("arm")
    robot.arm.write_torque_limit(600)
    ctl.release("arm", keep_position=True)
    assert torque_writes(robot)[-1] == robot.arm.cfg.grip_profiles.default.torque_limit


# --- H4 / M4 / M6 / live L2: one carrying signal ---------------------------------------------------------------------


def test_after_unverified_release_control_arm_refused_as_carrying(robot: FakeRobot) -> None:
    robot.arm.set_home()
    ctl = manager(robot)
    ctl.take("arm")
    plush_in_jaws(robot.arm_backend)
    result = GraspExecutor(robot.arm, robot.arm.cfg).grasp(
        ObjectSpec(**OBJECT), "top_down", grasp_params(robot.arm.cfg.grasp, None), None, None, lambda: False
    )
    assert result.outcome == "unverified", result.reasons
    carrying = ctl.state().arm.carrying
    assert carrying.carrying is True and any("unverified" in r for r in carrying.reasons)
    with pytest.raises(ToolError, match="cannot park while the gripper carries an object"):
        ctl.release("arm")
    assert robot.arm.control_held is True


def test_an_open_clears_the_unverified_hold(robot: FakeRobot) -> None:
    ctl = manager(robot)
    ctl.take("arm")
    plush_in_jaws(robot.arm_backend)
    GraspExecutor(robot.arm, robot.arm.cfg).grasp(
        ObjectSpec(**OBJECT), "top_down", grasp_params(robot.arm.cfg.grasp, None), None, None, lambda: False
    )
    assert ctl.state().arm.carrying.carrying is True
    empty_jaws(robot)
    robot.arm.set_gripper(open_fraction=0.8)
    assert ctl.state().arm.carrying.carrying is False
    assert ctl.release("arm").released == ["arm"]


def test_release_all_while_carrying_still_releases_base(robot: FakeRobot) -> None:
    robot.arm.set_home()
    ctl = manager(robot)
    ctl.take("arm")
    ctl.take("base")
    robot.arm.grasp_held = True
    result = ctl.release("all")
    assert result.released == ["base"]
    assert result.reset["base"] == "neutral"
    assert result.reset["arm"].startswith("refused: carrying (") and "last grasp" in result.reset["arm"]
    assert robot.arm.control_held is True and result.state.arm.held is True
    assert result.state.base.held is False
    with pytest.raises(ToolError, match="put the object down first"):
        ctl.release("arm")


def test_grasp_held_cleared_by_measured_open_jaw(robot: FakeRobot) -> None:
    ctl = manager(robot)
    ctl.take("arm")
    ctl.release("arm", keep_position=True, carried_object_radius_m=CARRIED)
    robot.arm.grasp_held = True
    assert ctl.state().arm.carrying.carrying is True
    # A human empties the gripper with the leader arm: the measured jaw is wide open (beyond any grasp).
    robot.arm_backend.positions["gripper"] = robot.arm.cfg.limits.gripper_grasp_max_open_rad + 0.1
    carrying = ctl.state().arm.carrying
    assert carrying.carrying is False and carrying.reasons == [] and carrying.radius_m is None
    assert robot.arm.grasp_held is False


def test_zero_radius_does_not_block_parking(robot: FakeRobot) -> None:
    robot.arm.set_home()
    ctl = manager(robot)
    ctl.take("arm")
    ctl.release("arm", keep_position=True, carried_object_radius_m=0.0)
    state = ctl.take("arm")
    assert state.arm.carrying.carrying is False and state.arm.carrying.radius_m is None
    assert ctl.release("arm").reset == {"arm": "homed"}


def test_carrying_visible_in_get_control_state_after_take_control(server: Any, robot: FakeRobot) -> None:
    robot.arm.set_home()
    call(server, "take_control", {"subsystem": "arm"})
    call(server, "release_control", {"subsystem": "arm", "keep_position": True, "carried_object_radius_m": 0.05})
    state = call(server, "take_control", {"subsystem": "arm"}).structured_content
    carrying = state["arm"]["carrying"]
    assert carrying["carrying"] is True and carrying["radius_m"] == 0.05
    assert any("carried_object_radius_m 0.05" in r for r in carrying["reasons"])
    again = call(server, "get_control_state", {}).structured_content["arm"]["carrying"]
    assert again == carrying
    with pytest.raises(ToolError) as info:
        call(server, "release_control", {"subsystem": "arm"})
    assert "carried_object_radius_m 0.05" in str(info.value)


def test_not_carrying_reports_an_empty_state(robot: FakeRobot) -> None:
    carrying = manager(robot).state().arm.carrying
    assert carrying.model_dump() == {"carrying": False, "reasons": [], "radius_m": None}


# --- H7: persistence next to home_file, orphan release note, /arm/home footprint --------------------------------------


def restarted(robot: FakeRobot) -> ArmController:
    cfg = robot.arm.cfg
    return ArmController(
        robot.arm_backend,
        ArmKinematics(cfg.arm.urdf_path, margin=cfg.limits.arm_limit_margin_rad),
        load_joint_limits(cfg.arm.urdf_path),
        cfg,
    )


def test_carried_radius_survives_restart(robot: FakeRobot) -> None:
    robot.arm.set_home()
    ctl = manager(robot)
    ctl.take("arm")
    robot.arm.grasp_held = True
    ctl.release("arm", keep_position=True, carried_object_radius_m=CARRIED)
    assert (robot.arm.cfg.arm.home_file.parent / CARRY_FILE_NAME).is_file()
    robot.arm = restarted(robot)  # mcp_server restarted (crash, redeploy)
    ctl2 = manager(robot)
    carrying = ctl2.state().arm.carrying
    assert carrying.carrying is True and carrying.radius_m == CARRIED
    assert any("last grasp" in r for r in carrying.reasons)
    ctl2.take("arm")
    with pytest.raises(ToolError, match="cannot park"):
        ctl2.release("arm")


def test_orphan_lease_release_notes_the_restart(robot: FakeRobot) -> None:
    robot.arm_backend.source = "autonomy"
    outcome = robot.arm.check_orphan_lease(started_at=robot.arm_backend.t - 1.0)
    assert outcome == "released"
    assert robot.arm.lost_note is not None and "mcp_server restarted" in robot.arm.lost_note


# --- M1 / M3 / L3 ------------------------------------------------------------------------------------------------------


def test_release_refused_while_an_arm_motion_runs(robot: FakeRobot) -> None:
    robot.arm.set_home()
    ctl = manager(robot)
    ctl.take("arm")
    with robot.arm.exclusive_motion():  # another arm tool's motion is running
        with pytest.raises(ToolError, match="arm motion is running"):
            ctl.release("arm")
    assert robot.arm.control_held is True and robot.arm_backend.releases == 0


def test_release_refused_while_another_blocking_tool_runs(robot: FakeRobot) -> None:
    ctl = manager(robot)
    ctl.blocking_tool = lambda: "navigate_to_pose"
    ctl.take("base")
    with pytest.raises(ToolError, match="navigate_to_pose is running"):
        ctl.release("base")
    ctl.blocking_tool = lambda: "release_control"  # release_control itself does not count
    assert ctl.release("base").released == ["base"]


def test_base_tool_refused_during_base_reset(robot: FakeRobot) -> None:
    clock = FakeClock()
    ctl = manager(robot, clock=clock)
    ctl.take("base")
    seen: list[str | None] = []

    def resetting() -> bool:
        seen.append(ctl.refusal("drive", "base"))
        return True

    robot.reset_base = resetting  # type: ignore[method-assign]
    ctl.release("base")
    assert seen and seen[0] is not None and "base control not held" in seen[0]


def test_take_base_during_release_is_kept(robot: FakeRobot) -> None:
    clock = FakeClock()
    ctl = manager(robot, clock=clock)
    ctl.take("base")
    robot.steer = {j: 0.4 for j in ("fl_steer", "fr_steer", "rl_steer", "rr_steer")}
    taken: list[bool] = []

    def sleep(seconds: float) -> None:
        clock.sleep(seconds)
        if not taken:
            ctl.take("base")  # the agent takes the base again while the release waits for neutral steering
            taken.append(True)
            robot.steer = {j: 0.0 for j in ("fl_steer", "fr_steer", "rl_steer", "rr_steer")}

    ctl.sleep = sleep
    result = ctl.release("base")
    assert result.reset == {"base": "neutral"}
    assert ctl.base_held is True and ctl.state().base.held is True


def test_stop_during_a_park_does_not_release_the_arm(robot: FakeRobot) -> None:
    store_home_then_move_away(robot)
    ctl = manager(robot)
    ctl.take("arm")
    robot.arm_backend.on_publish_command = lambda b: robot.arm.request_stop()  # stop pressed mid-park
    result = ctl.release("arm")
    assert result.reset["arm"].startswith("stopped:"), result.reset
    assert result.released == []
    assert robot.arm.control_held is True and robot.arm_backend.releases == 0


# --- L2: filter_node restart lost note -----------------------------------------------------------------------------


def test_lost_note_after_a_filter_node_restart(robot: FakeRobot) -> None:
    robot.arm.acquire()
    robot.arm_backend.t += 5.0
    robot.arm_backend.source = "none"
    robot.arm.keepalive_tick()
    assert robot.arm.control_held is False
    assert robot.arm.lost_note == "filter_node reports no active source (restarted?)"


# --- L1: home pose cache -------------------------------------------------------------------------------------------


def test_home_pose_read_once_until_the_file_changes(robot: FakeRobot, monkeypatch: pytest.MonkeyPatch) -> None:
    robot.arm.set_home()
    loads: list[Path] = []
    real = arm_module.load_home

    def counting(path: Path) -> Any:
        loads.append(path)
        return real(path)

    monkeypatch.setattr(arm_module, "load_home", counting)
    for _ in range(5):
        robot.arm.at_home()
        robot.arm.home_stored()
    assert len(loads) <= 1
    robot.arm_backend.positions["elbow_flex"] = 0.3
    robot.arm.set_home()
    assert robot.arm.at_home() is True  # the new pose is read after the write
    assert len(loads) <= 2


# --- M2: own Nav2 goals --------------------------------------------------------------------------------------------


def test_release_base_cancels_only_own_nav_goals() -> None:
    registry = NavGoalRegistry()
    mine = [object(), object()]
    human = object()  # a web UI goal: never registered
    for handle in mine:
        registry.add(handle)
    registry.discard(mine[1])  # finished on its own
    third = object()
    registry.add(third)
    cancelled: list[object] = []
    assert registry.cancel(lambda h: cancelled.append(h) or True) is True
    assert cancelled == [mine[0], third] and human not in cancelled
    assert registry.cancel(lambda h: False) is True  # nothing left to cancel
    registry.add(human)
    assert registry.cancel(lambda h: False) is False


def test_registry_is_thread_safe() -> None:
    registry = NavGoalRegistry()

    def add_many() -> None:
        for _ in range(200):
            registry.add(object())

    threads = [threading.Thread(target=add_many) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    count: list[int] = []
    registry.cancel(lambda h: count.append(1) or True)
    assert len(count) == 800
