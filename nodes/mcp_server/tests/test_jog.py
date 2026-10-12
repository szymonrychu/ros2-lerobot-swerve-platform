"""Tests for mcp_server.jog: the web-UI 'jog' action on /grasp/command (cartesian steps in the gripper frame through
IK, joint steps through move_joints, step caps, refusals and the last-request-wins coalescing)."""

import json
import math
import threading
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from mcp_server.arm import ArmController, ArmError, SelfCollisionError
from mcp_server.config import JogSettings, McpServerConfig
from mcp_server.grasp_tools import GraspService
from mcp_server.ik import ArmKinematics, UnreachableError, load_joint_limits
from mcp_server.jog import Jogger, JogSpec, gripper_axes

from .fakes import FakeArmBackend

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
START = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 1.2,
    "wrist_flex": 0.3,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}
CART_CAP = 0.01
JOINT_CAP = 0.05


class Cutoff:
    def is_cutoff(self) -> bool:
        return True

    def rejection_message(self) -> str:
        return "battery below cut-off: motion refused"


class FakeCollision(SelfCollisionError):
    """SelfCollisionError without a BodyReport (the body check itself is covered by test_self_collision)."""

    def __init__(self) -> None:
        self.details: dict[str, object] = {"obstacle": "chassis"}
        ArmError.__init__(self, "self_collision: gripper_link would hit chassis")


def make(
    tmp_path: Path, start: dict[str, float] | None = None, guard: Any = None
) -> tuple[ArmController, FakeArmBackend, GraspService, list[dict[str, Any]]]:
    be = FakeArmBackend(dict(start or START))
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "home.yaml"
    arm = ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg)
    sent: list[dict[str, Any]] = []
    return arm, be, GraspService(arm, cfg, guard=guard, publish=sent.append), sent


def jog(service: GraspService, kind: str, direction: str, step: float | None = None, rid: str = "j") -> Any:
    spec: dict[str, Any] = {"kind": kind, "direction": direction}
    if step is not None:
        spec["step"] = step
    return service.handle({"action": "jog", "request_id": rid, "jog": spec})


def tool(be: FakeArmBackend) -> Any:
    return KIN.forward(be.positions)


def test_jog_config_defaults_are_the_settled_caps() -> None:
    jog_cfg = McpServerConfig().jog
    assert isinstance(jog_cfg, JogSettings)
    assert jog_cfg.max_cartesian_step_m == CART_CAP
    assert jog_cfg.max_joint_step_rad == JOINT_CAP
    assert jog_cfg.max_gripper_step == 0.1
    with pytest.raises(ValueError):
        McpServerConfig.model_validate({"jog": {"max_cartesian_step_m": 0.01, "typo": 1}})


def test_jog_spec_accepts_only_the_directions_of_its_kind() -> None:
    assert JogSpec(kind="cartesian", direction="forward").step is None
    assert JogSpec(kind="gripper", direction="open", step=0.05).step == 0.05
    for kind, direction in [
        ("shoulder_pan", "up"),
        ("wrist_pitch", "left"),
        ("wrist_roll", "open"),
        ("gripper", "up"),
        ("cartesian", "close"),
    ]:
        with pytest.raises(ValueError):
            JogSpec.model_validate({"kind": kind, "direction": direction})
    with pytest.raises(ValueError):
        JogSpec.model_validate({"kind": "cartesian", "direction": "up", "speed": 1})
    with pytest.raises(ValueError):
        JogSpec(kind="cartesian", direction="up", step=-0.01)


def test_jog_request_validation(tmp_path: Path) -> None:
    arm, be, service, _ = make(tmp_path)
    assert service.handle({"action": "jog"})["ok"] is False  # 'jog' spec missing
    bad = service.handle({"action": "jog", "jog": {"kind": "cartesian", "direction": "sideways"}})
    assert bad["ok"] is False and "invalid request" in bad["error"]
    assert be.commands == []


def test_cartesian_up_raises_the_tool_point_by_the_capped_step_and_keeps_pitch(tmp_path: Path) -> None:
    arm, be, service, _ = make(tmp_path)
    before = tool(be)
    done = jog(service, "cartesian", "up", step=0.05)
    assert done["ok"] is True, done
    result = done["result"]
    assert result["step"] == pytest.approx(CART_CAP) and result["step_capped"] is True
    after = tool(be)
    axes = gripper_axes(KIN, START)
    moved = np.array([after.x - before.x, after.y - before.y, after.z - before.z])
    assert float(moved @ axes["up"]) == pytest.approx(CART_CAP, abs=0.002)
    assert float(np.linalg.norm(moved)) == pytest.approx(CART_CAP, abs=0.002)
    assert after.pitch == pytest.approx(before.pitch, abs=0.02)
    assert done["action"] == "jog" and done["request_id"] == "j"


def test_cartesian_steps_follow_the_gripper_frame(tmp_path: Path) -> None:
    axes = gripper_axes(KIN, START)
    pose = KIN.forward(START)
    approach = KIN.approach_vector(math.atan2(axes["forward"][1], axes["forward"][0]), pose.pitch)
    assert np.allclose(axes["forward"], approach, atol=1e-6)
    assert axes["left"][2] == pytest.approx(0.0) and float(axes["left"] @ axes["forward"]) == pytest.approx(0.0)
    assert float(axes["up"] @ axes["forward"]) == pytest.approx(0.0, abs=1e-9)
    assert axes["up"][2] > 0.0 and axes["left"][1] > 0.0  # at pan 0 left is +y of the arm base
    for direction, axis, sign in [("forward", "forward", 1), ("back", "forward", -1), ("left", "left", 1)]:
        arm, be, service, _ = make(tmp_path)
        before = tool(be)
        done = jog(service, "cartesian", direction)
        assert done["ok"] is True, done
        assert done["result"]["step"] == pytest.approx(CART_CAP) and done["result"]["step_capped"] is False
        after = tool(be)
        moved = np.array([after.x - before.x, after.y - before.y, after.z - before.z])
        assert float(moved @ axes[axis]) * sign == pytest.approx(CART_CAP, abs=0.002), direction


def test_joint_steps_move_one_joint_in_the_named_direction(tmp_path: Path) -> None:
    arm, be, service, _ = make(tmp_path)
    before = tool(be)
    done = jog(service, "shoulder_pan", "left", step=0.2)
    assert done["ok"] is True, done
    assert done["result"]["step"] == pytest.approx(JOINT_CAP) and done["result"]["step_capped"] is True
    assert abs(be.positions["shoulder_pan"]) == pytest.approx(JOINT_CAP, abs=1e-3)
    after = tool(be)
    assert math.atan2(after.y, after.x) > math.atan2(before.y, before.x)  # turned to the left (+y)
    others = {j: v for j, v in be.positions.items() if j != "shoulder_pan"}
    assert others == pytest.approx({j: v for j, v in START.items() if j != "shoulder_pan"}, abs=1e-6)

    arm, be, service, _ = make(tmp_path)
    before = tool(be)
    assert jog(service, "wrist_pitch", "up")["ok"] is True
    assert tool(be).pitch < before.pitch  # pitch is + down: up lowers it

    arm, be, service, _ = make(tmp_path)
    assert jog(service, "wrist_roll", "left", step=0.02)["ok"] is True
    assert be.positions["wrist_roll"] == pytest.approx(0.02, abs=1e-3)
    assert jog(service, "wrist_roll", "right", step=0.02)["ok"] is True
    assert be.positions["wrist_roll"] == pytest.approx(0.0, abs=1e-3)


def test_gripper_steps_are_open_fraction_steps(tmp_path: Path) -> None:
    arm, be, service, _ = make(tmp_path, START | {"gripper": CONFIG.arm.gripper_closed_rad})
    span = CONFIG.arm.gripper_open_rad - CONFIG.arm.gripper_closed_rad
    done = jog(service, "gripper", "open", step=0.5)
    assert done["ok"] is True, done
    assert done["result"]["step"] == pytest.approx(0.1) and done["result"]["step_capped"] is True
    assert be.positions["gripper"] == pytest.approx(CONFIG.arm.gripper_closed_rad + 0.1 * span, abs=1e-3)
    assert jog(service, "gripper", "close", step=0.1)["ok"] is True
    closed = jog(service, "gripper", "close")
    assert closed["ok"] is False and "closed" in closed["error"]


def test_jog_is_refused_without_fresh_joint_states(tmp_path: Path) -> None:
    arm, be, service, _ = make(tmp_path)
    be.no_samples = True
    for kind, direction in [("cartesian", "up"), ("shoulder_pan", "left"), ("gripper", "open")]:
        refused = jog(service, kind, direction)
        assert refused["ok"] is False and "joint_states" in refused["error"], refused
    assert be.commands == []


def test_jog_is_refused_under_the_battery_cutoff(tmp_path: Path) -> None:
    arm, be, service, _ = make(tmp_path, guard=Cutoff())
    refused = jog(service, "cartesian", "up")
    assert refused["ok"] is False and "battery" in refused["error"]
    assert be.commands == []


def test_jog_is_refused_when_the_target_is_unreachable(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    arm, be, service, _ = make(tmp_path)

    def unreachable(*_: Any, **__: Any) -> dict[str, float]:
        raise UnreachableError("unreachable: beyond the arm's reach")

    monkeypatch.setattr(arm, "solve_cartesian", unreachable)
    refused = jog(service, "cartesian", "forward")
    assert refused["ok"] is False and "unreachable" in refused["error"]
    assert be.commands == []


def test_jog_is_refused_when_the_target_collides(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    arm, be, service, _ = make(tmp_path)

    def collide(*_: Any, **__: Any) -> Any:
        raise FakeCollision()

    monkeypatch.setattr(arm, "check_body", collide)
    for kind, direction in [("cartesian", "down"), ("shoulder_pan", "right")]:
        refused = jog(service, kind, direction)
        assert refused["ok"] is False and "self_collision" in refused["error"], refused
    assert be.commands == []


def test_cartesian_jog_below_the_floor_is_refused(tmp_path: Path) -> None:
    arm, be, service, _ = make(tmp_path)
    above = tool(be).z - CONFIG.arm.floor_z_m
    assert above > CART_CAP
    # START points the gripper almost straight down: 'forward' (along the approach axis) is the descending step.
    drop = -CART_CAP * float(gripper_axes(KIN, START)["forward"][2])
    assert drop > 0.009
    # The floor clearance ends half a step below the tool point: the descending step would end under it.
    service.jogger.settings = service.jogger.settings.model_copy(update={"floor_clearance_m": above - drop / 2})
    refused = jog(service, "cartesian", "forward")
    assert refused["ok"] is False and "floor" in refused["error"]
    assert be.commands == []
    assert jog(service, "cartesian", "back")["ok"] is True  # moving away from the floor is never refused


def test_joint_step_at_its_limit_is_refused(tmp_path: Path) -> None:
    arm, _, _, _ = make(tmp_path)
    at_limit = START | {"shoulder_pan": arm.clamp_targets({"shoulder_pan": 10.0})["shoulder_pan"]}
    arm, be, service, _ = make(tmp_path, at_limit)
    left_sign = service.jogger.joint_sign("shoulder_pan", "left", at_limit)
    direction = "left" if left_sign > 0 else "right"
    refused = jog(service, "shoulder_pan", direction)
    assert refused["ok"] is False and "limit" in refused["error"], refused
    assert be.commands == []


def test_roll_jog_is_refused_with_the_gripper_open_past_the_roll_limit(tmp_path: Path) -> None:
    arm, be, service, _ = make(tmp_path, START | {"gripper": CONFIG.limits.roll_max_gripper_open_rad + 0.2})
    refused = jog(service, "wrist_roll", "left", step=0.02)
    assert refused["ok"] is False and "gripper" in refused["error"] and "roll" in refused["error"]
    assert be.commands == []


def test_jogs_coalesce_while_one_runs_and_the_last_request_wins(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    arm, be, service, sent = make(tmp_path)
    started = threading.Event()
    release = threading.Event()
    ran: list[str] = []
    real = Jogger.run

    def gated(self: Jogger, spec: JogSpec, head: dict[str, Any]) -> dict[str, Any]:
        ran.append(head["request_id"])
        if head["request_id"] == "a":
            started.set()
            assert release.wait(5.0)
        return real(self, spec, head)

    monkeypatch.setattr(Jogger, "run", gated)
    first: dict[str, Any] = {}
    worker = threading.Thread(target=lambda: first.update(r=jog(service, "cartesian", "up", rid="a")))
    worker.start()
    assert started.wait(5.0)
    assert jog(service, "cartesian", "up", rid="b") is None  # pending: answered later on /grasp/result
    assert jog(service, "cartesian", "up", rid="c") is None  # replaces b
    assert [m["request_id"] for m in sent] == ["b"]
    assert sent[0]["ok"] is False and "superseded" in sent[0]["error"]
    release.set()
    worker.join(5.0)
    assert first["r"] is None  # its answer went out through the sink because c was pending
    assert ran == ["a", "c"]
    assert [m["request_id"] for m in sent] == ["b", "a", "c"]
    assert sent[1]["ok"] is True and sent[2]["ok"] is True
    assert service.jogger.pending is None and service.jogger.running is False


def test_stop_drops_a_pending_jog(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    arm, be, service, sent = make(tmp_path)
    started = threading.Event()
    release = threading.Event()
    real = Jogger.run

    def gated(self: Jogger, spec: JogSpec, head: dict[str, Any]) -> dict[str, Any]:
        if head["request_id"] == "a":
            started.set()
            assert release.wait(5.0)
        return real(self, spec, head)

    monkeypatch.setattr(Jogger, "run", gated)
    worker = threading.Thread(target=lambda: jog(service, "cartesian", "up", rid="a"))
    worker.start()
    assert started.wait(5.0)
    assert jog(service, "cartesian", "up", rid="b") is None
    assert service.handle({"action": "stop"})["ok"] is True
    assert [m["request_id"] for m in sent] == ["b"] and "stop" in sent[0]["error"]
    release.set()
    worker.join(5.0)
    assert [m["request_id"] for m in sent] == ["b"]  # a answered through handle's return: nothing was pending


def test_jog_is_refused_while_a_grasp_action_runs(tmp_path: Path) -> None:
    arm, be, service, _ = make(tmp_path)
    assert service.busy.acquire(blocking=False)
    try:
        refused = jog(service, "cartesian", "up")
    finally:
        service.busy.release()
    assert refused["ok"] is False and "grasp action" in refused["error"]


def test_handle_json_answers_a_plain_jog_at_once(tmp_path: Path) -> None:
    arm, be, service, _ = make(tmp_path)
    answer = service.handle_json(json.dumps({"action": "jog", "jog": {"kind": "wrist_pitch", "direction": "down"}}))
    assert answer is not None and json.loads(answer)["ok"] is True


def test_jog_is_refused_while_a_human_source_commands_the_arm(tmp_path: Path) -> None:
    arm, be, service, _ = make(tmp_path)
    be.source = "leader"  # filter_node reports the leader arm (a human teleoperating) and mcp_server holds no lease
    for kind, direction in [("cartesian", "up"), ("shoulder_pan", "left"), ("gripper", "open")]:
        refused = jog(service, kind, direction)
        assert refused["ok"] is False and "leader" in refused["error"] and "take control" in refused["error"], refused
    assert be.commands == []
    be.source = "none"  # nobody commands the follower: the jog takes the lease like a web-UI grasp
    assert jog(service, "gripper", "open")["ok"] is True


def test_jog_runs_once_the_lease_is_held(tmp_path: Path) -> None:
    arm, be, service, _ = make(tmp_path)
    be.source = "leader"
    assert jog(service, "cartesian", "up")["ok"] is False
    arm.acquire()  # web UI Take control (/arm/take_control) acquires the autonomy lease
    be.source = "autonomy"
    assert jog(service, "cartesian", "up")["ok"] is True


def test_jog_is_refused_while_the_agent_holds_the_arm(tmp_path: Path) -> None:
    from mcp_server.tools import build_mcp_server

    from .test_tools import TOKEN, FakeRobot, call, take

    robot = FakeRobot(tmp_path)
    sent: list[dict[str, Any]] = []
    grasp = GraspService(robot.arm, McpServerConfig(), publish=sent.append)
    robot.grasp = grasp  # type: ignore[attr-defined]
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    take(server, "arm")
    refused = grasp.handle({"action": "jog", "request_id": "j", "jog": {"kind": "gripper", "direction": "open"}})
    assert refused is not None and refused["ok"] is False and "agent" in refused["error"], refused
    call(server, "release_control", {"subsystem": "arm"})
    done = grasp.handle({"action": "jog", "request_id": "k", "jog": {"kind": "gripper", "direction": "open"}})
    assert done is not None and done["ok"] is True, done
