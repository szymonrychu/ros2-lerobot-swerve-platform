"""Regression tests of the 2026-10-10 grip tuning session: the full grasp_object sequence on a fake follower that sags
under gravity (and, in one case, gets its shoulder_pan pushed off target while the jaws squeeze). Every arm joint must
stay commanded at its intended target through approach, close, lift and hold (sag compensation never touches
shoulder_pan), any joint off its intent by more than its converge tolerance must show up in the result's
residual_error and warnings, and the result must say where the jaw centre and the tool point (fixed jaw) are, so a
centred grasp is not mistaken for a sideways drift. Also the empty firm close of that session (stalled 0.055 rad short
of closed at load 148) must be a miss."""

from pathlib import Path
from typing import Any

import pytest

from mcp_server.arm import ArmController
from mcp_server.config import McpServerConfig, SagCompensationSettings
from mcp_server.grasp import GraspPlan, ObjectSpec, grasp_params
from mcp_server.grasp_tools import GraspExecutor, GraspResult
from mcp_server.ik import ArmKinematics, grasp_offset, load_joint_limits
from mcp_server.sag import GravityModel, SagCompensator

from .fakes import FakeArmBackend

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
FLOOR = CONFIG.arm.floor_z_m
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
# A 3 cm cube off to the side (the shift of a centred grasp is sideways to the reach, as on the robot).
OBJECT = {"frame": "arm", "x": 0.22, "y": 0.06, "support_z": FLOOR, "width_m": 0.03, "depth_m": 0.03, "height_m": 0.04}
START = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 1.2,
    "wrist_flex": 0.3,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}
OBJECT_STOP_RAD = 0.22  # jaw angle of a 3 cm gap (JawModel)
HOLD_EFFORT = 400.0
PAN_PUSH = 0.06  # the squeezed object pushes the pan off target: above its 0.03 converge, below its 0.08 settle band
# Deployed lifting gains (client.yml); the fake follower sags by the same model, so compensation lands on target.
GAINS = {"shoulder_lift": 0.141, "elbow_flex": 0.169}
SAG_GAINS = SagCompensationSettings(enabled=True, k=GAINS, k_lowering=GAINS, max_rad=0.12)
GRAVITY = SagCompensator(GravityModel(CONFIG.arm.urdf_path), GAINS, 0.12)


def make(tmp_path: Path) -> tuple[ArmController, FakeArmBackend, McpServerConfig]:
    be = FakeArmBackend(dict(START))
    be.follow = False  # object_in_jaws / sagging place the measured joints
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "home.yaml"
    cfg.arm.sag_compensation = SAG_GAINS
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg), be, cfg


def sagged(b: FakeArmBackend, pan_push: float) -> None:
    """Measured arm pose: the last command plus the gravity deflection of the pose it lands on, plus a pan push."""
    if not b.commands:
        return
    command = b.commands[-1]
    landed = dict(command)
    for _ in range(6):  # fixed point: the deflection depends on the pose the joint settles on
        d = GRAVITY.deflection({j: landed[j] for j in ARM_JOINTS}, GAINS)
        landed = command | {j: command[j] + d[j] for j in d}
    jaw = b.positions["gripper"]
    b.positions.update(landed)
    b.positions["gripper"] = jaw if "gripper" not in command else command["gripper"]
    b.positions["shoulder_pan"] += pan_push


def object_in_jaws(be: FakeArmBackend, pan_push: float = 0.0) -> None:
    """A sagging follower with an object between the jaws: the jaw cannot close past OBJECT_STOP_RAD and then feels
    load; once squeezed the object pushes shoulder_pan by pan_push (measured = commanded + pan_push from then on)."""
    pushed = {"on": False}

    def squeeze(b: FakeArmBackend) -> None:
        commanded = b.commands[-1]["gripper"] if b.commands else OBJECT_STOP_RAD
        loaded = commanded < OBJECT_STOP_RAD - 0.02
        pushed["on"] = pushed["on"] or loaded
        sagged(b, pan_push if pushed["on"] else 0.0)
        if b.positions["gripper"] < OBJECT_STOP_RAD:
            b.positions["gripper"] = OBJECT_STOP_RAD
        b.efforts["gripper"] = HOLD_EFFORT if loaded else 0.0

    be.on_sleep = squeeze


def run(
    arm: ArmController, cfg: McpServerConfig, monkeypatch: pytest.MonkeyPatch, grip: str | None = None
) -> tuple[GraspResult, GraspPlan, dict]:
    """Run grasp_object's executor; returns the result, the executed plan and the command index each step started at."""
    executor = GraspExecutor(arm, cfg)
    captured: dict[str, Any] = {}
    marks: dict[str, int] = {}
    be = arm.backend
    step = executor.step

    def spy_step(label: str, steps: list, stop: Any, motion: Any, *rest: Any, **kw: Any) -> Any:
        marks.setdefault(label, len(be.commands))
        return step(label, steps, stop, motion, *rest, **kw)

    execute = executor.execute

    def spy_execute(plan: GraspPlan, *args: Any) -> Any:
        captured["plan"] = plan
        return execute(plan, *args)

    monkeypatch.setattr(executor, "step", spy_step)
    monkeypatch.setattr(executor, "execute", spy_execute)
    params = grasp_params(cfg.grasp, None, grip)
    result = executor.grasp(ObjectSpec(**OBJECT), "top_down", params, None, None, lambda: False)
    return result, captured["plan"], marks


def pans(commands: list[dict[str, float]]) -> set[float]:
    return {round(c["shoulder_pan"], 6) for c in commands}


def test_full_grasp_keeps_pan_commanded_at_its_intent_through_close_lift_and_hold(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    arm, be, cfg = make(tmp_path)
    object_in_jaws(be)
    result, plan, marks = run(arm, cfg, monkeypatch)
    assert result.outcome == "grasped", result.reasons
    wp = {w.label: w for w in plan.waypoints}
    grasp_pan = round(wp["grasp"].joints["shoulder_pan"], 6)
    # close (and the hold check after it): the arm stays at the grasp pose, pan never compensated nor re-held measured
    assert pans(be.commands[marks["close"] : marks["lift"]]) == {grasp_pan}
    # lift and retreat stream through the planned samples: pan stays within their range (no drift beyond it)
    planned = [s["shoulder_pan"] for label in ("lift", "retreat") for s in plan.segments[label]]
    lo, hi = min(planned + [grasp_pan]) - 1e-6, max(planned + [grasp_pan]) + 1e-6
    assert all(lo <= p <= hi for p in pans(be.commands[marks["lift"] :]))
    # the hold after the grasp, also once the settle hold time passed (relax) and the keepalive republished it
    be.t += cfg.limits.arm_settle_hold_s + 0.5
    arm.keepalive_tick()
    assert be.commands[-1]["shoulder_pan"] == pytest.approx(wp["retreat"].joints["shoulder_pan"], abs=1e-9)
    for j in ARM_JOINTS:
        assert abs(be.positions[j] - wp["retreat"].joints[j]) <= cfg.limits.converge_tolerance_for(j)
    assert result.residual_error == {}
    assert result.warnings == []


def test_pan_pushed_off_its_intent_is_reported_as_residual_error_and_warning(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    arm, be, cfg = make(tmp_path)
    object_in_jaws(be, pan_push=PAN_PUSH)
    result, plan, marks = run(arm, cfg, monkeypatch)
    assert result.outcome == "grasped", result.reasons
    wp = {w.label: w for w in plan.waypoints}
    # still commanded at the intent while the object pushes it away
    assert pans(be.commands[marks["close"] : marks["lift"]]) == {round(wp["grasp"].joints["shoulder_pan"], 6)}
    close = next(s for s in result.steps if s.label == "close")
    assert close.residual_error["shoulder_pan"] == pytest.approx(-PAN_PUSH, abs=0.005)
    assert result.residual_error["shoulder_pan"] == pytest.approx(-PAN_PUSH, abs=0.005)
    assert any("shoulder_pan" in w and "close" in w for w in result.warnings), result.warnings
    assert any("shoulder_pan" in w and "retreat" in w for w in result.warnings), result.warnings


def test_result_reports_the_jaw_centre_and_the_tool_point_of_a_centred_grasp(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    arm, be, cfg = make(tmp_path)
    object_in_jaws(be)
    result, plan, _ = run(arm, cfg, monkeypatch)
    assert result.outcome == "grasped", result.reasons
    assert plan.center_width_m == pytest.approx(OBJECT["width_m"])
    shift = result.plan["grasp_shift"]
    assert shift["object_width_m"] == pytest.approx(OBJECT["width_m"])
    clearance = cfg.grasp.fixed_jaw_clearance_m
    assert shift["fixed_jaw_clearance_m"] == pytest.approx(clearance)
    assert shift["shift_m"] == pytest.approx(OBJECT["width_m"] / 2.0 + clearance)
    assert shift["jaw_open_axis"] == list(cfg.arm.jaw_open_axis)
    grasp_wp = next(w for w in result.plan["waypoints"] if w["label"] == "grasp")
    # the waypoint x, y, z is the jaw centre (object centre); tool_point is the fixed jaw, half the width plus the
    # fixed-jaw clearance beside it
    tool = grasp_wp["tool_point"]
    gap = (
        (tool["x"] - grasp_wp["x"]) ** 2 + (tool["y"] - grasp_wp["y"]) ** 2 + (tool["z"] - grasp_wp["z"]) ** 2
    ) ** 0.5
    assert gap == pytest.approx(shift["shift_m"], abs=0.002)
    wp = {w.label: w for w in plan.waypoints}
    fk = KIN.forward(wp["grasp"].joints)
    assert (tool["x"], tool["y"], tool["z"]) == pytest.approx((fk.x, fk.y, fk.z), abs=1e-3)
    # the held pose: where the jaw centre and the tool point are, measured, against the retreat waypoint
    held = result.held_pose
    assert held is not None
    centre = KIN.forward(
        {j: be.positions[j] for j in ARM_JOINTS}, grasp_offset(OBJECT["width_m"], cfg.arm.jaw_open_axis, clearance)
    )
    assert (held["jaw_centre"]["x"], held["jaw_centre"]["y"]) == pytest.approx((centre.x, centre.y), abs=1e-3)
    retreat = next(w for w in result.plan["waypoints"] if w["label"] == "retreat")
    assert held["expected_jaw_centre"] == {k: retreat[k] for k in ("x", "y", "z")}
    assert held["error_m"] < 0.01
    assert "tool_point" in held


def test_empty_firm_close_stalling_short_of_closed_is_a_miss(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """2026-10-10 yellow cube: the firm close on nothing stalled at -0.110 rad (0.055 short of closed) at load 148."""
    arm, be, cfg = make(tmp_path)
    stall = cfg.arm.gripper_closed_rad + 0.055

    def empty_stall(b: FakeArmBackend) -> None:
        sagged(b, 0.0)
        if b.positions["gripper"] < stall:
            b.positions["gripper"] = stall
        b.efforts["gripper"] = 148.0 if b.positions["gripper"] <= stall + 1e-9 else 0.0

    be.on_sleep = empty_stall
    result, _, _ = run(arm, cfg, monkeypatch, grip="firm")
    assert result.outcome == "missed", result.reasons
    close = next(s for s in result.steps if s.label == "close")
    assert close.status == "closed_no_contact", close.message
    assert result.holding_load is None
