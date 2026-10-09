"""Tests for ArmController.move_blend (one continuous trajectory through several targets) and solve_cartesian."""

import math
from pathlib import Path

import pytest

from mcp_server.arm import ArmController, ArmError
from mcp_server.config import McpServerConfig
from mcp_server.ik import ArmKinematics, UnreachableError, load_joint_limits

from .fakes import FakeArmBackend

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
RATE = CONFIG.limits.arm_rate_hz
LOW_SEED = {"shoulder_pan": 0.0, "shoulder_lift": 1.0, "elbow_flex": 0.5, "wrist_flex": 0.0, "wrist_roll": 0.0}


def make(tmp_path: Path, floor_guard: bool = False) -> tuple[ArmController, FakeArmBackend]:
    be = FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "home.yaml"
    cfg.floor_guard.enabled = floor_guard
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg, tilt_source=lambda: None), be


def pan_speeds(commands: list[dict[str, float]]) -> list[float]:
    return [(b["shoulder_pan"] - a["shoulder_pan"]) * RATE for a, b in zip(commands, commands[1:], strict=False)]


def test_blend_moves_through_every_target_without_stopping(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    vias: list[int] = []
    targets = [{"shoulder_pan": 0.3}, {"shoulder_pan": 0.6}, {"shoulder_pan": 0.9}]
    res = arm.move_blend(targets, on_via=vias.append, settle="final")
    assert res.status == "converged", res.message
    assert be.commands[-1]["shoulder_pan"] == pytest.approx(0.9)
    assert vias == [0, 1, 2]
    speeds = pan_speeds(be.commands[1:])
    moving = [s for s in speeds if s > 1e-6]
    # Between the start and the end the joint never comes to rest at the intermediate targets.
    first, last = speeds.index(moving[0]), len(speeds) - 1 - speeds[::-1].index(moving[-1])
    assert min(speeds[first + 2 : last - 2]) > 0.1


def test_blend_is_faster_than_separate_stop_and_go_moves(tmp_path: Path) -> None:
    targets = [{"shoulder_pan": 0.3}, {"shoulder_pan": 0.6}, {"shoulder_pan": 0.9}]
    arm, be = make(tmp_path)
    arm.move_blend(targets, settle="trajectory_end")
    blended = len(be.commands)
    arm2, be2 = make(tmp_path)
    for target in targets:
        arm2.move_joints(target, settle="trajectory_end")
    assert blended < len(be2.commands)


def test_blend_respects_the_velocity_cap(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.move_blend([{"elbow_flex": 0.5, "shoulder_pan": 0.2}, {"elbow_flex": 0.9, "shoulder_pan": -0.3}], 0.5)
    vmax = arm.velocity_for(0.5)
    for a, b in zip(be.commands, be.commands[1:], strict=False):
        assert max(abs(b[j] - a[j]) for j in KIN.joint_names) * RATE <= vmax * 1.01


def test_blend_unnamed_joints_keep_their_targets_across_vias(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.move_blend([{"elbow_flex": 0.4}, {"shoulder_pan": 0.2}])
    assert be.commands[-1]["elbow_flex"] == pytest.approx(0.4)
    assert be.commands[-1]["shoulder_pan"] == pytest.approx(0.2)


def test_blend_roll_guard_refuses_and_nothing_moves(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.positions["gripper"] = 1.2
    with pytest.raises(ArmError, match="wrist_roll"):
        arm.move_blend([{"shoulder_pan": 0.2}, {"wrist_roll": 1.0}])
    assert be.commands == []


def test_blend_roll_guard_counts_a_gripper_target_of_an_earlier_via(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    with pytest.raises(ArmError, match="wrist_roll"):
        arm.move_blend([{"gripper": 1.3}, {"wrist_roll": 1.0}])
    assert be.commands == []


def test_blend_slow_zone_is_applied_per_step(tmp_path: Path) -> None:
    low = KIN.inverse(0.2, 0.0, CONFIG.arm.floor_z_m + 0.005, math.pi / 2, LOW_SEED)
    above = KIN.inverse(0.2, 0.0, CONFIG.arm.floor_z_m + 0.15, math.pi / 2, LOW_SEED)
    plain, be_plain = make(tmp_path)
    assert plain.move_blend([above, low], 0.5).slow_zone is None
    arm, be = make(tmp_path, floor_guard=True)
    res = arm.move_blend([above, low], 0.5)
    assert res.slow_zone is not None and res.slow_zone["slowed_samples"] > 0
    assert len(be.commands) > len(be_plain.commands) + 5
    vmax = arm.velocity_for(0.5)
    tail = be.commands[-6:]
    for a, b in zip(tail, tail[1:], strict=False):
        assert max(abs(b[j] - a[j]) for j in KIN.joint_names) * RATE <= vmax * 0.2 * 1.05


def test_blend_stop_aborts_and_holds(tmp_path: Path) -> None:
    arm, be = make(tmp_path)

    def stop_midway(backend: FakeArmBackend) -> None:
        if len(backend.commands) == 10:
            arm.request_stop()

    be.on_sleep = stop_midway
    res = arm.move_blend([{"shoulder_pan": 0.5}, {"shoulder_pan": 1.0}])
    assert res.status == "stopped"


def test_blend_rejects_empty_and_invalid_targets(tmp_path: Path) -> None:
    arm, _ = make(tmp_path)
    with pytest.raises(ArmError):
        arm.move_blend([])
    with pytest.raises(ArmError):
        arm.move_blend([{"knee": 0.1}])


def test_motion_running_reflects_the_motion_lock(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    seen: list[bool] = []
    be.on_sleep = lambda _b: seen.append(arm.motion_running)
    assert arm.motion_running is False
    arm.move_joints({"shoulder_pan": 0.1})
    assert seen and all(seen)
    assert arm.motion_running is False


def test_solve_cartesian_matches_move_cartesian_target(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    seed = arm.command_base(arm.require_sample())
    solution = arm.solve_cartesian(0.2, 0.0, 0.0, None, seed)
    res = arm.move_cartesian(0.2, 0.0, 0.0, None, settle="final")
    assert res.status == "converged"
    for j in KIN.joint_names:
        assert be.commands[-1][j] == pytest.approx(arm.clamp_targets(solution)[j], abs=1e-6)


def test_solve_cartesian_unreachable_raises(tmp_path: Path) -> None:
    arm, _ = make(tmp_path)
    seed = arm.command_base(arm.require_sample())
    with pytest.raises(UnreachableError):
        arm.solve_cartesian(2.0, 0.0, 0.1, None, seed)
