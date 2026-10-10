"""ArmController gravity sag compensation (arm.sag_compensation): commanded = target - predicted deflection."""

import json
import logging
from pathlib import Path

import pytest
from pydantic import ValidationError

from mcp_server.arm import ArmController
from mcp_server.config import McpServerConfig, SagCompensationSettings
from mcp_server.home_store import save_home
from mcp_server.ik import ArmKinematics, load_joint_limits
from mcp_server.sag import APPROACH_LIFTING, APPROACH_LOWERING, GravityModel, SagCompensator

from .fakes import FakeArmBackend

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
GRAVITY = GravityModel(CONFIG.arm.urdf_path)
K = {"shoulder_lift": 0.1, "elbow_flex": 0.12}
TARGET = {"shoulder_lift": 0.8, "elbow_flex": -0.5}
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def make(
    tmp_path: Path,
    enabled: bool = True,
    k: dict[str, float] | None = None,
    k_lowering: dict[str, float] | None = None,
    max_rad: float = 0.12,
) -> tuple[ArmController, FakeArmBackend]:
    be = FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "arm" / "home.yaml"
    cfg.arm.sag_compensation = SagCompensationSettings(
        enabled=enabled, k=K if k is None else k, k_lowering=K if k_lowering is None else k_lowering, max_rad=max_rad
    )
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg), be


def predicted(pose: dict[str, float], gains: dict[str, float] = K, max_rad: float = 0.12) -> dict[str, float]:
    """Deflection the controller should predict at a pose (no joint offsets in KIN)."""
    return SagCompensator(GRAVITY, gains, max_rad).deflection(pose, gains)


def arm_pose(command: dict[str, float]) -> dict[str, float]:
    return {j: command[j] for j in ARM_JOINTS}


def gravity_sag(be: FakeArmBackend, gains: dict[str, float] = K, extra: dict[str, float] | None = None) -> None:
    """The fake follower settles where the model says: measured = command + deflection(target it lands on)."""
    be.follow = False

    def hook(b: FakeArmBackend) -> None:
        command = b.commands[-1]
        landed = dict(command)
        for _ in range(20):  # fixed point: the deflection depends on the pose the joint settles on
            d = predicted(arm_pose(landed), gains)
            landed = command | {j: command[j] + d[j] for j in d}
        b.positions.update({j: v + (extra or {}).get(j, 0.0) for j, v in landed.items()})

    be.on_sleep = hook


# --- disabled = today's behaviour ------------------------------------------------------------------------------


def run_scenario(arm: ArmController, be: FakeArmBackend) -> list:
    be.sag = {"shoulder_lift": 0.06}
    results = [
        arm.move_joints(TARGET, speed_scale=0.5).model_dump(),
        arm.move_joints({"wrist_flex": 0.3}, speed_scale=0.5, settle="trajectory_end").model_dump(),
    ]
    be.t += CONFIG.limits.arm_settle_hold_s + 0.1
    arm.keepalive_tick()
    return [results, list(be.commands)]


def test_disabled_compensation_is_todays_behaviour_exactly(tmp_path: Path) -> None:
    baseline_dir, disabled_dir = tmp_path / "a", tmp_path / "b"
    be0 = FakeArmBackend()
    cfg0 = CONFIG.model_copy(deep=True)
    cfg0.arm.home_file = baseline_dir / "home.yaml"
    baseline = ArmController(be0, KIN, load_joint_limits(cfg0.arm.urdf_path), cfg0)
    disabled, be1 = make(disabled_dir, enabled=False)
    assert run_scenario(baseline, be0) == run_scenario(disabled, be1)


def test_default_config_leaves_compensation_off() -> None:
    assert CONFIG.arm.sag_compensation.enabled is False


# --- compensation math -----------------------------------------------------------------------------------------


def test_published_target_is_compensated_against_gravity(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    res = arm.move_joints(TARGET, speed_scale=0.5, settle="trajectory_end")
    goal = arm_pose(be.commands[0]) | TARGET
    d = predicted(goal)
    last = be.commands[-1]
    for j in TARGET:
        assert last[j] == pytest.approx(TARGET[j] - d[j])
        assert d[j] > 0.0  # gravity turns both joints toward +: the command is placed on the other side
    for j in ("shoulder_pan", "wrist_roll", "gripper"):
        assert last[j] == be.commands[0][j]
    assert res.target == pytest.approx(goal | {"gripper": 0.0})  # the result reports the uncompensated target


def test_compensation_saturates_at_max_rad(tmp_path: Path) -> None:
    arm, be = make(tmp_path, k={"shoulder_lift": 5.0}, k_lowering={"shoulder_lift": 5.0}, max_rad=0.05)
    arm.move_joints(TARGET, speed_scale=0.5, settle="trajectory_end")
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(TARGET["shoulder_lift"] - 0.05)


def test_compensated_commands_never_leave_the_limit_band(tmp_path: Path) -> None:
    arm, be = make(tmp_path, k={"shoulder_lift": 5.0}, k_lowering={"shoulder_lift": 5.0})
    # Gravity pulls shoulder_lift toward + here, so the compensation lowers the command: put the lower limit at the
    # target to make the compensation run into the band edge.
    arm.limits["shoulder_lift"] = (0.7, 1.7)
    margin = CONFIG.limits.arm_limit_margin_rad
    arm.move_joints({"shoulder_lift": 0.7}, speed_scale=0.5, settle="trajectory_end")  # target clamped to 0.75
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(0.7 + margin)
    # On the way from 0 (outside the band) the command is never pushed below the trajectory itself.
    assert all(c["shoulder_lift"] >= -1e-9 for c in be.commands)
    assert all(
        b["shoulder_lift"] >= a["shoulder_lift"] - 1e-9 for a, b in zip(be.commands, be.commands[1:], strict=False)
    )


def test_compensation_is_applied_in_urdf_space_with_the_joint_offsets(tmp_path: Path) -> None:
    offsets = {"shoulder_lift": -0.2, "elbow_flex": 0.3}
    kin = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad, joint_offsets=offsets)
    be = FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "home.yaml"
    cfg.arm.sag_compensation = SagCompensationSettings(enabled=True, k=K, k_lowering=K)
    arm = ArmController(be, kin, load_joint_limits(cfg.arm.urdf_path), cfg)
    arm.move_joints(TARGET, speed_scale=0.5, settle="trajectory_end")
    goal = arm_pose(be.commands[0]) | TARGET
    d = SagCompensator(GRAVITY, K, 0.12, offsets).deflection(goal, K)
    for j in TARGET:
        assert be.commands[-1][j] == pytest.approx(TARGET[j] - d[j])


def test_compensation_ramps_in_without_a_step(tmp_path: Path) -> None:
    arm, be = make(tmp_path, k={"shoulder_lift": 0.2, "elbow_flex": 0.2}, k_lowering={})
    arm.acquire()
    n = len(be.commands)
    arm.move_joints(TARGET, speed_scale=0.5, settle="trajectory_end")
    steps = [max(abs(b[j] - a[j]) for j in TARGET) for a, b in zip(be.commands[n - 1 :], be.commands[n:], strict=False)]
    # 0.5 speed scale = 1 rad/s at 25 Hz: 0.04 rad per setpoint plus a share of the compensation, never a jump.
    assert max(steps) < 0.06


# --- approach modes --------------------------------------------------------------------------------------------


def test_lifting_and_lowering_approaches_use_their_own_gains(tmp_path: Path) -> None:
    arm, be = make(tmp_path, k={"shoulder_lift": 0.1}, k_lowering={"shoulder_lift": 0.02})
    arm.move_joints({"shoulder_lift": 0.8}, speed_scale=0.5, settle="trajectory_end")  # 0 -> 0.8: with gravity
    goal = arm_pose(be.commands[-1]) | {"shoulder_lift": 0.8}
    low = predicted(goal, {"shoulder_lift": 0.02})["shoulder_lift"]
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(0.8 - low)
    assert arm.sag_modes["shoulder_lift"] == APPROACH_LOWERING
    arm.move_joints({"shoulder_lift": 0.5}, speed_scale=0.5, settle="trajectory_end")  # back up: against gravity
    goal = goal | {"shoulder_lift": 0.5}
    lift = predicted(goal, {"shoulder_lift": 0.1})["shoulder_lift"]
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(0.5 - lift)
    assert arm.sag_modes["shoulder_lift"] == APPROACH_LIFTING


def test_gripper_only_motion_keeps_the_arm_compensation(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.move_joints(TARGET, speed_scale=0.5, settle="trajectory_end")
    held = {j: be.commands[-1][j] for j in TARGET}
    arm.move_joints({"gripper": 0.5}, speed_scale=0.5)
    for c in be.commands[-5:]:
        assert {j: c[j] for j in TARGET} == pytest.approx(held)


# --- convergence and hold --------------------------------------------------------------------------------------


def test_compensated_move_converges_on_the_uncompensated_target(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    gravity_sag(be)
    res = arm.move_joints(TARGET, speed_scale=0.5)
    assert res.status == "converged" and res.message == "reached target", res.message
    assert res.residual_error == {}
    for j, v in TARGET.items():
        assert res.positions[j] == pytest.approx(v, abs=1e-3)
        assert res.target[j] == v


def test_same_sag_without_compensation_settles_short(tmp_path: Path) -> None:
    arm, be = make(tmp_path, enabled=False)
    gravity_sag(be, gains={"shoulder_lift": 0.15, "elbow_flex": 0.12})
    res = arm.move_joints(TARGET, speed_scale=0.5)
    assert "settled with residual error" in res.message
    assert res.residual_error["shoulder_lift"] < 0.0


def test_keepalive_keeps_publishing_the_compensated_target(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    gravity_sag(be)
    arm.move_joints(TARGET, speed_scale=0.5)
    be.on_sleep = None
    compensated = dict(be.commands[-1])
    be.t += CONFIG.limits.arm_settle_hold_s + 0.1
    arm.keepalive_tick()
    assert be.commands[-1] == pytest.approx(compensated)


def test_relax_after_settle_time_holds_measured_plus_compensation(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    gravity_sag(be, extra={"shoulder_lift": 0.07})  # sags 0.07 rad more than predicted: a settled residual
    res = arm.move_joints(TARGET, speed_scale=0.5)
    assert res.residual_error["shoulder_lift"] == pytest.approx(-0.07, abs=1e-3)
    assert "sag compensation" in res.message
    be.on_sleep = None
    be.t += CONFIG.limits.arm_settle_hold_s + 0.1
    arm.keepalive_tick()
    measured = arm_pose(be.positions)
    d = predicted(measured)
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(measured["shoulder_lift"] - d["shoulder_lift"])
    assert be.commands[-1]["shoulder_lift"] < measured["shoulder_lift"]  # not the bare measured pose: no sag back


def test_hold_at_measured_pose_uses_the_mid_band_gain(tmp_path: Path) -> None:
    arm, be = make(tmp_path, k={"shoulder_lift": 0.1}, k_lowering={"shoulder_lift": 0.02})
    be.positions["shoulder_lift"] = 0.8
    measured = arm_pose(be.positions)
    arm.acquire()
    d = predicted(measured, {"shoulder_lift": 0.06})
    assert be.commands[-1]["shoulder_lift"] == pytest.approx(0.8 - d["shoulder_lift"])


def test_tracking_check_compares_measured_with_the_uncompensated_setpoint(tmp_path: Path) -> None:
    arm, be = make(tmp_path, k={"shoulder_lift": 1.0}, k_lowering={"shoulder_lift": 1.0}, max_rad=0.12)
    gravity_sag(be, gains={"shoulder_lift": 1.0}, extra={})
    res = arm.move_joints({"shoulder_lift": 0.8}, speed_scale=0.5)
    assert res.status == "converged" and res.tracking_error_rad == pytest.approx(0.0, abs=1e-3)


# --- other motion entry points ---------------------------------------------------------------------------------


def test_home_and_blend_moves_are_compensated(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    home = {j: 0.0 for j in ARM_JOINTS} | {"gripper": 0.0, "shoulder_lift": 0.6, "elbow_flex": -0.2}
    save_home(arm.cfg.arm.home_file, home)
    arm.home()
    d = predicted(arm_pose(home))
    published = [c for kind, c in be.events if kind == "command"]
    assert published[-1]["shoulder_lift"] == pytest.approx(0.6 - d["shoulder_lift"])
    arm.move_blend([{"shoulder_lift": 0.4}, TARGET], speed_scale=0.5, settle="trajectory_end")
    goal = arm_pose(home) | TARGET
    assert be.commands[-1]["elbow_flex"] == pytest.approx(TARGET["elbow_flex"] - predicted(goal)["elbow_flex"])


def test_cartesian_move_lands_on_the_ik_solution(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    gravity_sag(be)
    res = arm.move_cartesian(0.2, 0.0, 0.05, None, 0.5)
    assert res.status == "converged", res.message
    assert res.achieved_tool_pose["x"] == pytest.approx(0.2, abs=0.003)
    assert res.achieved_tool_pose["z"] == pytest.approx(0.05, abs=0.003)


# --- floor slow zone -------------------------------------------------------------------------------------------


class Lowering:
    """Stub compensator whose over-command drives shoulder_lift forward (down) by 0.4 rad."""

    def __init__(self, real: SagCompensator) -> None:
        self.real = real

    def deflection(self, measured: dict[str, float], gains: dict[str, float] | None = None) -> dict[str, float]:
        return {"shoulder_lift": -0.4, "elbow_flex": 0.0, "wrist_flex": 0.0}

    def __getattr__(self, name: str) -> object:
        return getattr(self.real, name)


def test_slow_zone_checks_the_over_command_too(tmp_path: Path) -> None:
    pose = {"shoulder_lift": 0.5, "elbow_flex": 0.2, "wrist_flex": 0.6}  # 63 mm above the floor, 15 mm below at +0.4
    plain, _ = make(tmp_path / "plain")
    assert plain.move_joints(pose, speed_scale=0.5, settle="trajectory_end").slow_zone is None
    arm, _ = make(tmp_path / "stub")
    assert arm.sag is not None
    arm.sag = Lowering(arm.sag)
    res = arm.move_joints(pose, speed_scale=0.5, settle="trajectory_end")
    assert res.slow_zone is not None and res.slow_zone["slowed_samples"] > 0


# --- structured settle records ---------------------------------------------------------------------------------


def settle_records(caplog: pytest.LogCaptureFixture) -> list[dict]:
    marker = "arm_settle "
    return [json.loads(r.getMessage()[len(marker) :]) for r in caplog.records if r.getMessage().startswith(marker)]


def test_final_move_logs_target_commanded_and_settled_pose(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    caplog.set_level(logging.INFO, logger="mcp_server.sag")
    arm, be = make(tmp_path)
    gravity_sag(be, extra={"shoulder_lift": 0.07})
    arm.move_joints(TARGET, speed_scale=0.5)
    (record,) = settle_records(caplog)
    assert record["status"] == "converged" and record["settled"] is True
    assert record["target"]["shoulder_lift"] == TARGET["shoulder_lift"]
    assert record["commanded"]["shoulder_lift"] < TARGET["shoulder_lift"]
    assert record["measured"]["shoulder_lift"] == pytest.approx(be.positions["shoulder_lift"], abs=1e-4)
    assert record["residual"]["shoulder_lift"] == pytest.approx(-0.07, abs=1e-3)
    assert record["modes"]["shoulder_lift"] == APPROACH_LOWERING
    assert record["compensation_enabled"] is True


def test_trajectory_end_move_logs_once_the_arm_had_time_to_settle(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.INFO, logger="mcp_server.sag")
    arm, be = make(tmp_path, enabled=False)
    be.sag = {"shoulder_lift": 0.06}
    arm.move_joints(TARGET, speed_scale=0.5, settle="trajectory_end")
    arm.keepalive_tick()
    assert settle_records(caplog) == []  # not settled yet
    be.t += 1.1
    arm.keepalive_tick()
    (record,) = settle_records(caplog)
    assert record["residual"]["shoulder_lift"] == pytest.approx(-0.06, abs=1e-4)
    assert record["commanded"] == record["target"]  # disabled: the target itself was commanded
    arm.keepalive_tick()
    assert len(settle_records(caplog)) == 1


def test_gripper_only_motion_logs_no_settle_record(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    caplog.set_level(logging.INFO, logger="mcp_server.sag")
    arm, _ = make(tmp_path)
    arm.move_joints({"gripper": 0.5}, speed_scale=0.5)
    assert settle_records(caplog) == []


# --- config ----------------------------------------------------------------------------------------------------


def test_config_rejects_unknown_joints_and_negative_gains() -> None:
    with pytest.raises(ValidationError):
        SagCompensationSettings(k={"shoulder_pan": 0.1})
    with pytest.raises(ValidationError):
        SagCompensationSettings(k_lowering={"elbow_flex": -0.1})
    with pytest.raises(ValidationError):
        SagCompensationSettings(max_rad=0.5)
    assert SagCompensationSettings().max_rad == pytest.approx(0.12)
