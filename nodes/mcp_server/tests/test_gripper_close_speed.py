"""Gripper-only motions near the floor and the robot body: only moving parts slow them (G3, the 10:27 close).

The 2026-10-11 10:27 close: the arm reached down to an object, the moving jaw tip 12.5 mm above the floor and the
STATIONARY upper arm 14.9 mm from the lidar dome. The floor slow zone (0.2) and the body soft margin (0.3) slowed every
one of the 65 close samples to 5 ticks (325 ticks, 13 s nominal, 35-40 s on the throttled robot).
"""

from pathlib import Path
from typing import Any

import numpy as np
import pytest

from mcp_server.arm import ArmController
from mcp_server.config import BodyCheckSettings, McpServerConfig
from mcp_server.floor_guard import GRIPPER_ONLY_SLOW_MARGIN_M, FloorGuard, FloorOverride, JawModel
from mcp_server.ik import ArmKinematics, load_joint_limits
from mcp_server.models import ArmMotionResult
from mcp_server.motion_queue import arm_result_data
from mcp_server.self_collision import GRIPPER_MOVING_PARTS, SelfCollisionChecker
from mcp_server.trajectory import plan_trajectory

from .fakes import FakeArmBackend
from .test_self_collision import deployed_config

# Measured pose (P) and commanded intent (T) of the 10:27 close (scratchpad gq_ref/close1027.py).
POSE_1027 = {
    "shoulder_pan": -0.6013204688511713,
    "shoulder_lift": 1.1397477253990314,
    "elbow_flex": -0.45559229400203544,
    "wrist_flex": 0.923456434307156,
    "wrist_roll": -0.4663301595172349,
    "gripper": 0.5062136600022615,
}
INTENT_1027 = {
    "shoulder_pan": -0.5878650616430877,
    "shoulder_lift": 1.1401632655959149,
    "elbow_flex": -0.44734933703915125,
    "wrist_flex": 0.9198963361583219,
    "wrist_roll": -0.45,
    "gripper": 0.5128428158120791,
}
MAX_TICK_RATIO = 1.2
EARLY_TICKS = 25
EARLY_TRAVEL_RAD = 0.05


def deployed_arm(tmp_path: Path, positions: dict[str, float]) -> tuple[ArmController, FakeArmBackend, McpServerConfig]:
    cfg = deployed_config()
    cfg.arm.home_file = tmp_path / "home.yaml"
    kin = ArmKinematics(
        cfg.arm.urdf_path,
        margin=cfg.limits.arm_limit_margin_rad,
        joint_offsets=cfg.arm.joint_offsets_rad.model_dump(),
        tool_offset=tuple(cfg.arm.tool_offset_m.model_dump().values()),
        limit_overrides=cfg.arm.joint_limit_overrides_rad,
    )
    be = FakeArmBackend(dict(positions))
    return ArmController(be, kin, load_joint_limits(cfg.arm.urdf_path), cfg), be, cfg


def close_1027(tmp_path: Path) -> tuple[ArmController, FakeArmBackend, McpServerConfig]:
    arm, be, cfg = deployed_arm(tmp_path, POSE_1027)
    be.follow = False  # the jaw does not move (worst case): only what is commanded is checked
    arm.acquire()
    arm.command(INTENT_1027)
    return arm, be, cfg


def test_the_1027_close_is_not_slowed_by_stationary_links(tmp_path: Path) -> None:
    arm, be, cfg = close_1027(tmp_path)
    profile = cfg.grip_profiles.presets["normal"]
    start = arm.command_base(be.joint_sample())
    goal = start | {"gripper": cfg.arm.gripper_closed_rad}
    unscaled = len(plan_trajectory(start, goal, profile.close_speed_rps, cfg.limits.arm_rate_hz))
    assert 60 <= unscaled <= 70  # 65 samples on the deployed config
    planned = arm.guarded_trajectory(start, goal, profile.close_speed_rps, None)
    assert len(planned) <= MAX_TICK_RATIO * unscaled
    n0 = len(be.commands)
    result = arm.set_gripper(close_until_effort=True, grip_profile="normal")
    assert result.slow_zone is None
    assert result.self_collision is None
    closing = [c["gripper"] for c in be.commands[n0:]]
    assert POSE_1027["gripper"] - closing[EARLY_TICKS - 1] >= EARLY_TRAVEL_RAD


def test_an_arm_motion_near_the_floor_from_the_1027_pose_is_still_slowed(tmp_path: Path) -> None:
    arm, be, _cfg = close_1027(tmp_path)
    start = arm.command_base(be.joint_sample())
    lower = start | {"shoulder_lift": start["shoulder_lift"] + 0.1}
    planned = arm.guarded_trajectory(start, lower, 0.5, None)
    assert arm._slow_zone is not None
    assert len(planned) > len(plan_trajectory(start, lower, 0.5, arm.cfg.limits.arm_rate_hz))


# --- floor guard: gripper-only slow zone ------------------------------------------------------------------------


@pytest.fixture
def floor_setup(tmp_path: Path) -> tuple[FloorGuard, list[dict[str, float]], list[float]]:
    """A jaw swinging from open to closed at the 10:27 pose, with the moving jaw tip clearance (flat floor) per sample."""
    arm, be, cfg = close_1027(tmp_path)
    start = arm.command_base(be.joint_sample())
    goal = start | {"gripper": cfg.arm.gripper_closed_rad}
    samples = [start | {"gripper": cfg.arm.gripper_open_rad}, *plan_trajectory(start, goal, 0.5, 25.0)]
    guard = arm.floor_guard
    surface = guard.surface(None, None, 0.0)
    jaw = [surface.clearance(guard.checked_points(s)["moving_jaw_tip"]) for s in samples]
    return guard, samples, jaw


def test_gripper_only_motion_above_the_gripper_margin_is_not_slowed(
    floor_setup: tuple[FloorGuard, list[dict[str, float]], list[float]],
) -> None:
    guard, samples, jaw = floor_setup
    assert min(jaw) < guard.settings.margin_m  # the arm-motion slow zone would slow it
    assert min(jaw) > GRIPPER_ONLY_SLOW_MARGIN_M
    report = guard.evaluate(samples, guard.surface(None, None, 0.0), gripper_only=True)
    assert set(report.scales) == {1.0}
    assert report.summary() is None
    plain = guard.evaluate(samples, guard.surface(None, None, 0.0))
    assert min(plain.scales) < 1.0


def test_gripper_only_motion_slows_only_where_the_jaw_descends_below_the_gripper_margin(
    floor_setup: tuple[FloorGuard, list[dict[str, float]], list[float]],
) -> None:
    guard, samples, jaw = floor_setup
    # Raise the floor so the lowest moving-jaw sample sits 2 mm above it.
    lift = min(jaw) - 0.002
    surface = guard.surface(FloorOverride(surface_z_m=lift), None, 0.0)
    shifted = np.array(jaw) - lift
    report = guard.evaluate(samples, surface, gripper_only=True)
    descending = [
        i > 0 and shifted[i] < shifted[i - 1] and shifted[i] < GRIPPER_ONLY_SLOW_MARGIN_M for i in range(len(samples))
    ]
    assert any(descending)
    assert [s < 1.0 for s in report.scales] == descending
    rising = [i for i in range(1, len(samples)) if shifted[i] >= shifted[i - 1]]
    assert all(report.scales[i] == 1.0 for i in rising)


def test_gripper_only_motion_stationary_points_below_the_margin_do_not_slow(
    floor_setup: tuple[FloorGuard, list[dict[str, float]], list[float]],
) -> None:
    guard, samples, _jaw = floor_setup
    still = [samples[0]] * 5  # nothing moves: whatever is close to the floor stays where it is
    surface = guard.surface(FloorOverride(surface_z_m=0.05), None, 0.0)  # every point below the surface
    assert set(guard.evaluate(still, surface, gripper_only=True).scales) == {1.0}
    assert min(guard.evaluate(still, surface).scales) < 1.0


# --- body check: only moving parts in the soft band --------------------------------------------------------------


def body_checker(cfg: McpServerConfig, settings: BodyCheckSettings | None = None) -> SelfCollisionChecker:
    kin = ArmKinematics(
        cfg.arm.urdf_path,
        margin=cfg.limits.arm_limit_margin_rad,
        joint_offsets=cfg.arm.joint_offsets_rad.model_dump(),
        tool_offset=tuple(cfg.arm.tool_offset_m.model_dump().values()),
        limit_overrides=cfg.arm.joint_limit_overrides_rad,
    )
    if settings is not None:
        cfg = cfg.model_copy(update={"body_check": settings})
    return SelfCollisionChecker.from_config(kin, cfg, JawModel(kin, cfg.arm.jaw_open_axis, cfg.arm.gripper_closed_rad))


def jaw_swing(cfg: McpServerConfig) -> list[dict[str, float]]:
    return [INTENT_1027 | {"gripper": g} for g in np.linspace(INTENT_1027["gripper"], cfg.arm.gripper_closed_rad, 20)]


def test_body_soft_band_of_a_gripper_motion_ignores_stationary_links() -> None:
    cfg = deployed_config()
    checker = body_checker(cfg)
    samples = jaw_swing(cfg)
    plain = checker.path(samples)
    assert min(plain.scales) < 1.0  # the upper arm sits inside the soft margin of the lidar
    report = checker.path(samples, moving_parts=GRIPPER_MOVING_PARTS)
    assert set(report.scales) == {1.0}
    assert report.summary() is None
    assert not report.rejected


def test_body_soft_band_of_a_gripper_motion_still_slows_the_moving_jaw() -> None:
    cfg = deployed_config()
    wide = cfg.body_check.model_copy(update={"soft_margin_m": 1.0})  # every part is inside a 1 m soft band
    report = body_checker(cfg, wide).path(jaw_swing(cfg), moving_parts=GRIPPER_MOVING_PARTS)
    assert set(report.scales) == {wide.slow_speed_scale}
    assert set(report.parts) == set(GRIPPER_MOVING_PARTS)


def test_hard_reject_still_applies_to_a_gripper_motion() -> None:
    cfg = deployed_config()
    tight = cfg.body_check.model_copy(update={"hard_margin_m": 0.5, "soft_margin_m": 0.6})
    report = body_checker(cfg, tight).path(jaw_swing(cfg), allow_escape=False, moving_parts=GRIPPER_MOVING_PARTS)
    assert report.rejected


def test_moving_parts_must_be_single_points() -> None:
    cfg = deployed_config()
    with pytest.raises(ValueError, match="moving_parts"):
        body_checker(cfg).path(jaw_swing(cfg), moving_parts=("upper arm",))


# --- motion timing in the result ---------------------------------------------------------------------------------


def test_motion_result_reports_time_to_first_setpoint_and_tick_rate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    arm, be, _cfg = deployed_arm(tmp_path, POSE_1027)
    plan = arm.guarded_trajectory

    def slow_plan(*args: Any, **kwargs: Any) -> list[dict[str, float]]:
        be.t += 0.25  # planning takes wall time on the robot
        return plan(*args, **kwargs)

    monkeypatch.setattr(arm, "guarded_trajectory", slow_plan)
    arm.acquire()
    n0 = len(be.commands)
    res = arm.set_gripper(open_fraction=0.4)
    assert res.time_to_first_setpoint_s == pytest.approx(0.25)
    assert res.ticks is not None and res.ticks <= len(be.commands) - n0
    assert res.trajectory_s is not None
    assert res.tick_rate_hz == pytest.approx(res.ticks / res.trajectory_s, rel=0.01)


def test_queued_motion_events_carry_the_timing() -> None:
    result = ArmMotionResult(status="converged", time_to_first_setpoint_s=0.12, ticks=50, tick_rate_hz=19.5)
    data = arm_result_data(result)
    assert data["time_to_first_setpoint_s"] == 0.12
    assert data["tick_rate_hz"] == 19.5
