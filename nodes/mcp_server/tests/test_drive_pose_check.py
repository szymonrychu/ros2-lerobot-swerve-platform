"""Tests for mcp_server.drive_pose_check: the FK checks of a drive pose and of every leg the arm takes to and from it."""

import math
from pathlib import Path

import numpy as np
import pytest

from mcp_server import drive_pose_check
from mcp_server.config import REPO_ROOT, McpServerConfig
from mcp_server.drive_pose_check import (
    CHECKS,
    DrivePoseChecker,
    DrivePoseViolation,
    check_drive_pose,
    deployed_config,
    in_frustum,
)

CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
# scripts/gen_nav_footprint.yaml parked_arm_joints (the stored home of the robot).
HOME = {
    "shoulder_pan": -0.003,
    "shoulder_lift": -1.35,
    "elbow_flex": 1.578,
    "wrist_flex": -0.1,
    "wrist_roll": -1.637,
    "gripper": 0.1,
}
HALF_H = math.radians(87.0 / 2.0)
HALF_V = math.radians(58.0 / 2.0)


@pytest.fixture(scope="module")
def config() -> McpServerConfig:
    return deployed_config(CLIENT_VARS)


@pytest.fixture(scope="module")
def checker(config: McpServerConfig) -> DrivePoseChecker:
    return DrivePoseChecker(config)


def failed(violations: list[DrivePoseViolation]) -> set[str]:
    return {v.check for v in violations}


def test_checks_are_the_six_named_ones() -> None:
    assert CHECKS == ("body", "floor", "stretched", "roll", "footprint", "view")


def test_frustum_contains_points_ahead_and_not_behind_or_beside() -> None:
    points = np.array(
        [
            [0.0, 0.0, 0.5],  # on the optical axis
            [0.0, 0.0, -0.5],  # behind the camera
            [0.5 * math.tan(HALF_H) + 0.05, 0.0, 0.5],  # 5 cm right of the right edge
            [0.5 * math.tan(HALF_H) - 0.01, 0.0, 0.5],  # 1 cm inside the right edge
            [0.0, 0.5 * math.tan(HALF_V) + 0.05, 0.5],  # 5 cm below the bottom edge
        ]
    )
    assert in_frustum(points, HALF_H, HALF_V, 0.0).tolist() == [True, False, False, True, False]


def test_frustum_grows_by_the_capsule_radius() -> None:
    beside = np.array([[0.5 * math.tan(HALF_H) + 0.01 / math.cos(HALF_H), 0.0, 0.5]])  # 1 cm off the right plane
    assert not in_frustum(beside, HALF_H, HALF_V, 0.005)[0]
    assert in_frustum(beside, HALF_H, HALF_V, 0.02)[0]


def test_the_parked_home_is_in_view_when_the_head_pans_right(checker: DrivePoseChecker) -> None:
    """Home itself passes every check but the view: its arm is in the D435 frustum while the head pans right."""
    assert failed(checker.check(HOME, HOME)) == {"view"}


def test_a_pose_through_the_crossbar_fails_body(checker: DrivePoseChecker) -> None:
    assert "body" in failed(checker.check(HOME, HOME | {"elbow_flex": 1.69, "wrist_flex": 1.6}))


def test_a_pose_with_the_tool_near_the_floor_fails_floor(checker: DrivePoseChecker) -> None:
    assert "floor" in failed(checker.check(HOME, HOME | {"shoulder_lift": 0.0}))


def test_a_stretched_elbow_fails_stretched(checker: DrivePoseChecker) -> None:
    assert "stretched" in failed(checker.check(HOME, HOME | {"shoulder_lift": 1.9, "elbow_flex": -0.3}))


@pytest.mark.parametrize("joint, value", [("wrist_roll", -1.0), ("gripper", 0.5)])
def test_a_changed_wrist_roll_or_gripper_fails_roll(checker: DrivePoseChecker, joint: str, value: float) -> None:
    assert "roll" in failed(checker.check(HOME, HOME | {joint: value}))


def test_a_shadow_outside_the_static_outline_fails_footprint(checker: DrivePoseChecker) -> None:
    """The arm reaching forward and up: its ground shadow leaves the static footprint."""
    assert "footprint" in failed(
        checker.check(HOME, HOME | {"shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0})
    )


def test_every_leg_of_the_route_is_checked_not_only_the_end_pose(
    checker: DrivePoseChecker, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A route whose end is home but whose intermediate leg takes the tool to the floor fails floor."""
    low = HOME | {"shoulder_lift": -0.3, "elbow_flex": 1.4}  # tool 9.3 cm above the floor, body clear
    monkeypatch.setattr(drive_pose_check, "home_routes", lambda kin, start, goal: [("via the floor", [low, goal])])
    violations = checker.check(HOME, HOME)
    assert "floor" in failed(violations)
    assert any("via the floor" in v.detail for v in violations)


def test_both_directions_are_checked(checker: DrivePoseChecker, monkeypatch: pytest.MonkeyPatch) -> None:
    """The lift (home -> pose) and the lower (pose -> home) each get their own route check."""
    calls: list[tuple[float, float]] = []
    real = drive_pose_check.home_routes

    def spy(kin: object, start: dict[str, float], goal: dict[str, float]) -> list:
        calls.append((start["shoulder_lift"], goal["shoulder_lift"]))
        return real(kin, start, goal)

    monkeypatch.setattr(drive_pose_check, "home_routes", spy)
    pose = HOME | {"shoulder_lift": -1.6}
    checker.check(HOME, pose)
    assert (HOME["shoulder_lift"], -1.6) in calls and (-1.6, HOME["shoulder_lift"]) in calls


def test_check_drive_pose_builds_a_checker(config: McpServerConfig) -> None:
    assert failed(check_drive_pose(config, HOME, HOME)) == {"view"}


def test_the_checker_needs_the_nav_footprint(config: McpServerConfig) -> None:
    with pytest.raises(ValueError, match="nav_footprint"):
        DrivePoseChecker(config.model_copy(update={"nav_footprint": None}))


def test_deployed_config_uses_this_checkout_urdfs(config: McpServerConfig) -> None:
    assert config.arm.urdf_path == McpServerConfig().arm.urdf_path
    assert Path(config.head.urdf_path).is_file()


def test_search_ranks_passing_candidates_by_shoulder_gravity_torque(
    checker: DrivePoseChecker, monkeypatch: pytest.MonkeyPatch
) -> None:
    """search keeps only candidates with no violation and orders them by |shoulder_lift gravity torque|."""
    monkeypatch.setattr(
        checker, "check", lambda home, pose: [] if pose["wrist_flex"] >= 0.0 else [DrivePoseViolation("view", "x")]
    )
    grid = {"shoulder_lift": [-1.35, 0.0], "elbow_flex": [1.578], "wrist_flex": [-0.1, 0.5]}
    ranked = drive_pose_check.search(checker, HOME, grid)
    assert [c.joints["wrist_flex"] for c in ranked] == [0.5, 0.5]
    torques = [abs(c.shoulder_lift_torque_nm) for c in ranked]
    assert torques == sorted(torques)
    assert all(c.joints["wrist_roll"] == HOME["wrist_roll"] and c.joints["gripper"] == HOME["gripper"] for c in ranked)
