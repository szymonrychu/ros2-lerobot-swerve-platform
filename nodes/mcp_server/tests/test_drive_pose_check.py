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
    floor_region,
    in_frustum,
    rays_blocked,
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


def test_rays_blocked_by_a_capsule_across_them_and_not_by_one_beside() -> None:
    origin = np.array([0.0, 0.0, 1.0])
    targets = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    across = (np.array([[-0.5, 0.0, 0.5]]), np.array([[0.5, 0.0, 0.5]]), np.array([0.02]))
    assert rays_blocked(origin, targets, *across).tolist() == [True, True]
    beside = (np.array([[-0.5, 0.1, 0.5]]), np.array([[0.5, 0.1, 0.5]]), np.array([0.02]))
    assert rays_blocked(origin, targets, *beside).tolist() == [False, False]
    touching = (np.array([[-0.5, 0.015, 0.5]]), np.array([[0.5, 0.015, 0.5]]), np.array([0.02]))
    assert rays_blocked(origin, targets[:1], *touching).tolist() == [True]


def test_floor_region_is_ahead_of_and_beside_the_right_wheels_within_the_lookahead() -> None:
    """head_aim.yaml footprint box (-0.215..0.3225, -0.196..0.196) and lookahead 0.4 m."""
    points = floor_region(-0.215, 0.3225, -0.196, 0.196, 0.4, 0.02)
    x, y = points[:, 0], points[:, 1]
    assert np.all(points[:, 2] == 0.0)
    inside_box = (x > -0.215) & (x < 0.3225) & (y > -0.196) & (y < 0.196)
    assert not inside_box.any()
    assert np.all(y <= 0.0) and np.all(y >= -0.596) and np.all(x <= 0.7225) and np.all(x >= -0.215)
    ahead = x >= 0.3225
    beside = y <= -0.196
    assert ahead.any() and beside.any() and np.all(ahead | beside)


def test_the_parked_home_occludes_the_floor_beside_the_right_wheels(checker: DrivePoseChecker) -> None:
    """Home passes every check but the view: its arm hides 1.60 m2 of the region summed over pans 0..-90 and
    1.12 m2 over pans -50..-90 (the guarded ones)."""
    assert failed(checker.check(HOME, HOME)) == {"view"}
    assert checker.region_occlusion_m2(HOME) == pytest.approx(1.60, abs=0.01)
    assert checker.guarded_occlusion_m2(HOME) == pytest.approx(1.12, abs=0.01)


@pytest.mark.parametrize(
    "home_m2, pose_m2, passes",
    [
        (1.12, 0.165, True),  # the pinned drive pose (home 1.12 m2 over pans -50..-90)
        (1.12, 0.0, True),
        (1.5, 0.181, False),  # above VIEW_MAX_OCCLUDED_M2
        (1.0, 0.16, False),  # above 15 % of home
        (1.0, 0.15, True),
    ],
)
def test_view_passes_at_most_0_18_m2_and_15_percent_of_home_over_pans_minus_50_to_minus_90(
    checker: DrivePoseChecker, monkeypatch: pytest.MonkeyPatch, home_m2: float, pose_m2: float, passes: bool
) -> None:
    """The view rule is a regression guard: no pose leaves the region fully visible (the upper arm next to the
    shoulder axis hides some floor at pans -50..-90), so the hidden area is bounded instead of zero."""
    monkeypatch.setattr(
        checker,
        "occluded_per_pan",
        lambda pose, floor, cell: [(-40.0, 5.0), (-90.0, home_m2 if pose is HOME else pose_m2)],  # -40 not guarded
    )
    pose = dict(HOME)
    violations = checker.pose_violations(HOME, pose)
    assert ("view" not in failed(violations)) is passes


def test_the_fixed_shoulder_root_is_ignored(checker: DrivePoseChecker) -> None:
    """The occluding upper arm starts one capsule radius from the shoulder_lift axis (the root sphere that only
    shoulder_pan moves is not counted)."""
    starts, _ends, radii = checker.occluders(HOME)
    shoulder = checker.clearance.part_points(HOME)["upper arm"][0]
    assert np.linalg.norm(starts[0] - shoulder) == pytest.approx(radii[0])
    assert len(starts) == 4  # upper arm, forearm, wrist link, wrist end -> tool point


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


def test_search_ranks_passing_candidates_by_occluded_floor_then_torque(
    checker: DrivePoseChecker, monkeypatch: pytest.MonkeyPatch
) -> None:
    """search keeps only candidates with no violation, least occluded floor first, then |shoulder_lift torque|."""
    monkeypatch.setattr(
        checker, "check", lambda home, pose: [] if pose["wrist_flex"] >= 0.0 else [DrivePoseViolation("view", "x")]
    )
    monkeypatch.setattr(checker, "pose_violations", lambda home, pose: [])
    monkeypatch.setattr(checker, "floor_occlusion_m2", lambda pose: 0.1 if pose["shoulder_lift"] == 0.0 else 0.2)
    grid = {"shoulder_lift": [-1.35, 0.0], "elbow_flex": [1.578], "wrist_flex": [-0.1, 0.5]}
    ranked = drive_pose_check.search(checker, HOME, grid)
    assert [c.joints["wrist_flex"] for c in ranked] == [0.5, 0.5]
    assert [(c.joints["shoulder_lift"], c.occluded_floor_m2) for c in ranked] == [(0.0, 0.1), (-1.35, 0.2)]
    assert all(c.joints["wrist_roll"] == HOME["wrist_roll"] and c.joints["gripper"] == HOME["gripper"] for c in ranked)
