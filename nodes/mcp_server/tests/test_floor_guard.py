"""Tests for mcp_server.floor_guard: mount conversions, effective surface (robot plane, IMU tilt, overrides), the
per-sample slow zone on the real URDF, the jaw model and the time scaling of trajectory segments."""

import logging
import math

import numpy as np
import pytest

from mcp_server.config import ArmBaseOffset, FloorGuardSettings, McpServerConfig
from mcp_server.floor_guard import (
    FloorGuard,
    FloorOverride,
    JawModel,
    Tilt,
    TiltOverrideDeg,
    TiltSample,
    arm_to_base_link,
    base_link_to_arm,
    effective_surface_z,
    monitor_tilt_sample,
    retime,
    step_scales,
    tilt_from_quaternion,
)
from mcp_server.ik import ArmKinematics

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
MOUNT = ArmBaseOffset(x=0.15, y=-0.04, z=0.15, yaw=0.0)
SEED = {"shoulder_pan": 0.0, "shoulder_lift": 1.0, "elbow_flex": 0.5, "wrist_flex": 0.0, "wrist_roll": 0.0}
NOW = 50.0


def guard(**settings: object) -> FloorGuard:
    return FloorGuard(
        KIN,
        FloorGuardSettings(**settings),
        MOUNT,
        JawModel(KIN, CONFIG.arm.jaw_open_axis, CONFIG.arm.gripper_closed_rad),
        gripper_joint="gripper",
    )


def pose_at(x: float, z: float, pitch: float = math.pi / 2) -> dict[str, float]:
    """Joint configuration (with an almost closed gripper) putting the tool point at (x, 0, z) in the arm frame."""
    return KIN.inverse(x, 0.0, z, pitch, SEED) | {"gripper": 0.0}


def test_mount_conversions_round_trip_with_yaw() -> None:
    assert arm_to_base_link((0.0, 0.0, 0.0), MOUNT) == pytest.approx([0.15, -0.04, 0.15])
    assert base_link_to_arm((0.15, -0.04, 0.0), MOUNT) == pytest.approx([0.0, 0.0, -0.15])
    yawed = ArmBaseOffset(x=0.1, y=0.2, z=0.15, yaw=math.pi / 2)
    assert arm_to_base_link((1.0, 0.0, 0.0), yawed) == pytest.approx([0.1, 1.2, 0.15])
    point = np.array([0.3, -0.1, 0.05])
    assert base_link_to_arm(arm_to_base_link(point, yawed), yawed) == pytest.approx(point)


def test_tilt_from_quaternion_matches_roll_and_pitch() -> None:
    pitch = 0.2
    tilt = tilt_from_quaternion(0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2))
    assert tilt.pitch_rad == pytest.approx(pitch) and tilt.roll_rad == pytest.approx(0.0)
    roll = -0.1
    tilt = tilt_from_quaternion(math.sin(roll / 2), 0.0, 0.0, math.cos(roll / 2))
    assert tilt.roll_rad == pytest.approx(roll) and tilt.pitch_rad == pytest.approx(0.0)


def test_effective_surface_is_the_higher_of_robot_plane_and_level_plane() -> None:
    assert effective_surface_z((0.5, 0.3), 0.0, None) == 0.0
    nose_down = Tilt(roll_rad=0.0, pitch_rad=0.2)
    assert effective_surface_z((0.5, 0.0), 0.0, nose_down) == pytest.approx(0.5 * math.tan(0.2))
    assert effective_surface_z((-0.5, 0.0), 0.0, nose_down) == 0.0  # behind: the robot plane is higher
    left_up = Tilt(roll_rad=0.2, pitch_rad=0.0)  # roll > 0: left side up, the right side is below the level plane
    assert effective_surface_z((0.0, -0.5), 0.0, left_up) == pytest.approx(0.5 * math.tan(0.2))
    assert effective_surface_z((0.5, 0.0), -0.18, nose_down) == pytest.approx(-0.18 + 0.5 * math.tan(0.2))


def test_surface_uses_fresh_imu_and_ignores_stale_imu_logging_once(caplog: pytest.LogCaptureFixture) -> None:
    g = guard()
    fresh = TiltSample(tilt=Tilt(roll_rad=0.0, pitch_rad=0.1), stamp=NOW - 0.5)
    surface = g.surface(None, fresh, NOW)
    assert surface.tilt_source == "imu" and surface.tilt == fresh.tilt
    stale = TiltSample(tilt=Tilt(roll_rad=0.0, pitch_rad=0.1), stamp=NOW - 1.5)
    with caplog.at_level(logging.WARNING, logger="mcp_server.floor_guard"):
        assert g.surface(None, stale, NOW).tilt_source == "none"
        assert g.surface(None, None, NOW).tilt is None
    assert sum("IMU" in r.getMessage() for r in caplog.records) == 1
    assert g.surface(None, fresh, NOW).tilt_source == "imu"  # recovers


def test_tilt_override_replaces_the_imu() -> None:
    g = guard()
    imu = TiltSample(tilt=Tilt(roll_rad=0.0, pitch_rad=0.3), stamp=NOW)
    surface = g.surface(FloorOverride(tilt_override_deg=TiltOverrideDeg(roll=0.0, pitch=0.0)), imu, NOW)
    assert surface.tilt_source == "override" and surface.tilt == Tilt(roll_rad=0.0, pitch_rad=0.0)
    surface = g.surface(FloorOverride(tilt_override_deg=TiltOverrideDeg(roll=10.0, pitch=-5.0)), None, NOW)
    assert surface.tilt == Tilt(roll_rad=pytest.approx(math.radians(10.0)), pitch_rad=pytest.approx(math.radians(-5)))


def test_flat_robot_slows_only_samples_below_the_floor_margin() -> None:
    g = guard()
    surface = g.surface(None, None, NOW)
    high = pose_at(0.2, -0.05)  # tool 10 cm above the floor
    low = pose_at(0.2, CONFIG.arm.floor_z_m + 0.01)  # tool 1 cm above the floor: inside the 2 cm margin
    report = g.evaluate([high, low], surface)
    assert report.scales == [1.0, 0.2]
    assert report.clearances[0] > 0.02 > report.clearances[1]
    assert report.lowest_points[1] in {"tool_point", "jaw_tip", "moving_jaw_tip"}


def test_tilted_robot_moves_the_slow_zone_up_in_front() -> None:
    g = guard()
    pose = pose_at(0.25, -0.10)  # 5 cm above the robot plane, 40 cm in front of base_link
    assert g.evaluate([pose], g.surface(None, None, NOW)).scales == [1.0]
    nose_down = TiltSample(tilt=Tilt(roll_rad=0.0, pitch_rad=0.3), stamp=NOW)
    assert g.evaluate([pose], g.surface(None, nose_down, NOW)).scales == [0.2]
    nose_up = TiltSample(tilt=Tilt(roll_rad=0.0, pitch_rad=-0.3), stamp=NOW)
    assert g.evaluate([pose], g.surface(None, nose_up, NOW)).scales == [1.0]  # the robot plane stays the higher one


def test_surface_override_allows_normal_speed_down_to_a_stair() -> None:
    g = guard()
    below_floor = pose_at(0.2, CONFIG.arm.floor_z_m - 0.02)  # 2 cm below the robot plane
    assert g.evaluate([below_floor], g.surface(None, None, NOW)).scales == [0.2]
    stair = g.surface(FloorOverride(surface_z_m=-0.18), None, NOW)
    assert stair.surface_z_m == -0.18
    assert g.evaluate([below_floor], stair).scales == [1.0]


def test_config_surface_and_margin_and_slow_scale_are_used() -> None:
    g = guard(margin_m=0.0, slow_speed_scale=0.5, surface_z_m=-0.05)
    pose = pose_at(0.2, CONFIG.arm.floor_z_m - 0.02)
    assert g.evaluate([pose], g.surface(None, None, NOW)).scales == [1.0]
    g2 = guard(margin_m=0.0, slow_speed_scale=0.5)
    assert g2.evaluate([pose], g2.surface(None, None, NOW)).scales == [0.5]


def test_disabled_guard_never_slows() -> None:
    g = guard(enabled=False)
    pose = pose_at(0.2, CONFIG.arm.floor_z_m - 0.02)
    assert g.evaluate([pose], g.surface(None, None, NOW)).scales == [1.0]


def test_elbow_and_wrist_are_checked_too() -> None:
    g = guard()
    pose = pose_at(0.2, -0.05)
    points = g.checked_points(pose)
    assert set(points) == {"elbow", "wrist", "jaw_tip", "tool_point", "moving_jaw_tip"}
    assert points["elbow"] == pytest.approx(KIN.link_frame(pose, "lower_arm_link")[:3, 3])
    assert points["wrist"] == pytest.approx(KIN.link_frame(pose, "wrist_link")[:3, 3])


def test_jaw_model_gap_grows_with_opening_and_inverts() -> None:
    jaw = JawModel(KIN, CONFIG.arm.jaw_open_axis, CONFIG.arm.gripper_closed_rad)
    closed = CONFIG.arm.gripper_closed_rad
    assert jaw.gap(closed) == pytest.approx(0.0, abs=1e-9)
    assert 0.0 < jaw.gap(0.5) < jaw.gap(1.0) < jaw.gap(1.5)
    angle = jaw.angle_for_gap(0.05)
    assert jaw.gap(angle) == pytest.approx(0.05, abs=1e-4)
    # The moving jaw tip of a closed gripper is at the tool point; an open one lies in the jaw opening direction.
    pose = pose_at(0.2, -0.05)
    t_gripper = KIN.link_frame(pose, "gripper_link")
    tip_closed = jaw.moving_tip(t_gripper, closed)
    tool = KIN.forward(pose)
    assert tip_closed == pytest.approx([tool.x, tool.y, tool.z], abs=1e-6)


def test_step_scales_take_the_slower_end_of_each_step() -> None:
    assert step_scales([1.0, 1.0, 0.2, 1.0]) == [1.0, 0.2, 0.2]
    assert step_scales([1.0]) == []


def test_retime_subdivides_slow_steps_and_ends_at_the_goal() -> None:
    start = {"a": 0.0}
    points = [{"a": 0.1}, {"a": 0.2}, {"a": 0.3}]
    out = retime(start, points, [1.0, 0.2, 1.0])
    assert len(out) == 1 + 5 + 1
    assert out[0] == {"a": 0.1}
    assert [p["a"] for p in out[1:6]] == pytest.approx([0.12, 0.14, 0.16, 0.18, 0.2])
    assert out[-1] == {"a": 0.3}
    assert retime(start, points, [1.0, 1.0, 1.0]) == points


def test_monitor_imu_record_becomes_a_tilt_sample() -> None:
    assert monitor_tilt_sample(None) is None
    sample = monitor_tilt_sample((12.5, 10.0, -5.0, 11.2))
    assert sample is not None and sample.stamp == 12.5
    assert sample.tilt.roll_rad == pytest.approx(math.radians(10.0))
    assert sample.tilt.pitch_rad == pytest.approx(math.radians(-5.0))
