"""Tests of the joint head and arm offsets fit (shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py) on
synthetic D435 scenes: the gripper-held 6x5 plate in the colour camera and floor planes in the depth frame, seen
through a truth head chain (the applied floor-fit chain moved by `SYNTH_TRUTH_DELTAS`) and true arm offsets that
differ from the current ones by `PLATE_OFFSET_CHANGES` (tests/head_arm_synthetic.py).
"""

import math

import numpy as np
import pytest
import yaml
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.arm_offsets_fit import (
    WRIST_ROLL_FITTED_NOTE,
    WRIST_ROLL_FIXED_NOTE,
    HeadArmFit,
    arm_report_markdown,
    fit_head_arm,
    offsets_snippet_yaml,
)
from ros2_common.head_rgbd_calibration.chain import UrdfChain, arm_in_robot_base, pose_from_xyz_rpy
from ros2_common.head_rgbd_calibration.config import (
    ARM_JOINT_NAMES,
    DEFAULT_JOINT_OFFSETS_RAD,
    ArmFitLimits,
    HeadArmFitConfig,
    PlateMountConfig,
)
from ros2_common.head_rgbd_calibration.head_arm_fit import GRIPPER_LINK, start_params
from scipy.spatial.transform import Rotation

from tests.head_arm_synthetic import (
    FLOOR_GRID,
    FLOOR_NOISE_DEG,
    FLOOR_NOISE_MM,
    PLATE_ARM_POSES,
    SYNTH_CAMERA_MATRIX,
    SYNTH_DIST,
    SYNTH_TOOL_TRUTH_M,
    SYNTH_TRUTH_DELTAS,
    SYNTH_WRIST_ROLL_NEUTRAL_RAD,
    PlateTruth,
    floor_scene,
    gripper_frame_in_gripper,
    plate_scene,
    plate_truth,
    synth_arm_urdf,
    synth_head_urdf,
    synth_plate_config,
    tool_in_plate,
)

HEAD_URDF = synth_head_urdf()
ARM_URDF = synth_arm_urdf()
NOISE_PX = 0.3
SEED = 11
FLOOR_SEED = 5
SESSION = "arm-20261028-120000"
DATE = "2026-10-28"
CURRENT = dict(DEFAULT_JOINT_OFFSETS_RAD)
FREE_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")
SIGMA_FACTOR = 3.0
OFFSET_TOLERANCE_RAD = 0.004
HEAD_TOLERANCE_MM = 2.0
YAW_TOLERANCE_DEG = 0.2
PLATE_TOLERANCE_DEG = 0.2
HOLDOUT_TOOL_MM = 3.0
TOOL_TOLERANCE_MM = 2.0
CAD_WRIST_ROLL_RAD = 0.06
WRIST_ROLL_ZERO_ERROR_DEG = 2.0
ABSORBED_TOOL_SHIFT_MM = 1.0
TOO_FEW_VIEWS = 20


def fit(plate_views: list, floor_views: list, cfg: HeadArmFitConfig) -> HeadArmFit:
    return fit_head_arm(floor_views, plate_views, SYNTH_CAMERA_MATRIX, SYNTH_DIST, HEAD_URDF, ARM_URDF, cfg, CURRENT)


def plate_views(t: PlateTruth) -> list:
    return plate_scene(t, PLATE_ARM_POSES, NOISE_PX, SEED)


def floor_views(t: PlateTruth) -> list:
    return floor_scene(t, FLOOR_GRID, FLOOR_NOISE_DEG, FLOOR_NOISE_MM, FLOOR_SEED)


def rotation_error_deg(a: np.ndarray, b: np.ndarray) -> float:
    """Angle (deg) of the rotation between two 3x3 rotations."""
    return math.degrees(Rotation.from_matrix(a.T @ b).magnitude())


def fitted_plate(result: HeadArmFit) -> np.ndarray:
    return pose_from_xyz_rpy(result.plate_pose["xyz"], result.plate_pose["rpy"])


def tool_in_base(offsets: dict[str, float], joints: dict[str, float], tool_m: tuple[float, float, float]) -> np.ndarray:
    """The tool point (given in gripper_frame_link) in base_link by arm FK with the given offsets."""
    arm = UrdfChain.from_text(ARM_URDF)
    base = arm_in_robot_base(HeadArmFitConfig().base_in_base_link.model_dump())
    gripper = base @ arm.transform(GRIPPER_LINK, joints, offsets=offsets)
    return (gripper @ gripper_frame_in_gripper(ARM_URDF) @ np.append(tool_m, 1.0))[:3]


@pytest.fixture(scope="module")
def truth() -> PlateTruth:
    return plate_truth()


@pytest.fixture(scope="module")
def views(truth: PlateTruth) -> tuple[list, list]:
    return plate_views(truth), floor_views(truth)


@pytest.fixture(scope="module")
def tool_cfg(truth: PlateTruth) -> HeadArmFitConfig:
    return synth_plate_config(truth, tool_in_board_m=tool_in_plate(truth))


@pytest.fixture(scope="module")
def default_fit(views: tuple[list, list], tool_cfg: HeadArmFitConfig) -> HeadArmFit:
    """Default mode (no CAD rotation) with the CAD tool offset measured."""
    return fit(*views, tool_cfg)


def test_recovers_arm_offsets_and_head_yaw(default_fit: HeadArmFit, truth: PlateTruth) -> None:
    for joint in FREE_JOINTS:
        error = abs(default_fit.offsets[joint] - truth.joint_offsets_rad[joint])
        sigma = default_fit.sigma[f"{joint}_offset_rad"]
        assert error <= OFFSET_TOLERANCE_RAD, (joint, default_fit.offsets[joint], truth.joint_offsets_rad[joint])
        assert error <= SIGMA_FACTOR * sigma, (joint, error, sigma)
    start = start_params(UrdfChain.from_text(HEAD_URDF), ("mount_x_m", "mount_y_m", "mount_yaw_deg"))
    for name in ("mount_x_m", "mount_y_m"):
        error_mm = abs(default_fit.params[name] - (start[name] + SYNTH_TRUTH_DELTAS[name])) * 1000.0
        assert error_mm <= HEAD_TOLERANCE_MM, (name, error_mm)
    yaw_error = abs(
        default_fit.params["mount_yaw_deg"] - (start["mount_yaw_deg"] + SYNTH_TRUTH_DELTAS["mount_yaw_deg"])
    )
    assert yaw_error <= YAW_TOLERANCE_DEG
    assert default_fit.current_offsets == CURRENT
    assert default_fit.holdout_tool_mm < HOLDOUT_TOOL_MM
    assert default_fit.rms_px < 0.5
    assert default_fit.holdout_rms_px < 0.6
    assert default_fit.failed_limits == []
    assert default_fit.passed


def test_default_fits_plate_rotation_with_wrist_roll_fixed(
    default_fit: HeadArmFit, truth: PlateTruth, tool_cfg: HeadArmFitConfig
) -> None:
    assert not default_fit.wrist_roll_fitted
    assert default_fit.offsets["wrist_roll"] == SYNTH_WRIST_ROLL_NEUTRAL_RAD
    assert "wrist_roll_offset_rad" not in default_fit.params
    plate = fitted_plate(default_fit)
    assert rotation_error_deg(plate[:3, :3], truth.plate_pose()[:3, :3]) <= PLATE_TOLERANCE_DEG
    report = arm_report_markdown(default_fit, tool_cfg, SESSION, DATE)
    assert WRIST_ROLL_FIXED_NOTE == "wrist_roll fixed at the recorded neutral; plate rotation fitted"
    assert WRIST_ROLL_FIXED_NOTE in report
    for joint in ARM_JOINT_NAMES:
        assert joint in report
    assert "holdout_tool_mm" in report and "holdout_coverage" in report and "PASS" in report
    # The recorded neutral is required whenever wrist_roll is fixed.
    with pytest.raises(ValueError, match="wrist_roll_offset_rad"):
        fit(plate_views(truth)[:6], floor_views(truth), tool_cfg.model_copy(update={"wrist_roll_offset_rad": None}))


def test_wrist_roll_zero_error_is_absorbed_by_the_plate(truth: PlateTruth) -> None:
    """A 2 deg error of the recorded wrist_roll neutral turns the fitted plate about the wrist_roll axis and moves the
    FK tool point by less than 1 mm (the tool point is 8 mm off that axis)."""
    error = math.radians(WRIST_ROLL_ZERO_ERROR_DEG)
    off = plate_truth(wrist_roll_rad=SYNTH_WRIST_ROLL_NEUTRAL_RAD + error)
    cfg = synth_plate_config(off)
    result = fit(plate_views(off), floor_views(off), cfg)
    assert not result.wrist_roll_fitted
    assert result.offsets["wrist_roll"] == SYNTH_WRIST_ROLL_NEUTRAL_RAD
    # Absorbed: the plate turned by the zero error about gripper_link z (the wrist_roll axis).
    absorbed = Rotation.from_rotvec([0.0, 0.0, error]).as_matrix() @ off.plate_pose()[:3, :3]
    assert rotation_error_deg(fitted_plate(result)[:3, :3], absorbed) <= PLATE_TOLERANCE_DEG
    assert result.rms_px < 0.5
    assert result.passed, result.failed_limits
    tool = tuple(cfg.tool_offset_m)
    shift = max(
        float(np.linalg.norm(tool_in_base(result.offsets, j, tool) - tool_in_base(off.joint_offsets_rad, j, tool)))
        for j in PLATE_ARM_POSES
    )
    assert shift * 1000.0 < ABSORBED_TOOL_SHIFT_MM


def test_cad_rotation_frees_wrist_roll() -> None:
    cad = plate_truth(wrist_roll_rad=CAD_WRIST_ROLL_RAD)
    cfg = synth_plate_config(cad, rpy=cad.plate_rpy)
    result = fit(plate_views(cad), floor_views(cad), cfg)
    assert result.wrist_roll_fitted
    assert abs(result.offsets["wrist_roll"] - CAD_WRIST_ROLL_RAD) <= OFFSET_TOLERANCE_RAD
    assert "wrist_roll_offset_rad" in result.sigma
    # The plate rotation is the CAD rotation; only its translation is fitted.
    np.testing.assert_allclose(fitted_plate(result)[:3, :3], cad.plate_pose()[:3, :3], atol=1e-9)
    for joint in FREE_JOINTS:
        assert abs(result.offsets[joint] - cad.joint_offsets_rad[joint]) <= OFFSET_TOLERANCE_RAD, joint
    assert result.passed, result.failed_limits
    report = arm_report_markdown(result, cfg, SESSION, DATE)
    assert WRIST_ROLL_FITTED_NOTE in report
    assert WRIST_ROLL_FIXED_NOTE not in report


def test_tool_point_fitted_and_gated(
    default_fit: HeadArmFit, views: tuple[list, list], truth: PlateTruth, tool_cfg: HeadArmFitConfig
) -> None:
    # Fitted: the plate pose composed with the CAD board-to-jaw-tip offset, in gripper_frame_link.
    assert default_fit.tool_fitted
    error_mm = np.linalg.norm(np.subtract(default_fit.tool_offset_m, SYNTH_TOOL_TRUTH_M)) * 1000.0
    assert error_mm < TOOL_TOLERANCE_MM
    assert default_fit.holdout_tool_mm < tool_cfg.arm_limits.max_tool_holdout_mm
    assert "tool_offset_m" in yaml.safe_load(offsets_snippet_yaml(default_fit, SESSION, DATE))

    # Above the held-out limit: not fitted, the current tool point reported unchanged with the reason.
    limit = default_fit.holdout_tool_mm / 2.0
    tight = tool_cfg.model_copy(update={"arm_limits": ArmFitLimits(max_tool_holdout_mm=limit)})
    gated = fit(*views, tight)
    assert not gated.tool_fitted
    assert gated.tool_offset_m == tuple(tight.tool_offset_m)
    assert gated.holdout_tool_mm >= limit
    assert "holdout_tool_mm" in gated.failed_limits
    assert not gated.passed
    assert "held-out tool error" in gated.tool_note
    assert gated.tool_note in arm_report_markdown(gated, tight, SESSION, DATE)
    assert "tool_offset_m" not in yaml.safe_load(offsets_snippet_yaml(gated, SESSION, DATE))

    # Not measured (tool_in_board_m null): not fitted, the reason says so.
    unmeasured = synth_plate_config(truth)
    blind = fit(*views, unmeasured)
    assert not blind.tool_fitted
    assert blind.tool_offset_m == tuple(unmeasured.tool_offset_m)
    assert "tool_in_board_m" in blind.tool_note
    assert blind.passed, blind.failed_limits


def test_holdout_needs_three_head_settings(views: tuple[list, list], tool_cfg: HeadArmFitConfig) -> None:
    plates, floors = views
    two = [v for v in plates if v.head_offset in (0, 1)]
    result = fit(two, floors, tool_cfg)
    assert result.holdout_head_settings == 2
    assert "holdout_coverage" in result.failed_limits
    assert not result.passed


def test_offsets_snippet_round_trips(default_fit: HeadArmFit) -> None:
    # shoulder_pan is the gauge: never fitted, the current value.
    assert default_fit.offsets["shoulder_pan"] == CURRENT["shoulder_pan"]
    assert "shoulder_pan_offset_rad" not in default_fit.params
    text = offsets_snippet_yaml(default_fit, SESSION, DATE)
    lines = text.splitlines()
    assert lines[0].lstrip().startswith("#")
    assert SESSION in lines[0] and DATE in lines[0] and "px" in lines[0] and "mm" in lines[0]
    assert sum(line.lstrip().startswith("#") for line in lines) == 1
    # Indented for the client.yml mcp_server `arm:` block (its keys sit 8 spaces in).
    assert all(line.startswith(" " * 8) and not line.startswith(" " * 9) for line in lines if ":" in line[:12])
    data = yaml.safe_load(text)
    assert list(data["joint_offsets_rad"]) == list(ARM_JOINT_NAMES)
    for joint in ARM_JOINT_NAMES:
        assert data["joint_offsets_rad"][joint] == pytest.approx(round(default_fit.offsets[joint], 4), abs=1e-12)
    assert data["joint_offsets_rad"]["shoulder_pan"] == pytest.approx(CURRENT["shoulder_pan"], abs=1e-12)
    tool = data["tool_offset_m"]
    assert [tool[k] for k in ("x", "y", "z")] == pytest.approx([round(v, 4) for v in default_fit.tool_offset_m])


def test_too_few_plate_views_fail(views: tuple[list, list], tool_cfg: HeadArmFitConfig) -> None:
    plates, floors = views
    result = fit(plates[:TOO_FEW_VIEWS], floors, tool_cfg)
    assert "min_views" in result.failed_limits
    assert not result.passed
    with pytest.raises(ValueError, match="3 plate views"):
        fit(plates[:2], floors, tool_cfg)


def test_fit_inputs_are_checked(views: tuple[list, list], tool_cfg: HeadArmFitConfig) -> None:
    plates, floors = views
    with pytest.raises(ValueError, match="plate_mount"):
        fit(plates, floors, tool_cfg.model_copy(update={"plate_mount": None}))
    with pytest.raises(ValueError, match="current_offsets"):
        fit_head_arm(
            floors,
            plates,
            SYNTH_CAMERA_MATRIX,
            SYNTH_DIST,
            HEAD_URDF,
            ARM_URDF,
            tool_cfg,
            {k: v for k, v in CURRENT.items() if k != "elbow_flex"},
        )


def test_arm_offsets_config() -> None:
    cfg = HeadArmFitConfig()
    assert cfg.plate_mount is None
    assert cfg.wrist_roll_offset_rad is None
    assert cfg.plate == cfg.board  # the rigid 6x5 board
    assert cfg.arm_limits.max_tool_holdout_mm == 5.0
    assert cfg.arm_limits.min_holdout_head_settings == 3
    assert tuple(cfg.tool_offset_m) == (0.0010, -0.0056, -0.0014)  # client.yml mcp_server arm.tool_offset_m
    # The CAD board-to-jaw-tip offset has no default: the key is required, null until it is measured.
    with pytest.raises(ValidationError, match="tool_in_board_m"):
        PlateMountConfig(xyz_init=(0.0, 0.0, 0.0), rpy_init=(0.0, 0.0, 0.0))
    unmeasured = PlateMountConfig(xyz_init=(0.0, 0.0, 0.0), rpy_init=(0.0, 0.0, 0.0), tool_in_board_m=None)
    assert unmeasured.tool_in_board_m is None and unmeasured.rpy is None
    with pytest.raises(ValidationError):
        PlateMountConfig(xyz_init=(0.0, 0.0, 0.0), rpy_init=(0.0, 0.0, 0.0), tool_in_board_m=[0.0, 0.0])
