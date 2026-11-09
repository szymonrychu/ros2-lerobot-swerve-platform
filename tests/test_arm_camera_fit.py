"""Tests of the arm to camera fit (shared/ros2_common/head_rgbd_calibration/arm_camera_fit.py) on synthetic D435
scenes: the gripper-held 6x5 board seen through the applied (exact, fixed) head chain, with arm offsets, an arm base,
a board seat skew and a gravity sag that differ from the current values (tests/head_arm_synthetic.py
`arm_camera_scene`).
"""

import dataclasses
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from arm_collision.settings import ArmModelSettings
from ros2_common.head_rgbd_calibration.arm_camera_fit import (
    ArmCameraFit,
    ArmFk,
    arm_camera_report,
    assess,
    calibration_snippet_yaml,
    fit_arm_camera,
    gripper_in_base,
    prediction_error_mm,
)
from ros2_common.head_rgbd_calibration.chain import UrdfChain, pose_from_xyz_rpy
from ros2_common.head_rgbd_calibration.config import (
    ARM_JOINT_NAMES,
    ArmCameraBounds,
    ArmCameraFitConfig,
    ArmFitLimits,
)
from scipy.spatial.transform import Rotation

from tests.head_arm_synthetic import (
    ARM_CAMERA_SAG_RAD_PER_NM,
    PLATE_ARM_POSES,
    SYNTH_CAMERA_MATRIX,
    SYNTH_DIST,
    arm_camera_config,
    arm_camera_scene,
    arm_camera_truth,
    synth_arm_urdf,
    synth_head_urdf,
)

NOISE_PX = 0.3
SEED = 17
OFFSET_TOLERANCE_RAD = 0.003
BASE_TOLERANCE_M = 0.0015
SKEW_TOLERANCE_MM = 1.0
SKEW_TOLERANCE_DEG = 0.3
SAG_TOLERANCE_RAD_PER_NM = 0.12
GAUGE_TOLERANCE_M = 1e-5  # the URDF rpy is pi to 6 digits: the pan axis is vertical to 3e-6 rad
REPORT_KEYS = {
    "plausible", "refusals", "warnings", "score", "params", "accuracy_examples", "comparison", "skew", "sag", "notes",
}  # fmt: skip
SCORE_KEYS = {"rms_px", "heldout_rms_px", "heldout_tool_mm", "views", "poses"}
PARAM_KEYS = {"name", "value", "sigma", "unit", "applied_value", "change"}
EXAMPLE_LABELS = (
    "tool point at 30 cm reach",
    "tool point at full reach",
    "gripper position seen by the head camera at 50 cm",
)


def fit_scene(truth: object, **overrides: object) -> ArmCameraFit:
    """Fit the standard arm to camera scene of `truth` with the arm_camera config overrides."""
    views = arm_camera_scene(truth, PLATE_ARM_POSES, NOISE_PX, SEED)
    cfg = arm_camera_config(**overrides)
    return fit_arm_camera(views, SYNTH_CAMERA_MATRIX, SYNTH_DIST, synth_head_urdf(), synth_arm_urdf(), cfg)


@pytest.fixture(scope="module")
def standard() -> tuple[object, ArmCameraFit]:
    truth = arm_camera_truth()
    return truth, fit_scene(truth)


def test_recovers_arm_offsets_and_base(standard: tuple[object, ArmCameraFit]) -> None:
    truth, fit = standard
    for joint in ARM_JOINT_NAMES:
        assert abs(fit.offsets[joint] - truth.joint_offsets_rad[joint]) <= OFFSET_TOLERANCE_RAD, joint
    assert fit.offsets["wrist_roll"] == fit.current_offsets["wrist_roll"]  # the board rotation gauge
    for key in ("x", "y", "z"):
        assert abs(fit.base[key] - truth.base[key]) <= BASE_TOLERANCE_M, key
    assert fit.base["yaw"] == fit.current_base["yaw"]  # the shoulder_pan gauge


def test_recovers_board_skew_and_sag(standard: tuple[object, ArmCameraFit]) -> None:
    truth, fit = standard
    cfg = arm_camera_config()
    cad = pose_from_xyz_rpy(cfg.board_mount.xyz_init, cfg.board_mount.rpy_init)
    seat = pose_from_xyz_rpy(truth.board_xyz, truth.board_rpy)
    want_mm = (seat[:3, 3] - cad[:3, 3]) * 1000.0
    want_deg = np.degrees(Rotation.from_matrix(cad[:3, :3].T @ seat[:3, :3]).as_euler("xyz"))
    assert np.allclose(fit.skew["xyz_mm"], want_mm, atol=SKEW_TOLERANCE_MM)
    assert np.allclose(fit.skew["rpy_deg"], want_deg, atol=SKEW_TOLERANCE_DEG)
    assert np.linalg.norm(want_mm) > 3.0  # the skew is real, the prior did not hide it
    assert abs(math.radians(fit.sag["deg_per_nm"]) - ARM_CAMERA_SAG_RAD_PER_NM) <= SAG_TOLERANCE_RAD_PER_NM
    assert fit.sag["at_reference_deg"] > 1.0 and fit.sag["range_deg"][0] < fit.sag["range_deg"][1]


def test_score_is_the_noise_and_the_fit_is_plausible(standard: tuple[object, ArmCameraFit]) -> None:
    _, fit = standard
    assert fit.rms_px < 2 * NOISE_PX and fit.holdout_rms_px < 2 * NOISE_PX
    assert fit.holdout_tool_mm < 1.0
    assert fit.holdout_views > 0 and fit.views > fit.holdout_views
    assert fit.max_correlation.value < 0.995
    refusals, _ = assess(fit, arm_camera_config())
    assert refusals == []


def test_current_calibration_scores_worse(standard: tuple[object, ArmCameraFit]) -> None:
    _, fit = standard
    assert fit.current.rms_px > 3 * fit.rms_px
    assert all(shift_mm > 1.0 for _, shift_mm in fit.shifts)


def test_report_matches_the_contract(standard: tuple[object, ArmCameraFit]) -> None:
    _, fit = standard
    report = arm_camera_report(fit, arm_camera_config())
    assert set(report) == REPORT_KEYS
    assert report["plausible"] is True and report["refusals"] == []
    assert set(report["score"]) == SCORE_KEYS
    assert report["score"]["poses"] == len(PLATE_ARM_POSES)
    assert all(set(p) == PARAM_KEYS for p in report["params"])
    names = {p["name"] for p in report["params"]}
    assert {"shoulder_pan_offset", "wrist_roll_offset", "base_x", "base_yaw", "sag"} <= names
    pan = next(p for p in report["params"] if p["name"] == "shoulder_pan_offset")
    assert pan["unit"] == "deg" and pan["change"] == pytest.approx(pan["value"] - pan["applied_value"])
    roll = next(p for p in report["params"] if p["name"] == "wrist_roll_offset")
    assert roll["sigma"] is None and roll["change"] == 0.0  # fixed: the gauge
    labels = [e["label"] for e in report["accuracy_examples"]]
    for label in EXAMPLE_LABELS:
        assert f"{label}, 1 sigma" in labels and f"{label}, 95 %" in labels
    assert all(e["unit"] == "mm" and e["value"] > 0 for e in report["accuracy_examples"])
    assert set(report["skew"]) == {"xyz_mm", "rpy_deg"}
    assert set(report["comparison"]) == {"applied", "current_score", "shift_examples"}
    assert report["comparison"]["applied"] is None
    assert any("gauge" in note for note in report["notes"])


def test_base_yaw_is_a_gauge_of_the_shoulder_pan_offset() -> None:
    """A base yaw turn equals the same shoulder_pan offset change plus a base x, y shift: the fit fixes the yaw."""
    arm = UrdfChain.from_text(synth_arm_urdf())
    offsets = dict(arm_camera_config().joint_offsets_rad)
    base = {"x": 0.06, "y": -0.05, "z": 0.1, "yaw": 0.0}
    turn = 0.05
    pan_xyz, pan_rpy = arm.origin("shoulder_pan")
    pan_xyz = np.asarray(pan_xyz)
    # The shoulder_pan axis points down in the arm base (URDF rpy pi, 0, -pi): a +yaw turn is a -offset change.
    up = float((pose_from_xyz_rpy(pan_xyz, pan_rpy)[:3, :3] @ np.asarray(arm.joints["shoulder_pan"].axis))[2])
    rotation = Rotation.from_euler("z", turn).as_matrix()
    shift = rotation @ pan_xyz - pan_xyz
    turned = {**base, "yaw": turn}
    shifted = {**base, "x": base["x"] + shift[0], "y": base["y"] + shift[1]}
    moved = {**offsets, "shoulder_pan": offsets["shoulder_pan"] + turn * up}
    for joints in PLATE_ARM_POSES[:6]:
        a = gripper_in_base(arm, joints, offsets, turned)
        b = gripper_in_base(arm, joints, moved, shifted)
        assert np.allclose(a, b, atol=GAUGE_TOLERANCE_M)


def test_wrist_roll_offset_is_a_gauge_of_the_board_rotation() -> None:
    """A wrist_roll offset change is the board turned about the wrist_roll axis: the fit fixes wrist_roll."""
    arm = UrdfChain.from_text(synth_arm_urdf())
    offsets = dict(arm_camera_config().joint_offsets_rad)
    base = {"x": 0.06, "y": -0.05, "z": 0.1, "yaw": 0.0}
    board = pose_from_xyz_rpy((0.01, 0.02, -0.1), (math.pi, 0.0, 0.2))
    delta = 0.07
    spin = np.eye(4)
    spin[:3, :3] = Rotation.from_rotvec(np.asarray(arm.joints["wrist_roll"].axis) * delta).as_matrix()
    rolled = {**offsets, "wrist_roll": offsets["wrist_roll"] + delta}
    for joints in PLATE_ARM_POSES[:6]:
        a = gripper_in_base(arm, joints, rolled, base) @ board
        b = gripper_in_base(arm, joints, offsets, base) @ spin @ board
        assert np.allclose(a, b, atol=GAUGE_TOLERANCE_M)


def test_implausible_changes_are_refused(standard: tuple[object, ArmCameraFit]) -> None:
    _, fit = standard
    cfg = arm_camera_config()
    wild = dataclasses.replace(
        fit,
        offsets={**fit.offsets, "elbow_flex": fit.current_offsets["elbow_flex"] + math.radians(20.0)},
        base={**fit.base, "x": fit.current_base["x"] + 0.12},
        skew={"xyz_mm": [11.0, 0.0, 0.0], "rpy_deg": [0.0, 0.0, 12.0]},
        sag={**fit.sag, "at_reference_deg": 9.0},
    )
    refusals, _ = assess(wild, cfg)
    names = {r["name"] for r in refusals}
    assert {"offset_change:elbow_flex", "base_move_mm", "skew_mm", "skew_deg", "sag_deg"} <= names
    elbow = next(r for r in refusals if r["name"] == "offset_change:elbow_flex")
    assert elbow["value"] == pytest.approx(20.0) and elbow["limit"] == cfg.arm_camera.bounds.max_offset_change_deg
    assert elbow["unit"] == "deg" and "elbow_flex" in elbow["message"]
    report = arm_camera_report(wild, cfg)
    assert report["plausible"] is False and report["refusals"] == refusals


def test_tight_bounds_refuse_the_standard_fit(standard: tuple[object, ArmCameraFit]) -> None:
    _, fit = standard
    cfg = arm_camera_config(bounds=ArmCameraBounds(max_offset_change_deg=1.0))
    names = {r["name"] for r in assess(fit, cfg)[0]}
    assert {"offset_change:shoulder_pan", "offset_change:wrist_flex"} <= names


def test_non_finite_values_are_refused(standard: tuple[object, ArmCameraFit]) -> None:
    _, fit = standard
    broken = dataclasses.replace(fit, offsets={**fit.offsets, "shoulder_lift": math.nan})
    assert "non_finite" in {r["name"] for r in assess(broken, arm_camera_config())[0]}


def test_advisory_limits_are_warnings_not_refusals(standard: tuple[object, ArmCameraFit]) -> None:
    _, fit = standard
    cfg = arm_camera_config().model_copy(update={"arm_limits": ArmFitLimits(rms_px=0.01, min_views=10_000)})
    refusals, warnings = assess(fit, cfg)
    assert refusals == []
    by_name = {w["name"]: w for w in warnings}
    assert {"rms_px", "min_views"} <= set(by_name)
    assert by_name["rms_px"]["limit"] == 0.01 and by_name["rms_px"]["value"] == pytest.approx(fit.rms_px)


def test_prediction_error_grows_with_the_covariance() -> None:
    jac = np.array([[1.0, 0.0, 0.5], [0.0, 2.0, 0.0], [0.3, 0.0, 1.0]])
    cov = np.diag([1e-6, 4e-6, 2e-6])
    sigma, p95 = prediction_error_mm(jac, cov)
    assert 0 < sigma < p95
    for factor in (2.0, 9.0):
        bigger = prediction_error_mm(jac, cov * factor)
        assert bigger[0] == pytest.approx(sigma * math.sqrt(factor))
        assert bigger[1] == pytest.approx(p95 * math.sqrt(factor))
    assert prediction_error_mm(jac, np.full((3, 3), math.inf)) == (math.inf, math.inf)


def test_accuracy_examples_follow_the_sigma(standard: tuple[object, ArmCameraFit]) -> None:
    """More noise gives a larger covariance and larger estimated errors at every example."""
    truth, fit = standard
    noisy_views = arm_camera_scene(truth, PLATE_ARM_POSES, 4 * NOISE_PX, SEED)
    noisy = fit_arm_camera(
        noisy_views, SYNTH_CAMERA_MATRIX, SYNTH_DIST, synth_head_urdf(), synth_arm_urdf(), arm_camera_config()
    )
    assert [e.label for e in fit.accuracy] == list(EXAMPLE_LABELS)
    for quiet, loud in zip(fit.accuracy, noisy.accuracy, strict=True):
        assert 0 < quiet.sigma_mm < loud.sigma_mm and quiet.p95_mm < loud.p95_mm
        assert quiet.sigma_mm <= quiet.p95_mm


def test_snippet_is_what_the_mcp_server_arm_model_reads(standard: tuple[object, ArmCameraFit]) -> None:
    """The reviewed client.yml snippet validates as mcp_server's `arm:` block (arm_collision ArmModelSettings)."""
    _, fit = standard
    snippet = calibration_snippet_yaml(fit, "calib-20261109-120000", "r-20261109-121000", "2026-11-09")
    block = yaml.safe_load(snippet)
    arm = ArmModelSettings.model_validate(block)
    assert arm.joint_offsets_rad.model_dump() == pytest.approx(fit.offsets, abs=1e-4)
    mount = arm.base_in_base_link
    assert (mount.x, mount.y, mount.z, mount.yaw) == pytest.approx(
        (fit.base["x"], fit.base["y"], fit.base["z"], fit.base["yaw"]), abs=1e-4
    )
    assert arm.arm_base_height_m == pytest.approx(fit.base["z"], abs=1e-4)
    assert all(line.startswith(" " * 8) for line in snippet.splitlines())


def test_too_few_views_is_an_error() -> None:
    views = arm_camera_scene(arm_camera_truth(), PLATE_ARM_POSES[:1], NOISE_PX, SEED)[:2]
    with pytest.raises(ValueError, match="at least 3"):
        fit_arm_camera(views, SYNTH_CAMERA_MATRIX, SYNTH_DIST, synth_head_urdf(), synth_arm_urdf(), arm_camera_config())


def test_board_mass_model_defaults_are_the_arm_board_cad() -> None:
    """The default board mass model is the ARM board metadata (markers/charuco-arm-6x5-28mm.json)."""
    meta = json.loads((Path(__file__).resolve().parents[1] / "markers" / "charuco-arm-6x5-28mm.json").read_text())
    model = ArmCameraFitConfig()
    assert model.board_mass_kg == pytest.approx(meta["mass_g"] / 1000.0, abs=1e-6)
    assert model.board_com_m == pytest.approx(meta["centre_of_mass_m"], abs=1e-6)
    assert model.grip_point_m == pytest.approx(meta["handle_in_board"]["tool_point_m"], abs=1e-6)
    assert model.grip_axis == pytest.approx(meta["handle_in_board"]["open_axis"], abs=1e-9)


def test_fast_fk_equals_the_urdf_chain() -> None:
    arm = UrdfChain.from_text(synth_arm_urdf())
    fk = ArmFk(arm)
    offsets = dict(arm_camera_config().joint_offsets_rad)
    for joints in PLATE_ARM_POSES[::5]:
        assert np.allclose(fk.pose(joints, offsets), arm.transform("gripper_link", joints, offsets=offsets), atol=1e-12)


def test_without_sag_the_fitted_sag_is_near_zero() -> None:
    fit = fit_scene(arm_camera_truth(sag_rad_per_nm=0.0))
    assert abs(math.radians(fit.sag["deg_per_nm"])) <= SAG_TOLERANCE_RAD_PER_NM
    assert abs(fit.sag["at_reference_deg"]) < 0.3


def test_switches_free_wrist_roll_droop_and_fix_base_z() -> None:
    truth = arm_camera_truth()
    fit = fit_scene(truth, fit_wrist_roll=True, fit_droop=True, fit_base_z=False)
    assert {"wrist_roll_offset", "droop"} <= set(fit.free) and "base_z" not in fit.free
    assert fit.base["z"] == fit.current_base["z"]
    assert fit.sag["droop_mm_per_nm"] is not None and abs(fit.sag["droop_at_reference_mm"]) < 1.0
    assert any("wrist_roll offset fitted" in note for note in fit.notes)
    assert any("base z fixed" in note for note in fit.notes)
    report = arm_camera_report(fit, arm_camera_config(fit_droop=True))
    assert any(p["name"] == "droop" and p["unit"] == "mm/(N m)" for p in report["params"])
    roll = next(p for p in report["params"] if p["name"] == "wrist_roll_offset")
    assert roll["sigma"] is not None


def test_solver_summary(standard: tuple[object, ArmCameraFit]) -> None:
    _, fit = standard
    assert set(fit.solver) == {"status", "message", "nfev", "cost", "start_rms_px", "free"}
    assert fit.solver["nfev"] > 0 and fit.solver["status"] > 0 and fit.solver["free"] == list(fit.free)
    assert fit.solver["start_rms_px"] > fit.rms_px
