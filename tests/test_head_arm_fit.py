"""Tests of the head-from-arm fit (shared/ros2_common/head_rgbd_calibration/head_arm_fit.py) on synthetic scenes.

Every scene uses nodes/web_ui/urdf/head.urdf with the applied floor-fit chain of 2026-10-22 as overrides (the start
chain) and nodes/web_ui/urdf/so101_arm.urdf; the truth chain is the start chain moved by known amounts.
"""

import json
import math
from dataclasses import replace

import cv2
import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.chain import UrdfChain, arm_in_robot_base, pose_from_xyz_rpy
from ros2_common.head_rgbd_calibration.config import HeadArmFitConfig
from ros2_common.head_rgbd_calibration.head_arm_fit import (
    CAMERA_FRAME,
    PAN_AXIS_PARAMS,
    PARAM_NAMES,
    HeadArmFit,
    aim_head,
    camera_pose,
    fit_head_from_arm,
    fit_param_names,
    head_arm_report_markdown,
    head_origins,
    limit_checks,
    start_params,
)
from ros2_common.head_rgbd_calibration.urdf_text import Origin

from tests.head_arm_synthetic import (
    APPLIED_FLOOR,
    DEFAULT_HEAD_OFFSETS_DEG,
    SYNTH_ARM_POSES,
    SYNTH_CAMERA_MATRIX,
    SYNTH_DIST,
    SYNTH_TRUTH_DELTAS,
    TOOL_POINT,
    URDF_DIR,
    HeadArmTruth,
    arm_board_scene,
    nominal_board_mount,
    synth_arm_urdf,
    synth_fit_config,
    synth_head_urdf,
    synth_truth,
)

HEAD_URDF = synth_head_urdf()
ARM_URDF = synth_arm_urdf()
NOISE_PX = 0.3
SEED = 7
TOLERANCE_MM = 1.5
TOLERANCE_DEG = 0.15
SIGMA_FACTOR = 3.0
FLOOR_OFFSET_DEG = 2.0
# Pan axis moved back and right of the head (m), the kind of shift the 2026-10-23 session showed (-38 mm, -11 mm).
PAN_AXIS_SHIFT_M = (-0.030, -0.010)
FIXED_PAN_MISFIT_PX = 2.0
ALL_NAMES = (*PARAM_NAMES, *PAN_AXIS_PARAMS)


def truth(
    deltas: dict[str, float], extra: dict[str, Origin] | None = None, pan_axis_m: tuple[float, float] = (0.0, 0.0)
) -> HeadArmTruth:
    """Truth: the start chain moved by `deltas` and its pan axis by `pan_axis_m`, plus `extra` origins, and the
    nominal board mount."""
    return synth_truth(HEAD_URDF, deltas, extra, pan_axis_m=pan_axis_m)


def scene(t: HeadArmTruth, noise_px: float, cfg: HeadArmFitConfig | None = None) -> list:
    return arm_board_scene(
        t, HEAD_URDF, ARM_URDF, SYNTH_ARM_POSES, DEFAULT_HEAD_OFFSETS_DEG, noise_px, SEED, cfg or synth_fit_config()
    )


def fit(views: list, cfg: HeadArmFitConfig | None = None, floor: dict[str, float] | None = APPLIED_FLOOR) -> HeadArmFit:
    return fit_head_from_arm(
        views, SYNTH_CAMERA_MATRIX, SYNTH_DIST, HEAD_URDF, ARM_URDF, cfg or synth_fit_config(), floor
    )


def expected(deltas: dict[str, float]) -> dict[str, float]:
    """Parameter values (m, deg) of the start chain moved by `deltas` (pan axis entries included)."""
    start = start_params(UrdfChain.from_text(HEAD_URDF), ALL_NAMES)
    return {name: start[name] + deltas.get(name, 0.0) for name in ALL_NAMES}


@pytest.fixture(scope="module")
def pan_axis_views() -> list:
    return scene(truth(SYNTH_TRUTH_DELTAS, pan_axis_m=PAN_AXIS_SHIFT_M), NOISE_PX)


@pytest.fixture(scope="module")
def noisy_views() -> list:
    return scene(truth(SYNTH_TRUTH_DELTAS), NOISE_PX)


@pytest.fixture(scope="module")
def noisy_fit(noisy_views: list) -> HeadArmFit:
    return fit(noisy_views)


def tolerance(name: str) -> float:
    return TOLERANCE_MM / 1000.0 if name.endswith("_m") else TOLERANCE_DEG


def test_recovers_perturbed_head_chain_with_noise(noisy_fit: HeadArmFit) -> None:
    want = expected(SYNTH_TRUTH_DELTAS)
    assert tuple(noisy_fit.params) == ALL_NAMES
    for name in ALL_NAMES:
        error = abs(noisy_fit.params[name] - want[name])
        assert error <= tolerance(name), (name, noisy_fit.params[name], want[name])
        assert error <= SIGMA_FACTOR * noisy_fit.sigma[name], (name, error, noisy_fit.sigma[name])
    assert noisy_fit.rms_px < 0.5
    assert noisy_fit.holdout_rms_px < 0.6
    assert noisy_fit.holdout_board_mm < 2.0
    assert noisy_fit.failed_limits == []
    assert noisy_fit.passed


def test_fit_reports_coverage_and_holdout(noisy_fit: HeadArmFit, noisy_views: list) -> None:
    cfg = synth_fit_config()
    poses = sorted({v.arm_pose for v in noisy_views})
    held = set(poses[cfg.limits.holdout_every - 1 :: cfg.limits.holdout_every])
    assert noisy_fit.arm_poses == len(poses)
    assert noisy_fit.holdout_views == sum(v.arm_pose in held for v in noisy_views)
    assert noisy_fit.views == len(noisy_views) - noisy_fit.holdout_views
    assert {e.view_id for e in noisy_fit.per_view} == {v.view_id for v in noisy_views}
    assert all(e.held_out == (e.arm_pose in held) for e in noisy_fit.per_view)
    assert noisy_fit.pan_span_deg >= cfg.limits.min_pan_span_deg
    assert noisy_fit.tilt_span_deg >= cfg.limits.min_tilt_span_deg
    assert set(noisy_fit.floor_delta) == set(APPLIED_FLOOR)
    pair, value = noisy_fit.max_correlation.pair, noisy_fit.max_correlation.value
    assert len(pair) == 2 and set(pair) <= set(ALL_NAMES)
    assert abs(value) < cfg.limits.max_correlation  # no free parameter is degenerate with another (design D32)
    data = json.loads(json.dumps(noisy_fit.to_json(), allow_nan=False))
    assert data["passed"] is True
    assert set(data["origins"]) == {"head_mount_joint", "head_pan", "head_tilt"}
    assert set(data["per_view"][0]) == {"view_id", "arm_pose", "rms_px", "held_out"}
    offsets = {entry["head_offset"]: entry for entry in data["offset_rms_px"]}
    assert sorted(offsets) == list(range(len(DEFAULT_HEAD_OFFSETS_DEG)))
    assert sum(entry["views"] for entry in offsets.values()) == len(noisy_views)
    assert all(entry["rms_px"] < 0.5 for entry in offsets.values())


def test_noiseless_recovery_exact() -> None:
    result = fit(scene(truth(SYNTH_TRUTH_DELTAS), 0.0))
    want = expected(SYNTH_TRUTH_DELTAS)
    for name in ALL_NAMES:
        assert result.params[name] == pytest.approx(want[name], abs=1e-6), name
    np.testing.assert_allclose(
        pose_from_xyz_rpy(result.board_mount["xyz"], result.board_mount["rpy"]),
        pose_from_xyz_rpy(*nominal_board_mount(HeadArmFitConfig())),
        atol=1e-6,
    )


def test_pan_zero_folds_into_mount_yaw() -> None:
    chain = UrdfChain.from_text(HEAD_URDF)
    pan_xyz, (pan_roll, pan_pitch, pan_yaw) = chain.origin("head_pan")
    pan_origin = Origin(pan_xyz, (pan_roll, pan_pitch, pan_yaw + math.radians(1.0)))
    result = fit(scene(truth({}, {"head_pan": pan_origin}), NOISE_PX))
    start = start_params(chain)
    assert result.params["mount_yaw_deg"] - start["mount_yaw_deg"] == pytest.approx(1.0, abs=0.05)
    assert result.rms_px < 0.5


def test_narrow_pan_span_fails(noisy_views: list) -> None:
    middle = float(np.median([v.joints["head_pan"] for v in noisy_views]))
    narrow = [v for v in noisy_views if abs(math.degrees(v.joints["head_pan"] - middle)) < 5.0]
    result = fit(narrow)
    assert result.pan_span_deg < 10.0
    assert "min_pan_span_deg" in result.failed_limits
    assert not result.passed


def test_floor_cross_check_limits(noisy_views: list) -> None:
    off = {**APPLIED_FLOOR, "tilt_zero_deg": expected(SYNTH_TRUTH_DELTAS)["tilt_zero_deg"] + FLOOR_OFFSET_DEG}
    result = fit(noisy_views, floor=off)
    assert "floor_delta:tilt_zero_deg" in result.failed_limits
    assert not result.passed
    blind = fit(noisy_views, floor=None)
    assert "floor_cross_check" in blind.failed_limits
    assert blind.floor_delta == {}
    assert not blind.passed


def test_floor_prior_weight_pulls(noisy_views: list, noisy_fit: HeadArmFit) -> None:
    off = {**APPLIED_FLOOR, "tilt_zero_deg": expected(SYNTH_TRUTH_DELTAS)["tilt_zero_deg"] + FLOOR_OFFSET_DEG}
    weighted = fit(noisy_views, cfg=synth_fit_config(floor_prior_weight=100.0), floor=off)
    free = abs(noisy_fit.params["tilt_zero_deg"] - off["tilt_zero_deg"])
    pulled = abs(weighted.params["tilt_zero_deg"] - off["tilt_zero_deg"])
    assert pulled < free - 0.5


def test_noisy_views_fail_a_sigma_limit() -> None:
    result = fit(scene(truth(SYNTH_TRUTH_DELTAS), 3.0))
    assert {"max_sigma_mm", "max_sigma_deg"} & set(result.failed_limits)
    assert not result.passed


def test_fixed_components_stay_at_urdf(noisy_fit: HeadArmFit) -> None:
    chain = UrdfChain.from_text(HEAD_URDF)
    pan_xyz, pan_rpy = chain.origin("head_pan")
    tilt_xyz, tilt_rpy = chain.origin("head_tilt")
    assert set(noisy_fit.origins) == {"head_mount_joint", "head_pan", "head_tilt"}
    pan = noisy_fit.origins["head_pan"]
    assert pan.xyz[:2] == pytest.approx((noisy_fit.params["pan_axis_x_m"], noisy_fit.params["pan_axis_y_m"]), abs=1e-12)
    assert pan.xyz[2] == pytest.approx(pan_xyz[2], abs=1e-12)
    assert pan.rpy == pytest.approx(pan_rpy, abs=1e-12)
    # The head stays put at pan 0: the head_tilt origin moves back by the pan axis shift (head_pan rpy is zero).
    shift = np.subtract(pan.xyz, pan_xyz)
    assert noisy_fit.origins["head_tilt"].xyz == pytest.approx(tuple(np.subtract(tilt_xyz, shift)), abs=1e-12)
    assert noisy_fit.origins["head_tilt"].rpy[2] == pytest.approx(tilt_rpy[2], abs=1e-12)
    values = [noisy_fit.params[n] if n.endswith("_m") else math.radians(noisy_fit.params[n]) for n in ALL_NAMES]
    model = head_origins(chain, np.array(values), ALL_NAMES)
    assert set(model) == {"head_mount_joint", "head_pan", "head_tilt"}
    for joint in model:
        assert model[joint].xyz == pytest.approx(noisy_fit.origins[joint].xyz, abs=1e-12), joint
        assert model[joint].rpy == pytest.approx(noisy_fit.origins[joint].rpy, abs=1e-12), joint
    # head_camera_joint and the optical joint are not overridden: the model uses the URDF for them.
    assert "head_camera_joint" not in model


def test_fixed_pan_axis_keeps_mount_and_tilt_only() -> None:
    cfg = synth_fit_config(fit_pan_axis=False)
    assert fit_param_names(cfg) == PARAM_NAMES
    result = fit(scene(truth(SYNTH_TRUTH_DELTAS), NOISE_PX), cfg=cfg)
    assert tuple(result.params) == PARAM_NAMES
    assert set(result.origins) == {"head_mount_joint", "head_tilt"}
    chain = UrdfChain.from_text(HEAD_URDF)
    assert result.origins["head_tilt"].xyz == pytest.approx(chain.origin("head_tilt")[0], abs=1e-12)
    assert result.passed, result.failed_limits


def test_pan_axis_parameters_move_the_axis_not_the_head() -> None:
    chain = UrdfChain.from_text(HEAD_URDF)
    start = start_params(chain, ALL_NAMES)
    shift = np.array([0.02, -0.015])
    values = [start[n] if n.endswith("_m") else math.radians(start[n]) for n in ALL_NAMES]
    values[len(PARAM_NAMES) :] = list(np.add(values[len(PARAM_NAMES) :], shift))
    moved = head_origins(chain, np.array(values), ALL_NAMES)
    still = {"head_pan": 0.0, "head_tilt": 0.2}
    np.testing.assert_allclose(camera_pose(chain, still, moved), camera_pose(chain, still), atol=1e-12)
    mount = moved["head_mount_joint"]
    pan_base = chain.transform("head_pan_base", {}, {"head_mount_joint": (mount.xyz, mount.rpy)})
    axis_point = pan_base @ np.append(moved["head_pan"].xyz, 1.0)
    axis_dir = pan_base[:3, :3] @ np.array([0.0, 0.0, 1.0])
    turn = 0.3
    rotation = cv2.Rodrigues(axis_dir * turn)[0]
    at_zero = camera_pose(chain, still, moved)
    turned = camera_pose(chain, {**still, "head_pan": turn}, moved)
    np.testing.assert_allclose(turned[:3, 3], axis_point[:3] + rotation @ (at_zero[:3, 3] - axis_point[:3]), atol=1e-12)
    np.testing.assert_allclose(turned[:3, :3], rotation @ at_zero[:3, :3], atol=1e-12)


def test_pan_axis_fit_recovers_moved_pan_axis(pan_axis_views: list) -> None:
    result = fit(pan_axis_views)
    want = expected({**SYNTH_TRUTH_DELTAS, "pan_axis_x_m": PAN_AXIS_SHIFT_M[0], "pan_axis_y_m": PAN_AXIS_SHIFT_M[1]})
    for name in ALL_NAMES:
        error = abs(result.params[name] - want[name])
        assert error <= tolerance(name), (name, result.params[name], want[name])
        assert error <= SIGMA_FACTOR * result.sigma[name], (name, error, result.sigma[name])
    assert result.rms_px < 0.5
    assert result.holdout_board_mm < 2.0
    assert all(value < 0.5 for value in result.offset_rms_px.values())
    assert abs(result.max_correlation.value) < synth_fit_config().limits.max_correlation
    assert result.passed, result.failed_limits


def test_fixed_pan_axis_fit_cannot_absorb_moved_pan_axis(pan_axis_views: list) -> None:
    result = fit(pan_axis_views, cfg=synth_fit_config(fit_pan_axis=False))
    assert result.rms_px > FIXED_PAN_MISFIT_PX
    by_offset = result.offset_rms_px
    # The +-10 deg pan offsets carry the misfit (as the 2026-10-23 session: 13 px against 10 px).
    assert min(by_offset[1], by_offset[2]) > by_offset[0]
    assert "rms_px" in result.failed_limits
    assert not result.passed


def test_max_correlation_limit(noisy_fit: HeadArmFit) -> None:
    cfg = synth_fit_config()
    names = [name for name, *_ in limit_checks(noisy_fit, cfg.limits)]
    assert "max_correlation" in names
    tight = cfg.limits.model_copy(update={"max_correlation": 0.1})
    failed = [name for name, _, _, ok in limit_checks(noisy_fit, tight) if not ok]
    assert failed == ["max_correlation"]


def test_pan_axis_delta_limit(noisy_fit: HeadArmFit) -> None:
    cfg = synth_fit_config()
    names = {name for name, *_ in limit_checks(noisy_fit, cfg.limits)}
    assert names >= {"start_delta:pan_axis_x_m", "start_delta:pan_axis_y_m"}
    shifted = noisy_fit.start_params["pan_axis_x_m"] - 0.06
    moved = replace(noisy_fit, params={**noisy_fit.params, "pan_axis_x_m": shifted})
    failed = [name for name, _, _, ok in limit_checks(moved, cfg.limits) if not ok]
    assert failed == ["start_delta:pan_axis_x_m"]


def test_aim_head() -> None:
    chain = UrdfChain.from_text(HEAD_URDF)
    for point in ([0.5, -0.2, 0.2], [0.4, 0.3, 0.45], [0.8, 0.0, 0.0]):
        target = np.array(point)
        pan, tilt = aim_head(chain, target)
        camera = chain.transform(CAMERA_FRAME, {"head_pan": pan, "head_tilt": tilt})
        local = np.linalg.inv(camera) @ np.append(target, 1.0)
        assert local[2] > 0
        assert math.atan2(math.hypot(local[0], local[1]), local[2]) < 1e-6, point
    pan_low, pan_high = chain.limits("head_pan")
    tilt_low, tilt_high = chain.limits("head_tilt")
    assert aim_head(chain, np.array([-0.5, 1.0, 0.3]))[0] == pytest.approx(pan_high)
    assert aim_head(chain, np.array([-0.5, -1.0, 0.3]))[0] == pytest.approx(pan_low)
    assert aim_head(chain, np.array([0.3, 0.1, -1.0]))[1] == pytest.approx(tilt_high)
    assert aim_head(chain, np.array([0.3, 0.1, 2.0]))[1] == pytest.approx(tilt_low)


def test_report_names_every_parameter(noisy_fit: HeadArmFit) -> None:
    text = head_arm_report_markdown(noisy_fit, synth_fit_config(), "arm-20261022-120000", "2026-10-22")
    for name in ALL_NAMES:
        assert name in text
    assert "pan zero folded into mount_yaw_deg" in text
    assert "## Residuals per head offset" in text
    assert "| 1 |" in text
    assert "holdout_board_mm" in text
    assert "PASS" in text


def test_synthetic_start_chain_is_applied_floor_fit() -> None:
    start = start_params(UrdfChain.from_text(HEAD_URDF))
    raw = start_params(UrdfChain.from_text((URDF_DIR / "head.urdf").read_text()))
    for name, value in APPLIED_FLOOR.items():
        assert start[name] == pytest.approx(value, abs=1e-6), name  # URDF text precision
    for name in ("mount_x_m", "mount_y_m", "mount_yaw_deg"):
        assert start[name] == pytest.approx(raw[name], abs=1e-12), name


def test_synth_arm_poses_cover_joints_above_floor() -> None:
    cfg = HeadArmFitConfig()
    arm = UrdfChain.from_text(ARM_URDF)
    base = arm_in_robot_base(cfg.base_in_base_link.model_dump())
    assert len(SYNTH_ARM_POSES) >= 24
    for joint in ("shoulder_pan", "elbow_flex", "wrist_flex", "wrist_roll"):
        assert len({pose[joint] for pose in SYNTH_ARM_POSES}) >= 3, joint
    for pose in SYNTH_ARM_POSES:
        tool = base @ arm.transform("gripper_link", pose, offsets=cfg.joint_offsets_rad) @ np.append(TOOL_POINT, 1.0)
        assert tool[2] > 0.0, pose


def test_view_missing_a_joint_raises(noisy_views: list) -> None:
    broken = replace(noisy_views[1], joints={k: v for k, v in noisy_views[1].joints.items() if k != "wrist_flex"})
    with pytest.raises(ValueError, match="wrist_flex"):
        fit([noisy_views[0], broken, *noisy_views[2:]])


def test_fewer_than_three_views_raise(noisy_views: list) -> None:
    with pytest.raises(ValueError, match="3 views"):
        fit(noisy_views[:2])
