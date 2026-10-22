"""Tests of the head-from-arm fit (shared/ros2_common/head_rgbd_calibration/head_arm_fit.py) on synthetic scenes.

Every scene uses nodes/web_ui/urdf/head.urdf with the applied floor-fit chain of 2026-10-22 as overrides (the start
chain) and nodes/web_ui/urdf/so101_arm.urdf; the truth chain is the start chain moved by known amounts.
"""

import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.chain import UrdfChain, pose_from_xyz_rpy
from ros2_common.head_rgbd_calibration.config import BoardMountConfig, HeadArmFitConfig
from ros2_common.head_rgbd_calibration.head_arm_fit import (
    CAMERA_FRAME,
    PARAM_NAMES,
    HeadArmFit,
    aim_head,
    fit_head_from_arm,
    head_arm_report_markdown,
    head_origins,
    start_params,
)
from ros2_common.head_rgbd_calibration.urdf_text import Origin, rewrite_joint_origin

from tests.head_arm_synthetic import (
    DEFAULT_HEAD_OFFSETS_DEG,
    SYNTH_ARM_POSES,
    SYNTH_CAMERA_MATRIX,
    SYNTH_DIST,
    HeadArmTruth,
    arm_board_scene,
    nominal_board_mount,
    perturbed_board_mount,
    perturbed_origins,
)

URDF_DIR = Path(__file__).resolve().parents[1] / "nodes" / "web_ui" / "urdf"
APPLIED_MOUNT = Origin((0.15, 0.10, 0.246), (math.radians(-0.32), math.radians(1.50), 0.0))
APPLIED_TILT = Origin((0.0, 0.0, 0.04), (math.radians(0.92), math.radians(-6.50), 0.0))
APPLIED_FLOOR = {
    "mount_z_m": 0.246,
    "mount_roll_deg": -0.32,
    "mount_pitch_deg": 1.50,
    "tilt_zero_deg": -6.50,
    "tilt_roll_deg": 0.92,
}
TRUTH_DELTAS = {
    "mount_x_m": 0.008,
    "mount_y_m": -0.006,
    "mount_z_m": 0.005,
    "mount_roll_deg": 0.4,
    "mount_pitch_deg": -0.5,
    "mount_yaw_deg": 1.5,
    "tilt_zero_deg": 0.6,
    "tilt_roll_deg": -0.4,
}
NOISE_PX = 0.3
SEED = 7
TOLERANCE_MM = 1.5
TOLERANCE_DEG = 0.15
SIGMA_FACTOR = 3.0
FLOOR_OFFSET_DEG = 2.0
MOUNT_INIT_SHIFT_M = 0.010
MOUNT_INIT_TURN_DEG = 8.0


def applied_head_urdf() -> str:
    """head.urdf with the applied floor-fit chain of 2026-10-22 written into its origins."""
    text = (URDF_DIR / "head.urdf").read_text()
    text = rewrite_joint_origin(text, "head_mount_joint", APPLIED_MOUNT, "floor fit 2026-10-22")
    return rewrite_joint_origin(text, "head_tilt", APPLIED_TILT, "floor fit 2026-10-22")


HEAD_URDF = applied_head_urdf()
ARM_URDF = (URDF_DIR / "so101_arm.urdf").read_text()


def fit_config(**overrides: object) -> HeadArmFitConfig:
    """Fit config whose board mount start is 10 mm / 8 deg off the truth mount."""
    xyz, rpy = perturbed_board_mount(*nominal_board_mount(HeadArmFitConfig()), MOUNT_INIT_SHIFT_M, MOUNT_INIT_TURN_DEG)
    return HeadArmFitConfig(board_mount=BoardMountConfig(xyz_init=xyz, rpy_init=rpy), **overrides)


def truth(deltas: dict[str, float], extra: dict[str, Origin] | None = None) -> HeadArmTruth:
    """Truth: the start chain moved by `deltas`, plus `extra` origins, and the nominal board mount."""
    xyz, rpy = nominal_board_mount(HeadArmFitConfig())
    return HeadArmTruth(origins={**perturbed_origins(HEAD_URDF, deltas), **(extra or {})}, board_xyz=xyz, board_rpy=rpy)


def scene(t: HeadArmTruth, noise_px: float, cfg: HeadArmFitConfig | None = None) -> list:
    return arm_board_scene(
        t, HEAD_URDF, ARM_URDF, SYNTH_ARM_POSES, DEFAULT_HEAD_OFFSETS_DEG, noise_px, SEED, cfg or fit_config()
    )


def fit(views: list, cfg: HeadArmFitConfig | None = None, floor: dict[str, float] | None = APPLIED_FLOOR) -> HeadArmFit:
    return fit_head_from_arm(views, SYNTH_CAMERA_MATRIX, SYNTH_DIST, HEAD_URDF, ARM_URDF, cfg or fit_config(), floor)


def expected(deltas: dict[str, float]) -> dict[str, float]:
    """Parameter values (m, deg) of the start chain moved by `deltas`."""
    start = start_params(UrdfChain.from_text(HEAD_URDF))
    return {name: start[name] + deltas.get(name, 0.0) for name in PARAM_NAMES}


@pytest.fixture(scope="module")
def noisy_views() -> list:
    return scene(truth(TRUTH_DELTAS), NOISE_PX)


@pytest.fixture(scope="module")
def noisy_fit(noisy_views: list) -> HeadArmFit:
    return fit(noisy_views)


def tolerance(name: str) -> float:
    return TOLERANCE_MM / 1000.0 if name.endswith("_m") else TOLERANCE_DEG


def test_recovers_perturbed_head_chain_with_noise(noisy_fit: HeadArmFit) -> None:
    want = expected(TRUTH_DELTAS)
    for name in PARAM_NAMES:
        error = abs(noisy_fit.params[name] - want[name])
        assert error <= tolerance(name), (name, noisy_fit.params[name], want[name])
        assert error <= SIGMA_FACTOR * noisy_fit.sigma[name], (name, error, noisy_fit.sigma[name])
    assert noisy_fit.rms_px < 0.5
    assert noisy_fit.holdout_rms_px < 0.6
    assert noisy_fit.holdout_board_mm < 2.0
    assert noisy_fit.failed_limits == []
    assert noisy_fit.passed


def test_fit_reports_coverage_and_holdout(noisy_fit: HeadArmFit, noisy_views: list) -> None:
    cfg = fit_config()
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
    assert len(pair) == 2 and set(pair) <= set(PARAM_NAMES)
    assert abs(value) < 0.999  # no free parameter is degenerate with another (design D32)
    data = json.loads(json.dumps(noisy_fit.to_json(), allow_nan=False))
    assert data["passed"] is True
    assert set(data["origins"]) == {"head_mount_joint", "head_tilt"}
    assert set(data["per_view"][0]) == {"view_id", "arm_pose", "rms_px", "held_out"}


def test_noiseless_recovery_exact() -> None:
    result = fit(scene(truth(TRUTH_DELTAS), 0.0))
    want = expected(TRUTH_DELTAS)
    for name in PARAM_NAMES:
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
    off = {**APPLIED_FLOOR, "tilt_zero_deg": expected(TRUTH_DELTAS)["tilt_zero_deg"] + FLOOR_OFFSET_DEG}
    result = fit(noisy_views, floor=off)
    assert "floor_delta:tilt_zero_deg" in result.failed_limits
    assert not result.passed
    blind = fit(noisy_views, floor=None)
    assert "floor_cross_check" in blind.failed_limits
    assert blind.floor_delta == {}
    assert not blind.passed


def test_floor_prior_weight_pulls(noisy_views: list, noisy_fit: HeadArmFit) -> None:
    off = {**APPLIED_FLOOR, "tilt_zero_deg": expected(TRUTH_DELTAS)["tilt_zero_deg"] + FLOOR_OFFSET_DEG}
    weighted = fit(noisy_views, cfg=fit_config(floor_prior_weight=100.0), floor=off)
    free = abs(noisy_fit.params["tilt_zero_deg"] - off["tilt_zero_deg"])
    pulled = abs(weighted.params["tilt_zero_deg"] - off["tilt_zero_deg"])
    assert pulled < free - 0.5


def test_noisy_views_fail_a_sigma_limit() -> None:
    result = fit(scene(truth(TRUTH_DELTAS), 3.0))
    assert {"max_sigma_mm", "max_sigma_deg"} & set(result.failed_limits)
    assert not result.passed


def test_fixed_components_stay_at_urdf(noisy_fit: HeadArmFit) -> None:
    chain = UrdfChain.from_text(HEAD_URDF)
    assert set(noisy_fit.origins) == {"head_mount_joint", "head_tilt"}
    assert noisy_fit.origins["head_tilt"].xyz == pytest.approx(chain.origin("head_tilt")[0], abs=1e-12)
    assert noisy_fit.origins["head_tilt"].rpy[2] == pytest.approx(chain.origin("head_tilt")[1][2], abs=1e-12)
    values = [noisy_fit.params[n] if n.endswith("_m") else math.radians(noisy_fit.params[n]) for n in PARAM_NAMES]
    model = head_origins(chain, np.array(values))
    assert set(model) == {"head_mount_joint", "head_tilt"}
    assert model["head_mount_joint"].xyz == pytest.approx(noisy_fit.origins["head_mount_joint"].xyz, abs=1e-12)
    assert model["head_mount_joint"].rpy == pytest.approx(noisy_fit.origins["head_mount_joint"].rpy, abs=1e-12)
    assert model["head_tilt"].rpy == pytest.approx(noisy_fit.origins["head_tilt"].rpy, abs=1e-12)
    # head_pan, head_camera_joint and the optical joint are not overridden: the model uses the URDF for them.
    assert "head_pan" not in model and "head_camera_joint" not in model


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
    text = head_arm_report_markdown(noisy_fit, fit_config(), "arm-20261022-120000", "2026-10-22")
    for name in PARAM_NAMES:
        assert name in text
    assert "pan zero folded into mount_yaw_deg" in text
    assert "holdout_board_mm" in text
    assert "PASS" in text


def test_view_missing_a_joint_raises(noisy_views: list) -> None:
    broken = replace(noisy_views[1], joints={k: v for k, v in noisy_views[1].joints.items() if k != "wrist_flex"})
    with pytest.raises(ValueError, match="wrist_flex"):
        fit([noisy_views[0], broken, *noisy_views[2:]])


def test_fewer_than_three_views_raise(noisy_views: list) -> None:
    with pytest.raises(ValueError, match="3 views"):
        fit(noisy_views[:2])
