"""Tests for the floor-plane head chain fit in shared/ros2_common/head_rgbd_calibration/floor_fit.py (design D2-D4)."""

import json
from pathlib import Path

import numpy as np
import pytest
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration import floor_fit
from ros2_common.head_rgbd_calibration.chain import UrdfChain
from ros2_common.head_rgbd_calibration.config import FloorFitConfig, FloorFitLimits, FloorPlaneConfig
from ros2_common.head_rgbd_calibration.floor_fit import (
    FloorObservation,
    PlaneFit,
    fit_head_chain_floor,
    fit_plane,
    predicted_floor,
)
from ros2_common.head_rgbd_calibration.urdf_text import Origin

from tests.head_floor_synthetic import synthetic_clouds

URDF_PATH = Path(__file__).resolve().parents[1] / "nodes" / "web_ui" / "urdf" / "head.urdf"
MOUNT = "head_mount_joint"
TILT = "head_tilt"
TRUTH_DZ_M = 0.03
TRUTH_ROLL_DEG = 1.5
TRUTH_PITCH_DEG = -3.0
TRUTH_TILT_ZERO_DEG = 2.0
GRID_PANS_DEG = (-60, -30, 0, 30, 60)
GRID_TILTS_DEG = (12, 20, 28)
GRID = [(np.radians(p), np.radians(t)) for p in GRID_PANS_DEG for t in GRID_TILTS_DEG]
NOISE_M = 0.005
OUTLIER_RATIO = 0.2


@pytest.fixture(scope="module")
def urdf_text() -> str:
    return URDF_PATH.read_text()


@pytest.fixture(scope="module")
def chain(urdf_text: str) -> UrdfChain:
    return UrdfChain.from_text(urdf_text)


@pytest.fixture(scope="module")
def truth(chain: UrdfChain) -> dict[str, Origin]:
    (mx, my, mz), (mr, mp, myaw) = chain.origin(MOUNT)
    txyz, (tr, tp, tyaw) = chain.origin(TILT)
    return {
        MOUNT: Origin(
            (mx, my, mz + TRUTH_DZ_M), (mr + np.radians(TRUTH_ROLL_DEG), mp + np.radians(TRUTH_PITCH_DEG), myaw)
        ),
        TILT: Origin(txyz, (tr, tp + np.radians(TRUTH_TILT_ZERO_DEG), tyaw)),
    }


def angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b), -1.0, 1.0))))


def exact_observations(
    chain: UrdfChain,
    origins: dict[str, Origin],
    grid: list[tuple[float, float]],
    per_pose: int,
    rng: np.random.Generator,
    height_noise_m: float = 0.0,
    height_bias_m: float = 0.0,
) -> list[FloorObservation]:
    """Observations whose planes are the truth floor (plus optional height noise and bias), no clouds."""
    out = []
    for pose, (pan, tilt) in enumerate(grid):
        normal, height = predicted_floor(chain, pan, tilt, origins)
        for _ in range(per_pose):
            h = height + height_bias_m + rng.normal(0.0, height_noise_m)
            out.append(FloorObservation(pose, pan, tilt, PlaneFit(normal, h, 1000, 1000, 0.0)))
    return out


def test_floor_fit_config_defaults_and_ranges() -> None:
    cfg = FloorFitConfig()
    assert cfg.plane == FloorPlaneConfig()
    assert (cfg.plane.ransac_distance_m, cfg.plane.ransac_iterations, cfg.plane.min_inliers) == (0.015, 200, 300)
    assert (cfg.plane.min_inlier_ratio, cfg.plane.max_range_m, cfg.plane.candidate_band_m) == (0.5, 1.8, 0.25)
    assert cfg.plane.seed == 0
    assert cfg.limits == FloorFitLimits()
    assert (cfg.limits.min_observations, cfg.limits.min_poses, cfg.limits.min_pan_span_deg) == (30, 8, 60.0)
    assert (cfg.limits.max_height_rms_mm, cfg.limits.max_normal_rms_deg) == (8.0, 0.6)
    assert (cfg.limits.max_sigma_z_mm, cfg.limits.max_sigma_deg) == (3.0, 0.3)
    assert (cfg.limits.max_delta_z_m, cfg.limits.max_delta_deg) == (0.10, 10.0)
    assert (cfg.sigma_height_mm, cfg.sigma_normal_deg, cfg.huber_scale) == (5.0, 0.5, 1.0)
    for bad in (
        {"ransac_distance_m": 0.0},
        {"ransac_distance_m": 0.2},
        {"ransac_iterations": 9},
        {"min_inliers": 9},
        {"min_inlier_ratio": 0.0},
        {"min_inlier_ratio": 1.5},
        {"max_range_m": 0.0},
        {"candidate_band_m": 0.01},
        {"unknown": 1},
    ):
        with pytest.raises(ValidationError):
            FloorPlaneConfig(**bad)
    for name in FloorFitLimits.model_fields:
        with pytest.raises(ValidationError):
            FloorFitLimits(**{name: 0})
    for name in ("sigma_height_mm", "sigma_normal_deg", "huber_scale"):
        with pytest.raises(ValidationError):
            FloorFitConfig(**{name: 0})
    with pytest.raises(ValidationError):
        FloorFitConfig(limits={"bogus": 1})


def test_predicted_floor_at_zero_is_035_m_below_along_optical_minus_y(chain: UrdfChain) -> None:
    normal, height = predicted_floor(chain, 0.0, 0.0, None)
    assert height == pytest.approx(0.35, abs=1e-9)
    np.testing.assert_allclose(normal, [0.0, -1.0, 0.0], atol=1e-4)
    assert np.linalg.norm(normal) == pytest.approx(1.0)


def test_predicted_floor_tilt_down_turns_floor_toward_view(chain: UrdfChain) -> None:
    normal, _ = predicted_floor(chain, 0.0, np.radians(20.0), None)
    # Looking down, the floor normal (toward the sensor) gains a -z (toward the camera) optical component.
    assert normal[2] < -0.3
    origins = {MOUNT: Origin((0.15, 0.10, 0.41), (0.0, 0.0, 0.0))}
    _, raised = predicted_floor(chain, 0.0, 0.0, origins)
    assert raised == pytest.approx(0.45, abs=1e-9)
    _, raised_tuple = predicted_floor(chain, 0.0, 0.0, {MOUNT: ((0.15, 0.10, 0.41), (0.0, 0.0, 0.0))})
    assert raised_tuple == pytest.approx(0.45, abs=1e-9)


def test_fit_plane_recovers_known_plane_with_noise_and_outliers(
    urdf_text: str, chain: UrdfChain, truth: dict[str, Origin]
) -> None:
    cfg = FloorPlaneConfig()
    rng = np.random.default_rng(1)
    grid = [(np.radians(-40.0), np.radians(12.0)), (0.0, np.radians(20.0)), (np.radians(50.0), np.radians(28.0))]
    clouds = synthetic_clouds(urdf_text, truth, grid, NOISE_M, OUTLIER_RATIO, rng)
    for cloud in clouds:
        true_n, true_h = predicted_floor(chain, cloud.pan_rad, cloud.tilt_rad, truth)
        prior = predicted_floor(chain, cloud.pan_rad, cloud.tilt_rad, None)
        fit = fit_plane(cloud.points, prior, cfg, rng)
        assert fit is not None
        assert abs(fit.height_m - true_h) < 0.002
        assert angle_deg(fit.normal, true_n) < 0.1
        assert np.linalg.norm(fit.normal) == pytest.approx(1.0)
        assert cfg.min_inliers <= fit.inliers < fit.candidates <= len(cloud.points)
        assert fit.rms_m == pytest.approx(NOISE_M, rel=0.3)


def test_fit_plane_none_for_too_few_floor_points(urdf_text: str, chain: UrdfChain, truth: dict[str, Origin]) -> None:
    cfg = FloorPlaneConfig()
    rng = np.random.default_rng(2)
    pan, tilt = 0.0, np.radians(20.0)
    (cloud,) = synthetic_clouds(urdf_text, truth, [(pan, tilt)], NOISE_M, 0.0, rng)
    prior = predicted_floor(chain, pan, tilt, None)
    assert fit_plane(cloud.points[:200], prior, cfg, rng) is None
    nan_padded = np.concatenate([cloud.points[:200], np.full((5000, 3), np.nan)])
    assert fit_plane(nan_padded, prior, cfg, rng) is None
    far = cloud.points * 10.0
    assert fit_plane(far, prior, cfg, rng) is None
    # Enough candidates but mostly scattered: the inlier ratio is below min_inlier_ratio.
    n, h = prior
    scatter = rng.uniform(-1.0, 1.0, (4000, 3))
    scatter = scatter - ((scatter @ n + h) - rng.uniform(-0.2, 0.2, 4000))[:, None] * n
    mixed = np.concatenate([cloud.points[:600], scatter])
    assert fit_plane(mixed, prior, cfg, rng) is None


def test_fit_recovers_mount_and_tilt_zero_from_15_pose_grid(
    urdf_text: str, chain: UrdfChain, truth: dict[str, Origin]
) -> None:
    cfg = FloorFitConfig()
    rng = np.random.default_rng(cfg.plane.seed)
    clouds = synthetic_clouds(urdf_text, truth, GRID, NOISE_M, OUTLIER_RATIO, rng, clouds_per_pose=5)
    observations = []
    for cloud in clouds:
        plane = fit_plane(cloud.points, predicted_floor(chain, cloud.pan_rad, cloud.tilt_rad, None), cfg.plane, rng)
        assert plane is not None
        observations.append(FloorObservation(cloud.pose, cloud.pan_rad, cloud.tilt_rad, plane))
    result = fit_head_chain_floor(observations, urdf_text, cfg)

    assert result.passed, result.failed_limits
    assert result.failed_limits == []
    (mx, my, mz), (mr, mp, myaw) = chain.origin(MOUNT)
    txyz, (tr, tp, tyaw) = chain.origin(TILT)
    p = result.params
    assert abs(p["mount_z_m"]["value"] - (mz + TRUTH_DZ_M)) < 0.002
    assert abs(p["mount_roll_deg"]["value"] - (np.degrees(mr) + TRUTH_ROLL_DEG)) < 0.15
    assert abs(p["mount_pitch_deg"]["value"] - (np.degrees(mp) + TRUTH_PITCH_DEG)) < 0.15
    assert abs(p["tilt_zero_deg"]["value"] - (np.degrees(tp) + TRUTH_TILT_ZERO_DEG)) < 0.15
    assert p["mount_z_m"]["urdf"] == mz
    assert p["tilt_zero_deg"]["urdf"] == np.degrees(tp)
    for entry in p.values():
        assert 0.0 <= entry["sigma"] < 0.3

    mount, tilt = result.origins[MOUNT], result.origins[TILT]
    assert set(result.origins) == {MOUNT, TILT}
    assert (mount.xyz[0], mount.xyz[1], mount.rpy[2]) == (mx, my, myaw)
    assert tilt.xyz == txyz
    assert (tilt.rpy[0], tilt.rpy[2]) == (tr, tyaw)
    assert mount.xyz[2] == p["mount_z_m"]["value"]
    assert tilt.rpy[1] == pytest.approx(np.radians(p["tilt_zero_deg"]["value"]))

    assert result.height_rms_mm < 1.0
    assert result.normal_rms_deg < 0.1
    assert len(result.poses) == len(GRID)
    first = result.poses[0]
    assert first["pan_deg"] == pytest.approx(GRID_PANS_DEG[0])
    assert first["tilt_deg"] == pytest.approx(GRID_TILTS_DEG[0])
    assert first["observations"] == 5
    assert first["height_residual_mm"] >= 0.0 and first["normal_residual_deg"] >= 0.0

    data = result.to_json()
    text = json.dumps(data, allow_nan=False)
    assert json.loads(text) == data
    assert data["passed"] is True
    assert data["origins"][MOUNT]["xyz"] == list(mount.xyz)
    assert data["origins"][TILT]["rpy"] == list(tilt.rpy)
    assert set(data) >= {"passed", "failed_limits", "params", "height_rms_mm", "normal_rms_deg", "poses", "origins"}


def test_pan_zero_only_grid_fails_pan_span(urdf_text: str, chain: UrdfChain, truth: dict[str, Origin]) -> None:
    rng = np.random.default_rng(3)
    grid = [(0.0, np.radians(t)) for t in range(10, 30, 2)]
    observations = exact_observations(chain, truth, grid, 4, rng, height_noise_m=0.0005)
    result = fit_head_chain_floor(observations, urdf_text, FloorFitConfig())
    assert not result.passed
    assert "min_pan_span_deg" in result.failed_limits
    assert "min_poses" not in result.failed_limits
    assert "min_observations" not in result.failed_limits
    assert result.to_json()["passed"] is False
    json.dumps(result.to_json(), allow_nan=False)


def test_too_few_observations_fails_without_solver(
    urdf_text: str, chain: UrdfChain, truth: dict[str, Origin], monkeypatch: pytest.MonkeyPatch
) -> None:
    def no_solver(*args: object, **kwargs: object) -> None:
        raise AssertionError("solver called")

    monkeypatch.setattr(floor_fit, "least_squares", no_solver)
    observations = exact_observations(chain, truth, GRID, 1, np.random.default_rng(4))
    result = fit_head_chain_floor(observations, urdf_text, FloorFitConfig())
    assert not result.passed
    assert result.failed_limits == ["min_observations"]
    for entry in result.params.values():
        assert entry["value"] == entry["urdf"]
    assert result.origins[MOUNT] == Origin(*chain.origin(MOUNT))
    json.dumps(result.to_json(), allow_nan=False)

    empty = fit_head_chain_floor([], urdf_text, FloorFitConfig())
    assert not empty.passed
    assert {"min_observations", "min_poses", "min_pan_span_deg"} <= set(empty.failed_limits)
    assert empty.poses == []
    json.dumps(empty.to_json(), allow_nan=False)


def test_height_bias_fails_delta_limit(urdf_text: str, chain: UrdfChain) -> None:
    observations = exact_observations(chain, {}, GRID, 3, np.random.default_rng(5), height_bias_m=0.2)
    result = fit_head_chain_floor(observations, urdf_text, FloorFitConfig())
    assert not result.passed
    assert "max_delta_z_m" in result.failed_limits
    assert result.params["mount_z_m"]["value"] - result.params["mount_z_m"]["urdf"] == pytest.approx(0.2, abs=0.005)


def test_30mm_noise_fails_rms_limit(urdf_text: str, chain: UrdfChain, truth: dict[str, Origin]) -> None:
    observations = exact_observations(chain, truth, GRID, 3, np.random.default_rng(6), height_noise_m=0.03)
    result = fit_head_chain_floor(observations, urdf_text, FloorFitConfig())
    assert not result.passed
    assert "max_height_rms_mm" in result.failed_limits
    assert result.height_rms_mm > 8.0
