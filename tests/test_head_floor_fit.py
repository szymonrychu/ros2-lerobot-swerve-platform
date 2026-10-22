"""Tests for the floor-plane head chain fit in shared/ros2_common/head_rgbd_calibration/floor_fit.py (design D2-D4)."""

import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType

import numpy as np
import pytest
import yaml
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
from scipy.spatial.transform import Rotation

from tests.head_floor_synthetic import synthetic_clouds

REPO_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "head.urdf"
NODE_DIR = REPO_ROOT / "nodes" / "head_rgbd_calibration"
NODE_CONFIG_PY = NODE_DIR / "head_rgbd_calibration" / "config.py"
NODE_CONFIG_YAML = NODE_DIR / "config" / "head_rgbd_calibration.yaml"
NODE_CONFIG_MODULE = "head_rgbd_calibration_node_config"
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
TRUTH_TILT_ROLL_DEG = 1.2
CLOUD_SCATTER_DEG = 1.2
POSE_BIAS_DEG = 1.5
CLOUD_HEIGHT_SCATTER_M = 0.015
POSE_HEIGHT_BIAS_M = 0.012


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


def node_config() -> ModuleType:
    """The head_rgbd_calibration node config module, loaded from its file (it needs yaml, pydantic, ros2_common)."""
    if NODE_CONFIG_MODULE not in sys.modules:
        spec = importlib.util.spec_from_file_location(NODE_CONFIG_MODULE, NODE_CONFIG_PY)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[NODE_CONFIG_MODULE] = module
        spec.loader.exec_module(module)
    return sys.modules[NODE_CONFIG_MODULE]


def tilted(normal: np.ndarray, axis: np.ndarray, angle_deg_: float) -> np.ndarray:
    """`normal` turned by `angle_deg_` about the unit `axis` (orthogonal to it)."""
    return Rotation.from_rotvec(np.radians(angle_deg_) * axis).apply(normal)


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
    assert (cfg.plane.ransac_distance_m, cfg.plane.ransac_iterations, cfg.plane.min_inliers) == (0.03, 200, 150)
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


def test_predicted_floor_at_zero_matches_urdf(chain: UrdfChain) -> None:
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


def test_fit_plane_recovers_plane_and_refuses_sparse_cloud(
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

    # Too few floor points: refuse rather than make up a plane.
    rng = np.random.default_rng(2)
    pan, tilt = 0.0, np.radians(20.0)
    (cloud,) = synthetic_clouds(urdf_text, truth, [(pan, tilt)], NOISE_M, 0.0, rng)
    prior = predicted_floor(chain, pan, tilt, None)
    assert fit_plane(cloud.points[:100], prior, cfg, rng) is None
    nan_padded = np.concatenate([cloud.points[:100], np.full((5000, 3), np.nan)])
    assert fit_plane(nan_padded, prior, cfg, rng) is None
    far = cloud.points * 10.0
    assert fit_plane(far, prior, cfg, rng) is None
    # Enough candidates but mostly scattered: the inlier ratio is below min_inlier_ratio.
    n, h = prior
    scatter = rng.uniform(-1.0, 1.0, (4000, 3))
    scatter = scatter - ((scatter @ n + h) - rng.uniform(-0.2, 0.2, 4000))[:, None] * n
    mixed = np.concatenate([cloud.points[:600], scatter])
    assert fit_plane(mixed, prior, cfg, rng) is None


def test_fit_recovers_perturbed_chain(urdf_text: str, chain: UrdfChain, truth: dict[str, Origin]) -> None:
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
    assert tilt.rpy[2] == tyaw
    assert tilt.rpy[0] == pytest.approx(np.radians(p["tilt_roll_deg"]["value"]))
    assert abs(p["tilt_roll_deg"]["value"] - np.degrees(tr)) < 0.15
    assert mount.xyz[2] == p["mount_z_m"]["value"]
    assert tilt.rpy[1] == pytest.approx(np.radians(p["tilt_zero_deg"]["value"]))

    assert result.height_rms_mm < 1.0
    assert result.normal_rms_deg < 0.1
    assert len(result.poses) == len(GRID)
    first = result.poses[0]
    assert first["pan_deg"] == pytest.approx(GRID_PANS_DEG[0])
    assert first["tilt_deg"] == pytest.approx(GRID_TILTS_DEG[0])
    assert first["observations"] == 5
    assert abs(first["height_residual_mm"]) < 1.0 and 0.0 <= first["normal_residual_deg"] < 0.1
    assert first["observation_height_rms_mm"] >= abs(first["height_residual_mm"])
    assert first["observation_normal_rms_deg"] >= 0.0
    assert result.observation_height_rms_mm >= result.height_rms_mm

    data = result.to_json()
    text = json.dumps(data, allow_nan=False)
    assert json.loads(text) == data
    assert data["passed"] is True
    assert data["origins"][MOUNT]["xyz"] == list(mount.xyz)
    assert data["origins"][TILT]["rpy"] == list(tilt.rpy)
    assert set(data) >= {"passed", "failed_limits", "params", "height_rms_mm", "normal_rms_deg", "poses", "origins"}


def test_pan_span_limit_guards_degenerate_grid(urdf_text: str, chain: UrdfChain, truth: dict[str, Origin]) -> None:
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


def test_limits_refuse_bad_results(
    urdf_text: str, chain: UrdfChain, truth: dict[str, Origin], monkeypatch: pytest.MonkeyPatch
) -> None:
    def no_solver(*args: object, **kwargs: object) -> None:
        raise AssertionError("solver called")

    # Too few observations: named, and the solver is never called.
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
    monkeypatch.undo()

    # A 0.2 m height bias exceeds the delta-from-URDF limit.
    observations = exact_observations(chain, {}, GRID, 3, np.random.default_rng(5), height_bias_m=0.2)
    result = fit_head_chain_floor(observations, urdf_text, FloorFitConfig())
    assert not result.passed
    assert "max_delta_z_m" in result.failed_limits
    assert result.params["mount_z_m"]["value"] - result.params["mount_z_m"]["urdf"] == pytest.approx(0.2, abs=0.005)

    # 30 mm height noise exceeds the height RMS limit.
    observations = exact_observations(chain, truth, GRID, 3, np.random.default_rng(6), height_noise_m=0.03)
    result = fit_head_chain_floor(observations, urdf_text, FloorFitConfig())
    assert not result.passed
    assert "max_height_rms_mm" in result.failed_limits
    assert result.height_rms_mm > 8.0


def test_fit_plane_defaults_fit_a_noisy_tof_cloud() -> None:
    """Real ToF clouds (~700 points, 2-3 cm floor noise) fit with the defaults; the old 1.5 cm gate returned None."""
    rng = np.random.default_rng(7)
    normal = np.array([0.0, -np.sin(np.radians(25.0)), -np.cos(np.radians(25.0))])
    normal = normal / np.linalg.norm(normal)
    height_m = 0.42
    u = np.cross(normal, [1.0, 0.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    floor_count, outlier_count = 600, 100
    plane_xy = rng.uniform([-0.8, 0.3], [0.8, 1.4], (floor_count, 2))
    floor = plane_xy[:, :1] * u + plane_xy[:, 1:] * v - height_m * normal
    floor = floor + rng.normal(0.0, 0.02, floor_count)[:, None] * normal
    outliers = rng.uniform(-1.0, 1.0, (outlier_count, 3))
    cloud = np.concatenate([floor, outliers])
    prior = (normal, height_m)

    fit = fit_plane(cloud, prior, FloorPlaneConfig(), rng)

    assert fit is not None
    assert abs(fit.height_m - height_m) < 0.01
    assert angle_deg(fit.normal, normal) < 2.0
    old = FloorPlaneConfig(ransac_distance_m=0.015, min_inliers=300)
    assert fit_plane(cloud, prior, old, np.random.default_rng(7)) is None


def test_tilt_roll_is_recovered(urdf_text: str, chain: UrdfChain, truth: dict[str, Origin]) -> None:
    cfg = FloorFitConfig()
    txyz, (tr, tp, tyaw) = chain.origin(TILT)
    rolled = {**truth, TILT: Origin(txyz, (tr + np.radians(TRUTH_TILT_ROLL_DEG), truth[TILT].rpy[1], tyaw))}
    rng = np.random.default_rng(11)
    clouds = synthetic_clouds(urdf_text, rolled, GRID, NOISE_M, OUTLIER_RATIO, rng, clouds_per_pose=5)
    observations = []
    for cloud in clouds:
        plane = fit_plane(cloud.points, predicted_floor(chain, cloud.pan_rad, cloud.tilt_rad, None), cfg.plane, rng)
        assert plane is not None
        observations.append(FloorObservation(cloud.pose, cloud.pan_rad, cloud.tilt_rad, plane))

    result = fit_head_chain_floor(observations, urdf_text, cfg)

    assert result.passed, result.failed_limits
    roll = result.params["tilt_roll_deg"]
    assert roll["urdf"] == np.degrees(tr)
    assert 0.0 < roll["sigma"] <= cfg.limits.max_sigma_deg
    assert abs(roll["value"] - (np.degrees(tr) + TRUTH_TILT_ROLL_DEG)) <= max(3.0 * roll["sigma"], 0.05)
    assert result.origins[TILT].rpy[0] == pytest.approx(np.radians(roll["value"]))
    assert result.origins[TILT].rpy[1] == pytest.approx(np.radians(result.params["tilt_zero_deg"]["value"]))
    p = result.params
    assert abs(p["mount_roll_deg"]["value"] - (np.degrees(chain.origin(MOUNT)[1][0]) + TRUTH_ROLL_DEG)) < 0.15
    assert abs(p["tilt_zero_deg"]["value"] - (np.degrees(tp) + TRUTH_TILT_ZERO_DEG)) < 0.15
    assert result.to_json()["params"]["tilt_roll_deg"]["value"] == pytest.approx(roll["value"])

    # The sigma limit covers the tilt roll: a grid that cannot separate it from the mount roll is refused.
    tight = FloorFitConfig(limits={"max_sigma_deg": roll["sigma"] / 2.0})
    refused = fit_head_chain_floor(observations, urdf_text, tight)
    assert "max_sigma_deg" in refused.failed_limits


def scattered_observations(
    chain: UrdfChain,
    origins: dict[str, Origin],
    per_pose: int,
    cloud_scatter_deg: float,
    cloud_height_scatter_m: float,
    pose_bias_deg: float,
    pose_height_bias_m: float,
    rng: np.random.Generator,
) -> list[FloorObservation]:
    """Per pose: clouds whose normals are turned by `cloud_scatter_deg` in symmetric tangent directions and whose
    heights scatter with RMS `cloud_height_scatter_m` and zero mean (Gaussian draws, centred and scaled), so the pose
    mean is the pose's floor, the pose's floor being the truth turned by `pose_bias_deg` about a random tangent axis
    and shifted by a random-sign `pose_height_bias_m`."""
    out = []
    for pose, (pan, tilt) in enumerate(GRID):
        normal, height = predicted_floor(chain, pan, tilt, origins)
        e1, e2 = floor_fit.tangent_basis(normal)
        phase = rng.uniform(0.0, 2.0 * np.pi)
        normal = tilted(normal, np.cos(phase) * e1 + np.sin(phase) * e2, pose_bias_deg)
        height += pose_height_bias_m * rng.choice([-1.0, 1.0])
        e1, e2 = floor_fit.tangent_basis(normal)
        scatter = rng.normal(0.0, 1.0, per_pose)
        scatter -= scatter.mean()
        scatter *= cloud_height_scatter_m / max(float(np.sqrt(np.mean(scatter**2))), 1e-12)
        for k in range(per_pose):
            angle = 2.0 * np.pi * k / per_pose
            n = tilted(normal, np.cos(angle) * e1 + np.sin(angle) * e2, cloud_scatter_deg)
            h = height + scatter[k]
            out.append(FloorObservation(pose, pan, tilt, PlaneFit(n, h, 1000, 1000, 0.0)))
    return out


def test_limits_use_pose_mean_residuals(urdf_text: str, chain: UrdfChain, truth: dict[str, Origin]) -> None:
    cfg = FloorFitConfig()
    limits = cfg.limits
    rng = np.random.default_rng(12)
    # Per-cloud normals 1.2 deg off and heights 15 mm off, but every pose mean is the truth floor: passes.
    observations = scattered_observations(chain, truth, 8, CLOUD_SCATTER_DEG, CLOUD_HEIGHT_SCATTER_M, 0.0, 0.0, rng)
    result = fit_head_chain_floor(observations, urdf_text, cfg)
    assert result.passed, result.failed_limits
    assert result.observation_normal_rms_deg == pytest.approx(CLOUD_SCATTER_DEG, abs=0.05)
    assert result.observation_normal_rms_deg > limits.max_normal_rms_deg
    assert result.observation_height_rms_mm == pytest.approx(CLOUD_HEIGHT_SCATTER_M * 1000.0, abs=0.5)
    assert result.observation_height_rms_mm > limits.max_height_rms_mm
    assert result.normal_rms_deg < 0.05
    assert result.height_rms_mm < 2.0
    for pose in result.poses:
        assert pose["observations"] == 8
        assert pose["normal_residual_deg"] < 0.05
        assert abs(pose["height_residual_mm"]) < 2.0
        assert pose["observation_normal_rms_deg"] == pytest.approx(CLOUD_SCATTER_DEG, abs=0.05)
        assert pose["observation_height_rms_mm"] == pytest.approx(CLOUD_HEIGHT_SCATTER_M * 1000.0, abs=0.5)
    data = result.to_json()
    json.dumps(data, allow_nan=False)
    assert data["observation_normal_rms_deg"] == pytest.approx(result.observation_normal_rms_deg)
    assert data["observation_height_rms_mm"] == pytest.approx(result.observation_height_rms_mm)
    assert set(data["poses"][0]) == {
        "pose", "pan_deg", "tilt_deg", "observations", "height_residual_mm", "normal_residual_deg",
        "observation_height_rms_mm", "observation_normal_rms_deg",
    }  # fmt: skip

    # Pose means that disagree with any chain fail, even with no scatter inside a pose.
    observations = scattered_observations(chain, truth, 4, 0.0, 0.0, POSE_BIAS_DEG, 0.0, rng)
    result = fit_head_chain_floor(observations, urdf_text, cfg)
    assert "max_normal_rms_deg" in result.failed_limits
    assert result.normal_rms_deg > limits.max_normal_rms_deg
    observations = scattered_observations(chain, truth, 4, 0.0, 0.0, 0.0, 2.0 * POSE_HEIGHT_BIAS_M, rng)
    result = fit_head_chain_floor(observations, urdf_text, cfg)
    assert "max_height_rms_mm" in result.failed_limits
    assert result.height_rms_mm > limits.max_height_rms_mm


def test_floor_sweep_defaults() -> None:
    config = node_config()
    sweep = config.FloorSweepConfig()
    assert sweep.clouds_per_pose == 10
    assert sweep.min_clouds_per_pose == 5
    shipped = yaml.safe_load(NODE_CONFIG_YAML.read_text())["floor"]
    assert shipped["clouds_per_pose"] == 10
    assert shipped["min_clouds_per_pose"] == 5
    # The minimum follows clouds_per_pose (half, rounded up) unless it is set, so a client.yml with 5 still works.
    assert config.FloorSweepConfig(clouds_per_pose=5).min_clouds_per_pose == 3
    assert config.FloorSweepConfig(clouds_per_pose=1).min_clouds_per_pose == 1
    assert config.FloorSweepConfig(clouds_per_pose=7, min_clouds_per_pose=None).min_clouds_per_pose == 4
    assert config.FloorSweepConfig(clouds_per_pose=10, min_clouds_per_pose=8).min_clouds_per_pose == 8
    for bad in ({"min_clouds_per_pose": 0}, {"clouds_per_pose": 4, "min_clouds_per_pose": 5}):
        with pytest.raises(ValidationError):
            config.FloorSweepConfig(**bad)


def test_client_floor_grid_avoids_arm_and_far_floor() -> None:
    # Pan -60 looks at the parked arm (all 3 poses skipped 2026-10-22) and tilt 12 sees the floor 2 m out, where a
    # room's floor (e.g. a garage sloping to its drain) is no longer one plane: the client grid stays near and clear.
    group_vars = yaml.safe_load((REPO_ROOT / "ansible" / "group_vars" / "client.yml").read_text())
    node = next(n for n in group_vars["ros2_nodes"] if n["name"] == "head_rgbd_calibration")
    floor = yaml.safe_load(node["config"])["floor"]
    limits = floor["fit"]["limits"]
    assert all(-45 <= p <= 45 for p in floor["pan_deg"])
    assert max(floor["pan_deg"]) - min(floor["pan_deg"]) >= limits["min_pan_span_deg"]
    assert min(floor["tilt_deg"]) >= 20
    assert len(floor["pan_deg"]) * len(floor["tilt_deg"]) >= limits["min_poses"] + 4
