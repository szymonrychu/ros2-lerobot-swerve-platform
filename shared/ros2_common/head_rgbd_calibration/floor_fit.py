"""Floor-plane head chain fit: the floor plane of the head depth sensor over a pan/tilt sweep gives the head mount z,
roll, pitch and the head_tilt origin pitch (tilt zero) and roll (design D2-D4).

Frames: the floor is base_link z = 0 and the robot stands level on it; planes are expressed in the depth sensor's
optical frame `sensor_frame` of the head URDF (default `d435_depth_optical_frame`, the frame of /head/d435/points; z
forward, x right, y down) with the unit normal pointing from the floor toward the sensor, so a plane is
`normal . p = -height_m` with `height_m` the sensor origin's distance above it. numpy and scipy only, no ROS.
"""

import math
from collections.abc import Mapping
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

from ros2_common.head_rgbd_calibration.chain import UrdfChain, XyzRpy
from ros2_common.head_rgbd_calibration.config import FloorFitConfig, FloorPlaneConfig
from ros2_common.head_rgbd_calibration.urdf_text import Origin

D435_DEPTH_FRAME = "d435_depth_optical_frame"
PAN_JOINT = "head_pan"
TILT_JOINT = "head_tilt"
MOUNT_JOINT = "head_mount_joint"
FLOOR_UP = np.array([0.0, 0.0, 1.0])
MM_PER_M = 1000.0
RESIDUALS_PER_OBSERVATION = 3
MIN_SAMPLE_AREA = 1e-12
PARAM_NAMES = ("mount_z_m", "mount_roll_deg", "mount_pitch_deg", "tilt_zero_deg", "tilt_roll_deg")
Z_PARAM = "mount_z_m"
PARAM_SCALE = (1.0, 180.0 / math.pi, 180.0 / math.pi, 180.0 / math.pi, 180.0 / math.pi)
INT_POSE_KEYS = ("pose", "observations")


@dataclass(frozen=True)
class PlaneFit:
    """A floor plane in the depth sensor frame.

    Attributes:
        normal: Unit normal (3,) pointing from the floor toward the sensor.
        height_m: Distance of the sensor origin above the plane; the plane is `normal . p = -height_m`.
        inliers: Points within `ransac_distance_m` of the plane.
        candidates: Points that passed the range and prior-band gates.
        rms_m: RMS distance of the inliers to the plane.
    """

    normal: np.ndarray
    height_m: float
    inliers: int
    candidates: int
    rms_m: float


@dataclass(frozen=True)
class FloorObservation:
    """One cloud's floor plane at the measured head angles; `pose` is the sweep pose index."""

    pose: int
    pan_rad: float
    tilt_rad: float
    plane: PlaneFit


@dataclass
class FloorFitResult:
    """Result of `fit_head_chain_floor`.

    Attributes:
        passed: True only when every limit is met.
        failed_limits: Names of the `FloorFitLimits` fields not met, in check order.
        params: `{mount_z_m, mount_roll_deg, mount_pitch_deg, tilt_zero_deg, tilt_roll_deg: {value, sigma, urdf}}`
            (m or deg); sigma is inf when the fit did not run or the normal matrix is singular.
        height_rms_mm: RMS over the poses of the pose-mean height residual (the mean of a pose's observations); the
            `max_height_rms_mm` limit applies to it (at the URDF values when the fit did not run).
        normal_rms_deg: RMS over the poses of the angle between the pose-mean observed and predicted normals; the
            `max_normal_rms_deg` limit applies to it.
        observation_height_rms_mm: RMS of the height residuals of every observation (reported, not limited).
        observation_normal_rms_deg: RMS of the normal angle of every observation (reported, not limited).
        poses: Per pose `{pose, pan_deg, tilt_deg, observations, height_residual_mm, normal_residual_deg,
            observation_height_rms_mm, observation_normal_rms_deg}`: the pose-mean height residual (signed) and normal
            angle, and the RMS over the pose's observations.
        origins: New `head_mount_joint` and `head_tilt` origins; every component that is not fitted is the URDF value.
    """

    passed: bool
    failed_limits: list[str]
    params: dict[str, dict[str, float]]
    height_rms_mm: float
    normal_rms_deg: float
    observation_height_rms_mm: float = math.nan
    observation_normal_rms_deg: float = math.nan
    poses: list[dict[str, float | int]] = field(default_factory=list)
    origins: dict[str, Origin] = field(default_factory=dict)

    def to_json(self) -> dict:
        """Plain floats and lists; non-finite numbers (inf sigma, nan RMS of no observations) become None.

        Returns:
            dict: JSON-serialisable result (strict JSON, no NaN or Infinity).
        """
        return {
            "passed": bool(self.passed),
            "failed_limits": list(self.failed_limits),
            "params": {name: {k: json_float(v) for k, v in entry.items()} for name, entry in self.params.items()},
            "height_rms_mm": json_float(self.height_rms_mm),
            "normal_rms_deg": json_float(self.normal_rms_deg),
            "observation_height_rms_mm": json_float(self.observation_height_rms_mm),
            "observation_normal_rms_deg": json_float(self.observation_normal_rms_deg),
            "poses": [
                {k: (int(v) if k in INT_POSE_KEYS else json_float(v)) for k, v in pose.items()} for pose in self.poses
            ],
            "origins": {
                name: {"xyz": [float(v) for v in o.xyz], "rpy": [float(v) for v in o.rpy]}
                for name, o in self.origins.items()
            },
        }


def json_float(value: float) -> float | None:
    """`float(value)`, or None when it is not finite.

    Args:
        value: Number.

    Returns:
        float | None: The plain float, None for NaN and infinities.
    """
    value = float(value)
    return value if math.isfinite(value) else None


def as_xyz_rpy(origin: Origin | XyzRpy) -> XyzRpy:
    """(xyz, rpy) tuples of an `Origin` or an (xyz, rpy) pair.

    Args:
        origin: Origin or (xyz, rpy).

    Returns:
        tuple[xyz, rpy]: Translation (m) and fixed-axis angles (rad).
    """
    if isinstance(origin, Origin):
        return origin.xyz, origin.rpy
    return origin


def predicted_floor(
    chain: UrdfChain,
    pan_rad: float,
    tilt_rad: float,
    origins: Mapping[str, Origin | XyzRpy] | None,
    sensor_frame: str = D435_DEPTH_FRAME,
) -> tuple[np.ndarray, float]:
    """The floor (base_link z = 0) as the chain predicts it in the depth sensor frame.

    Args:
        chain: Head URDF chain rooted at base_link.
        pan_rad: Measured head_pan angle (rad).
        tilt_rad: Measured head_tilt angle (rad), + is down.
        origins: Per-joint origin replacement (`Origin` or (xyz, rpy)); None keeps the URDF.
        sensor_frame: Optical frame of the depth sensor in the head URDF.

    Returns:
        tuple[np.ndarray, float]: Unit floor normal toward the sensor (3,) and the sensor height above the floor (m).
    """
    overrides = {name: as_xyz_rpy(o) for name, o in (origins or {}).items()}
    pose = chain.transform(sensor_frame, {PAN_JOINT: pan_rad, TILT_JOINT: tilt_rad}, overrides)
    return pose[:3, :3].T @ FLOOR_UP, float(pose[2, 3])


def svd_plane(points: np.ndarray) -> tuple[np.ndarray, float]:
    """Least-squares plane through points, normal oriented toward the origin (the sensor).

    Args:
        points: (N, 3) points, N >= 3.

    Returns:
        tuple[np.ndarray, float]: Unit normal and height so that `normal . p = -height` with height >= 0.
    """
    centroid = points.mean(axis=0)
    normal = np.linalg.svd(points - centroid, full_matrices=False)[2][-1]
    height = -float(normal @ centroid)
    if height < 0:
        normal, height = -normal, -height
    return normal, height


def fit_plane(
    points: np.ndarray,
    prior: tuple[np.ndarray, float],
    cfg: FloorPlaneConfig,
    rng: np.random.Generator,
) -> PlaneFit | None:
    """Floor plane of one depth cloud: gate around the prior, RANSAC over 3-point samples, SVD refit of the inliers.

    Args:
        points: (N, 3) points in the depth sensor frame; non-finite rows are dropped.
        prior: (normal, height) of the plane the current chain predicts (`predicted_floor`).
        cfg: Plane fit settings.
        rng: Random generator for the RANSAC samples.

    Returns:
        PlaneFit | None: The fitted plane, or None when candidates or inliers are below `min_inliers` or the inlier
        ratio is below `min_inlier_ratio`.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    pts = pts[np.linalg.norm(pts, axis=1) <= cfg.max_range_m]
    prior_normal = np.asarray(prior[0], dtype=float)
    prior_normal = prior_normal / np.linalg.norm(prior_normal)
    candidates = pts[np.abs(pts @ prior_normal + prior[1]) <= cfg.candidate_band_m]
    count = len(candidates)
    if count < cfg.min_inliers:
        return None

    samples = rng.integers(0, count, size=(cfg.ransac_iterations, 3))
    best_mask: np.ndarray | None = None
    best_inliers = 0
    for a, b, c in candidates[samples]:
        normal = np.cross(b - a, c - a)
        area = np.linalg.norm(normal)
        if area < MIN_SAMPLE_AREA:
            continue
        normal = normal / area
        mask = np.abs((candidates - a) @ normal) <= cfg.ransac_distance_m
        inliers = int(mask.sum())
        if inliers > best_inliers:
            best_mask, best_inliers = mask, inliers
    if best_mask is None or best_inliers < cfg.min_inliers:
        return None

    normal, height = svd_plane(candidates[best_mask])
    mask = np.abs(candidates @ normal + height) <= cfg.ransac_distance_m
    inliers = int(mask.sum())
    if inliers < cfg.min_inliers or inliers / count < cfg.min_inlier_ratio:
        return None
    normal, height = svd_plane(candidates[mask])
    dist = candidates[mask] @ normal + height
    return PlaneFit(normal, height, inliers, count, float(np.sqrt(np.mean(dist**2))))


def tangent_basis(normal: np.ndarray) -> np.ndarray:
    """Two unit vectors spanning the plane orthogonal to `normal`.

    Args:
        normal: Unit vector (3,).

    Returns:
        np.ndarray: (2, 3) orthonormal tangent vectors.
    """
    helper = np.zeros(3)
    helper[int(np.argmin(np.abs(normal)))] = 1.0
    e1 = np.cross(normal, helper)
    e1 /= np.linalg.norm(e1)
    return np.stack([e1, np.cross(normal, e1)])


def chain_origins(chain: UrdfChain, x: np.ndarray) -> dict[str, Origin]:
    """Mount and tilt origins for a parameter vector; every other component is the URDF value.

    Args:
        chain: Head URDF chain.
        x: [mount z (m), mount roll (rad), mount pitch (rad), tilt origin pitch (rad), tilt origin roll (rad)].

    Returns:
        dict[str, Origin]: Origins of head_mount_joint and head_tilt.
    """
    (mx, my, _), (_, _, myaw) = chain.origin(MOUNT_JOINT)
    txyz, (_, _, tyaw) = chain.origin(TILT_JOINT)
    return {
        MOUNT_JOINT: Origin((mx, my, float(x[0])), (float(x[1]), float(x[2]), myaw)),
        TILT_JOINT: Origin(txyz, (float(x[4]), float(x[3]), tyaw)),
    }


def observation_errors(
    chain: UrdfChain,
    observations: list[FloorObservation],
    origins: dict[str, Origin],
    sensor_frame: str = D435_DEPTH_FRAME,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Per observation: normal error (n_obs - n_pred), height error (m) and normal angle error (deg).

    Args:
        chain: Head URDF chain.
        observations: Floor observations.
        origins: Origins to predict with.
        sensor_frame: Optical frame of the depth sensor the planes are in.

    Returns:
        tuple: (M, 3) normal differences, (M,) height differences h_obs - h_pred, (M,) angles in degrees.
    """
    dn = np.zeros((len(observations), 3))
    dh = np.zeros(len(observations))
    angle = np.zeros(len(observations))
    for i, obs in enumerate(observations):
        n_pred, h_pred = predicted_floor(chain, obs.pan_rad, obs.tilt_rad, origins, sensor_frame)
        n_obs = obs.plane.normal / np.linalg.norm(obs.plane.normal)
        dn[i] = n_obs - n_pred
        dh[i] = obs.plane.height_m - h_pred
        angle[i] = math.degrees(math.acos(float(np.clip(n_obs @ n_pred, -1.0, 1.0))))
    return dn, dh, angle


def pose_mean_errors(
    chain: UrdfChain,
    observations: list[FloorObservation],
    origins: dict[str, Origin],
    idx: list[int],
    dh: np.ndarray,
    sensor_frame: str = D435_DEPTH_FRAME,
) -> tuple[float, float]:
    """Pose-mean residuals of one pose: the mean height error and the angle between the mean normals.

    Averaging a pose's observations removes the per-cloud plane scatter (structured depth noise) that no chain
    explains.

    Args:
        chain: Head URDF chain.
        observations: Floor observations.
        origins: Origins to predict with.
        idx: Indices of the pose's observations (at least one).
        dh: (M,) height errors h_obs - h_pred of every observation (m).
        sensor_frame: Optical frame of the depth sensor the planes are in.

    Returns:
        tuple[float, float]: Mean height error (m) and the angle (deg) between the normalised mean observed and mean
        predicted normals.
    """
    n_obs = np.mean([observations[i].plane.normal / np.linalg.norm(observations[i].plane.normal) for i in idx], axis=0)
    n_pred = np.mean(
        [
            predicted_floor(chain, observations[i].pan_rad, observations[i].tilt_rad, origins, sensor_frame)[0]
            for i in idx
        ],
        axis=0,
    )
    cos = float(n_obs @ n_pred) / float(np.linalg.norm(n_obs) * np.linalg.norm(n_pred))
    return float(np.mean(dh[idx])), math.degrees(math.acos(float(np.clip(cos, -1.0, 1.0))))


def rms(values: np.ndarray | list[float]) -> float:
    """Root mean square; nan for no values."""
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else math.nan


def fit_head_chain_floor(
    observations: list[FloorObservation],
    urdf_text: str,
    cfg: FloorFitConfig,
    sensor_frame: str = D435_DEPTH_FRAME,
) -> FloorFitResult:
    """Fit the head mount z, roll, pitch and the head_tilt origin pitch (tilt zero) and roll (design D3, D4).

    Every other origin component of every joint stays at its value in `urdf_text` (the repo URDF with the applied
    overrides), the depth sensor mount (d435_joint) and its optical joints included. Residuals per observation are
    the two tangent components of `n_obs - n_pred` over `sigma_normal_deg` and the height error over
    `sigma_height_mm`, minimised with a Huber loss from the URDF values; sigma per parameter is
    `sqrt(diag((J^T J)^-1 * s^2))` with `s^2 = 2 * cost / (m - n)`. The height and normal RMS limits apply to the
    pose-mean residuals (`pose_mean_errors`); the per-observation RMS is reported next to them. With fewer than
    `min_observations` the solver is not called.

    Args:
        observations: Floor observations of the sweep, planes in `sensor_frame`.
        urdf_text: Head URDF text (base_link root) with the applied overrides.
        cfg: Fit settings and limits.
        sensor_frame: Optical frame of the depth sensor in the head URDF (default the D435 depth frame).

    Returns:
        FloorFitResult: Fitted values, sigma, residuals and every failed limit.
    """
    chain = UrdfChain.from_text(urdf_text)
    limits = cfg.limits
    (_, _, mount_z), (mount_roll, mount_pitch, _) = chain.origin(MOUNT_JOINT)
    tilt_roll, tilt_pitch, _ = chain.origin(TILT_JOINT)[1]
    x0 = np.array([mount_z, mount_roll, mount_pitch, tilt_pitch, tilt_roll], dtype=float)

    failed: list[str] = []
    pans = [math.degrees(o.pan_rad) for o in observations]
    if len(observations) < limits.min_observations:
        failed.append("min_observations")
    if len({o.pose for o in observations}) < limits.min_poses:
        failed.append("min_poses")
    if (max(pans) - min(pans) if pans else 0.0) < limits.min_pan_span_deg:
        failed.append("min_pan_span_deg")

    x = x0
    sigma = np.full(len(x0), math.inf)
    solve = "min_observations" not in failed
    if solve:
        sigma_n = math.radians(cfg.sigma_normal_deg)
        bases = [tangent_basis(o.plane.normal / np.linalg.norm(o.plane.normal)) for o in observations]

        def residuals(params: np.ndarray) -> np.ndarray:
            dn, dh, _ = observation_errors(chain, observations, chain_origins(chain, params), sensor_frame)
            tangent = np.einsum("mij,mj->mi", np.asarray(bases), dn) / sigma_n
            return np.column_stack([tangent, dh * MM_PER_M / cfg.sigma_height_mm]).ravel()

        res = least_squares(residuals, x0, loss="huber", f_scale=cfg.huber_scale)
        x = res.x
        dof = len(res.fun) - len(x)
        if dof > 0:
            try:
                cov = np.linalg.inv(res.jac.T @ res.jac) * (2.0 * res.cost / dof)
                sigma = np.sqrt(np.abs(np.diag(cov)))
            except np.linalg.LinAlgError:
                sigma = np.full(len(x0), math.inf)

    origins = chain_origins(chain, x)
    _, dh, angle = observation_errors(chain, observations, origins, sensor_frame)
    poses = []
    for pose in sorted({o.pose for o in observations}):
        idx = [i for i, o in enumerate(observations) if o.pose == pose]
        mean_dh, mean_angle = pose_mean_errors(chain, observations, origins, idx, dh, sensor_frame)
        poses.append(
            {
                "pose": pose,
                "pan_deg": float(np.mean([math.degrees(observations[i].pan_rad) for i in idx])),
                "tilt_deg": float(np.mean([math.degrees(observations[i].tilt_rad) for i in idx])),
                "observations": len(idx),
                "height_residual_mm": mean_dh * MM_PER_M,
                "normal_residual_deg": mean_angle,
                "observation_height_rms_mm": rms(dh[idx]) * MM_PER_M,
                "observation_normal_rms_deg": rms(angle[idx]),
            }
        )
    height_rms_mm = rms([float(p["height_residual_mm"]) for p in poses])
    normal_rms_deg = rms([float(p["normal_residual_deg"]) for p in poses])

    params = {
        name: {
            "value": float(x[i] * PARAM_SCALE[i]),
            "sigma": float(sigma[i] * PARAM_SCALE[i]),
            "urdf": float(x0[i] * PARAM_SCALE[i]),
        }
        for i, name in enumerate(PARAM_NAMES)
    }
    if solve:
        angles = [params[name] for name in PARAM_NAMES if name != Z_PARAM]
        z = params[Z_PARAM]
        checks = (
            ("max_height_rms_mm", height_rms_mm > limits.max_height_rms_mm),
            ("max_normal_rms_deg", normal_rms_deg > limits.max_normal_rms_deg),
            ("max_sigma_z_mm", not z["sigma"] * MM_PER_M <= limits.max_sigma_z_mm),
            ("max_sigma_deg", any(not p["sigma"] <= limits.max_sigma_deg for p in angles)),
            ("max_delta_z_m", abs(z["value"] - z["urdf"]) > limits.max_delta_z_m),
            ("max_delta_deg", any(abs(p["value"] - p["urdf"]) > limits.max_delta_deg for p in angles)),
        )
        failed.extend(name for name, bad in checks if bad)

    return FloorFitResult(
        passed=not failed,
        failed_limits=failed,
        params=params,
        height_rms_mm=height_rms_mm,
        normal_rms_deg=normal_rms_deg,
        observation_height_rms_mm=rms(dh) * MM_PER_M,
        observation_normal_rms_deg=rms(angle),
        poses=poses,
        origins=origins,
    )
