"""Synthetic D435 depth floor clouds seen through a known head chain, for the floor-plane head chain fit tests.

Floor points (base_link z = 0) inside the D435 depth field of view, plus points of a vertical wall in front of the head
as outliers, are expressed in `d435_depth_optical_frame` of a perturbed (truth) chain while the reported pan/tilt are
the commanded grid angles, so a fit started from the URDF has to recover the perturbation.
"""

from dataclasses import dataclass

import numpy as np
from ros2_common.head_rgbd_calibration.chain import UrdfChain
from ros2_common.head_rgbd_calibration.urdf_text import Origin

SENSOR_FRAME = "d435_depth_optical_frame"
# D435 depth field of view at 16:9 (datasheet Table 3-49, docs/research/2026-10-27-d435-docs.md 4.3).
FOV_H_DEG = 87.0
FOV_V_DEG = 58.0
MIN_RANGE_M = 0.3
MAX_RANGE_M = 1.8
RAYS = (60, 45)
WALL_DISTANCE_M = 1.2
WALL_SAMPLE_ROUNDS = 50
WALL_SAMPLE_BATCH = 4000


@dataclass(frozen=True)
class SyntheticCloud:
    """One depth cloud: the pose index, the reported head angles (rad) and the (N, 3) points in SENSOR_FRAME."""

    pose: int
    pan_rad: float
    tilt_rad: float
    points: np.ndarray


def ray_grid(rays: tuple[int, int] = RAYS) -> np.ndarray:
    """Unit ray directions of a regular grid over the depth field of view (optical: x right, y down, z forward).

    Args:
        rays: (columns, rows) of the grid.

    Returns:
        np.ndarray: (columns * rows, 3) unit vectors.
    """
    ax = np.radians(np.linspace(-FOV_H_DEG / 2, FOV_H_DEG / 2, rays[0]))
    ay = np.radians(np.linspace(-FOV_V_DEG / 2, FOV_V_DEG / 2, rays[1]))
    gx, gy = np.meshgrid(np.tan(ax), np.tan(ay))
    dirs = np.stack([gx.ravel(), gy.ravel(), np.ones(gx.size)], axis=1)
    return dirs / np.linalg.norm(dirs, axis=1, keepdims=True)


def random_rays(count: int, rng: np.random.Generator) -> np.ndarray:
    """Uniformly random unit ray directions inside the depth field of view.

    Args:
        count: Number of rays.
        rng: Random generator.

    Returns:
        np.ndarray: (count, 3) unit vectors in the depth optical frame.
    """
    ax = np.radians(rng.uniform(-FOV_H_DEG / 2, FOV_H_DEG / 2, count))
    ay = np.radians(rng.uniform(-FOV_V_DEG / 2, FOV_V_DEG / 2, count))
    dirs = np.stack([np.tan(ax), np.tan(ay), np.ones(count)], axis=1)
    return dirs / np.linalg.norm(dirs, axis=1, keepdims=True)


def sensor_pose(chain: UrdfChain, pan_rad: float, tilt_rad: float, origins: dict[str, Origin]) -> np.ndarray:
    """Pose of SENSOR_FRAME in base_link for the given head angles and origin overrides.

    Args:
        chain: Parsed head URDF.
        pan_rad: head_pan angle (rad).
        tilt_rad: head_tilt angle (rad), + is down.
        origins: Origin override per joint name.

    Returns:
        np.ndarray: 4x4 pose.
    """
    overrides = {name: (o.xyz, o.rpy) for name, o in origins.items()}
    return chain.transform(SENSOR_FRAME, {"head_pan": pan_rad, "head_tilt": tilt_rad}, overrides)


def floor_points(pose: np.ndarray, dirs: np.ndarray) -> np.ndarray:
    """Intersections of the rays with the floor (base_link z = 0) within the depth range.

    Args:
        pose: 4x4 pose of the depth optical frame in base_link.
        dirs: (N, 3) unit ray directions in the depth optical frame.

    Returns:
        np.ndarray: (M, 3) floor points in the depth optical frame.
    """
    down = dirs @ pose[:3, :3].T[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        s = -pose[2, 3] / down
    keep = (down < 0) & (s >= MIN_RANGE_M) & (s <= MAX_RANGE_M)
    return dirs[keep] * s[keep, None]


def wall_points(pose: np.ndarray, count: int, rng: np.random.Generator) -> np.ndarray:
    """Points of a vertical wall WALL_DISTANCE_M ahead of the sensor (along its horizontal heading), above the floor.

    Args:
        pose: 4x4 pose of the depth optical frame in base_link.
        count: Number of points wanted.
        rng: Random generator.

    Returns:
        np.ndarray: (count, 3) wall points in the depth optical frame.

    Raises:
        ValueError: The wall is not in view often enough to give `count` points.
    """
    rot = pose[:3, :3]
    heading = rot[:, 2].copy()
    heading[2] = 0.0
    heading /= np.linalg.norm(heading)
    wall_normal = rot.T @ heading
    found: list[np.ndarray] = []
    total = 0
    for _ in range(WALL_SAMPLE_ROUNDS):
        if total >= count:
            break
        dirs = random_rays(WALL_SAMPLE_BATCH, rng)
        along = dirs @ wall_normal
        with np.errstate(divide="ignore", invalid="ignore"):
            s = WALL_DISTANCE_M / along
        z = pose[2, 3] + s * (dirs @ rot[2])
        keep = (along > 0) & (s >= MIN_RANGE_M) & (s <= MAX_RANGE_M) & (z > 0)
        pts = dirs[keep] * s[keep, None]
        found.append(pts)
        total += len(pts)
    if total < count:
        raise ValueError(f"wall gives {total} points, {count} wanted")
    return np.concatenate(found)[:count]


def synthetic_clouds(
    urdf_text: str,
    truth_origins: dict[str, Origin],
    grid: list[tuple[float, float]],
    noise_m: float,
    outlier_ratio: float,
    rng: np.random.Generator,
    clouds_per_pose: int = 1,
    rays: tuple[int, int] = RAYS,
) -> list[SyntheticCloud]:
    """Depth clouds of a flat floor (and a wall) seen through the truth chain at each grid pose.

    Args:
        urdf_text: Head URDF text (base_link root).
        truth_origins: Origin override per joint of the truth chain (the perturbed mount and tilt).
        grid: Reported (pan_rad, tilt_rad) per pose.
        noise_m: Gaussian noise sigma per coordinate (m), on floor and wall points.
        outlier_ratio: Fraction of all points of a cloud that are wall points (0 <= v < 1).
        rng: Random generator.
        clouds_per_pose: Clouds per grid pose, each with its own noise.
        rays: (columns, rows) of the floor ray grid.

    Returns:
        list[SyntheticCloud]: Clouds in grid order, `clouds_per_pose` per pose, points shuffled.
    """
    chain = UrdfChain.from_text(urdf_text)
    dirs = ray_grid(rays)
    clouds = []
    for pose_index, (pan, tilt) in enumerate(grid):
        pose = sensor_pose(chain, pan, tilt, truth_origins)
        floor = floor_points(pose, dirs)
        n_wall = int(round(outlier_ratio / (1.0 - outlier_ratio) * len(floor)))
        for _ in range(clouds_per_pose):
            parts = [floor]
            if n_wall:
                parts.append(wall_points(pose, n_wall, rng))
            pts = np.concatenate(parts)
            pts = pts + rng.normal(0.0, noise_m, pts.shape)
            clouds.append(SyntheticCloud(pose_index, pan, tilt, rng.permutation(pts)))
    return clouds
