"""Robot body obstacles for the arm self-collision check: pure geometry (numpy), no ROS.

The body seen from above is an H: two side frames (boxes along x above the wheels), the crossbar between them (box,
the arm and the lidar stand on it), four wheels modelled as their steering envelopes (vertical cylinders around the
steer axes) and the lidar column (vertical cylinder from the crossbar top to the dome top). Everything is in the robot
base_link (x forward, y left, z up from the floor), configured in config.BodySettings.

Distances are signed (negative = penetration depth) and exact for points. Every obstacle is convex, so the signed
distance along a capsule axis is a convex function of the axis parameter: its minimum is found by golden-section search
(GOLDEN_ITERATIONS shrink the bracket below 1e-10 of the axis length), vectorised over many capsules at once.
"""

import math
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np

from .config import BodySettings

GOLDEN_RATIO = (math.sqrt(5.0) - 1.0) / 2.0
GOLDEN_ITERATIONS = 48
ROUND_M = 4


class Obstacle(Protocol):
    """A convex body obstacle with an exact point signed distance."""

    name: str

    @property
    def top_z(self) -> float:
        """Highest point (m, base_link z)."""
        ...

    def signed_distance(self, points: np.ndarray) -> np.ndarray:
        """Signed distance of points (..., 3) to the obstacle (m, negative inside)."""
        ...

    def describe(self) -> dict[str, Any]:
        """JSON summary with the dimensions."""
        ...


@dataclass(frozen=True)
class BoxObstacle:
    """Axis-aligned box between corners lo and hi (base_link, m)."""

    name: str
    lo: np.ndarray
    hi: np.ndarray

    @property
    def top_z(self) -> float:
        """Box top (m).

        Returns:
            float: hi z.
        """
        return float(self.hi[2])

    def signed_distance(self, points: np.ndarray) -> np.ndarray:
        """Exact signed distance (outside: Euclidean distance to the box; inside: minus the depth to the nearest face).

        Args:
            points (np.ndarray): Points (..., 3) in base_link (m).

        Returns:
            np.ndarray: Distances (...) in m.
        """
        centre = (self.lo + self.hi) / 2.0
        half = (self.hi - self.lo) / 2.0
        q = np.abs(np.asarray(points, dtype=np.float64) - centre) - half
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=-1)
        inside = np.minimum(np.max(q, axis=-1), 0.0)
        return outside + inside

    def describe(self) -> dict[str, Any]:
        """JSON summary.

        Returns:
            dict[str, Any]: name, shape 'box', center_m, size_m, z range.
        """
        return {
            "name": self.name,
            "shape": "box",
            "center_m": [round(float(v), ROUND_M) for v in (self.lo + self.hi) / 2.0],
            "size_m": [round(float(v), ROUND_M) for v in self.hi - self.lo],
            "z_min_m": round(float(self.lo[2]), ROUND_M),
            "z_max_m": round(float(self.hi[2]), ROUND_M),
        }


@dataclass(frozen=True)
class CylinderObstacle:
    """Vertical cylinder around (x, y) from z_min to z_max (base_link, m)."""

    name: str
    center_xy: tuple[float, float]
    radius: float
    z_min: float
    z_max: float

    @property
    def top_z(self) -> float:
        """Cylinder top (m).

        Returns:
            float: z_max.
        """
        return self.z_max

    def signed_distance(self, points: np.ndarray) -> np.ndarray:
        """Exact signed distance to the solid cylinder (mantle, top and bottom discs).

        Args:
            points (np.ndarray): Points (..., 3) in base_link (m).

        Returns:
            np.ndarray: Distances (...) in m.
        """
        p = np.asarray(points, dtype=np.float64)
        radial = np.hypot(p[..., 0] - self.center_xy[0], p[..., 1] - self.center_xy[1]) - self.radius
        mid = (self.z_min + self.z_max) / 2.0
        vertical = np.abs(p[..., 2] - mid) - (self.z_max - self.z_min) / 2.0
        outside = np.hypot(np.maximum(radial, 0.0), np.maximum(vertical, 0.0))
        inside = np.minimum(np.maximum(radial, vertical), 0.0)
        return outside + inside

    def describe(self) -> dict[str, Any]:
        """JSON summary.

        Returns:
            dict[str, Any]: name, shape 'vertical_cylinder', center_xy_m, radius_m, z range.
        """
        return {
            "name": self.name,
            "shape": "vertical_cylinder",
            "center_xy_m": [round(self.center_xy[0], ROUND_M), round(self.center_xy[1], ROUND_M)],
            "radius_m": round(self.radius, ROUND_M),
            "z_min_m": round(self.z_min, ROUND_M),
            "z_max_m": round(self.z_max, ROUND_M),
        }


def capsule_signed_distance(obstacle: Obstacle, a: np.ndarray, b: np.ndarray, radius: float) -> np.ndarray:
    """Signed distance of capsules (axis a -> b, radius) to a convex obstacle: min over the axis minus the radius.

    Args:
        obstacle (Obstacle): Convex obstacle.
        a (np.ndarray): Axis starts (N, 3) in base_link (m).
        b (np.ndarray): Axis ends (N, 3).
        radius (float): Capsule radius (m).

    Returns:
        np.ndarray: (N,) distances (m), negative = penetration.
    """
    a = np.asarray(a, dtype=np.float64)
    d = np.asarray(b, dtype=np.float64) - a
    n = a.shape[0]
    lo, hi = np.zeros(n), np.ones(n)

    def at(t: np.ndarray) -> np.ndarray:
        return obstacle.signed_distance(a + d * t[:, None])

    x1 = hi - GOLDEN_RATIO * (hi - lo)
    x2 = lo + GOLDEN_RATIO * (hi - lo)
    f1, f2 = at(x1), at(x2)
    for _ in range(GOLDEN_ITERATIONS):
        left = f1 < f2  # the minimum lies in [lo, x2]
        hi = np.where(left, x2, hi)
        lo = np.where(left, lo, x1)
        x2_new = np.where(left, x1, lo + GOLDEN_RATIO * (hi - lo))
        x1_new = np.where(left, hi - GOLDEN_RATIO * (hi - lo), x2)
        f_new = at(np.where(left, x1_new, x2_new))
        f1, f2 = np.where(left, f_new, f2), np.where(left, f1, f_new)
        x1, x2 = x1_new, x2_new
    best = np.minimum(np.minimum(f1, f2), np.minimum(at(np.zeros(n)), at(np.ones(n))))
    return best - radius


class BodyModel:
    """The configured body obstacles."""

    def __init__(self, obstacles: list[Obstacle]) -> None:
        """Hold the obstacles.

        Args:
            obstacles (list[Obstacle]): Boxes and cylinders, in config order.
        """
        self.obstacles = obstacles

    @classmethod
    def from_settings(cls, settings: BodySettings) -> "BodyModel":
        """Build the model from the config (boxes first, then cylinders).

        Args:
            settings (BodySettings): Body config.

        Returns:
            BodyModel: The model.
        """
        obstacles: list[Obstacle] = []
        for box in settings.boxes:
            centre, half = np.array(box.center), np.array(box.size) / 2.0
            obstacles.append(BoxObstacle(box.name, centre - half, centre + half))
        for cyl in settings.cylinders:
            obstacles.append(CylinderObstacle(cyl.name, (cyl.x, cyl.y), cyl.radius, cyl.z_min, cyl.z_max))
        return cls(obstacles)

    def by_name(self, name: str) -> Any:
        """Obstacle by name.

        Args:
            name (str): Obstacle name.

        Returns:
            Any: The obstacle.

        Raises:
            KeyError: For an unknown name.
        """
        for obstacle in self.obstacles:
            if obstacle.name == name:
                return obstacle
        raise KeyError(name)

    def describe(self) -> list[dict[str, Any]]:
        """JSON summary of every obstacle.

        Returns:
            list[dict[str, Any]]: One describe() per obstacle.
        """
        return [o.describe() for o in self.obstacles]
