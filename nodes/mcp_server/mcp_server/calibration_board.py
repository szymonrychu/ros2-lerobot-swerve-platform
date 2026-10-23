"""The calibration board on the gripper, for the board park: mcp_server's own geometry, a caller-chosen mount only.

POST /admin/park_calibration_board (head_rgbd_calibration, own token) sends the board mount pose it believes in. The
outline, the clearances and the floor height come from `config.calibration_board`; a mount further than the configured
tolerance from the configured one is refused, so no caller can shrink or move the checked board to park a real object.
"""

import math
from dataclasses import dataclass

import numpy as np
from ros2_common.head_rgbd_calibration.chain import pose_from_xyz_rpy

from .config import CalibrationBoardSettings

MM_PER_M = 1000.0


@dataclass(frozen=True)
class BoardPark:
    """The calibration board a park is checked with.

    Attributes:
        points: (N, 3) outline corners and lattice points in gripper_link (m).
        margin_m: Clearance the board keeps from every body obstacle (m), sag margin included.
        min_height_m: Lowest board point height above the floor (base_link z, m), sag margin included.
    """

    points: np.ndarray
    margin_m: float
    min_height_m: float

    @classmethod
    def from_mount(cls, settings: CalibrationBoardSettings, xyz: object, rpy: object) -> "BoardPark":
        """The configured board at a requested mount.

        Args:
            settings: mcp_server's board config (outline, margins, floor height, mount and tolerance).
            xyz: Requested board frame origin in gripper_link (m), three finite numbers.
            rpy: Requested fixed-axis xyz rotation (rad), three finite numbers.

        Returns:
            BoardPark: The board outline and lattice placed at the requested mount, with the configured limits.

        Raises:
            ValueError: The mount is not three finite numbers each, or lies further than the configured tolerance
                from the configured mount (translation in m, rotation in deg).
        """
        requested = (finite_triple(xyz, "xyz"), finite_triple(rpy, "rpy"))
        configured = pose_from_xyz_rpy(settings.mount_xyz_m, settings.mount_rpy_rad)
        mount = pose_from_xyz_rpy(*requested)
        shift = float(np.linalg.norm(mount[:3, 3] - configured[:3, 3]))
        turn = math.degrees(
            math.acos(max(-1.0, min(1.0, (np.trace(configured[:3, :3].T @ mount[:3, :3]) - 1.0) / 2.0)))
        )
        if shift > settings.mount_tolerance_m or turn > settings.mount_tolerance_deg:
            raise ValueError(
                f"board mount differs from the configured mount by {shift * MM_PER_M:.1f} mm and {turn:.1f} deg "
                f"(limit {settings.mount_tolerance_m * MM_PER_M:.0f} mm, {settings.mount_tolerance_deg:g} deg)"
            )
        width = settings.squares_x * settings.square_mm / MM_PER_M
        height = settings.squares_y * settings.square_mm / MM_PER_M
        margin = settings.outline_margin_m
        x0, y0, x1, y1 = -margin, -margin, width + margin, height + margin
        corners = np.array([[x0, y0, 0.0], [x1, y0, 0.0], [x1, y1, 0.0], [x0, y1, 0.0]])
        xs, ys = np.linspace(x0, x1, settings.lattice_points), np.linspace(y0, y1, settings.lattice_points)
        lattice = np.array([[x, y, 0.0] for y in ys for x in xs])
        board = np.vstack([corners, lattice])
        return cls(board @ mount[:3, :3].T + mount[:3, 3], settings.margin_m, settings.min_height_m)


def finite_triple(value: object, name: str) -> tuple[float, float, float]:
    """Three finite numbers.

    Args:
        value: The candidate (a list of three numbers).
        name: Its name in the message.

    Returns:
        tuple[float, float, float]: The numbers.

    Raises:
        ValueError: Not a list of three finite numbers.
    """
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"board mount {name} must be three numbers")
    try:
        numbers = tuple(float(v) for v in value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"board mount {name} must be three numbers") from exc
    if not all(math.isfinite(v) for v in numbers):
        raise ValueError(f"board mount {name} must be finite")
    return numbers  # type: ignore[return-value]
