"""Paddle handle of a gripper-held ChArUco board: spec, solid, face-down turn and the seat in the SO-101 jaws.

Mesh coordinates are the print coordinates after the face-down turn: the plate spans x in [0, W], y in [0, H], z in
[0, T] with the bed at z = 0. The handle tab sticks out off the y = 0 edge (y in [-extent, 0]), centred at x = W / 2,
bottom on the bed. A shoulder block (y in [-neck, 0]) stops the moving finger tip. The grip (y in [-extent, -neck])
has a top plane that rises from thickness_tip at the shoulder to thickness_end at the free end, with a tapered channel
for the moving finger between two rails. Every wall is vertical and every top face rises, so nothing overhangs.

Seated in the gripper the bed face (mesh z = 0) lies on the fixed jaw, the moving finger in the channel on the top
plane, its tip at the shoulder face. Mesh +z is gripper_link OPEN_AXIS and mesh +y is TOOL_AXIS (the jaw tips point
toward the plate).
"""

from typing import Literal, Self

import numpy as np
import trimesh
from manifold3d import CrossSection, Manifold
from pydantic import Field, model_validator

from scripts.markers.jaw_fit import (
    CONTACT_HI_RAD,
    OPEN_AXIS,
    TOOL_AXIS,
    WIDTH_AXIS,
    Gripper,
    contact_angle_rad,
    fixed_jaw_face_mm,
    moving_jaw,
    moving_tip_mm,
)
from scripts.markers.mesh import box
from scripts.markers.models import StrictModel

MAX_CONTACT_WIDTH_MM = 12.0  # moving finger width at its tip: the channel floor must not be wider
SEAT_ITERATIONS = 10
SEAT_TOL_MM = 0.01
SWEEP_STEP_RAD = 0.005  # gripper angle step of the finger sweep that places the shoulder face
CUT_HEIGHT_MM = 100.0  # height of the channel cutter above the top plane (well above any rail)
CUT_OVERSHOOT_MM = 1.0  # the channel runs this far past the free end so it opens cleanly
SEAT_CROP_MM = 0.5  # the seat tests the solid below mesh y = this (handle plus a plate sliver), where the jaws are


class HandleSpec(StrictModel):
    """Paddle handle of a gripper-held board. All lengths in mm.

    length_mm: grip length behind the shoulder (the jaw depth it covers). neck_mm: shoulder block from the plate edge
    to the shoulder face. thickness_tip_mm / thickness_end_mm: top plane height over the bed at the shoulder and at
    the free end (chosen from the jaw gap at the contact angle). channel_tip_mm / channel_end_mm: channel width at the
    shoulder and at the free end (finger width plus 2 x clearance_mm). rail_width_mm / rail_height_mm: rails beside
    the channel, rail_height_mm above the top plane (the shoulder rises as high). squeeze_rad: how far the grip angle
    closes past the contact angle. density_g_per_cm3: print material density for the mass. tool_offset_m: the tool
    point in gripper_frame_link (m), the mcp_server arm tool_offset_m.
    """

    length_mm: float = Field(ge=25.0, le=40.0)
    neck_mm: float = Field(gt=0)
    thickness_tip_mm: float = Field(gt=0)
    thickness_end_mm: float = Field(gt=0)
    channel_tip_mm: float = Field(gt=0)
    channel_end_mm: float = Field(gt=0)
    clearance_mm: float = Field(ge=0.2, le=0.4)
    rail_width_mm: float = Field(gt=0)
    rail_height_mm: float = Field(gt=0)
    squeeze_rad: float = Field(gt=0, lt=0.03)
    density_g_per_cm3: float = Field(gt=0)
    tool_offset_m: tuple[float, float, float]

    @model_validator(mode="after")
    def check_profile(self) -> Self:
        """thickness_end_mm >= thickness_tip_mm, channel_end_mm >= channel_tip_mm, and the channel floor at the tip
        (channel_tip_mm - 2 x clearance_mm) is at most MAX_CONTACT_WIDTH_MM."""
        if self.thickness_end_mm < self.thickness_tip_mm:
            raise ValueError(f"thickness_end_mm {self.thickness_end_mm} must be >= thickness_tip_mm")
        if self.channel_end_mm < self.channel_tip_mm:
            raise ValueError(f"channel_end_mm {self.channel_end_mm} must be >= channel_tip_mm")
        if self.channel_tip_mm - 2 * self.clearance_mm > MAX_CONTACT_WIDTH_MM:
            raise ValueError(
                f"channel_tip_mm {self.channel_tip_mm} - 2 x clearance_mm exceeds {MAX_CONTACT_WIDTH_MM} mm"
            )
        return self

    @property
    def extent_mm(self) -> float:
        """Handle length off the plate edge (neck plus grip) in mm."""
        return self.neck_mm + self.length_mm

    @property
    def width_mm(self) -> float:
        """Handle width (widest channel plus both rails) in mm."""
        return self.channel_end_mm + 2 * self.rail_width_mm


def grip_profile(handle: HandleSpec, low: tuple[float, float], high: tuple[float, float], width: float) -> Manifold:
    """A prism along mesh x over `width` centred at x = 0, whose y-z section runs from the shoulder to the free end.

    Args:
        handle (HandleSpec): The handle (for neck and extent).
        low (tuple[float, float]): Lower edge z at the shoulder and at the free end (mm).
        high (tuple[float, float]): Upper edge z at the shoulder and at the free end (mm).
        width (float): Prism width along x (mm).

    Returns:
        Manifold: The prism; the section is linear between the shoulder (y = -neck) and the free end (y = -extent).
    """
    section = CrossSection(
        [
            [
                (-handle.extent_mm, low[1]),
                (-handle.neck_mm, low[0]),
                (-handle.neck_mm, high[0]),
                (-handle.extent_mm, high[1]),
            ]
        ]
    )
    # extrude along local z, then local (u, v, w) -> mesh (y, z, x): a cyclic permutation (proper rotation)
    turn = np.array([[0.0, 0.0, 1.0, -width / 2], [1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
    return Manifold.extrude(section, width).transform(turn)


def handle_solid(handle: HandleSpec, plate_mm: tuple[float, float]) -> Manifold:
    """The handle tab in print (face-down) coordinates.

    Args:
        handle (HandleSpec): The handle.
        plate_mm (tuple[float, float]): Plate (W, H) in mm; the tab is centred at x = W / 2.

    Returns:
        Manifold: Shoulder block plus grip with rails and channel, y in [-extent, 0], bottom at z = 0.
    """
    half = handle.width_mm / 2
    top = (handle.thickness_tip_mm, handle.thickness_end_mm)
    rails = (top[0] + handle.rail_height_mm, top[1] + handle.rail_height_mm)
    shoulder = box(-half, -handle.neck_mm, 0.0, half, 0.0, rails[0])
    grip = grip_profile(handle, (0.0, 0.0), rails, handle.width_mm)
    slope_tip = (handle.channel_end_mm - handle.channel_tip_mm) / handle.length_mm
    far_y = -handle.extent_mm - CUT_OVERSHOOT_MM
    far_half = (handle.channel_end_mm + slope_tip * CUT_OVERSHOOT_MM) / 2
    tip_half = handle.channel_tip_mm / 2
    channel = Manifold.extrude(
        CrossSection(
            [[(-tip_half, -handle.neck_mm), (-far_half, far_y), (far_half, far_y), (tip_half, -handle.neck_mm)]]
        ),
        CUT_HEIGHT_MM + rails[1],
    )
    rise = (top[1] - top[0]) / handle.length_mm * CUT_OVERSHOOT_MM
    above = Manifold.extrude(
        CrossSection(
            [
                [
                    (far_y, top[1] + rise),
                    (-handle.neck_mm, top[0]),
                    (-handle.neck_mm, CUT_HEIGHT_MM),
                    (far_y, CUT_HEIGHT_MM),
                ]
            ]
        ),
        handle.width_mm + 2 * CUT_OVERSHOOT_MM,
    ).transform(np.array([[0.0, 0.0, 1.0, -half - CUT_OVERSHOOT_MM], [1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]))
    solid = (shoulder + grip) - (channel ^ above)
    return solid.translate((plate_mm[0] / 2, 0.0, 0.0))


def face_down(solid: Manifold, plate_mm: tuple[float, float], thickness_mm: float) -> Manifold:
    """Turn a face-up body 180 deg about the line x = W / 2, z = T / 2 (parallel to y): pattern on the bed.

    Args:
        solid (Manifold): Body in face-up coordinates.
        plate_mm (tuple[float, float]): Plate (W, H) in mm.
        thickness_mm (float): Plate thickness T in mm.

    Returns:
        Manifold: The turned body, (x, y, z) -> (W - x, y, T - z).
    """
    turn = np.array([[-1.0, 0.0, 0.0, plate_mm[0]], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, -1.0, thickness_mm]])
    return solid.transform(turn)


def board_in_mesh(
    plate_mm: tuple[float, float], margin_mm: float, pattern_face: Literal["top", "bottom"], thickness_mm: float
) -> np.ndarray:
    """The OpenCV board frame (origin at the top-left outer corner, x right, y down, z into the board) in the mesh.

    Args:
        plate_mm (tuple[float, float]): Plate (W, H) in mm.
        margin_mm (float): Plate margin m around the pattern (mm).
        pattern_face (str): "top" (pattern at z = T, read from above) or "bottom" (pattern on the bed, read from
            below).
        thickness_mm (float): Plate thickness T in mm (the "top" origin lies on z = T).

    Returns:
        np.ndarray: 4x4 transform in mm. bottom: R = diag(-1, -1, 1), t = (W - m, H - m, 0); top: R = diag(1, -1, -1),
        t = (m, H - m, T).
    """
    width, height = plate_mm
    pose = np.eye(4)
    if pattern_face == "bottom":
        pose[:3, :3] = np.diag([-1.0, -1.0, 1.0])
        pose[:3, 3] = (width - margin_mm, height - margin_mm, 0.0)
    elif pattern_face == "top":
        pose[:3, :3] = np.diag([1.0, -1.0, -1.0])
        pose[:3, 3] = (margin_mm, height - margin_mm, thickness_mm)
    else:
        raise ValueError(f"pattern_face {pattern_face!r} is not 'top' or 'bottom'")
    return pose


def mesh_to_gripper(
    fixed_face: float, centre: float, shoulder: float, handle: HandleSpec, plate_mm: tuple[float, float]
) -> np.ndarray:
    """Mesh pose in gripper_link for a seat: mesh +z on OPEN_AXIS, mesh +y on TOOL_AXIS (mesh x on -WIDTH_AXIS).

    Args:
        fixed_face (float): OPEN_AXIS coordinate of the fixed jaw face, where mesh z = 0 goes (mm).
        centre (float): WIDTH_AXIS coordinate of the moving finger centre, where mesh x = W / 2 goes (mm).
        shoulder (float): TOOL_AXIS coordinate of the shoulder face, where mesh y = -neck goes (mm).
        handle (HandleSpec): The handle.
        plate_mm (tuple[float, float]): Plate (W, H) in mm.

    Returns:
        np.ndarray: 4x4 transform (mm).
    """
    opening, tool, width = (np.asarray(a) for a in (OPEN_AXIS, TOOL_AXIS, WIDTH_AXIS))
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack([-width, tool, opening])
    pose[:3, 3] = opening * fixed_face + tool * (shoulder + handle.neck_mm) + width * (centre + plate_mm[0] / 2)
    return pose


def shoulder_stop_mm(gripper: Gripper, contact_rad: float, top_mm: float) -> float:
    """TOOL_AXIS coordinate of the shoulder face: the furthest the moving jaw reaches below the shoulder top while the
    gripper closes from CONTACT_HI_RAD down to the contact angle.

    The finger tip moves along an arc, so it reaches further at some angles above the contact. Closing on the handle
    it pushes the shoulder there; at the contact it rests just behind it. A face at the tip of the contact angle
    alone would be penetrated at those angles.

    Args:
        gripper (Gripper): Loaded gripper.
        contact_rad (float): Contact angle (rad).
        top_mm (float): OPEN_AXIS coordinate of the shoulder top (mm).

    Returns:
        float: The shoulder face coordinate along TOOL_AXIS (mm).
    """
    opening, tool = np.asarray(OPEN_AXIS), np.asarray(TOOL_AXIS)
    reach = float(moving_tip_mm(gripper, contact_rad) @ tool)
    for angle in np.arange(contact_rad, CONTACT_HI_RAD, SWEEP_STEP_RAD):
        vertices = moving_jaw(gripper, float(angle)).vertices
        low = vertices[vertices @ opening <= top_mm]
        if len(low):
            reach = max(reach, float((low @ tool).max()))
    return reach


def seat(
    gripper: Gripper, solid: trimesh.Trimesh, handle: HandleSpec, plate_mm: tuple[float, float]
) -> tuple[np.ndarray, float]:
    """Seat the printed solid in the jaws and find the gripper contact angle.

    The bed face lies on the fixed jaw face (over the grip length behind the fixed tip), the channel centre on the
    moving finger centre and the shoulder face on the finger sweep (`shoulder_stop_mm`). The contact angle comes
    from `contact_angle_rad` on the seated solid; seat and angle are iterated (at most SEAT_ITERATIONS, until the
    seat moves less than SEAT_TOL_MM; the returned seat is the one the angle was measured on). The contact is found
    on the solid below mesh y = SEAT_CROP_MM (capped): the jaw tips stay behind the shoulder face, so the plate
    cannot touch them, and the small mesh keeps the contact search fast.

    Args:
        gripper (Gripper): Loaded gripper.
        solid (trimesh.Trimesh): Watertight printed solid in mesh coordinates (mm).
        handle (HandleSpec): The handle.
        plate_mm (tuple[float, float]): Plate (W, H) in mm.

    Returns:
        tuple[np.ndarray, float]: (mesh pose in gripper_link, 4x4 mm; contact angle, rad).

    Raises:
        ValueError: When the seat does not converge or the contact angle is unreachable.
    """
    width = np.asarray(WIDTH_AXIS)
    reachable = trimesh.intersections.slice_mesh_plane(solid, (0.0, -1.0, 0.0), (0.0, SEAT_CROP_MM, 0.0), cap=True)
    fixed_face = fixed_jaw_face_mm(gripper, (0.0, handle.length_mm))
    top = fixed_face + handle.thickness_tip_mm + handle.rail_height_mm
    contact = 0.0
    pose = mesh_to_gripper(
        fixed_face,
        float(moving_tip_mm(gripper, contact) @ width),
        shoulder_stop_mm(gripper, contact, top),
        handle,
        plate_mm,
    )
    for _ in range(SEAT_ITERATIONS):
        contact = contact_angle_rad(gripper, reachable.copy().apply_transform(pose))
        new = mesh_to_gripper(
            fixed_face,
            float(moving_tip_mm(gripper, contact) @ width),
            shoulder_stop_mm(gripper, contact, top),
            handle,
            plate_mm,
        )
        if float(np.linalg.norm(new[:3, 3] - pose[:3, 3])) < SEAT_TOL_MM:
            return pose, contact  # the seat the contact angle was measured on, so the pair is consistent
        pose = new
    raise ValueError(f"handle seat did not converge in {SEAT_ITERATIONS} iterations")
