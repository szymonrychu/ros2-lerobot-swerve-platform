"""Pydantic configuration for the scene, the joint mapping and the replay thresholds."""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, PositiveFloat, model_validator

SUPPORT_Z_TOLERANCE_M = 1e-6
# Arm base (URDF base_link origin) height above the floor measured on the robot 2026-10-10 (client.yml
# arm.arm_base_height_m): the floor is at z = -0.104 in the arm frame.
DEFAULT_BASE_HEIGHT_M = 0.104
DEFAULT_OBJECT_X_M = 0.20
DEFAULT_SUPPORT_MARGIN_M = 0.08
DEFAULT_FRICTION = (1.0, 0.005, 0.0001)
# mcp_server widens the URDF shoulder_lift upper limit to 1.9 rad (ansible/group_vars/client.yml joint_limit_overrides_rad).
DEFAULT_LIMIT_OVERRIDES: dict[str, tuple[float, float]] = {"shoulder_lift": (-1.745, 1.9)}

SupportKind = Literal["floor", "ledge", "stair"]


class StrictModel(BaseModel):
    """Base model that rejects unknown keys so config typos fail loudly."""

    model_config = ConfigDict(extra="forbid")


class JointOffsets(StrictModel):
    """Follower-vs-URDF zero offsets in rad: urdf_angle = measured_angle + offset.

    Defaults are the values solved on 2026-10-08 and set in ansible/group_vars/client.yml (mcp_server
    joint_offsets_rad). The gripper has no offset. Use all zeros to replay in raw URDF/MuJoCo angles.
    """

    shoulder_pan: float = -0.0619
    shoulder_lift: float = -0.0103
    elbow_flex: float = -0.1381
    wrist_flex: float = 0.2477
    wrist_roll: float = -0.0710
    gripper: float = 0.0


class BoxObjectConfig(StrictModel):
    """A free box to grasp. Sizes are full edge lengths in metres (x forward from the arm, y left, z up)."""

    size_m: tuple[PositiveFloat, PositiveFloat, PositiveFloat] = (0.03, 0.03, 0.04)
    mass_kg: PositiveFloat = 0.05
    friction: tuple[PositiveFloat, float, float] = DEFAULT_FRICTION
    x_m: float = DEFAULT_OBJECT_X_M
    y_m: float = 0.0
    yaw_rad: float = 0.0
    # Clear height under the object (m): > 0 rests it on two thin rails along its x edges, leaving a slot a scoop's
    # fixed jaw can slide into; 0 = flat on the support.
    gap_below_m: float = Field(default=0.0, ge=0.0, le=0.2)


class SceneConfig(StrictModel):
    """Arm mount, ground, optional ledge/stair support and the object. Frame: the arm base frame (z up).

    The arm base (URDF base_link origin) is at the world origin, the floor top is at z = -base_height_m.
    support_z_m is the top surface of the surface the object sits on, in the same frame: None or the floor
    height means the object is on the floor, above it is a ledge/table, below it is a lower stair.
    """

    base_height_m: PositiveFloat = DEFAULT_BASE_HEIGHT_M
    joint_offsets_rad: JointOffsets = Field(default_factory=JointOffsets)
    limit_overrides_rad: dict[str, tuple[float, float]] = Field(default_factory=lambda: dict(DEFAULT_LIMIT_OVERRIDES))
    object: BoxObjectConfig | None = Field(default_factory=BoxObjectConfig)
    support_z_m: float | None = None
    support_edge_x_m: float | None = None
    support_depth_m: PositiveFloat = 0.6
    support_width_m: PositiveFloat = 0.6
    support_friction: tuple[PositiveFloat, float, float] = DEFAULT_FRICTION
    # Jaw closing point in gripper_frame_link (m), e.g. mcp_server arm.tool_offset_m; None keeps the stock jaws.
    tool_offset_m: tuple[float, float, float] | None = None

    @property
    def floor_z(self) -> float:
        """Height of the floor top in the arm frame (m)."""
        return -self.base_height_m

    @property
    def support_kind(self) -> SupportKind:
        """Whether the object sits on the floor, on a ledge above it or on a stair below it."""
        if self.support_z_m is None or abs(self.support_z_m - self.floor_z) < SUPPORT_Z_TOLERANCE_M:
            return "floor"
        return "ledge" if self.support_z_m > self.floor_z else "stair"

    @property
    def support_top_z(self) -> float:
        """Height of the surface under the object in the arm frame (m)."""
        return self.floor_z if self.support_kind == "floor" else float(self.support_z_m or 0.0)

    @property
    def support_start_x(self) -> float:
        """Arm-frame x where the ledge/stair starts (m)."""
        if self.support_edge_x_m is not None:
            return self.support_edge_x_m
        object_x = self.object.x_m if self.object is not None else DEFAULT_OBJECT_X_M
        return object_x - DEFAULT_SUPPORT_MARGIN_M

    @model_validator(mode="after")
    def check_support_needs_object_clearance(self) -> "SceneConfig":
        """Reject a support edge that starts behind the arm base."""
        if self.support_kind != "floor" and self.support_start_x < 0.0:
            raise ValueError("support edge must start in front of the arm base (x >= 0)")
        return self


class SimConfig(StrictModel):
    """Replay thresholds and options."""

    settle_s: PositiveFloat = 0.5
    hold_end_s: float = Field(default=0.3, ge=0.0)
    tilt_threshold_deg: PositiveFloat = 10.0
    push_threshold_m: PositiveFloat = 0.01
    lift_min_height_m: PositiveFloat = 0.02
    lift_hold_distance_m: PositiveFloat = 0.06
    contact_allowed_labels: tuple[str, ...] = ("grasp", "close", "lift", "retreat")
    allow_jaw_surface_contact: bool = True
    clearance_stride: int = Field(default=4, ge=1)
    clearance_distmax_m: PositiveFloat = 0.5
