"""Arm collision settings: the pydantic models of the arm mount and calibration, the robot body obstacles and the body
check margins, shared by every node that places the arm against the robot body (mcp_server, filter_node)."""

import math
from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field, PositiveFloat, field_validator, model_validator

# settings.py -> shared/arm_collision/arm_collision -> repo root is three levels above the package.
REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_URDF = Path("nodes/web_ui/urdf/so101_arm.urdf")
# The one body definition (load_robot_body), packaged next to this module.
ROBOT_BODY_FILE = Path(__file__).resolve().with_name("robot_body.yaml")
# Body obstacles above the floor-level H (the head on its post): in every arm check, left out of the nav footprint.
HEAD_OBSTACLES = ("head_post", "head_arm", "head")
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
# Arm mount (URDF base_link origin) in base_link (m), measured on the robot 2026-10-10: shoulder_pan axis 98 mm forward and
# 50 mm right of the centre between the wheels, URDF origin 38.8 mm behind and 62.4 mm below it; height from tip
# touch-downs on the floor. The mount height is also the floor height under the arm (floor_z_m = -ARM_BASE_HEIGHT_M).
ARM_BASE_HEIGHT_M = 0.100
ARM_MOUNT_X_M = 0.0592
ARM_MOUNT_Y_M = -0.05  # 5 cm to the right of the base_link centre line
MOUNT_HEIGHT_TOLERANCE_M = 1e-6


class StrictModel(BaseModel):
    """Base model rejecting unknown keys so config typos fail loudly."""

    model_config = ConfigDict(extra="forbid")


class ArmBaseOffset(StrictModel):
    """Pose of the arm base frame (URDF base_link, z = 0 on the mount plane) in the robot base_link frame.

    Attributes:
        x: Arm base x in base_link (m).
        y: Arm base y in base_link (m).
        z: Arm base height above the floor (base_link z = 0 is the floor); normally arm_base_height_m.
        yaw: Rotation of the arm base about z (rad).
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


class JointOffsets(StrictModel):
    """Zero offsets (rad) of the five kinematic arm joints: urdf_angle = measured_angle + offset (gripper excluded)."""

    shoulder_pan: float = 0.0
    shoulder_lift: float = 0.0
    elbow_flex: float = 0.0
    wrist_flex: float = 0.0
    wrist_roll: float = 0.0


class ToolOffset(StrictModel):
    """Offset (m) of the tool centre point from the URDF tool frame gripper_frame_link, in that frame's axes."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class BodyBox(StrictModel):
    """Axis-aligned box obstacle of the robot body in base_link.

    Attributes:
        name: Obstacle name reported in rejections, slowdowns and metrics (e.g. frame_left, crossbar).
        center: Box centre (x, y, z) in base_link (m).
        size: Full edge lengths along base_link x, y, z (m, > 0).
    """

    name: str = Field(min_length=1)
    center: tuple[float, float, float]
    size: tuple[PositiveFloat, PositiveFloat, PositiveFloat]


class BodyCylinder(StrictModel):
    """Vertical cylinder obstacle of the robot body in base_link (a steering wheel envelope, the lidar column).

    Attributes:
        name: Obstacle name (e.g. wheel_fl, lidar).
        x: Axis x in base_link (m).
        y: Axis y in base_link (m).
        radius: Radius (m, > 0).
        z_min: Bottom (m, base_link z).
        z_max: Top (m), above z_min.
    """

    name: str = Field(min_length=1)
    x: float
    y: float
    radius: PositiveFloat
    z_min: float
    z_max: float

    @model_validator(mode="after")
    def positive_height(self) -> "BodyCylinder":
        """Require z_max above z_min.

        Returns:
            BodyCylinder: The validated obstacle.
        """
        if not self.z_max > self.z_min:
            raise ValueError(f"body cylinder {self.name}: z_max ({self.z_max}) must be above z_min ({self.z_min})")
        return self


class BodySettings(StrictModel):
    """Robot body obstacles the arm must not hit (body_model, self_collision), all in base_link.

    The arm base and its shoulder link stand on the crossbar and only turn in place about the pan axis: they are not
    modelled, so they are never checked against the crossbar (or anything else).
    """

    boxes: list[BodyBox]
    cylinders: list[BodyCylinder]

    @model_validator(mode="after")
    def unique_names(self) -> "BodySettings":
        """Require unique obstacle names (they label rejections and metrics).

        Returns:
            BodySettings: The validated settings.
        """
        names = [o.name for o in [*self.boxes, *self.cylinders]]
        duplicates = sorted({n for n in names if names.count(n) > 1})
        if duplicates:
            raise ValueError(f"body obstacle names must be unique, duplicated: {duplicates}")
        return self


class BodyCheckSettings(StrictModel):
    """Arm self-collision soft boundary (self_collision): every executed arm trajectory sample is checked against the body.

    A sample whose link clearance to an obstacle is below soft_margin_m runs at slow_speed_scale (the floor guard's
    scale applies too; the slower wins); below hard_margin_m (or penetrating) the whole motion is rejected before it
    starts. get_arm_reach's envelope grid: shoulder_pan headings every reach_heading_step_deg, tool heights 0 ..
    reach_height_max_m every reach_height_step_m (base_link z).
    """

    enabled: bool = True
    soft_margin_m: float = Field(default=0.03, gt=0.0, le=0.2)
    hard_margin_m: float = Field(default=0.002, ge=0.0, le=0.2)
    slow_speed_scale: float = Field(default=0.3, gt=0.0, le=1.0)
    reach_heading_step_deg: float = Field(default=15.0, ge=5.0, le=90.0)
    reach_height_max_m: float = Field(default=0.35, gt=0.0, le=1.0)
    reach_height_step_m: float = Field(default=0.05, ge=0.01, le=0.5)
    reach_wait_s: float = Field(default=10.0, gt=0.0, le=60.0)  # get_arm_reach waits this long for the envelope
    reach_max_points: int = Field(default=20, ge=1, le=50)
    # Envelope cache on disk, keyed by every input (body, mount, joint limits, grid, margins): a restart reuses it
    # instead of recomputing (95 s on the busy RPi5 under the unit's CPU quota). None keeps it in memory only.
    reach_cache_file: Path | None = None

    @model_validator(mode="after")
    def hard_below_soft(self) -> "BodyCheckSettings":
        """Require hard_margin_m below soft_margin_m (the slow band lies between them).

        Returns:
            BodyCheckSettings: The validated settings.
        """
        if not self.hard_margin_m < self.soft_margin_m:
            raise ValueError(
                f"body_check.hard_margin_m ({self.hard_margin_m}) must be below soft_margin_m ({self.soft_margin_m})"
            )
        return self


class ArmModelSettings(StrictModel):
    """Arm model fields the kinematics, the jaw and the body check need: URDF, mount, calibration and gripper.

    Every node placing the arm against the body configures these with the same values (mcp_server's arm block is a
    superset of them).
    """

    urdf_path: Path = Field(default=DEFAULT_URDF, validate_default=True)
    gripper_joint: str = "gripper"
    # Follower-vs-URDF zero offsets: urdf_angle = measured_angle (/follower/joint_states) + offset. Used by all
    # kinematics (tool pose, link frames, IK and URDF-space limit clamping); joint targets and reported positions
    # stay in measured space. Solve them from the stored calibration samples (mcp_server README, "Joint zero offsets").
    joint_offsets_rad: JointOffsets = Field(default_factory=JointOffsets)
    # Where the jaws actually close, relative to gripper_frame_link and expressed in its frame (measured on the robot).
    # tool_pose, move_arm_cartesian and the camera tool overlays all refer to this point.
    tool_offset_m: ToolOffset = Field(default_factory=ToolOffset)
    # Direction (in gripper_frame_link, normalised on load) in which the moving jaw opens away from the fixed jaw. With
    # move_arm_cartesian(object_width_m=w) the tool point is placed w/2 against this direction from the object centre.
    jaw_open_axis: tuple[float, float, float] = (-1.0, 0.0, 0.0)
    # Replacement (lower, upper) limits in URDF space (rad) of named joints (chain joints or gripper): IK bounds,
    # within-limits checks and target clamping all use them instead of the URDF values. Empty = URDF limits.
    joint_limit_overrides_rad: dict[str, tuple[float, float]] = Field(default_factory=dict)
    # Follower gripper joint position of a closed jaw (as in /follower/joint_states). Measured closed: -0.172 rad (URDF
    # lower limit -0.1745); -0.165 is just above it and inside the gripper limit margin (arm_limit_margin_overrides).
    gripper_closed_rad: float = -0.165
    # Height of the arm mount plane (the URDF base_link origin) above the floor (0.100 m, corrected 2026-10-10 from a
    # tip-on-floor check; earlier 0.104 from tip touch-downs, 0.165 and 0.15 m before).
    arm_base_height_m: float = Field(default=ARM_BASE_HEIGHT_M, gt=0.0, le=1.0)
    # Arm base frame pose in the robot base_link (measured 2026-10-10): 5.92 cm forward, 5 cm right, z equal to
    # arm_base_height_m. null disables every arm <-> base_link conversion (camera results, grasp base_link input).
    base_in_base_link: ArmBaseOffset | None = Field(
        default_factory=lambda: ArmBaseOffset(x=ARM_MOUNT_X_M, y=ARM_MOUNT_Y_M, z=ARM_BASE_HEIGHT_M, yaw=0.0)
    )

    @model_validator(mode="after")
    def mount_height_consistent(self) -> "ArmModelSettings":
        """Require the mount z to equal arm_base_height_m (both are the arm base height above the robot plane).

        Returns:
            ArmModelSettings: The validated settings.
        """
        mount = self.base_in_base_link
        if mount is not None and abs(mount.z - self.arm_base_height_m) > MOUNT_HEIGHT_TOLERANCE_M:
            raise ValueError(
                f"arm.base_in_base_link.z ({mount.z}) must equal arm.arm_base_height_m ({self.arm_base_height_m})"
            )
        return self

    @field_validator("jaw_open_axis")
    @classmethod
    def normalise_jaw_axis(cls, v: tuple[float, float, float]) -> tuple[float, float, float]:
        """Normalise the jaw opening direction to a unit vector.

        Args:
            v (tuple[float, float, float]): Configured direction.

        Returns:
            tuple[float, float, float]: Unit direction.
        """
        norm = math.sqrt(sum(c * c for c in v))
        if norm < 1e-9:
            raise ValueError("arm.jaw_open_axis must not be the zero vector")
        return (v[0] / norm, v[1] / norm, v[2] / norm)

    @field_validator("joint_limit_overrides_rad")
    @classmethod
    def check_limit_overrides(cls, v: dict[str, tuple[float, float]]) -> dict[str, tuple[float, float]]:
        """Require known joint names and lower < upper.

        Args:
            v (dict[str, tuple[float, float]]): Joint name -> (lower, upper) rad.

        Returns:
            dict[str, tuple[float, float]]: The validated overrides.
        """
        for name, (lo, hi) in v.items():
            if name not in ARM_JOINTS:
                raise ValueError(
                    f"arm.joint_limit_overrides_rad: unknown joint {name!r}; use one of {list(ARM_JOINTS)}"
                )
            if not lo < hi:
                raise ValueError(f"arm.joint_limit_overrides_rad[{name}]: lower {lo} must be below upper {hi}")
        return v

    @field_validator("urdf_path")
    @classmethod
    def resolve_urdf(cls, v: Path) -> Path:
        """Resolve a relative URDF path against the repository root.

        Args:
            v (Path): Configured URDF path.

        Returns:
            Path: Absolute URDF path.
        """
        return v if v.is_absolute() else REPO_ROOT / v


def load_robot_body() -> BodySettings:
    """The robot body obstacles of the packaged robot_body.yaml (the single body definition of every node).

    Returns:
        BodySettings: Validated boxes and cylinders (a fresh model per call).
    """
    return BodySettings.model_validate(yaml.safe_load(ROBOT_BODY_FILE.read_text()))


def footprint_body(body: BodySettings) -> BodySettings:
    """The floor-level body of the nav footprint: the body without the head post, head arm and head envelope.

    Args:
        body (BodySettings): Full body (load_robot_body).

    Returns:
        BodySettings: Boxes and cylinders not in HEAD_OBSTACLES, in body order.
    """
    return BodySettings(
        boxes=[b for b in body.boxes if b.name not in HEAD_OBSTACLES],
        cylinders=[c for c in body.cylinders if c.name not in HEAD_OBSTACLES],
    )
