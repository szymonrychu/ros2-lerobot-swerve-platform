"""Grasp planner: pure geometry on the arm URDF (ArmKinematics), no ROS and no motion.

A strategy (registry STRATEGIES, name -> callable) turns an object into grasp geometry: where the tool point (the
fixed jaw's inner face) ends up, the approach direction and pitch, the wanted jaw orientation and opening. The planner
then realizes it as ordered waypoints (pre_grasp, open, approach, grasp, close, lift, retreat) with IK solutions and
interpolated joint samples for the straight-line segments, checks feasibility and annotates the below-surface slow
zone. Infeasible plans carry human-readable reasons; the planner never raises for an infeasible object.

The SO101 is a 5-DOF arm: the approach always lies in the vertical plane of shoulder_pan, so every strategy
approaches radially from the arm base; the base has to turn for any other approach yaw.

Surface regions (SurfaceModel.terrain, from the per-call 'surfaces': stairs, table tops, holes) make every planned
sample a hard check: the jaw points and the forearm / wrist link capsules must stay clear of every surface and of the
vertical step faces between surfaces (reasons name the step edge). For an object beyond a step (a higher surface
between the arm and the object) the planner adds candidates with a pre-grasp above the upper surface and steeper
approach pitches, and the object's support_z must match the region under it. Without surfaces nothing of this runs.
"""

import math
from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Any, Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, ValidationError

from .config import GraspSettings, GripProfileOverride, LimitSettings, McpServerConfig
from .floor_guard import FloorGuard, JawModel, SurfaceModel, arm_to_base_link, base_link_to_arm, rotate
from .ik import ArmKinematics, UnreachableError, grasp_offset
from .surfaces import Clearance, SurfaceRegion

GraspParams = GraspSettings  # per-call grasp parameters: the configured defaults with overrides applied
AUTO = "auto"
ROLL_JOINT = "wrist_roll"
SHOULDER_LIFT = "shoulder_lift"
ELBOW_FLEX = "elbow_flex"
TOOL_FRAME_LINK = "gripper_frame_link"
ROLL_PROBE_RAD = 0.5
UP = np.array([0.0, 0.0, 1.0])
DOWN = np.array([0.0, 0.0, -1.0])
TOP_DOWN_PITCH_RAD = math.pi / 2
MIN_DIRECTION_NORM = 1e-6
# Waypoint labels in execution order.
WAYPOINT_ORDER = ("pre_grasp", "open", "approach", "grasp", "close", "lift", "retreat")
LINEAR_LABELS = ("approach", "grasp", "lift", "retreat")
# Waypoints checked for reachability before the straight lines to them are interpolated, hardest first (the grasp is
# the lowest point; the lifted retreat and lift are the closest to the base, where a pitched gripper runs out of
# reach; approach and pre_grasp are checked by their straight lines).
KEY_LABELS = ("grasp", "retreat", "lift")
# Effort caps in ikpy runs (about 3 ms each on a Mac, 4 to 5 times that on the RPi 5). The largest feasible plan seen
# (angled 45 deg on a 0 m ledge at x 0.25) needs about 2650; a candidate or strategy beyond the cap is reported infeasible
# instead of searching until the caller times out.
MAX_IK_SOLVES_PER_CANDIDATE = 3200
MAX_IK_SOLVES_PER_STRATEGY = 8000
# Extra strategy budget factor when the step candidates (object beyond a step) are tried as well.
STEP_BUDGET_FACTOR = 3
# Link bodies checked against surface regions as capsules between FK link origins (radius = half the STS3215 servo
# body that each link carries, about 4 cm across).
FOREARM_CAPSULE = ("forearm", "elbow", "wrist", 0.02)
WRIST_CAPSULE = ("wrist link", "wrist", "gripper", 0.02)
JAW_POINTS = ("jaw_tip", "tool_point", "moving_jaw_tip")
GRIPPER_BODY = "gripper body"
# Hull of the gripper body and fixed jaw in gripper_link (m): the corners of the housing collision boxes and the extreme
# points of the fixed finger mesh per 1 cm slice, taken from the grasp_sim MuJoCo model with the jaws calibrated to
# the 2026-10-10 tool point (its gripper body frame is gripper_link). Checked against surfaces with
# surface_jaw_clearance_m: at a diagonal step edge the jaw body, not its tip, is what touches first.
GRIPPER_BODY_POINTS = np.array(
    [
        (-0.038, -0.015, -0.038),
        (-0.037, -0.021, -0.040),
        (-0.037, -0.021, -0.030),
        (-0.037, 0.009, -0.040),
        (-0.037, 0.009, -0.030),
        (-0.035, -0.015, -0.037),
        (-0.035, -0.015, -0.007),
        (-0.035, 0.015, -0.037),
        (-0.035, 0.015, -0.007),
        (-0.034, -0.012, -0.049),
        (-0.034, 0.000, -0.049),
        (-0.023, -0.002, -0.081),
        (-0.020, -0.022, -0.033),
        (-0.020, 0.010, -0.033),
        (-0.019, -0.009, -0.091),
        (-0.018, 0.005, -0.039),
        (-0.017, -0.021, -0.040),
        (-0.017, -0.021, -0.030),
        (-0.017, 0.009, -0.040),
        (-0.017, 0.009, -0.030),
        (-0.016, -0.015, -0.056),
        (-0.016, -0.014, -0.054),
        (-0.016, 0.002, -0.058),
        (-0.016, 0.002, -0.054),
        (-0.015, -0.012, -0.091),
        (-0.014, -0.013, -0.087),
        (-0.014, 0.000, -0.090),
        (-0.014, 0.001, -0.083),
        (-0.013, -0.001, -0.090),
        (-0.012, -0.006, -0.099),
        (0.030, -0.015, -0.037),
        (0.030, -0.015, -0.007),
        (0.030, 0.015, -0.037),
        (0.030, 0.015, -0.007),
    ]
)
MIN_STEP_M = 1e-3

WaypointLabel = Literal["pre_grasp", "open", "approach", "grasp", "close", "lift", "retreat"]
GripperAction = Literal["set", "close", "keep"]


class ObjectSpec(BaseModel):
    """Object to grasp, as a box standing on a support.

    Attributes:
        frame: 'arm' (arm base frame, z = 0 on the mount plane) or 'base_link' (robot frame, z = 0 under the wheels).
        x: Object centre x (m).
        y: Object centre y (m).
        support_z: Height of the object's bottom (the surface it rests on) (m).
        width_m: Size across the jaws for angled/top_down grasps (m).
        depth_m: Size along the approach (m).
        height_m: Vertical size (m); a scoop closes the jaws on it.
        yaw: Direction of the object's width axis in `frame` (rad); None = across the approach direction.
        gap_below_m: Clear height under the object's bottom along the approach (m): the object overhangs a ledge or
            rests on something narrower than itself. 0 = flat on its support, which rules out a scoop.
        surfaces: Surface regions around the object (heights relative to the robot floor); the grasp tools add them
            after the call-level surfaces, so they shape the plan and the slow zone of the whole grasp.
    """

    model_config = ConfigDict(extra="forbid")

    frame: Literal["arm", "base_link"] = "arm"
    x: float
    y: float
    support_z: float
    width_m: float = Field(gt=0.0, le=0.5)
    depth_m: float = Field(gt=0.0, le=0.5)
    height_m: float = Field(gt=0.0, le=0.5)
    yaw: float | None = None
    gap_below_m: float = Field(default=0.0, ge=0.0, le=0.5)
    surfaces: list[SurfaceRegion] | None = Field(
        default=None,
        max_length=16,
        description="Surface regions around the object (stair, table, hole); added after the call's surfaces",
    )


class Waypoint(BaseModel):
    """One plan step: the tool point target (arm frame), pitch/roll, gripper command and speed."""

    label: WaypointLabel
    x: float
    y: float
    z: float
    pitch: float
    roll: float
    gripper: float | None = Field(description="Gripper target (rad) for 'set', the expected hold angle for 'close'")
    gripper_action: GripperAction
    speed_scale: float
    linear: bool = Field(description="Reached along a straight line through the interpolated joint samples")
    joints: dict[str, float] = Field(description="IK solution of the five arm joints (measured space)")
    tool_point: dict[str, float] | None = Field(
        default=None,
        description="Tool point (fixed jaw inner face, arm frame, m) at this waypoint. x, y, z is the jaw centre "
        "(the object centre) when the plan centres the object between the jaws (grasp_shift), else the tool point",
    )


class GraspPlan(BaseModel):
    """Planner output: feasibility, reasons, ordered waypoints, straight-line joint samples and annotations."""

    strategy: str
    feasible: bool
    reasons: list[str] = Field(default_factory=list)
    object_arm: ObjectSpec | None = None
    approach_pitch_rad: float | None = None
    wrist_roll_rad: float | None = None
    opening_m: float | None = None
    open_gripper_rad: float | None = None
    half_open_gripper_rad: float | None = None
    center_width_m: float | None = Field(default=None, description="Object width centred between the jaws, if any")
    grasp_shift: dict[str, Any] | None = Field(
        default=None,
        description="Centred grasp: {object_width_m, shift_m (half the width), jaw_open_axis (gripper_frame_link)}. "
        "Waypoint x, y, z are then the jaw centre; the tool point (fixed jaw) is shift_m beside it, against the "
        "opening direction, so the arm sits sideways of a plain move to the same x, y, z (a 6.5 cm object at 0.37 m "
        "reach: about 0.09 rad more shoulder_pan, 3.3 cm off at the tool point). Null when x, y, z is the tool point",
    )
    skim: bool = False
    waypoints: list[Waypoint] = Field(default_factory=list)
    segments: dict[str, list[dict[str, float]]] = Field(default_factory=dict)
    slow_zone: list[dict[str, Any]] = Field(default_factory=list)
    surface: dict[str, Any] | None = None
    attempts: list[dict[str, Any]] = Field(default_factory=list)
    rejected_candidates: list[str] = Field(
        default_factory=list, description="Why candidates tried before the chosen one were rejected (surfaces only)"
    )

    def summary(self) -> dict[str, Any]:
        """JSON-ready plan summary without the joint samples.

        Returns:
            dict[str, Any]: strategy, feasibility, reasons, key values, waypoints (no joints), slow zone, attempts.
        """
        out: dict[str, Any] = {
            "strategy": self.strategy,
            "feasible": self.feasible,
            "reasons": self.reasons,
            "object_arm": None if self.object_arm is None else self.object_arm.model_dump(),
            "approach_pitch_deg": None
            if self.approach_pitch_rad is None
            else round(math.degrees(self.approach_pitch_rad), 1),
            "wrist_roll_rad": None if self.wrist_roll_rad is None else round(self.wrist_roll_rad, 3),
            "opening_m": None if self.opening_m is None else round(self.opening_m, 4),
            "open_gripper_rad": None if self.open_gripper_rad is None else round(self.open_gripper_rad, 3),
            "skim": self.skim,
            "grasp_shift": self.grasp_shift,
            "waypoints": [
                {k: (round(v, 4) if isinstance(v, float) else v) for k, v in w.model_dump(exclude={"joints"}).items()}
                for w in self.waypoints
            ],
            "slow_zone": self.slow_zone,
            "surface": self.surface,
            "attempts": self.attempts,
        }
        if self.rejected_candidates:
            out["rejected_candidates"] = self.rejected_candidates
        return out


@dataclass(frozen=True)
class GraspRequest:
    """What a strategy sees: the object in the arm frame, parameters and the surface under the object."""

    obj: ObjectSpec
    params: GraspParams
    heading: float  # horizontal direction from the shoulder pan axis to the object centre (rad)
    surface_z: float  # effective surface height under the object centre, arm frame (m)
    approach_pitch_deg: float | None  # per-call pitch (angled)


@dataclass(frozen=True)
class GraspGeometry:
    """One candidate grasp a strategy proposes (the planner tries candidates in order).

    Attributes:
        pitch: Approach pitch of the gripper (rad, + down).
        grasp: Tool point target at the grasp (object centre when center_width is set), arm frame.
        approach_dir: Unit direction of the final approach/slide into the grasp.
        approach_len: Length of that straight approach (m).
        jaw_dir: Wanted world direction of the jaw opening (moving jaw away from the fixed jaw).
        jaw_sign_matters: True when jaw_dir must not be flipped (scoop: moving jaw on top).
        opening_m: Jaw gap needed to pass the object (m).
        gripped_m: Expected gap once closed on the object (m).
        center_width: Object width to centre between the jaws (grasp_offset), None to place the tool point itself.
        skim: The fixed jaw skims the surface instead of going below the object bottom.
        lift_speed: Speed scale of the lift (None = slide_speed_scale); tall narrow objects lift slower.
        pre_grasp_clearance: Lift of the pre-grasp above the approach start (m); None = params.pre_grasp_clearance_m.
            Raised for an object beyond a step so the arm descends from above the upper surface.
        lift_height: Lift of the grasped object before the retreat (m); None = params.lift_height_m. Raised for an
            object beyond a step so the retreat does not drag the gripper over the step edge.
    """

    pitch: float
    grasp: tuple[float, float, float]
    approach_dir: tuple[float, float, float]
    approach_len: float
    jaw_dir: tuple[float, float, float]
    jaw_sign_matters: bool
    opening_m: float
    gripped_m: float
    center_width: float | None = None
    skim: bool = False
    lift_speed: float | None = None
    pre_grasp_clearance: float | None = None
    lift_height: float | None = None


@dataclass
class IkBudget:
    """Cap on the ikpy runs one grasp candidate may spend.

    Attributes:
        kin: Kinematics whose solve_count is watched.
        limit: Most ikpy runs allowed from creation.
        start: solve_count at creation.
    """

    kin: ArmKinematics
    limit: int
    start: int = 0

    def __post_init__(self) -> None:
        self.start = self.kin.solve_count

    def spent(self) -> bool:
        """Whether the cap has been used up.

        Returns:
            bool: True once limit ikpy runs have happened since creation.
        """
        return self.kin.solve_count - self.start >= self.limit


class Ineligible(ValueError):
    """Raised by a strategy that cannot handle the object at all; the message is the plan's reason."""


StrategyFn = Callable[[GraspRequest], list[GraspGeometry]]


def radial(heading: float) -> np.ndarray:
    """Horizontal unit vector of a heading.

    Args:
        heading (float): Heading (rad).

    Returns:
        np.ndarray: (cos, sin, 0).
    """
    return np.array([math.cos(heading), math.sin(heading), 0.0])


def width_axis(req: GraspRequest) -> np.ndarray:
    """Horizontal direction of the object's width axis (across the approach when the object yaw is unknown).

    Args:
        req (GraspRequest): Request.

    Returns:
        np.ndarray: Unit vector.
    """
    yaw = req.obj.yaw if req.obj.yaw is not None else req.heading + math.pi / 2
    return radial(yaw)


def as_tuple(v: np.ndarray) -> tuple[float, float, float]:
    """3-vector as a float tuple.

    Args:
        v (np.ndarray): Vector.

    Returns:
        tuple[float, float, float]: Components.
    """
    return (float(v[0]), float(v[1]), float(v[2]))


def scoop_strategy(req: GraspRequest) -> list[GraspGeometry]:
    """Fixed jaw underneath (wrist_roll about 0), moving jaw closes from above, horizontal radial slide.

    Only for objects with room under them: gap_below_m must hold the fixed jaw (jaw_thickness_m plus
    scoop_gap_margin_m). Against an object resting flat the jaw cannot get under it; it pushes or tips the object
    (validated in the MuJoCo sim, sim/README.md). The fixed jaw top goes below_object_offset_m under the object bottom; when that would put the jaw into the
    surface the object rests on, it skims at surface + skim_clearance_m instead. Pitches from scoop_pitch_deg up to
    scoop_max_pitch_deg are offered in order (a near-horizontal gripper cannot reach low near the base).

    Args:
        req (GraspRequest): Request.

    Returns:
        list[GraspGeometry]: Candidates, flattest pitch first.

    Raises:
        Ineligible: When the gap under the object is too small for the fixed jaw.
    """
    p, obj = req.params, req.obj
    needed = p.jaw_thickness_m + p.scoop_gap_margin_m
    if obj.gap_below_m < needed:
        raise Ineligible(
            f"no gap under object for the fixed jaw: gap_below_m {obj.gap_below_m * 100:.1f} cm, the scoop needs "
            f"jaw_thickness_m + scoop_gap_margin_m = {needed * 100:.1f} cm (an overhang or a raised object)"
        )
    jaw_top = obj.support_z - p.below_object_offset_m
    skim = jaw_top - p.jaw_thickness_m < req.surface_z + p.skim_clearance_m
    if skim:
        jaw_top = req.surface_z + p.skim_clearance_m + p.jaw_thickness_m
    raised = max(0.0, jaw_top - obj.support_z)  # the object rides up onto a skimming jaw
    out: list[GraspGeometry] = []
    pitch_deg = p.scoop_pitch_deg
    while pitch_deg <= p.scoop_max_pitch_deg + 1e-9:
        out.append(
            GraspGeometry(
                pitch=math.radians(pitch_deg),
                grasp=(obj.x, obj.y, jaw_top),
                approach_dir=as_tuple(radial(req.heading)),
                approach_len=p.approach_distance_m + obj.depth_m / 2.0,
                jaw_dir=(0.0, 0.0, 1.0),
                jaw_sign_matters=True,
                opening_m=obj.height_m + p.jaw_open_margin_m + raised,
                gripped_m=obj.height_m + raised,
                skim=skim,
            )
        )
        pitch_deg += p.scoop_pitch_step_deg
    return out


def centred_grasp(req: GraspRequest, pitch: float, approach: np.ndarray, extent: float) -> GraspGeometry:
    """Grasp across the object width with the object centred between the jaws (angled and top_down).

    The tool point goes to mid-height of the object; a tall narrow object (height / width above tall_ratio) is
    gripped lower, at tall_grasp_height_fraction of its height, and lifted at lift_speed_scale so it does not pivot
    out of the jaws. Never lower than a jaw thickness plus the skim clearance above the surface.

    Args:
        req (GraspRequest): Request.
        pitch (float): Approach pitch (rad).
        approach (np.ndarray): Unit approach direction.
        extent (float): Object extent along the approach before its centre (m).

    Returns:
        GraspGeometry: The grasp.
    """
    p, obj = req.params, req.obj
    tall = obj.height_m / obj.width_m > p.tall_ratio
    fraction = p.tall_grasp_height_fraction if tall else 0.5
    z = max(obj.support_z + obj.height_m * fraction, req.surface_z + p.skim_clearance_m + p.jaw_thickness_m)
    return GraspGeometry(
        pitch=pitch,
        grasp=(obj.x, obj.y, z),
        approach_dir=as_tuple(approach),
        approach_len=p.approach_distance_m + extent,
        jaw_dir=as_tuple(width_axis(req)),
        jaw_sign_matters=False,
        opening_m=obj.width_m + p.jaw_open_margin_m,
        gripped_m=obj.width_m,
        center_width=obj.width_m,
        lift_speed=p.lift_speed_scale if tall else None,
    )


def angled_strategy(req: GraspRequest) -> list[GraspGeometry]:
    """Radial approach pitched down (approach_pitch_deg, default angled_pitch_deg), jaws across the object width.

    Args:
        req (GraspRequest): Request.

    Returns:
        list[GraspGeometry]: One candidate.
    """
    pitch = math.radians(req.params.angled_pitch_deg if req.approach_pitch_deg is None else req.approach_pitch_deg)
    approach = math.cos(pitch) * radial(req.heading) + math.sin(pitch) * DOWN
    extent = req.obj.depth_m / 2.0 * math.cos(pitch) + req.obj.height_m / 2.0 * math.sin(pitch)
    return [centred_grasp(req, pitch, approach, extent)]


def top_down_strategy(req: GraspRequest) -> list[GraspGeometry]:
    """Gripper pointing straight down, jaws across the object width, vertical approach.

    Args:
        req (GraspRequest): Request.

    Returns:
        list[GraspGeometry]: One candidate.
    """
    return [centred_grasp(req, TOP_DOWN_PITCH_RAD, DOWN, req.obj.height_m / 2.0)]


def over_step(geo: GraspGeometry, upper_z: float, params: GraspParams) -> GraspGeometry:
    """Candidate whose pre-grasp and lift lie at least step_pre_grasp_clearance_m above an upper surface: the arm
    descends to the object from above the step and lifts it above the step before retreating over the edge.

    Args:
        geo (GraspGeometry): Candidate.
        upper_z (float): Highest surface between the arm and the object (arm frame z, m).
        params (GraspParams): pre_grasp_clearance_m, lift_height_m, step_pre_grasp_clearance_m.

    Returns:
        GraspGeometry: The candidate with pre_grasp_clearance and lift_height set (never below the configured ones).
    """
    above = upper_z + params.step_pre_grasp_clearance_m
    approach_z = geo.grasp[2] - geo.approach_dir[2] * geo.approach_len
    return replace(
        geo,
        pre_grasp_clearance=max(params.pre_grasp_clearance_m, above - approach_z),
        lift_height=max(params.lift_height_m, above - geo.grasp[2]),
    )


def step_candidates(
    strategy: str, fn: "StrategyFn", req: GraspRequest, base: list[GraspGeometry], upper_z: float
) -> list[GraspGeometry]:
    """Extra candidates for an object beyond a step: the same geometry with the pre-grasp and lift above the step,
    then (angled) steeper approach pitches from params.step_pitches_deg, also over the step.

    Args:
        strategy (str): Strategy name.
        fn (StrategyFn): The strategy.
        req (GraspRequest): Request.
        base (list[GraspGeometry]): The strategy's own candidates.
        upper_z (float): Highest surface between the arm and the object (arm frame z, m).

    Returns:
        list[GraspGeometry]: New candidates (none repeating a base one), in the order to try.
    """
    p = req.params
    out = [over_step(g, upper_z, p) for g in base]
    if strategy == "angled":
        current = p.angled_pitch_deg if req.approach_pitch_deg is None else req.approach_pitch_deg
        for pitch in p.step_pitches_deg:
            if pitch > current + 1e-9:
                out.extend(over_step(g, upper_z, p) for g in fn(replace(req, approach_pitch_deg=pitch)))
    unique: list[GraspGeometry] = []
    for g in out:
        if g not in base and g not in unique:
            unique.append(g)
    return unique


def candidate_label(geo: GraspGeometry) -> str:
    """Reason prefix of a candidate: its pitch, and the raised pre-grasp when there is one.

    Args:
        geo (GraspGeometry): Candidate.

    Returns:
        str: E.g. 'pitch 45 deg' or 'pitch 65 deg, pre-grasp 9.2 cm up'.
    """
    label = f"pitch {math.degrees(geo.pitch):.0f} deg"
    if geo.pre_grasp_clearance is not None:
        label += f", pre-grasp {geo.pre_grasp_clearance * 100:.1f} cm up"
    if geo.lift_height is not None:
        label += f", lift {geo.lift_height * 100:.1f} cm"
    return label


def clearance_reason(label: str, part: str, clearance: Clearance, need: float) -> str:
    """Reason for a sample too close to a surface or step face.

    Args:
        label (str): Waypoint label.
        part (str): Checked part (e.g. 'wrist link').
        clearance (Clearance): Its clearance and feature.
        need (float): Required clearance (m).

    Returns:
        str: The reason.
    """
    return (
        f"{label}: {part} clearance {clearance.value * 100:.1f} cm to the {clearance.feature}, "
        f"needs {need * 100:.1f} cm"
    )


STRATEGIES: dict[str, StrategyFn] = {
    "scoop": scoop_strategy,
    "angled": angled_strategy,
    "top_down": top_down_strategy,
}


def grasp_params(
    settings: GraspSettings,
    overrides: dict[str, Any] | None,
    grip_profile: str | GripProfileOverride | dict[str, Any] | None = None,
) -> GraspParams:
    """Grasp parameters: the configured defaults with per-call overrides, validated.

    Args:
        settings (GraspSettings): Configured defaults.
        overrides (dict[str, Any] | None): Field -> value.
        grip_profile (str | GripProfileOverride | dict[str, Any] | None): Grip profile of the call (a top-level tool
            or request argument); replaces overrides['grip_profile'] when given. The name is checked when it is
            applied (GraspExecutor.resolve_grip).

    Returns:
        GraspParams: Validated parameters.

    Raises:
        ValueError: For unknown keys or invalid values.
    """
    if grip_profile is not None:
        if isinstance(grip_profile, GripProfileOverride):
            grip_profile = grip_profile.model_dump(exclude_unset=True)
        overrides = {**(overrides or {}), "grip_profile": grip_profile}
    if not overrides:
        return settings
    try:
        return GraspSettings.model_validate({**settings.model_dump(), **overrides})
    except ValidationError as exc:
        raise ValueError(f"invalid grasp params: {exc}") from exc


def is_stall_pose(joints: dict[str, float], params: GraspParams) -> bool:
    """Stretched arm (elbow_flex at or below stretched_elbow_max_rad) lifted beyond stall_shoulder_lift_rad.

    Args:
        joints (dict[str, float]): Measured joint angles.
        params (GraspParams): Thresholds.

    Returns:
        bool: True for a pose the shoulder servo cannot hold.
    """
    return (
        joints[SHOULDER_LIFT] > params.stall_shoulder_lift_rad and joints[ELBOW_FLEX] <= params.stretched_elbow_max_rad
    )


def roll_guard_violations(waypoints: list[Waypoint], limits: LimitSettings) -> list[str]:
    """Check that the wrist roll changes only at the lifted pre-grasp, with the gripper at most half open.

    Args:
        waypoints (list[Waypoint]): Plan waypoints in order.
        limits (LimitSettings): roll_guard_min_change_rad and roll_max_gripper_open_rad.

    Returns:
        list[str]: Violations (empty when the plan respects the roll guard).
    """
    out: list[str] = []
    if waypoints and waypoints[0].label == "pre_grasp":
        first = waypoints[0]
        if first.gripper is None or first.gripper > limits.roll_max_gripper_open_rad:
            out.append(
                f"roll guard: the gripper must be at most {limits.roll_max_gripper_open_rad} rad open at the pre-grasp "
                "where the wrist rolls"
            )
    for prev, cur in zip(waypoints, waypoints[1:], strict=False):
        if abs(cur.roll - prev.roll) > limits.roll_guard_min_change_rad:
            out.append(
                f"roll guard: wrist roll changes between {prev.label} and {cur.label}; it may only change at the lifted pre_grasp"
            )
    return out


class GraspPlanner:
    """Plans grasps on the arm kinematics; see the module docstring."""

    def __init__(self, kin: ArmKinematics, config: McpServerConfig, guard: FloorGuard, jaw: JawModel) -> None:
        """Bind the planner to the arm model.

        Args:
            kin (ArmKinematics): Kinematics on the arm URDF.
            config (McpServerConfig): Node configuration (arm, limits).
            guard (FloorGuard): Slow-zone evaluator for the annotations.
            jaw (JawModel): Jaw opening model.
        """
        self.kin = kin
        self.cfg = config
        self.guard = guard
        self.jaw = jaw

    def center_offset(self, plan: GraspPlan) -> tuple[float, float, float] | None:
        """Extra tool offset that makes the waypoint targets the object centre (None when the tool point is the target).

        Args:
            plan (GraspPlan): A plan.

        Returns:
            tuple[float, float, float] | None: Offset in gripper_frame_link (m).
        """
        if plan.center_width_m is None:
            return None
        return grasp_offset(plan.center_width_m, self.cfg.arm.jaw_open_axis)

    def to_arm(self, obj: ObjectSpec) -> ObjectSpec:
        """Object in the arm base frame.

        Args:
            obj (ObjectSpec): Object in 'arm' or 'base_link'.

        Returns:
            ObjectSpec: The object with frame 'arm'.

        Raises:
            ValueError: For a base_link object without a configured arm mount.
        """
        if obj.frame == "arm":
            return obj
        mount = self.cfg.arm.base_in_base_link
        if mount is None:
            raise ValueError("object given in base_link but arm.base_in_base_link (the arm mount) is not configured")
        centre = base_link_to_arm((obj.x, obj.y, obj.support_z), mount)
        yaw = None if obj.yaw is None else obj.yaw - mount.yaw
        return obj.model_copy(
            update={
                "frame": "arm",
                "x": float(centre[0]),
                "y": float(centre[1]),
                "support_z": float(centre[2]),
                "yaw": yaw,
            }
        )

    def plan(
        self,
        obj: ObjectSpec,
        strategy: str,
        params: GraspParams,
        surface: SurfaceModel,
        seed: dict[str, float],
        approach_pitch_deg: float | None = None,
    ) -> GraspPlan:
        """Plan a grasp; 'auto' tries params.auto_order and returns the first feasible plan.

        Args:
            obj (ObjectSpec): Object.
            strategy (str): A STRATEGIES name or 'auto'.
            params (GraspParams): Grasp parameters.
            surface (SurfaceModel): Effective surface (expected height, tilt) for skimming and the slow zone.
            seed (dict[str, float]): IK seed (typically the current arm pose, measured space).
            approach_pitch_deg (float | None): Approach pitch for 'angled' (default params.angled_pitch_deg).

        Returns:
            GraspPlan: Feasible plan, or an infeasible one with reasons (never raises for an infeasible object).
        """
        try:
            arm_obj = self.to_arm(obj)
        except ValueError as exc:
            return GraspPlan(strategy=strategy, feasible=False, reasons=[str(exc)])
        if strategy != AUTO:
            return self.plan_strategy(arm_obj, strategy, params, surface, seed, approach_pitch_deg)
        attempts: list[dict[str, Any]] = []
        reasons: list[str] = []
        for entry in params.auto_order:
            pitch = approach_pitch_deg if entry.approach_pitch_deg is None else entry.approach_pitch_deg
            result = self.plan_strategy(arm_obj, entry.strategy, params, surface, seed, pitch)
            attempts.append({"strategy": entry.strategy, "feasible": result.feasible, "reasons": result.reasons})
            if result.feasible:
                return result.model_copy(update={"attempts": attempts})
            reasons.extend(f"{entry.strategy}: {r}" for r in result.reasons)
        return GraspPlan(strategy=AUTO, feasible=False, reasons=reasons, object_arm=arm_obj, attempts=attempts)

    def plan_strategy(
        self,
        obj: ObjectSpec,
        strategy: str,
        params: GraspParams,
        surface: SurfaceModel,
        seed: dict[str, float],
        approach_pitch_deg: float | None,
    ) -> GraspPlan:
        """Plan one named strategy: try its candidates in order, first feasible wins.

        Args:
            obj (ObjectSpec): Object in the arm frame.
            strategy (str): Strategy name.
            params (GraspParams): Parameters.
            surface (SurfaceModel): Effective surface.
            seed (dict[str, float]): IK seed.
            approach_pitch_deg (float | None): Pitch for 'angled'.

        Returns:
            GraspPlan: The plan.
        """
        fn = STRATEGIES.get(strategy)
        if fn is None:
            return GraspPlan(
                strategy=strategy,
                feasible=False,
                object_arm=obj,
                reasons=[f"unknown strategy {strategy!r}; use one of {[*STRATEGIES, AUTO]}"],
            )
        heading = math.atan2(obj.y - self.kin.pan_axis_xy[1], obj.x - self.kin.pan_axis_xy[0])
        centre = np.array([obj.x, obj.y, obj.support_z])
        if surface.terrain is None:
            surface_z = obj.support_z - surface.clearance(centre)
        else:
            mismatch = self.support_mismatch(obj, surface, params)
            if mismatch is not None:
                return GraspPlan(
                    strategy=strategy, feasible=False, reasons=[mismatch], object_arm=obj, surface=surface.describe()
                )
            surface_z = surface.surface_z_arm(centre)
        req = GraspRequest(obj, params, heading, surface_z, approach_pitch_deg)
        reasons: list[str] = []
        try:
            candidates = fn(req)
        except Ineligible as exc:
            return GraspPlan(
                strategy=strategy, feasible=False, reasons=[str(exc)], object_arm=obj, surface=surface.describe()
            )
        upper_z = self.upper_surface_z(obj, surface)
        extra = [] if upper_z is None else step_candidates(strategy, fn, req, candidates, upper_z)
        budget = MAX_IK_SOLVES_PER_STRATEGY * (STEP_BUDGET_FACTOR if extra else 1)
        start = self.kin.solve_count
        for geo in candidates + extra:
            if reasons and self.kin.solve_count - start >= budget:
                reasons.append(f"IK budget of {budget} solves spent, remaining pitches not tried")
                break
            result = self.realize(strategy, obj, geo, params, surface, seed)
            if result.feasible:
                if surface.terrain is not None and reasons:
                    return result.model_copy(update={"rejected_candidates": reasons})
                return result
            prefix = candidate_label(geo) if surface.terrain is not None else f"pitch {math.degrees(geo.pitch):.0f} deg"
            for r in result.reasons:
                reason = f"{prefix}: {r}"
                if reason not in reasons:
                    reasons.append(reason)
        return GraspPlan(strategy=strategy, feasible=False, reasons=reasons, object_arm=obj, surface=surface.describe())

    def support_mismatch(self, obj: ObjectSpec, surface: SurfaceModel, params: GraspParams) -> str | None:
        """Check that the object's support_z (minus gap_below_m) is the region height under its centre.

        Args:
            obj (ObjectSpec): Object in the arm frame.
            surface (SurfaceModel): Surface with terrain.
            params (GraspParams): surface_mismatch_tolerance_m.

        Returns:
            str | None: The mismatch as a reason, or None when consistent (or without surfaces).
        """
        terrain = surface.terrain
        if terrain is None:
            return None
        p = arm_to_base_link((obj.x, obj.y, obj.support_z), surface.mount)
        height = terrain.height_at(p[:2])
        region = terrain.region_at(p[:2])
        local_z = height - surface.mount.z
        bottom = obj.support_z - obj.gap_below_m
        if abs(bottom - local_z) <= params.surface_mismatch_tolerance_m:
            return None
        where = "the robot floor (no region)" if region is None else f"region '{region}'"
        gap = f" minus gap_below_m {obj.gap_below_m:g}" if obj.gap_below_m > 0.0 else ""
        return (
            f"object support_z {obj.support_z:.3f} m{gap} does not match the surface under the object: {where} at "
            f"{height:+.3f} m relative to the robot floor = arm z {local_z:.3f} m (tolerance "
            f"{params.surface_mismatch_tolerance_m:g} m); fix support_z or the surfaces"
        )

    def upper_surface_z(self, obj: ObjectSpec, surface: SurfaceModel) -> float | None:
        """Highest surface between the shoulder pan axis and the object when it is above the object's surface.

        Args:
            obj (ObjectSpec): Object in the arm frame.
            surface (SurfaceModel): Surface (terrain optional).

        Returns:
            float | None: That surface in arm frame z (m), None without surfaces or without a step up on the way.
        """
        terrain = surface.terrain
        if terrain is None:
            return None
        pan = arm_to_base_link((self.kin.pan_axis_xy[0], self.kin.pan_axis_xy[1], 0.0), surface.mount)[:2]
        target = arm_to_base_link((obj.x, obj.y, 0.0), surface.mount)[:2]
        height = terrain.height_at(pan)
        upper = height
        for crossing in terrain.steps_between(pan, target):
            height -= crossing.drop_m
            upper = max(upper, height)
        local = terrain.height_at(target)
        if upper <= local + MIN_STEP_M:
            return None
        return upper - surface.mount.z

    def link_clearances(self, joints: dict[str, float], surface: SurfaceModel) -> dict[str, Clearance]:
        """Clearance of the forearm and wrist link capsules and of the gripper body hull against the surfaces.

        Args:
            joints (dict[str, float]): Arm joints (measured space).
            surface (SurfaceModel): Surface.

        Returns:
            dict[str, Clearance]: 'forearm' and 'wrist link' (capsule radius subtracted) and 'gripper body'.
        """
        frames = self.kin.link_frames(joints)
        gripper = frames["gripper_link"]
        origins = {
            "elbow": frames["lower_arm_link"][:3, 3],
            "wrist": frames["wrist_link"][:3, 3],
            "gripper": gripper[:3, 3],
        }
        out = {
            name: surface.capsule_clearance(origins[a], origins[b], radius)
            for name, a, b, radius in (FOREARM_CAPSULE, WRIST_CAPSULE)
        }
        out[GRIPPER_BODY] = surface.points_clearance(GRIPPER_BODY_POINTS @ gripper[:3, :3].T + gripper[:3, 3])
        return out

    def surface_problem(
        self, label: str, samples: list[dict[str, float]], surface: SurfaceModel, params: GraspParams
    ) -> str | None:
        """Check plan samples against the surface regions: jaw points and link capsules must keep their clearance.

        Args:
            label (str): Waypoint label.
            samples (list[dict[str, float]]): Joint samples including the gripper.
            surface (SurfaceModel): Surface; nothing is checked without terrain.
            params (GraspParams): surface_jaw_clearance_m, surface_link_clearance_m.

        Returns:
            str | None: The first violation as a reason, or None.
        """
        if surface.terrain is None:
            return None
        for sample in samples:
            points = self.guard.checked_points(sample)
            for name in JAW_POINTS:
                clearance = surface.clearance_detail(points[name])
                if clearance.value < params.surface_jaw_clearance_m:
                    return clearance_reason(label, name.replace("_", " "), clearance, params.surface_jaw_clearance_m)
            for name, clearance in self.link_clearances(sample, surface).items():
                need = params.surface_jaw_clearance_m if name == GRIPPER_BODY else params.surface_link_clearance_m
                if clearance.value < need:
                    return clearance_reason(label, name, clearance, need)
        return None

    def roll_for(self, joints: dict[str, float], jaw_dir: np.ndarray, sign_matters: bool) -> float | None:
        """Wrist roll that points the jaw opening along jaw_dir (projected normal to the approach axis).

        Rolling the wrist rotates the jaw axis about the approach axis (the tool frame z axis); the sense of that
        rotation is probed with forward kinematics. Among equivalent rolls (2 pi, or pi when the sign does not matter)
        the one inside the limits (minus margin) with the smallest magnitude is chosen.

        Args:
            joints (dict[str, float]): Arm pose (measured space) at the grasp.
            jaw_dir (np.ndarray): Wanted opening direction.
            sign_matters (bool): Whether the opposite direction is unacceptable.

        Returns:
            float | None: Roll (rad, measured space), or None when no equivalent roll lies within the limits.
        """
        axis = np.array(self.cfg.arm.jaw_open_axis)
        frame = self.kin.link_frame(joints, TOOL_FRAME_LINK)
        approach = frame[:3, 2]
        v0 = frame[:3, :3] @ axis
        probe = self.kin.link_frame(joints | {ROLL_JOINT: joints[ROLL_JOINT] + ROLL_PROBE_RAD}, TOOL_FRAME_LINK)
        v1 = probe[:3, :3] @ axis
        sense = (
            1.0
            if np.linalg.norm(rotate(v0, approach, ROLL_PROBE_RAD) - v1)
            < np.linalg.norm(rotate(v0, approach, -ROLL_PROBE_RAD) - v1)
            else -1.0
        )
        want = jaw_dir - approach * float(approach @ jaw_dir)
        if np.linalg.norm(want) < MIN_DIRECTION_NORM:
            return joints[ROLL_JOINT]
        want = want / np.linalg.norm(want)
        phi = math.atan2(float(approach @ np.cross(v0, want)), float(v0 @ want))
        base = joints[ROLL_JOINT] + sense * phi
        period = 2.0 * math.pi if sign_matters else math.pi
        offset = self.kin.offsets[ROLL_JOINT]
        lo, hi = self.kin.limits[ROLL_JOINT]
        lo, hi = lo - offset + self.kin.margin, hi - offset - self.kin.margin
        candidates = [base + k * period for k in range(-3, 4) if lo <= base + k * period <= hi]
        return min(candidates, key=abs) if candidates else None

    def realize(
        self,
        strategy: str,
        obj: ObjectSpec,
        geo: GraspGeometry,
        params: GraspParams,
        surface: SurfaceModel,
        seed: dict[str, float],
        budget: IkBudget | None = None,
    ) -> GraspPlan:
        """Turn one candidate geometry into waypoints and joint samples, checking feasibility.

        Args:
            strategy (str): Strategy name.
            obj (ObjectSpec): Object in the arm frame.
            geo (GraspGeometry): Candidate.
            params (GraspParams): Parameters.
            surface (SurfaceModel): Effective surface.
            seed (dict[str, float]): IK seed.
            budget (IkBudget | None): Effort cap for this candidate (default MAX_IK_SOLVES_PER_CANDIDATE).

        Returns:
            GraspPlan: Feasible plan or one with reasons.
        """
        if budget is None:
            budget = IkBudget(self.kin, MAX_IK_SOLVES_PER_CANDIDATE)
        self.kin.last_heading_bias = 0.0  # each candidate starts cold so a plan never depends on the one planned before
        base = GraspPlan(
            strategy=strategy,
            feasible=False,
            object_arm=obj,
            approach_pitch_rad=geo.pitch,
            opening_m=geo.opening_m,
            center_width_m=geo.center_width,
            grasp_shift=None
            if geo.center_width is None
            else {
                "object_width_m": round(geo.center_width, 4),
                "shift_m": round(geo.center_width / 2.0, 4),
                "jaw_open_axis": list(self.cfg.arm.jaw_open_axis),
            },
            skim=geo.skim,
            surface=surface.describe(),
        )
        reasons = self.opening_reasons(geo, params)
        if reasons:
            return base.model_copy(update={"reasons": reasons})
        open_angle = self.jaw.angle_for_gap(geo.opening_m)
        half = min(self.cfg.limits.roll_max_gripper_open_rad, open_angle)
        hold = self.jaw.angle_for_gap(geo.gripped_m)
        extra = None if geo.center_width is None else grasp_offset(geo.center_width, self.cfg.arm.jaw_open_axis)
        grasp = np.array(geo.grasp)
        direction = np.array(geo.approach_dir)
        approach = grasp - direction * geo.approach_len
        pre = approach + UP * (
            params.pre_grasp_clearance_m if geo.pre_grasp_clearance is None else geo.pre_grasp_clearance
        )
        lift = grasp + UP * (params.lift_height_m if geo.lift_height is None else geo.lift_height)
        horizontal = np.array([direction[0], direction[1], 0.0])
        if np.linalg.norm(horizontal) < MIN_DIRECTION_NORM:
            horizontal = radial(math.atan2(obj.y - self.kin.pan_axis_xy[1], obj.x - self.kin.pan_axis_xy[0]))
        retreat = lift - horizontal / np.linalg.norm(horizontal) * params.retreat_distance_m
        seed = {j: seed.get(j, 0.0) for j in self.kin.joint_names}
        try:
            first = self.kin.inverse(*grasp, geo.pitch, seed | {ROLL_JOINT: 0.0}, extra)
        except UnreachableError as exc:
            return base.model_copy(update={"reasons": [f"grasp unreachable: {exc}"]})
        roll = self.roll_for(first, np.array(geo.jaw_dir), geo.jaw_sign_matters)
        if roll is None:
            return base.model_copy(
                update={"reasons": ["the wrist roll that aligns the jaws is outside the roll limits"]}
            )
        fast = self.cfg.limits.arm_max_speed_scale
        slow = params.slide_speed_scale
        steps: list[tuple[WaypointLabel, np.ndarray, float | None, GripperAction, float, bool]] = [
            ("pre_grasp", pre, half, "set", fast, False),
            ("open", pre, open_angle, "set", fast, False),
            ("approach", approach, open_angle, "keep", slow, True),
            ("grasp", grasp, open_angle, "keep", slow, True),
            ("close", grasp, hold, "close", slow, False),
            ("lift", lift, hold, "keep", geo.lift_speed or slow, True),
            ("retreat", retreat, hold, "keep", slow, True),
        ]
        # Key waypoints first: an unreachable one rejects the plan before any straight-line sample is interpolated.
        key_seed = first | {ROLL_JOINT: roll}
        points = {label: point for label, point, *_ in steps}
        jaws = {label: open_angle if gripper is None else gripper for label, _, gripper, *_ in steps}
        for label in KEY_LABELS:
            solved, problem = self.solve_point(label, points[label], geo.pitch, key_seed, extra, params, budget)
            if problem is None:
                problem = self.surface_problem(
                    label, [s | {self.guard.gripper: jaws[label]} for s in solved], surface, params
                )
            if problem is not None:
                return base.model_copy(update={"reasons": [problem], "wrist_roll_rad": roll})
        waypoints: list[Waypoint] = []
        segments: dict[str, list[dict[str, float]]] = {}
        annotations: list[dict[str, Any]] = []
        joints = seed | {ROLL_JOINT: roll}
        prev_point: np.ndarray | None = None
        for label, point, gripper, action, speed, linear in steps:
            if linear and prev_point is not None:
                samples, problem = self.line(label, prev_point, point, geo.pitch, joints, extra, params, budget)
            else:
                samples, problem = self.solve_point(label, point, geo.pitch, joints, extra, params, budget)
            if problem is not None:
                return base.model_copy(update={"reasons": [problem], "wrist_roll_rad": roll})
            joints = samples[-1]
            jaw = gripper if gripper is not None else open_angle
            with_jaw = [s | {self.guard.gripper: jaw} for s in samples]
            problem = self.surface_problem(label, with_jaw, surface, params)
            if problem is not None:
                return base.model_copy(update={"reasons": [problem], "wrist_roll_rad": roll})
            if linear:
                segments[label] = with_jaw
            annotations.append(self.annotate(label, with_jaw, surface))
            waypoints.append(
                Waypoint(
                    label=label,
                    x=float(point[0]),
                    y=float(point[1]),
                    z=float(point[2]),
                    pitch=geo.pitch,
                    roll=roll,
                    gripper=gripper,
                    gripper_action=action,
                    speed_scale=speed,
                    linear=linear,
                    joints=dict(joints),
                    tool_point=self.tool_point(joints),
                )
            )
            prev_point = point
        reasons = roll_guard_violations(waypoints, self.cfg.limits)
        return base.model_copy(
            update={
                "feasible": not reasons,
                "reasons": reasons,
                "wrist_roll_rad": roll,
                "open_gripper_rad": open_angle,
                "half_open_gripper_rad": half,
                "waypoints": waypoints,
                "segments": segments,
                "slow_zone": annotations,
            }
        )

    def tool_point(self, joints: dict[str, float]) -> dict[str, float]:
        """Tool point (fixed jaw inner face) of a joint configuration.

        Args:
            joints (dict[str, float]): Arm joints (measured space).

        Returns:
            dict[str, float]: x, y, z in the arm frame (m, rounded to 0.1 mm).
        """
        pose = self.kin.forward(joints)
        return {"x": round(pose.x, 4), "y": round(pose.y, 4), "z": round(pose.z, 4)}

    def opening_reasons(self, geo: GraspGeometry, params: GraspParams) -> list[str]:
        """Reasons the jaws cannot open far enough for the object, or cannot hold one that narrow.

        Args:
            geo (GraspGeometry): Candidate.
            params (GraspParams): Parameters (max_object_width_m, min_object_width_m).

        Returns:
            list[str]: Reasons (empty when the opening fits).
        """
        reasons: list[str] = []
        if geo.gripped_m < params.min_object_width_m:
            reasons.append(
                f"the object is {geo.gripped_m * 100:.1f} cm across the jaws, below min_object_width_m "
                f"{params.min_object_width_m * 100:.1f} cm: the jaws cannot hold it"
            )
        if geo.opening_m > params.max_object_width_m:
            reasons.append(
                f"needs a {geo.opening_m * 100:.1f} cm jaw opening, more than max_object_width_m "
                f"{params.max_object_width_m * 100:.1f} cm"
            )
        elif geo.opening_m > self.jaw.gap(self.cfg.arm.gripper_open_rad):
            reasons.append(
                f"needs a {geo.opening_m * 100:.1f} cm jaw opening; the gripper opens about "
                f"{self.jaw.gap(self.cfg.arm.gripper_open_rad) * 100:.1f} cm at gripper_open_rad"
            )
        return reasons

    def solve_point(
        self,
        label: str,
        point: np.ndarray,
        pitch: float,
        seed: dict[str, float],
        extra: tuple[float, float, float] | None,
        params: GraspParams,
        budget: IkBudget | None = None,
    ) -> tuple[list[dict[str, float]], str | None]:
        """IK of one waypoint with the pose checks.

        Args:
            label (str): Waypoint label (for reasons).
            point (np.ndarray): Target (arm frame).
            pitch (float): Approach pitch (rad).
            seed (dict[str, float]): IK seed (its wrist_roll is kept).
            extra (tuple[float, float, float] | None): Centring tool offset.
            params (GraspParams): Thresholds.
            budget (IkBudget | None): Effort cap; a spent budget is a reason instead of another search.

        Returns:
            tuple[list[dict[str, float]], str | None]: [solution] and None, or ([], reason).
        """
        if budget is not None and budget.spent():
            return [], f"{label}: IK budget of {budget.limit} solves spent (planning effort cap)"
        try:
            sol = self.kin.inverse(*point, pitch, seed, extra, self.kin.last_heading_bias)
        except UnreachableError as exc:
            return [], f"{label} unreachable: {exc}"
        problem = self.pose_problem(label, sol, params)
        return ([sol], None) if problem is None else ([], problem)

    def pose_problem(self, label: str, joints: dict[str, float], params: GraspParams) -> str | None:
        """Limit-margin and shoulder-stall checks of one pose.

        Args:
            label (str): Waypoint label.
            joints (dict[str, float]): Measured joint angles.
            params (GraspParams): Thresholds.

        Returns:
            str | None: Reason, or None when the pose is fine.
        """
        if not self.kin.within_limits(self.kin.to_urdf(joints)):
            return f"{label}: joints outside the limits minus the {self.kin.margin} rad margin"
        if is_stall_pose(joints, params):
            return (
                f"{label}: stretched arm (elbow_flex {joints[ELBOW_FLEX]:.2f} rad) with shoulder_lift "
                f"{joints[SHOULDER_LIFT]:.2f} rad above {params.stall_shoulder_lift_rad} rad would stall the shoulder"
            )
        return None

    def line(
        self,
        label: str,
        start: np.ndarray,
        end: np.ndarray,
        pitch: float,
        seed: dict[str, float],
        extra: tuple[float, float, float] | None,
        params: GraspParams,
        budget: IkBudget | None = None,
    ) -> tuple[list[dict[str, float]], str | None]:
        """Straight tool-point line sampled every interpolation_step_m, each IK seeded from the previous sample.

        Args:
            label (str): Waypoint label.
            start (np.ndarray): Line start (arm frame).
            end (np.ndarray): Line end.
            pitch (float): Constant approach pitch (rad).
            seed (dict[str, float]): Pose at the line start.
            extra (tuple[float, float, float] | None): Centring tool offset.
            params (GraspParams): Step, jump and stall thresholds.
            budget (IkBudget | None): Effort cap shared with the other IK of the candidate.

        Returns:
            tuple[list[dict[str, float]], str | None]: Samples (end included) and None, or ([], reason).
        """
        n = max(1, math.ceil(float(np.linalg.norm(end - start)) / params.interpolation_step_m))
        samples: list[dict[str, float]] = []
        prev = seed
        for i in range(1, n + 1):
            point = start + (end - start) * i / n
            solved, problem = self.solve_point(label, point, pitch, prev, extra, params, budget)
            if problem is not None:
                return [], problem
            sol = solved[0]
            jump = max(abs(sol[j] - prev[j]) for j in self.kin.joint_names)
            if jump > params.max_joint_jump_rad:
                return [], (
                    f"{label}: joint jump {jump:.2f} rad between straight-line samples at "
                    f"({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}) exceeds max_joint_jump_rad {params.max_joint_jump_rad}"
                )
            samples.append(sol)
            prev = sol
        return samples, None

    def annotate(self, label: str, samples: list[dict[str, float]], surface: SurfaceModel) -> dict[str, Any]:
        """Slow-zone annotation of one plan step.

        Args:
            label (str): Waypoint label.
            samples (list[dict[str, float]]): Joint samples incl. the gripper.
            surface (SurfaceModel): Effective surface.

        Returns:
            dict[str, Any]: label, slowed_samples, samples, min_clearance_m.
        """
        report = self.guard.evaluate(samples, surface)
        finite = [c for c in report.clearances if math.isfinite(c)]
        return {
            "label": label,
            "slowed_samples": sum(1 for s in report.scales if s < 1.0),
            "samples": len(samples),
            "min_clearance_m": round(min(finite), 4) if finite else None,
        }
