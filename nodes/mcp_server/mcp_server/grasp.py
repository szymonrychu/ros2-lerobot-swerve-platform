"""Grasp planner: pure geometry on the arm URDF (ArmKinematics), no ROS and no motion.

A strategy (registry STRATEGIES, name -> callable) turns an object into grasp geometry: where the tool point (the
fixed jaw's inner face) ends up, the approach direction and pitch, the wanted jaw orientation and opening. The planner
then realizes it as ordered waypoints (pre_grasp, open, approach, grasp, close, lift, retreat) with IK solutions and
interpolated joint samples for the straight-line segments, checks feasibility and annotates the below-surface slow
zone. Infeasible plans carry human-readable reasons; the planner never raises for an infeasible object.

The SO101 is a 5-DOF arm: the approach always lies in the vertical plane of shoulder_pan, so every strategy
approaches radially from the arm base; the base has to turn for any other approach yaw.
"""

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, ValidationError

from .config import GraspSettings, LimitSettings, McpServerConfig
from .floor_guard import FloorGuard, JawModel, SurfaceModel, base_link_to_arm, rotate
from .ik import ArmKinematics, UnreachableError, grasp_offset

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
    skim: bool = False
    waypoints: list[Waypoint] = Field(default_factory=list)
    segments: dict[str, list[dict[str, float]]] = Field(default_factory=dict)
    slow_zone: list[dict[str, Any]] = Field(default_factory=list)
    surface: dict[str, Any] | None = None
    attempts: list[dict[str, Any]] = Field(default_factory=list)

    def summary(self) -> dict[str, Any]:
        """JSON-ready plan summary without the joint samples.

        Returns:
            dict[str, Any]: strategy, feasibility, reasons, key values, waypoints (no joints), slow zone, attempts.
        """
        return {
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
            "waypoints": [
                {k: (round(v, 4) if isinstance(v, float) else v) for k, v in w.model_dump(exclude={"joints"}).items()}
                for w in self.waypoints
            ],
            "slow_zone": self.slow_zone,
            "surface": self.surface,
            "attempts": self.attempts,
        }


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

    The fixed jaw top goes below_object_offset_m under the object bottom; when that would put the jaw into the
    surface the object rests on, it skims at surface + skim_clearance_m instead. Pitches from scoop_pitch_deg up to
    scoop_max_pitch_deg are offered in order (a near-horizontal gripper cannot reach low near the base).

    Args:
        req (GraspRequest): Request.

    Returns:
        list[GraspGeometry]: Candidates, flattest pitch first.
    """
    p, obj = req.params, req.obj
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

    The tool point goes to mid-height of the object, but never lower than a jaw thickness plus the skim clearance
    above the surface.

    Args:
        req (GraspRequest): Request.
        pitch (float): Approach pitch (rad).
        approach (np.ndarray): Unit approach direction.
        extent (float): Object extent along the approach before its centre (m).

    Returns:
        GraspGeometry: The grasp.
    """
    p, obj = req.params, req.obj
    z = max(obj.support_z + obj.height_m / 2.0, req.surface_z + p.skim_clearance_m + p.jaw_thickness_m)
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


STRATEGIES: dict[str, StrategyFn] = {
    "scoop": scoop_strategy,
    "angled": angled_strategy,
    "top_down": top_down_strategy,
}


def grasp_params(settings: GraspSettings, overrides: dict[str, Any] | None) -> GraspParams:
    """Grasp parameters: the configured defaults with per-call overrides, validated.

    Args:
        settings (GraspSettings): Configured defaults.
        overrides (dict[str, Any] | None): Field -> value.

    Returns:
        GraspParams: Validated parameters.

    Raises:
        ValueError: For unknown keys or invalid values.
    """
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
        surface_z = obj.support_z - surface.clearance(centre)
        req = GraspRequest(obj, params, heading, surface_z, approach_pitch_deg)
        reasons: list[str] = []
        for geo in fn(req):
            result = self.realize(strategy, obj, geo, params, surface, seed)
            if result.feasible:
                return result
            for r in result.reasons:
                reason = f"pitch {math.degrees(geo.pitch):.0f} deg: {r}"
                if reason not in reasons:
                    reasons.append(reason)
        return GraspPlan(strategy=strategy, feasible=False, reasons=reasons, object_arm=obj, surface=surface.describe())

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
    ) -> GraspPlan:
        """Turn one candidate geometry into waypoints and joint samples, checking feasibility.

        Args:
            strategy (str): Strategy name.
            obj (ObjectSpec): Object in the arm frame.
            geo (GraspGeometry): Candidate.
            params (GraspParams): Parameters.
            surface (SurfaceModel): Effective surface.
            seed (dict[str, float]): IK seed.

        Returns:
            GraspPlan: Feasible plan or one with reasons.
        """
        base = GraspPlan(
            strategy=strategy,
            feasible=False,
            object_arm=obj,
            approach_pitch_rad=geo.pitch,
            opening_m=geo.opening_m,
            center_width_m=geo.center_width,
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
        pre = approach + UP * params.pre_grasp_clearance_m
        lift = grasp + UP * params.lift_height_m
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
            ("lift", lift, hold, "keep", slow, True),
            ("retreat", retreat, hold, "keep", slow, True),
        ]
        waypoints: list[Waypoint] = []
        segments: dict[str, list[dict[str, float]]] = {}
        annotations: list[dict[str, Any]] = []
        joints = seed | {ROLL_JOINT: roll}
        prev_point: np.ndarray | None = None
        for label, point, gripper, action, speed, linear in steps:
            if linear and prev_point is not None:
                samples, problem = self.line(label, prev_point, point, geo.pitch, joints, extra, params)
            else:
                samples, problem = self.solve_point(label, point, geo.pitch, joints, extra, params)
            if problem is not None:
                return base.model_copy(update={"reasons": [problem], "wrist_roll_rad": roll})
            joints = samples[-1]
            jaw = gripper if gripper is not None else open_angle
            with_jaw = [s | {self.guard.gripper: jaw} for s in samples]
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

    def opening_reasons(self, geo: GraspGeometry, params: GraspParams) -> list[str]:
        """Reasons the jaws cannot open far enough for the object.

        Args:
            geo (GraspGeometry): Candidate.
            params (GraspParams): Parameters (max_object_width_m).

        Returns:
            list[str]: Reasons (empty when the opening fits).
        """
        reasons: list[str] = []
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
    ) -> tuple[list[dict[str, float]], str | None]:
        """IK of one waypoint with the pose checks.

        Args:
            label (str): Waypoint label (for reasons).
            point (np.ndarray): Target (arm frame).
            pitch (float): Approach pitch (rad).
            seed (dict[str, float]): IK seed (its wrist_roll is kept).
            extra (tuple[float, float, float] | None): Centring tool offset.
            params (GraspParams): Thresholds.

        Returns:
            tuple[list[dict[str, float]], str | None]: [solution] and None, or ([], reason).
        """
        try:
            sol = self.kin.inverse(*point, pitch, seed, extra)
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

        Returns:
            tuple[list[dict[str, float]], str | None]: Samples (end included) and None, or ([], reason).
        """
        n = max(1, math.ceil(float(np.linalg.norm(end - start)) / params.interpolation_step_m))
        samples: list[dict[str, float]] = []
        prev = seed
        for i in range(1, n + 1):
            point = start + (end - start) * i / n
            solved, problem = self.solve_point(label, point, pitch, prev, extra, params)
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
