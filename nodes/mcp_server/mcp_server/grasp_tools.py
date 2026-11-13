"""Grasp execution and its interfaces: GraspExecutor (runs a GraspPlan through ArmController), the MCP tools
plan_grasp / grasp_object / release_object, and GraspService, the JSON request handler behind the /grasp/command and
/grasp/result topics the web UI uses (contract in the README, "Grasp macros").

Held closes of the agent tools (grasp_object, set_gripper) are not trusted on the sensors alone: attach_jaw_crop aims the
head D435 at the FK tool point, takes a fresh head frame after the motion, crops it to the jaws and leaves the hold
'unverified' until the agent looks at it and calls confirm_grasp, which fuses its verdict with the sensor band
(grasp_check.fuse). Only the head moves for the image; the arm keeps its hold."""

import base64
import json
import logging
import math
import threading
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Annotated, Any, Literal, Protocol

import cv2
import numpy as np
from arm_collision.jaw import tool_point_in_base
from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import CallToolResult, ContentBlock, ImageContent, TextContent
from pydantic import BaseModel, ConfigDict, Field, ValidationError

from .arm import ROLL_JOINT, STALE_OFFSETS_ITEM, ArmController, ArmError, finite_effort
from .camera_point import gate_refusal
from .camera_scene import build_scene, camera_setup, head_parent_transform
from .config import HARD_MAX_IMAGE_PX, GripProfileOverride, McpServerConfig
from .floor_guard import FloorOverride, TiltOverrideDeg, arm_to_base_link
from .grasp import GraspParams, GraspPlan, GraspPlanner, ObjectSpec, Waypoint, grasp_params
from .grasp_check import Band, CropInfo, FusedOutcome, GraspCheck, GripEvidence, ImageState, ImageVerdict
from .grip import ResolvedGrip, resolve_grip_profile
from .head import CalibrationInfo, HeadError
from .head_kinematics import PAN_JOINT, TILT_JOINT, HeadAimError
from .jaw_crop import JawCropError, crop_jpeg, jaw_crop
from .jog import Jogger, JogSpec
from .lookup_tools import record_lookup
from .metrics import GRASP_ATTEMPTS, GRASP_PLANS, GRASP_RESULT_BY_OUTCOME, GRASP_VERDICTS
from .models import ArmMotionResult, CarryingState, RobotError
from .surfaces import MAX_REGIONS, SurfaceRegion
from .tool_context import ToolContext

TOOL_NAMES = ("plan_grasp", "grasp_object", "release_object", "confirm_grasp")
# IK seed when no fresh joint states are available for a dry-run plan (the folded arm).
DEFAULT_SEED = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 1.2, "wrist_flex": 0.3, "wrist_roll": 0.0}
MOTION_OK = frozenset({"converged"})
CLOSE_OK = frozenset({"grasped", "closed_no_contact", "blocked"})
# Executor outcomes of a close that holds something: the agent tools turn them into 'unverified' until confirm_grasp.
HELD_OUTCOMES = frozenset({"grasped", "unverified"})
LOGGER = logging.getLogger("mcp_server.grasp")
# Warning of a camera-sourced grasp after an arm re-zero (T-273 decision 593: warn only, the grasp is still allowed).
REZERO_WARNING = (
    "the arm was re-zeroed after the accepted arm to camera calibration was captured: "
    "re-run the arm to camera calibration"
)
# How the fuse of the sensor band and the agent's image verdict reads (shared by the tool descriptions).
FUSE_NOTE = (
    "The result's grasp_check.band is the sensors' verdict (object: contact, load and jaw gap say an object is held; "
    "empty: jaw in the empty band without load; ambiguous: anything else) and confirm_grasp fuses your answer with "
    "it: band object + yes = grasped; band empty + no = missed (nothing held: call release_object); every other "
    "combination, also unsure and every ambiguous band, stays unverified (look again from another wrist roll or "
    "get_camera_image 'head', or release_object and retry). The image alone never proves a grasp, and only fused "
    "'grasped' counts as success."
)

USE_GROUND_DESC = (
    "use the measured floor regions (steps, drop-offs) from the head D435 ground estimate; refused when no fresh "
    "estimate exists"
)
# How the agent describes stairs, tables and holes (shared by the grasp and arm motion tools).
SURFACES_DESC = (
    "Surface regions when the robot floor and the object's surface differ (stair down, table top, hole), instead of "
    "a single surface_z_m: a list of {name, height_m (relative to the robot floor, m: -0.10 stair 10 cm down, 0.15 "
    "table top), frame ('arm' default or 'base_link'), and EITHER edge {point [x, y], direction [dx, dy], side "
    "'left' (default) | 'right'}: a half-plane, the surface lies on that side of the edge line looking along "
    "direction (a stair whose step edge runs across the robot 0.28 m ahead: point [0.28, 0], direction [0, -1]) OR "
    "polygon [[x, y], ...]: a convex polygon (table top, hole)}. Outside every region the robot floor applies (or "
    "surface_z_m); a later region wins where regions overlap. The vertical step face between regions of different "
    "height counts as a surface too: plans keep the jaws, wrist and forearm clear of it (infeasible reasons name the "
    "step edge) and arm motions slow near it."
)
StrategyChoice = Literal["auto", "scoop", "angled", "top_down"]
# What an 'unverified' reason tells the agent to do on the paths that attach no image (queued grasp, web UI).
TILT_DESC = (
    "Robot tilt {roll, pitch} in deg replacing the IMU for the slow zone (roll > 0 left side up, pitch > 0 nose down)"
)
PITCH_DESC = "Approach pitch for 'angled' (deg, 0 horizontal, 90 straight down)"
LOOK_CALL = "call get_camera_image 'head' and look at it"
GRASP_CAMERA = "head"  # the head D435 colour camera, aimed at the FK tool point for the jaw crop
JAW_AT_LIMIT = "jaw outside view at the head limit"  # the clamped aim did not bring the jaw into a croppable view
GraspOutcome = Literal["planned", "infeasible", "grasped", "unverified", "missed", "aborted", "released"]


class CutoffGuard(Protocol):
    """Battery cut-off check (ros2_common BatteryGuard)."""

    def is_cutoff(self) -> bool:
        """Whether motion must be refused."""
        ...

    def rejection_message(self) -> str:
        """Why motion is refused."""
        ...


class GraspStep(BaseModel):
    """One executed step of a grasp or release."""

    label: str
    status: str
    message: str = ""
    interrupted_by: str | None = Field(
        default=None, description="Critical event type that stopped this step (ArmMotionResult.interrupted_by)"
    )
    slow_zone: dict[str, Any] | None = None
    self_collision: dict[str, Any] | None = Field(
        default=None, description="Robot body slowdown of this step (ArmMotionResult.self_collision); null when none"
    )
    residual_error: dict[str, float] = Field(
        default_factory=dict,
        description="Arm joints off their intended target of this step by more than their converge tolerance when "
        "it ended: target - measured (rad). Also joints a gripper step does not move (e.g. pushed while closing)",
    )


@dataclass
class RunLog:
    """Mutable record of one grasp run, shared by the executor phases.

    Attributes:
        steps (list[GraspStep]): Executed steps in order.
        reached (Waypoint | None): Last waypoint commanded (the arm's intended pose); None before the first.
        close (ArmMotionResult | None): The close step's result once it ran.
    """

    steps: list[GraspStep] = field(default_factory=list)
    reached: Waypoint | None = None
    close: ArmMotionResult | None = None


@dataclass(frozen=True)
class CloseVerdict:
    """What the close step found.

    Attributes:
        close (ArmMotionResult): The close result.
        unverified (str | None): Reason of a closed_no_contact close (jaw kept closed with the hold step), else None.
        missed (str | None): Why nothing is held (failed verification or a blocked close), else None.
    """

    close: ArmMotionResult
    unverified: str | None
    missed: str | None


class GraspResult(BaseModel):
    """Outcome of plan_grasp / grasp_object / release_object (and the GraspService results)."""

    outcome: GraspOutcome = Field(
        description="planned (dry run, feasible), infeasible (nothing moved), grasped (holding, lifted and retreated), "
        "unverified (holding but not confirmed: the jaw closed into the empty band, kept closed with the profile "
        "squeeze, lifted and retreated, and the object may be soft and squashed or absent; a grasp_object result also "
        "stays unverified after a verified contact until confirm_grasp: look at the attached jaw crop (otherwise "
        "call get_camera_image 'head') and call confirm_grasp(grasp_id, ...), release_object and retry when "
        "nothing is held), missed (a contact that failed verification or a "
        "blocked close: opened and retreated), aborted (stopped and held; see reasons), released"
    )
    reasons: list[str] = Field(default_factory=list, description="Why a plan is infeasible or a run ended early")
    plan: dict[str, Any] | None = Field(default=None, description="Plan summary: waypoints, pitch, roll, slow zone")
    steps: list[GraspStep] = Field(default_factory=list)
    gripper_position_rad: float | None = None
    gripper_effort: float | None = None
    grip_profile: dict[str, Any] | None = Field(
        default=None,
        description="Grip profile of the close (planned for a dry run): {name, squeeze_rad, torque_limit, "
        "close_speed_rps, target_load, contact_effort_threshold, crush_load, capped}",
    )
    holding_load: float | None = Field(
        default=None, description="Gripper load (decoded effort) a short moment after the hold; null without a grasp"
    )
    slipping: bool | None = Field(
        default=None, description="Jaw kept closing after the hold: the object slips or gives way (try firmer)"
    )
    crush_risk: bool | None = Field(
        default=None, description="Holding load above the profile's crush_load: squeezed too hard (try gentler)"
    )
    retightened: bool | None = Field(
        default=None,
        description="The hold relaxed to about 0 load with the jaw short of closed (a soft object) and was stepped "
        "once further toward closed within the squeeze cap; holding_load is measured after it",
    )
    retighten_reverted: bool | None = Field(
        default=None,
        description="The re-tightened hold showed crush_risk and went back to the squeeze before it",
    )
    residual_error: dict[str, float] = Field(
        default_factory=dict,
        description="Arm joints off their intended target (the last executed waypoint) by more than their converge "
        "tolerance at the end: target - measured (rad); empty when every joint is held on target",
    )
    warnings: list[str] = Field(
        default_factory=list,
        description="One line per step and joint that ended off its intended target by more than its tolerance",
    )
    grasp_check: GraspCheck | None = Field(
        default=None,
        description="Grasp verification of the close: {grasp_id (confirm_grasp takes it), band (sensor verdict: "
        "object / empty / ambiguous), evidence (baseline load, which contact ended the close, load and current "
        "deltas, jaw gap), image (pending: a jaw crop is attached and confirm_grasp is awaited; not_attached: the "
        "camera failed, call get_camera_image 'head'; not_requested: web UI / queue grasps, band-only; then the "
        "confirm_grasp verdict), crop {lookup_id, box_px, full_frame_reason}, fused (null until confirm_grasp)}; "
        "null when no close ran",
    )
    held_pose: dict[str, Any] | None = Field(
        default=None,
        description="Where the gripper ended, from the measured joints: {waypoint, expected_jaw_centre {x,y,z} (that "
        "waypoint's x, y, z), jaw_centre {x,y,z}, tool_point {x,y,z} (fixed jaw inner face), error_m (jaw centre to "
        "expected)}. With a centred grasp (plan grasp_shift) the tool point is shift_m beside the jaw centre by design",
    )


class GraspAbort(Exception):
    """Internal: a step failed; the run stops, holds and reports the reason."""


def floor_override(
    surface_z_m: float | None,
    tilt_override_deg: TiltOverrideDeg | None,
    surfaces: list[SurfaceRegion] | None = None,
    use_ground_estimate: bool = False,
) -> FloorOverride | None:
    """Slow-zone override of a call, or None when the call overrides nothing.

    Args:
        surface_z_m (float | None): Expected surface height relative to the robot plane (m).
        tilt_override_deg (TiltOverrideDeg | None): Tilt replacing the IMU.
        surfaces (list[SurfaceRegion] | None): Surface regions (stairs, tables, holes); empty = none.
        use_ground_estimate (bool): Add the measured ground regions of /ground/state.

    Returns:
        FloorOverride | None: The override.
    """
    if surface_z_m is None and tilt_override_deg is None and not surfaces and not use_ground_estimate:
        return None
    return FloorOverride(
        surface_z_m=surface_z_m,
        tilt_override_deg=tilt_override_deg,
        surfaces=surfaces or None,
        use_ground_estimate=use_ground_estimate,
    )


def with_object_surfaces(floor: FloorOverride | None, obj: ObjectSpec) -> FloorOverride | None:
    """The call's override with the object's own surfaces appended (they shape the plan and the whole grasp).

    Args:
        floor (FloorOverride | None): Call-level override.
        obj (ObjectSpec): Object (its surfaces optional).

    Returns:
        FloorOverride | None: The merged override (floor unchanged when the object has no surfaces).
    """
    if not obj.surfaces:
        return floor
    base = floor or FloorOverride()
    return base.model_copy(update={"surfaces": [*(base.surfaces or []), *obj.surfaces]})


def plan_result(plan: GraspPlan, notes: list[str] | None = None, grip: ResolvedGrip | None = None) -> GraspResult:
    """Dry-run result of a plan.

    Args:
        plan (GraspPlan): The plan.
        notes (list[str] | None): Extra notes (e.g. planned from a default seed).
        grip (ResolvedGrip | None): Grip profile the close would use.

    Returns:
        GraspResult: 'planned' or 'infeasible'.
    """
    return GraspResult(
        outcome="planned" if plan.feasible else "infeasible",
        reasons=[*plan.reasons, *(notes or [])],
        plan=plan.summary(),
        grip_profile=None if grip is None else grip.report(),
    )


def arm_only(samples: list[dict[str, float]], gripper: str) -> list[dict[str, float]]:
    """Drop the gripper from joint samples (the gripper keeps its commanded target during arm paths).

    Args:
        samples (list[dict[str, float]]): Samples.
        gripper (str): Gripper joint name.

    Returns:
        list[dict[str, float]]: Samples without the gripper.
    """
    return [{j: v for j, v in s.items() if j != gripper} for s in samples]


class GraspExecutor:
    """Plans with GraspPlanner and runs plans through ArmController (lease, stop, all guards and the slow zone)."""

    def __init__(
        self,
        arm: ArmController,
        config: McpServerConfig,
        calibration: Callable[[], CalibrationInfo] | None = None,
    ) -> None:
        """Bind to the arm.

        Args:
            arm (ArmController): Arm controller.
            config (McpServerConfig): Node configuration.
            calibration (Callable[[], CalibrationInfo] | None): The applied head calibration, checked against
                camera_arm.max_residual_mm for an object whose position_source is 'camera'; None refuses such an object.
        """
        self.arm = arm
        self.cfg = config
        self.calibration = calibration
        self.planner = GraspPlanner(arm.kin, config, arm.floor_guard, arm.jaw, arm.body_check, arm.head_source)

    def fraction(self, angle: float) -> float:
        """Gripper open fraction of a gripper angle.

        Args:
            angle (float): Gripper joint position (rad).

        Returns:
            float: Fraction in [0, 1] (0 closed, 1 gripper_open_rad).
        """
        closed, opened = self.cfg.arm.gripper_closed_rad, self.cfg.arm.gripper_open_rad
        return min(1.0, max(0.0, (angle - closed) / (opened - closed)))

    def resolve_grip(self, params: GraspParams) -> ResolvedGrip:
        """Resolve the grip profile of a grasp (params.grip_profile, None = the default preset).

        Args:
            params (GraspParams): Parameters.

        Returns:
            ResolvedGrip: The capped profile.

        Raises:
            ValueError: For an unknown preset or invalid overrides.
        """
        return resolve_grip_profile(self.cfg.grip_profiles, self.cfg.limits, params.grip_profile)

    def plan(
        self,
        obj: ObjectSpec,
        strategy: str,
        params: GraspParams,
        approach_pitch_deg: float | None,
        floor: FloorOverride | None,
    ) -> tuple[GraspPlan, list[str]]:
        """Plan from the current arm pose (or a default seed without fresh joint states).

        Args:
            obj (ObjectSpec): Object.
            strategy (str): Strategy or 'auto'.
            params (GraspParams): Parameters.
            approach_pitch_deg (float | None): Pitch for 'angled'.
            floor (FloorOverride | None): Slow-zone overrides (also the surface the scoop skims).

        Returns:
            tuple[GraspPlan, list[str]]: The plan and notes about how it was planned.
        """
        floor = with_object_surfaces(floor, obj)
        sample = self.arm.fresh_sample()
        notes: list[str] = []
        if sample is None:
            seed = dict(DEFAULT_SEED)
            notes.append("no fresh joint states: planned from a default folded seed pose")
        else:
            seed = self.arm.command_base(sample)
        surface = self.arm.floor_surface(floor)
        return self.planner.plan(obj, strategy, params, surface, seed, approach_pitch_deg), notes

    def correction_path(
        self, landed: GraspPlan, new: GraspPlan, params: GraspParams, floor: FloorOverride | None
    ) -> tuple[list[dict[str, float]], str | None]:
        """Straight, jaws-open tool line from the landed grasp point to a nearby new one (planned from the arm now).

        Args:
            landed (GraspPlan): Plan the jaws landed with.
            new (GraspPlan): Feasible plan of the corrected object position.
            params (GraspParams): Step, jump and surface clearance thresholds.
            floor (FloorOverride | None): Slow-zone overrides (the surface the line is checked against).

        Returns:
            tuple[list[dict[str, float]], str | None]: Joint samples and None, or ([], reason) (also when there are
                no fresh joint states).
        """
        try:
            seed = self.arm.command_base(self.arm.require_sample())
        except ArmError as exc:
            return [], str(exc)
        return self.planner.correction_path(landed, new, seed, params, self.arm.floor_surface(floor))

    def camera_refusal(self, obj: ObjectSpec) -> str | None:
        """Why a camera-derived position may not be grasped (the one gate every execute path goes through).

        Args:
            obj (ObjectSpec): Object.

        Returns:
            str | None: Refusal text for a 'camera' position whose applied calibration residual is unknown or above
                camera_arm.max_residual_mm (or when no calibration source is bound); None otherwise.
        """
        if obj.position_source != "camera":
            return None
        if self.calibration is None:
            return "no head calibration is available to check against the camera-to-arm residual gate"
        return gate_refusal(self.calibration(), self.cfg.camera_arm.max_residual_mm)

    def rezero_warning(self, obj: ObjectSpec) -> str | None:
        """Warning for a camera-sourced position while the arm re-zero marker lists joint_offsets (never a refusal).

        Args:
            obj (ObjectSpec): Object.

        Returns:
            str | None: REZERO_WARNING when the object's position_source is 'camera' and the stale marker lists
                joint_offsets (an unreadable marker counts as stale, as in ArmController.home); None otherwise.
        """
        if obj.position_source != "camera":
            return None
        marker = self.arm.stale_marker()
        return REZERO_WARNING if marker is not None and STALE_OFFSETS_ITEM in marker.stale else None

    def prepare(
        self,
        obj: ObjectSpec,
        strategy: str,
        params: GraspParams,
        approach_pitch_deg: float | None,
        floor: FloorOverride | None,
    ) -> tuple[GraspPlan, list[str], ResolvedGrip, FloorOverride | None]:
        """Everything before the first motion: resolve the grip, gate a camera position, merge surfaces, plan.

        Args:
            obj (ObjectSpec): Object.
            strategy (str): Strategy or 'auto'.
            params (GraspParams): Parameters.
            approach_pitch_deg (float | None): Pitch for 'angled'.
            floor (FloorOverride | None): Slow-zone overrides.

        Returns:
            tuple[GraspPlan, list[str], ResolvedGrip, FloorOverride | None]: The plan, notes about how it was
                planned, the resolved grip and the floor overrides merged with the object's surfaces.

        Raises:
            ValueError: For an invalid grip profile, or a camera-derived position the calibration residual gate
                refuses (nothing moved).
        """
        grip = self.resolve_grip(params)  # an invalid profile refuses before anything moves
        refusal = self.camera_refusal(obj)
        if refusal is not None:
            raise ValueError(f"grasp refused (nothing moved): the object position comes from the camera and {refusal}")
        floor = with_object_surfaces(floor, obj)
        plan, notes = self.plan(obj, strategy, params, approach_pitch_deg, floor)
        warning = self.rezero_warning(obj)
        if warning is not None:
            notes.append(warning)
        return plan, notes, grip, floor

    def grasp(
        self,
        obj: ObjectSpec,
        strategy: str,
        params: GraspParams,
        approach_pitch_deg: float | None,
        floor: FloorOverride | None,
        stop_requested: Callable[[], bool],
    ) -> GraspResult:
        """Plan and execute a grasp.

        Args:
            obj (ObjectSpec): Object.
            strategy (str): Strategy or 'auto'.
            params (GraspParams): Parameters.
            approach_pitch_deg (float | None): Pitch for 'angled'.
            floor (FloorOverride | None): Slow-zone overrides.
            stop_requested (Callable[[], bool]): True once a stop was requested (checked between steps).

        Returns:
            GraspResult: infeasible / grasped / unverified / missed / aborted.

        Raises:
            ValueError: For an invalid grip profile, or a camera-derived position the calibration residual gate
                refuses (nothing moved).
        """
        plan, notes, grip, floor = self.prepare(obj, strategy, params, approach_pitch_deg, floor)
        rezero = [n for n in notes if n == REZERO_WARNING]
        notes = [n for n in notes if n != REZERO_WARNING]
        result = (
            plan_result(plan, notes, grip) if not plan.feasible else self.execute(plan, params, floor, stop_requested)
        )
        result.warnings = [*rezero, *result.warnings]
        GRASP_ATTEMPTS.labels(strategy, GRASP_RESULT_BY_OUTCOME[result.outcome]).inc()
        return result

    def step(
        self,
        label: str,
        steps: list[GraspStep],
        stop_requested: Callable[[], bool],
        motion: Callable[[], ArmMotionResult],
        ok: frozenset[str] = MOTION_OK,
        intent: dict[str, float] | None = None,
    ) -> ArmMotionResult:
        """Run one motion step, recording it; abort the run on a stop, a refusal or an unexpected status.

        Args:
            label (str): Step label.
            steps (list[GraspStep]): Step log (appended).
            stop_requested (Callable[[], bool]): Stop check.
            motion (Callable[[], ArmMotionResult]): The arm call.
            ok (frozenset[str]): Statuses that continue the run.
            intent (dict[str, float] | None): Intended arm pose at the end of the step (every arm joint, also for a
                gripper step); joints off it beyond their tolerance are recorded as the step's residual_error.

        Returns:
            ArmMotionResult: The motion result.

        Raises:
            GraspAbort: When the run must end.
        """
        if stop_requested():
            raise GraspAbort(f"{label}: stop requested")
        try:
            result = motion()
        except ArmError as exc:
            steps.append(GraspStep(label=label, status="refused", message=str(exc)))
            raise GraspAbort(f"{label}: {exc}") from exc
        residual = {j: e for j, e in result.residual_error.items() if j != self.arm.gripper}
        if intent is not None:
            residual |= self.intent_errors(intent)
        steps.append(
            GraspStep(
                label=label,
                status=result.status,
                message=result.message,
                interrupted_by=result.interrupted_by,
                slow_zone=result.slow_zone,
                self_collision=result.self_collision,
                residual_error=residual,
            )
        )
        if result.status not in ok:
            raise GraspAbort(f"{label}: {result.status}: {result.message}")
        return result

    def intent_errors(self, intent: dict[str, float]) -> dict[str, float]:
        """Arm joints of the current fresh sample off their intended target by more than their converge tolerance.

        Args:
            intent (dict[str, float]): Intended arm joint positions (measured space).

        Returns:
            dict[str, float]: target - measured (rad, rounded to 1e-4); empty without fresh joint states.
        """
        sample = self.arm.fresh_sample()
        if sample is None:
            return {}
        limits = self.cfg.limits
        return {
            j: round(intent[j] - sample.positions[j], 4)
            for j in self.arm.joint_names
            if j in intent
            and j in sample.positions
            and abs(intent[j] - sample.positions[j]) > limits.converge_tolerance_for(j)
        }

    def step_warnings(self, steps: list[GraspStep]) -> list[str]:
        """One warning per step and joint that ended off its intended target.

        Args:
            steps (list[GraspStep]): Step log.

        Returns:
            list[str]: The warnings.
        """
        limits = self.cfg.limits
        return [
            f"{s.label}: {j} {e:+.3f} rad off its target (tolerance {limits.converge_tolerance_for(j):g} rad)"
            for s in steps
            for j, e in s.residual_error.items()
        ]

    def held_pose(self, plan: GraspPlan, waypoint: Waypoint) -> dict[str, Any] | None:
        """Where the jaw centre and the tool point are now (measured joints) against a waypoint's target.

        Args:
            plan (GraspPlan): Executed plan (its centring decides the jaw centre).
            waypoint (Waypoint): Last executed waypoint.

        Returns:
            dict[str, Any] | None: {waypoint, expected_jaw_centre, jaw_centre, tool_point, error_m}; None without
                fresh joint states.
        """
        sample = self.arm.fresh_sample()
        if sample is None:
            return None
        joints = self.arm.measured(sample)
        tool = self.arm.kin.forward(joints)
        centre = self.arm.kin.forward(joints, self.planner.center_offset(plan))
        expected = {"x": round(waypoint.x, 4), "y": round(waypoint.y, 4), "z": round(waypoint.z, 4)}
        error = math.dist((centre.x, centre.y, centre.z), (waypoint.x, waypoint.y, waypoint.z))
        return {
            "waypoint": waypoint.label,
            "expected_jaw_centre": expected,
            "jaw_centre": {"x": round(centre.x, 4), "y": round(centre.y, 4), "z": round(centre.z, 4)},
            "tool_point": {"x": round(tool.x, 4), "y": round(tool.y, 4), "z": round(tool.z, 4)},
            "error_m": round(error, 4),
        }

    def finish(
        self,
        outcome: GraspOutcome,
        reasons: list[str],
        plan: GraspPlan | None,
        steps: list[GraspStep],
        close: ArmMotionResult | None = None,
        reached: Waypoint | None = None,
    ) -> GraspResult:
        """Result with the final gripper state, the close's grip report and the hold check against the intent.

        Args:
            outcome (GraspOutcome): Outcome.
            reasons (list[str]): Reasons.
            plan (GraspPlan | None): Executed plan.
            steps (list[GraspStep]): Step log.
            close (ArmMotionResult | None): Close result (grip_profile, holding_load, slipping, crush_risk,
                retightened).
            reached (Waypoint | None): Last waypoint the run commanded (the intended hold); None for a release.

        Returns:
            GraspResult: The result.
        """
        sample = self.arm.fresh_sample()
        gripper = self.arm.gripper
        residual = {} if reached is None else self.intent_errors(reached.joints)
        warnings = self.step_warnings(steps)
        if residual and (not steps or steps[-1].residual_error != residual):
            warnings.append(f"hold: {', '.join(f'{j} {e:+.3f} rad' for j, e in residual.items())} off the target")
        return GraspResult(
            outcome=outcome,
            reasons=reasons,
            plan=None if plan is None else plan.summary(),
            steps=steps,
            gripper_position_rad=None if sample is None else sample.positions.get(gripper),
            gripper_effort=None if sample is None else finite_effort(sample.efforts, gripper),
            grip_profile=None if close is None else close.grip_profile,
            holding_load=None if close is None else close.holding_load,
            slipping=None if close is None else close.slipping,
            crush_risk=None if close is None else close.crush_risk,
            retightened=None if close is None else close.retightened,
            retighten_reverted=None if close is None else close.retighten_reverted,
            residual_error=residual,
            warnings=warnings,
            held_pose=None if plan is None or reached is None else self.held_pose(plan, reached),
            grasp_check=None if close is None else close.grasp_check,
        )

    def unverified_reason(self, close: ArmMotionResult) -> str:
        """Why a closed_no_contact close is 'unverified' rather than a miss (the jaw keeps its hold).

        Args:
            close (ArmMotionResult): Close result with status closed_no_contact.

        Returns:
            str: The reason, with what the agent should do next.
        """
        sample = self.arm.fresh_sample()
        jaw = "" if sample is None else f" at {sample.positions[self.arm.gripper]:.3f} rad"
        limit = self.cfg.limits.gripper_empty_stall_rad
        return (
            f"jaw closed{jaw} into the empty band (within gripper_empty_stall_rad {limit:g} rad of closed; close: "
            f"{close.message}): the object may be soft and squashed between the jaws (plush) or absent, the load "
            f"cannot tell. Kept closed and lifted: {LOOK_CALL}; if the object is in the jaws treat it as held, "
            "otherwise call release_object (opens and lifts) and retry"
        )

    def verify(self, close: ArmMotionResult, params: GraspParams) -> str | None:
        """Grasp check after the close: the jaw stopped short of closed and the gripper felt the object.

        Args:
            close (ArmMotionResult): Close result.
            params (GraspParams): min_hold_gap_rad, hold_effort_min.

        Returns:
            str | None: Why nothing is held, or None for a verified grasp.
        """
        if close.status != "grasped":
            return f"close ended '{close.status}': {close.message}"
        return self.hold_problem(params, close)

    def hold_problem(
        self, params: GraspParams, close: ArmMotionResult | None = None, check_gap: bool = True
    ) -> str | None:
        """Sample-based hold check: a fresh sample with the jaw stopped short of closed under load.

        Args:
            params (GraspParams): min_hold_gap_rad, hold_effort_min.
            close (ArmMotionResult | None): Close result whose gripper effort also counts as a load reading.
            check_gap (bool): False for soft objects (ambiguous close band): a squashed plush legitimately leaves
                the jaw within min_hold_gap_rad of closed, so only the load counts.

        Returns:
            str | None: Why nothing is held, or None while the jaws hold something.
        """
        sample = self.arm.fresh_sample()
        if sample is None:
            return "no fresh joint states to verify the grasp"
        jaw = sample.positions[self.arm.gripper]
        gap = jaw - self.cfg.arm.gripper_closed_rad
        measured = [
            abs(e)
            for e in (None if close is None else close.gripper_effort, finite_effort(sample.efforts, self.arm.gripper))
            if e is not None and math.isfinite(e)
        ]
        if check_gap and gap < params.min_hold_gap_rad:
            return (
                f"jaw reached {jaw:.3f} rad, within {params.min_hold_gap_rad} rad of closed: nothing between the jaws"
            )
        if not measured:
            return "gripper load unmeasured (no finite effort on the close or now): the hold cannot be verified"
        load = max(measured)
        if load < params.hold_effort_min:
            return f"gripper load {load:.0f} below hold_effort_min {params.hold_effort_min:.0f}: nothing held"
        return None

    def execute(
        self,
        plan: GraspPlan,
        params: GraspParams,
        floor: FloorOverride | None,
        stop_requested: Callable[[], bool],
    ) -> GraspResult:
        """Run a feasible plan: (half-open,) pre-grasp, roll, open, approach, slide, close, verify, lift, retreat.

        A close into the empty band (closed_no_contact: a squashed soft object or nothing) keeps the jaw closed with
        the profile's squeeze and torque limit (step 'hold'), lifts and retreats and is 'unverified'. A miss (a
        contact that fails verification, or a blocked close) opens the gripper, lifts and retreats; any abort stops
        and holds the arm.

        Args:
            plan (GraspPlan): Feasible plan.
            params (GraspParams): Parameters.
            floor (FloorOverride | None): Slow-zone overrides.
            stop_requested (Callable[[], bool]): Stop check between steps.

        Returns:
            GraspResult: grasped / unverified / missed / aborted.
        """
        log = RunLog(steps=[])
        unverified: str | None = None
        missed: str | None = None
        try:
            self.land(plan, params, floor, stop_requested, log)
            verdict = self.close_and_verify(plan, params, floor, stop_requested, log)
            unverified, missed = verdict.unverified, verdict.missed
            if missed is not None:
                self.open_jaws(plan, floor, stop_requested, log)
            self.lift_and_retreat(plan, floor, stop_requested, log)
        except GraspAbort as exc:
            self.halt()
            return self.finish("aborted", [str(exc)], plan, log.steps, log.close, log.reached)
        if unverified is not None:
            return self.finish("unverified", [unverified], plan, log.steps, log.close, log.reached)
        if missed is not None:
            return self.finish("missed", [missed], plan, log.steps, log.close, log.reached)
        return self.finish("grasped", [], plan, log.steps, log.close, log.reached)

    def land(
        self,
        plan: GraspPlan,
        params: GraspParams,
        floor: FloorOverride | None,
        stop_requested: Callable[[], bool],
        log: RunLog,
    ) -> None:
        """Steps (half_open,) pre_grasp, (roll,) open, approach, grasp: the jaws open at the grasp pose.

        Args:
            plan (GraspPlan): Feasible plan.
            params (GraspParams): Parameters (slide_speed_scale).
            floor (FloorOverride | None): Slow-zone overrides.
            stop_requested (Callable[[], bool]): Stop check between steps.
            log (RunLog): Run log (steps appended, reached updated; reached ends as the grasp waypoint).

        Raises:
            GraspAbort: When the run must end.
        """
        arm, gripper, limits = self.arm, self.arm.gripper, self.cfg.limits
        steps = log.steps
        wp: dict[str, Waypoint] = {w.label: w for w in plan.waypoints}
        slide = params.slide_speed_scale
        try:
            sample = arm.require_sample()
        except ArmError as exc:
            raise GraspAbort(str(exc)) from exc
        pre = wp["pre_grasp"]
        current = arm.command_base(sample)
        roll_change = abs(pre.roll - current[ROLL_JOINT]) > limits.roll_guard_min_change_rad
        opening = max(sample.positions[gripper], current.get(gripper, -math.inf))
        if roll_change and opening > limits.roll_max_gripper_open_rad and pre.gripper is not None:
            frac = self.fraction(pre.gripper)
            self.step("half_open", steps, stop_requested, lambda: arm.set_gripper(open_fraction=frac, floor=floor))
        targets = dict(pre.joints)
        if roll_change:
            targets[ROLL_JOINT] = current[ROLL_JOINT]  # roll only once lifted at the pre-grasp
        log.reached = pre
        free = min(pre.speed_scale, limits.arm_max_speed_scale)  # approach_speed_scale (planned)
        self.step("pre_grasp", steps, stop_requested, lambda: arm.move_joints(targets, free, floor), intent=targets)
        if roll_change:
            self.step(
                "roll",
                steps,
                stop_requested,
                lambda: arm.move_joints({ROLL_JOINT: pre.roll}, free, floor),
                intent=pre.joints,
            )
        open_frac = self.fraction(wp["open"].gripper or self.cfg.arm.gripper_open_rad)
        self.step(
            "open",
            steps,
            stop_requested,
            lambda: arm.set_gripper(open_fraction=open_frac, floor=floor),
            intent=pre.joints,
        )
        for label in ("approach", "grasp"):
            path = arm_only(plan.segments[label], gripper)
            log.reached = wp[label]
            self.step(label, steps, stop_requested, lambda p=path: arm.move_path(p, slide, floor), intent=path[-1])

    def close_and_verify(
        self,
        plan: GraspPlan,
        params: GraspParams,
        floor: FloorOverride | None,
        stop_requested: Callable[[], bool],
        log: RunLog,
    ) -> CloseVerdict:
        """Step close, then hold (closed_no_contact: unverified) or the grasp check (a failure is a miss). Never opens.

        Args:
            plan (GraspPlan): Feasible plan, landed at its grasp waypoint.
            params (GraspParams): close_effort_threshold, grip_profile, hold thresholds.
            floor (FloorOverride | None): Slow-zone overrides.
            stop_requested (Callable[[], bool]): Stop check between steps.
            log (RunLog): Run log (steps appended, close set).

        Returns:
            CloseVerdict: The close result with the unverified or missed reason (or neither: a verified grasp).

        Raises:
            GraspAbort: When the run must end.
        """
        held = next(w for w in plan.waypoints if w.label == "grasp").joints
        close = self.step(
            "close",
            log.steps,
            stop_requested,
            lambda: self.arm.set_gripper(
                close_until_effort=True,
                effort_threshold=params.close_effort_threshold,
                floor=floor,
                grip_profile=params.grip_profile,
            ),
            CLOSE_OK,
            intent=held,
        )
        log.close = close
        if close.status == "closed_no_contact":
            unverified = self.unverified_reason(close)
            self.hold_step(log.steps, stop_requested, params)
            return CloseVerdict(close, unverified, None)
        return CloseVerdict(close, None, self.verify(close, params))

    def open_jaws(
        self, plan: GraspPlan, floor: FloorOverride | None, stop_requested: Callable[[], bool], log: RunLog
    ) -> None:
        """Step open: the jaws to the plan's open gripper angle (after a miss).

        Args:
            plan (GraspPlan): Plan whose open waypoint gives the angle.
            floor (FloorOverride | None): Slow-zone overrides.
            stop_requested (Callable[[], bool]): Stop check.
            log (RunLog): Run log (steps appended); log.reached.joints is the intended pose.

        Raises:
            GraspAbort: When the run must end.
        """
        open_wp = next(w for w in plan.waypoints if w.label == "open")
        open_frac = self.fraction(open_wp.gripper or self.cfg.arm.gripper_open_rad)
        self.step(
            "open",
            log.steps,
            stop_requested,
            lambda: self.arm.set_gripper(open_fraction=open_frac, floor=floor),
            intent=None if log.reached is None else log.reached.joints,
        )

    def lift_and_retreat(
        self, plan: GraspPlan, floor: FloorOverride | None, stop_requested: Callable[[], bool], log: RunLog
    ) -> None:
        """Steps lift and retreat along the planned segments.

        Args:
            plan (GraspPlan): Feasible plan.
            floor (FloorOverride | None): Slow-zone overrides.
            stop_requested (Callable[[], bool]): Stop check between steps.
            log (RunLog): Run log (steps appended, reached updated).

        Raises:
            GraspAbort: When the run must end.
        """
        wp: dict[str, Waypoint] = {w.label: w for w in plan.waypoints}
        for label in ("lift", "retreat"):
            path = arm_only(plan.segments[label], self.arm.gripper)
            speed = wp[label].speed_scale  # a tall narrow object lifts at lift_speed_scale
            log.reached = wp[label]
            self.step(
                label,
                log.steps,
                stop_requested,
                lambda p=path, v=speed: self.arm.move_path(p, v, floor),
                intent=path[-1],
            )

    def back_out(
        self,
        plan: GraspPlan,
        params: GraspParams,
        floor: FloorOverride | None,
        stop_requested: Callable[[], bool],
        log: RunLog,
    ) -> None:
        """Steps back_out_grasp and back_out_approach: slide back along the reversed grasp, then approach segments.

        Args:
            plan (GraspPlan): Plan the jaws landed with.
            params (GraspParams): slide_speed_scale.
            floor (FloorOverride | None): Slow-zone overrides.
            stop_requested (Callable[[], bool]): Stop check between steps.
            log (RunLog): Run log (steps appended); reached ends as the open waypoint (the pre-grasp pose).

        Raises:
            GraspAbort: When the run must end.
        """
        slide = params.slide_speed_scale
        for label, segment in (("back_out_grasp", "grasp"), ("back_out_approach", "approach")):
            path = arm_only(plan.segments[segment], self.arm.gripper)[::-1]
            self.step(
                label,
                log.steps,
                stop_requested,
                lambda p=path: self.arm.move_path(p, slide, floor),
                intent=path[-1],
            )
        log.reached = next(w for w in plan.waypoints if w.label == "open")

    def open_and_rise(
        self,
        open_fraction: float,
        rise_m: float,
        speed_scale: float,
        floor: FloorOverride | None,
        stop_requested: Callable[[], bool],
        log: RunLog,
        rise_label: str = "rise",
    ) -> ArmMotionResult:
        """Step open, then the tool point rise_m straight up (same pitch).

        Args:
            open_fraction (float): Gripper open fraction.
            rise_m (float): Rise (m).
            speed_scale (float): Speed scale of the rise.
            floor (FloorOverride | None): Slow-zone overrides.
            stop_requested (Callable[[], bool]): Stop check between steps.
            log (RunLog): Run log (steps appended).
            rise_label (str): Label of the rise step.

        Returns:
            ArmMotionResult: The rise result (status converged, or unreachable when the rise was skipped).

        Raises:
            GraspAbort: When the run must end.
        """
        arm = self.arm
        self.step("open", log.steps, stop_requested, lambda: arm.set_gripper(open_fraction=open_fraction, floor=floor))
        try:
            pose = arm.kin.forward(arm.command_base(arm.require_sample()))
        except ArmError as exc:
            raise GraspAbort(str(exc)) from exc
        return self.step(
            rise_label,
            log.steps,
            stop_requested,
            lambda: arm.move_cartesian(pose.x, pose.y, pose.z + rise_m, pose.pitch, speed_scale, floor=floor),
            MOTION_OK | {"unreachable"},
        )

    def halt(self) -> None:
        """Stop and hold the arm and never leave the gripper weakened (or firm) after an abort."""
        self.arm.stop_hold()
        self.arm.restore_default_torque_limit()

    def landing_error(self, plan: GraspPlan) -> dict[str, Any] | None:
        """Where the jaws landed against the plan's grasp waypoint: held_pose plus the tool point error.

        Args:
            plan (GraspPlan): Executed plan.

        Returns:
            dict[str, Any] | None: held_pose of the grasp waypoint plus planned_tool_point (the waypoint's tool
                point) and tool_point_error_m (rounded to 1e-4; both absent when the waypoint has no tool point);
                None without fresh joint states.
        """
        grasp = next(w for w in plan.waypoints if w.label == "grasp")
        held = self.held_pose(plan, grasp)
        if held is None:
            return None
        planned = grasp.tool_point
        if planned is None:
            return held
        tool = held["tool_point"]
        held["planned_tool_point"] = {axis: round(planned[axis], 4) for axis in ("x", "y", "z")}
        held["tool_point_error_m"] = round(
            math.dist((tool["x"], tool["y"], tool["z"]), tuple(planned[a] for a in "xyz")), 4
        )
        return held

    def hold_step(self, steps: list[GraspStep], stop_requested: Callable[[], bool], params: GraspParams) -> None:
        """Step 'hold': keep an empty-band close gripping with the profile's squeeze and torque limit.

        Args:
            steps (list[GraspStep]): Step log (appended).
            stop_requested (Callable[[], bool]): Stop check.
            params (GraspParams): grip_profile of the close.

        Raises:
            GraspAbort: On a stop or when the hold cannot be commanded.
        """
        if stop_requested():
            raise GraspAbort("hold: stop requested")
        try:
            message = self.arm.hold_empty_close(params.grip_profile)
        except ArmError as exc:
            steps.append(GraspStep(label="hold", status="refused", message=str(exc)))
            raise GraspAbort(f"hold: {exc}") from exc
        steps.append(GraspStep(label="hold", status="holding", message=message))

    def release(
        self, params: GraspParams, floor: FloorOverride | None, stop_requested: Callable[[], bool]
    ) -> GraspResult:
        """Open the gripper to release_open_fraction, then lift the tool point release_lift_m straight up.

        Args:
            params (GraspParams): release_open_fraction, release_lift_m, slide_speed_scale.
            floor (FloorOverride | None): Slow-zone overrides.
            stop_requested (Callable[[], bool]): Stop check between steps.

        Returns:
            GraspResult: released (reasons note a skipped, unreachable lift) or aborted.
        """
        log = RunLog(steps=[])
        try:
            lift = self.open_and_rise(
                params.release_open_fraction,
                params.release_lift_m,
                params.slide_speed_scale,
                floor,
                stop_requested,
                log,
                rise_label="lift",
            )
        except GraspAbort as exc:
            self.arm.stop_hold()
            return self.finish("aborted", [str(exc)], None, log.steps)
        reasons = [] if lift.status == "converged" else [f"released; lift skipped: {lift.message}"]
        return self.finish("released", reasons, None, log.steps)


class GraspServiceRequest(BaseModel):
    """JSON request on /grasp/command (see the README for the full contract)."""

    model_config = ConfigDict(extra="forbid")

    action: Literal["plan", "execute", "release", "stop", "jog"]
    request_id: str | None = None
    jog: JogSpec | None = None  # required for action 'jog' (jog.JogSpec)
    object: ObjectSpec | None = None
    strategy: StrategyChoice = "auto"
    params: dict[str, Any] | None = None
    approach_pitch_deg: float | None = Field(default=None, ge=0.0, le=90.0)
    surface_z_m: float | None = Field(default=None, ge=-1.0, le=1.0)
    tilt_override_deg: TiltOverrideDeg | None = None
    grip_profile: str | GripProfileOverride | None = None
    surfaces: list[SurfaceRegion] | None = Field(default=None, max_length=MAX_REGIONS)
    use_ground_estimate: bool = False


class GraspService:
    """Web-UI grasp actions as JSON in, JSON out; one execute/release at a time, 'stop' always answers at once."""

    def __init__(
        self,
        arm: ArmController,
        config: McpServerConfig,
        guard: CutoffGuard | None = None,
        stop_count: Callable[[], int] | None = None,
        publish: Callable[[dict[str, Any]], None] | None = None,
        calibration: Callable[[], CalibrationInfo] | None = None,
    ) -> None:
        """Create the service.

        Args:
            arm (ArmController): Arm controller.
            config (McpServerConfig): Node configuration.
            guard (CutoffGuard | None): Battery cut-off guard; execute/release/jog are refused in cut-off.
            stop_count (Callable[[], int] | None): Robot stop counter (a stop tool call aborts a running grasp).
            publish (Callable[[dict[str, Any]], None] | None): Sends an answer on /grasp/result outside handle's
                return (coalesced jogs); None logs and drops such answers.
            calibration (Callable[[], CalibrationInfo] | None): The applied head calibration for the camera-position
                gate (GraspExecutor); None refuses camera positions.
        """
        self.arm = arm
        self.cfg = config
        self.guard = guard
        self.stop_count = stop_count or (lambda: 0)
        self.executor = GraspExecutor(arm, config, calibration)
        self.cancel = threading.Event()
        self.busy = threading.Lock()
        self.jogger = Jogger(arm, config.jog, guard, publish or unsent_answer)

    def handle_json(self, text: str) -> str | None:
        """handle() for a JSON string.

        Args:
            text (str): Request JSON.

        Returns:
            str | None: Response JSON, or None when the answer goes out later through publish (a coalesced jog).
        """
        try:
            request = json.loads(text)
        except json.JSONDecodeError as exc:
            return json.dumps({"ok": False, "request_id": None, "action": None, "error": f"invalid JSON: {exc}"})
        if not isinstance(request, dict):
            return json.dumps({"ok": False, "request_id": None, "action": None, "error": "request must be an object"})
        answer = self.handle(request)
        return None if answer is None else json.dumps(answer)

    def handle(self, request: dict[str, Any]) -> dict[str, Any] | None:
        """Run one request.

        Args:
            request (dict[str, Any]): {action, request_id?, object?, strategy?, params?, approach_pitch_deg?,
                surface_z_m?, tilt_override_deg?, grip_profile?, surfaces?, use_ground_estimate?, jog?}.

        Returns:
            dict[str, Any] | None: {ok, request_id, action, result} or {ok: false, request_id, action, error}; None
                for a jog whose answer goes out through publish (it was coalesced, see jog.Jogger.submit).
        """
        head = {"request_id": request.get("request_id"), "action": request.get("action")}
        try:
            req = GraspServiceRequest.model_validate(request)
        except ValidationError as exc:
            return head | {"ok": False, "error": f"invalid request: {exc}"}
        if req.action == "stop":
            self.cancel.set()
            self.jogger.drop_pending()
            held, note = self.arm.stop_hold()
            return head | {"ok": True, "result": {"arm_held": held, "message": note}}
        if req.action != "plan" and self.guard is not None and self.guard.is_cutoff():
            return head | {"ok": False, "error": self.guard.rejection_message()}
        if req.action == "jog":
            return self.jog(req, head)
        try:
            params = grasp_params(self.cfg.grasp, req.params, req.grip_profile)
            grip = self.executor.resolve_grip(params)
        except ValueError as exc:
            return head | {"ok": False, "error": str(exc)}
        floor = floor_override(req.surface_z_m, req.tilt_override_deg, req.surfaces, req.use_ground_estimate)
        if req.action in ("plan", "execute") and req.object is None:
            return head | {"ok": False, "error": f"'object' is required for {req.action}"}
        if req.action == "plan":
            assert req.object is not None
            plan, notes = self.executor.plan(req.object, req.strategy, params, req.approach_pitch_deg, floor)
            return head | {"ok": True, "result": plan_result(plan, notes, grip).model_dump(mode="json")}
        if not self.busy.acquire(blocking=False):
            return head | {"ok": False, "error": "another grasp action is running; send action 'stop' first"}
        try:
            self.cancel.clear()
            baseline = self.stop_count()

            def stop_requested() -> bool:
                return self.cancel.is_set() or self.stop_count() != baseline

            if req.action == "execute":
                assert req.object is not None
                try:
                    result = self.executor.grasp(
                        req.object, req.strategy, params, req.approach_pitch_deg, floor, stop_requested
                    )
                except ValueError as exc:
                    return head | {"ok": False, "error": str(exc)}
            else:
                result = self.executor.release(params, floor, stop_requested)
        finally:
            self.busy.release()
        return head | {"ok": True, "result": result.model_dump(mode="json")}

    def jog(self, req: GraspServiceRequest, head: dict[str, Any]) -> dict[str, Any] | None:
        """Hand a jog to the coalescing jogger, unless the request has no spec or a grasp action runs.

        Args:
            req (GraspServiceRequest): Validated request with action 'jog'.
            head (dict[str, Any]): {request_id, action}.

        Returns:
            dict[str, Any] | None: The answer, or None when it goes out through publish.
        """
        if req.jog is None:
            return head | {"ok": False, "error": "'jog' is required for jog"}
        if self.busy.locked():
            return head | {"ok": False, "error": "jog refused: a grasp action is running; send action 'stop' first"}
        return self.jogger.submit(req.jog, head)


def unsent_answer(answer: dict[str, Any]) -> None:
    """Default publish of GraspService without a /grasp/result publisher: log the answer.

    Args:
        answer (dict[str, Any]): The answer that had no publisher.
    """
    LOGGER.warning("no /grasp/result publisher for answer %s", json.dumps(answer))


def grasp_tool_result(result: BaseModel, images: list[ImageContent]) -> CallToolResult:
    """MCP result of a grasp tool: the result model as structured content and JSON text, plus images.

    Args:
        result (BaseModel): The GraspResult, GraspFlowResult or ArmMotionResult.
        images (list[ImageContent]): Attached images in order (a jaw crop, a marked frame); empty when none.

    Returns:
        CallToolResult: The tool result.
    """
    data = result.model_dump(mode="json")
    content: list[ContentBlock] = [TextContent(type="text", text=json.dumps(data))]
    content.extend(images)
    return CallToolResult(content=content, structured_content=data)


def awaiting_confirm(check: GraspCheck) -> str:
    """What the agent must do with a held close whose verdict is still open.

    Args:
        check (GraspCheck): Grasp check of the close (grasp_id, band, image).

    Returns:
        str: The instruction, naming confirm_grasp and the grasp_id.
    """
    where = (
        "look at the attached jaw crop"
        if check.image == "pending"
        else "no image could be attached: call get_camera_image 'head' and look at it"
    )
    return (
        f"awaiting confirm_grasp(grasp_id={check.grasp_id!r}): is the object between the fingers? {where}, then "
        f"call confirm_grasp (sensor band {check.band}; only fused 'grasped' counts as held)"
    )


def jpeg_size(jpeg: bytes) -> tuple[int, int]:
    """Pixel size of a JPEG.

    Args:
        jpeg (bytes): Encoded image.

    Returns:
        tuple[int, int]: (width, height).

    Raises:
        ValueError: When the bytes are not a decodable image.
    """
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("the head camera frame is not a decodable image")
    return img.shape[1], img.shape[0]


def aim_head_at_base_point(
    ctx: ToolContext, tool: str, point: np.ndarray, what: str = "the tool point"
) -> tuple[str | None, bool]:
    """Aim the head D435 colour camera at a base_link point and wait for the head to settle.

    The head moves only while the agent holds a lease that commands it (the arm or the base lease, as the head tools).
    When the aim needs angles beyond the head limits (a point low in front of the robot needs more tilt than the
    head has), the head aims at the nearest in-limit pose instead (each angle clamped to its limits): the point may
    still be in the image. HeadController.move waits until both joints are within head.tolerance_rad or
    head.move_timeout_s passes.

    Args:
        ctx (ToolContext): Shared registration context (robot, config, control).
        tool (str): Calling tool (named in a lease refusal).
        point (np.ndarray): (3,) point in base_link (m).
        what (str): What the point is, with its article (worded into the reasons).

    Returns:
        tuple[str | None, bool]: (None when the head settled on the aim, else why it was not aimed (nothing moved) or
            did not settle; whether the aim was clamped to the head limits).
    """
    robot, config = ctx.robot, ctx.config
    refusal = None if ctx.control is None else ctx.control.refusal(tool, "any")
    if refusal is not None:
        return f"head not aimed at {what}: {refusal}", False
    kin = robot.head.kinematics
    at_limit = False
    try:
        try:
            pan, tilt = kin.look_at(point)
        except HeadAimError as exc:
            if kin.within_limits(exc.pan, exc.tilt):
                raise  # too near the camera, or no solve: not a limit the clamp can help with
            (lo_pan, hi_pan), (lo_tilt, hi_tilt) = kin.limits[PAN_JOINT], kin.limits[TILT_JOINT]
            pan, tilt, at_limit = min(max(exc.pan, lo_pan), hi_pan), min(max(exc.tilt, lo_tilt), hi_tilt), True
        result = robot.head.move(pan, tilt)
    except (HeadAimError, HeadError) as exc:
        return f"head not aimed at {what}: {exc}", False
    if result.status != "converged":
        head = config.head
        return (
            f"the head did not settle on {what} ({result.status}: not within head.tolerance_rad "
            f"{head.tolerance_rad:g} rad after head.move_timeout_s {head.move_timeout_s:g} s)"
        ), at_limit
    return None, at_limit


def aim_head_at_arm_point(
    ctx: ToolContext, tool: str, point: tuple[float, float, float], what: str
) -> tuple[str | None, bool]:
    """Aim the head D435 at a point given in the arm base frame (see aim_head_at_base_point).

    Args:
        ctx (ToolContext): Shared registration context.
        tool (str): Calling tool (named in a lease refusal).
        point (tuple[float, float, float]): (x, y, z) in the arm base frame (m).
        what (str): What the point is, with its article (worded into the reasons).

    Returns:
        tuple[str | None, bool]: As aim_head_at_base_point.
    """
    offset = ctx.config.arm.base_in_base_link
    if offset is None:
        return f"arm.base_in_base_link is not configured: {what} cannot be placed for the head aim", False
    return aim_head_at_base_point(ctx, tool, arm_to_base_link(point, offset), what)


def aim_head_at_tool_point(ctx: ToolContext, tool: str, joints: dict[str, float] | None) -> tuple[str | None, bool]:
    """Aim the head D435 colour camera at the FK tool point and wait for the head to settle.

    Args:
        ctx (ToolContext): Shared registration context (robot, config, control).
        tool (str): Calling tool (named in a lease refusal).
        joints (dict[str, float] | None): Measured arm joints; None without fresh joint states.

    Returns:
        tuple[str | None, bool]: As aim_head_at_base_point (None when the head settled on the aim, else why not;
            whether the aim was clamped to the head limits).
    """
    robot = ctx.robot
    if joints is None:
        return "no fresh arm joint states to place the tool point: the head was not aimed", False
    offset = ctx.config.arm.base_in_base_link
    if offset is None:
        return "arm.base_in_base_link is not configured: the tool point cannot be placed for the head aim", False
    return aim_head_at_base_point(ctx, tool, tool_point_in_base(robot.arm.kin, robot.arm.jaw, joints, offset))


def jaw_crop_jpeg(ctx: ToolContext, tool: str) -> tuple[bytes, int, int, CropInfo]:
    """A fresh head camera frame, the head aimed at the tool point, cropped to the region between the jaws.

    The head is aimed first (aim_head_at_tool_point, clamped to the head limits when the tool point lies beyond them);
    the frame is newer than the motion and the aim that just ended
    (camera_image waits for the next frame). The box is projected through the head camera scene at the measured head
    angles in the calibrated image size and scaled to the frame's size. When the head could not aim, the camera is
    uncalibrated or the jaw cannot be cropped (outside the image, inside the D435 min-Z, too small), the image is the
    full frame and the crop info carries the reason.

    Args:
        ctx (ToolContext): Shared registration context (robot, config, control).
        tool (str): Calling tool (named in a lease refusal).

    Returns:
        tuple[bytes, int, int, CropInfo]: (JPEG, width, height, crop info without the lookup id).

    Raises:
        RobotError: When the camera gives no frame.
        ValueError: When the frame cannot be decoded.
    """
    robot, config = ctx.robot, ctx.config
    settings = config.grasp_check
    max_px = min(settings.image_px, config.limits.max_image_px)
    before = robot.arm.fresh_sample()
    reason, at_limit = aim_head_at_tool_point(ctx, tool, None if before is None else robot.arm.measured(before))
    try:
        setup = camera_setup(config, GRASP_CAMERA, source=robot)
        calibrated: tuple[int, int] | None = (setup.intrinsics.width, setup.intrinsics.height)
    except ValueError as exc:  # not calibrated: no projection, the full frame is the image
        setup, calibrated, reason = None, None, reason or str(exc)
    request_px = max_px if calibrated is None else min(HARD_MAX_IMAGE_PX, max(calibrated))
    frame = robot.camera_image(GRASP_CAMERA, request_px)
    box: tuple[int, int, int, int] | None = None
    if reason is None and setup is not None and calibrated is not None:
        sample = robot.arm.fresh_sample()
        try:
            if sample is None:
                raise ValueError("no fresh arm joint states to project the jaws")
            scene = build_scene(setup, config, head_parent_transform(config, robot.head))
            x0, y0, x1, y1 = jaw_crop(scene, robot.arm.kin, robot.arm.jaw, robot.arm.measured(sample), settings)
            sx, sy = frame.width / calibrated[0], frame.height / calibrated[1]
            box = (round(x0 * sx), round(y0 * sy), round(x1 * sx), round(y1 * sy))
        except JawCropError as exc:
            reason = f"{JAW_AT_LIMIT}: {exc}" if at_limit else str(exc)
        except ValueError as exc:  # no fresh arm sample, or stale head joint states for the scene
            reason = str(exc)
    if box is not None:
        try:
            jpeg = crop_jpeg(frame.jpeg, box, max_px)
            width, height = jpeg_size(jpeg)
            return jpeg, width, height, CropInfo(lookup_id=None, box_px=box, full_frame_reason=None)
        except ValueError as exc:
            reason = f"the jaw crop failed: {exc}"
    jpeg = frame.jpeg
    if max(frame.width, frame.height) > max_px:
        jpeg = crop_jpeg(jpeg, (0, 0, frame.width, frame.height), max_px)
        return jpeg, *jpeg_size(jpeg), CropInfo(lookup_id=None, box_px=None, full_frame_reason=reason)
    return jpeg, frame.width, frame.height, CropInfo(lookup_id=None, box_px=None, full_frame_reason=reason)


def attach_jaw_crop(
    ctx: ToolContext, tool: str, check: GraspCheck
) -> tuple[ImageContent | None, GraspCheck, str | None]:
    """Attach the jaw crop to a held close and leave its hold unverified until confirm_grasp.

    Takes the frame (jaw_crop_jpeg: the head aimed at the tool point), records it as a lookup, and marks the close's
    carry record unverified with a pending verdict (ArmController.mark_visual_pending). A camera failure attaches
    nothing: image 'not_attached' and a warning, the hold is still unverified.

    Args:
        ctx (ToolContext): Shared registration context.
        tool (str): Calling tool (the lookup's tool name).
        check (GraspCheck): Grasp check of the held close.

    Returns:
        tuple[ImageContent | None, GraspCheck, str | None]: (image, check with image pending / not_attached and the
            crop info, warning for the result or None).
    """
    image: ImageContent | None = None
    warning: str | None = None
    updates: dict[str, Any]
    try:
        jpeg, width, height, crop = jaw_crop_jpeg(ctx, tool)
        # a crop's field of view is not the camera's: only the uncropped fallback frame records one
        lookup_id = record_lookup(ctx, tool, GRASP_CAMERA, jpeg, width, height, full_frame=crop.box_px is None)
        image = ImageContent(type="image", data=base64.b64encode(jpeg).decode(), mime_type="image/jpeg")
        updates = {"image": "pending", "crop": crop.model_copy(update={"lookup_id": lookup_id})}
    except (RobotError, ValueError) as exc:
        warning = (
            f"no head camera image attached: {exc}; call get_camera_image 'head', look at it and call confirm_grasp"
        )
        updates = {"image": "not_attached", "crop": None}
    try:
        ctx.robot.arm.mark_visual_pending(check.grasp_id)
    except ArmError as exc:  # superseded meanwhile (e.g. a stop opened the gripper): nothing is pending any more
        warning = f"{warning + '; ' if warning else ''}{exc}"
    return image, check.model_copy(update=updates), warning


class ConfirmGraspResult(BaseModel):
    """Reply of confirm_grasp."""

    grasp_id: str
    band: Band = Field(description="Sensor verdict of the close: object / empty / ambiguous")
    image: ImageState = Field(description="Your verdict, as recorded: yes / no / unsure")
    fused: FusedOutcome = Field(description="grasped (held), unverified (look again or release) or missed (empty)")
    evidence: GripEvidence | None = Field(
        description="Sensor evidence of the close; null after an mcp_server restart (only the band is kept)"
    )
    carrying: CarryingState = Field(description="get_control_state arm.carrying after the verdict")
    next: str = Field(description="What to do now")


def verdict_next(fused: FusedOutcome, band: Band, image: ImageVerdict, relaxed: bool) -> str:
    """What the agent does after a verdict.

    Args:
        fused (FusedOutcome): Fused outcome.
        band (Band): Sensor band.
        image (ImageVerdict): The agent's verdict.
        relaxed (bool): The unverified-hold torque relax already restored the default torque limit.

    Returns:
        str: The advice.
    """
    if fused == "grasped":
        note = (
            " The unverified hold waited longer than grip_profiles.unverified_hold_relax_s, so the default torque "
            "limit was restored and the hold is weaker than the profile's: re-grip (release_object and grasp again) "
            "if the object slips."
            if relaxed
            else ""
        )
        return f"grasped: the object is held (band {band}, image {image}); continue, complete_phase may follow.{note}"
    if fused == "missed":
        return (
            f"missed: sensors (band {band}) and image ({image}) agree the jaws are empty and nothing is carried "
            "any more, but the jaws are still closed: call release_object (opens and lifts), then retry the grasp"
        )
    return (
        f"unverified: band {band} with image {image} does not confirm a grasp: look again (another wrist roll, "
        "get_camera_image 'head') and call confirm_grasp again, or release_object and retry"
    )


def grip_profile_description(config: McpServerConfig) -> str:
    """Tool argument description of grip_profile with the configured presets.

    Args:
        config (McpServerConfig): Node configuration.

    Returns:
        str: The description.
    """
    grip = config.grip_profiles
    presets = "; ".join(
        f"{name}: torque_limit {p.torque_limit}, squeeze {p.squeeze_rad:g} rad, close {p.close_speed_rps:g} rad/s"
        + (f", stop at closing load {p.target_load:g}" if p.target_load is not None else "")
        for name, p in grip.presets.items()
    )
    return (
        f"Grip strength: a preset name ({presets}) or inline overrides {{base?, squeeze_rad?, torque_limit?, "
        "close_speed_rps?, target_load?, contact_effort_threshold?, crush_load?, retighten?}. Default "
        f"{grip.default_grip_profile!r}. gentle for fragile or light objects; normal for ordinary ones; firm "
        "for heavy, slippery objects and tools; soft for plush, foam or fabric (plush / soft toys: they compress "
        "under the hold, soft squeezes the full cap past the contact). With a retighten profile (not gentle), a hold "
        "that relaxes to about 0 load with the jaw short of closed is re-tightened once within the caps "
        "(retightened: true; back to the previous hold when it then shows crush_risk: retighten_reverted: true). "
        "Hard caps (enforced whatever you pass): torque_limit <= "
        f"{grip.torque_limit_max}, squeeze_rad <= {grip.squeeze_max_rad:g}. The default torque limit is restored "
        "after an open, release or abort."
    )


def object_description(config: McpServerConfig) -> str:
    """Tool argument description of the object box (shared by plan_grasp, grasp_object and grasp_approach).

    Args:
        config (McpServerConfig): Node configuration.

    Returns:
        str: The description.
    """
    return (
        "Object as a box: {frame: 'arm' (arm base frame, z up, floor at z = "
        f"{config.arm.floor_z_m:.3f}) or 'base_link' (robot frame, floor z = 0), x, y (centre, m), support_z (height "
        "of the object's bottom = the surface it stands on, m), width_m (across the jaws), depth_m (along the "
        "approach), height_m, yaw (rad, direction of the width axis; omit = across the approach), gap_below_m (clear "
        "height under the object's bottom, m: an overhang or a raised object; default 0 = resting flat), surfaces "
        "(optional regions around the object, as the surfaces argument), position_source ('manual' default, or "
        "'camera' when x, y, support_z were read with pixel_to_point / camera_point_to_arm / pixel_to_ground: "
        "grasp_object then refuses unless the camera-to-arm residual (the accepted arm to camera calibration's held-out "
        "tool error when it was fitted against the applied head chain, else the head-from-arm residual) is known and "
        f"at most {config.camera_arm.max_residual_mm:g} mm; plan_grasp only notes it). With surfaces, support_z must match the "
        "region under the object centre (else infeasible)}"
    )


def strategy_description(config: McpServerConfig) -> str:
    """Tool argument description of the strategy choice.

    Args:
        config (McpServerConfig): Node configuration.

    Returns:
        str: The description.
    """
    grasp = config.grasp
    return (
        "'scoop': fixed jaw slides under the object (wrist_roll about 0, moving jaw closes from above), only with a gap "
        "under the object (gap_below_m >= jaw_thickness_m + scoop_gap_margin_m; a box resting flat gets pushed, so "
        "the plan is infeasible: 'no gap under object for the fixed jaw'), 'angled': "
        "radial approach pitched down by approach_pitch_deg (default "
        f"{grasp.angled_pitch_deg:g}), jaws across the width, 'top_down': gripper straight down, jaws across the width, "
        "'auto' (default): tries " + ", ".join(e.strategy for e in grasp.auto_order) + " and takes the first feasible. "
        f"Tall narrow objects (height / width above tall_ratio {grasp.tall_ratio:g}) are gripped at "
        f"tall_grasp_height_fraction {grasp.tall_grasp_height_fraction:g} of their height and lifted at "
        f"lift_speed_scale {grasp.lift_speed_scale:g}; objects narrower than min_object_width_m "
        f"{grasp.min_object_width_m:g} m are rejected"
    )


def params_description(config: McpServerConfig) -> str:
    """Tool argument description of the grasp parameter overrides.

    Args:
        config (McpServerConfig): Node configuration.

    Returns:
        str: The description.
    """
    grasp = config.grasp
    return (
        "Overrides of the grasp parameters (config grasp section), e.g. approach_distance_m, pre_grasp_clearance_m, "
        f"approach_speed_scale (free moves to the lifted pre-grasp and its roll, default {grasp.approach_speed_scale:g}, "
        f"capped to {config.limits.arm_max_speed_scale:g}), slide_speed_scale (straight approach, slide and retreat, "
        f"default {grasp.slide_speed_scale:g}), lift_height_m, retreat_distance_m, jaw_thickness_m, jaw_open_margin_m, "
        "below_object_offset_m, skim_clearance_m, max_object_width_m, close_effort_threshold (default: the grip "
        "profile's contact_effort_threshold), grip_profile, interpolation_step_m, "
        "scoop_max_pitch_deg, scoop_gap_margin_m, tall_ratio, tall_grasp_height_fraction, lift_speed_scale, "
        "min_object_width_m, angled_pitch_deg, release_open_fraction, release_lift_m, auto_order, "
        "surface_jaw_clearance_m, surface_link_clearance_m, surface_mismatch_tolerance_m, step_pitches_deg, "
        "step_pre_grasp_clearance_m"
    )


def surface_description(config: McpServerConfig) -> str:
    """Tool argument description of surface_z_m.

    Args:
        config (McpServerConfig): Node configuration.

    Returns:
        str: The description.
    """
    slow = config.floor_guard
    return (
        "Expected surface height relative to the robot plane (m, base_link z; default "
        f"{slow.surface_z_m:g}): e.g. -0.18 for an object on a stair below or in a hole. Arm motions run at normal "
        f"speed down to that surface; within {slow.margin_m:g} m of it or below they slow to {slow.slow_speed_scale:g} "
        "of their speed (the slow zone never blocks). The scoop skims this surface."
    )


def slow_description(config: McpServerConfig) -> str:
    """Slow-zone sentence shared by the arm tool descriptions.

    Args:
        config (McpServerConfig): Node configuration.

    Returns:
        str: The description.
    """
    slow = config.floor_guard
    return (
        "Every arm motion slows (never stops) where a jaw tip, the wrist or the elbow comes within "
        f"{slow.margin_m:g} m of the effective surface: the higher of the robot plane at surface_z_m (or the local "
        "height of the surfaces regions, incl. their step faces) and the level plane from the IMU tilt. For an "
        "object beyond a step edge the planner tries a higher pre-grasp and steeper pitches so the wrist clears the "
        "edge."
    )


def register(ctx: ToolContext) -> None:
    """Register plan_grasp, grasp_object and release_object.

    Args:
        ctx (ToolContext): Shared registration context.
    """
    robot, config = ctx.robot, ctx.config
    executor = GraspExecutor(robot.arm, config, lambda: robot.head.calibration())
    grasp = config.grasp
    grasp_check = config.grasp_check
    object_desc = object_description(config)
    strategy_desc = strategy_description(config)
    params_desc = params_description(config)
    surface_desc = surface_description(config)
    tilt_desc, pitch_desc = TILT_DESC, PITCH_DESC
    grip_desc = grip_profile_description(config)
    slow_note = slow_description(config)

    def run(fn: Callable[[], GraspResult]) -> GraspResult:
        try:
            return fn()
        except (ArmError, ValueError) as exc:
            raise ToolError(str(exc)) from exc

    @ctx.server.tool(
        description=(
            "Dry run of a grasp (no motion): plans waypoints pre_grasp (lifted; roll set here with the gripper at most "
            "half open), open, approach, grasp (slide), close, lift, retreat with IK along straight lines, and "
            "returns feasibility, human-readable reasons when infeasible, the chosen strategy, approach pitch, wrist "
            "roll, jaw opening, the waypoints (arm frame) and where the slow zone applies. angled and top_down centre "
            "the jaw opening on the object (plan grasp_shift; both jaws clear its side faces by fixed_jaw_clearance_m, "
            "the moving jaw also deep in the jaws, where its open face is tilted): waypoint x, y, z is then the JAW "
            "CENTRE (the object centre) and each waypoint's tool_point (the fixed jaw inner face, what get_arm_state "
            "reports as the tool pose) lies shift_m (half the object width plus that clearance) beside it, so the arm "
            "stands sideways of a plain "
            "move_arm_cartesian to the same x, y, z: that is the centring, not a drift. The 5-DOF arm can only "
            "approach radially from its base: turn the robot for another approach "
            f"direction. Call it before grasp_object. {slow_note}"
        )
    )
    def plan_grasp(
        object: Annotated[ObjectSpec, Field(description=object_desc)],
        strategy: Annotated[StrategyChoice, Field(description=strategy_desc)] = "auto",
        params: Annotated[dict[str, Any] | None, Field(description=params_desc)] = None,
        approach_pitch_deg: Annotated[float | None, Field(ge=0.0, le=90.0, description=pitch_desc)] = None,
        surface_z_m: Annotated[float | None, Field(ge=-1.0, le=1.0, description=surface_desc)] = None,
        tilt_override_deg: Annotated[TiltOverrideDeg | None, Field(description=tilt_desc)] = None,
        use_ground_estimate: Annotated[bool, Field(description=USE_GROUND_DESC)] = False,
        grip_profile: Annotated[str | GripProfileOverride | None, Field(description=grip_desc)] = None,
        surfaces: Annotated[
            list[SurfaceRegion] | None, Field(max_length=MAX_REGIONS, description=SURFACES_DESC)
        ] = None,
    ) -> GraspResult:
        """Plan a grasp; the tool description is passed to the decorator so it can state the slow zone."""

        def body() -> GraspResult:
            floor = floor_override(surface_z_m, tilt_override_deg, surfaces, use_ground_estimate)
            grasp_call = grasp_params(grasp, params, grip_profile)
            grip = executor.resolve_grip(grasp_call)
            plan, notes = executor.plan(object, strategy, grasp_call, approach_pitch_deg, floor)
            refusal = executor.camera_refusal(object)
            if refusal is not None:
                notes = [*notes, f"grasp_object will refuse this camera position: {refusal}"]
            rezero = executor.rezero_warning(object)
            if rezero is not None:
                notes = [*notes, rezero]
            GRASP_PLANS.labels(strategy, "feasible" if plan.feasible else "infeasible").inc()
            return plan_result(plan, notes, grip)

        return run(body)

    @ctx.server.tool(
        description=(
            "Plan and execute a grasp (see plan_grasp; needs the arm lease, take_control(subsystem='arm')): rolls only at the lifted pre-grasp with the "
            "gripper half open, opens to the object size plus a margin, approaches and slides in slowly, closes until "
            "the gripper feels the object (never a full squeeze), verifies the grasp (jaw stopped short of closed and "
            "load above threshold), then lifts and retreats. The grip_profile sets how hard it grips (torque limit, close "
            "speed, squeeze); the result reports grip_profile, holding_load, slipping, crush_risk and retightened (a "
            "relaxed hold stepped once further toward closed). Every arm joint "
            "stays commanded at its planned target; residual_error and warnings list joints that ended a step (or the "
            "hold) off it by more than their tolerance, held_pose where the jaw centre and the tool point ended. "
            "Outcome: every close that holds something (a verified contact, or a jaw that closed into the empty band, "
            "within gripper_empty_stall_rad of closed, where the load cannot tell a soft, squashed object from empty "
            "jaws) stays closed with the profile squeeze, lifts and retreats and comes back 'unverified': the result "
            "carries a fresh head camera image (the head D435 aimed at the tool point first, at the nearest pose "
            "inside the head limits when it lies beyond them; only the head moves) "
            f"cropped to the jaws (max {grasp_check.image_px} px; the full frame with grasp_check.crop.full_frame_reason "
            "when the head cannot aim (no lease, not settled), the camera is uncalibrated or the jaw lies outside the "
            "image (also at the head limit) or inside the D435 min-Z) and grasp_check.grasp_id. Look "
            "at the attached crop and call confirm_grasp(grasp_id, yes|no|unsure, evidence) before anything else. "
            f"{FUSE_NOTE} missed (a contact that failed verification: opened and retreated - check a picture and "
            "correct the object position), aborted (stopped and held; reasons say why) and infeasible (nothing "
            "moved) attach no image. Soft objects (plush toys, foam, fabric) compress under the jaws: use grip_profile "
            "soft and expect the band to be ambiguous or empty. A stop call "
            f"aborts it. Keeps the arm lease: call release_control when done. {slow_note}"
        )
    )
    def grasp_object(
        object: Annotated[ObjectSpec, Field(description=object_desc)],
        strategy: Annotated[StrategyChoice, Field(description=strategy_desc)] = "auto",
        params: Annotated[dict[str, Any] | None, Field(description=params_desc)] = None,
        approach_pitch_deg: Annotated[float | None, Field(ge=0.0, le=90.0, description=pitch_desc)] = None,
        surface_z_m: Annotated[float | None, Field(ge=-1.0, le=1.0, description=surface_desc)] = None,
        tilt_override_deg: Annotated[TiltOverrideDeg | None, Field(description=tilt_desc)] = None,
        use_ground_estimate: Annotated[bool, Field(description=USE_GROUND_DESC)] = False,
        grip_profile: Annotated[str | GripProfileOverride | None, Field(description=grip_desc)] = None,
        surfaces: Annotated[
            list[SurfaceRegion] | None, Field(max_length=MAX_REGIONS, description=SURFACES_DESC)
        ] = None,
    ) -> Annotated[CallToolResult, GraspResult]:
        """Grasp an object; the tool description is passed to the decorator. A held close is 'unverified' and carries
        a jaw crop of a fresh head camera frame (head aimed at the tool point) until the agent calls confirm_grasp."""
        ctx.battery_gate("grasp_object")
        baseline = robot.stop_count()

        def body() -> GraspResult:
            floor = floor_override(surface_z_m, tilt_override_deg, surfaces, use_ground_estimate)
            return executor.grasp(
                object,
                strategy,
                grasp_params(grasp, params, grip_profile),
                approach_pitch_deg,
                floor,
                lambda: robot.stop_count() != baseline,
            )

        result = run(body)
        image: ImageContent | None = None
        if result.outcome in HELD_OUTCOMES and result.grasp_check is not None:
            image, result.grasp_check, warning = attach_jaw_crop(ctx, "grasp_object", result.grasp_check)
            result.outcome = "unverified"
            result.reasons = [awaiting_confirm(result.grasp_check)]
            if warning is not None:
                result.warnings.append(warning)
        return grasp_tool_result(result, [image] if image else [])

    @ctx.server.tool(
        description=(
            f"Release a held object: open the gripper to params.release_open_fraction (default "
            f"{grasp.release_open_fraction:g}) and lift the tool point params.release_lift_m (default "
            f"{grasp.release_lift_m:g} m) straight up. Outcome released (reasons note a skipped, unreachable lift) or "
            "aborted. After an 'unverified' grasp_object whose attached image (jaw crop) shows empty jaws (or not "
            "the object), or a confirm_grasp that came back 'missed', call this to open and lift away, then retry the grasp. Needs the arm lease. "
            f"{slow_note}"
        )
    )
    def release_object(
        params: Annotated[dict[str, Any] | None, Field(description=params_desc)] = None,
        surface_z_m: Annotated[float | None, Field(ge=-1.0, le=1.0, description=surface_desc)] = None,
        tilt_override_deg: Annotated[TiltOverrideDeg | None, Field(description=tilt_desc)] = None,
        use_ground_estimate: Annotated[bool, Field(description=USE_GROUND_DESC)] = False,
        surfaces: Annotated[
            list[SurfaceRegion] | None, Field(max_length=MAX_REGIONS, description=SURFACES_DESC)
        ] = None,
    ) -> GraspResult:
        """Release an object; the tool description is passed to the decorator."""
        ctx.battery_gate("release_object")
        baseline = robot.stop_count()

        def body() -> GraspResult:
            floor = floor_override(surface_z_m, tilt_override_deg, surfaces, use_ground_estimate)
            return executor.release(grasp_params(grasp, params), floor, lambda: robot.stop_count() != baseline)

        return run(body)

    @ctx.server.tool(
        description=(
            "Record your visual verdict on the last held close (grasp_object or set_gripper close_until_effort) and "
            "get the fused outcome (never moves anything, needs no lease). Look at the jaw crop that came with the "
            "result, then answer object_between_fingers: yes (the object is clearly between the fingers), no (the "
            f"jaws are empty or hold something else), unsure; evidence is one sentence on what you saw. {FUSE_NOTE} "
            "Fused grasped sets the carry record to a confirmed grasp; unverified keeps it unverified; missed clears "
            "it (the jaws stay closed: call release_object). grasp_id must be the one of the last close: a later "
            "close or an open supersedes it (error, nothing changes). Calling again overwrites the earlier verdict. "
            "Returns {grasp_id, band, image, fused, evidence (the sensor evidence), carrying, next}."
        )
    )
    def confirm_grasp(
        grasp_id: Annotated[str, Field(description="grasp_check.grasp_id of the grasp_object / set_gripper result")],
        object_between_fingers: Annotated[
            ImageVerdict, Field(description="Is the object between the fingers in the jaw crop: yes, no or unsure")
        ],
        evidence: Annotated[
            str, Field(min_length=1, max_length=500, description="What you saw in the crop that supports the answer")
        ],
    ) -> ConfirmGraspResult:
        """Fuse the agent's verdict with the sensor band; the tool description is passed to the decorator."""
        arm = robot.arm
        control = ctx.control
        if control is None:
            raise ToolError("the control manager is not available")
        try:
            previous = arm.current_grasp(grasp_id)
            relaxed = previous.grasp_kind == "unverified" and arm._unverified_since is None
            record = arm.record_visual_verdict(grasp_id, object_between_fingers)
        except ArmError as exc:
            raise ToolError(str(exc)) from exc
        assert record.band is not None and record.fused is not None  # a recorded verdict sets both
        LOGGER.info("confirm_grasp %s: %s (%s)", grasp_id, object_between_fingers, evidence)
        GRASP_VERDICTS.labels(record.band, object_between_fingers, record.fused).inc()
        check = arm.last_grasp_check
        return ConfirmGraspResult(
            grasp_id=grasp_id,
            band=record.band,
            image=object_between_fingers,
            fused=record.fused,
            evidence=None if check is None or check.grasp_id != grasp_id else check.evidence,
            carrying=control.carrying_state(),
            next=verdict_next(record.fused, record.band, object_between_fingers, relaxed),
        )
