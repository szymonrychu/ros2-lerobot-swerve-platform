"""Grasp execution and its interfaces: GraspExecutor (runs a GraspPlan through ArmController), the MCP tools
plan_grasp / grasp_object / release_object, and GraspService, the JSON request handler behind the /grasp/command and
/grasp/result topics the web UI uses (contract in the README, "Grasp macros")."""

import json
import math
import threading
from collections.abc import Callable
from typing import Annotated, Any, Literal, Protocol

from mcp.server.mcpserver.exceptions import ToolError
from pydantic import BaseModel, ConfigDict, Field, ValidationError

from .arm import ROLL_JOINT, ArmController, ArmError
from .config import McpServerConfig
from .floor_guard import FloorOverride, TiltOverrideDeg
from .grasp import GraspParams, GraspPlan, GraspPlanner, ObjectSpec, Waypoint, grasp_params
from .models import ArmMotionResult
from .tool_context import ToolContext

TOOL_NAMES = ("plan_grasp", "grasp_object", "release_object")
# IK seed when no fresh joint states are available for a dry-run plan (folded arm, gripper camera looking forward).
DEFAULT_SEED = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 1.2, "wrist_flex": 0.3, "wrist_roll": 0.0}
MOTION_OK = frozenset({"converged"})
CLOSE_OK = frozenset({"grasped", "closed_no_contact", "blocked"})

StrategyChoice = Literal["auto", "scoop", "angled", "top_down"]
GraspOutcome = Literal["planned", "infeasible", "grasped", "missed", "aborted", "released"]


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
    slow_zone: dict[str, Any] | None = None


class GraspResult(BaseModel):
    """Outcome of plan_grasp / grasp_object / release_object (and the GraspService results)."""

    outcome: GraspOutcome = Field(
        description="planned (dry run, feasible), infeasible (nothing moved), grasped (holding, lifted and retreated), "
        "missed (closed without an object: opened and retreated), aborted (stopped and held; see reasons), released"
    )
    reasons: list[str] = Field(default_factory=list, description="Why a plan is infeasible or a run ended early")
    plan: dict[str, Any] | None = Field(default=None, description="Plan summary: waypoints, pitch, roll, slow zone")
    steps: list[GraspStep] = Field(default_factory=list)
    gripper_position_rad: float | None = None
    gripper_effort: float | None = None


class GraspAbort(Exception):
    """Internal: a step failed; the run stops, holds and reports the reason."""


def floor_override(surface_z_m: float | None, tilt_override_deg: TiltOverrideDeg | None) -> FloorOverride | None:
    """Slow-zone override of a call, or None when the call overrides nothing.

    Args:
        surface_z_m (float | None): Expected surface height relative to the robot plane (m).
        tilt_override_deg (TiltOverrideDeg | None): Tilt replacing the IMU.

    Returns:
        FloorOverride | None: The override.
    """
    if surface_z_m is None and tilt_override_deg is None:
        return None
    return FloorOverride(surface_z_m=surface_z_m, tilt_override_deg=tilt_override_deg)


def plan_result(plan: GraspPlan, notes: list[str] | None = None) -> GraspResult:
    """Dry-run result of a plan.

    Args:
        plan (GraspPlan): The plan.
        notes (list[str] | None): Extra notes (e.g. planned from a default seed).

    Returns:
        GraspResult: 'planned' or 'infeasible'.
    """
    return GraspResult(
        outcome="planned" if plan.feasible else "infeasible",
        reasons=[*plan.reasons, *(notes or [])],
        plan=plan.summary(),
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

    def __init__(self, arm: ArmController, config: McpServerConfig) -> None:
        """Bind to the arm.

        Args:
            arm (ArmController): Arm controller.
            config (McpServerConfig): Node configuration.
        """
        self.arm = arm
        self.cfg = config
        self.planner = GraspPlanner(arm.kin, config, arm.floor_guard, arm.jaw)

    def fraction(self, angle: float) -> float:
        """Gripper open fraction of a gripper angle.

        Args:
            angle (float): Gripper joint position (rad).

        Returns:
            float: Fraction in [0, 1] (0 closed, 1 gripper_open_rad).
        """
        closed, opened = self.cfg.arm.gripper_closed_rad, self.cfg.arm.gripper_open_rad
        return min(1.0, max(0.0, (angle - closed) / (opened - closed)))

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
        sample = self.arm.fresh_sample()
        notes: list[str] = []
        if sample is None:
            seed = dict(DEFAULT_SEED)
            notes.append("no fresh joint states: planned from a default folded seed pose")
        else:
            seed = self.arm.command_base(sample)
        surface = self.arm.floor_guard.surface(floor, self.arm.tilt_source(), self.arm.backend.now())
        return self.planner.plan(obj, strategy, params, surface, seed, approach_pitch_deg), notes

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
            GraspResult: infeasible / grasped / missed / aborted.
        """
        plan, notes = self.plan(obj, strategy, params, approach_pitch_deg, floor)
        if not plan.feasible:
            return plan_result(plan, notes)
        return self.execute(plan, params, floor, stop_requested)

    def step(
        self,
        label: str,
        steps: list[GraspStep],
        stop_requested: Callable[[], bool],
        motion: Callable[[], ArmMotionResult],
        ok: frozenset[str] = MOTION_OK,
    ) -> ArmMotionResult:
        """Run one motion step, recording it; abort the run on a stop, a refusal or an unexpected status.

        Args:
            label (str): Step label.
            steps (list[GraspStep]): Step log (appended).
            stop_requested (Callable[[], bool]): Stop check.
            motion (Callable[[], ArmMotionResult]): The arm call.
            ok (frozenset[str]): Statuses that continue the run.

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
        steps.append(GraspStep(label=label, status=result.status, message=result.message, slow_zone=result.slow_zone))
        if result.status not in ok:
            raise GraspAbort(f"{label}: {result.status}: {result.message}")
        return result

    def finish(
        self, outcome: GraspOutcome, reasons: list[str], plan: GraspPlan | None, steps: list[GraspStep]
    ) -> GraspResult:
        """Result with the final gripper state.

        Args:
            outcome (GraspOutcome): Outcome.
            reasons (list[str]): Reasons.
            plan (GraspPlan | None): Executed plan.
            steps (list[GraspStep]): Step log.

        Returns:
            GraspResult: The result.
        """
        sample = self.arm.fresh_sample()
        gripper = self.arm.gripper
        return GraspResult(
            outcome=outcome,
            reasons=reasons,
            plan=None if plan is None else plan.summary(),
            steps=steps,
            gripper_position_rad=None if sample is None else sample.positions.get(gripper),
            gripper_effort=None if sample is None else sample.efforts.get(gripper),
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
        sample = self.arm.fresh_sample()
        if sample is None:
            return "no fresh joint states to verify the grasp"
        jaw = sample.positions[self.arm.gripper]
        gap = jaw - self.cfg.arm.gripper_closed_rad
        load = max(abs(close.gripper_effort or 0.0), abs(sample.efforts.get(self.arm.gripper, 0.0)))
        if gap < params.min_hold_gap_rad:
            return (
                f"jaw reached {jaw:.3f} rad, within {params.min_hold_gap_rad} rad of closed: nothing between the jaws"
            )
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

        A miss (closed without load) opens the gripper, lifts and retreats; any abort stops and holds the arm.

        Args:
            plan (GraspPlan): Feasible plan.
            params (GraspParams): Parameters.
            floor (FloorOverride | None): Slow-zone overrides.
            stop_requested (Callable[[], bool]): Stop check between steps.

        Returns:
            GraspResult: grasped / missed / aborted.
        """
        steps: list[GraspStep] = []
        arm, gripper, limits = self.arm, self.arm.gripper, self.cfg.limits
        wp: dict[str, Waypoint] = {w.label: w for w in plan.waypoints}
        slide = params.slide_speed_scale
        try:
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
            self.step("pre_grasp", steps, stop_requested, lambda: arm.move_joints(targets, None, floor))
            if roll_change:
                self.step("roll", steps, stop_requested, lambda: arm.move_joints({ROLL_JOINT: pre.roll}, None, floor))
            open_frac = self.fraction(wp["open"].gripper or self.cfg.arm.gripper_open_rad)
            self.step("open", steps, stop_requested, lambda: arm.set_gripper(open_fraction=open_frac, floor=floor))
            for label in ("approach", "grasp"):
                path = arm_only(plan.segments[label], gripper)
                self.step(label, steps, stop_requested, lambda p=path: arm.move_path(p, slide, floor))
            close = self.step(
                "close",
                steps,
                stop_requested,
                lambda: arm.set_gripper(
                    close_until_effort=True, effort_threshold=params.close_effort_threshold, floor=floor
                ),
                CLOSE_OK,
            )
            missed = self.verify(close, params)
            if missed is not None:
                self.step("open", steps, stop_requested, lambda: arm.set_gripper(open_fraction=open_frac, floor=floor))
            for label in ("lift", "retreat"):
                path = arm_only(plan.segments[label], gripper)
                speed = wp[label].speed_scale  # a tall narrow object lifts at lift_speed_scale
                self.step(label, steps, stop_requested, lambda p=path, v=speed: arm.move_path(p, v, floor))
        except GraspAbort as exc:
            arm.stop_hold()
            return self.finish("aborted", [str(exc)], plan, steps)
        if missed is not None:
            return self.finish("missed", [missed], plan, steps)
        return self.finish("grasped", [], plan, steps)

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
        steps: list[GraspStep] = []
        arm = self.arm
        frac = params.release_open_fraction
        try:
            self.step("open", steps, stop_requested, lambda: arm.set_gripper(open_fraction=frac, floor=floor))
            try:
                pose = arm.kin.forward(arm.command_base(arm.require_sample()))
            except ArmError as exc:
                raise GraspAbort(str(exc)) from exc
            lift = self.step(
                "lift",
                steps,
                stop_requested,
                lambda: arm.move_cartesian(
                    pose.x, pose.y, pose.z + params.release_lift_m, pose.pitch, params.slide_speed_scale, floor=floor
                ),
                MOTION_OK | {"unreachable"},
            )
        except GraspAbort as exc:
            arm.stop_hold()
            return self.finish("aborted", [str(exc)], None, steps)
        reasons = [] if lift.status == "converged" else [f"released; lift skipped: {lift.message}"]
        return self.finish("released", reasons, None, steps)


class GraspServiceRequest(BaseModel):
    """JSON request on /grasp/command (see the README for the full contract)."""

    model_config = ConfigDict(extra="forbid")

    action: Literal["plan", "execute", "release", "stop"]
    request_id: str | None = None
    object: ObjectSpec | None = None
    strategy: StrategyChoice = "auto"
    params: dict[str, Any] | None = None
    approach_pitch_deg: float | None = Field(default=None, ge=0.0, le=90.0)
    surface_z_m: float | None = Field(default=None, ge=-1.0, le=1.0)
    tilt_override_deg: TiltOverrideDeg | None = None


class GraspService:
    """Web-UI grasp actions as JSON in, JSON out; one execute/release at a time, 'stop' always answers at once."""

    def __init__(
        self,
        arm: ArmController,
        config: McpServerConfig,
        guard: CutoffGuard | None = None,
        stop_count: Callable[[], int] | None = None,
    ) -> None:
        """Create the service.

        Args:
            arm (ArmController): Arm controller.
            config (McpServerConfig): Node configuration.
            guard (CutoffGuard | None): Battery cut-off guard; execute/release are refused in cut-off.
            stop_count (Callable[[], int] | None): Robot stop counter (a stop tool call aborts a running grasp).
        """
        self.arm = arm
        self.cfg = config
        self.guard = guard
        self.stop_count = stop_count or (lambda: 0)
        self.executor = GraspExecutor(arm, config)
        self.cancel = threading.Event()
        self.busy = threading.Lock()

    def handle_json(self, text: str) -> str:
        """handle() for a JSON string.

        Args:
            text (str): Request JSON.

        Returns:
            str: Response JSON.
        """
        try:
            request = json.loads(text)
        except json.JSONDecodeError as exc:
            return json.dumps({"ok": False, "request_id": None, "action": None, "error": f"invalid JSON: {exc}"})
        if not isinstance(request, dict):
            return json.dumps({"ok": False, "request_id": None, "action": None, "error": "request must be an object"})
        return json.dumps(self.handle(request))

    def handle(self, request: dict[str, Any]) -> dict[str, Any]:
        """Run one request.

        Args:
            request (dict[str, Any]): {action, request_id?, object?, strategy?, params?, approach_pitch_deg?,
                surface_z_m?, tilt_override_deg?}.

        Returns:
            dict[str, Any]: {ok, request_id, action, result} or {ok: false, request_id, action, error}.
        """
        head = {"request_id": request.get("request_id"), "action": request.get("action")}
        try:
            req = GraspServiceRequest.model_validate(request)
        except ValidationError as exc:
            return head | {"ok": False, "error": f"invalid request: {exc}"}
        if req.action == "stop":
            self.cancel.set()
            held, note = self.arm.stop_hold()
            return head | {"ok": True, "result": {"arm_held": held, "message": note}}
        if req.action != "plan" and self.guard is not None and self.guard.is_cutoff():
            return head | {"ok": False, "error": self.guard.rejection_message()}
        try:
            params = grasp_params(self.cfg.grasp, req.params)
        except ValueError as exc:
            return head | {"ok": False, "error": str(exc)}
        floor = floor_override(req.surface_z_m, req.tilt_override_deg)
        if req.action in ("plan", "execute") and req.object is None:
            return head | {"ok": False, "error": f"'object' is required for {req.action}"}
        if req.action == "plan":
            assert req.object is not None
            plan, notes = self.executor.plan(req.object, req.strategy, params, req.approach_pitch_deg, floor)
            return head | {"ok": True, "result": plan_result(plan, notes).model_dump(mode="json")}
        if not self.busy.acquire(blocking=False):
            return head | {"ok": False, "error": "another grasp action is running; send action 'stop' first"}
        try:
            self.cancel.clear()
            baseline = self.stop_count()

            def stop_requested() -> bool:
                return self.cancel.is_set() or self.stop_count() != baseline

            if req.action == "execute":
                assert req.object is not None
                result = self.executor.grasp(
                    req.object, req.strategy, params, req.approach_pitch_deg, floor, stop_requested
                )
            else:
                result = self.executor.release(params, floor, stop_requested)
        finally:
            self.busy.release()
        return head | {"ok": True, "result": result.model_dump(mode="json")}


def register(ctx: ToolContext) -> None:
    """Register plan_grasp, grasp_object and release_object.

    Args:
        ctx (ToolContext): Shared registration context.
    """
    robot, config = ctx.robot, ctx.config
    executor = GraspExecutor(robot.arm, config)
    grasp = config.grasp
    slow = config.floor_guard
    object_desc = (
        "Object as a box: {frame: 'arm' (arm base frame, z up, floor at z = "
        f"{config.arm.floor_z_m:.3f}) or 'base_link' (robot frame, floor z = 0), x, y (centre, m), support_z (height "
        "of the object's bottom = the surface it stands on, m), width_m (across the jaws), depth_m (along the "
        "approach), height_m, yaw (rad, direction of the width axis; omit = across the approach), gap_below_m (clear "
        "height under the object's bottom, m: an overhang or a raised object; default 0 = resting flat)}"
    )
    strategy_desc = (
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
    params_desc = (
        "Overrides of the grasp parameters (config grasp section), e.g. approach_distance_m, pre_grasp_clearance_m, "
        "slide_speed_scale, lift_height_m, retreat_distance_m, jaw_thickness_m, jaw_open_margin_m, "
        "below_object_offset_m, skim_clearance_m, max_object_width_m, close_effort_threshold, interpolation_step_m, "
        "scoop_max_pitch_deg, scoop_gap_margin_m, tall_ratio, tall_grasp_height_fraction, lift_speed_scale, "
        "min_object_width_m, angled_pitch_deg, release_open_fraction, release_lift_m, auto_order"
    )
    surface_desc = (
        "Expected surface height relative to the robot plane (m, base_link z; default "
        f"{slow.surface_z_m:g}): e.g. -0.18 for an object on a stair below or in a hole. Arm motions run at normal "
        f"speed down to that surface; within {slow.margin_m:g} m of it or below they slow to {slow.slow_speed_scale:g} "
        "of their speed (the slow zone never blocks). The scoop skims this surface."
    )
    tilt_desc = "Robot tilt {roll, pitch} in deg replacing the IMU for the slow zone (roll > 0 left side up, pitch > 0 nose down)"
    pitch_desc = "Approach pitch for 'angled' (deg, 0 horizontal, 90 straight down)"
    slow_note = (
        "Every arm motion slows (never stops) where a jaw tip, the wrist or the elbow comes within "
        f"{slow.margin_m:g} m of the effective surface: the higher of the robot plane at surface_z_m and the level "
        "plane from the IMU tilt."
    )

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
            "roll, jaw opening, the waypoints (tool point = fixed jaw inner face, arm frame) and where the slow zone "
            "applies. The 5-DOF arm can only approach radially from its base: turn the robot for another approach "
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
    ) -> GraspResult:
        """Plan a grasp; the tool description is passed to the decorator so it can state the slow zone."""

        def body() -> GraspResult:
            floor = floor_override(surface_z_m, tilt_override_deg)
            plan, notes = executor.plan(object, strategy, grasp_params(grasp, params), approach_pitch_deg, floor)
            return plan_result(plan, notes)

        return run(body)

    @ctx.server.tool(
        description=(
            "Plan and execute a grasp (see plan_grasp): takes arm control, rolls only at the lifted pre-grasp with the "
            "gripper half open, opens to the object size plus a margin, approaches and slides in slowly, closes until "
            "the gripper feels the object (never a full squeeze), verifies the grasp (jaw stopped short of closed and "
            "load above threshold), then lifts and retreats. Outcome: grasped, missed (closed on nothing: opened and "
            "retreated - check a picture and correct the object position), aborted (stopped and held; reasons say "
            "why) or infeasible (nothing moved). A stop call aborts it. Keeps arm control: call release_control when "
            f"done. {slow_note}"
        )
    )
    def grasp_object(
        object: Annotated[ObjectSpec, Field(description=object_desc)],
        strategy: Annotated[StrategyChoice, Field(description=strategy_desc)] = "auto",
        params: Annotated[dict[str, Any] | None, Field(description=params_desc)] = None,
        approach_pitch_deg: Annotated[float | None, Field(ge=0.0, le=90.0, description=pitch_desc)] = None,
        surface_z_m: Annotated[float | None, Field(ge=-1.0, le=1.0, description=surface_desc)] = None,
        tilt_override_deg: Annotated[TiltOverrideDeg | None, Field(description=tilt_desc)] = None,
    ) -> GraspResult:
        """Grasp an object; the tool description is passed to the decorator."""
        ctx.battery_gate("grasp_object")
        baseline = robot.stop_count()

        def body() -> GraspResult:
            floor = floor_override(surface_z_m, tilt_override_deg)
            return executor.grasp(
                object,
                strategy,
                grasp_params(grasp, params),
                approach_pitch_deg,
                floor,
                lambda: robot.stop_count() != baseline,
            )

        return run(body)

    @ctx.server.tool(
        description=(
            f"Release a held object: open the gripper to params.release_open_fraction (default "
            f"{grasp.release_open_fraction:g}) and lift the tool point params.release_lift_m (default "
            f"{grasp.release_lift_m:g} m) straight up. Outcome released (reasons note a skipped, unreachable lift) or "
            f"aborted. {slow_note}"
        )
    )
    def release_object(
        params: Annotated[dict[str, Any] | None, Field(description=params_desc)] = None,
        surface_z_m: Annotated[float | None, Field(ge=-1.0, le=1.0, description=surface_desc)] = None,
        tilt_override_deg: Annotated[TiltOverrideDeg | None, Field(description=tilt_desc)] = None,
    ) -> GraspResult:
        """Release an object; the tool description is passed to the decorator."""
        ctx.battery_gate("release_object")
        baseline = robot.stop_count()

        def body() -> GraspResult:
            floor = floor_override(surface_z_m, tilt_override_deg)
            return executor.release(grasp_params(grasp, params), floor, lambda: robot.stop_count() != baseline)

        return run(body)
