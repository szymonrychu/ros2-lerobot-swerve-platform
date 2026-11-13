"""MCP tools of the guided grasp flow (grasp_approach, grasp_close, grasp_lift, grasp_abort) and their head photo.

Every flow tool ends with a head camera photo: the head aims at the FK tool point after the arm motion, and the frame
is marked with the tool point and the flow's target points so the agent can judge the jaws against the object."""

import base64
import logging
from collections.abc import Callable
from typing import Annotated, Any, Literal

import cv2
import numpy as np
from arm_collision.jaw import jaw_points
from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import CallToolResult, ImageContent
from pydantic import BaseModel, Field

from . import camera_overlay
from .arm import ArmError, finite_effort
from .camera_scene import build_scene, camera_setup, head_parent_transform
from .config import HARD_MAX_IMAGE_PX, GripProfileOverride
from .floor_guard import TiltOverrideDeg
from .grasp import GraspPlan, ObjectSpec, grasp_params
from .grasp_check import FusedOutcome, GraspCheck
from .grasp_flow import FlowPhase, GraspFlow, NextStep, next_steps, reapproach_route
from .grasp_tools import (
    GO_BACK_DESC,
    GRASP_CAMERA,
    PITCH_DESC,
    SURFACES_DESC,
    TILT_DESC,
    USE_GROUND_DESC,
    GraspAbort,
    GraspExecutor,
    GraspStep,
    RunLog,
    StrategyChoice,
    aim_head_at_arm_point,
    aim_head_at_tool_point,
    arm_only,
    attach_jaw_crop,
    floor_override,
    grasp_tool_result,
    grip_profile_description,
    jpeg_size,
    object_description,
    params_description,
    slow_description,
    strategy_description,
    surface_description,
)
from .jaw_crop import crop_jpeg
from .lookup_tools import record_lookup
from .metrics import GRASP_ATTEMPTS, GRASP_FLOW_STEPS, GRASP_VERDICTS
from .models import ArmMotionResult, RobotError
from .surfaces import MAX_REGIONS, SurfaceRegion
from .tool_context import ToolContext

TOOL_NAMES = ("grasp_approach", "grasp_close", "grasp_lift", "grasp_abort")
LOGGER = logging.getLogger("mcp_server.grasp_flow")
RELEASE_TOOL = "release_object"
MARK_TARGET = "target"
MARK_TOOL = "tool"
TOOL_COLOR = (0, 255, 255)  # BGR yellow: the FK tool point
TARGET_COLOR = (0, 200, 0)  # BGR green: the flow's target marks
JPEG_QUALITY = 85


class FlowPhoto(BaseModel):
    """The head photo a flow tool attached."""

    kind: Literal["marked_frame", "jaw_crop"] = Field(
        description="marked_frame: the head frame with the tool point and target marks; jaw_crop: the frame cropped "
        "to the jaws (grasp_close)"
    )
    lookup_id: str | None = Field(
        description="Photo POI id of the frame (list_lookups / get_lookup_image); null when not recorded"
    )
    marked: list[str] = Field(description="Labels of the marks drawn on the frame (tool is the FK tool point)")
    note: str | None = Field(
        default=None, description="Why a mark or the head aim is missing; null when the photo is complete"
    )


def marked_frame_jpeg(
    ctx: ToolContext, tool: str, marks: list[tuple[str, tuple[float, float, float]]]
) -> tuple[bytes, int, int, list[str], str | None]:
    """A fresh head frame (head aimed at the FK tool point) marked with the tool point and extra arm-frame points.

    Args:
        ctx (ToolContext): Shared registration context (robot, config, control).
        tool (str): Calling tool (named in a lease refusal of the head aim).
        marks (list[tuple[str, tuple[float, float, float]]]): (label, arm-frame point) circles to draw.

    Returns:
        tuple[bytes, int, int, list[str], str | None]: (JPEG, width, height, labels drawn, note). Without a camera
            calibration the frame is plain with no marks; a mark behind the camera or outside the image is skipped
            and named in the note.

    Raises:
        RobotError: When the camera gives no frame.
        ValueError: When the frame cannot be decoded.
    """
    robot, config = ctx.robot, ctx.config
    max_px = min(config.grasp_check.image_px, config.limits.max_image_px)
    sample = robot.arm.fresh_sample()
    joints = None if sample is None else robot.arm.measured(sample)
    notes: list[str] = []
    aim_problem, _ = aim_head_at_tool_point(ctx, tool, joints)
    if aim_problem is not None:
        notes.append(aim_problem)
    try:
        setup = camera_setup(config, GRASP_CAMERA, source=robot)
        calibrated: tuple[int, int] | None = (setup.intrinsics.width, setup.intrinsics.height)
    except ValueError as exc:  # not calibrated: no projection, the plain frame is the image
        setup, calibrated = None, None
        notes.append(str(exc))
    request_px = max_px if calibrated is None else min(HARD_MAX_IMAGE_PX, max(calibrated))
    frame = robot.camera_image(GRASP_CAMERA, request_px)
    img = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("the head camera frame is not a decodable image")
    marked: list[str] = []
    if setup is not None and calibrated is not None and joints is not None:
        try:
            scene = build_scene(setup, config, head_parent_transform(config, robot.head))
            kin = robot.arm.kin
            tool_point = jaw_points(
                kin, robot.arm.jaw, kin.link_frames(joints), joints.get("gripper", robot.arm.jaw.closed_rad)
            )["tool_point"]
            points = [(MARK_TOOL, tuple(float(v) for v in tool_point), "cross", TOOL_COLOR)] + [
                (label, point, "circle", TARGET_COLOR) for label, point in marks
            ]
            refs = [scene.arm_to_ref(np.array(point)) for _, point, _, _ in points]
            if any(ref is None for ref in refs):
                raise ValueError("arm.base_in_base_link is not configured: no mark can be placed")
            pixels, depths = camera_overlay.project_points(scene.intr, scene.t_ref_optical, np.array(refs))
            sx, sy = frame.width / calibrated[0], frame.height / calibrated[1]
            for (label, _, shape, color), (u, v), depth in zip(points, pixels, depths, strict=True):
                if depth <= camera_overlay.MIN_DEPTH or not (0 <= u < calibrated[0] and 0 <= v < calibrated[1]):
                    notes.append(f"{label} is outside the image (not marked)")
                    continue
                camera_overlay.draw_marker(img, (u * sx, v * sy), color, shape, label)
                marked.append(label)
        except ValueError as exc:  # stale head joint states for the scene, or no arm offset
            notes.append(str(exc))
    elif joints is None:
        notes.append("no fresh arm joint states to place the marks")
    ok, encoded = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        raise ValueError("the marked head frame could not be encoded")
    jpeg = encoded.tobytes()
    if max(frame.width, frame.height) > max_px:
        jpeg = crop_jpeg(jpeg, (0, 0, frame.width, frame.height), max_px)
    width, height = jpeg_size(jpeg)
    return jpeg, width, height, marked, "; ".join(notes) or None


def flow_photo(
    ctx: ToolContext, tool: str, marks: list[tuple[str, tuple[float, float, float]]]
) -> tuple[ImageContent | None, FlowPhoto | None, str | None]:
    """The marked head photo of a flow tool, recorded as a lookup.

    Args:
        ctx (ToolContext): Shared registration context.
        tool (str): Calling tool (the lookup's tool name).
        marks (list[tuple[str, tuple[float, float, float]]]): (label, arm-frame point) circles to draw.

    Returns:
        tuple[ImageContent | None, FlowPhoto | None, str | None]: (image, photo info, warning). A camera failure
            gives (None, None, "no head camera photo: <reason>").
    """
    try:
        jpeg, width, height, marked, note = marked_frame_jpeg(ctx, tool, marks)
    except (RobotError, ValueError) as exc:
        return None, None, f"no head camera photo: {exc}"
    lookup_id = record_lookup(ctx, tool, GRASP_CAMERA, jpeg, width, height, full_frame=True)
    image = ImageContent(type="image", data=base64.b64encode(jpeg).decode(), mime_type="image/jpeg")
    return image, FlowPhoto(kind="marked_frame", lookup_id=lookup_id, marked=marked, note=note), None


FlowOutcome = Literal["landed", "infeasible", "closed", "lifted", "dropped", "aborted", "interrupted", "stopped"]


class GraspFlowResult(BaseModel):
    """Result of a guided grasp tool (grasp_approach, grasp_close, grasp_lift, grasp_abort)."""

    tool: str = Field(description="The flow tool that returned this result")
    outcome: FlowOutcome = Field(
        description="landed (jaws open at the grasp point), infeasible (nothing moved), closed (jaws closed, not "
        "lifted), lifted (object lifted and held), dropped (lifted but the jaws ended empty), aborted (jaws opened "
        "and the arm rose; flow ended), interrupted (a critical event stopped the motion: only grasp_abort is "
        "allowed), stopped (a stop call ended the tool; the flow is idle)"
    )
    phase: FlowPhase = Field(description="Flow phase after this tool: idle, landed, closed, lifted or interrupted")
    next_steps: list[NextStep] = Field(description="What to call next and when; read it before choosing")
    reasons: list[str] = Field(default_factory=list, description="Why the plan is infeasible or the tool ended early")
    route: Literal["fresh", "straight", "via_pre_grasp"] | None = Field(
        default=None,
        description="grasp_approach only: fresh (full approach), straight (re-approach in a straight tool line, jaws "
        "open, grasp point moved by at most 3 cm) or via_pre_grasp (backed out to the pre-grasp, then approached again)",
    )
    plan: dict[str, Any] | None = Field(default=None, description="Plan summary: waypoints, pitch, roll, slow zone")
    steps: list[GraspStep] = Field(default_factory=list, description="Arm steps this tool ran, in order")
    landing: dict[str, Any] | None = Field(
        default=None,
        description="Where the jaws landed against the plan: {waypoint, expected_jaw_centre, jaw_centre, tool_point, "
        "error_m, planned_tool_point, tool_point_error_m}; null without fresh joint states",
    )
    gripper_position_rad: float | None = Field(default=None, description="Gripper joint position at the end (rad)")
    gripper_effort: float | None = Field(default=None, description="Gripper effort (decoded load) at the end")
    grip_profile: dict[str, Any] | None = Field(default=None, description="Grip profile of the close, when one ran")
    holding_load: float | None = Field(default=None, description="Gripper load a moment after the hold")
    slipping: bool | None = Field(default=None, description="Jaw kept closing after the hold: the object slips")
    crush_risk: bool | None = Field(default=None, description="Holding load above the profile's crush_load")
    retightened: bool | None = Field(default=None, description="The hold was stepped once further toward closed")
    verify_problem: str | None = Field(
        default=None, description="grasp_close: why the sensors do not confirm a hold (null when they do)"
    )
    grasp_check: GraspCheck | None = Field(default=None, description="grasp_close: the sensor verdict of the close")
    fused: FusedOutcome | None = Field(default=None, description="grasp_lift: the sensor band fused with your verdict")
    interrupted_by: str | None = Field(default=None, description="Critical event type that interrupted the motion")
    photos: list[FlowPhoto] = Field(default_factory=list, description="The photos attached to this result, in order")
    warnings: list[str] = Field(default_factory=list, description="Off-target joints, a missing photo or head aim")


def flow_result(
    executor: GraspExecutor,
    tool: str,
    outcome: FlowOutcome,
    phase: FlowPhase,
    holding: bool,
    steps: list[GraspStep],
    close: ArmMotionResult | None = None,
    **fields: Any,
) -> GraspFlowResult:
    """A GraspFlowResult with the gripper state, the close's grip report, next steps and the step warnings filled in.

    Args:
        executor (GraspExecutor): Executor (fresh sample, step warnings).
        tool (str): The flow tool.
        outcome (FlowOutcome): Outcome.
        phase (FlowPhase): Flow phase after the tool.
        holding (bool): The jaws carry the object (picks the lifted next steps).
        steps (list[GraspStep]): Steps the tool ran.
        close (ArmMotionResult | None): Close result whose grip report goes into the result.
        **fields (Any): Further GraspFlowResult fields; warnings given here come first.

    Returns:
        GraspFlowResult: The result.
    """
    sample = executor.arm.fresh_sample()
    gripper = executor.arm.gripper
    warnings = [*fields.pop("warnings", []), *executor.step_warnings(steps)]
    return GraspFlowResult(
        tool=tool,
        outcome=outcome,
        phase=phase,
        next_steps=next_steps(phase, holding),
        steps=steps,
        gripper_position_rad=None if sample is None else sample.positions.get(gripper),
        gripper_effort=None if sample is None else finite_effort(sample.efforts, gripper),
        grip_profile=None if close is None else close.grip_profile,
        holding_load=None if close is None else close.holding_load,
        slipping=None if close is None else close.slipping,
        crush_risk=None if close is None else close.crush_risk,
        retightened=None if close is None else close.retightened,
        warnings=warnings,
        **fields,
    )


def grasp_waypoint(plan: GraspPlan, label: str = "grasp") -> tuple[float, float, float]:
    """Arm-frame position of a plan waypoint.

    Args:
        plan (GraspPlan): Feasible plan.
        label (str): Waypoint label.

    Returns:
        tuple[float, float, float]: (x, y, z).
    """
    wp = next(w for w in plan.waypoints if w.label == label)
    return wp.x, wp.y, wp.z


def register(ctx: ToolContext) -> None:
    """Register the guided grasp tools.

    Args:
        ctx (ToolContext): Shared registration context.
    """
    robot, config = ctx.robot, ctx.config
    arm = robot.arm
    executor = GraspExecutor(arm, config, lambda: robot.head.calibration())
    flow = GraspFlow()
    object_desc = object_description(config)
    strategy_desc = strategy_description(config)
    params_desc = params_description(config)
    surface_desc = surface_description(config)
    grip_desc = grip_profile_description(config)
    slow_note = slow_description(config)

    FlowBody = Callable[[FlowPhase, Callable[[], bool], RunLog], tuple[GraspFlowResult, list[ImageContent]]]

    def run(tool: str, body: FlowBody) -> CallToolResult:
        """Common wrapper: one flow tool at a time, battery gate, phase gate, stop and interrupt handling."""
        if not flow.busy.acquire(blocking=False):
            raise ToolError(f"{tool} refused: another grasp flow tool is running")
        try:
            ctx.battery_gate(tool)
            phase = flow.require(arm, tool)
            baseline = robot.stop_count()
            log = RunLog(steps=[])
            try:
                result, images = body(phase, lambda: robot.stop_count() != baseline, log)
            except GraspAbort as exc:
                executor.halt()
                interrupted_by = next((s.interrupted_by for s in reversed(log.steps) if s.interrupted_by), None)
                if robot.stop_count() != baseline:
                    flow.reset()
                    result = flow_result(executor, tool, "stopped", "idle", False, log.steps, reasons=[str(exc)])
                else:
                    flow.advance(arm, "interrupted", interrupted_by=interrupted_by)
                    result = flow_result(
                        executor,
                        tool,
                        "interrupted",
                        "interrupted",
                        arm.grasp_held,
                        log.steps,
                        reasons=[str(exc)],
                        interrupted_by=interrupted_by,
                    )
                images = []
            except (ArmError, ValueError) as exc:
                raise ToolError(str(exc)) from exc
            GRASP_FLOW_STEPS.labels(tool, result.outcome).inc()
            return grasp_tool_result(result, images)
        finally:
            flow.busy.release()

    def attach_photo(
        tool: str, result: GraspFlowResult, marks: list[tuple[str, tuple[float, float, float]]]
    ) -> list[ImageContent]:
        """Take the marked head photo and record it in the result."""
        image, photo, warning = flow_photo(ctx, tool, marks)
        if warning is not None:
            result.warnings.append(warning)
        if photo is not None and image is not None:
            result.photos.append(photo)
            if photo.note:
                result.warnings.append(photo.note)
            return [image]
        return []

    @ctx.server.tool(
        description=(
            "Guided grasp, step 1 (needs the arm lease, take_control(subsystem='arm')): plan the grasp of the object "
            "(same object, strategy, params and surfaces as plan_grasp) and move the arm so the jaws stand OPEN around "
            "the grasp point, then stop. It does not close. A camera-sourced object position is gated like the macro "
            "grasp (the accepted arm to camera calibration residual) before anything moves. The head aims at the "
            "grasp point before the arm moves and at the FK tool point for the photo afterwards (no tracking); the "
            "result carries that head photo with the tool point and the target marked, the landing error against the "
            "plan and next_steps: look at the photo, then call grasp_close when the object is between the open jaws, "
            "call grasp_approach again with a corrected object position when the jaws are off target, or grasp_abort. "
            "A re-approach whose grasp point moves by at most 3 cm (same strategy and angles) goes in a straight tool "
            "line with the jaws open; a larger one backs out to the pre-grasp and slides in again (route in the "
            "result). go_back is optional and you decide: the default false keeps the jaws at the object and nudges "
            "in a straight line for corrections of 3 cm or less; set it true to back out to the pre-grasp first "
            "(use it when the jaws might drag the object or the angle changes); corrections over 3 cm always go "
            "via the pre-grasp. go_back is ignored for a fresh approach. Called after a lift it approaches again only when the jaws are empty; "
            "while holding it is refused and names release_object. It is not approach_for_grasp (which drives the base). "
            f"{slow_note}"
        )
    )
    def grasp_approach(
        object: Annotated[ObjectSpec, Field(description=object_desc)],
        strategy: Annotated[StrategyChoice, Field(description=strategy_desc)] = "auto",
        params: Annotated[dict[str, Any] | None, Field(description=params_desc)] = None,
        approach_pitch_deg: Annotated[float | None, Field(ge=0.0, le=90.0, description=PITCH_DESC)] = None,
        surface_z_m: Annotated[float | None, Field(ge=-1.0, le=1.0, description=surface_desc)] = None,
        tilt_override_deg: Annotated[TiltOverrideDeg | None, Field(description=TILT_DESC)] = None,
        use_ground_estimate: Annotated[bool, Field(description=USE_GROUND_DESC)] = False,
        grip_profile: Annotated[str | GripProfileOverride | None, Field(description=grip_desc)] = None,
        surfaces: Annotated[
            list[SurfaceRegion] | None, Field(max_length=MAX_REGIONS, description=SURFACES_DESC)
        ] = None,
        go_back: Annotated[bool, Field(description=GO_BACK_DESC)] = False,
    ) -> Annotated[CallToolResult, GraspFlowResult]:
        """Land the open jaws at the grasp point; the tool description is passed to the decorator."""
        tool = "grasp_approach"

        def body(phase: FlowPhase, stop: Callable[[], bool], log: RunLog) -> tuple[GraspFlowResult, list[ImageContent]]:
            if phase == "idle" and arm.grasp_held:
                raise ToolError(
                    f"{tool} refused (nothing moved): the jaws carry an object; put it down with {RELEASE_TOOL} first"
                )
            old_plan, old_params, old_floor = flow.plan, flow.params, flow.floor
            if (
                phase == "lifted" and arm.grasp_held
            ):  # the same record grasp_lift left: lifted keeps it, dropped clears it
                raise ToolError(
                    f"{tool} refused (nothing moved): the jaws still hold the object; put it down with "
                    f"{RELEASE_TOOL} first"
                )
            floor = floor_override(surface_z_m, tilt_override_deg, surfaces, use_ground_estimate)
            call_params = grasp_params(config.grasp, params, grip_profile)
            plan, notes, grip, floor = executor.prepare(object, strategy, call_params, approach_pitch_deg, floor)
            if not plan.feasible:
                result = flow_result(
                    executor, tool, "infeasible", phase, arm.grasp_held, [], reasons=plan.reasons, warnings=notes
                )
                return result, []
            warnings = list(notes)
            if phase == "lifted":
                arm.note_grasp(None)  # the jaws are empty: the object dropped
            aim, _ = aim_head_at_arm_point(ctx, tool, grasp_waypoint(plan), "the grasp point")
            if aim is not None:
                warnings.append(aim)
            route: Literal["fresh", "straight", "via_pre_grasp"] = "fresh"
            if phase in ("landed", "closed") and old_plan is not None:
                prior = (old_params or call_params, old_floor)
                if phase == "closed":
                    log.reached = next(w for w in old_plan.waypoints if w.label == "grasp")
                    executor.open_jaws(old_plan, prior[1], stop, log)
                    GRASP_ATTEMPTS.labels(old_plan.strategy, "missed").inc()
                route = "via_pre_grasp" if go_back else reapproach_route(old_plan, plan)
                path: list[dict[str, float]] = []
                if route == "straight":
                    path, why = executor.correction_path(old_plan, plan, call_params, floor)
                    if why is not None:
                        warnings.append(f"straight correction not possible ({why}): backed out to the pre-grasp")
                        route = "via_pre_grasp"
                if route == "straight":
                    assert plan.open_gripper_rad is not None
                    frac = executor.fraction(plan.open_gripper_rad)
                    executor.step("open", log.steps, stop, lambda: arm.set_gripper(open_fraction=frac, floor=floor))
                    arm_path = arm_only(path, arm.gripper)
                    executor.step(
                        "correct",
                        log.steps,
                        stop,
                        lambda: arm.move_path(arm_path, call_params.slide_speed_scale, floor),
                        intent=arm_path[-1],
                    )
                    log.reached = next(w for w in plan.waypoints if w.label == "grasp")
                else:
                    executor.back_out(old_plan, prior[0], prior[1], stop, log)
                    executor.land(plan, call_params, floor, stop, log)
            else:
                executor.land(plan, call_params, floor, stop, log)
            flow.advance(
                arm,
                "landed",
                object=object,
                strategy=strategy,
                params=call_params,
                approach_pitch_deg=approach_pitch_deg,
                floor=floor,
                plan=plan,
                grip=grip,
                close=None,
                grasp_id=None,
                interrupted_by=None,
            )
            result = flow_result(
                executor,
                tool,
                "landed",
                "landed",
                False,
                log.steps,
                reasons=[],
                route=route,
                plan=plan.summary(),
                landing=executor.landing_error(plan),
                warnings=warnings,
            )
            return result, attach_photo(tool, result, [(MARK_TARGET, grasp_waypoint(plan))])

        return run(tool, body)

    @ctx.server.tool(
        description=(
            "Guided grasp, step 2 (needs the arm lease; only after grasp_approach landed): close the jaws on the "
            "object with the flow's grip profile (or the grip_profile given here) until the gripper feels it, verify "
            "the sensors (jaw stopped short of closed under load) and stop WITHOUT lifting. Never opens: a failed "
            "verification is reported in verify_problem with the jaws still closed, and next_steps tell you to reapproach "
            "(the jaws reopen first), lift or abort. The result carries grasp_check (the sensor band: object, empty or "
            "ambiguous, with the evidence) and two images: the jaw crop first (the head aimed at the tool point), the "
            "marked head frame second. Look at them: is the object between the closed fingers? Then call grasp_lift "
            "with your verdict. The arm does not move, so the head is not aimed before the close. A foreign arm motion "
            "since grasp_approach (set_gripper, move_arm_*, arm_home, a queued step, the web UI, a stop or a lease "
            "change) ends the flow and this tool is refused. Sensor bands: object (contact, load and jaw gap say an "
            "object is held), empty (jaw in the empty band without load: a squashed soft object or nothing), "
            "ambiguous (anything else); the image never proves a grasp on its own."
        )
    )
    def grasp_close(
        grip_profile: Annotated[str | GripProfileOverride | None, Field(description=grip_desc)] = None,
    ) -> Annotated[CallToolResult, GraspFlowResult]:
        """Close the jaws on the landed object; the tool description is passed to the decorator."""
        tool = "grasp_close"

        def body(phase: FlowPhase, stop: Callable[[], bool], log: RunLog) -> tuple[GraspFlowResult, list[ImageContent]]:
            plan, base = flow.plan, flow.params
            assert plan is not None and base is not None  # the landed phase stores both
            params = base if grip_profile is None else grasp_params(config.grasp, base.model_dump(), grip_profile)
            executor.resolve_grip(params)  # an invalid profile refuses before anything moves
            verdict = executor.close_and_verify(plan, params, flow.floor, stop, log)
            check = verdict.close.grasp_check
            flow.advance(
                arm, "closed", close=verdict.close, grasp_id=None if check is None else check.grasp_id, params=params
            )
            images: list[ImageContent] = []
            photos: list[FlowPhoto] = []
            warnings: list[str] = []
            if arm.grasp_held and check is not None:
                crop_image, check, warning = attach_jaw_crop(ctx, tool, check)
                if warning is not None:
                    warnings.append(warning)
                if crop_image is not None:
                    images.append(crop_image)
                    crop = check.crop
                    photos.append(
                        FlowPhoto(
                            kind="jaw_crop",
                            lookup_id=None if crop is None else crop.lookup_id,
                            marked=[],
                            note=None if crop is None else crop.full_frame_reason,
                        )
                    )
            result = flow_result(
                executor,
                tool,
                "closed",
                "closed",
                arm.grasp_held,
                log.steps,
                close=verdict.close,
                grasp_check=check,
                verify_problem=verdict.missed or verdict.unverified,
                photos=photos,
                warnings=warnings,
            )
            return result, [*images, *attach_photo(tool, result, [])]

        return run(tool, body)

    @ctx.server.tool(
        description=(
            "Guided grasp, step 3 (needs the arm lease; only after grasp_close): record your visual verdict on the "
            "jaw crop and, when it allows, lift the object and retreat. verdict yes = the object is clearly between "
            "the closed fingers; unsure = you cannot tell. The verdict is fused with the sensor band (grasp_check.band) "
            "and always recorded and counted. Lift rule: band object + yes lifts (fused grasped); band ambiguous + yes "
            "also lifts (fused unverified: after the lift the jaws are re-checked and the result says lifted, the "
            "carry stays held, or dropped); an empty band (any verdict) and unsure (any band) refuse the lift with "
            "nothing moved and the flow stays closed: grasp_approach again (the jaws reopen first) or grasp_abort. The "
            "head aims at the lift point before the lift and the result carries a head photo after it. Outcome lifted "
            "(held: next release_object at the place location), dropped (the jaws ended empty: grasp_approach or "
            f"grasp_abort). The result carries fused. {slow_note}"
        )
    )
    def grasp_lift(
        verdict: Annotated[
            Literal["yes", "unsure"],
            Field(description="Is the object between the closed fingers in the jaw crop: yes, or unsure"),
        ],
        evidence: Annotated[
            str, Field(min_length=1, max_length=500, description="What you saw in the crop that supports the verdict")
        ],
        surface_z_m: Annotated[float | None, Field(ge=-1.0, le=1.0, description=surface_desc)] = None,
        tilt_override_deg: Annotated[TiltOverrideDeg | None, Field(description=TILT_DESC)] = None,
        use_ground_estimate: Annotated[bool, Field(description=USE_GROUND_DESC)] = False,
        surfaces: Annotated[
            list[SurfaceRegion] | None, Field(max_length=MAX_REGIONS, description=SURFACES_DESC)
        ] = None,
    ) -> Annotated[CallToolResult, GraspFlowResult]:
        """Record the verdict, then lift and retreat when allowed; the tool description is passed to the decorator."""
        tool = "grasp_lift"

        def body(phase: FlowPhase, stop: Callable[[], bool], log: RunLog) -> tuple[GraspFlowResult, list[ImageContent]]:
            plan, params = flow.plan, flow.params
            assert plan is not None and params is not None  # the closed phase stores both
            if flow.grasp_id is None:
                raise ToolError(f"{tool} refused (nothing moved): the close left no grasp check to record a verdict on")
            record = arm.record_visual_verdict(flow.grasp_id, verdict)
            band, fused = record.band, record.fused
            assert band is not None and fused is not None  # a recorded verdict sets both
            LOGGER.info("grasp_lift %s: band %s + %s -> %s (%s)", flow.grasp_id, band, verdict, fused, evidence)
            GRASP_VERDICTS.labels(band, verdict, fused).inc()
            if verdict != "yes" or band == "empty":
                raise ToolError(
                    f"{tool} refused (nothing moved): sensor band {band} with your verdict {verdict} fuses to "
                    f"{fused}; the lift needs verdict yes with band object or ambiguous (an empty band or unsure "
                    "never lifts). The verdict is recorded and the flow stays closed. "
                    + "; ".join(f"{step.tool}: {step.when}" for step in next_steps("closed", False))
                )
            floor = floor_override(surface_z_m, tilt_override_deg, surfaces, use_ground_estimate) or flow.floor
            warnings: list[str] = []
            aim, _ = aim_head_at_arm_point(ctx, tool, grasp_waypoint(plan, "lift"), "the lift point")
            if aim is not None:
                warnings.append(aim)
            executor.lift_and_retreat(plan, floor, stop, log)
            # a soft object (ambiguous band) may sit near closed: only its load counts
            problem = executor.hold_problem(params, check_gap=band != "ambiguous")
            reasons: list[str] = []
            if problem is None:
                outcome: FlowOutcome = "lifted"
                GRASP_ATTEMPTS.labels(plan.strategy, "lifted").inc()
            else:
                outcome = "dropped"
                arm.note_grasp(None)
                GRASP_ATTEMPTS.labels(plan.strategy, "missed").inc()
                reasons.append(problem)
            flow.advance(arm, "lifted", floor=floor)
            result = flow_result(
                executor,
                tool,
                outcome,
                "lifted",
                outcome == "lifted",
                log.steps,
                close=flow.close,
                reasons=reasons,
                fused=fused,
                grasp_check=arm.last_grasp_check,
                warnings=warnings,
            )
            return result, attach_photo(tool, result, [])

        return run(tool, body)

    @ctx.server.tool(
        description=(
            "Guided grasp, abort (needs the arm lease; allowed in every phase): open the jaws, rise straight up by "
            "params.pre_grasp_clearance_m, end the grasp flow and return a head photo. Use it to give up a grasp or to "
            "recover from an interrupted flow. Refused (nothing moved) while the jaws hold a lifted object or when no "
            f"flow ran and the jaws carry something: call {RELEASE_TOOL} for that. An unreachable rise is reported in "
            f"reasons, not an error. {slow_note}"
        )
    )
    def grasp_abort(
        surface_z_m: Annotated[float | None, Field(ge=-1.0, le=1.0, description=surface_desc)] = None,
        tilt_override_deg: Annotated[TiltOverrideDeg | None, Field(description=TILT_DESC)] = None,
        use_ground_estimate: Annotated[bool, Field(description=USE_GROUND_DESC)] = False,
        surfaces: Annotated[
            list[SurfaceRegion] | None, Field(max_length=MAX_REGIONS, description=SURFACES_DESC)
        ] = None,
    ) -> Annotated[CallToolResult, GraspFlowResult]:
        """Open the jaws, rise and end the flow; the tool description is passed to the decorator."""
        tool = "grasp_abort"

        def body(phase: FlowPhase, stop: Callable[[], bool], log: RunLog) -> tuple[GraspFlowResult, list[ImageContent]]:
            if arm.grasp_held and phase in ("idle", "lifted", "interrupted"):
                raise ToolError(
                    f"{tool} refused (nothing moved): the jaws hold an object; put it down with {RELEASE_TOOL}"
                )
            plan, params = flow.plan, flow.params or config.grasp
            floor = floor_override(surface_z_m, tilt_override_deg, surfaces, use_ground_estimate) or flow.floor
            rise = executor.open_and_rise(
                1.0 if plan is None or plan.open_gripper_rad is None else executor.fraction(plan.open_gripper_rad),
                params.pre_grasp_clearance_m,
                params.slide_speed_scale,
                floor,
                stop,
                log,
            )
            if phase == "closed" and plan is not None:
                GRASP_ATTEMPTS.labels(plan.strategy, "aborted").inc()
            flow.reset()
            reasons = [] if rise.status == "converged" else [f"the jaws opened; the rise was skipped: {rise.message}"]
            result = flow_result(executor, tool, "aborted", "idle", False, log.steps, reasons=reasons)
            return result, attach_photo(tool, result, [])

        return run(tool, body)
