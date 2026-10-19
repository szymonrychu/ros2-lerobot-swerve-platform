"""MCP tools suggest_approach_pose and approach_for_grasp: where to park the base for a grasp, and driving there.

suggest_approach_pose is read-only (no lease, battery and tilt independent): the inverse reachability map
(reach_map.py) gives candidate base poses, approach.py scores them on the global costmap with the current NavFootprint,
the best are verified with point_reach (the grasp planner's IK and body check) and path-planned (ComputePathToPose,
planning only). approach_for_grasp drives to the best candidate: servoed on the grasp docking server (DockRobot with the
detection loop feeding it real camera detections) when a calibrated camera sees the target from the staging and the
dock pose, else as a precise Nav2 goal. The base motion lock, monitor.base_motion and the BASE_INTERRUPTS watch live in
RobotApi.dock / RobotApi.navigate (the same code path as navigate_to_pose). After a successful arrival the target is
re-checked with point_reach at the measured pose: out of reach is never reported as success.
"""

import logging
import math
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from typing import Annotated, Any, Literal

import cv2
import numpy as np
from mcp.server.mcpserver.exceptions import ToolError
from pydantic import BaseModel, ConfigDict, Field

from .approach import (
    ApproachSuggestion,
    ApproachTarget,
    ApproachUnavailable,
    Candidate,
    PixelTarget,
    TargetPoint,
    XyzTarget,
    resolve_target,
    suggest,
    target_in_base,
    wrap_angle,
)
from .arm import ArmError
from .arm_reach import DEFAULT_SEED, ReachEnvelope, point_reach
from .base_dock import DockGoal, DockResult
from .camera_scene import (
    CameraNotCalibratedError,
    CameraSetup,
    Scene,
    base_to_map,
    build_scene,
    camera_setup,
    parent_transform,
)
from .config import HARD_MAX_IMAGE_PX, McpServerConfig
from .models import BasePose, NavigationResult, RobotError
from .object_memory import OBJECT_KIND
from .reach_map import ReachMap, ReachMapMissing, ReachMapStale, pan_geometry
from .self_collision import SelfCollisionChecker
from .target_tracker import Detection, DetectionLoop, TargetTracker, TrackerInput, visible
from .tool_context import RobotApi, ToolContext

LOGGER = logging.getLogger(__name__)
TOOL_NAMES = ("suggest_approach_pose", "approach_for_grasp")
SENSOR_TOOL_NAMES = ("suggest_approach_pose",)
GRASP_DOCK_TYPE = "grasp_servoed"
CAMERAS = ("front", "gripper")
POI_KINDS = ("point", "area", "marker")
ALL_CANDIDATES = 100_000
# A given pose counts as the same candidate within this distance / heading difference.
POSE_TOL_M = 0.02
POSE_TOL_RAD = 0.05
MAX_RESULTS_LIMIT = 10
Mode = Literal["servoed", "navigate"]
PreferMode = Literal["auto", "servoed", "navigate"]
ApproachStatus = Literal[
    "planned", "succeeded", "arrived_out_of_reach", "failed", "rejected", "interrupted", "canceled", "timeout"
]
NAV_STATUS = {
    "succeeded": "succeeded",
    "canceled": "canceled",
    "rejected": "rejected",
    "interrupted": "interrupted",
    "timeout": "timeout",
}
TARGET_HELP = (
    "Target (exactly one form): a map point x, y, z (z = height above the floor, m); object_id (a remembered object) "
    "or poi_id (a point, area or marker POI) plus z_m, the height of the grasp point above the floor (stored objects "
    "and point/area POIs have no height; a marker POI carries its own z, so z_m may be omitted for it and overrides it); or a camera pixel camera, u, v (pixels of the CALIBRATED image) plus surface_height_m, the height "
    "of the surface the pixel lies on."
)


class ApproachPose(BaseModel):
    """A base pose in the map frame (one of suggest_approach_pose's candidates)."""

    model_config = ConfigDict(extra="forbid")

    x: float = Field(description="m")
    y: float = Field(description="m")
    yaw: float = Field(description="rad, CCW from +x")


class ApproachResult(BaseModel):
    """approach_for_grasp result (also the dry run: status 'planned')."""

    status: ApproachStatus = Field(
        description="planned (dry_run), succeeded, arrived_out_of_reach (arrived, but the target is not reachable from "
        "the measured pose), failed, rejected, interrupted, canceled or timeout"
    )
    dry_run: bool = False
    mode: Mode
    mode_reason: str
    message: str = ""
    target: TargetPoint
    plan: Candidate = Field(description="The chosen candidate: dock pose, staging pose, reach, costmap, path")
    goal: BasePose = Field(description="The dock pose sent (or, in a dry run, that would be sent)")
    would_send: dict[str, Any] | None = Field(default=None, description="dry_run only: the goal that would be sent")
    docking: dict[str, Any] = Field(description="Docking server readiness {server_ready, action}")
    tracker: dict[str, Any] | None = Field(default=None, description="dry_run, servoed only: one tracker step")
    error_code: int | None = Field(default=None, description="DockRobot error code (90x) of a failed servoed dock")
    error_name: str | None = None
    num_retries: int | None = None
    interrupted_by: str | None = None
    final_pose: BasePose | None = None
    reach_after: dict[str, Any] | None = Field(
        default=None, description="point_reach of the target from the measured pose after an arrival"
    )
    detections_published: int = Field(default=0, description="Detections the tracker published during a servoed dock")
    detections_rejected: int = Field(
        default=0, description="Tracker matches skipped for jumping beyond max_correction_m during a servoed dock"
    )
    duration_s: float = 0.0


@contextmanager
def approach_errors() -> Iterator[None]:
    """Turn planner, camera, arm and robot failures into MCP tool errors the model can read.

    Yields:
        None: Context body.
    """
    try:
        yield
    except (ApproachUnavailable, ArmError, RobotError, ReachMapMissing, ReachMapStale, ValueError) as exc:
        raise ToolError(str(exc)) from exc


def load_reach_map(config: McpServerConfig, checker: SelfCollisionChecker) -> ReachMap:
    """Load the committed reach map and require it to match the current body config.

    Args:
        config (McpServerConfig): Node configuration (reach_map settings).
        checker (SelfCollisionChecker): The arm's body checker.

    Returns:
        ReachMap: The map.

    Raises:
        ReachMapMissing: When the file is absent or unreadable.
        ReachMapStale: When it was built for other inputs.
    """
    return ReachMap.load(config.reach_map.file, checker, config.reach_map)


def footprint_points(ctx: ToolContext) -> list[tuple[float, float]]:
    """The footprint Nav2 uses now (base_link polygon), so a released arm or carried object counts.

    Args:
        ctx (ToolContext): Registration context (footprint).

    Returns:
        list[tuple[float, float]]: Polygon vertices (m).

    Raises:
        RobotError: When no nav_footprint is configured.
    """
    if ctx.footprint is None:
        raise RobotError("nav_footprint is not configured: the approach planner needs the robot footprint")
    return list(ctx.footprint.current.points)


def build_target(
    x: float | None,
    y: float | None,
    z: float | None,
    object_id: str | None,
    poi_id: str | None,
    z_m: float | None,
    camera: str | None,
    u: float | None,
    v: float | None,
    surface_height_m: float | None,
) -> ApproachTarget:
    """Assemble the target forms of the tool arguments.

    Args:
        x (float | None): Point x (map, m).
        y (float | None): Point y (map, m).
        z (float | None): Point height above the floor (m).
        object_id (str | None): Remembered object id.
        poi_id (str | None): POI id.
        z_m (float | None): Height for object_id / poi_id (m).
        camera (str | None): Camera of a pixel target.
        u (float | None): Pixel x.
        v (float | None): Pixel y.
        surface_height_m (float | None): Surface height of a pixel target (m).

    Returns:
        ApproachTarget: The validated target.

    Raises:
        ValueError: For a partial point or pixel, or when no or several forms are given.
    """
    point = (x, y, z)
    pixel = (camera, u, v)
    if any(p is not None for p in point) and any(p is None for p in point):
        raise ValueError("a point target needs x, y and z")
    if any(p is not None for p in pixel) and any(p is None for p in pixel):
        raise ValueError("a pixel target needs camera, u and v")
    if surface_height_m is not None and camera is None:
        raise ValueError("surface_height_m is only for a pixel target")
    return ApproachTarget(
        xyz=None if x is None or y is None or z is None else XyzTarget(x=x, y=y, z=z),
        object_id=object_id,
        poi_id=poi_id,
        pixel=None
        if camera is None or u is None or v is None
        else PixelTarget(camera=camera, u=u, v=v, surface_height_m=surface_height_m or 0.0),
        z_m=z_m,
    )


def poi_lookup(robot: RobotApi, kinds: tuple[str, ...]) -> Callable[[str], tuple[float, float] | None]:
    """Lookup of a POI's map position (an area's x, y is its centroid) by id.

    Args:
        robot (RobotApi): Robot (poi_list).
        kinds (tuple[str, ...]): POI kinds that count.

    Returns:
        Callable[[str], tuple[float, float] | None]: id -> (x, y), None when unknown.
    """

    def lookup(poi_id: str) -> tuple[float, float] | None:
        for poi in robot.poi_list()[0]:
            if str(poi.get("id")) == poi_id and poi.get("kind") in kinds:
                return float(poi["x"]), float(poi["y"])
        return None

    return lookup


def poi_z_lookup(robot: RobotApi, kinds: tuple[str, ...]) -> Callable[[str], float | None]:
    """Lookup of a marker POI's own height by id.

    Args:
        robot (RobotApi): Robot (poi_list).
        kinds (tuple[str, ...]): POI kinds that count.

    Returns:
        Callable[[str], float | None]: id -> z (m), None for an unknown POI or one without a height.
    """

    def lookup(poi_id: str) -> float | None:
        for poi in robot.poi_list()[0]:
            if str(poi.get("id")) == poi_id and poi.get("kind") == "marker" and poi.get("kind") in kinds:
                z = poi.get("z")
                return float(z) if isinstance(z, int | float) and not isinstance(z, bool) else None
        return None

    return lookup


def camera_scene(robot: RobotApi, config: McpServerConfig, camera: str) -> tuple[Scene, CameraSetup]:
    """Scene of a calibrated camera at the current arm pose and the current measured head pose.

    The front scene uses the head angles measured now; the view checks built on it assume the head keeps that pose
    while the base moves to the staging and dock poses.

    Args:
        robot (RobotApi): Robot (arm kinematics and measured joints).
        config (McpServerConfig): Node configuration.
        camera (str): 'front' or 'gripper'.

    Returns:
        tuple[Scene, CameraSetup]: The scene and the camera's setup.

    Raises:
        CameraNotCalibratedError: When intrinsics or mount are missing.
        ArmError: When the gripper camera needs joint states and they are stale.
    """
    setup = camera_setup(config, camera)
    joints = robot.arm.measured(robot.arm.require_sample()) if camera == "gripper" else None
    return build_scene(setup, config, parent_transform(config, camera, robot.arm.kin, joints, head=robot.head)), setup


def grab_frame(robot: RobotApi, setup: CameraSetup) -> tuple[np.ndarray, int]:
    """A fresh frame at the calibrated image size and its capture stamp.

    Args:
        robot (RobotApi): Robot (camera_image).
        setup (CameraSetup): The camera's setup.

    Returns:
        tuple[np.ndarray, int]: BGR image and the capture stamp (ns since the epoch).

    Raises:
        RobotError: When the frame cannot be read or decoded.
    """
    intr = setup.intrinsics
    frame = robot.camera_image(setup.name, min(HARD_MAX_IMAGE_PX, max(intr.width, intr.height)))
    img = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise RobotError(f"cannot decode the {setup.name} camera frame")
    if img.shape[1] != intr.width or img.shape[0] != intr.height:
        img = cv2.resize(img, (intr.width, intr.height), interpolation=cv2.INTER_AREA)
    return img, round(frame.stamp_s * 1e9)


def detection_summary(detection: Detection) -> dict[str, Any]:
    """JSON view of a tracker detection.

    Args:
        detection (Detection): The detection.

    Returns:
        dict[str, Any]: pose, stamp_ns, target_map, pixel, score.
    """
    return {
        "pose": detection.pose.model_dump(exclude={"age_s"}),
        "stamp_ns": detection.stamp_ns,
        "target_map": {"x": detection.target_map[0], "y": detection.target_map[1]},
        "pixel": {"u": detection.pixel[0], "v": detection.pixel[1]},
        "score": detection.score,
    }


def register(ctx: ToolContext) -> None:
    """Register suggest_approach_pose and approach_for_grasp.

    Loads the reach map now: a stale or missing file is logged as an error and both tools then refuse with the
    regenerate command (the rest of the server keeps working).

    Args:
        ctx (ToolContext): Shared registration context.
    """
    server, robot, config = ctx.server, ctx.robot, ctx.config
    settings = config.approach
    checker = robot.arm.body_check
    envelope = ReachEnvelope(checker, config.body_check, None)  # never started: only suggests for unreachable points
    reach_map: ReachMap | None = None
    reach_error: str | None = None
    try:
        reach_map = load_reach_map(config, checker)
    except (ReachMapMissing, ReachMapStale) as exc:
        reach_error = str(exc)
        LOGGER.error("approach tools disabled: reach map unusable: %s", reach_error)
    pan_axis: list[tuple[float, float]] = []
    nav_max = config.timeouts.nav_max_timeout_s

    def pan_axis_xy() -> tuple[float, float]:
        """Pan axis in base_link (computed on first use)."""
        if not pan_axis:
            axis = pan_geometry(checker).axis
            pan_axis.append((float(axis[0]), float(axis[1])))
        return pan_axis[0]

    def ik_seed() -> dict[str, float]:
        """IK seed: the measured arm pose, else the default."""
        sample = robot.arm.fresh_sample()
        return DEFAULT_SEED if sample is None else robot.arm.measured(sample)

    def reach_at(target: TargetPoint, pose: tuple[float, float, float], pitch_deg: float | None) -> dict[str, Any]:
        """point_reach of the map target seen from a base pose."""
        x, y, z = target_in_base(target, pose)
        pitch = None if pitch_deg is None else math.radians(pitch_deg)
        return point_reach(checker, {"x": x, "y": y, "z": z, "pitch": pitch}, ik_seed(), envelope)

    def resolve(target: ApproachTarget) -> TargetPoint:
        """Resolve a target to a map point (object / POI lookups, pixel through the calibrated camera)."""
        objects = poi_lookup(robot, (OBJECT_KIND,))

        def pixel_xy(camera: str, u: float, v: float, surface_height_m: float) -> tuple[float, float] | None:
            scene, _ = camera_scene(robot, config, camera)
            if not (0.0 <= u < scene.intr.width and 0.0 <= v < scene.intr.height):
                raise ValueError(f"pixel ({u:g}, {v:g}) is outside the image ({scene.intr.width}x{scene.intr.height})")
            ground = scene.ground_point(u, v, surface_height_m)
            if ground is None:
                return None
            base = scene.ref_to_base(ground)
            if base is None:
                raise ValueError(f"the {camera} camera has no base_link frame: arm.base_in_base_link is not configured")
            pose = robot.robot_pose()
            if pose is None:
                raise RobotError("robot pose unknown: cannot place the pixel on the map")
            return base_to_map(pose, float(base[0]), float(base[1]))

        return resolve_target(target, objects, poi_lookup(robot, POI_KINDS), pixel_xy, poi_z_lookup(robot, POI_KINDS))

    def docking_state() -> dict[str, Any]:
        """Docking server readiness."""
        try:
            ready = bool(robot.dock_ready())
        except RobotError:
            ready = False
        return {"server_ready": ready, "action": settings.dock_action}

    def plan(target: TargetPoint, pitch_deg: float | None, max_results: int, check_path: bool) -> ApproachSuggestion:
        """Ranked approach candidates for a resolved target (refuses without a usable reach map or costmap)."""
        if reach_map is None:
            raise ToolError(f"approach refused: {reach_error}")
        pose = robot.robot_pose()
        seed = ik_seed()
        result = suggest(
            target,
            reach_map.cells(target.z, pitch_deg),
            pan_axis_xy(),
            footprint_points(ctx),
            robot.global_costmap(),
            settings,
            None if pose is None else (pose.x, pose.y),
            pitch_deg,
            max_results,
            verify=lambda point: point_reach(checker, point, seed, envelope),
            plan=robot.plan_path if check_path else None,
        )
        info = reach_map.describe()
        return result.model_copy(
            update={
                "reach_map": {"key": info["key"], "created_at": info["created_at"]},
                "docking": docking_state(),
            }
        )

    @server.tool(
        description=(
            "Where to park the base before a grasp (read-only, no lease, no motion). Inverts the arm's offline reach "
            "map around the target and scores base poses on the global costmap with the CURRENT robot footprint "
            "(dock pose, staging pose 0.30 m behind it and the corridor between must be free and known), then "
            "verifies the best with point_reach (the grasp planner's IK and body check) and plans paths to the best "
            "(ComputePathToPose, planning only). Returns ranked candidates (pose, staging_pose, reach with margins, "
            "costmap costs, verified, path), rejection counts, the costmap age, the reach map key and the docking "
            "server readiness. Empty candidates carry the reason (out_of_reach, collision, unknown_space, ...); no "
            "fallback pose is invented. Refuses when the reach map is stale (the message has the regenerate command) "
            "or the global costmap is missing or older than "
            f"{settings.costmap_stale_s:g} s. Pass a candidate pose to approach_for_grasp to drive there. {TARGET_HELP}"
        )
    )
    def suggest_approach_pose(
        x: Annotated[float | None, Field(description="Target x (map, m)")] = None,
        y: Annotated[float | None, Field(description="Target y (map, m)")] = None,
        z: Annotated[float | None, Field(description="Target height above the floor (m)")] = None,
        object_id: Annotated[str | None, Field(description="Remembered object id (needs z_m)")] = None,
        poi_id: Annotated[
            str | None, Field(description="Point, area or marker POI id (needs z_m, except a marker: its own z)")
        ] = None,
        z_m: Annotated[float | None, Field(description="Grasp height above the floor for object_id / poi_id")] = None,
        camera: Annotated[str | None, Field(description="'front' or 'gripper' (pixel target)")] = None,
        u: Annotated[float | None, Field(description="Pixel x of the calibrated image (pixel target)")] = None,
        v: Annotated[float | None, Field(description="Pixel y of the calibrated image (pixel target)")] = None,
        surface_height_m: Annotated[
            float | None, Field(description="Height of the surface under the pixel above the floor (m, default 0)")
        ] = None,
        pitch_deg: Annotated[
            float | None, Field(description="Approach pitch (deg, + down) the arm must reach; default any")
        ] = None,
        max_results: Annotated[int, Field(ge=1, le=MAX_RESULTS_LIMIT, description="Candidates returned")] = 5,
        check_path: Annotated[bool, Field(description="Plan a path to the best candidates (planning only)")] = True,
    ) -> ApproachSuggestion:
        """Ranked approach poses; the tool description is passed to the decorator (it states the costmap limit)."""
        with approach_errors():
            resolved = resolve(build_target(x, y, z, object_id, poi_id, z_m, camera, u, v, surface_height_m))
            return plan(resolved, pitch_deg, max_results, check_path)

    def camera_view_problem(camera: str, cand: Candidate, target: TargetPoint) -> str | None:
        """Why a camera cannot track the target from the staging and the dock pose, None when it can."""
        try:
            scene, _ = camera_scene(robot, config, camera)
        except CameraNotCalibratedError:
            return "not calibrated"
        except (ArmError, RobotError, ValueError) as exc:
            return "head joint states stale" if camera == "front" and isinstance(exc, ValueError) else str(exc)
        margin = max(settings.tracker.edge_margin_px, settings.tracker.template_px / 2)
        for label, pose in (("staging", cand.staging_pose), ("dock", cand.pose)):
            if not visible(scene, np.array(target_in_base(target, (pose.x, pose.y, pose.yaw))), margin):
                return f"target not in view from the {label} pose"
        return None

    def choose_mode(prefer: PreferMode, cand: Candidate, target: TargetPoint) -> tuple[Mode, str, str | None]:
        """Servoed only when a calibrated camera sees the target from the staging and the dock pose."""
        if prefer == "navigate":
            return "navigate", "prefer_mode=navigate", None
        reasons: list[str] = []
        for camera in CAMERAS:
            problem = camera_view_problem(camera, cand, target)
            if problem is None:
                return "servoed", f"{camera} camera sees the target from the staging and dock poses", camera
            reasons.append(f"{camera}: {problem}")
        reason = "; ".join(reasons)
        if prefer == "servoed":
            raise ToolError(f"prefer_mode='servoed' is not possible: {reason}")
        return "navigate", reason, None

    def pick_candidate(suggestion: ApproachSuggestion, given: ApproachPose | None) -> Candidate:
        """The best candidate, or the given pose re-validated against the current planner output."""
        if given is None:
            if not suggestion.candidates:
                raise ToolError(
                    f"no valid approach candidate; rejected: {suggestion.rejected.model_dump()}; "
                    f"{'; '.join(suggestion.notes)}"
                )
            return suggestion.candidates[0]
        for cand in suggestion.candidates:
            turn = abs(float(wrap_angle(np.array([cand.pose.yaw - given.yaw]))[0]))
            if math.hypot(cand.pose.x - given.x, cand.pose.y - given.y) <= POSE_TOL_M and turn <= POSE_TOL_RAD:
                return cand
        raise ToolError(
            f"pose ({given.x:.3f}, {given.y:.3f}, {given.yaw:.3f}) is not a valid candidate now (changed costmap, "
            f"reach or target?); rejected: {suggestion.rejected.model_dump()}; call suggest_approach_pose again"
        )

    def make_tracker(cand: Candidate, target: TargetPoint) -> TargetTracker:
        """Tracker of the target for a dock pose (t_rel = the target in the planned dock frame)."""
        rel = target_in_base(target, (cand.pose.x, cand.pose.y, cand.pose.yaw))
        return TargetTracker((target.x, target.y), cand.pose.yaw, (rel[0], rel[1]), settings.tracker)

    def tracker_source(camera: str, surface_z: float) -> Callable[[], TrackerInput | None]:
        """Newest frame with its geometry (current arm pose, current robot pose), None without a robot pose."""

        def source() -> TrackerInput | None:
            pose = robot.robot_pose()
            if pose is None:
                return None
            scene, setup = camera_scene(robot, config, camera)
            frame, stamp_ns = grab_frame(robot, setup)
            return TrackerInput(frame, stamp_ns, scene, pose, surface_z)

        return source

    def tracker_dry_run(camera: str, cand: Candidate, target: TargetPoint) -> dict[str, Any]:
        """One tracker step on a fresh frame (a second one when the first only seeded); nothing is published."""
        tracker = make_tracker(cand, target)
        source = tracker_source(camera, target.z)
        out: dict[str, Any] = {"camera": camera, "frames": 0, "seeded": False, "detection": None, "note": ""}
        try:
            for _ in range(2):
                sample = source()
                if sample is None:
                    out["note"] = "no robot pose: no frame was processed"
                    break
                out["frames"] += 1
                detection = tracker.step(
                    sample.frame_bgr, sample.stamp_ns, sample.scene, sample.base_pose, sample.surface_z
                )
                if detection is not None:
                    out["detection"] = detection_summary(detection)
                    break
                if not tracker.seeded:
                    break
        except (ArmError, RobotError, ValueError) as exc:
            out["note"] = f"frame unavailable: {exc}"
        out["seeded"] = tracker.seeded
        if not out["note"] and out["detection"] is None:
            out["note"] = (
                "seeded, but no match was accepted on the next frame"
                if tracker.seeded
                else "target not in view from the current pose: the tracker cannot seed yet (the servoed run seeds "
                "once the robot has reached the staging pose)"
            )
        return out

    def timeout_of(value: float | None, default: float) -> float:
        """A timeout, the default when none was given, capped at nav_max_timeout_s."""
        return min(value if value is not None else default, nav_max)

    def finish(
        status: ApproachStatus,
        message: str,
        mode: Mode,
        reason: str,
        target: TargetPoint,
        cand: Candidate,
        started: float,
        **fields: Any,
    ) -> ApproachResult:
        """Build the result with the common fields."""
        return ApproachResult(
            status=status,
            message=message,
            mode=mode,
            mode_reason=reason,
            target=target,
            plan=cand,
            goal=cand.pose,
            docking=docking_state(),
            duration_s=round(time.monotonic() - started, 3),
            **fields,
        )

    @server.tool(
        description=(
            "Drive the base to the best approach pose for a grasp (needs the base lease; Nav2 plans around "
            "obstacles; blocks until done). Runs suggest_approach_pose internally (or re-validates the candidate "
            "`pose` you pass; a pose that is no longer valid is refused), then picks the mode: 'servoed' ONLY when a "
            "calibrated camera (front, or the gripper camera at the current arm pose) sees the target from both the "
            "staging and the dock pose: the grasp docking server (DockRobot, dock_type grasp_servoed) navigates to the "
            "staging pose and then servos onto detections of the target that a tracker publishes from camera frames; "
            "otherwise 'navigate' (a precise Nav2 goal), the reason is in mode_reason. prefer_mode forces one "
            "(servoed is refused when it is not possible). After an arrival the target is re-checked with point_reach "
            "at the measured pose: status 'arrived_out_of_reach' means the base is parked but the arm cannot reach the "
            "target from there (never a success: suggest again and re-approach). A failed servoed dock (e.g. "
            "FAILED_TO_DETECT_DOCK 904) suggests prefer_mode='navigate'. dry_run=true sends nothing and publishes "
            "nothing: it returns the plan, the mode and reason, the goal it would send, the docking server readiness "
            "and, in servoed mode, one tracker step on a fresh frame. The grasp itself is a separate call "
            f"(plan_grasp, grasp_object). {TARGET_HELP}"
        )
    )
    def approach_for_grasp(
        x: Annotated[float | None, Field(description="Target x (map, m)")] = None,
        y: Annotated[float | None, Field(description="Target y (map, m)")] = None,
        z: Annotated[float | None, Field(description="Target height above the floor (m)")] = None,
        object_id: Annotated[str | None, Field(description="Remembered object id (needs z_m)")] = None,
        poi_id: Annotated[
            str | None, Field(description="Point, area or marker POI id (needs z_m, except a marker: its own z)")
        ] = None,
        z_m: Annotated[float | None, Field(description="Grasp height above the floor for object_id / poi_id")] = None,
        camera: Annotated[str | None, Field(description="'front' or 'gripper' (pixel target)")] = None,
        u: Annotated[float | None, Field(description="Pixel x of the calibrated image (pixel target)")] = None,
        v: Annotated[float | None, Field(description="Pixel y of the calibrated image (pixel target)")] = None,
        surface_height_m: Annotated[
            float | None, Field(description="Height of the surface under the pixel above the floor (m, default 0)")
        ] = None,
        pose: Annotated[
            ApproachPose | None, Field(description="A candidate pose of suggest_approach_pose (re-validated)")
        ] = None,
        pitch_deg: Annotated[
            float | None, Field(description="Approach pitch (deg, + down) the arm must reach; default any")
        ] = None,
        prefer_mode: Annotated[PreferMode, Field(description="auto (default), servoed or navigate")] = "auto",
        timeout_s: Annotated[
            float | None, Field(gt=0.0, description="Give up after this many seconds (default: configured)")
        ] = None,
        dry_run: Annotated[bool, Field(description="Plan only: send and publish nothing")] = False,
    ) -> ApproachResult:
        """Drive to the approach pose; the tool description is passed to the decorator."""
        ctx.battery_gate("approach_for_grasp")
        ctx.tilt_gate("approach_for_grasp", "nav")
        with approach_errors():
            started = time.monotonic()
            resolved = resolve(build_target(x, y, z, object_id, poi_id, z_m, camera, u, v, surface_height_m))
            if robot.robot_pose() is None:
                raise RobotError("robot pose unknown: cannot plan the approach")
            suggestion = plan(resolved, pitch_deg, ALL_CANDIDATES, True)
            cand = pick_candidate(suggestion, pose)
            mode, reason, tracking_camera = choose_mode(prefer_mode, cand, resolved)
            goal = DockGoal(
                dock_pose=cand.pose,
                dock_type=GRASP_DOCK_TYPE,
                navigate_to_staging_pose=True,
                max_staging_time_s=timeout_of(timeout_s, config.timeouts.nav_default_timeout_s),
            )
            default_total_s = settings.dock_timeout_s if mode == "servoed" else config.timeouts.nav_default_timeout_s
            total_s = timeout_of(timeout_s, default_total_s)
            if dry_run:
                nav_goal = {
                    "x": cand.pose.x,
                    "y": cand.pose.y,
                    "yaw": cand.pose.yaw,
                    "frame": cand.pose.frame,
                    "precise": True,
                    "timeout_s": total_s,
                }
                would_send = (
                    {"dock_goal": goal.model_dump(), "timeout_s": total_s, "action": settings.dock_action}
                    if mode == "servoed"
                    else {"navigate_to_pose": nav_goal}
                )
                tracker = None if tracking_camera is None else tracker_dry_run(tracking_camera, cand, resolved)
                return finish(
                    "planned",
                    "dry run: nothing was sent",
                    mode,
                    reason,
                    resolved,
                    cand,
                    started,
                    dry_run=True,
                    would_send=would_send,
                    tracker=tracker,
                )
            if mode == "servoed":
                assert tracking_camera is not None
                loop = DetectionLoop(
                    make_tracker(cand, resolved),
                    tracker_source(tracking_camera, resolved.z),
                    lambda detection: robot.publish_detection(detection.pose, detection.stamp_ns),
                    settings.tracker.rate_hz,
                )
                loop.start()
                try:
                    docked: DockResult = robot.dock(goal, total_s)
                finally:
                    loop.stop()
                status: ApproachStatus = docked.status
                message = docked.message
                if status == "failed":
                    message = f"{message}; retry with prefer_mode='navigate'".lstrip("; ")
                extra: dict[str, Any] = {
                    "error_code": docked.error_code,
                    "error_name": docked.error_name,
                    "num_retries": docked.num_retries,
                    "interrupted_by": docked.interrupted_by,
                    "detections_published": loop.published,
                    "detections_rejected": loop.tracker.rejected,
                }
                final = docked.final_pose
            else:
                nav: NavigationResult = robot.navigate(
                    cand.pose.x, cand.pose.y, cand.pose.yaw, cand.pose.frame, total_s, True
                )
                status = NAV_STATUS.get(nav.status, "failed")  # type: ignore[assignment]
                message = nav.message
                extra = {"interrupted_by": nav.interrupted_by}
                final = nav.final_pose
            reach_after = None
            if status == "succeeded":
                at = final or robot.robot_pose()
                if at is None:
                    reach_after = {"checked": False, "reason": "robot pose unknown after the motion"}
                else:
                    r = reach_at(resolved, (at.x, at.y, at.yaw), pitch_deg)
                    ok = bool(r["reachable"] and r["collision_free"])
                    reach_after = {
                        "checked": True,
                        "reachable": r["reachable"],
                        "collision_free": r["collision_free"],
                        "pitch_range_deg": r["pitch_range_deg"],
                        "pose": {"x": at.x, "y": at.y, "yaw": at.yaw},
                        "suggestion": r.get("suggestion"),
                    }
                    if not ok:
                        status = "arrived_out_of_reach"
                        message = (
                            "arrived, but the target is out of reach from the measured pose (reachable="
                            f"{r['reachable']}, collision_free={r['collision_free']}); call suggest_approach_pose "
                            "again and re-approach"
                        )
            return finish(
                status,
                message,
                mode,
                reason,
                resolved,
                cand,
                started,
                final_pose=final,
                reach_after=reach_after,
                **extra,
            )
