"""MCP tool definitions (official MCP Python SDK 2.x MCPServer) and the bearer-token protected Streamable HTTP app."""

import base64
import hmac
import json
import logging
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from typing import Annotated, Any, Literal

from mcp.server.auth.provider import AccessToken
from mcp.server.auth.settings import AuthSettings
from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError, UnexpectedToolError
from mcp.server.streamable_http import MCP_SESSION_ID_HEADER
from mcp.types import CallToolResult, ImageContent, TextContent
from pydantic import Field
from ros2_common.battery import BatteryGuard
from starlette.applications import Starlette
from starlette.concurrency import run_in_threadpool
from starlette.requests import Request
from starlette.responses import JSONResponse

from . import body_tools, camera_tools, grasp_tools, motion_tools, perception_tools
from .arm import MAX_OBJECT_WIDTH_M, ArmError
from .base_motion import DriveError, DriveOutcome
from .config import HARD_MAX_DRIVE_S, HARD_MAX_IMAGE_PX, HARD_MAX_SPEED_SCALE, McpServerConfig
from .floor_guard import TiltOverrideDeg
from .grasp_tools import floor_override
from .models import (
    ArmMotionResult,
    ArmState,
    ControlResult,
    HomeSetResult,
    NavigationResult,
    RobotError,
    RobotState,
    SettlePolicy,
    StopResult,
)
from .monitor import DIGEST_DEFAULT_SESSION, RobotMonitor
from .motion_queue import MotionQueue
from .poi_client import PoiStoreDown, PoiTimeout
from .tool_context import RobotApi, ToolContext

LOGGER = logging.getLogger("mcp_server.timing")
# Keys of an arm motion result copied into the per-call timing line (trajectory seconds vs settle seconds apart).
TIMING_RESULT_KEYS = ("status", "trajectory_s", "settle_s", "settle", "settling", "tracking_error_rad")
SERVER_NAME = "robot"
TOKEN_CLIENT_ID = "robot-mcp-client"
ADMIN_CLEAR_AGENT_POIS_PATH = "/admin/clear_agent_pois"
ADMIN_POI_CREATOR = "agent"
TOOL_NAMES = (
    "get_robot_state",
    "get_camera_image",
    "get_map_summary",
    "navigate_to_pose",
    "move_relative",
    "drive",
    "stop",
    "get_arm_state",
    "acquire_control",
    "release_control",
    "move_arm_joints",
    "move_arm_cartesian",
    "set_gripper",
    "arm_home",
    "arm_set_home",
    "get_body_state",
    *camera_tools.TOOL_NAMES,
    *perception_tools.TOOL_NAMES,
    *grasp_tools.TOOL_NAMES,
    *motion_tools.TOOL_NAMES,
)
# Battery cut-off classification, the single source of truth (also read by claude_agent for its effector caps).
# MOTION_TOOLS move the base or the arm/gripper (arm_set_home is included: it rewrites the stored home pose a later
# arm_home drives to) and are refused while the battery is below cut-off. ALWAYS_ALLOWED_TOOLS (stop, sensors, state,
# acquire/release control) keep working so the robot can always be stopped, inspected and handed back.
MOTION_TOOLS = frozenset(
    {
        "navigate_to_pose",
        "move_relative",
        "drive",
        "move_arm_joints",
        "move_arm_cartesian",
        "set_gripper",
        "arm_home",
        "arm_set_home",
        "look_around",
        "grasp_object",
        "release_object",
        "enqueue_motions",
    }
)
ALWAYS_ALLOWED_TOOLS = frozenset(
    {
        "get_robot_state",
        "get_camera_image",
        "get_map_summary",
        "stop",
        "get_arm_state",
        "acquire_control",
        "release_control",
        "get_body_state",
        *camera_tools.TOOL_NAMES,
        *perception_tools.SENSOR_TOOL_NAMES,
        "plan_grasp",
        "get_motion_status",
        "cancel_motions",
        "wait_for_event",
    }
)
# Blocking motion tools refused while the motion queue runs (the queue's worker is the single motion owner then).
# enqueue_motions appends to the queue and arm_set_home moves nothing.
QUEUE_EXCLUSIVE_TOOLS = MOTION_TOOLS - {"enqueue_motions", "arm_set_home"}
DIGEST_EVENTS_KEY = "robot_events_since_last_call"
DIGEST_VITALS_KEY = "vitals"
INSTRUCTIONS = """Controls a swerve-drive mobile robot with an SO101 5-DOF arm and gripper (ROS 2 Jazzy, Nav2, SLAM).
Frames: 'map' is the SLAM map (x/y metres, yaw radians CCW); 'base_link' is the robot (x forward, y left).
Look before moving: call get_robot_state, get_map_summary and get_camera_image first. Prefer navigate_to_pose /
move_relative (Nav2 plans around obstacles) over drive. Arm motions are slow, clamped to joint limits and abort on
stale feedback or tracking error. Arm control (the autonomy lease) is sticky: once taken (acquire_control or any arm
motion) the leader arm and web UI are ignored until you call release_control, so release it as soon as you are done
with the arm. Call stop at once if anything looks wrong; it is always available. Every tool result ends with
robot_events_since_last_call (body events such as overheat, collision_stop, stall, bump, battery_low raised since the
previous tool call) and a vitals one-liner (battery V, hottest servo C, CPU C); get_body_state gives the full picture.
A motion that ends with status 'interrupted' was stopped safely because of a critical event (interrupted_by): read
the event, check get_body_state and decide before moving again. Motion results compare expected with achieved.
Non-blocking motion: enqueue_motions queues several steps and returns at once (consecutive arm steps blend into one
continuous trajectory), wait_for_event blocks until the queue drains or something relevant happens and returns the
events plus a state digest, cancel_motions or stop clear the queue. While the queue runs the blocking motion tools are
refused."""

Camera = Literal["gripper", "front"]


class StaticTokenVerifier:
    """TokenVerifier accepting exactly one shared bearer token (constant-time comparison)."""

    def __init__(self, token: str) -> None:
        """Store the expected token.

        Args:
            token (str): The shared secret.
        """
        self.token = token

    async def verify_token(self, token: str) -> AccessToken | None:
        """Accept the configured token only.

        Args:
            token (str): Bearer token from the Authorization header.

        Returns:
            AccessToken | None: Access info when valid, else None (401).
        """
        if hmac.compare_digest(token.encode(), self.token.encode()):
            return AccessToken(token=token, client_id=TOKEN_CLIENT_ID, scopes=[])
        return None


@contextmanager
def tool_errors() -> Iterator[None]:
    """Turn robot-side failures into MCP tool errors the model can read.

    Yields:
        None: Context body.
    """
    try:
        yield
    except (ArmError, RobotError, DriveError, ValueError) as exc:
        raise ToolError(str(exc)) from exc


def image_content(data: bytes, mime_type: str) -> ImageContent:
    """Base64 image content block.

    Args:
        data (bytes): Encoded image.
        mime_type (str): MIME type.

    Returns:
        ImageContent: Content block.
    """
    return ImageContent(type="image", data=base64.b64encode(data).decode(), mime_type=mime_type)


def session_key(context: Any) -> str:
    """Digest cursor key of a request: its Mcp-Session-Id header, DIGEST_DEFAULT_SESSION when there is none.

    Args:
        context (Any): Request context handed to call_tool (None outside a request).

    Returns:
        str: Session id.
    """
    try:
        headers = None if context is None else context.headers
    except (AttributeError, ValueError):  # no request attached (direct call_tool)
        headers = None
    return (headers.get(MCP_SESSION_ID_HEADER) if headers else None) or DIGEST_DEFAULT_SESSION


class RobotMCPServer(MCPServer):
    """MCPServer that appends the robot event digest and vitals to the result of EVERY tool call.

    Overriding call_tool (the single entry the MCP request handler goes through) covers all current and future tools
    without any per-tool code. The digest holds the events raised since the previous tool call of the same MCP session
    (cursor per Mcp-Session-Id, at most RobotMonitor.max_digest_sessions tracked, LRU; calls without a session id share
    one cursor) and a one-line vitals summary.
    """

    def __init__(self, *args: Any, monitor: RobotMonitor, **kwargs: Any) -> None:
        """Create the server.

        Args:
            *args (Any): MCPServer positional arguments.
            monitor (RobotMonitor): Source of the digest.
            **kwargs (Any): MCPServer keyword arguments.
        """
        super().__init__(*args, **kwargs)
        self.monitor = monitor
        self.motion_queue: MotionQueue | None = None
        self.blocking: dict[str, int] = {}  # running blocking motion tools (QUEUE_EXCLUSIVE_TOOLS) -> count

    def blocking_tool(self) -> str | None:
        """Name of a blocking motion tool running now (the motion queue refuses to start meanwhile).

        Returns:
            str | None: A tool name, or None.
        """
        return next((name for name, count in list(self.blocking.items()) if count > 0), None)

    async def call_tool(self, name: str, arguments: dict[str, Any], context: Any = None) -> Any:
        """Run a tool with its digest and log one timing line (tool, start, end, duration; arm motions also the
        trajectory and settle seconds apart) on the ``mcp_server.timing`` logger, also for failing calls.

        Args:
            name (str): Tool name.
            arguments (dict[str, Any]): Tool arguments.
            context (Any): Request context passed through to MCPServer.

        Returns:
            Any: The tool result with robot_events_since_last_call and vitals added.
        """
        started_at = time.time()
        t0 = time.monotonic()
        ok = False
        timing: dict[str, Any] = {}
        try:
            result = await self.call_tool_with_digest(name, arguments, context)
            ok = True
            if isinstance(result, CallToolResult) and isinstance(result.structured_content, dict):
                timing = {k: result.structured_content[k] for k in TIMING_RESULT_KEYS if k in result.structured_content}
            return result
        finally:
            duration = time.monotonic() - t0
            line = {
                "tool": name,
                "started_at": round(started_at, 3),
                "ended_at": round(started_at + duration, 3),
                "duration_s": round(duration, 3),
                "ok": ok,
                **timing,
            }
            LOGGER.info("tool_call %s", json.dumps(line))

    async def call_tool_with_digest(self, name: str, arguments: dict[str, Any], context: Any = None) -> Any:
        """Run a tool, then attach the digest to its result (or to its error message).

        Args:
            name (str): Tool name.
            arguments (dict[str, Any]): Tool arguments.
            context (Any): Request context passed through to MCPServer.

        Returns:
            Any: The tool result with robot_events_since_last_call and vitals added.
        """
        exclusive = name in QUEUE_EXCLUSIVE_TOOLS
        try:
            if exclusive and self.motion_queue is not None and self.motion_queue.busy():
                raise ToolError(
                    f"{name} refused: the motion queue is running (it owns the robot's motion). Use enqueue_motions, "
                    "wait for it with wait_for_event, or end it with cancel_motions / stop"
                )
            if exclusive:
                self.blocking[name] = self.blocking.get(name, 0) + 1
            try:
                result = await super().call_tool(name, arguments, context)
            finally:
                if exclusive:
                    self.blocking[name] -= 1
        except UnexpectedToolError:
            raise
        except ToolError as exc:
            events, vitals = self.monitor.digest(session_key(context))
            raise ToolError(f"{exc}\n{json.dumps({DIGEST_EVENTS_KEY: events, DIGEST_VITALS_KEY: vitals})}") from exc
        if not isinstance(result, CallToolResult):
            return result
        events, vitals = self.monitor.digest(session_key(context))
        payload = {DIGEST_EVENTS_KEY: events, DIGEST_VITALS_KEY: vitals}
        update: dict[str, Any] = {"content": [*result.content, TextContent(type="text", text=json.dumps(payload))]}
        if result.structured_content is not None:
            update["structured_content"] = {**result.structured_content, **payload}
        else:  # unstructured (image) tools stay unstructured so clients keep rendering their content
            update["meta"] = {**(result.meta or {}), **payload}
        return result.model_copy(update=update)


def build_mcp_server(
    robot: RobotApi,
    config: McpServerConfig,
    token: str,
    guard: BatteryGuard | None = None,
    monitor: RobotMonitor | None = None,
    extra_modules: tuple[Callable[[ToolContext], None], ...] = (),
) -> RobotMCPServer:
    """Create the MCP server with every robot tool, protected by a static bearer token.

    Args:
        robot (RobotApi): Robot implementation.
        config (McpServerConfig): Node configuration.
        token (str): Bearer token clients must send.
        guard (BatteryGuard | None): Battery cut-off guard; None disables the gate.
        monitor (RobotMonitor | None): Body monitor (digest, get_body_state); a standalone one when None.
        extra_modules (tuple[Callable[[ToolContext], None], ...]): Additional tool modules (tests).

    Returns:
        RobotMCPServer: Configured server.
    """
    if not token:
        raise ValueError("a bearer token is required")
    auth = AuthSettings(
        issuer_url=f"http://{config.server.host}:{config.server.port}",
        resource_server_url=None,
        required_scopes=[],
    )
    monitor = monitor or RobotMonitor(config.monitor, guard, autonomy_source=config.arm.autonomy_source_name)
    server = RobotMCPServer(
        SERVER_NAME,
        instructions=INSTRUCTIONS,
        token_verifier=StaticTokenVerifier(token),
        auth=auth,
        monitor=monitor,
    )
    register_tools(server, robot, config, guard, monitor, extra_modules)
    return server


def register_tools(
    server: MCPServer,
    robot: RobotApi,
    config: McpServerConfig,
    guard: BatteryGuard | None = None,
    monitor: RobotMonitor | None = None,
    extra_modules: tuple[Callable[[ToolContext], None], ...] = (),
) -> None:
    """Register every tool module (TOOL_MODULES plus extra_modules) with one shared ToolContext.

    Args:
        server (MCPServer): Target server.
        robot (RobotApi): Robot implementation.
        config (McpServerConfig): Node configuration.
        guard (BatteryGuard | None): Battery cut-off guard; motion tools are refused while it is in cut-off.
        monitor (RobotMonitor | None): Body monitor; a standalone one when None.
        extra_modules (tuple[Callable[[ToolContext], None], ...]): Additional tool modules.
    """
    monitor = monitor or RobotMonitor(config.monitor, guard, autonomy_source=config.arm.autonomy_source_name)
    queue = MotionQueue(
        robot,
        config,
        guard,
        grasp_runner=motion_tools.make_grasp_runner(robot, config),
        external_busy=server.blocking_tool if isinstance(server, RobotMCPServer) else None,
    )
    if isinstance(server, RobotMCPServer):
        server.motion_queue = queue
    ctx = ToolContext(server=server, robot=robot, config=config, guard=guard, monitor=monitor, queue=queue)
    for module in (*TOOL_MODULES, *extra_modules):
        module(ctx)


def register_core_tools(ctx: ToolContext) -> None:
    """Register the base, arm, camera and map tools.

    Args:
        ctx (ToolContext): Shared registration context.
    """
    server, robot, config = ctx.server, ctx.robot, ctx.config
    tool: Callable[..., Callable[[Callable[..., object]], Callable[..., object]]] = server.tool
    nav_default = config.timeouts.nav_default_timeout_s
    nav_max = config.timeouts.nav_max_timeout_s
    nav_note = (
        f"By default a goal ends as soon as the robot is within {config.nav.intermediate_xy_tolerance_m * 100:g} cm "
        f"and {config.nav.intermediate_yaw_tolerance_deg:g} deg of the target (the goal is then cancelled, status "
        f"'succeeded'). Pass precise=true for the final approach before a grasp or fine positioning: Nav2 then "
        f"finishes within {config.nav.goal_xy_tolerance_m * 100:g} cm and {config.nav.goal_yaw_tolerance_deg:g} deg "
        "(slower). If the goal is to the side or behind, the robot "
        "first turns toward the path (front leading, because the lidar sees best ahead) and then turns back to the "
        "goal heading at the end."
    )
    precise_desc = (
        f"true waits for Nav2's tight goal checker ({config.nav.goal_xy_tolerance_m * 100:g} cm, "
        f"{config.nav.goal_yaw_tolerance_deg:g} deg; use it for the final approach); false (default) ends the goal "
        f"within {config.nav.intermediate_xy_tolerance_m * 100:g} cm and {config.nav.intermediate_yaw_tolerance_deg:g} deg"
    )
    floor_note = (
        f"The floor is at z = {config.arm.floor_z_m:.3f} m in this frame "
        f"(the arm mount is {config.arm.arm_base_height_m * 100:.1f} cm above it)."
    )
    settle_note = (
        "Settle policy (`settle`): 'trajectory_end' returns as soon as the streamed trajectory finished, with status "
        "'converged' and settling=true plus the current tracking_error_rad when joints are still closing in (the goal "
        "stays commanded; a following move starts the joints it names from their measured state), saving the "
        f"convergence wait (about {config.timeouts.arm_converge_timeout_s:g} s at most); 'final' waits for convergence. "
        f"Default: {config.limits.arm_default_settle} ('final' when the call moves the gripper joint): pass "
        "settle='final' before closing the gripper on an object or before a camera image that must show the arm at "
        "rest. Convergence uses per-joint tolerances (the loaded shoulder_lift and elbow_flex get wider ones) and does "
        "not wait for a gripper that is holding an object. "
        "Joints you do not name keep their last commanded target (not their measured, gravity-sagged position). "
        f"Status 'converged' means every moved joint is within {config.limits.arm_converge_tolerance_rad} rad of its "
        "target, or stopped short of it under load (servo steady-state error) by at most "
        f"{config.limits.arm_settle_tolerance_rad} rad: then residual_error lists target - measured (rad) per joint "
        f"and the target stays commanded for {config.limits.arm_settle_hold_s} s; after that the hold of joints still "
        "off target relaxes to their measured pose (a stall against an obstacle is not pushed indefinitely) while "
        "the intended target is kept for later motions, so do not re-send it to compensate. Gripper targets are "
        "follower gripper joint positions (rad, as in get_arm_state)."
    )

    speed_desc = (
        f"{HARD_MAX_SPEED_SCALE:g} = max joint speed ({config.limits.arm_max_joint_velocity_rps:g} rad/s) and the default; "
        "lower is slower. Use a lower value only for the last few centimetres of a grasp or near obstacles"
    )
    roll_note = (
        f"Wrist roll guard: a roll change above {config.limits.roll_guard_min_change_rad:g} rad is refused (nothing "
        f"moves) while the gripper is open wider than {config.limits.roll_max_gripper_open_rad:g} rad (measured or "
        "targeted in the same call): set the gripper about half open first (set_gripper open_fraction about 0.5, "
        "enough to keep the finger out of the picture), lift the arm clear of the robot body and objects, then roll."
    )

    slow = config.floor_guard
    slow_note = (
        "Below-surface slow zone: where a jaw tip, the wrist or the elbow comes within "
        f"{slow.margin_m:g} m of the effective surface (the higher of the robot plane at surface_z_m, default "
        f"{slow.surface_z_m:g} m in base_link, and the gravity-level plane from the IMU tilt) the motion slows to "
        f"{slow.slow_speed_scale:g} of its speed (never blocked; reported as slow_zone). Pass surface_z_m (e.g. -0.18 "
        "for a stair or hole below) to allow normal speed down to that surface, tilt_override_deg to replace the IMU."
    )
    settle_desc = (
        "'trajectory_end': return when the streamed trajectory finished (status 'converged', settling=true while "
        "joints still close in); 'final': wait for convergence. Default: "
        f"{config.limits.arm_default_settle}, but 'final' for a call that moves the gripper joint"
    )

    def resolve_settle(settle: SettlePolicy | None, moves_gripper: bool = False) -> SettlePolicy:
        """Settle policy of a call: the given one, else 'final' when it moves the gripper joint, else the configured default."""
        if settle is not None:
            return settle
        return "final" if moves_gripper else config.limits.arm_default_settle

    surface_desc = "Expected surface height relative to the robot plane (m, base_link z), e.g. -0.18 for a stair below"
    tilt_desc = (
        "Robot tilt {roll, pitch} (deg) replacing the IMU for the slow zone (roll > 0 left up, pitch > 0 nose down)"
    )

    battery_gate = ctx.battery_gate

    def nav_timeout(timeout_s: float | None) -> float:
        value = nav_default if timeout_s is None else timeout_s
        if value > nav_max:
            raise ToolError(f"timeout_s must be <= {nav_max} s")
        return float(value)

    @tool()
    def get_robot_state() -> RobotState:
        """Snapshot of the whole robot: base pose in the map (TF map->base_link), odometry velocity, the latest Nav2
        navigation goal status, the collision monitor action (if published), arm joint positions/efforts, gripper
        effort, which arm source filter_node has active (leader, web_ui or autonomy), whether this server holds the
        arm autonomy lease, and the age in seconds of every data source. Stale sources are omitted (never invented)
        and listed in `notes`. Call this first, then at checkpoints (phase boundaries, before an irreversible action such as
        closing the gripper, and when a tool reports a problem); a motion tool's own result already reports
        expected vs achieved, so it needs no routine check afterwards."""
        with tool_errors():
            return robot.robot_state()

    @tool(
        structured_output=False,
        description=(
            "Take one fresh photo from a robot camera and return it as a JPEG image plus its capture timestamp. "
            "'gripper' looks out of the gripper (use it to aim grasps); 'front' is the overhead camera (640x480) "
            "looking down at the front of the robot, the arm and the floor in front of it: the best view for judging "
            "gripper-to-object position (take it first for an overview, then use 'gripper' to aim). The default size "
            f"is {config.limits.default_camera_px} px on the longest side (cheap, fine for aiming checks): pass a "
            "larger max_px only when you must read fine detail, images are what fills the context. Fails (instead of "
            "returning an old picture) when no frame arrives within the timeout or the newest frame is older than 1 s."
        ),
    )
    def get_camera_image(
        camera: Annotated[
            Camera,
            Field(
                description="'gripper' (USB camera on the gripper) or 'front' (overhead camera looking down at the front of the robot, 640x480)"
            ),
        ],
        max_px: Annotated[
            int | None,
            Field(
                ge=32,
                le=HARD_MAX_IMAGE_PX,
                description=f"Longest image side in pixels (default {config.limits.default_camera_px}: enough to aim; "
                "ask for more only to read fine detail, larger images cost more context)",
            ),
        ] = None,
    ) -> list[ImageContent | TextContent]:
        """Photo of a camera; the tool description is passed to the decorator so it can state the default size."""
        with tool_errors():
            size = config.limits.default_camera_px if max_px is None else max_px
            frame = robot.camera_image(camera, min(size, config.limits.max_image_px))
        meta = frame.model_dump_json()
        return [image_content(frame.jpeg, "image/jpeg"), TextContent(type="text", text=meta)]

    @tool(structured_output=False)
    def get_map_summary(
        include_png: Annotated[bool, Field(description="Also return a small PNG of the map around the robot")] = False,
        radius_m: Annotated[float, Field(gt=0.5, le=20.0, description="Half size of the PNG crop (m)")] = 4.0,
    ) -> CallToolResult:
        """Describe the surroundings: the nearest lidar obstacle (/scan_filtered) in each of 8 sectors around the
        robot (front, front_left, left, rear_left, rear, rear_right, right, front_right; distances from base_link in
        metres), SLAM map size and how many cells are known/occupied/free, and the robot pose. Optionally a PNG of
        the occupancy map around the robot (white free, black occupied, grey unknown, red arrow = robot, map +y up)."""
        with tool_errors():
            summary, png = robot.map_summary(include_png, radius_m, config.limits.map_png_max_px)
        content: list[TextContent | ImageContent] = [TextContent(type="text", text=summary.model_dump_json())]
        if png is not None:
            content.append(image_content(png, "image/png"))
        return CallToolResult(content=content, structured_content=summary.model_dump(mode="json"))

    @tool(
        description=(
            "Drive the base to a pose with Nav2 (path planning, obstacle avoidance, velocity smoothing, collision "
            "monitor). Blocks until Nav2 reports a result or the timeout expires (the goal is then cancelled), and "
            "returns the result and the final pose. Use get_map_summary first to pick a reachable free-space goal. "
            f"{nav_note}"
        )
    )
    def navigate_to_pose(
        x: Annotated[float, Field(description="Goal x (m) in `frame`")],
        y: Annotated[float, Field(description="Goal y (m) in `frame`")],
        yaw: Annotated[float, Field(description="Goal heading (rad, CCW from +x)")] = 0.0,
        frame: Annotated[str, Field(description="Frame of the goal, normally 'map'")] = "map",
        timeout_s: Annotated[float | None, Field(gt=0.0, description="Give up after this many seconds")] = None,
        precise: Annotated[bool, Field(description=precise_desc)] = False,
    ) -> NavigationResult:
        """Nav2 NavigateToPose; the tool description is passed to the decorator (it states the goal precision)."""
        battery_gate("navigate_to_pose")
        with tool_errors():
            return robot.navigate(x, y, yaw, frame, nav_timeout(timeout_s), precise)

    @tool(
        description=(
            "Move relative to the robot's current pose (base_link frame) through Nav2, e.g. dx=0.5 drives half a "
            "metre forward, dyaw=1.57 turns left 90 degrees. Same blocking/obstacle-avoiding behaviour as "
            "navigate_to_pose. Small relative moves (a few cm) now really move the robot. "
            f"{nav_note}"
        )
    )
    def move_relative(
        dx: Annotated[float, Field(description="Forward displacement (m), negative = backwards")],
        dy: Annotated[float, Field(description="Leftward displacement (m), negative = right")] = 0.0,
        dyaw: Annotated[float, Field(description="Heading change (rad), positive = turn left")] = 0.0,
        timeout_s: Annotated[float | None, Field(gt=0.0, description="Give up after this many seconds")] = None,
        precise: Annotated[bool, Field(description=precise_desc)] = False,
    ) -> NavigationResult:
        """Nav2 NavigateToPose relative to base_link; the tool description is passed to the decorator."""
        battery_gate("move_relative")
        with tool_errors():
            return robot.move_relative(dx, dy, dyaw, nav_timeout(timeout_s), precise)

    @tool()
    def drive(
        vx: Annotated[float, Field(description="Forward velocity (m/s), clamped to +-0.25")],
        vy: Annotated[float, Field(description="Leftward velocity (m/s), clamped to +-0.25")] = 0.0,
        wz: Annotated[float, Field(description="Yaw rate (rad/s), clamped to +-0.5")] = 0.0,
        duration_s: Annotated[float, Field(gt=0.0, le=HARD_MAX_DRIVE_S, description="Seconds to drive (max 2)")] = 1.0,
    ) -> DriveOutcome:
        """Nudge the base with a direct velocity for at most 2 s (20 Hz on /cmd_vel_nav, then zero). Commands still
        pass the velocity smoother and the collision monitor, but there is no path planning: prefer move_relative
        for anything beyond small adjustments."""
        battery_gate("drive")
        with tool_errors():
            return robot.drive(vx, vy, wz, duration_s)

    @tool()
    def stop() -> StopResult:
        """Emergency stop, always available: cancels every Nav2 navigation goal and publishes a zero velocity. The
        arm is frozen at its measured pose (aborting any arm motion) only if this server holds arm control or an arm
        motion is running; if it does not hold control the arm is not touched (arm_held false), so a human driving
        it with the leader arm or web UI keeps it. The motion queue is cleared first (pending steps dropped, listed in
        motion_queue_dropped; the running step ends with the stop). Call it whenever anything looks wrong."""
        dropped = ctx.queue.halt_for_stop() if ctx.queue is not None else []
        with tool_errors():
            result = robot.stop()
        return result.model_copy(update={"motion_queue_dropped": dropped})

    @tool(
        description=(
            "Arm joint positions (rad) and efforts, gripper effort, gripper tool point pose (x, y, z, pitch in the "
            "arm base_link), the filter_node active source, whether this server holds the autonomy lease, whether a "
            "home pose is stored, and the joint_states age. Positions are omitted when the feedback is stale. "
            f"{floor_note} It is also returned as floor_z_m."
        )
    )
    def get_arm_state() -> ArmState:
        """Current arm state; the tool description is passed to the decorator so it can state the floor height."""
        with tool_errors():
            return robot.arm.state()

    @tool()
    def acquire_control() -> ControlResult:
        """Take arm control (autonomy lease) by commanding the current measured pose, so the arm does not move.
        Motion tools acquire implicitly. The lease is sticky: while it is held the leader arm and web UI are
        ignored, and it is not handed back automatically - release it explicitly with release_control when done."""
        with tool_errors():
            return robot.arm.acquire()

    @tool()
    def release_control() -> ControlResult:
        """Give arm control back to the other sources (leader arm / web UI): aborts any running arm motion, stops
        the setpoint keepalive and publishes the autonomy release. Call release_control whenever you are done with
        the arm; control taken by acquire_control or an arm motion is otherwise kept indefinitely."""
        with tool_errors():
            return robot.arm.release()

    @tool(
        description=(
            "Move arm joints to targets along a smooth (quintic) trajectory streamed at 25 Hz. Targets are clamped "
            "to the URDF limits minus a margin (reported in `clamped`). Blocks until converged or timed out; aborts "
            "and holds the measured pose if joint feedback goes stale (>0.3 s), the tracking error grows too large, "
            "the joints do not settle near the target (status 'timeout'), or stop is called. Takes arm control if "
            f"not already held and keeps it afterwards: call release_control when done. {roll_note} {settle_note} "
            f"{slow_note}"
        )
    )
    def move_arm_joints(
        targets: Annotated[
            dict[str, float],
            Field(
                description="Joint name -> target rad; joints: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, "
                "wrist_roll, gripper. Unnamed joints keep their last commanded target."
            ),
        ],
        speed_scale: Annotated[
            float,
            Field(gt=0.0, le=HARD_MAX_SPEED_SCALE, description=speed_desc),
        ] = HARD_MAX_SPEED_SCALE,
        surface_z_m: Annotated[float | None, Field(ge=-1.0, le=1.0, description=surface_desc)] = None,
        tilt_override_deg: Annotated[TiltOverrideDeg | None, Field(description=tilt_desc)] = None,
        settle: Annotated[SettlePolicy | None, Field(description=settle_desc)] = None,
    ) -> ArmMotionResult:
        """Move arm joints; the tool description is passed to the decorator so it can state the tolerances."""
        battery_gate("move_arm_joints")
        policy = resolve_settle(settle, config.arm.gripper_joint in targets)
        with tool_errors():
            return robot.arm.move_joints(targets, speed_scale, floor_override(surface_z_m, tilt_override_deg), policy)

    @tool(
        description=(
            "Move the gripper tool point to (x, y, z) in the arm's base_link frame (arm URDF root: x forward "
            "along the arm at shoulder_pan=0, z up), optionally with an approach pitch. Solves inverse "
            "kinematics on the arm URDF (5-DOF: position + pitch) and streams the joint motion "
            "like move_arm_joints. wrist_roll (rad, as in get_arm_state; clamped to the limits) sets the roll for this "
            "target and the motion rolls the wrist to it; omitted, the current roll is kept. The roll is your choice "
            "per object: it sets the camera view and the jaw orientation (-1.57 camera nearly straight down, 0, "
            "+1.57 camera parallel to the ground), so pick the one that closes the jaws across the object's narrow "
            "side. The tool point is the fixed jaw's inner face; the moving jaw opens away from it. With "
            "object_width_m (m, estimated from the pictures) x, y, z are the OBJECT CENTRE: the tool point is "
            "placed half a width from it so the fixed jaw's inner face lies on the object's side and the object sits "
            "centred between the jaws (the result reports grasp_shift incl. the fixed jaw point); without it the "
            "tool point itself goes to x, y, z. Returns status 'unreachable' without moving when no solution exists "
            "within joint limits (the arm can reach somewhat below the floor, limited by the joint limits). "
            "Keeps arm control afterwards like move_arm_joints: call release_control when done. "
            f"{roll_note} {floor_note} {settle_note} {slow_note}"
        )
    )
    def move_arm_cartesian(
        x: Annotated[float, Field(description="Gripper tool point x (m), forward of the arm base")],
        y: Annotated[float, Field(description="Tool point y (m), left of the arm base")],
        z: Annotated[float, Field(description="Tool point z (m), above the arm base")],
        pitch: Annotated[
            float | None, Field(description="Approach pitch (rad): 0 horizontal, +1.57 pointing straight down")
        ] = None,
        frame: Annotated[Literal["base_link"], Field(description="Arm URDF base_link (the arm mount)")] = "base_link",
        speed_scale: Annotated[
            float, Field(gt=0.0, le=HARD_MAX_SPEED_SCALE, description=speed_desc)
        ] = HARD_MAX_SPEED_SCALE,
        wrist_roll: Annotated[
            float | None,
            Field(description="Wrist roll (rad, measured space) kept by the IK and moved to; None keeps the current"),
        ] = None,
        object_width_m: Annotated[
            float | None,
            Field(
                gt=0.0,
                le=MAX_OBJECT_WIDTH_M,
                description="Object width across the jaws (m): x, y, z are then the object centre, not the tool point",
            ),
        ] = None,
        surface_z_m: Annotated[float | None, Field(ge=-1.0, le=1.0, description=surface_desc)] = None,
        tilt_override_deg: Annotated[TiltOverrideDeg | None, Field(description=tilt_desc)] = None,
        settle: Annotated[SettlePolicy | None, Field(description=settle_desc)] = None,
    ) -> ArmMotionResult:
        """Move the tool point; the tool description is passed to the decorator so it can state the floor."""
        battery_gate("move_arm_cartesian")
        del frame  # the only supported frame
        with tool_errors():
            return robot.arm.move_cartesian(
                x,
                y,
                z,
                pitch,
                speed_scale,
                wrist_roll,
                object_width_m,
                floor_override(surface_z_m, tilt_override_deg),
                resolve_settle(settle),
            )

    @tool()
    def set_gripper(
        open_fraction: Annotated[float | None, Field(ge=0.0, le=1.0, description="0 = closed, 1 = fully open")] = None,
        close_until_effort: Annotated[
            bool, Field(description="Close slowly and stop as soon as the gripper feels an object")
        ] = False,
        effort_threshold: Annotated[
            float | None, Field(gt=0.0, description="Gripper load counted as contact (servo load units)")
        ] = None,
        surface_z_m: Annotated[float | None, Field(ge=-1.0, le=1.0, description=surface_desc)] = None,
        tilt_override_deg: Annotated[TiltOverrideDeg | None, Field(description=tilt_desc)] = None,
    ) -> ArmMotionResult:
        """Open the gripper to a fraction, or close it until it grips something (status 'grasped' and the gripper
        holds that position; 'closed_no_contact' if it closed fully without touching anything). Closing (also
        open_fraction=0) reports 'grasped' too when the jaw stalls before the closed position: contact is inferred
        from the stall and the gripper holds the stall position plus a small squeeze. A contact only counts as
        'grasped' when the jaw closed at least gripper_grasp_min_travel_rad (0.15) from where it started and stopped
        no more open than gripper_grasp_max_open_rad (1.2); otherwise the status is 'blocked' (the jaw is pressing on
        an object, not holding it) and the measured jaw position is held without squeeze. The load is ignored for
        the first 0.3 s of a close and counts only after the jaw moved or stalled. The arm stays held at its intended
        targets while the gripper moves. Give exactly one of
        open_fraction or close_until_effort=true. Keeps arm control afterwards: call release_control when done.
        Below-surface slow zone: near the surface (the moving jaw tip is checked too) the gripper moves slow; pass
        surface_z_m / tilt_override_deg as for the arm motion tools."""
        battery_gate("set_gripper")
        with tool_errors():
            return robot.arm.set_gripper(
                open_fraction, close_until_effort, effort_threshold, floor_override(surface_z_m, tilt_override_deg)
            )

    @tool(
        description=(
            "Move the arm to its stored home pose. Afterwards (also after a failed motion) arm control is kept only "
            "if it was already held before this call; otherwise it is released so the leader arm and web UI work "
            "again. (The /arm/home ROS service, used by the web UI, always releases.) Fails if no home pose has been "
            "stored yet with arm_set_home. Reports 'converged' with residual_error (target - measured, rad) when "
            f"joints settled short of the home pose by at most {config.limits.arm_settle_tolerance_rad} rad. "
            f"{slow_note}"
        )
    )
    def arm_home(
        surface_z_m: Annotated[float | None, Field(ge=-1.0, le=1.0, description=surface_desc)] = None,
        tilt_override_deg: Annotated[TiltOverrideDeg | None, Field(description=tilt_desc)] = None,
    ) -> ArmMotionResult:
        """Move home; the tool description is passed to the decorator so it can state the settle tolerance."""
        battery_gate("arm_home")
        with tool_errors():
            return robot.arm.home(keep_prior_control=True, floor=floor_override(surface_z_m, tilt_override_deg))

    @tool()
    def arm_set_home() -> HomeSetResult:
        """Store the arm's current measured pose as the home pose (same as the /arm/set_home ROS service)."""
        battery_gate("arm_set_home")
        with tool_errors():
            pose = robot.arm.set_home()
        return HomeSetResult(home=pose, path=str(config.arm.home_file))


# The one place tool modules are registered: a new module (e.g. perception tools) is one function taking a ToolContext
# and one entry here.
TOOL_MODULES: tuple[Callable[[ToolContext], None], ...] = (
    register_core_tools,
    body_tools.register,
    camera_tools.register,
    perception_tools.register,
    grasp_tools.register,
    motion_tools.register,
)


def build_app(
    server: MCPServer, config: McpServerConfig, robot: RobotApi | None = None, token: str | None = None
) -> Starlette:
    """Streamable HTTP ASGI app at config.server.path plus the admin route, both behind the bearer token.

    The admin route is plain HTTP, not an MCP tool, so the model cannot call it. It exists for claude_agent, which runs
    as another Linux user and cannot reach poi_store over DDS.

    Args:
        server (MCPServer): Server from build_mcp_server (its token verifier guards the MCP path).
        config (McpServerConfig): Node configuration.
        robot (RobotApi | None): Robot for the admin route; no admin route when None.
        token (str | None): Bearer token of the admin route; no admin route when None.

    Returns:
        Starlette: ASGI app for uvicorn.
    """
    app = server.streamable_http_app(streamable_http_path=config.server.path, host=config.server.host)
    if robot is not None and token:
        verifier = StaticTokenVerifier(token)

        async def clear_agent_pois(request: Request) -> JSONResponse:
            scheme, _, presented = request.headers.get("authorization", "").partition(" ")
            if scheme.lower() != "bearer" or await verifier.verify_token(presented.strip()) is None:
                return JSONResponse({"ok": False, "error": "unauthorized"}, status_code=401)
            try:
                result = await run_in_threadpool(robot.poi_clear, ADMIN_POI_CREATOR)
            except PoiStoreDown as exc:
                return JSONResponse({"ok": False, "error": str(exc)}, status_code=503)
            except PoiTimeout as exc:
                return JSONResponse({"ok": False, "error": str(exc)}, status_code=504)
            except RobotError as exc:
                return JSONResponse({"ok": False, "error": str(exc)}, status_code=502)
            return JSONResponse({"ok": True, "removed": int((result.get("poi") or {}).get("removed", 0))})

        app.add_route(ADMIN_CLEAR_AGENT_POIS_PATH, clear_agent_pois, methods=["POST"])
    return app
