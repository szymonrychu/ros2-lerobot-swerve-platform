"""MCP tool definitions (official MCP Python SDK 2.x MCPServer) and the bearer-token protected Streamable HTTP app."""

import base64
import hmac
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from typing import Annotated, Literal, Protocol

from mcp.server.auth.provider import AccessToken
from mcp.server.auth.settings import AuthSettings
from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import CallToolResult, ImageContent, TextContent
from pydantic import Field
from starlette.applications import Starlette

from .arm import ArmController, ArmError
from .base_motion import DriveError, DriveOutcome
from .config import HARD_MAX_DRIVE_S, HARD_MAX_IMAGE_PX, HARD_MAX_SPEED_SCALE, McpServerConfig
from .models import (
    ArmMotionResult,
    ArmState,
    CameraFrame,
    ControlResult,
    HomeSetResult,
    MapSummary,
    NavigationResult,
    RobotError,
    RobotState,
    StopResult,
)

SERVER_NAME = "robot"
TOKEN_CLIENT_ID = "robot-mcp-client"
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
)
INSTRUCTIONS = """Controls a swerve-drive mobile robot with an SO101 5-DOF arm and gripper (ROS 2 Jazzy, Nav2, SLAM).
Frames: 'map' is the SLAM map (x/y metres, yaw radians CCW); 'base_link' is the robot (x forward, y left).
Look before moving: call get_robot_state, get_map_summary and get_camera_image first. Prefer navigate_to_pose /
move_relative (Nav2 plans around obstacles) over drive. Arm motions are slow, clamped to joint limits and abort on
stale feedback or tracking error. Call stop at once if anything looks wrong; it is always available."""

Camera = Literal["gripper", "realsense"]


class RobotApi(Protocol):
    """Robot operations the tools call (implemented by ros_iface.RosRobot, faked in tests)."""

    arm: ArmController

    def robot_state(self) -> RobotState:
        """Robot snapshot."""
        ...

    def camera_image(self, camera: str, max_px: int) -> CameraFrame:
        """One fresh camera frame."""
        ...

    def map_summary(self, include_png: bool, radius_m: float, png_max_px: int) -> tuple[MapSummary, bytes | None]:
        """Obstacle sectors, map stats and optional PNG crop."""
        ...

    def navigate(self, x: float, y: float, yaw: float, frame: str, timeout_s: float) -> NavigationResult:
        """Blocking NavigateToPose."""
        ...

    def move_relative(self, dx: float, dy: float, dyaw: float, timeout_s: float) -> NavigationResult:
        """Blocking NavigateToPose relative to base_link."""
        ...

    def drive(self, vx: float, vy: float, wz: float, duration_s: float) -> DriveOutcome:
        """Timed velocity command."""
        ...

    def stop(self) -> StopResult:
        """Cancel navigation, zero the base, hold the arm."""
        ...


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


def build_mcp_server(robot: RobotApi, config: McpServerConfig, token: str) -> MCPServer:
    """Create the MCP server with every robot tool, protected by a static bearer token.

    Args:
        robot (RobotApi): Robot implementation.
        config (McpServerConfig): Node configuration.
        token (str): Bearer token clients must send.

    Returns:
        MCPServer: Configured server.
    """
    if not token:
        raise ValueError("a bearer token is required")
    auth = AuthSettings(
        issuer_url=f"http://{config.server.host}:{config.server.port}",
        resource_server_url=None,
        required_scopes=[],
    )
    server = MCPServer(SERVER_NAME, instructions=INSTRUCTIONS, token_verifier=StaticTokenVerifier(token), auth=auth)
    register_tools(server, robot, config)
    return server


def register_tools(server: MCPServer, robot: RobotApi, config: McpServerConfig) -> None:
    """Register all robot tools on a server.

    Args:
        server (MCPServer): Target server.
        robot (RobotApi): Robot implementation.
        config (McpServerConfig): Node configuration.
    """
    tool: Callable[..., Callable[[Callable[..., object]], Callable[..., object]]] = server.tool
    nav_default = config.timeouts.nav_default_timeout_s
    nav_max = config.timeouts.nav_max_timeout_s

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
        and listed in `notes`. Call this first and after every motion."""
        with tool_errors():
            return robot.robot_state()

    @tool(structured_output=False)
    def get_camera_image(
        camera: Annotated[Camera, Field(description="'gripper' (USB camera on the gripper) or 'realsense' (RGB-D)")],
        max_px: Annotated[int, Field(ge=32, le=HARD_MAX_IMAGE_PX, description="Longest image side in pixels")] = 768,
    ) -> list[ImageContent | TextContent]:
        """Take one fresh photo from a robot camera and return it as a JPEG image plus its capture timestamp.
        'gripper' looks out of the gripper (use it to aim grasps); 'realsense' is the forward RGB-D colour camera.
        Fails (instead of returning an old picture) when no frame arrives within the timeout or the newest frame is
        older than 1 s."""
        with tool_errors():
            frame = robot.camera_image(camera, min(max_px, config.limits.max_image_px))
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

    @tool()
    def navigate_to_pose(
        x: Annotated[float, Field(description="Goal x (m) in `frame`")],
        y: Annotated[float, Field(description="Goal y (m) in `frame`")],
        yaw: Annotated[float, Field(description="Goal heading (rad, CCW from +x)")] = 0.0,
        frame: Annotated[str, Field(description="Frame of the goal, normally 'map'")] = "map",
        timeout_s: Annotated[float | None, Field(gt=0.0, description="Give up after this many seconds")] = None,
    ) -> NavigationResult:
        """Drive the base to a pose with Nav2 (path planning, obstacle avoidance, velocity smoothing, collision
        monitor). Blocks until Nav2 reports a result or the timeout expires (the goal is then cancelled), and
        returns the result and the final pose. Use get_map_summary first to pick a reachable free-space goal."""
        with tool_errors():
            return robot.navigate(x, y, yaw, frame, nav_timeout(timeout_s))

    @tool()
    def move_relative(
        dx: Annotated[float, Field(description="Forward displacement (m), negative = backwards")],
        dy: Annotated[float, Field(description="Leftward displacement (m), negative = right")] = 0.0,
        dyaw: Annotated[float, Field(description="Heading change (rad), positive = turn left")] = 0.0,
        timeout_s: Annotated[float | None, Field(gt=0.0, description="Give up after this many seconds")] = None,
    ) -> NavigationResult:
        """Move relative to the robot's current pose (base_link frame) through Nav2, e.g. dx=0.5 drives half a
        metre forward, dyaw=1.57 turns left 90 degrees. Same blocking/obstacle-avoiding behaviour as
        navigate_to_pose."""
        with tool_errors():
            return robot.move_relative(dx, dy, dyaw, nav_timeout(timeout_s))

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
        with tool_errors():
            return robot.drive(vx, vy, wz, duration_s)

    @tool()
    def stop() -> StopResult:
        """Emergency stop, always available: cancels every Nav2 navigation goal, publishes a zero velocity and
        freezes the arm at its measured pose (aborting any arm motion). Call it whenever anything looks wrong."""
        with tool_errors():
            return robot.stop()

    @tool()
    def get_arm_state() -> ArmState:
        """Arm joint positions (rad) and efforts, gripper effort, gripper tool point pose (x, y, z, pitch in the
        arm base_link), the filter_node active source, whether this server holds the autonomy lease, whether a home
        pose is stored, and the joint_states age. Positions are omitted when the feedback is stale."""
        with tool_errors():
            return robot.arm.state()

    @tool()
    def acquire_control() -> ControlResult:
        """Take arm control (autonomy lease) by commanding the current measured pose, so the arm does not move.
        Motion tools acquire implicitly; the leader arm or web UI can take over again, which ends the lease."""
        with tool_errors():
            return robot.arm.acquire()

    @tool()
    def release_control() -> ControlResult:
        """Give arm control back to the other sources (leader arm / web UI) and stop streaming setpoints."""
        with tool_errors():
            return robot.arm.release()

    @tool()
    def move_arm_joints(
        targets: Annotated[
            dict[str, float],
            Field(
                description="Joint name -> target rad; joints: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, "
                "wrist_roll, gripper. Unnamed joints stay where they are."
            ),
        ],
        speed_scale: Annotated[
            float,
            Field(gt=0.0, le=HARD_MAX_SPEED_SCALE, description="0.5 = max joint speed (0.5 rad/s); lower is slower"),
        ] = HARD_MAX_SPEED_SCALE,
    ) -> ArmMotionResult:
        """Move arm joints to targets along a smooth (quintic) trajectory streamed at 25 Hz. Targets are clamped
        to the URDF limits minus a margin (reported in `clamped`). Blocks until converged or timed out; aborts and
        holds the measured pose if joint feedback goes stale (>0.3 s), the tracking error grows too large, or stop
        is called."""
        with tool_errors():
            return robot.arm.move_joints(targets, speed_scale)

    @tool()
    def move_arm_cartesian(
        x: Annotated[float, Field(description="Gripper tool point x (m), forward of the arm base")],
        y: Annotated[float, Field(description="Tool point y (m), left of the arm base")],
        z: Annotated[float, Field(description="Tool point z (m), above the arm base")],
        pitch: Annotated[
            float | None, Field(description="Approach pitch (rad): 0 horizontal, +1.57 pointing straight down")
        ] = None,
        frame: Annotated[Literal["base_link"], Field(description="Arm URDF base_link (the arm mount)")] = "base_link",
        speed_scale: Annotated[float, Field(gt=0.0, le=HARD_MAX_SPEED_SCALE)] = HARD_MAX_SPEED_SCALE,
    ) -> ArmMotionResult:
        """Move the gripper tool point to (x, y, z) in the arm's base_link frame (arm URDF root: x forward along
        the arm at shoulder_pan=0, z up), optionally with an approach pitch. Solves inverse kinematics on the arm
        URDF (5-DOF: position + pitch, wrist_roll kept) and streams the joint motion like move_arm_joints. Returns
        status 'unreachable' without moving when no solution exists within joint limits."""
        del frame  # the only supported frame
        with tool_errors():
            return robot.arm.move_cartesian(x, y, z, pitch, speed_scale)

    @tool()
    def set_gripper(
        open_fraction: Annotated[float | None, Field(ge=0.0, le=1.0, description="0 = closed, 1 = fully open")] = None,
        close_until_effort: Annotated[
            bool, Field(description="Close slowly and stop as soon as the gripper feels an object")
        ] = False,
        effort_threshold: Annotated[
            float | None, Field(gt=0.0, description="Gripper load counted as contact (servo load units)")
        ] = None,
    ) -> ArmMotionResult:
        """Open the gripper to a fraction, or close it until it grips something (status 'grasped' and the gripper
        holds that position; 'closed_no_contact' if it closed fully without touching anything). Give exactly one of
        open_fraction or close_until_effort=true."""
        with tool_errors():
            return robot.arm.set_gripper(open_fraction, close_until_effort, effort_threshold)

    @tool()
    def arm_home() -> ArmMotionResult:
        """Move the arm to its stored home pose (same as the /arm/home ROS service). Fails if no home pose has been
        stored yet with arm_set_home."""
        with tool_errors():
            return robot.arm.home()

    @tool()
    def arm_set_home() -> HomeSetResult:
        """Store the arm's current measured pose as the home pose (same as the /arm/set_home ROS service)."""
        with tool_errors():
            pose = robot.arm.set_home()
        return HomeSetResult(home=pose, path=str(config.arm.home_file))


def build_app(server: MCPServer, config: McpServerConfig) -> Starlette:
    """Streamable HTTP ASGI app at config.server.path (bearer auth enforced by the server's token verifier).

    Args:
        server (MCPServer): Server from build_mcp_server.
        config (McpServerConfig): Node configuration.

    Returns:
        Starlette: ASGI app for uvicorn.
    """
    return server.streamable_http_app(streamable_http_path=config.server.path, host=config.server.host)
