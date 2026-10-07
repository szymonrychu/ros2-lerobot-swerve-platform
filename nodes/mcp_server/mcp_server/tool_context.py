"""Shared context handed to every tool module (one place to register modules, see tools.TOOL_MODULES)."""

from dataclasses import dataclass
from typing import Protocol

from mcp.server.mcpserver import MCPServer
from ros2_common.battery import BatteryGuard

from .arm import ArmController
from .base_motion import DriveOutcome
from .config import McpServerConfig
from .models import CameraFrame, MapSummary, NavigationResult, RobotState, StopResult
from .monitor import RobotMonitor


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
        """Cancel navigation, zero the base, hold the arm only if this server controls it."""
        ...


@dataclass(frozen=True)
class ToolContext:
    """Everything a tool module needs to register its tools on the server."""

    server: MCPServer
    robot: RobotApi
    config: McpServerConfig
    guard: BatteryGuard | None
    monitor: RobotMonitor
