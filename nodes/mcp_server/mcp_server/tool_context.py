"""Shared context handed to every tool module (one place to register modules, see tools.TOOL_MODULES)."""

import logging
from dataclasses import dataclass
from typing import Any, Protocol

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from ros2_common.battery import BatteryGuard

from .arm import ArmController
from .base_motion import DriveOutcome
from .config import McpServerConfig
from .models import BasePose, CameraFrame, MapSummary, NavigationResult, RobotState, ScanPoints, StopResult
from .monitor import RobotMonitor
from .topdown import TopdownInputs

LOGGER = logging.getLogger("mcp_server.tools")


class RobotApi(Protocol):
    """Robot operations the tools call (implemented by ros_iface.RosRobot, faked in tests)."""

    arm: ArmController

    def robot_state(self) -> RobotState:
        """Robot snapshot."""
        ...

    def camera_image(self, camera: str, max_px: int) -> CameraFrame:
        """One fresh camera frame."""
        ...

    def scan_points(self) -> ScanPoints | None:
        """Latest fresh lidar scan as (x, y, z, range) points in base_link; None when missing, stale or no TF."""
        ...

    def map_summary(self, include_png: bool, radius_m: float, png_max_px: int) -> tuple[MapSummary, bytes | None]:
        """Obstacle sectors, map stats and optional PNG crop."""
        ...

    def navigate(
        self, x: float, y: float, yaw: float, frame: str, timeout_s: float, precise: bool = False
    ) -> NavigationResult:
        """Blocking NavigateToPose (precise: wait for Nav2's tight goal checker, else end within the looser tolerance)."""
        ...

    def move_relative(
        self, dx: float, dy: float, dyaw: float, timeout_s: float, precise: bool = False
    ) -> NavigationResult:
        """Blocking NavigateToPose relative to base_link."""
        ...

    def drive(self, vx: float, vy: float, wz: float, duration_s: float) -> DriveOutcome:
        """Timed velocity command."""
        ...

    def stop(self) -> StopResult:
        """Cancel navigation, zero the base, hold the arm only if this server controls it."""
        ...

    def robot_pose(self) -> BasePose | None:
        """Fresh map -> base_link pose, None when missing or stale."""
        ...

    def stop_count(self) -> int:
        """Number of stop() calls so far (lets a multi-step tool notice a stop issued between its motions)."""
        ...

    def event_seq(self) -> int:
        """Sequence number of the newest robot event (mark for interrupt_since)."""
        ...

    def interrupt_since(self, seq: int) -> str | None:
        """Type of a critical event that interrupts a base motion, raised after `seq`; None when there is none."""
        ...

    def topdown_inputs(self) -> TopdownInputs:
        """Snapshot of the map, local costmap, lidar points, plan and pose for the top-down view."""
        ...

    def poi_list(self) -> tuple[list[dict[str, Any]], int]:
        """Latest /poi/list (POIs, revision); RobotError when poi_store never published."""
        ...

    def poi_request(self, op: str, poi: dict[str, Any]) -> dict[str, Any]:
        """Send a /poi/command and wait for its /poi/result; RobotError when poi_store is down, slow or rejects."""
        ...

    def poi_clear(self, created_by: str) -> dict[str, Any]:
        """Clear every POI of one creator; the result's ``poi`` is {"removed": count}. RobotError as poi_request."""
        ...


@dataclass(frozen=True)
class ToolContext:
    """Everything a tool module needs to register its tools on the server."""

    server: MCPServer
    robot: RobotApi
    config: McpServerConfig
    guard: BatteryGuard | None
    monitor: RobotMonitor

    def battery_gate(self, tool_name: str) -> None:
        """Refuse a motion tool while the battery is below cut-off, before the robot is touched.

        Args:
            tool_name (str): Tool being called (logged with the refusal).
        """
        if self.guard is not None and self.guard.is_cutoff():
            message = self.guard.rejection_message()
            LOGGER.warning("%s refused: %s", tool_name, message)
            raise ToolError(message)
