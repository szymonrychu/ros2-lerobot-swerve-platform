"""Shared context handed to every tool module (one place to register modules, see tools.TOOL_MODULES)."""

import logging
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Protocol

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from ros2_common.battery import BatteryGuard

from .arm import ArmController
from .base_motion import DriveOutcome, SpinOutcome
from .config import McpServerConfig
from .frontiers import FrontierTracker, MapGrid
from .models import BasePose, CameraFrame, MapSummary, NavigationResult, PathPlan, RobotState, ScanPoints, StopResult
from .monitor import BaseUse, RobotMonitor
from .topdown import TopdownInputs

if TYPE_CHECKING:  # control imports motion_queue, which imports this module; lookup_tools imports this module
    from .control import ControlManager
    from .guard_rules import GuardEngine
    from .lookup_tools import LookupRecorder

LOGGER = logging.getLogger("mcp_server.tools")
# Tilt guard classification of the base motion tools: automatic navigation (Nav2 goals, autonomous turns) is refused
# while the guard is at warning or error; manual base moves only at error (base drive inhibited). enqueue_motions is
# refused when it carries a base step (base_relative / navigate_to_pose are Nav2 goals). Arm tools are unaffected.
TILT_NAV_TOOLS = frozenset(
    {"navigate_to_pose", "move_relative", "look_around", "go_to_frontier", "look_toward_frontier"}
)
TILT_MANUAL_BASE_TOOLS = frozenset({"drive"})


class MotionOwner(Protocol):
    """The motion queue as the other tools see it (motion_queue.MotionQueue)."""

    def busy(self) -> bool:
        """Whether queued motions run or are pending."""
        ...

    def halt_for_stop(self) -> list[str]:
        """Drop every pending step at once (stop tool); returns the dropped job ids."""
        ...


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

    def spin(
        self, wz: float, angle_rad: float, marks_rad: list[float], on_mark: Callable[[int], bool], timeout_s: float
    ) -> SpinOutcome:
        """Continuous in-place rotation (base motion lock, stop, critical events, timeout; zero at the end)."""
        ...

    def stop(self) -> StopResult:
        """Cancel navigation, zero the base, hold the arm only if this server controls it."""
        ...

    def reset_base(self) -> bool:
        """Base half of stop (release_control base reset): cancel every Nav2 goal and publish zero twist.

        Returns True when the Nav2 cancel service answered."""
        ...

    def steer_positions(self) -> dict[str, float] | None:
        """Fresh steer joint positions (rad) from the swerve joint states, None when missing or stale."""
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

    def map_grid(self) -> MapGrid | None:
        """Latest /map within frontiers.map_max_age_s in the map frame (version = stamp, size, origin); None otherwise."""
        ...

    def compute_path(self, x: float, y: float, yaw: float, timeout_s: float) -> PathPlan:
        """Ask Nav2 for a path from the current pose to a map goal (no motion); RobotError when the planner is down."""
        ...

    def nav_active(self) -> bool:
        """Whether a base motion runs or Nav2 has an accepted / executing goal."""
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
    queue: MotionOwner | None = None
    control: "ControlManager | None" = None
    lookups: "LookupRecorder | None" = None  # camera lookup recorder (lookup_tools.record_lookup)
    engine: "GuardEngine | None" = None  # guard rule engine (guard_tools.check_guard_rules)
    frontiers: FrontierTracker | None = None  # frontier tracker (frontier_tools); None without frontier tools

    def lease_gate(self, tool_name: str, kinds: list[str]) -> None:
        """Refuse an enqueue whose steps need a control lease that is not held, before the robot is touched.

        Args:
            tool_name (str): Tool being called (named in the refusal).
            kinds (list[str]): Step kinds (arm / gripper / grasp need the arm lease, Nav2 steps the base lease).
        """
        reason = None if self.control is None else self.control.steps_refusal(tool_name, kinds)
        if reason is not None:
            LOGGER.warning("%s", reason)
            raise ToolError(reason)

    def battery_gate(self, tool_name: str) -> None:
        """Refuse a motion tool while the battery is below cut-off, before the robot is touched.

        Args:
            tool_name (str): Tool being called (logged with the refusal).
        """
        if self.guard is not None and self.guard.is_cutoff():
            message = self.guard.rejection_message()
            LOGGER.warning("%s refused: %s", tool_name, message)
            raise ToolError(message)

    def tilt_gate(self, tool_name: str, kind: BaseUse | None = None) -> None:
        """Refuse a base motion tool the tilt guard does not allow now, before the robot is touched.

        Args:
            tool_name (str): Tool being called (named in the refusal).
            kind (BaseUse | None): 'nav' or 'manual'; None derives it from TILT_NAV_TOOLS / TILT_MANUAL_BASE_TOOLS.
        """
        use: BaseUse = kind or ("manual" if tool_name in TILT_MANUAL_BASE_TOOLS else "nav")
        reason = self.monitor.tilt_refusal(use)
        if reason is not None:
            LOGGER.warning("%s refused: %s", tool_name, reason)
            raise ToolError(f"{tool_name} refused: {reason}")
