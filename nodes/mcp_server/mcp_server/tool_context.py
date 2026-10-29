"""Shared context handed to every tool module (one place to register modules, see tools.TOOL_MODULES)."""

import logging
import threading
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Protocol

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from ros2_common.battery import BatteryGuard
from ros2_common.maps import MapResult, MapStatus

from .approach import CostGrid, PathInfo
from .arm import ArmController
from .base_dock import DockGoal, DockResult
from .base_motion import DriveOutcome, SpinOutcome
from .camera_scene import CameraInfoData
from .config import McpServerConfig
from .frontiers import FrontierTracker, MapGrid
from .head import HeadController
from .models import (
    BasePose,
    CameraFrame,
    DepthPoints,
    GroundState,
    Map3dCloud,
    MapSummary,
    NavigationResult,
    PathPlan,
    RobotState,
    ScanPoints,
    StopResult,
)
from .monitor import BaseUse, RobotMonitor
from .topdown import TopdownInputs

if TYPE_CHECKING:  # control imports motion_queue, which imports this module; lookup_tools imports this module
    from .control import ControlManager
    from .footprint import NavFootprint
    from .guard_rules import GuardEngine
    from .lookup_tools import LookupRecorder

LOGGER = logging.getLogger("mcp_server.tools")
# Tilt guard classification of the base motion tools: automatic navigation (Nav2 goals, autonomous turns) is refused
# while the guard is at warning or error; manual base moves only at error (base drive inhibited). enqueue_motions is
# refused when it carries a base step (base_relative / navigate_to_pose are Nav2 goals). Arm tools are unaffected.
TILT_NAV_TOOLS = frozenset(
    {
        "navigate_to_pose",
        "move_relative",
        "look_around",
        "go_to_frontier",
        "look_toward_frontier",
        "approach_for_grasp",
    }
)
TILT_MANUAL_BASE_TOOLS = frozenset({"drive"})


class PoiWriteCounter:
    """Camera and look_around results since the last agent POI write (poi_hint nudge, see tools.POI_HINT_TOOLS)."""

    def __init__(self) -> None:
        """Start at zero."""
        self.lock = threading.Lock()
        self.images = 0

    def note_image(self) -> int:
        """Count one camera or look_around result.

        Returns:
            int: Results counted since the last reset, this one included.
        """
        with self.lock:
            self.images += 1
            return self.images

    def reset(self) -> None:
        """A POI was written (add_poi, update_poi, remember_object, annotate_lookup): start counting again."""
        with self.lock:
            self.images = 0


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
    head: HeadController

    def robot_state(self) -> RobotState:
        """Robot snapshot."""
        ...

    def servo_released(self, group: str) -> bool | None:
        """Whether a servo group (arm, base, head) is released (limp) per the bridge torque_state; None unknown/stale."""
        ...

    def camera_image(self, camera: str, max_px: int) -> CameraFrame:
        """One fresh camera frame."""
        ...

    def camera_info(self, camera: str) -> CameraInfoData | None:
        """Latest CameraInfo of a camera with a camera_info_topic (subscribed on demand); None without one or before
        a first message."""
        ...

    def scan_points(self) -> ScanPoints | None:
        """Latest fresh lidar scan as (x, y, z, range) points in base_link; None when missing, stale or no TF."""
        ...

    def map_summary(self, include_png: bool, radius_m: float, png_max_px: int) -> tuple[MapSummary, bytes | None]:
        """Obstacle sectors, map stats and optional PNG crop."""
        ...

    def ground_state(self) -> tuple[GroundState, float] | None:
        """Latest /ground/state with its receive age (s); None when absent or stale."""
        ...

    def depth_points(self) -> DepthPoints | None:
        """Latest head depth (D435) cloud in base_link (TF at the cloud stamp); None when missing, stale or no TF."""
        ...

    def navigate(
        self, x: float, y: float, yaw: float, frame: str, timeout_s: float, precise: bool = False
    ) -> NavigationResult:
        """Blocking NavigateToPose (precise: wait for Nav2's tight goal checker, else end within the looser tolerance)."""
        ...

    def move_relative(
        self, dx: float, dy: float, dyaw: float, timeout_s: float, precise: bool = False
    ) -> NavigationResult:
        """Blocking move relative to base_link: a backing or sideways move of at most 0.5 m with dyaw 0 is one straight
        twist on cmd_vel_nav (no turn first); a turn (dyaw != 0, or a goal Nav2 starts with an in-place turn) is
        refused (RobotError) before any command when the scan has a return inside the rotation circle or no fresh
        scan; anything else is a NavigateToPose goal."""
        ...

    def dock(self, goal: DockGoal, timeout_s: float) -> DockResult:
        """Blocking DockRobot goal on the grasp docking server (same lock, stop and interrupts as navigate)."""
        ...

    def dock_ready(self) -> bool:
        """Whether the grasp docking action server answers (waits at most the action server wait)."""
        ...

    def plan_path(self, goal: BasePose) -> PathInfo:
        """ComputePathToPose from the robot's pose (planning only): ok with the length, no_path or unavailable."""
        ...

    def global_costmap(self) -> CostGrid | None:
        """Latest global costmap with its age (subscribes lazily), None when none was received."""
        ...

    def publish_detection(self, pose: BasePose, stamp_ns: int) -> None:
        """Publish a dock detection pose (map frame) stamped with the image capture time (ns)."""
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
        """Base half of stop (release_control base reset): cancel the Nav2 and docking goals this server sent and publish
        zero twist.

        Returns True when every cancel was answered."""
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

    def poi_list_age_s(self) -> float | None:
        """Seconds since /poi/list was last received; None when none arrived yet."""
        ...

    def poi_request(self, op: str, poi: dict[str, Any]) -> dict[str, Any]:
        """Send a /poi/command and wait for its /poi/result; RobotError when poi_store is down, slow or rejects."""
        ...

    def map3d_cloud(self) -> Map3dCloud | None:
        """Latest /map3d/cloud decoded (subscribed on demand; waits a bounded time for the first one); None when none arrived."""
        ...

    def map3d_status(self) -> dict[str, Any] | None:
        """Latest /map3d/status JSON object (subscribed on demand); None when none arrived or it was unreadable."""
        ...

    def map_status(self) -> MapStatus:
        """Latest /map_manager/status; RobotError when none arrived."""
        ...

    def map_request(self, command: dict[str, Any]) -> MapResult:
        """Send a /map_manager/command (MapCommand fields without request_id) and wait for its result; RobotError when map_manager is down, slow or rejects."""
        ...

    def poi_clear(
        self,
        created_by: str | None,
        statuses: list[str] | None = None,
        kinds: list[str] | None = None,
        keep_zones: bool = False,
    ) -> dict[str, Any]:
        """Clear POIs (creator None = every creator; only the given statuses/kinds when set; keep_zones keeps zones); the result's ``poi`` is {"removed": count}. RobotError as poi_request."""
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
    footprint: "NavFootprint | None" = None  # the published Nav2 footprint (None without nav_footprint config)
    poi_writes: PoiWriteCounter = field(default_factory=PoiWriteCounter)  # shared with RobotMCPServer

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
