"""Per-subsystem control leases: take_control / release_control / get_control_state and the lease check.

The arm lease is the filter_node autonomy lease (ArmController.acquire / release): the hardware arbitration honours
it. The agent's lease is the ArmController lease generation recorded at take_control: an implicit hold of the arm by
mcp_server (the web UI grasp service, a stop during a foreign motion) is not the agent's lease, and a tool bound to a
lease (lease_scope, arm.EXPECTED_LEASE) is refused as soon as that lease ended or changed hands instead of re-taking
the arm. The base lease is a flag inside mcp_server (nothing downstream arbitrates /cmd_vel_nav; humans can still
drive). Leases are global to the server, not per MCP session. Every arm or base tool is refused without its lease
(tools.TOOL_SUBSYSTEM, checked centrally before the tool body). Releasing resets by default: the arm drives to its
stored home pose, the base is zeroed (only this server's Nav2 goals cancelled, zero twist) and the steering is verified
neutral. A failed reset is reported, but the release happens (except a park stopped by the stop tool: the arm then
holds and the lease is kept).

The web UI takes the same lease through take_web_ui / release_web_ui (std_srvs/Trigger /arm/take_control and
/arm/release_control, holder web_ui): refused while the agent holds the arm, under battery cut-off and while the leader
arm moves (LeaderActivity).

The navigation footprint (footprint.NavFootprint) follows the arm lease: take_control('arm') publishes the static
body footprint (the arm may move from then on), an arm release that parked the arm publishes it too (as does a
converged /arm/home service call), and any other arm release (keep_position=true, or a park that failed) publishes the
footprint of the measured, now frozen arm pose, with the carried object circle when carried_object_radius_m is given.

Carrying is one signal (ControlState.arm.carrying, carrying_state): the persisted carry record (carry.py: the last
close held an object, grasped or unverified, and the carried radius of the last keep release, 0 meaning none), cleared
when the measured jaw is wide open, plus a live holding load on the gripper. Parking the arm is refused while it
carries (subsystem='all' still releases the base and reports reset.arm 'refused: carrying (...)'), and a keep_position
release while carrying keeps the grip profile's torque limit.

The pan/tilt head (head.HeadController) belongs to both leases: the head tools need either one (LeaseNeed "any"). A
release_control decides the head last, after the base and arm resets, and reports it as reset.head (only when a head
is configured): keep_position=true keeps it ('kept'); a release that leaves the agent holding the other lease does not
move it ('kept: <arm|base> lease held', the hand-over); a call that released no lease adds no head entry and moves
nothing; otherwise (the call released a lease and none remains) the head goes home ('homed', 'failed: <status>', or
'skipped: battery_cutoff' under battery cut-off). A head failure never blocks the arm/base release. A lease lost
involuntarily (web UI take, generation change) and the stop tool never move the head.
"""

import threading
import time
from collections import deque
from collections.abc import Callable, Iterable, Iterator
from contextlib import AbstractContextManager, contextmanager
from typing import Literal, Protocol

from mcp.server.mcpserver.exceptions import ToolError
from ros2_common.battery import BatteryGuard

from .arm import EXPECTED_LEASE, SOURCE_NONE, ArmController, ArmError, finite_effort
from .carry import CarryRecord
from .config import McpServerConfig
from .footprint import NavFootprint
from .head import HeadController
from .models import ArmControlState, BaseControlState, CarryingState, ControlState, ReleaseResult
from .motion_queue import ARM_KINDS, BASE_KINDS, LiveState, Precondition, evaluate_precondition

Subsystem = Literal["arm", "base"]
ReleaseTarget = Literal["arm", "base", "all"]
# A tool's lease need: one subsystem, or "any" (either lease satisfies it: the head tools, so a hand-over from one
# lease to the other never moves the head; ControlManager.release_head homes it once no lease remains).
LeaseNeed = Literal["arm", "base", "any"]
# enqueue_motions step kinds that need the arm lease; base steps (Nav2 goals) are motion_queue.BASE_KINDS.
ARM_LEASE_KINDS = ARM_KINDS | {"gripper", "grasp"}
SUBSYSTEMS: tuple[Subsystem, ...] = ("arm", "base")
RELEASE_ORDER: tuple[Subsystem, ...] = ("base", "arm")  # base first: stop the wheels before the arm moves home
QUEUE_BUSY_REFUSAL = "release_control refused: the motion queue is running - cancel_motions or wait_for_event first"
MOTION_BUSY_REFUSAL = "release_control refused: {what} - call stop first or wait for it to finish"
RELEASE_TOOL = "release_control"  # counts as a blocking tool itself while it parks
NOT_HELD = "skipped: not held"
# A stop during the park: the arm holds where it stopped and the lease is kept (not handed to the leader / web UI).
STOPPED_PARK = "stopped: the park was stopped; the arm holds where it stopped and the lease is kept"
# Expected lease generation of a scope whose arm lease ended before it started (never a real generation).
LOST_LEASE = -1
# Arm/base outcomes after which the lease is still held (or was never held): not counted as released.
KEPT_PREFIXES = (NOT_HELD, "stopped:", "refused:")
# carried_object_radius_m range (m): the rough size of the object in the gripper.
MAX_CARRIED_RADIUS_M = 0.3
RADIUS_NEEDS_KEEP = (
    "release_control refused: carried_object_radius_m is only valid with keep_position=true (the arm cannot park "
    "with a carried object)"
)
RADIUS_NEEDS_ARM = "release_control refused: carried_object_radius_m applies to subsystem='arm' or 'all'"
RADIUS_NEEDS_LEASE = "release_control refused: carried_object_radius_m needs the arm lease held (the footprint is set at the arm release)"
CARRYING_REFUSAL = (
    "release_control refused: the arm cannot park while the gripper carries an object ({reasons}) - put the object "
    "down first (release_object), or release with keep_position=true"
)


WEB_UI_HOLDER = "web_ui"  # ControlState.arm.held_by of a lease taken by the web UI (/arm/take_control)
WEB_UI_NOT_HELD = "arm control not held by web_ui"
WEB_UI_AGENT_REFUSAL = "take_control refused: the agent holds the arm - wait for it to release or stop it first"
WEB_UI_BATTERY_REFUSAL = "take_control refused: battery below cut-off"
WEB_UI_LEADER_REFUSAL = "take_control refused: the leader arm is moving - hold it still and try again"


class LeaderActivity:
    """Recent leader arm joint motion (the master2master output feeding filter_node), for the web UI take refusal."""

    def __init__(self, threshold_rad: float, window_s: float) -> None:
        """Create the tracker.

        Args:
            threshold_rad (float): A joint whose position spread in the window exceeds this counts as moving (rad).
            window_s (float): Look-back window (s).
        """
        self.threshold_rad = threshold_rad
        self.window_s = window_s
        self.samples: deque[tuple[float, dict[str, float]]] = deque()
        self.lock = threading.Lock()

    def feed(self, positions: dict[str, float], now: float) -> None:
        """Record one leader sample and drop the ones older than the window.

        Args:
            positions (dict[str, float]): Joint name -> position (rad).
            now (float): Sample time (s, monotonic).
        """
        with self.lock:
            self.samples.append((now, dict(positions)))
            self.trim(now)

    def trim(self, now: float) -> None:
        """Drop samples older than the window (caller holds the lock).

        Args:
            now (float): Current time (s, monotonic).
        """
        while self.samples and now - self.samples[0][0] > self.window_s:
            self.samples.popleft()

    def moving(self, now: float) -> bool:
        """Whether any leader joint moved more than the threshold within the window.

        Args:
            now (float): Current time (s, monotonic).

        Returns:
            bool: True when the leader is actively moving.
        """
        with self.lock:
            self.trim(now)
            spread: dict[str, tuple[float, float]] = {}
            for _, positions in self.samples:
                for joint, value in positions.items():
                    low, high = spread.get(joint, (value, value))
                    spread[joint] = (min(low, value), max(high, value))
        return any(high - low > self.threshold_rad for low, high in spread.values())


def grasp_reason(record: CarryRecord) -> str:
    """Carrying reason of a held grasp: its verification state (pending confirm_grasp, the fused outcome) or kind.

    Args:
        record (CarryRecord): Carry record with grasp_held set.

    Returns:
        str: The reason.
    """
    if record.image == "pending":
        return (
            f"the last grasp is an unverified hold awaiting confirm_grasp (grasp_id {record.grasp_id}, band "
            f"{record.band})"
        )
    if record.fused == "grasped":
        return f"the last grasp is held (fused grasped: band {record.band}, image {record.image})"
    if record.fused is not None:
        return f"the last grasp is an unverified hold (fused {record.fused}: band {record.band}, image {record.image})"
    if record.grasp_kind == "unverified":
        return "the last grasp is an unverified hold (closed into the empty band, kept gripping)"
    return "the last grasp is held (the gripper was not opened since)"


class BaseOps(Protocol):
    """Base operations the base reset needs (implemented by ros_iface.RosRobot, faked in tests)."""

    def reset_base(self) -> bool:
        """Cancel every Nav2 goal and publish zero twist; True when the Nav2 cancel service answered."""
        ...

    def steer_positions(self) -> dict[str, float] | None:
        """Fresh steer joint positions (rad) from the swerve joint states, None when missing or stale."""
        ...


def steering_neutral(positions: dict[str, float] | None, joints: Iterable[str], tolerance_rad: float) -> bool | None:
    """Whether every steer joint is within the tolerance of straight ahead (0 rad).

    Args:
        positions (dict[str, float] | None): Steer joint positions (rad), None when unknown.
        joints (Iterable[str]): Steer joint names that must all be present.
        tolerance_rad (float): Largest |angle| counted as neutral (rad).

    Returns:
        bool | None: True / False, None when positions are unknown or a joint is missing.
    """
    names = list(joints)
    if positions is None or any(j not in positions for j in names):
        return None
    return all(abs(positions[j]) <= tolerance_rad for j in names)


def wait_steering_neutral(
    read: Callable[[], dict[str, float] | None],
    joints: Iterable[str],
    tolerance_rad: float,
    timeout_s: float,
    clock: Callable[[], float],
    sleep: Callable[[float], None],
    poll_s: float,
) -> tuple[bool, str]:
    """Poll the steer joints until all are neutral or the timeout passes.

    Args:
        read (Callable[[], dict[str, float] | None]): Fresh steer positions (rad), None when missing or stale.
        joints (Iterable[str]): Steer joint names.
        tolerance_rad (float): Largest |angle| counted as neutral (rad).
        timeout_s (float): Longest wait (s).
        clock (Callable[[], float]): Monotonic clock (s).
        sleep (Callable[[float], None]): Sleep (s).
        poll_s (float): Check period (s).

    Returns:
        tuple[bool, str]: (True, "") when neutral, else (False, why).
    """
    names = list(joints)
    deadline = clock() + timeout_s
    while True:
        positions = read()
        if steering_neutral(positions, names, tolerance_rad):
            return True, ""
        if clock() >= deadline:
            break
        sleep(poll_s)
    if positions is None or any(j not in positions for j in names):
        return False, f"no fresh steer joint states ({', '.join(names)}) within {timeout_s:g} s"
    worst = max(names, key=lambda j: abs(positions[j]))
    return False, (
        f"steering not neutral within {timeout_s:g} s ({worst} {positions[worst]:.2f} rad, tolerance "
        f"{tolerance_rad:g} rad)"
    )


class NavGoalRegistry:
    """Nav2 goals this server sent (run_nav), so the base reset cancels only its own goals, never a human's web UI goal.

    Handles are opaque (rclpy ClientGoalHandle in ros_iface); a goal is registered when Nav2 accepts it and discarded
    when its result arrives.
    """

    def __init__(self) -> None:
        """Create an empty registry."""
        self.lock = threading.Lock()
        self.handles: list[object] = []

    def add(self, handle: object) -> None:
        """Register an accepted goal.

        Args:
            handle (object): Goal handle.
        """
        with self.lock:
            self.handles.append(handle)

    def discard(self, handle: object) -> None:
        """Forget a finished goal (no error when unknown).

        Args:
            handle (object): Goal handle.
        """
        with self.lock:
            self.handles = [h for h in self.handles if h is not handle]

    def cancel(self, cancel_one: Callable[[object], bool]) -> bool:
        """Cancel every registered goal and forget them.

        Args:
            cancel_one (Callable[[object], bool]): Cancels one goal; True when Nav2 answered the cancel.

        Returns:
            bool: True when every cancel was answered (also when there was nothing to cancel).
        """
        with self.lock:
            handles, self.handles = self.handles, []
        return all([cancel_one(h) for h in handles])


class ControlManager:
    """Both control leases: take, release (with the reset), state, the lease checks and the vitals digest.

    The agent's arm lease is the ArmController lease generation recorded at take_control: an implicit hold of the arm
    (the web UI grasp service, a stop during a foreign motion) is not this agent's lease, and a lease that ended or
    changed hands (a new generation) is not held any more.
    """

    def __init__(
        self,
        arm: ArmController,
        base: BaseOps,
        config: McpServerConfig,
        guard: BatteryGuard | None = None,
        queue_busy: Callable[[], bool] = lambda: False,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        wall: Callable[[], float] = time.time,
        footprint: NavFootprint | None = None,
        blocking_tool: Callable[[], str | None] = lambda: None,
        head: HeadController | None = None,
    ) -> None:
        """Create the manager.

        Args:
            arm (ArmController): Arm controller (owns the arm lease and the carry record).
            base (BaseOps): Base zeroing and steer joint feed.
            config (McpServerConfig): Node configuration (control section).
            guard (BatteryGuard | None): Battery guard; under cut-off the arm reset is skipped.
            queue_busy (Callable[[], bool]): Whether the motion queue runs (release is refused meanwhile).
            clock (Callable[[], float]): Monotonic clock for the steering wait (s).
            sleep (Callable[[float], None]): Sleep for the steering wait (s).
            wall (Callable[[], float]): Wall clock for held_since (s since the epoch).
            footprint (NavFootprint | None): Published navigation footprint; None publishes nothing.
            blocking_tool (Callable[[], str | None]): Name of a blocking motion tool running now (RobotMCPServer
                .blocking_tool); release is refused while one other than release_control runs.
            head (HeadController | None): Pan/tilt head, homed by a release that leaves no lease (release_head); None
                when no head is configured (no reset.head entry).
        """
        self.arm = arm
        self.config = config
        self.footprint = footprint
        self.base = base
        self.cfg = config.control
        self.guard = guard
        self.queue_busy = queue_busy
        self.blocking_tool = blocking_tool
        self.head = head
        self.clock = clock
        self.sleep = sleep
        self.wall = wall
        self.lock = threading.Lock()
        self.base_since: float | None = None  # wall time the base lease was taken, None while not held
        # Lease generation of the agent's arm lease (ArmController.lease_generation at take_control) and when it was
        # taken; None while the agent does not hold the arm.
        self.arm_generation: int | None = None
        self.arm_since: float | None = None
        # Lease generation of the web UI's arm lease (take_web_ui); None while the web UI does not hold the arm.
        self.web_ui_generation: int | None = None
        # Whether the leader arm is moving now (ros_iface binds LeaderActivity.moving); take_web_ui refuses then.
        self.leader_moving: Callable[[], bool] = lambda: False

    @property
    def web_ui_held(self) -> bool:
        """Whether the web UI holds the arm lease: taken with take_web_ui and not ended or re-taken since.

        Returns:
            bool: True while held (a stale record is cleared).
        """
        generation = self.web_ui_generation
        if generation is None:
            return False
        if self.arm.control_held and self.arm.lease_generation == generation:
            return True
        with self.lock:
            if self.web_ui_generation == generation:
                self.web_ui_generation = None
        return False

    def take_web_ui(self) -> tuple[bool, str]:
        """/arm/take_control: the web UI takes the arm (lease holder web_ui, commands the measured pose, moves nothing).

        Returns:
            tuple[bool, str]: success and message; refused while the agent holds the arm, under battery cut-off and
                while the leader arm is moving.
        """
        if self.arm_held:
            return False, WEB_UI_AGENT_REFUSAL
        if self.guard is not None and self.guard.is_cutoff():
            return False, WEB_UI_BATTERY_REFUSAL
        if self.web_ui_held:
            return True, "arm control already held by web_ui"
        if self.leader_moving():
            return False, WEB_UI_LEADER_REFUSAL
        try:
            if not self.arm.control_held:
                self.arm.acquire()
        except ArmError as exc:
            return False, f"take_control refused: {exc}"
        with self.lock:
            self.web_ui_generation = self.arm.lease_generation
        if self.footprint is not None:
            self.footprint.publish_static()
        return True, "arm control taken by web_ui at the measured pose"

    def release_web_ui(self) -> tuple[bool, str]:
        """/arm/release_control: publish the autonomy release so the follower hands back to the leader (filter_node's
        proximity rule applies) and clear the holder. The arm stays where it is.

        Returns:
            tuple[bool, str]: success and message; refused when the web UI does not hold the arm.
        """
        if not self.web_ui_held:
            return False, WEB_UI_NOT_HELD
        self.arm.release()
        with self.lock:
            self.web_ui_generation = None
        self.publish_footprint("kept", None)
        return True, "arm control released by web_ui; the leader takes over once it is close to the arm"

    @property
    def base_held(self) -> bool:
        """Whether the base lease is held.

        Returns:
            bool: True while held.
        """
        return self.base_since is not None

    @property
    def arm_held(self) -> bool:
        """Whether the agent holds the arm lease: taken with take_control and not ended or re-taken since.

        Returns:
            bool: True while held (a stale record is cleared).
        """
        generation = self.arm_generation
        if generation is None:
            return False
        if self.arm.control_held and self.arm.lease_generation == generation:
            return True
        with self.lock:
            if self.arm_generation == generation:
                self.arm_generation = None
                self.arm_since = None
        return False

    def held(self, subsystem: Subsystem) -> bool:
        """Whether one lease is held by the agent.

        Args:
            subsystem (Subsystem): arm or base.

        Returns:
            bool: True while held.
        """
        return self.arm_held if subsystem == "arm" else self.base_held

    def take(self, subsystem: Subsystem) -> ControlState:
        """Take one lease; a no-op (keeping held_since) when it is already held. Moves nothing.

        Args:
            subsystem (Subsystem): arm (commands the measured pose, needs fresh joint states; an arm held implicitly
                by mcp_server is adopted as is) or base.

        Returns:
            ControlState: Both leases afterwards.
        """
        if subsystem == "arm":
            if not self.arm_held:
                if not self.arm.control_held:
                    self.arm.acquire()
                with self.lock:
                    self.arm_generation = self.arm.lease_generation
                    self.arm_since = self.wall()
                    self.web_ui_generation = None  # the agent adopts the lease: the web UI no longer holds it
            if self.footprint is not None:
                self.footprint.publish_static()
        else:
            with self.lock:
                if self.base_since is None:
                    self.base_since = self.wall()
        return self.state()

    @contextmanager
    def lease_scope(self, need: LeaseNeed | None) -> Iterator[None]:
        """Run a tool body (or queued step) bound to the arm lease it was checked against: an arm motion inside it is
        refused once that lease ended or changed hands (arm.EXPECTED_LEASE), instead of re-taking the arm.

        Args:
            need (LeaseNeed | None): The tool's lease need; only 'arm' binds the arm lease.

        Yields:
            None: Runs the body.
        """
        generation = self.arm_generation if self.arm_held else None
        # Lost between the lease check and here: an impossible generation, so no motion re-takes the arm.
        token = EXPECTED_LEASE.set(LOST_LEASE if generation is None else generation) if need == "arm" else None
        try:
            yield
        finally:
            if token is not None:
                EXPECTED_LEASE.reset(token)

    def step_scope(self, kind: str) -> AbstractContextManager[None]:
        """lease_scope for one queued step kind (motion_queue dispatch).

        Args:
            kind (str): Step kind.

        Returns:
            AbstractContextManager[None]: The scope.
        """
        return self.lease_scope("arm" if "arm" in self.step_needs([kind]) else None)

    def release(
        self, subsystem: ReleaseTarget, keep_position: bool = False, carried_object_radius_m: float | None = None
    ) -> ReleaseResult:
        """Release leases, resetting each by default. The release always happens; a failed reset is reported.

        Args:
            subsystem (ReleaseTarget): arm, base or all.
            keep_position (bool): True skips the reset (arm keeps its last goal, base only clears the flag).
            carried_object_radius_m (float | None): keep_position=true arm release only: radius (0..0.3 m) of the
                carried object, a circle around the tool point added to the footprint (0 means none).

        Returns:
            ReleaseResult: Released subsystems, per subsystem reset outcome and the state afterwards. With
                subsystem='all' while carrying, the base is still released and reset.arm says 'refused: carrying (...)'.
                reset.head (head configured, after the arm/base resets) is decided by release_head.

        Raises:
            ToolError: While the motion queue or another blocking motion tool runs, while an arm motion runs (arm
                targeted), for an invalid carried_object_radius_m, or when subsystem='arm' would park while carrying
                an object (nothing is released).
        """
        targets = RELEASE_ORDER if subsystem == "all" else (subsystem,)
        if carried_object_radius_m is not None:
            self.check_carried_radius(carried_object_radius_m, keep_position, "arm" in targets)
            carried_object_radius_m = carried_object_radius_m or None  # 0 = no carried object
        if self.queue_busy():
            raise ToolError(QUEUE_BUSY_REFUSAL)
        other = self.blocking_tool()
        if other is not None and other != RELEASE_TOOL:
            raise ToolError(MOTION_BUSY_REFUSAL.format(what=f"{other} is running"))
        if "arm" in targets and self.arm_held and self.arm.motion_running:
            raise ToolError(MOTION_BUSY_REFUSAL.format(what="an arm motion is running"))
        carry_refusal: str | None = None
        if not keep_position and "arm" in targets and self.arm_held:
            reasons = self.carrying_state().reasons
            if reasons:
                if subsystem != "all":
                    raise ToolError(CARRYING_REFUSAL.format(reasons="; ".join(reasons)))
                carry_refusal = f"refused: carrying ({'; '.join(reasons)})"
        released: list[str] = []
        reset: dict[str, str] = {}
        for name in targets:
            if name == "base":
                outcome = self.release_base(keep_position)
            elif carry_refusal is not None:
                outcome = carry_refusal
            else:
                outcome = self.release_arm(keep_position, carried_object_radius_m)
            reset[name] = outcome
            if not outcome.startswith(KEPT_PREFIXES):
                released.append(name)
        head = self.release_head(keep_position, bool(released))
        if head is not None:
            reset["head"] = head
        return ReleaseResult(released=released, reset=reset, state=self.state())

    def release_head(self, keep_position: bool, released_any: bool) -> str | None:
        """Head outcome of a release_control call, decided after the arm/base resets (the releases already happened,
        so a head failure never blocks them).

        Args:
            keep_position (bool): The release keeps positions (the head is kept too).
            released_any (bool): The call released at least one lease.

        Returns:
            str | None: None without a head or when nothing was released (no motion); else kept | kept: <arm|base>
                lease held | skipped: battery_cutoff | homed | failed: <status or error>.
        """
        if self.head is None:
            return None
        if keep_position:
            return "kept"
        if not released_any:
            return None
        for name in SUBSYSTEMS:
            if self.held(name):
                return f"kept: {name} lease held"
        if self.guard is not None and self.guard.is_cutoff():
            return "skipped: battery_cutoff"
        try:
            result = self.head.home()
        except RuntimeError as exc:  # HeadError, RobotError: report it, the leases are already released
            return f"failed: {exc}"
        return "homed" if result.status == "converged" else f"failed: {result.status}"

    def check_carried_radius(self, radius: float, keep_position: bool, arm_targeted: bool) -> None:
        """Validate carried_object_radius_m for a release.

        Args:
            radius (float): Radius (m).
            keep_position (bool): The release keeps the position.
            arm_targeted (bool): The release includes the arm.

        Raises:
            ToolError: Without keep_position, without the arm (target or lease), or outside 0..0.3 m.
        """
        if not keep_position:
            raise ToolError(RADIUS_NEEDS_KEEP)
        if not arm_targeted:
            raise ToolError(RADIUS_NEEDS_ARM)
        if not 0.0 <= radius <= MAX_CARRIED_RADIUS_M:
            raise ToolError(
                f"release_control refused: carried_object_radius_m must be within 0..{MAX_CARRIED_RADIUS_M:g} m "
                f"(got {radius:g})"
            )
        if not self.arm_held:
            raise ToolError(RADIUS_NEEDS_LEASE)

    def carrying_state(self) -> CarryingState:
        """The one carrying signal: the carry record (radius of the last keep release, the last close held an object)
        unless the measured jaw is wide open (then the record is cleared), plus a live holding load on the gripper.

        Returns:
            CarryingState: carrying, reasons and the carried radius.
        """
        sample = self.arm.fresh_sample()
        record = self.arm.carry.get()
        if sample is not None and (record.grasp_held or record.radius_m is not None) and self.arm.jaw_wide_open(sample):
            record = self.arm.carry.clear()  # a human (or an open outside this server) emptied the gripper
        reasons = []
        if record.radius_m is not None:
            reasons.append(f"carried_object_radius_m {record.radius_m:g} m set at the last arm release")
        if record.grasp_held:
            reasons.append(grasp_reason(record))
        if sample is not None:
            live = LiveState(
                joints=self.arm.measured(sample),
                gripper_effort=finite_effort(sample.efforts, self.arm.gripper),
                currents=dict(sample.currents),
            )
            holding, why = evaluate_precondition(Precondition(type="gripper_holding"), live, self.config)
            if holding:
                reasons.append(f"holding load: {why}")
        return CarryingState(carrying=bool(reasons), reasons=reasons, radius_m=record.radius_m)

    def carrying(self) -> list[str]:
        """Why the gripper counts as carrying an object (empty when it does not).

        Returns:
            list[str]: carrying_state().reasons.
        """
        return self.carrying_state().reasons

    def release_arm(self, keep_position: bool, carried_object_radius_m: float | None = None) -> str:
        """Release the arm lease, homing first unless kept, skipped under battery cut-off or no home is stored; then
        publish the footprint (static after a home, else the frozen measured pose with the carried object).

        A stop during the park keeps the lease (the arm holds where the stop left it) instead of handing the arm to
        the leader arm or the web UI. A keep release while carrying keeps the grip profile's torque limit.

        Args:
            keep_position (bool): Release only.
            carried_object_radius_m (float | None): Carried object radius (keep_position only; None = none).

        Returns:
            str: homed | kept | failed: <why> | skipped: <why> | stopped: <why> (a footprint that could not be
                computed is appended).
            The footprint follows: static when homed, else the measured pose or, without fresh joint states, the conservative one.
        """
        if not self.arm_held:
            return NOT_HELD if self.arm.lost_note is None else f"{NOT_HELD} ({self.arm.lost_note})"
        stops = self.arm.stop_requests
        if keep_position:
            outcome = "kept"
        elif self.guard is not None and self.guard.is_cutoff():
            outcome = "skipped: battery_cutoff"
        elif not self.arm.home_stored():
            outcome = "failed: no home pose stored"
        else:
            try:
                result = self.arm.home(keep_prior_control=True)
            except ArmError as exc:
                outcome = f"failed: {exc}"
            else:
                outcome = "homed" if result.status == "converged" else f"failed: {result.status}: {result.message}"
            if self.arm.stop_requests != stops and self.arm.control_held:
                self.arm.hold()
                return STOPPED_PARK
        carrying = self.carrying_state().carrying
        if self.arm.control_held:
            self.arm.release(restore_torque=not (keep_position and carrying))
        with self.lock:
            self.arm_generation = None
            self.arm_since = None
        radius = carried_object_radius_m if keep_position else None
        self.arm.carry.update(radius_m=radius)
        return self.publish_footprint(outcome, radius)

    def publish_footprint(self, outcome: str, radius: float | None) -> str:
        """Footprint after an arm release: static when homed, the measured (frozen) pose when joint states are
        fresh, else the conservative max-reach footprint (the pose is unknown), each with the carried object.

        Args:
            outcome (str): Arm reset outcome.
            radius (float | None): Carried object radius (m).

        Returns:
            str: The outcome, unchanged.
        """
        if self.footprint is None:
            return outcome
        if outcome == "homed":
            self.footprint.publish_static()
            return outcome
        sample = self.arm.fresh_sample()
        if sample is None:
            self.footprint.publish_conservative(radius)
            return outcome
        self.footprint.publish_dynamic(self.arm.body_check.part_points(self.arm.measured(sample)), radius)
        return outcome

    def release_base(self, keep_position: bool) -> str:
        """Release the base lease, zeroing the base and waiting for neutral steering unless kept.

        The lease is cleared before the reset starts: base tools are refused during the wait, and a take_control
        ('base') meanwhile is a new lease that stays held.

        Args:
            keep_position (bool): Clear the flag only.

        Returns:
            str: neutral | kept | failed: <why> | skipped: not held.
        """
        with self.lock:
            if self.base_since is None:
                return NOT_HELD
            self.base_since = None
        if keep_position:
            return "kept"
        cancelled = self.base.reset_base()
        ok, why = wait_steering_neutral(
            self.base.steer_positions,
            self.cfg.steer_joint_names,
            self.cfg.steer_neutral_tolerance_rad,
            self.cfg.base_reset_timeout_s,
            self.clock,
            self.sleep,
            self.cfg.steer_poll_s,
        )
        if not ok:
            return f"failed: {why}"
        return "neutral" if cancelled else "failed: Nav2 cancel service did not answer (zero twist sent)"

    def held_by(self) -> str | None:
        """Who commands the arm now.

        Returns:
            str | None: agent (take_control), mcp_server (an implicit hold: web UI grasp, stop), filter_node's active
                source otherwise (leader), None when unknown.
                web_ui is the lease the web UI took with /arm/take_control.
        """
        if self.arm_held:
            return "agent"
        if self.web_ui_held:
            return WEB_UI_HOLDER
        if self.arm.control_held:
            return "mcp_server"
        src = self.arm.backend.active_source()
        if src is None or src.value in (self.config.arm.autonomy_source_name, SOURCE_NONE):
            return None
        return src.value

    def state(self) -> ControlState:
        """Both leases now.

        Returns:
            ControlState: Arm and base lease state.
        """
        src = self.arm.backend.active_source()
        held = self.arm_held
        arm = ArmControlState(
            held=held,
            held_since=round(self.arm_since, 3) if held and self.arm_since is not None else None,
            held_by=self.held_by(),
            carrying=self.carrying_state(),
            at_home=self.arm.at_home(),
            home_stored=self.arm.home_stored(),
            active_source=None if src is None else src.value,
            lost=self.arm.lost_note,
        )
        base = BaseControlState(
            held=self.base_held,
            held_since=self.base_since,
            steering_neutral=steering_neutral(
                self.base.steer_positions(), self.cfg.steer_joint_names, self.cfg.steer_neutral_tolerance_rad
            ),
        )
        footprint = None if self.footprint is None else self.footprint.state()
        return ControlState(arm=arm, base=base, footprint=footprint)

    def refusal(self, tool_name: str, need: LeaseNeed) -> str | None:
        """Refusal text for a tool whose lease is not held.

        Args:
            tool_name (str): Tool being called (named in the text).
            need (LeaseNeed): arm, base or any (either lease).

        Returns:
            str | None: The refusal, None when the lease is held.
        """
        if need == "any":
            if self.arm_held or self.base_held:
                return None
            return (
                f"{tool_name} refused: arm or base control not held - call take_control(subsystem='arm') or "
                "take_control(subsystem='base')"
            )
        return None if self.held(need) else self.missing_text(tool_name, [need])

    def missing_text(self, tool_name: str, missing: list[Subsystem]) -> str:
        """Refusal text naming the leases not held (the arm one says who took it when known).

        Args:
            tool_name (str): Tool being called.
            missing (list[Subsystem]): Leases not held (arm before base).

        Returns:
            str: e.g. "move_arm_joints refused: arm control not held - call take_control(subsystem='arm')".
        """
        parts = []
        for name in missing:
            note = f" ({self.arm.lost_note})" if name == "arm" and self.arm.lost_note else ""
            parts.append(f"{name} control not held{note} - call take_control(subsystem='{name}')")
        return f"{tool_name} refused: " + "; ".join(parts)

    def steps_refusal(self, tool_name: str, kinds: Iterable[str]) -> str | None:
        """Refusal for an enqueue whose steps need a lease that is not held.

        Args:
            tool_name (str): Tool being called.
            kinds (Iterable[str]): Step kinds.

        Returns:
            str | None: The refusal, None when every needed lease is held.
        """
        needed = self.step_needs(kinds)
        missing = [s for s in SUBSYSTEMS if s in needed and not self.held(s)]
        return self.missing_text(tool_name, missing) if missing else None

    def step_refusal(self, kind: str) -> str | None:
        """Motion queue dispatch check: refusal for one step whose lease is no longer held (e.g. lost mid-queue).

        Args:
            kind (str): Step kind.

        Returns:
            str | None: The refusal, None when allowed.
        """
        return self.steps_refusal(f"queued {kind} step", [kind])

    @staticmethod
    def step_needs(kinds: Iterable[str]) -> set[Subsystem]:
        """Leases a set of enqueue_motions steps needs.

        Args:
            kinds (Iterable[str]): Step kinds.

        Returns:
            set[Subsystem]: arm for arm / gripper / grasp steps, base for Nav2 steps (BASE_KINDS).
        """
        needs: set[Subsystem] = set()
        for kind in kinds:
            if kind in ARM_LEASE_KINDS:
                needs.add("arm")
            elif kind in BASE_KINDS:
                needs.add("base")
        return needs

    def digest(self) -> str:
        """Short lease summary for the vitals line.

        Returns:
            str: e.g. "ctl arm=H@home base=-" (H held, - not held; @home / @away when known).
        """
        at_home = self.arm.at_home()
        where = "" if at_home is None else ("@home" if at_home else "@away")
        arm = "H" if self.arm_held else "-"
        return f"ctl arm={arm}{where} base={'H' if self.base_held else '-'}"
