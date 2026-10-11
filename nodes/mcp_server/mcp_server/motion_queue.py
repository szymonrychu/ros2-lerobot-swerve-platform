"""Non-blocking motion queue: one FIFO of motion steps run by a background worker, with blending and events.

The worker is the single motion owner while the queue is busy (the MCP layer refuses blocking motion tools then). Every
step reuses the normal code paths (ArmController.move_blend / set_gripper, RobotApi.navigate / move_relative), so the
lease, stop, stale/tracking/effort guards, critical-event interrupts and the floor slow zone all stay in force.
Consecutive arm steps form a blend group and run as ONE continuous trajectory through their targets; a step with
settle 'final', a gripper/base/wait/grasp step, a precondition, a different speed scale or slow-zone override ends a
group. Preconditions are evaluated at dispatch time against live state. Outcomes are published as events that
wait_for_event blocks on.

Watch constraints (enqueue(..., watch_constraints)) bind to the job ids of their enqueue call and are evaluated by a
separate 'motion-watch' thread while one of those steps runs (never inside the arm setpoint stream), from the cached
robot state only (guard_state.WatchSampler). A trip aborts the queue (the cancel path) or pauses it (the running group
is stopped, the pending steps wait for resume()).
"""

import logging
import math
import operator
import threading
import time
from collections import deque
from collections.abc import Callable
from contextlib import AbstractContextManager, nullcontext
from dataclasses import dataclass, field
from typing import Annotated, Any, Literal, Protocol

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .arm import ArmError, SelfCollisionError, finite_effort
from .config import ARM_JOINTS, HARD_MAX_SPEED_SCALE, GripProfileOverride, McpServerConfig, StrictModel
from .floor_guard import FloorOverride, TiltOverrideDeg
from .grasp import ObjectSpec
from .grasp_tools import floor_override
from .guard_rules import GUARD_ORIGIN, GuardRejection
from .ik import UnreachableError
from .metrics import MOTION_QUEUE_DEPTH, MOTION_STEPS, MOTION_TRACKING_ERROR, MOTION_WATCH_TRIGGERS
from .models import ArmMotionResult, BasePose, NavigationResult, RobotError, SettlePolicy
from .surfaces import MAX_REGIONS, SurfaceRegion
from .tool_context import RobotApi

LOGGER = logging.getLogger("mcp_server.motion_queue")

ARM_KINDS = frozenset({"arm_joints", "arm_cartesian"})
BASE_KINDS = frozenset({"base_relative", "navigate_to_pose"})  # Nav2 goals (refused by the tilt guard at warning)
# Arm statuses that complete a step; every other status fails it ('stopped' ends the queue as a stop).
ARM_OK = frozenset({"converged", "grasped", "closed_no_contact"})
CONTACT_STATUSES = frozenset({"grasped", "blocked"})
NAV_OK = frozenset({"succeeded"})
NAV_STOPPED = frozenset({"canceled"})
GRASP_OK = frozenset({"grasped", "released"})
# Wake-up events of a failure. 'stopped' / 'cancelled' themselves do not wake a wait while the aborted step is still
# winding down: its 'step_aborted' (or the queue going idle) does, so the state digest shows the robot at rest. A watch
# trip ('watch_triggered', and 'paused' for action pause) wakes at once: the agent must learn why the queue ended.
FAILURE_EVENTS = frozenset(
    {"step_failed", "precondition_failed", "queue_stopped", "step_aborted", "watch_triggered", "paused"}
)
WAKE_EVENTS: dict[str, frozenset[str]] = {
    "queue_empty": FAILURE_EVENTS | {"contact", "queue_empty"},
    "step_done": FAILURE_EVENTS | {"contact", "queue_empty", "step_done", "step_skipped"},
    "failure": FAILURE_EVENTS,
}
# Data key marking a failure the queue continued past (on_fail='skip'): it does not end a 'queue_empty' wait.
SKIPPED_KEY = "on_fail"
DIGEST_DECIMALS = 3
# Watch constraints.
DISTANCE_METRIC = "distance_travelled_m"  # integrated by the watcher from fresh base_speed_mps
SPEED_METRIC = "base_speed_mps"
JOINT_METRICS = frozenset({"joint_position", "joint_effort"})
WATCH_NAME_MAX = 40
WATCH_MAX_FOR_S = 5.0
WATCH_TIME_EPS = 1e-9  # for_s comparison slack (float sums of clock steps)
WATCH_OPS: dict[str, Callable[[float, float], bool]] = {
    "lt": operator.lt,
    "le": operator.le,
    "gt": operator.gt,
    "ge": operator.ge,
}

PreconditionType = Literal["none", "gripper_holding", "gripper_open", "arm_near", "base_still", "battery_ok"]
OnFail = Literal["stop_queue", "skip"]
WaitUntil = Literal["any", "queue_empty", "step_done", "failure"]
WatchMetric = Literal[
    "gripper_effort",
    "gripper_position",
    "joint_position",
    "joint_effort",
    "tilt_deg",
    "roll_deg",
    "pitch_deg",
    "base_speed_mps",
    "distance_travelled_m",
    "battery_v",
    "tool_height_m",
]
WatchSample = Callable[[str, str | None], float | None]
EventType = Literal[
    "step_started",
    "planned",
    "step_done",
    "step_failed",
    "step_skipped",
    "step_aborted",
    "precondition_failed",
    "contact",
    "queue_stopped",
    "queue_empty",
    "cancelled",
    "replaced",
    "stopped",
    "watch_triggered",
    "paused",
    "resumed",
]


class QueueError(RuntimeError):
    """An enqueue request was refused (nothing was queued); the message lists the reasons."""


class Precondition(BaseModel):
    """Whitelisted predicate checked against live state when its step is dispatched.

    Attributes:
        type: 'none' (always true), 'gripper_holding' (jaw short of closed, not wide open and loaded),
            'gripper_open' (open fraction >= min_fraction), 'arm_near' (every named joint within tol of `joints`),
            'base_still' (odometry speed below the still thresholds), 'battery_ok' (fresh reading, not in cut-off).
        joints: arm_near: joint name -> rad (measured space, as get_arm_state reports).
        tol: arm_near tolerance (rad).
        min_fraction: gripper_open threshold (0 closed, 1 fully open).
    """

    model_config = ConfigDict(extra="forbid")

    type: PreconditionType = "none"
    joints: dict[str, float] | None = None
    tol: float = Field(default=0.05, gt=0.0, le=1.0)
    min_fraction: float = Field(default=0.5, ge=0.0, le=1.0)

    @model_validator(mode="after")
    def joints_for_arm_near(self) -> "Precondition":
        """arm_near needs joints.

        Returns:
            Precondition: The validated precondition.
        """
        if self.type == "arm_near" and not self.joints:
            raise ValueError("arm_near needs joints {name: rad}")
        return self


class StepBase(BaseModel):
    """Fields every queued step has."""

    model_config = ConfigDict(extra="forbid")

    precondition: Precondition = Field(
        default_factory=Precondition, description="Checked against live state right before the step starts"
    )
    on_fail: OnFail = Field(
        default="stop_queue", description="'stop_queue' (default) drops the rest of the queue; 'skip' continues"
    )
    label: str | None = Field(default=None, max_length=80, description="Your name for the step (echoed in events)")


class ArmMotionFields(StepBase):
    """Speed, settle and slow-zone fields of an arm step."""

    speed_scale: float | None = Field(default=None, gt=0.0, le=HARD_MAX_SPEED_SCALE)
    settle: SettlePolicy | None = None
    surface_z_m: float | None = Field(default=None, ge=-1.0, le=1.0)
    tilt_override_deg: TiltOverrideDeg | None = None
    surfaces: list[SurfaceRegion] | None = Field(default=None, max_length=MAX_REGIONS)


class ArmJointsStep(ArmMotionFields):
    """Move arm joints (as move_arm_joints)."""

    kind: Literal["arm_joints"]
    targets: dict[str, float]


class ArmCartesianStep(ArmMotionFields):
    """Move the tool point (as move_arm_cartesian); solved by IK at enqueue time."""

    kind: Literal["arm_cartesian"]
    x: float
    y: float
    z: float
    pitch: float | None = None
    wrist_roll: float | None = None
    object_width_m: float | None = Field(default=None, gt=0.0, le=0.08)


class GripperStep(StepBase):
    """Open to a fraction or close until contact (as set_gripper)."""

    kind: Literal["gripper"]
    open_fraction: float | None = Field(default=None, ge=0.0, le=1.0)
    close_until_effort: bool = False
    effort_threshold: float | None = Field(default=None, gt=0.0)
    grip_profile: str | GripProfileOverride | None = None  # close_until_effort only (as set_gripper)
    surface_z_m: float | None = Field(default=None, ge=-1.0, le=1.0)
    tilt_override_deg: TiltOverrideDeg | None = None
    surfaces: list[SurfaceRegion] | None = Field(default=None, max_length=MAX_REGIONS)

    @model_validator(mode="after")
    def exactly_one(self) -> "GripperStep":
        """Exactly one of open_fraction and close_until_effort; grip_profile only with close_until_effort.

        Returns:
            GripperStep: The validated step.
        """
        if (self.open_fraction is None) == (not self.close_until_effort):
            raise ValueError("give exactly one of open_fraction or close_until_effort=true")
        if self.grip_profile is not None and not self.close_until_effort:
            raise ValueError("grip_profile applies to close_until_effort=true only")
        return self


class BaseRelativeStep(StepBase):
    """Relative base move through Nav2 (as move_relative)."""

    kind: Literal["base_relative"]
    dx: float = 0.0
    dy: float = 0.0
    dyaw: float = 0.0
    timeout_s: float | None = Field(default=None, gt=0.0)
    precise: bool = False


class NavigateStep(StepBase):
    """Nav2 goal pose (as navigate_to_pose)."""

    kind: Literal["navigate_to_pose"]
    x: float
    y: float
    yaw: float = 0.0
    frame: str = "map"
    timeout_s: float | None = Field(default=None, gt=0.0)
    precise: bool = False


class WaitStep(StepBase):
    """Pause the queue (e.g. let a camera settle); ends early on cancel or stop."""

    kind: Literal["wait_s"]
    seconds: float = Field(gt=0.0, le=60.0)


class GraspMotionStep(StepBase):
    """Plan and execute a grasp (as grasp_object)."""

    kind: Literal["grasp"]
    object: ObjectSpec
    strategy: str = "auto"
    params: dict[str, Any] | None = None
    approach_pitch_deg: float | None = Field(default=None, ge=0.0, le=90.0)
    surface_z_m: float | None = Field(default=None, ge=-1.0, le=1.0)
    tilt_override_deg: TiltOverrideDeg | None = None
    surfaces: list[SurfaceRegion] | None = Field(default=None, max_length=MAX_REGIONS)


ArmStep = ArmJointsStep | ArmCartesianStep
MotionStep = Annotated[
    ArmJointsStep | ArmCartesianStep | GripperStep | BaseRelativeStep | NavigateStep | WaitStep | GraspMotionStep,
    Field(discriminator="kind"),
]
GraspRunner = Callable[[GraspMotionStep, Callable[[], bool]], tuple[str, list[str]]]


class WatchConstraint(StrictModel):
    """A condition watched while the steps of its enqueue call run; a trip aborts or pauses the queue."""

    name: str | None = Field(default=None, max_length=WATCH_NAME_MAX, description="Label in events and status")
    metric: WatchMetric = Field(
        description="Cached robot value: efforts as absolute values, distance_travelled_m integrated from fresh "
        "odometry speed since the watch became active, tool_height_m from the measured pose"
    )
    joint: str | None = Field(default=None, description="Arm joint, required for joint_position / joint_effort only")
    op: Literal["lt", "le", "gt", "ge"]
    value: float = Field(allow_inf_nan=False, description="Threshold the metric is compared against")
    for_s: float = Field(
        default=0.0, ge=0.0, le=WATCH_MAX_FOR_S, description="The condition must hold continuously this long (s)"
    )
    action: Literal["abort", "pause"] = "abort"
    on_unknown: Literal["ignore", "trigger"] = Field(
        default="ignore",
        description="Unknown value (stale cache, NaN effort): 'ignore' neither trips nor resets the debounce, "
        "'trigger' counts it as a violation",
    )

    @model_validator(mode="after")
    def joint_matches_metric(self) -> "WatchConstraint":
        """Require an arm joint for the joint metrics and none for the others.

        Returns:
            WatchConstraint: The validated constraint.

        Raises:
            ValueError: Missing, unknown or superfluous joint.
        """
        if self.metric in JOINT_METRICS:
            if self.joint not in ARM_JOINTS:
                raise ValueError(f"{self.metric} needs joint, one of {list(ARM_JOINTS)} (got {self.joint!r})")
        elif self.joint is not None:
            raise ValueError(f"joint is only allowed with {sorted(JOINT_METRICS)}, not with {self.metric}")
        return self

    @property
    def label(self) -> str:
        """Name in events and status: the given name, else '<metric>[<joint>] <op> <value>'.

        Returns:
            str: The label.
        """
        if self.name:
            return self.name
        joint = f"[{self.joint}]" if self.joint else ""
        return f"{self.metric}{joint} {self.op} {self.value:g}"


def watch_violated(constraint: WatchConstraint, value: float | None) -> bool | None:
    """Compare a sampled value with a watch threshold.

    Args:
        constraint (WatchConstraint): The watch.
        value (float | None): Sampled value; None = unknown.

    Returns:
        bool | None: True when violated, False when not, None when unknown and on_unknown='ignore'.
    """
    if value is None:
        return True if constraint.on_unknown == "trigger" else None
    return WATCH_OPS[constraint.op](value, constraint.value)


class CutoffGuard(Protocol):
    """The BatteryGuard calls the queue needs."""

    def is_cutoff(self) -> bool:
        """Whether motion is refused."""
        ...

    def rejection_message(self) -> str:
        """Refusal text."""
        ...

    def state(self) -> dict[str, Any]:
        """voltage, stale, cutoff, ..."""
        ...


@dataclass(frozen=True)
class LiveState:
    """Live robot state a precondition is evaluated against (None = unknown)."""

    joints: dict[str, float] | None = None
    gripper_effort: float | None = None
    base_twist: tuple[float, float, float] | None = None
    battery_ok: bool | None = None


class MotionEvent(BaseModel):
    """One queue event (seq increases by one per event)."""

    seq: int
    t: float = Field(description="Wall-clock time (s)")
    type: EventType
    job_id: str | None = None
    kind: str | None = None
    label: str | None = None
    message: str = ""
    data: dict[str, Any] = Field(default_factory=dict)


class QueuedStepView(BaseModel):
    """A queued step as reported by get_motion_status."""

    job_id: str
    kind: str
    label: str | None = None
    summary: str


class CurrentStepView(QueuedStepView):
    """The step being executed."""

    elapsed_s: float
    phase: Literal["planning", "executing"] = Field(
        default="executing",
        description="'planning': arm steps while the safety checks run, before the first setpoint (normal work, not a "
        "stall); 'executing': the motion is running",
    )
    planning_s: float | None = Field(
        default=None, description="Seconds spent planning so far (only while phase=planning)"
    )
    progress: float | None = Field(
        default=None,
        description="0..1 when known: arm steps = fraction of the published trajectory time (0 while planning), "
        "wait elapsed",
    )
    blend_group: list[str] = Field(default_factory=list, description="Job ids blended into this one motion")


class WatchView(BaseModel):
    """A watch constraint as reported by get_motion_status."""

    name: str
    metric: str
    joint: str | None = None
    op: str
    value: float
    last_value: float | None = Field(default=None, description="Last sampled value (None = unknown / not sampled)")
    job_ids: list[str]
    active: bool = Field(description="One of its steps runs now (the watcher evaluates it)")
    triggered: bool


class PauseView(BaseModel):
    """Why and since when the queue is paused."""

    by: str = Field(description="Label of the watch that paused the queue")
    at: float = Field(description="Wall-clock time (s)")


class ResumeResult(BaseModel):
    """resume() result."""

    resumed: bool
    message: str
    queue_length: int = Field(description="Pending steps that now run (or still wait)")


class MotionStatus(BaseModel):
    """get_motion_status result."""

    running: bool
    current: CurrentStepView | None = None
    queue: list[QueuedStepView] = Field(default_factory=list)
    last_events: list[MotionEvent] = Field(default_factory=list)
    event_seq: int = 0
    paused: PauseView | None = Field(
        default=None, description="Set while a watch paused the queue: resume_motions continues, cancel_motions ends"
    )
    watches: list[WatchView] = Field(default_factory=list, description="Watch constraints of the queued steps")


class EnqueueResult(BaseModel):
    """enqueue_motions result."""

    job_ids: list[str]
    queue_length: int = Field(description="Pending steps after this call (the running one excluded)")
    replaced: list[str] = Field(default_factory=list, description="Pending job ids dropped by replace=true")
    blend_groups: list[list[str]] = Field(
        default_factory=list, description="How the new steps will run: job ids blended into one continuous motion"
    )


class CancelResult(BaseModel):
    """cancel_motions result."""

    dropped: list[str]
    running: list[str] | None = Field(default=None, description="Job ids of the step that was aborted, if any")
    message: str = ""


class WaitResult(BaseModel):
    """wait_for_event result (the tool adds the state digest)."""

    reason: str = Field(description="Event type that ended the wait, 'idle' (queue empty) or 'timeout'")
    timed_out: bool = False
    events: list[MotionEvent] = Field(default_factory=list)
    status: MotionStatus | None = None
    state: dict[str, Any] | None = None


@dataclass
class QueuedStep:
    """A validated step with its job id and (arm steps) its resolved joint targets."""

    job_id: str
    step: Any
    targets: dict[str, float] | None = None


@dataclass
class WatchState:
    """A watch constraint bound to the job ids of its enqueue call, with the watcher's per-watch state."""

    constraint: WatchConstraint
    job_ids: list[str]
    active: bool = False
    triggered: bool = False
    last_value: float | None = None
    since: float | None = None  # start of the current continuous violation (for_s)
    distance: float = 0.0  # distance_travelled_m integral while active
    speed_at: float | None = None  # time of the last fresh speed sample integrated

    @property
    def ids(self) -> frozenset[str]:
        """The bound job ids as a set.

        Returns:
            frozenset[str]: Job ids.
        """
        return frozenset(self.job_ids)

    def view(self) -> WatchView:
        """Status view.

        Returns:
            WatchView: The view.
        """
        c = self.constraint
        return WatchView(
            name=c.label,
            metric=c.metric,
            joint=c.joint,
            op=c.op,
            value=c.value,
            last_value=None if self.last_value is None else round(self.last_value, DIGEST_DECIMALS),
            job_ids=list(self.job_ids),
            active=self.active,
            triggered=self.triggered,
        )


@dataclass
class Running:
    """The group being executed."""

    group: list[QueuedStep]
    started: float
    passed: int = 0
    expected_s: float | None = None
    phase: Literal["planning", "executing"] = "executing"
    ticks: int = 0
    total_ticks: int = 0
    done: set[str] = field(default_factory=set)


def resolve_settle(step: ArmStep, config: McpServerConfig, targets: dict[str, float] | None) -> SettlePolicy:
    """Settle policy of an arm step: its own, else 'final' when it moves the gripper joint, else the default.

    Args:
        step (ArmStep): Arm step.
        config (McpServerConfig): Node configuration.
        targets (dict[str, float] | None): Its joint targets (when known).

    Returns:
        SettlePolicy: The policy.
    """
    if step.settle is not None:
        return step.settle
    if targets is not None and config.arm.gripper_joint in targets:
        return "final"
    return config.limits.arm_default_settle


def step_floor(step: Any) -> FloorOverride | None:
    """Slow-zone override of a step (None when it overrides nothing).

    Args:
        step (Any): A step with surface_z_m / tilt_override_deg / surfaces.

    Returns:
        FloorOverride | None: The override.
    """
    return floor_override(step.surface_z_m, step.tilt_override_deg, step.surfaces)


def can_blend(prev: QueuedStep, nxt: QueuedStep, config: McpServerConfig) -> bool:
    """Whether nxt continues prev's blend group (one continuous trajectory).

    Args:
        prev (QueuedStep): Last step of the group so far.
        nxt (QueuedStep): Candidate.
        config (McpServerConfig): Node configuration.

    Returns:
        bool: True when both are arm steps, prev does not settle 'final', nxt has no precondition and both share the
            speed scale and slow-zone override.
    """
    a, b = prev.step, nxt.step
    if a.kind not in ARM_KINDS or b.kind not in ARM_KINDS:
        return False
    if resolve_settle(a, config, prev.targets) == "final" or b.precondition.type != "none":
        return False
    return a.speed_scale == b.speed_scale and step_floor(a) == step_floor(b)


def group_queued(items: list[QueuedStep], config: McpServerConfig) -> list[list[QueuedStep]]:
    """Split queued steps into blend groups (non-arm steps are groups of one).

    Args:
        items (list[QueuedStep]): Steps in order.
        config (McpServerConfig): Node configuration.

    Returns:
        list[list[QueuedStep]]: Groups in order.
    """
    groups: list[list[QueuedStep]] = []
    for item in items:
        if groups and can_blend(groups[-1][-1], item, config):
            groups[-1].append(item)
        else:
            groups.append([item])
    return groups


def blend_groups(steps: list[Any], config: McpServerConfig) -> list[list[Any]]:
    """Blend groups of raw steps (arm_joints targets are known; cartesian targets count as arm-only).

    Args:
        steps (list[Any]): Validated steps.
        config (McpServerConfig): Node configuration.

    Returns:
        list[list[Any]]: The steps grouped.
    """
    items = [QueuedStep(str(i), s, dict(s.targets) if s.kind == "arm_joints" else None) for i, s in enumerate(steps)]
    return [[q.step for q in group] for group in group_queued(items, config)]


def evaluate_precondition(pre: Precondition, state: LiveState, config: McpServerConfig) -> tuple[bool, str]:
    """Evaluate a precondition against live state; unknown state never passes (except 'none').

    Args:
        pre (Precondition): The precondition.
        state (LiveState): Live state.
        config (McpServerConfig): Thresholds (motion_queue, arm gripper range).

    Returns:
        tuple[bool, str]: Whether it holds, and why (always set).
    """
    if pre.type == "none":
        return True, "no precondition"
    q = config.motion_queue
    gripper = config.arm.gripper_joint
    if pre.type == "battery_ok":
        return bool(state.battery_ok), "battery ok" if state.battery_ok else "battery low, in cut-off or unknown"
    if pre.type == "base_still":
        if state.base_twist is None:
            return False, "odometry unavailable: cannot confirm the base is still"
        vx, vy, wz = state.base_twist
        still = math.hypot(vx, vy) < q.base_still_linear_mps and abs(wz) < q.base_still_angular_rps
        return still, f"base speed {math.hypot(vx, vy):.3f} m/s, {wz:.3f} rad/s"
    if state.joints is None or gripper not in state.joints:
        return False, "no fresh joint states"
    jaw = state.joints[gripper]
    closed, opened = config.arm.gripper_closed_rad, config.arm.gripper_open_rad
    if pre.type == "gripper_open":
        fraction = (jaw - closed) / (opened - closed)
        return fraction >= pre.min_fraction, f"gripper open fraction {fraction:.2f} (need >= {pre.min_fraction:g})"
    if pre.type == "gripper_holding":
        gap = (jaw - closed) * (1.0 if opened > closed else -1.0)
        jaw_text = f"jaw {jaw:.3f} rad ({gap:.3f} rad short of closed)"
        if state.gripper_effort is None or not math.isfinite(state.gripper_effort):
            return False, f"{jaw_text}, gripper load unmeasured"
        effort = abs(state.gripper_effort)
        holding = (
            gap >= q.holding_min_gap_rad
            and jaw <= config.limits.gripper_grasp_max_open_rad
            and effort >= q.holding_min_effort
        )
        return holding, f"{jaw_text}, gripper load {effort:.0f}"
    joints = pre.joints or {}
    off = {j: round(state.joints.get(j, math.inf) - v, 3) for j, v in joints.items()}
    far = {j: e for j, e in off.items() if not abs(e) <= pre.tol}
    if far:
        return False, f"arm not near the given pose: {far} rad off (tol {pre.tol:g})"
    return True, f"arm within {pre.tol:g} rad of the given pose"


def wakes(event: MotionEvent, until: WaitUntil) -> bool:
    """Whether an event ends a wait: a failure the queue skipped past does not end a 'queue_empty' wait.

    Args:
        event (MotionEvent): The event.
        until (WaitUntil): The wait's condition.

    Returns:
        bool: True when the wait should return on this event.
    """
    if until == "any":
        return True
    if until == "queue_empty" and event.data.get(SKIPPED_KEY) == "skip":
        return False
    return event.type in WAKE_EVENTS[until]


def read_live_state(robot: RobotApi, guard: CutoffGuard | None, need_twist: bool) -> LiveState:
    """Live state for preconditions.

    Args:
        robot (RobotApi): Robot.
        guard (CutoffGuard | None): Battery guard; None = no battery gate (battery_ok holds).
        need_twist (bool): Read the odometry twist (robot_state) too.

    Returns:
        LiveState: Joints (fresh only), gripper load, base twist and battery verdict.
    """
    sample = robot.arm.fresh_sample()
    twist = None
    if need_twist:
        odom = robot.robot_state().odom_twist
        twist = None if odom is None else (odom.vx, odom.vy, odom.wz)
    battery_ok = True
    if guard is not None:
        state = guard.state()
        battery_ok = not state.get("stale", True) and not state.get("cutoff", False)
    return LiveState(
        joints=None if sample is None else robot.arm.measured(sample),
        gripper_effort=None if sample is None else finite_effort(sample.efforts, robot.arm.gripper),
        base_twist=twist,
        battery_ok=battery_ok,
    )


def state_digest(robot: RobotApi, guard: CutoffGuard | None) -> dict[str, Any]:
    """Compact state attached to wait_for_event (so a separate get_robot_state is rarely needed).

    Args:
        robot (RobotApi): Robot.
        guard (CutoffGuard | None): Battery guard.

    Returns:
        dict[str, Any]: arm joints (rad), gripper position/effort/open fraction, base pose, battery V and lease;
            unknown values are null (never invented).
    """
    arm = robot.arm
    sample = arm.fresh_sample()
    joints = None if sample is None else {j: round(v, DIGEST_DECIMALS) for j, v in arm.measured(sample).items()}
    closed, opened = arm.cfg.arm.gripper_closed_rad, arm.cfg.arm.gripper_open_rad
    gripper = None
    if sample is not None and arm.gripper in sample.positions:
        jaw = sample.positions[arm.gripper]
        gripper = {
            "position": round(jaw, DIGEST_DECIMALS),
            "open_fraction": round((jaw - closed) / (opened - closed), 2),
            "effort": finite_effort(sample.efforts, arm.gripper),
        }
    pose: BasePose | None = robot.robot_pose()
    battery = None
    if guard is not None:
        state = guard.state()
        if state.get("voltage") is not None and not state.get("stale", True):
            battery = round(float(state["voltage"]), 2)
    return {
        "arm_joints": joints,
        "gripper": gripper,
        "base_pose": None
        if pose is None
        else {"frame": pose.frame, "x": round(pose.x, 3), "y": round(pose.y, 3), "yaw": round(pose.yaw, 3)},
        "battery_v": battery,
        "control_held": arm.control_held,
    }


def describe(step: Any) -> str:
    """Short human-readable summary of a step.

    Args:
        step (Any): Validated step.

    Returns:
        str: Summary.
    """
    if step.kind == "arm_joints":
        return "joints " + ", ".join(f"{j}={v:.3f}" for j, v in step.targets.items())
    if step.kind == "arm_cartesian":
        pitch = "" if step.pitch is None else f" pitch {step.pitch:.2f}"
        return f"tool point ({step.x:.3f}, {step.y:.3f}, {step.z:.3f}){pitch}"
    if step.kind == "gripper":
        return "close until contact" if step.close_until_effort else f"open fraction {step.open_fraction:g}"
    if step.kind == "base_relative":
        return f"relative dx={step.dx:g} dy={step.dy:g} dyaw={step.dyaw:g}"
    if step.kind == "navigate_to_pose":
        return f"goal ({step.x:g}, {step.y:g}, yaw {step.yaw:g}) in {step.frame}"
    if step.kind == "wait_s":
        return f"wait {step.seconds:g} s"
    return f"grasp object at ({step.object.x:g}, {step.object.y:g}) strategy {step.strategy}"


def arm_result_data(result: ArmMotionResult) -> dict[str, Any]:
    """Compact fields of an arm result for an event.

    Args:
        result (ArmMotionResult): Result.

    Returns:
        dict[str, Any]: status, durations, tracking error and (when set) settling, residual, slow zone, interrupt,
            gripper effort and the grip report of a close (grip_profile, holding_load, slipping, crush_risk,
            retightened).
    """
    data: dict[str, Any] = {
        "status": result.status,
        "trajectory_s": result.trajectory_s,
        "settle_s": result.settle_s,
        "tracking_error_rad": result.tracking_error_rad,
        "time_to_first_setpoint_s": result.time_to_first_setpoint_s,
        "tick_rate_hz": result.tick_rate_hz,
    }
    for key in (
        "settling",
        "residual_error",
        "slow_zone",
        "self_collision",
        "interrupted_by",
        "clamped",
        "gripper_effort",
    ):
        value = getattr(result, key)
        if value:
            data[key] = value
    for key in ("grip_profile", "holding_load", "slipping", "crush_risk", "retightened", "retighten_reverted"):
        value = getattr(result, key)
        if value is not None:
            data[key] = value
    return data


class MotionQueue:
    """FIFO of motion steps executed by one background worker thread (see the module docstring)."""

    def __init__(
        self,
        robot: RobotApi,
        config: McpServerConfig,
        guard: CutoffGuard | None = None,
        grasp_runner: GraspRunner | None = None,
        external_busy: Callable[[], str | None] | None = None,
        live_state: Callable[[bool], LiveState] | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        tilt_gate: Callable[[], str | None] | None = None,
        lease_gate: Callable[[str], str | None] | None = None,
        lease_scope: Callable[[str], AbstractContextManager[None]] | None = None,
        guard_gate: Callable[[str, dict[str, Any]], None] | None = None,
        sampler: WatchSample | None = None,
        on_watch_trigger: Callable[[dict[str, Any]], None] | None = None,
        watch_sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        """Create the queue (the worker starts on the first enqueue).

        Args:
            robot (RobotApi): Robot (its arm controller and base motion methods).
            config (McpServerConfig): Node configuration.
            guard (CutoffGuard | None): Battery guard, re-checked before every step.
            grasp_runner (GraspRunner | None): Runs a grasp step; None refuses grasp steps.
            external_busy (Callable[[], str | None] | None): Name of a blocking motion tool running now, else None.
            live_state (Callable[[bool], LiveState] | None): Live state for preconditions (arg: odometry needed).
            clock (Callable[[], float]): Monotonic clock (s).
            sleep (Callable[[float], None]): Sleep (wait steps).
            tilt_gate (Callable[[], str | None] | None): Tilt guard refusal reason for a Nav2 base step (None when
                allowed), re-checked before every base step.
            lease_gate (Callable[[str], str | None] | None): Control lease refusal for a step kind (None when its
                lease is held, control.ControlManager.step_refusal), re-checked before every step: a lease lost
                mid-queue (web UI Home, a human take-over) fails the step instead of silently re-taking it.
            lease_scope (Callable[[str], AbstractContextManager[None]] | None): Binds a running step to the lease it
                was checked against (control.ControlManager.step_scope): a lease lost while the step plans or moves
                fails its next motion.
            guard_gate (Callable[[str, dict[str, Any]], None] | None): Guard rule step phase (GuardEngine.check_step:
                step kind, step fields), run for every step of a group after the tilt gate and before the
                precondition; raises guard_rules.GuardRejection to refuse. None = no rule engine.
            sampler (WatchSample | None): Cached value of a watch metric (guard_state.WatchSampler.sample: metric,
                joint -> value or None when unknown); None refuses watch_constraints.
            on_watch_trigger (Callable[[dict[str, Any]], None] | None): Called (outside the queue lock) with the
                watch_triggered data of every trip (RobotMonitor.report_watch_trigger).
            watch_sleep (Callable[[float], None]): Sleep of the watcher thread between ticks.
        """
        self.robot = robot
        self.cfg = config
        self.settings = config.motion_queue
        self.guard = guard
        self.grasp_runner = grasp_runner
        self.external_busy = external_busy or (lambda: None)
        self.live_state = live_state or (lambda need_twist: read_live_state(robot, guard, need_twist))
        self.clock = clock
        self.sleep = sleep
        self.tilt_gate = tilt_gate or (lambda: None)
        self.lease_gate = lease_gate or (lambda kind: None)
        self.lease_scope = lease_scope or (lambda kind: nullcontext())
        self.guard_gate = guard_gate or (lambda kind, fields: None)
        self.sampler = sampler
        self.on_watch_trigger = on_watch_trigger or (lambda data: None)
        self.watch_sleep = watch_sleep
        self._cond = threading.Condition()
        self._pending: deque[QueuedStep] = deque()
        self._events: deque[MotionEvent] = deque(maxlen=self.settings.event_history)
        self._seq = 0
        self._delivered = 0
        self._next_job = 0
        self._generation = 0
        self._running: Running | None = None
        self._stop_baseline: int | None = None
        self._thread: threading.Thread | None = None
        self._watcher: threading.Thread | None = None
        self._watches: list[WatchState] = []
        self._paused: PauseView | None = None

    # --- public API -----------------------------------------------------------------------------------------------

    def busy(self) -> bool:
        """Whether a step runs, steps are pending or a watch paused the queue.

        Returns:
            bool: True while busy.
        """
        with self._cond:
            return self.busy_locked()

    def enqueue(
        self, steps: list[Any], replace: bool = False, watch_constraints: list[WatchConstraint] | None = None
    ) -> EnqueueResult:
        """Validate and append steps (all or nothing); returns at once.

        Args:
            steps (list[Any]): Validated MotionStep models.
            replace (bool): Drop the pending steps first (a running step finishes; a pause is kept).
            watch_constraints (list[WatchConstraint] | None): Watched while these steps run (at most
                motion_queue.watch_max_per_call).

        Returns:
            EnqueueResult: New job ids, the pending length, replaced ids and the blend grouping.

        Raises:
            QueueError: When nothing was queued (invalid/infeasible step, too many steps, another motion owns the robot).
        """
        if not steps:
            raise QueueError("steps must hold at least one step")
        watches = list(watch_constraints or [])
        if len(watches) > self.settings.watch_max_per_call:
            raise QueueError(
                f"at most {self.settings.watch_max_per_call} watch_constraints per call ({len(watches)} given)"
            )
        if watches and self.sampler is None:
            raise QueueError("watch_constraints are not available on this server")
        with self._cond:
            if not self.busy_locked():
                other = self.external_busy()
                if other is not None:
                    raise QueueError(f"refused: the blocking motion tool {other} is running; wait for it or call stop")
                if self.robot.arm.motion_running:
                    raise QueueError(
                        "refused: another arm motion is running (a blocking tool, a web UI grasp or /arm/home); "
                        "wait for it or call stop"
                    )
            kept = [] if replace else list(self._pending)
            if len(kept) + len(steps) > self.settings.max_steps:
                raise QueueError(
                    f"the queue holds at most {self.settings.max_steps} pending steps "
                    f"({len(kept)} pending, {len(steps)} new)"
                )
            items = self.resolve(steps, kept)
            replaced: list[str] = []
            if replace and self._pending:
                replaced = [q.job_id for q in self._pending]
                self._pending.clear()
                self.emit_locked(
                    "replaced", message=f"replaced {len(replaced)} pending steps", data={"dropped": replaced}
                )
            if not self.busy_locked():
                self._stop_baseline = self.robot.stop_count()
            for item in items:
                self._next_job += 1
                item.job_id = f"m{self._next_job}"
                self._pending.append(item)
            if watches:
                job_ids = [q.job_id for q in items]
                self._watches.extend(WatchState(constraint=w, job_ids=job_ids) for w in watches)
            self.prune_watches_locked()
            MOTION_QUEUE_DEPTH.set(len(self._pending))
            self.ensure_worker_locked()
            self._cond.notify_all()
            return EnqueueResult(
                job_ids=[q.job_id for q in items],
                queue_length=len(self._pending),
                replaced=replaced,
                blend_groups=[[q.job_id for q in g] for g in group_queued(items, self.cfg)],
            )

    def cancel(self, reason: str = "cancel_motions") -> CancelResult:
        """Drop every pending step and abort the running one (the robot is stopped when a motion step runs).

        Args:
            reason (str): Why (event message).

        Returns:
            CancelResult: Dropped ids and the aborted step's ids.
        """
        with self._cond:
            was_paused = self._paused is not None
            dropped, running = self.halt_locked("cancelled", reason)
            moving = self.moving_locked()
        if moving:
            self.stop_robot()
        idle = not dropped and running is None and not was_paused
        message = "nothing to cancel" if idle else f"cancelled ({reason})"
        return CancelResult(dropped=dropped, running=running, message=message)

    def halt_for_stop(self) -> list[str]:
        """Stop tool hook (call before RobotApi.stop): drop every pending step at once, end the running group and
        clear a pause.

        Returns:
            list[str]: Dropped job ids (pending ones; the running step ends through the stop itself).
        """
        with self._cond:
            dropped, _running = self.halt_locked("stopped", "stop was called")
            self._stop_baseline = None
            return dropped

    def resume(self) -> ResumeResult:
        """Clear a watch pause: the pending steps run again (their watches re-armed).

        Returns:
            ResumeResult: resumed=False with a message when the queue is not paused.

        Raises:
            QueueError: Refused in battery cut-off, or while a watch of the pending steps still violates.
        """
        with self._cond:
            if self._paused is None:
                return ResumeResult(
                    resumed=False, message="not paused: nothing to resume", queue_length=len(self._pending)
                )
            pending = {q.job_id for q in self._pending}
            watches = [w for w in self._watches if w.ids & pending]
        if self.guard is not None and self.guard.is_cutoff():
            raise QueueError(f"refused: {self.guard.rejection_message()}; the queue stays paused")
        violations: list[str] = []
        for w in watches:
            c = w.constraint
            value = w.distance if c.metric == DISTANCE_METRIC else self.sample(c.metric, c.joint)
            if watch_violated(c, value):
                shown = "unknown" if value is None else f"{value:.{DIGEST_DECIMALS}f}"
                violations.append(f"watch '{c.label}' still violated ({c.metric}={shown} {c.op} {c.value:g})")
        if violations:
            raise QueueError(
                "refused: " + "; ".join(violations) + "; the queue stays paused (fix the cause or cancel_motions)"
            )
        with self._cond:
            if self._paused is None:  # a stop / cancel came first
                return ResumeResult(
                    resumed=False, message="not paused: nothing to resume", queue_length=len(self._pending)
                )
            by = self._paused.by
            self._paused = None
            pending = {q.job_id for q in self._pending}
            for w in self._watches:
                if w.ids & pending:
                    w.triggered, w.since = False, None
            self.emit_locked(
                "resumed",
                message=f"resumed after the pause by watch '{by}'",
                data={"pending": [q.job_id for q in self._pending]},
            )
            if not self._pending:
                self.emit_locked("queue_empty", message="queue drained")
            self._cond.notify_all()
            return ResumeResult(
                resumed=True, message=f"resumed ({len(self._pending)} pending steps)", queue_length=len(self._pending)
            )

    def status(self) -> MotionStatus:
        """Current step, pending steps and the last events.

        Returns:
            MotionStatus: Snapshot.
        """
        with self._cond:
            return self.status_locked()

    def wait(self, timeout_s: float, until: WaitUntil = "queue_empty", since_seq: int | None = None) -> WaitResult:
        """Block until a relevant event (or the queue is idle, or the timeout); returns the undelivered events.

        Args:
            timeout_s (float): Longest wait (s), capped at motion_queue.wait_max_s.
            until (WaitUntil): 'queue_empty' (drained, or a failure/contact/stop/cancel), 'step_done' (any step
                finished or skipped, or the former), 'failure' (failure/stop/cancel only), 'any' (any new event).
            since_seq (int | None): Return events after this seq; None continues after the last wait's events.

        Returns:
            WaitResult: Reason, timed_out, the events and the queue status.
        """
        deadline = time.monotonic() + min(max(0.0, timeout_s), self.settings.wait_max_s)
        with self._cond:
            cursor = self._delivered if since_seq is None else since_seq
            reason = "timeout"
            while True:
                fresh = [e for e in self._events if e.seq > cursor]
                wake = next((e.type for e in fresh if wakes(e, until)), None)
                if wake is not None:
                    reason = wake
                    break
                if not self.busy_locked():
                    reason = "idle"
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._cond.wait(remaining)
            if fresh:
                self._delivered = max(self._delivered, fresh[-1].seq)
            return WaitResult(
                reason=reason,
                timed_out=reason == "timeout",
                events=fresh[-self.settings.wait_events :],
                status=self.status_locked(),
            )

    # --- validation -----------------------------------------------------------------------------------------------

    def resolve(self, steps: list[Any], kept: list[QueuedStep]) -> list[QueuedStep]:
        """Validate steps and solve cartesian targets (seeded along the queued arm targets); caller holds the lock.

        Args:
            steps (list[Any]): New steps.
            kept (list[QueuedStep]): Pending steps they follow.

        Returns:
            list[QueuedStep]: Items (job ids assigned by the caller).

        Raises:
            QueueError: With every reason when any step is invalid.
        """
        arm = self.robot.arm
        nav_max = self.cfg.timeouts.nav_max_timeout_s
        chain: dict[str, float] | None = None
        for item in kept:
            if item.targets is not None:
                chain = (chain or {}) | item.targets
        reasons: list[str] = []
        items: list[QueuedStep] = []
        for index, step in enumerate(steps):
            targets: dict[str, float] | None = None
            try:
                if step.kind in ARM_KINDS:
                    arm.velocity_for(step.speed_scale)
                if step.kind == "arm_joints":
                    arm.validate_targets(step.targets)
                    targets = dict(step.targets)
                elif step.kind == "arm_cartesian":
                    targets = self.solve(step, chain)
                elif step.kind in BASE_KINDS:
                    if step.timeout_s is not None and step.timeout_s > nav_max:
                        raise QueueError(f"timeout_s must be <= {nav_max} s")
                elif step.kind == "grasp" and self.grasp_runner is None:
                    raise QueueError("grasp steps are not available on this server")
                if step.precondition.type == "arm_near":
                    arm.validate_targets(step.precondition.joints or {})
            except (ArmError, UnreachableError, QueueError, ValueError) as exc:
                reasons.append(f"step {index} ({step.kind}): {exc}")
            if targets is not None:
                chain = (chain or {}) | targets
            items.append(QueuedStep("", step, targets))
        if not reasons:
            reasons = self.body_reasons(items, kept)
        if reasons:
            raise QueueError("nothing queued: " + "; ".join(reasons))
        return items

    def body_reasons(self, items: list[QueuedStep], kept: list[QueuedStep]) -> list[str]:
        """Enqueue-time robot body check of the new arm blend groups, from the pose the queue will be in.

        The start is the current commanded pose plus the targets of the pending arm steps; each blend group is checked
        as the move_blend it becomes. A grasp step leaves the arm pose unknown, so groups after it are only checked when
        they start (the worker's motion re-checks every group from its actual start pose anyway).

        Args:
            items (list[QueuedStep]): New items (targets resolved).
            kept (list[QueuedStep]): Pending items they follow.

        Returns:
            list[str]: Self-collision reasons (empty = clear, or no fresh joint states to check from).
        """
        arm = self.robot.arm
        sample = arm.fresh_sample()
        if sample is None or any(q.step.kind == "grasp" for q in kept):
            return []
        pose = arm.command_base(sample)
        for item in kept:
            pose = pose | (item.targets or {})
        reasons: list[str] = []
        index = {id(q): i for i, q in enumerate(items)}
        for group in group_queued(items, self.cfg):
            if group[0].step.kind == "grasp":
                break
            if group[0].step.kind not in ARM_KINDS:
                continue
            try:
                arm.preview_blend([q.targets or {} for q in group], group[0].step.speed_scale, pose)
            except SelfCollisionError as exc:
                where = ", ".join(str(index[id(q)]) for q in group)
                reasons.append(f"step {where} ({group[0].step.kind}): {exc}")
            for q in group:
                pose = pose | (q.targets or {})
        return reasons

    def solve(self, step: ArmCartesianStep, chain: dict[str, float] | None) -> dict[str, float]:
        """IK of a cartesian step seeded with the previous queued arm target (or the current commanded pose).

        Args:
            step (ArmCartesianStep): The step.
            chain (dict[str, float] | None): Joint targets of the queued arm steps before it, merged.

        Returns:
            dict[str, float]: Joint targets (measured space).

        Raises:
            QueueError: Without fresh joint states.
            UnreachableError: When IK has no solution within the limits.
        """
        arm = self.robot.arm
        sample = arm.fresh_sample()
        if sample is None:
            raise QueueError("no fresh joint states: cannot solve the cartesian target")
        seed = arm.command_base(sample) | (chain or {})
        return arm.solve_cartesian(step.x, step.y, step.z, step.pitch, seed, step.wrist_roll, step.object_width_m)

    # --- worker ---------------------------------------------------------------------------------------------------

    def ensure_worker_locked(self) -> None:
        """Start the worker and the watcher thread once (caller holds the lock)."""
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self.run_worker, name="motion-queue", daemon=True)
            self._thread.start()
        if self.sampler is not None and (self._watcher is None or not self._watcher.is_alive()):
            self._watcher = threading.Thread(target=self.run_watcher, name="motion-watch", daemon=True)
            self._watcher.start()

    def run_worker(self) -> None:
        """Worker loop: take the next blend group, dispatch it, publish its outcome."""
        while True:
            with self._cond:
                while not self._pending or self._paused is not None:
                    self._cond.wait()
                group = self.take_group_locked()
                generation = self._generation
                self._running = Running(group=group, started=self.clock())
                self._cond.notify_all()  # wakes the watcher when this group has watches
            try:
                outcome = self.dispatch(group, generation)
            except Exception as exc:  # noqa: BLE001 - a worker crash must never wedge the queue silently
                LOGGER.exception("motion queue step crashed")
                outcome = Outcome(ok=False, status="error", message=f"internal error: {exc}")
            with self._cond:
                self.finish_locked(group, generation, outcome)

    def take_group_locked(self) -> list[QueuedStep]:
        """Pop the next blend group (caller holds the lock).

        Returns:
            list[QueuedStep]: One or more steps.
        """
        group = [self._pending.popleft()]
        while self._pending and can_blend(group[-1], self._pending[0], self.cfg):
            group.append(self._pending.popleft())
        MOTION_QUEUE_DEPTH.set(len(self._pending))
        return group

    def stop_seen(self) -> bool:
        """Whether a stop happened outside the queue since the baseline (re-baselines after the queue's own stop).

        Returns:
            bool: True when stop_count moved.
        """
        count = self.robot.stop_count()
        with self._cond:
            if self._stop_baseline is None:
                self._stop_baseline = count
                return False
            return count != self._stop_baseline

    def dispatch(self, group: list[QueuedStep], generation: int) -> "Outcome":
        """Check stop, battery, lease, tilt, guard rules and the precondition, then run the group.

        Args:
            group (list[QueuedStep]): Blend group (one step unless arm steps).
            generation (int): Queue generation at dispatch (a cancel/stop increments it).

        Returns:
            Outcome: What happened.
        """
        if self.stop_seen():
            return Outcome(ok=False, status="stopped", message="stop was called; queue halted", stopped=True)
        if self.guard is not None and self.guard.is_cutoff():
            return Outcome(ok=False, status="battery_cutoff", message=self.guard.rejection_message())
        head = group[0].step
        lease_refusal = self.lease_gate(head.kind)
        if lease_refusal is not None:
            return Outcome(ok=False, status="lease", message=lease_refusal)
        if head.kind in BASE_KINDS:
            refusal = self.tilt_gate()
            if refusal is not None:
                return Outcome(ok=False, status="tilt_guard", message=refusal)
        try:
            for q in group:
                self.guard_gate(q.step.kind, q.step.model_dump())
        except GuardRejection as exc:
            # Always halts (never on_fail='skip'): the later steps were planned assuming this one ran.
            rules = [v.rule for v in exc.verdicts]
            return Outcome(ok=False, status="guard_rule", message=exc.text, data={"rules": rules}, halt=True)
        if head.precondition.type != "none":
            state = self.live_state(head.precondition.type == "base_still")
            ok, why = evaluate_precondition(head.precondition, state, self.cfg)
            if not ok:
                return Outcome(ok=False, status="precondition_failed", message=why, precondition=True)
        with self._cond:
            if generation != self._generation:
                return Outcome(ok=False, status="cancelled", message="cancelled before it started")
            if self._running is not None and head.kind in ARM_KINDS:
                self._running.phase = "planning"
            for q in group:
                self.emit_locked("step_started", q, describe(q.step))
        origin = GUARD_ORIGIN.set(head.kind)  # the guard rule arm phase checks this group's motions by its kind
        try:
            with self.lease_scope(head.kind):
                return self.run_group(group, generation)
        finally:
            GUARD_ORIGIN.reset(origin)

    def run_group(self, group: list[QueuedStep], generation: int) -> "Outcome":
        """Run a dispatched group by its head's kind.

        Args:
            group (list[QueuedStep]): Blend group.
            generation (int): Dispatch generation.

        Returns:
            Outcome: What happened.
        """
        head = group[0].step
        kind = head.kind
        if kind in ARM_KINDS:
            return self.run_arm(group, generation)
        if kind == "gripper":
            return self.run_gripper(head)
        if kind in BASE_KINDS:
            return self.run_base(head)
        if kind == "wait_s":
            return self.run_wait(head, generation)
        return self.run_grasp(head, generation)

    def run_arm(self, group: list[QueuedStep], generation: int) -> "Outcome":
        """One blended arm motion through the group's targets.

        Args:
            group (list[QueuedStep]): Arm steps.
            generation (int): Dispatch generation.

        Returns:
            Outcome: Result of the motion.
        """
        first, last = group[0].step, group[-1].step

        def passed(index: int) -> None:
            with self._cond:
                if self._running is None or generation != self._generation:
                    return
                self._running.passed = index + 1
                if index < len(group) - 1:
                    item = group[index]
                    self._running.done.add(item.job_id)
                    MOTION_STEPS.labels(item.step.kind, "blended").inc()
                    self.emit_locked("step_done", item, "passed (blended, no stop)", {"blended": True})

        def ticked(published: int, total: int) -> None:
            with self._cond:
                running = self._running
                if running is None or generation != self._generation:
                    return
                running.ticks, running.total_ticks = published, total
                if running.phase == "planning":
                    running.phase = "executing"
                    planning_s = round(self.clock() - running.started, 3)
                    for item in group:
                        self.emit_locked("planned", item, f"planned in {planning_s} s", {"planning_s": planning_s})

        try:
            result = self.robot.arm.move_blend(
                [q.targets or {} for q in group],
                first.speed_scale,
                step_floor(first),
                resolve_settle(last, self.cfg, group[-1].targets),
                on_via=passed,
                on_tick=ticked,
            )
        except SelfCollisionError as exc:
            return Outcome(ok=False, status="self_collision", message=str(exc), data={"self_collision": exc.details})
        except ArmError as exc:
            return Outcome(ok=False, status="refused", message=str(exc))
        data = arm_result_data(result)
        return Outcome(
            ok=result.status in ARM_OK,
            status=result.status,
            message=result.message,
            data=data,
            stopped=result.status == "stopped",
        )

    def run_gripper(self, step: GripperStep) -> "Outcome":
        """Gripper step through ArmController.set_gripper.

        Args:
            step (GripperStep): The step.

        Returns:
            Outcome: Result ('blocked' fails; 'grasped'/'blocked' raise a contact event).
        """
        try:
            result = self.robot.arm.set_gripper(
                step.open_fraction, step.close_until_effort, step.effort_threshold, step_floor(step), step.grip_profile
            )
        except ArmError as exc:
            return Outcome(ok=False, status="refused", message=str(exc))
        return Outcome(
            ok=result.status in ARM_OK,
            status=result.status,
            message=result.message,
            data=arm_result_data(result),
            contact=result.status in CONTACT_STATUSES,
            stopped=result.status == "stopped",
        )

    def run_base(self, step: BaseRelativeStep | NavigateStep) -> "Outcome":
        """Base step through RobotApi.navigate / move_relative.

        Args:
            step (BaseRelativeStep | NavigateStep): The step.

        Returns:
            Outcome: Result ('canceled' counts as a stop).
        """
        timeout = step.timeout_s or self.cfg.timeouts.nav_default_timeout_s
        try:
            if isinstance(step, NavigateStep):
                nav: NavigationResult = self.robot.navigate(step.x, step.y, step.yaw, step.frame, timeout, step.precise)
            else:
                nav = self.robot.move_relative(step.dx, step.dy, step.dyaw, timeout, step.precise)
        except RobotError as exc:
            return Outcome(ok=False, status="refused", message=str(exc))
        data: dict[str, Any] = {"status": nav.status, "duration_s": nav.duration_s}
        if nav.final_pose is not None:
            data["final_pose"] = nav.final_pose.model_dump(include={"frame", "x", "y", "yaw"})
        if nav.interrupted_by:
            data["interrupted_by"] = nav.interrupted_by
        return Outcome(
            ok=nav.status in NAV_OK,
            status=nav.status,
            message=nav.message,
            data=data,
            stopped=nav.status in NAV_STOPPED,
        )

    def run_wait(self, step: WaitStep, generation: int) -> "Outcome":
        """Pause, ending early on cancel/stop.

        Args:
            step (WaitStep): The step.
            generation (int): Dispatch generation.

        Returns:
            Outcome: Done, or stopped.
        """
        end = self.clock() + step.seconds
        while True:
            remaining = end - self.clock()
            if remaining <= 0.0:
                return Outcome(ok=True, status="done", message=f"waited {step.seconds:g} s")
            with self._cond:
                if generation != self._generation:
                    return Outcome(ok=False, status="cancelled", message="wait ended by cancel/stop")
                if self._running is not None:
                    self._running.expected_s = step.seconds
            if self.stop_seen():
                return Outcome(ok=False, status="stopped", message="stop was called", stopped=True)
            self.sleep(min(self.settings.poll_s, remaining))

    def run_grasp(self, step: GraspMotionStep, generation: int) -> "Outcome":
        """Grasp step through the grasp runner (GraspExecutor.grasp).

        Args:
            step (GraspMotionStep): The step.
            generation (int): Dispatch generation (a cancel/stop makes stop_requested true).

        Returns:
            Outcome: 'grasped' succeeds (contact event); unverified (holding, needs a look: always halts the queue,
                also with on_fail='skip')/missed/aborted/infeasible fail.
        """
        assert self.grasp_runner is not None  # resolve() refuses grasp steps without a runner

        def stop_requested() -> bool:
            with self._cond:
                if generation != self._generation:
                    return True
            return self.stop_seen()

        try:
            outcome, reasons = self.grasp_runner(step, stop_requested)
        except (ArmError, ValueError) as exc:
            return Outcome(ok=False, status="refused", message=str(exc))
        return Outcome(
            ok=outcome in GRASP_OK,
            status=outcome,
            message="; ".join(reasons),
            data={"outcome": outcome, "reasons": reasons},
            contact=outcome == "grasped",
            halt=outcome == "unverified",
        )

    def finish_locked(self, group: list[QueuedStep], generation: int, outcome: "Outcome") -> None:
        """Publish a group's outcome and apply its failure policy (caller holds the lock).

        Args:
            group (list[QueuedStep]): The group.
            generation (int): Its dispatch generation.
            outcome (Outcome): What happened.
        """
        running = self._running
        done = set() if running is None else running.done
        rest = [q for q in group if q.job_id not in done]
        self._running = None
        self.count_steps(group, rest, outcome)
        if generation != self._generation:
            for q in rest:
                self.emit_locked("step_aborted", q, outcome.message, {"status": outcome.status, **outcome.data})
        elif outcome.ok:
            if outcome.contact:
                self.emit_locked("contact", rest[-1], outcome.message, {"status": outcome.status})
            for q in rest:
                self.emit_locked("step_done", q, outcome.message, outcome.data)
        elif outcome.stopped:
            for q in rest:
                self.emit_locked("step_aborted", q, outcome.message, {"status": outcome.status, **outcome.data})
            self.halt_locked("stopped", outcome.message)
        elif outcome.precondition:
            # Only the head carries the precondition: the steps blended behind it go back to the queue front.
            head = group[0]
            skip = head.step.on_fail == "skip"
            self.emit_locked("precondition_failed", head, outcome.message, {SKIPPED_KEY: "skip"} if skip else None)
            if skip:
                self.emit_locked("step_skipped", head, f"skipped (on_fail=skip): {outcome.message}")
                self._pending.extendleft(reversed(group[1:]))
            else:
                dropped, _ = self.halt_locked(None, outcome.message)
                self.emit_locked(
                    "queue_stopped",
                    head,
                    f"queue stopped: precondition of {head.job_id} failed: {outcome.message}",
                    {"dropped": [q.job_id for q in group[1:]] + dropped},
                )
        else:
            head = rest[0] if rest else group[-1]
            if outcome.contact:
                self.emit_locked("contact", head, outcome.message, {"status": outcome.status})
            policy_skip = not outcome.halt and all(q.step.on_fail == "skip" for q in rest)
            failed = {"status": outcome.status, **outcome.data}
            if policy_skip:
                failed[SKIPPED_KEY] = "skip"
            self.emit_locked("step_failed", head, outcome.message, failed)
            if policy_skip:
                for q in rest:
                    self.emit_locked("step_skipped", q, f"skipped (on_fail=skip): {outcome.message}")
            else:
                dropped, _ = self.halt_locked(None, outcome.message)
                self.emit_locked(
                    "queue_stopped",
                    head,
                    f"queue stopped after {head.job_id} failed: {outcome.message}",
                    {"dropped": [q.job_id for q in rest[1:]] + dropped},
                )
        MOTION_QUEUE_DEPTH.set(len(self._pending))
        self.prune_watches_locked()
        if not self._pending and self._paused is None:
            self.emit_locked("queue_empty", message="queue drained")
        self._cond.notify_all()

    # --- watch constraints ----------------------------------------------------------------------------------------

    def run_watcher(self) -> None:
        """Watcher loop ('motion-watch' thread): sleep on the queue condition while no watch set is active, else
        tick at motion_queue.watch_rate_hz. Never runs inside the arm setpoint stream."""
        period = 1.0 / self.settings.watch_rate_hz
        while True:
            with self._cond:
                while not self.watch_due_locked():
                    self._cond.wait()
            self.watch_sleep(period)
            try:
                self.watch_tick()
            except Exception:  # noqa: BLE001 - a broken sampler must not end the watcher silently
                LOGGER.exception("motion watch tick failed")

    def watch_due_locked(self) -> bool:
        """Whether a watch that has not triggered is bound to the running group (caller holds the lock).

        Returns:
            bool: True when the watcher should tick.
        """
        if self._running is None:
            return False
        ids = {q.job_id for q in self._running.group}
        return any(not w.triggered and w.ids & ids for w in self._watches)

    def activate_watches_locked(self, running: Running | None) -> list[WatchState]:
        """Update the active flags from the running group (caller holds the lock).

        A watch leaving the active state stops integrating distance (no speed gap is integrated across it).

        Args:
            running (Running | None): The running group.

        Returns:
            list[WatchState]: The active watches that have not triggered.
        """
        ids = set() if running is None else {q.job_id for q in running.group}
        active: list[WatchState] = []
        for w in self._watches:
            w.active = bool(w.ids & ids)
            if not w.active:
                w.speed_at = None
            elif not w.triggered:
                active.append(w)
        return active

    def sample(self, metric: str, joint: str | None) -> float | None:
        """One cached metric value (the sampler reads caches only; called outside the queue lock).

        Args:
            metric (str): Watch metric (distance_travelled_m is not sampled, see watch_value).
            joint (str | None): Joint of the joint metrics.

        Returns:
            float | None: The value, None when unknown.
        """
        assert self.sampler is not None  # enqueue refuses watches without a sampler
        return self.sampler(metric, joint)

    @staticmethod
    def watch_value(w: WatchState, samples: dict[tuple[str, str | None], float | None], now: float) -> float | None:
        """A watch's value this tick; integrates distance_travelled_m from the fresh speed (watcher only).

        Args:
            w (WatchState): The watch.
            samples (dict[tuple[str, str | None], float | None]): This tick's samples by (metric, joint).
            now (float): Tick time (s).

        Returns:
            float | None: The value; None when unknown (distance: while the speed is unknown, the gap is not
                integrated).
        """
        c = w.constraint
        if c.metric != DISTANCE_METRIC:
            return samples[(c.metric, c.joint)]
        speed = samples[(SPEED_METRIC, None)]
        if speed is None:
            w.speed_at = None
            return None
        if w.speed_at is not None:
            w.distance += abs(speed) * max(0.0, now - w.speed_at)
        w.speed_at = now
        return w.distance

    @staticmethod
    def watch_held(w: WatchState, value: float | None, now: float) -> bool:
        """Advance the for_s debounce with this tick's value.

        Args:
            w (WatchState): The watch.
            value (float | None): Its value (None = unknown).
            now (float): Tick time (s).

        Returns:
            bool: True when the violation held for for_s (trip).
        """
        violated = watch_violated(w.constraint, value)
        if violated is None:  # unknown, ignored: neither a violation nor the end of one
            return False
        if not violated:
            w.since = None
            return False
        if w.since is None:
            w.since = now
        return now - w.since >= w.constraint.for_s - WATCH_TIME_EPS

    def watch_tick(self) -> None:
        """One watcher evaluation: sample the caches outside the lock, then compare and trip under it.

        The trip is generation-checked: when the running group ended (or was cancelled) while sampling, the late
        trip is dropped, so a step that just finished is never aborted.
        """
        with self._cond:
            running = self._running
            generation = self._generation
            active = self.activate_watches_locked(running)
        if not active:
            return
        keys = {
            (SPEED_METRIC, None)
            if w.constraint.metric == DISTANCE_METRIC
            else (w.constraint.metric, w.constraint.joint)
            for w in active
        }
        samples = {key: self.sample(*key) for key in keys}
        now = self.clock()
        with self._cond:
            if running is None or self._running is not running or generation != self._generation:
                return
            trip: tuple[WatchState, float | None] | None = None
            for w in active:
                if w.triggered:
                    continue
                value = self.watch_value(w, samples, now)
                w.last_value = value
                if not self.watch_held(w, value, now):
                    continue
                if trip is None or (trip[0].constraint.action == "pause" and w.constraint.action == "abort"):
                    trip = (w, value)  # abort wins over pause in the same tick
            if trip is None:
                return
            data, moving = self.trip_locked(trip[0], trip[1], running)
        MOTION_WATCH_TRIGGERS.labels(data["metric"], data["action"]).inc()
        if moving:
            self.stop_robot()
        try:
            self.on_watch_trigger(data)
        except Exception:  # noqa: BLE001 - the report must not break the watcher; the queue already acted
            LOGGER.exception("watch trigger report failed")

    def trip_locked(self, w: WatchState, value: float | None, running: Running) -> tuple[dict[str, Any], bool]:
        """Act on a tripped watch (caller holds the lock): emit watch_triggered, then abort (the cancel path) or
        pause (end the running group without re-running it, keep the pending steps, block dispatch).

        Args:
            w (WatchState): The watch.
            value (float | None): Its value (None = unknown).
            running (Running): The running group.

        Returns:
            tuple[dict[str, Any], bool]: watch_triggered data, and whether the robot must be stopped (outside the
                lock).
        """
        c = w.constraint
        w.triggered = True
        shown = None if value is None else round(value, DIGEST_DECIMALS)
        data = {
            "watch": c.label,
            "metric": c.metric,
            "value": shown,
            "op": c.op,
            "threshold": c.value,
            "action": c.action,
        }
        item = running.group[min(running.passed, len(running.group) - 1)]
        moving = self.moving_locked()
        self.emit_locked(
            "watch_triggered",
            item,
            f"watch '{c.label}' triggered: {c.metric}={'unknown' if shown is None else shown} {c.op} {c.value:g} "
            f"-> {c.action}",
            data,
        )
        if c.action == "abort":
            self.halt_locked("cancelled", f"watch '{c.label}' triggered")
        else:
            self._generation += 1  # the running group ends as step_aborted and is not re-run
            self._paused = PauseView(by=c.label, at=round(time.time(), 3))
            self.emit_locked(
                "paused",
                item,
                f"paused by watch '{c.label}': resume_motions continues the pending steps, cancel_motions drops them",
                {"watch": c.label, "pending": [q.job_id for q in self._pending]},
            )
            self._cond.notify_all()
        return data, moving

    # --- helpers --------------------------------------------------------------------------------------------------

    @staticmethod
    def count_steps(group: list[QueuedStep], rest: list[QueuedStep], outcome: "Outcome") -> None:
        """Count the steps a group ended (mcp_motion_steps_total) and its tracking error.

        A failed precondition ends only the group's head (the steps blended behind it go back to the queue).

        Args:
            group (list[QueuedStep]): The finished group.
            rest (list[QueuedStep]): Its steps that did not end earlier as a blended pass-through.
            outcome (Outcome): What happened.
        """
        ended = group[:1] if outcome.precondition else rest
        for q in ended:
            MOTION_STEPS.labels(q.step.kind, outcome.status).inc()
        error = outcome.data.get("tracking_error_rad")
        if error is not None:
            MOTION_TRACKING_ERROR.labels(group[-1].step.kind).observe(error)

    def busy_locked(self) -> bool:
        """busy() with the lock held.

        Returns:
            bool: True while busy.
        """
        return self._running is not None or bool(self._pending) or self._paused is not None

    def moving_locked(self) -> bool:
        """Whether the running group moves the robot (a wait step does not).

        Returns:
            bool: True when a robot.stop is needed to end it.
        """
        return self._running is not None and any(q.step.kind != "wait_s" for q in self._running.group)

    def stop_robot(self) -> None:
        """Stop the robot for a queue-initiated abort (caller does NOT hold the lock) and reset the stop baseline."""
        self.robot.stop()
        with self._cond:
            self._stop_baseline = None

    def prune_watches_locked(self) -> None:
        """Drop the watch sets none of whose steps is pending or running any more (caller holds the lock)."""
        live = {q.job_id for q in self._pending}
        if self._running is not None:
            live |= {q.job_id for q in self._running.group}
        self._watches = [w for w in self._watches if w.ids & live]

    def halt_locked(self, event: EventType | None, message: str) -> tuple[list[str], list[str] | None]:
        """Drop the pending steps, clear a pause and bump the generation (the running group's outcome is then ignored).

        Args:
            event (EventType | None): Event to emit ('cancelled' / 'stopped'), None for none.
            message (str): Event message.

        Returns:
            tuple[list[str], list[str] | None]: Dropped job ids and the running group's ids (None when idle).
        """
        dropped = [q.job_id for q in self._pending]
        self._pending.clear()
        self._paused = None
        self.prune_watches_locked()
        running = None if self._running is None else [q.job_id for q in self._running.group]
        MOTION_QUEUE_DEPTH.set(0)
        if running is not None or dropped:
            self._generation += 1
        if event is not None:
            self.emit_locked(event, message=message, data={"dropped": dropped, "running": running})
        self._cond.notify_all()
        return dropped, running

    def emit_locked(
        self, type_: EventType, item: QueuedStep | None = None, message: str = "", data: dict[str, Any] | None = None
    ) -> None:
        """Append an event (caller holds the lock).

        Args:
            type_ (EventType): Event type.
            item (QueuedStep | None): Step it is about.
            message (str): Text.
            data (dict[str, Any] | None): Details.
        """
        self._seq += 1
        event = MotionEvent(
            seq=self._seq,
            t=round(time.time(), 3),
            type=type_,
            job_id=None if item is None else item.job_id,
            kind=None if item is None else item.step.kind,
            label=None if item is None else item.step.label,
            message=message,
            data=data or {},
        )
        self._events.append(event)
        LOGGER.info("motion_event %s", event.model_dump_json(exclude_none=True))

    def status_locked(self) -> MotionStatus:
        """status() with the lock held.

        Returns:
            MotionStatus: Snapshot.
        """
        current = None
        running = self._running
        if running is not None:
            index = min(running.passed, len(running.group) - 1)
            item = running.group[index]
            elapsed = self.clock() - running.started
            progress = None
            if running.total_ticks:
                progress = running.ticks / running.total_ticks
            elif running.phase == "planning":
                progress = 0.0
            elif running.expected_s:
                progress = min(1.0, elapsed / running.expected_s)
            current = CurrentStepView(
                job_id=item.job_id,
                kind=item.step.kind,
                label=item.step.label,
                summary=describe(item.step),
                elapsed_s=round(elapsed, 3),
                phase=running.phase,
                planning_s=round(elapsed, 3) if running.phase == "planning" else None,
                progress=None if progress is None else round(progress, 3),
                blend_group=[q.job_id for q in running.group] if len(running.group) > 1 else [],
            )
        return MotionStatus(
            running=self.busy_locked(),
            current=current,
            queue=[
                QueuedStepView(job_id=q.job_id, kind=q.step.kind, label=q.step.label, summary=describe(q.step))
                for q in self._pending
            ],
            last_events=list(self._events)[-self.settings.status_events :],
            event_seq=self._seq,
            paused=self._paused,
            watches=[w.view() for w in self._watches],
        )


@dataclass(frozen=True)
class Outcome:
    """Result of one dispatched group."""

    ok: bool
    status: str
    message: str = ""
    data: dict[str, Any] = field(default_factory=dict)
    contact: bool = False
    stopped: bool = False
    precondition: bool = False
    halt: bool = False  # always stop the queue, whatever on_fail says (an unverified grasp may hold the object)
