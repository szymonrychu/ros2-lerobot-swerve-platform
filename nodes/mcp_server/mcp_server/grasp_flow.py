"""Guided grasp flow: the phase state machine behind grasp_approach / grasp_close / grasp_lift / grasp_abort.

Pure data and rules (no arm motion): which tool is allowed in which phase, what the agent does next, when a foreign
arm motion invalidates the flow and whether a re-approach is a straight tool line or goes via the pre-grasp."""

import math
import threading
from typing import Literal, Protocol

from mcp.server.mcpserver.exceptions import ToolError
from pydantic import BaseModel

from .floor_guard import FloorOverride
from .grasp import GraspParams, GraspPlan, ObjectSpec
from .grip import ResolvedGrip
from .models import ArmMotionResult

REAPPROACH_STRAIGHT_MAX_M = 0.03  # a re-approach whose grasp point moves this little or less goes in a straight line
REAPPROACH_ANGLE_TOL_RAD = 0.05  # pitch and roll may differ this much for a straight re-approach
FLOW_TOOLS = ("grasp_approach", "grasp_close", "grasp_lift", "grasp_abort")
INVALID_FOREIGN_MOTION = (
    "an arm motion outside the grasp tools (move_arm_*, set_gripper, arm_home, release_object, a queued step, the web "
    "UI) or a stop ran since the last grasp tool"
)
INVALID_LEASE = "the arm lease changed since the last grasp tool"

FlowPhase = Literal["idle", "landed", "closed", "lifted", "interrupted"]

ALLOWED: dict[FlowPhase, tuple[str, ...]] = {
    "idle": ("grasp_approach", "grasp_abort"),
    "landed": ("grasp_close", "grasp_approach", "grasp_abort"),
    "closed": ("grasp_lift", "grasp_approach", "grasp_abort"),
    "lifted": ("grasp_approach", "grasp_abort"),
    "interrupted": ("grasp_abort",),
}


class NextStep(BaseModel):
    """One thing the agent may do next."""

    tool: str
    when: str


NEXT_STEPS: dict[str, list[tuple[str, str]]] = {
    "idle": [("grasp_approach", "to start a grasp: pass the object as for plan_grasp")],
    "landed": [
        ("grasp_close", "the photo shows the object between the open jaws"),
        ("grasp_approach", "the jaws are off target; pass a corrected object position"),
        ("grasp_abort", "the object is not graspable from here"),
    ],
    "closed": [
        (
            "grasp_lift",
            "the photo shows the object between the closed jaws: pass verdict yes (unsure refuses the lift)",
        ),
        (
            "grasp_approach",
            "the jaws closed off target or on nothing; pass a corrected object position (the jaws reopen first)",
        ),
        ("grasp_abort", "give up: the jaws open and the arm backs straight up"),
    ],
    "lifted_holding": [
        (
            "release_object",
            "at the place location, to put the object down (the grasp is done; before driving call release_control "
            "with keep_position and carried_object_radius_m)",
        )
    ],
    "lifted": [
        ("grasp_approach", "the object dropped; pass its new position to try again"),
        ("grasp_abort", "give up: the jaws open and the arm backs straight up"),
    ],
    "interrupted": [
        ("get_body_state", "read the event in interrupted_by first"),
        ("get_arm_state", "check where the arm and jaws stopped"),
        ("grasp_abort", "then open the jaws and back straight up"),
    ],
    "interrupted_holding": [
        ("get_body_state", "read the event in interrupted_by first"),
        ("get_arm_state", "check where the arm and jaws stopped"),
        ("release_object", "the jaws carry an object: put it down (grasp_abort is refused while holding)"),
    ],
}


class FlowArm(Protocol):
    """The arm counters a flow tracks (ArmController satisfies it)."""

    motion_epoch: int
    lease_generation: int


class GraspFlow:
    """State of one guided grasp: phase, what was planned and landed, and the arm counters at the last grasp tool."""

    def __init__(self) -> None:
        """Start idle."""
        self.busy = threading.Lock()
        self.reset()

    def reset(self) -> None:
        """Back to idle with every stored field cleared."""
        self.invalidated: str | None = None
        self.phase: FlowPhase = "idle"
        self.object: ObjectSpec | None = None
        self.strategy: str = "auto"
        self.params: GraspParams | None = None
        self.approach_pitch_deg: float | None = None
        self.floor: FloorOverride | None = None
        self.plan: GraspPlan | None = None
        self.grip: ResolvedGrip | None = None
        self.close: ArmMotionResult | None = None
        self.grasp_id: str | None = None
        self.interrupted_by: str | None = None
        self.epoch: int | None = None
        self.lease: int | None = None

    def current(self, arm: FlowArm) -> FlowPhase:
        """The phase, after dropping a flow whose arm state changed outside the grasp tools.

        Args:
            arm (FlowArm): Arm counters.

        Returns:
            FlowPhase: The phase (idle after an invalidation, with invalidated set to the reason).
        """
        if self.phase != "idle":
            if arm.lease_generation != self.lease:
                self.reset()
                self.invalidated = INVALID_LEASE
            elif arm.motion_epoch != self.epoch:
                self.reset()
                self.invalidated = INVALID_FOREIGN_MOTION
        return self.phase

    def require(self, arm: FlowArm, tool: str) -> FlowPhase:
        """Refuse a tool the phase does not allow.

        Args:
            arm (FlowArm): Arm counters.
            tool (str): The calling grasp tool.

        Returns:
            FlowPhase: The current phase.

        Raises:
            ToolError: When the tool is not allowed in the phase (nothing moved).
        """
        phase = self.current(arm)
        if tool not in ALLOWED[phase]:
            raise ToolError(refusal_text(tool, phase, self.invalidated))
        return phase

    def advance(self, arm: FlowArm, phase: FlowPhase, **fields: object) -> None:
        """Enter a phase after a grasp tool's last motion and record the arm counters.

        Args:
            arm (FlowArm): Arm counters (stored so own motions do not invalidate the flow).
            phase (FlowPhase): The new phase.
            **fields (object): Flow attributes to set (object, plan, params, ...).
        """
        self.phase = phase
        for name, value in fields.items():
            setattr(self, name, value)
        self.epoch = arm.motion_epoch
        self.lease = arm.lease_generation
        self.invalidated = None


def next_steps(phase: FlowPhase, holding: bool) -> list[NextStep]:
    """What the agent may do next.

    Args:
        phase (FlowPhase): Flow phase.
        holding (bool): The jaws carry the object (only matters in the lifted and interrupted phases).

    Returns:
        list[NextStep]: Tools with when to use them.
    """
    key = f"{phase}_holding" if phase in ("lifted", "interrupted") and holding else phase
    return [NextStep(tool=tool, when=when) for tool, when in NEXT_STEPS[key]]


def refusal_text(tool: str, phase: FlowPhase, invalidated: str | None) -> str:
    """Why a flow tool is refused in a phase, and what to call instead.

    Args:
        tool (str): The refused tool.
        phase (FlowPhase): Flow phase.
        invalidated (str | None): Why the flow was dropped to idle, when it was.

    Returns:
        str: The refusal text (nothing moved).
    """
    why = f" ({invalidated})" if invalidated else ""
    nxt = "; ".join(f"{s.tool}: {s.when}" for s in next_steps(phase, holding=False))
    return (
        f"{tool} refused (nothing moved): the grasp flow is in phase '{phase}'{why}; allowed now: "
        f"{', '.join(ALLOWED[phase])}. {nxt}"
    )


def reapproach_route(landed: GraspPlan, new: GraspPlan) -> Literal["straight", "via_pre_grasp"]:
    """Whether a corrected grasp point is reached in a straight tool line or via the pre-grasp.

    Args:
        landed (GraspPlan): Plan the jaws landed with.
        new (GraspPlan): Plan of the corrected object position.

    Returns:
        Literal["straight", "via_pre_grasp"]: straight when both use the same strategy, pitch and roll within
            REAPPROACH_ANGLE_TOL_RAD and the grasp points are at most REAPPROACH_STRAIGHT_MAX_M apart.
    """
    a = next(w for w in landed.waypoints if w.label == "grasp")
    b = next(w for w in new.waypoints if w.label == "grasp")
    same = (
        landed.strategy == new.strategy
        and abs((landed.approach_pitch_rad or 0.0) - (new.approach_pitch_rad or 0.0)) <= REAPPROACH_ANGLE_TOL_RAD
        and abs((landed.wrist_roll_rad or 0.0) - (new.wrist_roll_rad or 0.0)) <= REAPPROACH_ANGLE_TOL_RAD
        and math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)) <= REAPPROACH_STRAIGHT_MAX_M
    )
    return "straight" if same else "via_pre_grasp"
