"""Per-subsystem control leases: take_control / release_control / get_control_state and the lease check.

The arm lease is the filter_node autonomy lease (ArmController.acquire / release): the hardware arbitration honours
it. The base lease is a flag inside mcp_server (nothing downstream arbitrates /cmd_vel_nav; humans can still drive).
Every arm or base tool is refused without its lease (tools.TOOL_SUBSYSTEM, checked centrally before the tool body).
Releasing resets by default: the arm drives to its stored home pose, the base is zeroed (Nav2 cancelled, zero twist)
and the steering is verified neutral. A failed reset is reported, but the release always happens.
"""

import threading
import time
from collections.abc import Callable, Iterable
from typing import Literal, Protocol

from mcp.server.mcpserver.exceptions import ToolError
from ros2_common.battery import BatteryGuard

from .arm import ArmController, ArmError
from .config import McpServerConfig
from .models import ArmControlState, BaseControlState, ControlState, ReleaseResult
from .motion_queue import ARM_KINDS, BASE_KINDS

Subsystem = Literal["arm", "base"]
ReleaseTarget = Literal["arm", "base", "all"]
# A tool's lease need: one subsystem, or "any" (either lease satisfies it; for future head tools, so a base-to-arm
# hand-over needs no head reset).
LeaseNeed = Literal["arm", "base", "any"]
# enqueue_motions step kinds that need the arm lease; base steps (Nav2 goals) are motion_queue.BASE_KINDS.
ARM_LEASE_KINDS = ARM_KINDS | {"gripper", "grasp"}
SUBSYSTEMS: tuple[Subsystem, ...] = ("arm", "base")
RELEASE_ORDER: tuple[Subsystem, ...] = ("base", "arm")  # base first: stop the wheels before the arm moves home
QUEUE_BUSY_REFUSAL = "release_control refused: the motion queue is running - cancel_motions or wait_for_event first"
NOT_HELD = "skipped: not held"


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


class ControlManager:
    """Both control leases: take, release (with the reset), state, the lease checks and the vitals digest."""

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
    ) -> None:
        """Create the manager.

        Args:
            arm (ArmController): Arm controller (owns the arm lease).
            base (BaseOps): Base zeroing and steer joint feed.
            config (McpServerConfig): Node configuration (control section).
            guard (BatteryGuard | None): Battery guard; under cut-off the arm reset is skipped.
            queue_busy (Callable[[], bool]): Whether the motion queue runs (release is refused meanwhile).
            clock (Callable[[], float]): Monotonic clock for the steering wait (s).
            sleep (Callable[[float], None]): Sleep for the steering wait (s).
            wall (Callable[[], float]): Wall clock for held_since (s since the epoch).
        """
        self.arm = arm
        self.base = base
        self.cfg = config.control
        self.guard = guard
        self.queue_busy = queue_busy
        self.clock = clock
        self.sleep = sleep
        self.wall = wall
        self.lock = threading.Lock()
        self.base_since: float | None = None  # wall time the base lease was taken, None while not held

    @property
    def base_held(self) -> bool:
        """Whether the base lease is held.

        Returns:
            bool: True while held.
        """
        return self.base_since is not None

    def held(self, subsystem: Subsystem) -> bool:
        """Whether one lease is held.

        Args:
            subsystem (Subsystem): arm or base.

        Returns:
            bool: True while held.
        """
        return self.arm.control_held if subsystem == "arm" else self.base_held

    def take(self, subsystem: Subsystem) -> ControlState:
        """Take one lease; a no-op (keeping held_since) when it is already held. Moves nothing.

        Args:
            subsystem (Subsystem): arm (commands the measured pose, needs fresh joint states) or base.

        Returns:
            ControlState: Both leases afterwards.
        """
        if subsystem == "arm":
            if not self.arm.control_held:
                self.arm.acquire()
        else:
            with self.lock:
                if self.base_since is None:
                    self.base_since = self.wall()
        return self.state()

    def release(self, subsystem: ReleaseTarget, keep_position: bool = False) -> ReleaseResult:
        """Release leases, resetting each by default. The release always happens; a failed reset is reported.

        Args:
            subsystem (ReleaseTarget): arm, base or all.
            keep_position (bool): True skips the reset (arm keeps its last goal, base only clears the flag).

        Returns:
            ReleaseResult: Released subsystems, per subsystem reset outcome and the state afterwards.

        Raises:
            ToolError: While the motion queue runs (nothing is released).
        """
        if self.queue_busy():
            raise ToolError(QUEUE_BUSY_REFUSAL)
        targets = RELEASE_ORDER if subsystem == "all" else (subsystem,)
        released: list[str] = []
        reset: dict[str, str] = {}
        for name in targets:
            outcome = self.release_base(keep_position) if name == "base" else self.release_arm(keep_position)
            reset[name] = outcome
            if not outcome.startswith(NOT_HELD):
                released.append(name)
        return ReleaseResult(released=released, reset=reset, state=self.state())

    def release_arm(self, keep_position: bool) -> str:
        """Release the arm lease, homing first unless kept, skipped under battery cut-off or no home is stored.

        Args:
            keep_position (bool): Release only.

        Returns:
            str: homed | kept | failed: <why> | skipped: <why>.
        """
        if not self.arm.control_held:
            return NOT_HELD if self.arm.lost_note is None else f"{NOT_HELD} ({self.arm.lost_note})"
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
        if self.arm.control_held:
            self.arm.release()
        return outcome

    def release_base(self, keep_position: bool) -> str:
        """Release the base lease, zeroing the base and waiting for neutral steering unless kept.

        Args:
            keep_position (bool): Clear the flag only.

        Returns:
            str: neutral | kept | failed: <why> | skipped: not held.
        """
        with self.lock:
            if self.base_since is None:
                return NOT_HELD
        try:
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
        finally:
            with self.lock:
                self.base_since = None

    def state(self) -> ControlState:
        """Both leases now.

        Returns:
            ControlState: Arm and base lease state.
        """
        src = self.arm.backend.active_source()
        arm = ArmControlState(
            held=self.arm.control_held,
            held_since=round(self.arm.held_since, 3) if self.arm.control_held and self.arm.held_since else None,
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
        return ControlState(arm=arm, base=base)

    def refusal(self, tool_name: str, need: LeaseNeed) -> str | None:
        """Refusal text for a tool whose lease is not held.

        Args:
            tool_name (str): Tool being called (named in the text).
            need (LeaseNeed): arm, base or any (either lease).

        Returns:
            str | None: The refusal, None when the lease is held.
        """
        if need == "any":
            if self.arm.control_held or self.base_held:
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
        arm = "H" if self.arm.control_held else "-"
        return f"ctl arm={arm}{where} base={'H' if self.base_held else '-'}"
