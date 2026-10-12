"""Web-UI arm jog: small, safety-checked steps of the gripper (action 'jog' on /grasp/command).

A cartesian step moves the tool point along an axis of the gripper frame:

- forward / back: along the approach axis (the gripper's pointing direction, pitch included)
- left / right: horizontal, perpendicular to the approach heading (wrist roll is ignored, so left stays level)
- up / down: perpendicular to both (world up when the gripper points horizontally)

It starts from the commanded tool pose (ArmState.tool_pose when nothing was commanded in this lease, so gravity sag is
never locked in), keeps the approach pitch and goes through ArmController.solve_cartesian and move_cartesian, so the
reach (IK unreachable), self-collision and floor slow-zone checks apply; a step whose tool point would end below
arm.floor_z_m + jog.floor_clearance_m is refused as well. Joint steps (shoulder_pan, wrist_pitch = wrist_flex,
wrist_roll) go through move_joints after a joint-limit and body check; gripper steps are open-fraction steps through
set_gripper. Every step is capped by the jog config section (a larger request is clamped to the cap).

Jogs coalesce: while one runs, a newer request replaces the pending one (the replaced request is answered at once as
superseded), so at most one jog waits and the last request wins.
"""

import logging
import math
import threading
from collections.abc import Callable
from typing import Any, Literal, Protocol

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .arm import ROLL_JOINT, ArmController, ArmError
from .config import JogSettings
from .ik import ArmKinematics, UnreachableError
from .models import ArmMotionResult

LOGGER = logging.getLogger("mcp_server.jog")

JogKind = Literal["cartesian", "shoulder_pan", "wrist_pitch", "wrist_roll", "gripper"]
JogDirection = Literal["up", "down", "forward", "back", "left", "right", "open", "close"]
# Directions each kind accepts.
JOG_DIRECTIONS: dict[str, tuple[str, ...]] = {
    "cartesian": ("up", "down", "forward", "back", "left", "right"),
    "shoulder_pan": ("left", "right"),
    "wrist_pitch": ("up", "down"),
    "wrist_roll": ("left", "right"),
    "gripper": ("open", "close"),
}
# Joint moved by each joint kind (measured follower joint names).
JOG_JOINTS = {"shoulder_pan": "shoulder_pan", "wrist_pitch": "wrist_flex", "wrist_roll": ROLL_JOINT}
# Gripper-frame axis and sign of each cartesian direction.
CARTESIAN_AXES = {
    "forward": ("forward", 1.0),
    "back": ("forward", -1.0),
    "left": ("left", 1.0),
    "right": ("left", -1.0),
    "up": ("up", 1.0),
    "down": ("up", -1.0),
}
PROBE_RAD = 0.01  # joint probe for the sign of a joint direction
HORIZONTAL_EPS = 1e-3  # approach vectors with a smaller horizontal part point straight up/down
MOVED_EPS = 1e-4  # a joint target closer than this to the start does not move (at a limit)
GRIPPER_EPS = 1e-3  # open-fraction change below this does not move (fully open / closed)
SUPERSEDED = "superseded by a newer jog (jogs coalesce: the last request wins)"
STOPPED = "dropped by stop before it ran"


class CutoffCheck(Protocol):
    """Battery cut-off guard (battery_gate.CutoffGuard)."""

    def is_cutoff(self) -> bool:
        """Whether motion is refused."""
        ...

    def rejection_message(self) -> str:
        """Why motion is refused."""
        ...


class JogSpec(BaseModel):
    """One jog step: what moves, which way and, optionally, how far (m, rad or open fraction; capped)."""

    model_config = ConfigDict(extra="forbid")

    kind: JogKind
    direction: JogDirection
    step: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def direction_fits_kind(self) -> "JogSpec":
        """Reject a direction the kind does not have.

        Returns:
            JogSpec: The validated spec.
        """
        allowed = JOG_DIRECTIONS[self.kind]
        if self.direction not in allowed:
            raise ValueError(f"{self.kind} jogs {', '.join(allowed)}, not {self.direction!r}")
        return self


class JogRefused(Exception):
    """A jog refused before anything moved; the message is the reason."""


def heading_of(kin: ArmKinematics, joints: dict[str, float], approach: np.ndarray) -> float:
    """Horizontal heading of the gripper (rad, base_link): the approach heading, or for a gripper pointing straight up
    or down the bearing of the tool point around the shoulder_pan axis.

    Args:
        kin (ArmKinematics): Arm kinematics.
        joints (dict[str, float]): Joint name -> measured rad.
        approach (np.ndarray): Unit approach vector.

    Returns:
        float: Heading (rad).
    """
    if math.hypot(float(approach[0]), float(approach[1])) > HORIZONTAL_EPS:
        return math.atan2(float(approach[1]), float(approach[0]))
    pose = kin.forward(joints)
    px, py = kin.pan_axis_xy
    return math.atan2(pose.y - py, pose.x - px)


def gripper_axes(kin: ArmKinematics, joints: dict[str, float]) -> dict[str, np.ndarray]:
    """Unit axes of the jog's gripper frame in base_link.

    Args:
        kin (ArmKinematics): Arm kinematics.
        joints (dict[str, float]): Joint name -> measured rad.

    Returns:
        dict[str, np.ndarray]: forward (approach axis), left (horizontal, perpendicular to the heading) and up
            (forward x left).
    """
    frame = kin.chain.forward_kinematics(kin.to_vector(joints))
    approach = np.asarray(frame[:3, 2], dtype=np.float64)
    forward = approach / float(np.linalg.norm(approach))
    heading = heading_of(kin, joints, forward)
    left = np.array([-math.sin(heading), math.cos(heading), 0.0])
    up = np.cross(forward, left)
    return {"forward": forward, "left": left, "up": up / float(np.linalg.norm(up))}


class Jogger:
    """Runs jog steps one at a time; a jog arriving while one runs replaces the single pending jog."""

    def __init__(
        self,
        arm: ArmController,
        settings: JogSettings,
        guard: CutoffCheck | None,
        publish: Callable[[dict[str, Any]], None],
    ) -> None:
        """Create the jogger.

        Args:
            arm (ArmController): Arm controller.
            settings (JogSettings): Step caps, speed and floor clearance.
            guard (CutoffCheck | None): Battery cut-off guard; None never refuses.
            publish (Callable[[dict[str, Any]], None]): Sink of answers that are not returned by submit (a superseded
                or dropped jog, and every answer of a run that had a pending jog), i.e. the /grasp/result publisher.
        """
        self.arm = arm
        self.settings = settings
        self.guard = guard
        self.publish = publish
        self.lock = threading.Lock()
        self.running = False
        self.pending: tuple[JogSpec, dict[str, Any]] | None = None

    def submit(self, spec: JogSpec, head: dict[str, Any]) -> dict[str, Any] | None:
        """Run a jog now, or make it the pending jog when one runs (replacing and answering the previous pending one).

        The caller that runs a jog also runs the jogs that became pending meanwhile. When none did, its own answer is
        returned; otherwise every answer (its own first) goes to publish and None is returned.

        Args:
            spec (JogSpec): The step.
            head (dict[str, Any]): {request_id, action} echoed in the answer.

        Returns:
            dict[str, Any] | None: The answer, or None when it is (or will be) sent through publish.
        """
        with self.lock:
            if self.running:
                dropped, self.pending = self.pending, (spec, head)
                queued = True
            else:
                self.running, dropped, queued = True, None, False
        if dropped is not None:
            self.publish(dropped[1] | {"ok": False, "error": SUPERSEDED})
        if queued:
            return None
        answer: dict[str, Any] | None = self.run_safely(spec, head)
        while True:
            with self.lock:
                nxt, self.pending = self.pending, None
                if nxt is None:
                    self.running = False
                    return answer
            if answer is not None:
                self.publish(answer)
                answer = None
            self.publish(self.run_safely(*nxt))

    def drop_pending(self) -> None:
        """Answer and forget the pending jog (a stop request)."""
        with self.lock:
            dropped, self.pending = self.pending, None
        if dropped is not None:
            self.publish(dropped[1] | {"ok": False, "error": STOPPED})

    def run_safely(self, spec: JogSpec, head: dict[str, Any]) -> dict[str, Any]:
        """run() that never raises, so the coalescing loop always ends.

        Args:
            spec (JogSpec): The step.
            head (dict[str, Any]): {request_id, action}.

        Returns:
            dict[str, Any]: The answer.
        """
        try:
            return self.run(spec, head)
        except Exception as exc:  # any failure is answered, never leaves the jogger running
            LOGGER.exception("jog failed")
            return head | {"ok": False, "error": f"jog failed: {exc}"}

    def run(self, spec: JogSpec, head: dict[str, Any]) -> dict[str, Any]:
        """Check and run one jog step.

        Args:
            spec (JogSpec): The step.
            head (dict[str, Any]): {request_id, action}.

        Returns:
            dict[str, Any]: {ok: true, result: {kind, direction, step, step_capped, status, message, tool_pose,
                motion}} or {ok: false, error} for a refusal (nothing moved).
        """
        try:
            if self.guard is not None and self.guard.is_cutoff():
                raise JogRefused(self.guard.rejection_message())
            state = self.arm.state()
            if state.tool_pose is None:
                raise JogRefused(
                    "no fresh /follower/joint_states: jog refused until the follower reports its joints again"
                )
            cap = self.cap(spec)
            step = cap if spec.step is None else min(spec.step, cap)
            if spec.kind == "cartesian":
                motion = self.cartesian(spec.direction, step)
            elif spec.kind == "gripper":
                motion = self.gripper(spec.direction, step)
            else:
                motion = self.joint(JOG_JOINTS[spec.kind], spec.direction, step)
        except (JogRefused, ArmError) as exc:  # ArmError covers ArmBusyError, SelfCollisionError and the guards
            return head | {"ok": False, "error": f"jog refused: {exc}"}
        tool_pose = None
        if motion.positions is not None and all(j in motion.positions for j in self.arm.kin.joint_names):
            pose = self.arm.kin.forward(motion.positions)
            tool_pose = {"x": pose.x, "y": pose.y, "z": pose.z, "pitch": pose.pitch}
        result = {
            "kind": spec.kind,
            "direction": spec.direction,
            "step": step,
            "step_capped": spec.step is not None and spec.step > cap,
            "status": motion.status,
            "message": motion.message,
            "tool_pose": tool_pose,
            "motion": motion.model_dump(mode="json"),
        }
        return head | {"ok": True, "result": result}

    def cap(self, spec: JogSpec) -> float:
        """Step cap of a jog kind.

        Args:
            spec (JogSpec): The step.

        Returns:
            float: Cap (m, rad or open fraction).
        """
        if spec.kind == "cartesian":
            return self.settings.max_cartesian_step_m
        if spec.kind == "gripper":
            return self.settings.max_gripper_step
        return self.settings.max_joint_step_rad

    def speed(self) -> float:
        """Jog speed scale, capped to limits.arm_max_speed_scale.

        Returns:
            float: Speed scale.
        """
        return min(self.settings.speed_scale, self.arm.cfg.limits.arm_max_speed_scale)

    def start_pose(self) -> dict[str, float]:
        """Pose a jog starts from: the commanded pose while the lease holds one, else the measured pose.

        Returns:
            dict[str, float]: Every arm joint (measured space).
        """
        return self.arm.command_base(self.arm.require_sample())

    def cartesian(self, direction: str, step: float) -> ArmMotionResult:
        """Move the tool point one step along a gripper-frame axis, keeping the approach pitch.

        Args:
            direction (str): up, down, forward, back, left or right.
            step (float): Capped step (m).

        Returns:
            ArmMotionResult: The motion.

        Raises:
            JogRefused: When the target is below the floor clearance, unreachable or collides.
        """
        arm = self.arm
        base = self.start_pose()
        pose = arm.kin.forward(base)
        axis, sign = CARTESIAN_AXES[direction]
        delta = gripper_axes(arm.kin, base)[axis] * sign * step
        x, y, z = (float(v) for v in np.array([pose.x, pose.y, pose.z]) + delta)
        floor = arm.cfg.arm.floor_z_m + self.settings.floor_clearance_m
        if z < floor and z < pose.z:
            raise JogRefused(
                f"the tool point would end at z {z:.3f} m, below the floor clearance ({floor:.3f} m = floor "
                f"{arm.cfg.arm.floor_z_m:.3f} m + {self.settings.floor_clearance_m} m)"
            )
        try:
            solution = arm.solve_cartesian(x, y, z, pose.pitch, base)
        except UnreachableError as exc:
            raise JogRefused(unreachable_reason(str(exc))) from exc
        arm.preview_blend([solution], self.speed(), base)  # SelfCollisionError before the lease is taken
        motion = arm.move_cartesian(x, y, z, pose.pitch, self.speed())
        if motion.status == "unreachable":
            raise JogRefused(unreachable_reason(motion.message))
        return motion

    def joint(self, joint: str, direction: str, step: float) -> ArmMotionResult:
        """Move one joint one step.

        Args:
            joint (str): Joint name (measured space).
            direction (str): left, right, up or down.
            step (float): Capped step (rad).

        Returns:
            ArmMotionResult: The motion.

        Raises:
            JogRefused: When the joint is at its limit, the roll guard refuses or the target collides.
        """
        arm = self.arm
        base = self.start_pose()
        if joint == ROLL_JOINT:
            opening = max(arm.require_sample().positions[arm.gripper], base.get(arm.gripper, -math.inf))
            limit = arm.cfg.limits.roll_max_gripper_open_rad
            if opening > limit:
                raise JogRefused(
                    f"wrist roll while the gripper is open to {opening:.2f} rad (roll limit {limit} rad): the open "
                    "moving finger can jam against the robot body or an object; close the gripper to about half open "
                    "first"
                )
        target = base[joint] + self.joint_sign(joint, direction, base) * step
        clamped = arm.clamp_targets({joint: target})[joint]
        if abs(clamped - base[joint]) < MOVED_EPS:
            raise JogRefused(f"{joint} is at its joint limit ({base[joint]:.3f} rad); jog the other way")
        arm.preview_blend([{joint: clamped}], self.speed(), base)  # SelfCollisionError before the lease is taken
        return arm.move_joints({joint: clamped}, self.speed())

    def joint_sign(self, joint: str, direction: str, pose: dict[str, float]) -> float:
        """Sign of the joint change that moves in a direction (from the kinematics, so URDF axis signs never matter).

        shoulder_pan left turns the gripper heading to the left (counter-clockwise from above); wrist_pitch up lowers
        the approach pitch (tilts the gripper up); wrist_roll left is + roll.

        Args:
            joint (str): Joint name.
            direction (str): left, right, up or down.
            pose (dict[str, float]): Pose the step starts from.

        Returns:
            float: +1.0 or -1.0.
        """
        kin = self.arm.kin
        if joint == "shoulder_pan":
            probe = pose | {joint: pose[joint] + PROBE_RAD}
            turn = math.remainder(
                heading_of(kin, probe, gripper_axes(kin, probe)["forward"])
                - heading_of(kin, pose, gripper_axes(kin, pose)["forward"]),
                math.tau,
            )
            positive = turn > 0.0
            toward = direction == "left"
        elif joint == "wrist_flex":
            positive = kin.forward(pose | {joint: pose[joint] + PROBE_RAD}).pitch < kin.forward(pose).pitch
            toward = direction == "up"
        else:
            positive, toward = True, direction == "left"
        return 1.0 if positive == toward else -1.0

    def gripper(self, direction: str, step: float) -> ArmMotionResult:
        """Open or close the gripper by one open-fraction step.

        Args:
            direction (str): open or close.
            step (float): Capped open-fraction step.

        Returns:
            ArmMotionResult: The motion.

        Raises:
            JogRefused: When the gripper is already fully open or closed.
        """
        arm = self.arm
        closed, opened = arm.cfg.arm.gripper_closed_rad, arm.cfg.arm.gripper_open_rad
        now = (self.start_pose()[arm.gripper] - closed) / (opened - closed)
        now = min(max(now, 0.0), 1.0)
        target = min(max(now + (step if direction == "open" else -step), 0.0), 1.0)
        if abs(target - now) < GRIPPER_EPS:
            raise JogRefused(f"the gripper is already fully {'open' if direction == 'open' else 'closed'}")
        return arm.set_gripper(open_fraction=target)


def unreachable_reason(message: str) -> str:
    """Refusal text of an IK failure, always naming it unreachable.

    Args:
        message (str): IK message.

    Returns:
        str: The reason.
    """
    return message if message.startswith("unreachable") else f"unreachable: {message}"
