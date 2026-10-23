"""Guided head-from-arm capture sequence (design D37, D38, D46): the board on the gripper, moved by the trusted arm
through checked poses, the head aimed at it by the node.

Order: `move_to_base` (confirmed; no board) -> the user clips the board, `board_attached` -> tier 0 (wrist_roll) ->
tier 1 (wrist_flex, wrist_roll) -> `allow_big_joints` (elbow_flex, shoulder_lift, shoulder_pan; declined skips tier 2)
-> tier 2 -> back to the base pose -> `holding` -> `park` (confirmed) -> `idle`. Park works with the board on (design
D63, easy retries): the whole path from the measured joints to `arm_sequence.park_pose` is checked with the board
outline (the latest fitted board_mount when a result exists, else the configured one) and `park_board_sag_margin_m`
added to the board floor, body and head clearances; a pass moves there at `speed_scale` and parks through mcp_server's
`release_control(park_calibration_board=true)`, which ignores the holding load of the clamped stick and nothing else.

Every motion goes through RobotClient (mcp_server's guarded tools) and every arm command waits for the FK path check
from the measured joints to the target (`PoseChecker.check_path`); a refusal stops before anything moves. The gripper
target is sent once, with the base pose. Every exit of the sequence (end, stop, tool error, exception, confirm timeout)
stops the still capture and releases the arm with `keep_position=True`: the arm holds a checked pose and mcp_server
keeps the head where it is. The sequence never parks on its own and never sends `carried_object_radius_m`.

Threads (as floor_sweep.py): `move_to_base`, `board_attached`, `allow_big_joints`, `stop`, `park` and `state` run on
HTTP threads; one worker thread runs the sequence or the park. `lock` guards the shared fields and is only held for
short updates (never across a tool call or `blocked`); the worker waits through the injected `sleep` and measures
every wait on the injected `clock`.
"""

import logging
import math
import threading
import time
from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np
from ros2_common.head_rgbd_calibration.arm_pose_check import BOARD_LINK, GRIPPER_JOINT, PoseChecker
from ros2_common.head_rgbd_calibration.chain import UrdfChain, pose_from_xyz_rpy
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES
from ros2_common.head_rgbd_calibration.floor_fit import PAN_JOINT, TILT_JOINT
from ros2_common.head_rgbd_calibration.head_arm_fit import aim_head, board_centre

from head_rgbd_calibration.capture import CaptureController, StillOutcome
from head_rgbd_calibration.config import BIG_JOINTS, NOMINAL_BOARD_RPY, NOMINAL_BOARD_XYZ, NodeConfig
from head_rgbd_calibration.robot_client import RobotClient, ToolResult

LOG = logging.getLogger(__name__)

IDLE = "idle"
MOVING_TO_BASE = "moving_to_base"
WAITING_BOARD = "waiting_board"
CAPTURING = "capturing"
WAITING_BIG_JOINTS = "waiting_big_joints"
RETURNING = "returning_to_base"
HOLDING = "holding"
PARKING = "parking"
POLL_S = 0.05
CONVERGED = "converged"
HOMED = "homed"
MOUNT_TOLERANCE = 1e-6
LIMIT_TOLERANCE_RAD = 1e-9
MAX_QUOTED_ENTRIES = 3
TIER_COUNT = 3
BIG_JOINT_TIER = 2
REASON_STOPPED = "stopped"
MESSAGE_BOARD = "Clip the board to the fixed jaw, then press Board attached"
MESSAGE_PARK = "Press Park (the board may stay on when its park path is clear)"
HOLDING_LOAD_PREFIX = "holding load"  # mcp_server carrying reason of a gripper that clamps something (the board stick)
BASE_JOINTS = (*ARM_JOINT_NAMES, GRIPPER_JOINT)


class SequenceError(RuntimeError):
    """A step is refused or the sequence cannot start (the reason is the message; maps to 400)."""


class Busy(SequenceError):
    """A sequence or park runs, `blocked` names a reason, or the step does not fit the phase (maps to 409)."""


class Stopped(Exception):
    """Raised inside the worker when stop() was called."""


class StepFailed(Exception):
    """Raised inside the worker when a tool call fails, a check refuses or a confirm times out (the message)."""


@dataclass(frozen=True)
class Plan:
    """What one sequence works with, built before anything is called.

    Attributes:
        checker: Pose check on the arm URDF and the applied head URDF.
        head: The applied head chain the head is aimed on.
        poses: (tier, pose) of every scripted pose in order; the index is the `arm_pose` index.
        board_centre: Board centre in gripper_link at the nominal mount (homogeneous, (4,)).
    """

    checker: PoseChecker
    head: UrdfChain
    poses: list[tuple[int, dict[str, float]]]
    board_centre: np.ndarray


def nominal_mount(cfg: NodeConfig) -> bool:
    """Whether `fit.board_mount` is still the shipped hand-measured start (design D47; it gets the provisional margin).

    Args:
        cfg: Node config.

    Returns:
        bool: True when xyz_init and rpy_init equal NOMINAL_BOARD_XYZ / NOMINAL_BOARD_RPY.
    """
    mount = cfg.fit.board_mount
    pairs = zip((*mount.xyz_init, *mount.rpy_init), (*NOMINAL_BOARD_XYZ, *NOMINAL_BOARD_RPY), strict=True)
    return all(math.isclose(a, b, abs_tol=MOUNT_TOLERANCE) for a, b in pairs)


def scripted_poses(cfg: NodeConfig) -> list[tuple[int, dict[str, float]]]:
    """(tier, pose) of tier 0, tier 1 and tier 2 in sequence order.

    Args:
        cfg: Node config (`arm_sequence`).

    Returns:
        list[tuple[int, dict[str, float]]]: The whole pose set, measured joint space.
    """
    seq = cfg.arm_sequence
    return [(tier, dict(pose)) for tier in range(TIER_COUNT) for pose in getattr(seq, f"tier{tier}_poses")]


def outside_head_limits(chain: UrdfChain, pan_deg: float, tilt_deg: float) -> str | None:
    """Why a head command lies outside the URDF head limits, or None when it is inside.

    Args:
        chain: Head chain.
        pan_deg: Pan (deg, + left).
        tilt_deg: Tilt (deg, + down).

    Returns:
        str | None: "<joint> outside the URDF head limits" naming the first joint out, else None.
    """
    for joint, value in ((PAN_JOINT, pan_deg), (TILT_JOINT, tilt_deg)):
        limits = chain.limits(joint)
        if limits is None:
            continue
        angle = math.radians(value)
        if angle < limits[0] - LIMIT_TOLERANCE_RAD or angle > limits[1] + LIMIT_TOLERANCE_RAD:
            return f"{joint} outside the URDF head limits"
    return None


def tool_status(result: ToolResult) -> tuple[str | None, str]:
    """`status` and `message` of a motion result (move_arm_joints, set_head_angles).

    Args:
        result: A successful tool result.

    Returns:
        tuple[str | None, str]: The status (None when the result has none) and the message or the result text.
    """
    data = result.data or {}
    status = data.get("status")
    message = data.get("message") or ("" if status is not None else result.text)
    return (status if isinstance(status, str) else None), str(message)


def carrying_reasons(result: ToolResult) -> list[str]:
    """Why get_control_state says the gripper carries something (empty when it does not).

    Args:
        result: The get_control_state result.

    Returns:
        list[str]: `arm.carrying.reasons`, or a generic reason when it is carrying without one.

    Raises:
        SequenceError: The call failed or the result has no `arm.carrying.carrying`.
    """
    if result.is_error:
        raise SequenceError(f"Park refused: get_control_state failed: {result.text}")
    carrying = ((result.data or {}).get("arm") or {}).get("carrying")
    if not isinstance(carrying, dict) or not isinstance(carrying.get("carrying"), bool):
        raise SequenceError(f"Park refused: get_control_state reports no arm.carrying: {result.text}")
    if not carrying["carrying"]:
        return []
    return [str(reason) for reason in carrying.get("reasons") or []] or ["carrying"]


class Sequencer:
    """Runs one guided head-from-arm capture sequence (or one park) at a time on a worker thread."""

    def __init__(
        self,
        capture: CaptureController,
        robot: RobotClient,
        cfg: NodeConfig,
        head_urdf: Callable[[], str],
        arm_urdf: Callable[[], str],
        blocked: Callable[[], str | None],
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
        fitted_mount: Callable[[str | None], dict[str, list[float]] | None] | None = None,
    ) -> None:
        """Create an idle sequencer (nothing is called until move_to_base or park).

        Args:
            capture: Still capture of the head_arm views (start_still_session, capture_still, stop).
            robot: mcp_server client; every motion goes through it.
            cfg: Node config (`arm_sequence`, `fit`).
            head_urdf: Returns the applied head URDF (`floor_results.base_urdf`): the aim and the head envelope.
            arm_urdf: Returns the arm URDF text.
            blocked: Returns a reason while an RGB-D capture, a compute or a floor sweep forbids a sequence, else None.
            sleep: Sleeps seconds (the injected clock's sleep in tests).
            clock: Monotonic seconds; every wait is measured on it.
            fitted_mount: Given the capture session id, the board mount `{"xyz", "rpy"}` of its latest fit result, or
                None without one; the Park path check uses it instead of `fit.board_mount`.
        """
        self.fitted_mount = fitted_mount or (lambda _session_id: None)
        self.capture = capture
        self.robot = robot
        self.cfg = cfg
        self.seq = cfg.arm_sequence
        self.head_urdf = head_urdf
        self.arm_urdf = arm_urdf
        self.blocked = blocked
        self.sleep = sleep
        self.clock = clock
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.running = False
        self.phase = IDLE
        self.session_id: str | None = None
        self.tier: int | None = None
        self.arm_pose: int | None = None
        self.total_poses = len(scripted_poses(cfg))
        self.captured = 0
        self.skipped: Counter[str] = Counter()
        self.status = IDLE
        self.error: str | None = None
        self.waiting_since: float | None = None
        self.board_on = False
        self.big_joints_allowed: bool | None = None

    def state(self) -> dict[str, Any]:
        """The sequence state for the HTTP API.

        Returns:
            {running, phase, session_id, tier, arm_pose, total_poses, captured, skipped, status, error, big_joints,
            waiting_since}: arm_pose is the 0-based index over the whole pose set of the pose worked on (None before
            the first), skipped maps each skip reason to its count, big_joints names the joints tier 2 moves,
            waiting_since is the clock time a confirm wait began (None outside a wait).
        """
        with self.lock:
            return {
                "running": self.running,
                "phase": self.phase,
                "session_id": self.session_id,
                "tier": self.tier,
                "arm_pose": self.arm_pose,
                "total_poses": self.total_poses,
                "captured": self.captured,
                "skipped": dict(self.skipped),
                "status": self.status,
                "error": self.error,
                "big_joints": list(BIG_JOINTS),
                "waiting_since": self.waiting_since,
            }

    def wait(self, timeout: float | None = None) -> None:
        """Block until the worker thread has finished (no-op when none runs).

        Args:
            timeout: Longest wait in seconds, None waits without limit.
        """
        thread = self.thread
        if thread is not None:
            thread.join(timeout)

    def stop(self) -> dict[str, Any]:
        """Ask the running sequence to stop; it holds the arm (holding with the board on, else idle).

        A park in progress is not interrupted.

        Returns:
            The state() after the request.
        """
        self.stop_event.set()
        return self.state()

    def claim(self, phase: str, status: str) -> str:
        """Mark a sequence or park as running, then check `blocked` (outside the lock: it reads other controllers).

        Args:
            phase: Phase to enter.
            status: Status line to show.

        Returns:
            str: The phase before the claim.

        Raises:
            Busy: Something runs already, the arm holds the board pose (a new base move), or `blocked` names a reason.
        """
        with self.lock:
            if self.running:
                raise Busy(f"a head-arm sequence is running (phase {self.phase}): stop it first")
            if phase == MOVING_TO_BASE and self.phase == HOLDING:
                raise Busy(f"the arm holds the board pose: {MESSAGE_PARK} before a new sequence")
            previous = self.phase
            self.running = True
        reason = self.blocked()
        with self.lock:
            if reason:
                self.running = False
                raise Busy(reason)
            self.phase = phase
            self.status = status
            self.error = None
            self.waiting_since = None
        return previous

    def give_up(self, phase: str, error: str) -> None:
        """Undo a claim that failed before the worker started."""
        with self.lock:
            self.running = False
            self.phase = phase
            self.status = f"refused: {error}"
            self.error = error

    def move_to_base(self, confirmed: bool) -> dict[str, Any]:
        """Start a sequence: take the arm and move it, without the board, to the base pose.

        Args:
            confirmed: The user confirmed the base move (it moves shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
                wrist_roll and the gripper).

        Returns:
            The state() after the start.

        Raises:
            ValueError: Not confirmed (nothing is called).
            Busy: A sequence or park runs, the arm holds the board pose, or `blocked` names a reason.
            SequenceError: The arm or head URDF is unusable (nothing is called).
        """
        if not confirmed:
            raise ValueError(
                "the base move needs confirmed=True: it moves " + ", ".join(BASE_JOINTS) + " (no board attached)"
            )
        self.claim(MOVING_TO_BASE, "preparing the pose check")
        try:
            plan = self.prepare()
        except SequenceError as exc:
            self.give_up(IDLE, str(exc))
            raise
        with self.lock:
            self.session_id = None
            self.tier = self.arm_pose = None
            self.total_poses = len(plan.poses)
            self.captured = 0
            self.skipped = Counter()
            self.board_on = False
            self.big_joints_allowed = None
            self.status = "taking the arm"
        self.stop_event.clear()
        self.thread = threading.Thread(target=self.work, args=(plan,), name="head-arm-sequence", daemon=True)
        self.thread.start()
        LOG.info("head-arm sequence started: %d poses", len(plan.poses))
        return self.state()

    def prepare(self) -> Plan:
        """Build the pose check and the head chain from the arm URDF and the applied head URDF.

        Raises:
            SequenceError: A URDF cannot be read or parsed.
        """
        try:
            head_text = self.head_urdf()
            checker = PoseChecker(
                self.arm_urdf(),
                head_text,
                self.cfg.fit,
                self.seq.pose_check,
                self.seq.board_margin_m(nominal_mount(self.cfg)),
            )
            head = UrdfChain.from_text(head_text)
        except (ValueError, KeyError, OSError, SyntaxError) as exc:
            raise SequenceError(f"cannot read the arm or head URDF: {type(exc).__name__}: {exc}") from exc
        mount = self.cfg.fit.board_mount
        centre = pose_from_xyz_rpy(mount.xyz_init, mount.rpy_init) @ np.append(board_centre(self.cfg.fit.board), 1.0)
        return Plan(checker=checker, head=head, poses=scripted_poses(self.cfg), board_centre=centre)

    def board_attached(self) -> dict[str, Any]:
        """The user clipped the board to the fixed jaw: start the still session and tiers 0 and 1.

        Returns:
            The state() after the request.

        Raises:
            Busy: The sequence is not in phase waiting_board.
        """
        with self.lock:
            if not self.running or self.phase != WAITING_BOARD:
                raise Busy(f"Board attached is accepted in phase {WAITING_BOARD} only (phase {self.phase})")
            self.board_on = True
        return self.state()

    def allow_big_joints(self, confirmed: bool) -> dict[str, Any]:
        """The user's answer to the big-joint confirm: True runs tier 2, False skips it.

        Args:
            confirmed: Whether elbow_flex, shoulder_lift and shoulder_pan may move.

        Returns:
            The state() after the request.

        Raises:
            Busy: The sequence is not in phase waiting_big_joints.
        """
        with self.lock:
            if not self.running or self.phase != WAITING_BIG_JOINTS:
                raise Busy(f"the big-joint answer is accepted in phase {WAITING_BIG_JOINTS} only (phase {self.phase})")
            self.big_joints_allowed = bool(confirmed)
        return self.state()

    def park(self, confirmed: bool) -> dict[str, Any]:
        """Park the arm (mcp_server homes the arm, then the head), with the board on or off the gripper.

        Nothing carried: the plain park. Only a holding load while this sequencer's board is clipped on (the jaw
        clamps the board stick): the path to `park_pose` is checked with the board, then the arm moves there and
        parks with `park_calibration_board`. Any other carry reason refuses, as does a failed path check.

        Args:
            confirmed: The user confirmed the park move.

        Returns:
            The state() after the start of the park.

        Raises:
            ValueError: Not confirmed (nothing is called).
            Busy: A sequence or park runs, or `blocked` names a reason.
            SequenceError: get_control_state fails, reports a carried object other than the board (its reasons are
                quoted) or the board path check refuses (its rules are quoted); nothing moved.
        """
        if not confirmed:
            raise ValueError("Park needs confirmed=True: the arm moves to its park pose")
        previous = self.claim(PARKING, "checking what the gripper carries")
        board = False
        try:
            reasons = carrying_reasons(self.robot.control_state())
            if reasons:
                with self.lock:
                    board = self.board_on and all(r.startswith(HOLDING_LOAD_PREFIX) for r in reasons)
                if not board:
                    raise SequenceError(f"Park refused: mcp_server reports carrying ({'; '.join(reasons)})")
                with self.lock:
                    self.status = "checking the board's park path"
                self.check_board_park_path()
        except SequenceError as exc:
            self.give_up(previous, str(exc))
            raise
        except Exception as exc:
            error = f"Park refused: {type(exc).__name__}: {exc}"
            self.give_up(previous, error)
            raise SequenceError(error) from exc
        with self.lock:
            self.status = "parking: homing the arm, then the head"
        self.stop_event.clear()
        self.thread = threading.Thread(target=self.park_work, args=(previous, board), name="head-arm-park", daemon=True)
        self.thread.start()
        return self.state()

    def check_board_park_path(self) -> None:
        """Check the path from the measured joints to `park_pose` with the board on the gripper.

        The board keeps `park_board_sag_margin_m` more than the sequence margin from the body and the head, and that
        much more than `min_board_height_m` above the floor (the board sags below FK under its weight).

        Raises:
            SequenceError: A URDF is unusable, or the path check names a violated rule (nothing moved).
            StepFailed: get_arm_state fails or reports no measured joints.
        """
        seq = self.seq
        sag = seq.park_board_sag_margin_m
        fit = self.cfg.fit
        fitted = self.fitted_mount(self.session_id)
        if fitted is not None:
            mount = fit.board_mount.model_copy(
                update={"xyz_init": tuple(fitted["xyz"]), "rpy_init": tuple(fitted["rpy"])}
            )
            fit = fit.model_copy(update={"board_mount": mount})
        margin = seq.board_margin_m(fitted is None and nominal_mount(self.cfg)) + sag
        pose_check = seq.pose_check.model_copy(update={"min_board_height_m": seq.pose_check.min_board_height_m + sag})
        try:
            checker = PoseChecker(self.arm_urdf(), self.head_urdf(), fit, pose_check, margin)
        except (ValueError, KeyError, OSError, SyntaxError) as exc:
            raise SequenceError(f"cannot read the arm or head URDF: {type(exc).__name__}: {exc}") from exc
        start = self.measured()
        gripper = max(start.get(GRIPPER_JOINT, seq.gripper_rad), seq.gripper_rad)
        entries = checker.check_path(start, dict(seq.park_pose), gripper)
        if entries:
            quoted = "; ".join(entries[:MAX_QUOTED_ENTRIES])
            more = f" (+{len(entries) - MAX_QUOTED_ENTRIES} more)" if len(entries) > MAX_QUOTED_ENTRIES else ""
            raise SequenceError(
                f"Park refused by the path check with the board on, nothing was moved and the arm keeps holding: "
                f"{quoted}{more}"
            )

    # worker thread

    def park_work(self, previous: str, board: bool = False) -> None:
        """Worker: take the arm and release it with a park; a failure leaves the phase as it was.

        Args:
            previous: The phase before the park.
            board: The board is on the gripper: move to `park_pose` first (its path was checked), then release with
                `park_calibration_board`.
        """
        error: str | None = None
        status = ""
        lease = False
        try:
            self.call(self.robot.take_arm(), "take_control")
            lease = True
            if board:
                self.park_move()
            result = self.call(
                self.robot.release_arm(keep_position=False, park_calibration_board=board), "release_control (park)"
            )
            lease = False
            reset = (result.data or {}).get("reset") or {}
            if reset.get("arm") != HOMED:
                raise StepFailed(f"the park did not home the arm: reset.arm {reset.get('arm')!r}")
            status = f"parked: arm {reset.get('arm')}, head {reset.get('head', 'n/a')}"
        except StepFailed as exc:
            error = str(exc)
        except Exception as exc:  # reported as the park error; the phase stays as it was
            error = f"{type(exc).__name__}: {exc}"
            LOG.error("head-arm park failed: %s", error)
        if lease:
            held = self.robot.release_arm(keep_position=True)
            if held.is_error:
                error = f"{error}; release_control (hold) failed: {held.text}"
        with self.lock:
            self.running = False
            if error is None:
                self.phase = IDLE
                self.status = status
                self.error = None
            else:
                self.phase = previous
                self.status = f"park failed: {error}"
                self.error = error
        LOG.info("head-arm park %s", "done" if error is None else f"failed: {error}")

    def park_move(self) -> None:
        """Move to `park_pose` at the sequence speed scale (guarded by mcp_server); the path was checked.

        Raises:
            StepFailed: The move fails or does not converge.
        """
        result = self.call(
            self.robot.move_arm_joints(dict(self.seq.park_pose), self.seq.speed_scale),
            "move_arm_joints to the park pose",
        )
        status, message = tool_status(result)
        if status != CONVERGED:
            raise StepFailed(f"move_arm_joints to the park pose did not converge: {status}: {message}")

    def work(self, plan: Plan) -> None:
        """Worker: run the sequence, then always stop the capture and release the arm holding its position."""
        error: str | None = None
        outcome = "done"
        try:
            self.run_sequence(plan)
        except Stopped:
            outcome = REASON_STOPPED
        except StepFailed as exc:
            outcome, error = "error", str(exc)
        except Exception as exc:  # reported as the sequence error; the finally block still holds the arm
            outcome, error = "error", f"{type(exc).__name__}: {exc}"
            LOG.error("head-arm sequence failed: %s", error)
        finally:
            error = self.hold(error)
            with self.lock:
                self.running = False
                self.phase = HOLDING if self.board_on else IDLE
                self.waiting_since = None
                self.error = error
                detail = f": {error}" if error else ""
                follow = f"; {MESSAGE_PARK}" if self.phase == HOLDING else ""
                note = "; tier 2 skipped" if self.big_joints_allowed is False else ""
                self.status = f"{outcome}{detail}{note}{follow}"
        LOG.info("head-arm sequence %s", outcome)

    def hold(self, error: str | None) -> str | None:
        """Stop the still capture and release the arm keeping its position; failures are added to the error."""
        problems = [error] if error else []
        try:
            self.capture.stop()
        except Exception as exc:  # the arm is still released below
            problems.append(f"stopping the still capture failed: {type(exc).__name__}: {exc}")
        result = self.robot.release_arm(keep_position=True)
        if result.is_error:
            problems.append(f"release_control (hold) failed: {result.text}")
        return "; ".join(problems) or None

    def run_sequence(self, plan: Plan) -> None:
        """Base pose, the board confirm, tiers 0 and 1, the big-joint confirm, tier 2, back to the base pose."""
        self.call(self.robot.take_arm(), "take_control")
        self.check_alive()
        with self.lock:
            self.status = "moving to the base pose (no board)"
        base = dict(self.seq.base_pose)
        self.checked_move(plan, base, "the base pose", {**base, GRIPPER_JOINT: self.seq.gripper_rad})
        self.wait_confirm(WAITING_BOARD, MESSAGE_BOARD, lambda: self.board_on)
        session_id = self.capture.start_still_session(None)
        with self.lock:
            self.session_id = session_id
            self.phase = CAPTURING
        for tier in range(BIG_JOINT_TIER):
            self.capture_tier(plan, tier)
        big = ", ".join(BIG_JOINTS)
        self.wait_confirm(
            WAITING_BIG_JOINTS,
            f"Tier 2 moves {big}: allow these moves, or skip tier 2",
            lambda: self.big_joints_allowed is not None,
        )
        with self.lock:
            self.phase = CAPTURING
        if self.big_joints_allowed:
            self.capture_tier(plan, BIG_JOINT_TIER)
        self.check_alive()
        with self.lock:
            self.phase = RETURNING
            self.tier = None
            self.status = "moving back to the base pose"
        self.checked_move(plan, base, "the base pose (return)", base)

    def check_alive(self) -> None:
        """Raise Stopped after stop()."""
        if self.stop_event.is_set():
            raise Stopped

    def call(self, result: ToolResult, what: str) -> ToolResult:
        """The result of a tool call; a tool error stops the sequence quoting it."""
        if result.is_error:
            raise StepFailed(f"{what} failed: {result.text}")
        return result

    def measured(self) -> dict[str, float]:
        """Measured arm joints from get_arm_state (the gripper too when reported).

        Raises:
            StepFailed: The call failed, no positions were reported, or an arm joint is missing.
        """
        result = self.call(self.robot.arm_state(), "get_arm_state")
        positions = (result.data or {}).get("positions")
        if not isinstance(positions, dict):
            raise StepFailed("get_arm_state reports no measured joints (stale joint states); nothing was moved")
        missing = [name for name in ARM_JOINT_NAMES if not isinstance(positions.get(name), int | float)]
        if missing:
            raise StepFailed(f"get_arm_state reports no measured {', '.join(missing)}; nothing was moved")
        return {name: float(positions[name]) for name in BASE_JOINTS if isinstance(positions.get(name), int | float)}

    def checked_move(self, plan: Plan, pose: dict[str, float], label: str, targets: dict[str, float]) -> None:
        """Check the path from the measured joints to `pose`, then command `targets`; refusals stop before it.

        Args:
            plan: The sequence plan.
            pose: Arm joint target (measured joint space) the path is checked to.
            label: The pose named in the messages.
            targets: What move_arm_joints is sent (`pose`, plus the gripper for the base move only).

        Raises:
            StepFailed: get_arm_state fails, the path check refuses, the move fails or does not converge.
        """
        start = self.measured()
        gripper = max(start.get(GRIPPER_JOINT, self.seq.gripper_rad), self.seq.gripper_rad)
        entries = plan.checker.check_path(start, pose, gripper)
        if entries:
            quoted = "; ".join(entries[:MAX_QUOTED_ENTRIES])
            more = f" (+{len(entries) - MAX_QUOTED_ENTRIES} more)" if len(entries) > MAX_QUOTED_ENTRIES else ""
            raise StepFailed(f"{label} refused by the path check, nothing was moved: {quoted}{more}")
        self.check_alive()
        result = self.call(self.robot.move_arm_joints(targets, self.seq.speed_scale), f"move_arm_joints to {label}")
        status, message = tool_status(result)
        if status != CONVERGED:
            raise StepFailed(f"move_arm_joints to {label} did not converge: {status}: {message}")

    def wait_confirm(self, phase: str, message: str, ready: Callable[[], bool]) -> None:
        """Enter a confirm phase and wait (up to confirm_timeout_s) until `ready` holds.

        Raises:
            Stopped: stop() was called.
            StepFailed: confirm_timeout_s passed.
        """
        began = self.clock()
        with self.lock:
            self.phase = phase
            self.status = message
            self.waiting_since = began
        while True:
            self.check_alive()
            with self.lock:
                if ready():
                    self.waiting_since = None
                    return
            if self.clock() - began >= self.seq.confirm_timeout_s:
                raise StepFailed(
                    f"no answer in phase {phase} within confirm_timeout_s {self.seq.confirm_timeout_s:g} s"
                )
            self.sleep(POLL_S)

    def capture_tier(self, plan: Plan, tier: int) -> None:
        """Visit every pose of a tier: checked move, aim the head on the applied chain, one view per head offset."""
        indices = [index for index, (pose_tier, _) in enumerate(plan.poses) if pose_tier == tier]
        for number, index in enumerate(indices):
            self.check_alive()
            pose = plan.poses[index][1]
            label = f"tier {tier} pose {number} (arm pose {index})"
            with self.lock:
                self.tier = tier
                self.arm_pose = index
                self.status = (
                    f"tier {tier}: pose {number + 1} of {len(indices)} (arm pose {index + 1} of {len(plan.poses)})"
                )
            self.checked_move(plan, pose, label, pose)
            joints = self.measured()
            centre = (plan.checker.link_poses(joints)[BOARD_LINK] @ plan.board_centre)[:3]
            pan, tilt = aim_head(plan.head, centre)
            for offset, (dpan, dtilt) in enumerate(self.seq.head_offsets_deg):
                self.check_alive()
                self.view(plan, index, tier, offset, math.degrees(pan) + dpan, math.degrees(tilt) + dtilt)

    def view(self, plan: Plan, index: int, tier: int, offset: int, pan_deg: float, tilt_deg: float) -> None:
        """One head offset at an arm pose: skipped outside the head limits, else aim, settle and capture a still."""
        where = f"tier {tier} arm pose {index} head offset {offset}"
        reason = outside_head_limits(plan.head, pan_deg, tilt_deg)
        if reason is not None:
            self.count_skipped(reason, f"{where}: pan {pan_deg:.2f} deg, tilt {tilt_deg:.2f} deg")
            return
        result = self.call(self.robot.set_head_angles(pan_deg, tilt_deg), f"set_head_angles at {where}")
        status, message = tool_status(result)
        if status != CONVERGED:
            self.count_skipped(f"head move {status}", f"{where}: {message}")
            return
        self.sleep(self.seq.settle_s)
        self.check_alive()
        outcome: StillOutcome = self.capture.capture_still(index, tier, offset)
        if outcome.stored:
            with self.lock:
                self.captured += 1
            return
        if outcome.reason == REASON_STOPPED:
            raise StepFailed(f"the still capture session was stopped at {where}")
        self.count_skipped(outcome.reason, where)

    def count_skipped(self, reason: str, detail: str) -> None:
        """Count a skipped view by its reason."""
        with self.lock:
            self.skipped[reason] += 1
        LOG.warning("head-arm view skipped (%s): %s", reason, detail)
