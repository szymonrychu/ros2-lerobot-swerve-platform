"""Guided head-from-arm capture sequence (design D37, D38, D46): the board on the gripper, moved by the trusted arm
through checked poses, the head aimed at it by the node.

Order: `move_to_base` (confirmed; no board) -> the user seats the board handle, `board_attached` -> tier 0 (wrist_roll)
-> tier 1 (wrist_flex, wrist_roll) -> `allow_big_joints` (elbow_flex, shoulder_lift, shoulder_pan; declined skips tier
2) -> tier 2 -> back to the base pose -> `holding` -> `home` (confirmed) -> `idle`. Home works with the board on (design
D63, easy retries): the whole path from the measured joints to the stored home (`arm_home_file`, read at the press) is
checked with the board outline (the latest fitted board_mount when a result exists, else the configured one) and
`park_board_sag_margin_m` added to the board floor, body and head clearances; a pass moves there at `speed_scale` and
finishes through mcp_server's internal `POST /admin/park_calibration_board` (own token, the board mount as the only request content): mcp_server
ignores the holding load of the gripped board handle and nothing else, and re-checks the board on every home leg from
the measured joints. The agent-facing `release_control` has no board argument.

Every motion goes through RobotClient (mcp_server's guarded tools) and every arm command waits for the FK path check
from the measured joints to the target (`PoseChecker.check_path`); a refusal stops before anything moves. Every view (an
arm pose with the head aimed at the board plus one head offset) keeps every ChArUco corner at 0.20 m or more of depth
from `camera_frame` (the D435 min-Z plus a margin) and inside the colour image of the still session's camera_info
(`PoseChecker.check_view`), else it is skipped with the reason "inside D435 min-Z" or "outside colour FOV", as a view
outside the head limits is. A pose whose every view would be skipped, judged at its target joints, is not visited; the
others are judged again at the measured joints. The sequence waits up to `still.timeout_s` for the first colour
camera_info after the still session starts. The gripper opens to `insert_gripper_rad` with the base pose; after the
user's Board attached confirm it closes on the handle to `grip_rad` (handle contact minus the squeeze, below the mcp
converge tolerance so the target stays commanded), and the servo `Torque_Limit` (the default grip profile) is the effort
limit, as `move_arm_joints` has no per-call one. A jaw that ends below `handle_contact_rad - handle_squeeze_rad / 2`
closed past the handle (unseated), and the sequence stops holding the arm. Every exit of the sequence (end, stop, tool
error, exception, confirm timeout) stops the still capture and releases the arm with `keep_position=True`: the arm holds
a checked pose and mcp_server keeps the head where it is. The sequence never homes on its own and never sends
`carried_object_radius_m`.

Head model of the checks (the head is real, not a full-range envelope, once its pan/tilt is known): `held_head` is the
measured pan/tilt (the commanded one when the result carries none) of the last converged `set_head_angles` of the run;
it is None at every run start (a resume too) and after a head move that did not converge or failed. An arm move checks
its path with the real head shapes over `PoseChecker.head_band(held_head)` when the head is held, else with the
full-range envelope (the base move, the resume base move, the handle grip and the move to tier 0 pose 0). A view is
also skipped with the reason `head_clearance` when the head cannot move about its aim at the arm pose
(`check_head_move` over `head_band(aim)`), and every head move is checked again just before `set_head_angles` from the
measured joints over `head_span(held_head, aim)` (the envelope when not held): a refusal skips the view with that
reason and nothing is commanded. Home with the board on keeps the full-range envelope.

Resume with the board on (`resume_with_board`, confirmed): from `holding` with the board on (a stopped or ended
sequence; the board may be taped into the jaws), a new run (new run id) moves to the base pose through the same path
check from the measured joints with the board on, commanding the gripper to its measured angle (never
`insert_gripper_rad`: the jaws never open before the Home), skips the board confirm and the handle grip, and runs the
still session and the tiers exactly as after Board attached (`capture_tiers`), ending holding with the board on, then
Home. A refused start (not holding, no board on, running, `blocked`, an unusable URDF) moves nothing and keeps holding.

Resume from idle (`resume_with_board(confirmed, board_gripped=True)`): the phase and board_on live in memory only, so a
node restart (a deploy) leaves `idle` with the board still taped or seated in the jaws. From `idle` (not running) the
resume also needs the user's distinct statement that the board is gripped in the jaws (`board_gripped`; without it a
ValueError, nothing is called). Before anything moves the measured gripper angle (get_arm_state) must be at most
`handle_contact_rad + grip_converge_tolerance_rad` (jaws closed on the handle: the contact angle plus the mcp_server
converge tolerance; a larger angle is clearly open jaws), else the start is refused (SequenceError, nothing moved, the
phase stays idle). A pass sets board_on, writes the board record and logs `board_confirmed` (the user's statement, from
idle); the run is then the resume above (same mount fallback: the fit of the restored session, else the restored
board record mount, else the configured one; the same path check from the measured joints with the jaw kept), ending
holding with the board on. A refusal in the worker (path check, no gripper angle) ends holding with the board on, as
the user stated it is on.

Grip record (`confirm_grip`, confirmed; nothing moves): from `idle` or `holding` with nothing running, the user states the
board handle is seated in the closed jaws; the measured gripper angle (get_arm_state, recorded as measured) becomes the
active `handle_contact_rad`, written atomically to `grip_file` (GRIP_RECORD_FILE in state_dir) and loaded at the next
start (an invalid record is ignored with a WARNING). `self.seq` is the active arm sequence config (`config_seq` with the
recorded contact), so `grip_rad`, the unseated and slack limits and the closed-jaw limit of a resume from idle all
follow it. While it measures it holds a short claim (`running`, a `grip-...` run id, phase and status kept), so a
sequence or Home start is refused. `reset_grip` (confirmed) removes the file and makes `config_seq` active again.

Threads (as floor_sweep.py): `move_to_base`, `resume_with_board`, `board_attached`, `allow_big_joints`, `stop`, `home`,
`clear_board_record`, `confirm_grip`, `reset_grip` and `state` run on HTTP threads; one worker thread runs the sequence or the Home. `lock` guards the shared fields and is only held for
short updates (never across a tool call or `blocked`); the worker waits through the injected `sleep` and measures
every wait on the injected `clock`.

Events: every sequence start (a resume too), step outcome (base pose, board attached, handle grip, still session, each pose, the
big-joint answer, the return), sequence end (done, stopped, error), Home start, Home result and refusal is logged with
its message (errors at ERROR, stops and skipped poses at WARNING) and kept in a bounded list of the newest
EVENT_LIMIT events, `state()["events"]` (newest first), so the Calibrate page shows why a sequence ended.

Telemetry (telemetry.py): the journal lines of this module are `calib_event`s of flow `head_arm` under the run id of the
sequence (`seq-...`), Home (`home-...`), board record clear (`board-...`) or grip record (`grip-...`), plus the traceback of an unexpected exception; a UI event is that same line
(append_event), never a second one. The
worker steps are timed (start, ok, stopped or error with the exception and the step's inputs), and the measured joints
(get_arm_state), path checks (start, target, gripper, every violated entry), MCP results (status, message, result),
the gripper reading against its limits, each view and still outcome and the Home's carry reasons, mount and reset are
logged as they happen. Tokens are never logged.
"""

import json
import logging
import math
import os
import threading
import time
from collections import Counter, deque
from collections.abc import Callable
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np
from pydantic import AwareDatetime, BaseModel, ConfigDict, Field, ValidationError
from ros2_common.camera_geometry import CameraIntrinsics
from ros2_common.head_rgbd_calibration.arm_pose_check import (
    BOARD_LINK,
    GRIPPER_JOINT,
    HEAD_CLEARANCE_RULE,
    HeadRegion,
    PoseChecker,
)
from ros2_common.head_rgbd_calibration.chain import UrdfChain, pose_from_xyz_rpy
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES
from ros2_common.head_rgbd_calibration.floor_fit import PAN_JOINT, TILT_JOINT
from ros2_common.head_rgbd_calibration.head_arm_fit import aim_head, board_centre

from head_rgbd_calibration.arm_home import ArmHomeError, load_arm_home
from head_rgbd_calibration.capture import CaptureController, StillOutcome
from head_rgbd_calibration.config import BIG_JOINTS, CAD_BOARD_RPY, CAD_BOARD_XYZ, ArmSequenceConfig, NodeConfig
from head_rgbd_calibration.robot_client import RobotClient, ToolResult
from head_rgbd_calibration.telemetry import calib_event, new_run_id, step_timer

LOG = logging.getLogger(__name__)

IDLE = "idle"
MOVING_TO_BASE = "moving_to_base"
WAITING_BOARD = "waiting_board"
CAPTURING = "capturing"
WAITING_BIG_JOINTS = "waiting_big_joints"
RETURNING = "returning_to_base"
HOLDING = "holding"
HOMING = "homing"
POLL_S = 0.05
BOARD_RECORD_FILE = "head_arm_board.json"  # in state_dir, next to the sessions
CONVERGED = "converged"
HOMED = "homed"
MOUNT_TOLERANCE = 1e-6
LIMIT_TOLERANCE_RAD = 1e-9
MAX_QUOTED_ENTRIES = 3
TIER_COUNT = 3
BIG_JOINT_TIER = 2
REASON_STOPPED = "stopped"
REASON_HEAD_CLEARANCE = HEAD_CLEARANCE_RULE  # skip reason of a view whose head move would hit the arm or board
MESSAGE_BOARD = "Seat the arm board handle in the gripper jaws up to the shoulder, then press Board attached"
MESSAGE_UNSEATED = "the gripper closed past the handle contact: seat the handle up to the shoulder and start again"
MESSAGE_SLACK_GRIP = (
    "the gripper stopped short of the commanded grip by the mcp_server converge tolerance or more, so the clamp would"
    " relax: the handle is thicker than the configured contact;"
    " with the handle seated, record the measured contact with Confirm correct grip (POST /api/head-arm/grip/confirm)"
)
MESSAGE_STATE_BOARD = "; if the calibration board is on the gripper, state that the board is on and press Home again"
MESSAGE_HOME = "Press Home (the board may stay on when its home path is clear)"
MESSAGE_RESUME_PHASE = (
    "Resume with the board on is accepted only while holding with the board on, or from idle with the board confirmed"
    " gripped"
)
MESSAGE_RESUME_GRIPPED = (
    "resume with the board on from idle needs board_gripped=True: the user states the calibration board is gripped in"
    " the jaws (taped or seated); the jaws will not open"
)
MESSAGE_RESUME_OPEN_JAWS = (
    "resume from idle refused: the gripper is open (the board cannot be gripped in the jaws); nothing was moved"
)
BOARD_CONFIRMED_STEP = "board_confirmed"
MESSAGE_RESUME_CONFIRM = (
    "resume with the board on needs confirmed=True: it moves the arm with the board to the base pose (the gripper"
    " stays at its measured angle)"
)
HOLDING_LOAD_PREFIX = "holding load"  # mcp_server carrying reason of a gripper that clamps something (the board handle)
BASE_JOINTS = (*ARM_JOINT_NAMES, GRIPPER_JOINT)
EVENT_LIMIT = 50  # recent events kept in state()["events"]
EVENT_LEVELS = {logging.INFO: "info", logging.WARNING: "warning", logging.ERROR: "error"}
FLOW = "head_arm"
SEQUENCE_STEP = "sequence"
HOME_STEP = "home"
BOARD_RECORD_STEP = "board_record"
BOARD_CLEAR_STEP = "board_record_clear"
REASON_TOO_OLD = "too old"
REASON_UNKNOWN_SESSION = "not a stored head_arm session"
MESSAGE_CLEAR_CONFIRM = (
    "clearing the board record needs confirmed=True: the user states the calibration board is already off the gripper"
)
MESSAGE_CLEARED = "board record cleared: the user stated the calibration board is off the gripper (nothing moved)"
MESSAGE_NOTHING_TO_CLEAR = "no board record to clear (nothing moved)"
GRIP_RECORD_FILE = "head_arm_grip.json"  # in state_dir, next to the board record
GRIP_CONTACT_MIN_RAD = 0.03  # a recordable handle contact (rad, inclusive)
GRIP_CONTACT_MAX_RAD = 0.30
GRIP_STEP = "grip_record"
SOURCE_CONFIG = "config"
SOURCE_RECORDED = "recorded"
MESSAGE_GRIP_CONFIRM = (
    "recording the grip needs confirmed=True: the user states the board handle is seated in the closed jaws; the"
    " measured gripper angle becomes the handle contact (nothing moves)"
)
MESSAGE_GRIP_RESET_CONFIRM = (
    "resetting the grip record needs confirmed=True: the handle contact returns to the configured"
    " arm_sequence.handle_contact_rad (nothing moves)"
)
MESSAGE_GRIP_PHASE = (
    "the grip record can change only while idle or holding, with no sequence or Home running: stop it or let it end"
    " first"
)


class SequenceError(RuntimeError):
    """A step is refused or the sequence cannot start (the reason is the message; maps to 400)."""


class Busy(SequenceError):
    """A sequence or Home runs, `blocked` names a reason, or the step does not fit the phase (maps to 409)."""


@dataclass(frozen=True)
class View:
    """One head offset at an arm pose: the head command and why it is skipped (None: it is captured).

    Attributes:
        offset: Index of the head offset (`arm_sequence.head_offsets_deg`).
        pan_deg: Head pan command (deg).
        tilt_deg: Head tilt command (deg).
        reason: Skip reason (the head limit, MIN_Z_RULE or FOV_RULE), None for a view to capture.
        detail: The head command and every violated rule, for the log.
    """

    offset: int
    pan_deg: float
    tilt_deg: float
    reason: str | None
    detail: str


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
    """Whether `fit.board_mount` is still the shipped CAD seed (design D47; it gets the provisional margin).

    Args:
        cfg: Node config.

    Returns:
        bool: True when xyz_init and rpy_init equal CAD_BOARD_XYZ / CAD_BOARD_RPY.
    """
    mount = cfg.fit.board_mount
    pairs = zip((*mount.xyz_init, *mount.rpy_init), (*CAD_BOARD_XYZ, *CAD_BOARD_RPY), strict=True)
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
        raise SequenceError(f"Home refused: get_control_state failed: {result.text}")
    carrying = ((result.data or {}).get("arm") or {}).get("carrying")
    if not isinstance(carrying, dict) or not isinstance(carrying.get("carrying"), bool):
        raise SequenceError(f"Home refused: get_control_state reports no arm.carrying: {result.text}")
    if not carrying["carrying"]:
        return []
    return [str(reason) for reason in carrying.get("reasons") or []] or ["carrying"]


class GripRecord(BaseModel):
    """The confirmed grip: the measured gripper angle on the seated handle, the active handle contact.

    Attributes:
        handle_contact_rad (float): Handle contact (rad), GRIP_CONTACT_MIN_RAD to GRIP_CONTACT_MAX_RAD.
        recorded_at (AwareDatetime): When the user confirmed it (with a zone).
    """

    model_config = ConfigDict(extra="forbid")

    handle_contact_rad: float = Field(ge=GRIP_CONTACT_MIN_RAD, le=GRIP_CONTACT_MAX_RAD)
    recorded_at: AwareDatetime


def read_grip_record(path: Path) -> GripRecord:
    """Read and check a grip record file.

    Args:
        path (Path): The record `{handle_contact_rad, recorded_at}` (Sequencer grip_file).

    Returns:
        GripRecord: The checked record.

    Raises:
        OSError: The file cannot be read.
        ValueError: The content is not a valid record (pydantic ValidationError: not JSON, a missing or extra key, a
            contact out of range, a time without a zone).
    """
    return GripRecord.model_validate_json(path.read_text())


def read_board_record(path: Path) -> dict[str, Any]:
    """Read and check a board record file.

    Args:
        path (Path): The record `{session_id, mount: {xyz, rpy}, clipped_at}` (Sequencer board_file).

    Returns:
        dict[str, Any]: {session_id (str | None), clipped_at (str, ISO 8601 with a zone), clipped (datetime),
            mount ({xyz, rpy} as float lists)}.

    Raises:
        OSError: The file cannot be read.
        ValueError, AttributeError, KeyError, TypeError: The content is not a valid record.
    """
    record = json.loads(path.read_text())
    session = record.get("session_id")
    clipped = datetime.fromisoformat(record["clipped_at"])
    mount = {k: [float(v) for v in record["mount"][k]] for k in ("xyz", "rpy")}
    if session is not None and not isinstance(session, str):
        raise ValueError("session_id is not a string")
    if clipped.tzinfo is None:
        raise ValueError("clipped_at has no timezone")
    return {"session_id": session, "clipped_at": record["clipped_at"], "clipped": clipped, "mount": mount}


class Sequencer:
    """Runs one guided head-from-arm capture sequence (or one Home) at a time on a worker thread."""

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
        board_file: Path | None = None,
        session_exists: Callable[[str | None], bool] | None = None,
        now: Callable[[], datetime] | None = None,
        grip_file: Path | None = None,
    ) -> None:
        """Create an idle sequencer (nothing is called until move_to_base or home).

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
                None without one; the Home path check uses it instead of `fit.board_mount`.
            board_file: JSON record `{session_id, mount, clipped_at}` of a board attached to the gripper, written when
                the board is attached (or the user states it is on) and removed by a completed Home or a new base
                move, or by clear_board_record (the user states the board is off). state()["board_record"] shows it at
                any age; it is offered for a board Home (and a node start restores its session and mount) only when
                it is younger than `arm_sequence.board_record_max_age_s` and `session_exists` accepts its session; the
                user must still state that the board is on (home(board_on=True)) before a board Home. None: not kept.
            session_exists: Whether a capture session id is a stored head_arm session (default: any id).
            now: Wall clock (default: datetime.now(UTC)); stamps and ages the board record.
            grip_file (Path | None): JSON record `{handle_contact_rad, recorded_at}` of the confirmed grip
                (confirm_grip); a valid one at start replaces `arm_sequence.handle_contact_rad` everywhere until
                reset_grip removes it. None: a confirmed grip is kept in memory only (lost on a restart).
        """
        self.board_file = board_file
        self.session_exists = session_exists or (lambda session_id: session_id is not None)
        self.now = now or (lambda: datetime.now(UTC))
        self.fitted_mount = fitted_mount or (lambda _session_id: None)
        self.capture = capture
        self.robot = robot
        self.cfg = cfg
        self.config_seq: ArmSequenceConfig = cfg.arm_sequence
        self.seq: ArmSequenceConfig = cfg.arm_sequence
        self.grip_file = grip_file
        self.grip_recorded_at: str | None = None
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
        self.recorded_mount: dict[str, list[float]] | None = None
        self.events: deque[dict[str, str]] = deque(maxlen=EVENT_LIMIT)
        self.run_id: str | None = None
        self.current_step: str | None = None
        self.held_head: tuple[float, float] | None = None  # (pan, tilt) rad of the last converged head move of the run
        self.restore_board()
        self.restore_grip()

    def restore_board(self) -> None:
        """Offer the board record of a previous run: only a fresh one of a stored session, never as the state itself.

        A record older than `board_record_max_age_s`, of an unknown session or unreadable is not offered (logged); it
        is still shown by `board_record_state` until a Home or `clear_board_record` removes it. An offered one also
        restores its session and mount; a board Home still needs the user's fresh statement that the board is on.
        """
        if self.board_file is None or not self.board_file.is_file():
            return
        try:
            record = read_board_record(self.board_file)
        except (OSError, ValueError, AttributeError, KeyError, TypeError) as exc:
            self.log(BOARD_RECORD_STEP, "ignored", logging.WARNING, f"unreadable: {exc}", file=self.board_file)
            return
        age = (self.now() - record["clipped"]).total_seconds()
        problem = self.board_record_problem(record["session_id"], age)
        if problem == REASON_TOO_OLD:
            self.log(
                BOARD_RECORD_STEP,
                "ignored",
                logging.WARNING,
                problem,
                file=self.board_file,
                age_s=age,
                max_age_s=self.seq.board_record_max_age_s,
            )
            return
        if problem is not None:
            self.log(
                BOARD_RECORD_STEP,
                "ignored",
                logging.WARNING,
                problem,
                file=self.board_file,
                session_id=record["session_id"],
            )
            return
        session = record["session_id"]
        self.session_id = session
        self.recorded_mount = record["mount"]
        self.log(
            BOARD_RECORD_STEP, "offered", session_id=session, clipped_at=record["clipped_at"], mount=record["mount"]
        )

    def board_record_problem(self, session: str | None, age_s: float) -> str | None:
        """Why a board record is not offered for a board Home, or None when it is.

        Args:
            session (str | None): The record's session id.
            age_s (float): Its age by the node's wall clock (s).

        Returns:
            str | None: REASON_TOO_OLD (older than `board_record_max_age_s`, or from the future), REASON_UNKNOWN_SESSION
                (session_exists refuses it), else None.
        """
        if age_s > self.seq.board_record_max_age_s or age_s < 0:
            return REASON_TOO_OLD
        if not self.session_exists(session):
            return REASON_UNKNOWN_SESSION
        return None

    def board_record_state(self) -> dict[str, Any] | None:
        """The board record file as it is now, whatever its age (mcp_server refuses /arm/home while it exists).

        Returns:
            dict[str, Any] | None: None without a file, else {file, session_id, clipped_at, age_s, offered, problem}:
                file is its path, age_s its age in seconds (None, as session_id and clipped_at, when unreadable),
                offered whether a board Home would accept it as fresh, problem why not (REASON_TOO_OLD,
                REASON_UNKNOWN_SESSION or "unreadable: ..."; None when offered).
        """
        if self.board_file is None or not self.board_file.exists():
            return None
        view: dict[str, Any] = {"file": str(self.board_file), "session_id": None, "clipped_at": None, "age_s": None}
        try:
            record = read_board_record(self.board_file)
        except (OSError, ValueError, AttributeError, KeyError, TypeError) as exc:
            return {**view, "offered": False, "problem": f"unreadable: {exc}"}
        age = round((self.now() - record["clipped"]).total_seconds(), 1)
        problem = self.board_record_problem(record["session_id"], age)
        return {
            **view,
            "session_id": record["session_id"],
            "clipped_at": record["clipped_at"],
            "age_s": age,
            "offered": problem is None,
            "problem": problem,
        }

    def clear_board_record(self, confirmed: bool) -> dict[str, Any]:
        """The user states the calibration board is already off the gripper: remove the board record, move nothing.

        Refused while a sequence or a Home runs (its worker may still write the record). A holding phase (the
        sequence ended with the board on) returns to idle: the board is off, so the flow's Home and the arm Home
        (`/arm/home`) need no board path.

        Args:
            confirmed (bool): The user confirmed the board is off the gripper.

        Returns:
            dict[str, Any]: The state() after the clear.

        Raises:
            ValueError: Not confirmed (nothing changes).
            Busy: A sequence or Home runs.
            SequenceError: The file cannot be removed (logged; the record stays).
        """
        if not confirmed:
            raise ValueError(MESSAGE_CLEAR_CONFIRM)
        with self.lock:
            if self.running or (self.thread is not None and self.thread.is_alive()):
                raise Busy(f"a head-arm sequence or Home is running (phase {self.phase}): stop it or let it end first")
            previous = self.board_record_state()
            self.run_id = new_run_id("board", self.now())
            try:
                if self.board_file is not None:
                    self.board_file.unlink(missing_ok=True)
            except OSError as exc:
                self.append_event(
                    logging.ERROR,
                    f"board record not cleared: {exc}",
                    BOARD_CLEAR_STEP,
                    "error",
                    reason=str(exc),
                    file=str(self.board_file),
                )
                raise SequenceError(f"cannot remove the board record {self.board_file}: {exc}") from exc
            phase = self.phase
            self.board_on = False
            self.recorded_mount = None
            if self.phase == HOLDING:
                self.phase = IDLE
            message = MESSAGE_CLEARED if previous is not None else MESSAGE_NOTHING_TO_CLEAR
            self.status = message
            self.error = None
            self.append_event(
                logging.INFO,
                message,
                BOARD_CLEAR_STEP,
                "cleared" if previous is not None else "none",
                phase,
                file=None if self.board_file is None else str(self.board_file),
                record=previous,
                phase_after=self.phase,
            )
        return self.state()

    def grip_problem(self) -> str | None:
        """Why the grip record cannot change now, or None when it can (the caller holds `lock`).

        Returns:
            str | None: MESSAGE_GRIP_PHASE with the phase while a sequence, Home or grip measure runs, the worker thread
                is alive, or the phase is neither idle nor holding; else None.
        """
        alive = self.thread is not None and self.thread.is_alive()
        if self.running or alive or self.phase not in (IDLE, HOLDING):
            return f"{MESSAGE_GRIP_PHASE} (phase {self.phase}, running {self.running or alive})"
        return None

    def with_contact(self, contact: float) -> ArmSequenceConfig:
        """The configured arm sequence with another handle contact, checked by its own validator.

        Args:
            contact (float): Handle contact (rad).

        Returns:
            ArmSequenceConfig: `config_seq` with `handle_contact_rad` = contact (grip_rad follows).

        Raises:
            ValidationError: The config refuses the contact (e.g. at or below `handle_squeeze_rad`).
        """
        return ArmSequenceConfig.model_validate({**self.config_seq.model_dump(), "handle_contact_rad": contact})

    def restore_grip(self) -> None:
        """Load the grip record of a previous confirm: a valid one becomes the active handle contact.

        An unreadable, invalid or out-of-range record, or one the arm sequence config refuses, is ignored (logged at
        WARNING); the configured contact stays active and the file stays until reset_grip or a new confirm.
        """
        if self.grip_file is None or not self.grip_file.is_file():
            return
        try:
            record = read_grip_record(self.grip_file)
            seq = self.with_contact(record.handle_contact_rad)
        except (OSError, ValueError) as exc:
            self.log(GRIP_STEP, "ignored", logging.WARNING, f"unusable: {exc}", file=self.grip_file)
            return
        self.seq = seq
        self.grip_recorded_at = record.recorded_at.isoformat(timespec="seconds")
        self.log(
            GRIP_STEP,
            "loaded",
            file=self.grip_file,
            handle_contact_rad=record.handle_contact_rad,
            grip_rad=seq.grip_rad,
            recorded_at=self.grip_recorded_at,
            config_handle_contact_rad=self.config_seq.handle_contact_rad,
        )

    def grip_state(self) -> dict[str, Any]:
        """The active handle contact and where it comes from.

        Returns:
            dict[str, Any]: {handle_contact_rad (float), grip_rad (float), source ("config" or "recorded"),
                recorded_at (str | None, ISO 8601 of the confirm), config_handle_contact_rad (float)}.
        """
        recorded = self.grip_recorded_at is not None
        return {
            "handle_contact_rad": self.seq.handle_contact_rad,
            "grip_rad": self.seq.grip_rad,
            "source": SOURCE_RECORDED if recorded else SOURCE_CONFIG,
            "recorded_at": self.grip_recorded_at,
            "config_handle_contact_rad": self.config_seq.handle_contact_rad,
        }

    def confirm_grip(self, confirmed: bool) -> dict[str, Any]:
        """The user states the board handle is seated in the closed jaws: record the measured gripper angle as the
        active handle contact (grip_rad, the unseated and slack limits and the closed-jaw limit of a resume follow).

        Nothing moves: only get_arm_state is called. While it measures the sequencer holds a short claim (`running`,
        a new `grip-...` run id; phase and status unchanged), so a sequence or Home start is refused meanwhile. The
        value is recorded as measured; the file is written atomically before the new contact becomes active.

        Args:
            confirmed (bool): The user confirmed the handle is seated in the closed jaws.

        Returns:
            dict[str, Any]: The state() after the record.

        Raises:
            ValueError: Not confirmed (nothing is called).
            Busy: A sequence, Home or grip measure runs, the worker thread is alive, or the phase is neither idle nor
                holding (nothing is called).
            SequenceError: get_arm_state fails or reports no gripper angle, the angle is outside GRIP_CONTACT_MIN_RAD
                to GRIP_CONTACT_MAX_RAD or refused by the arm sequence config, or the file cannot be written (the
                active contact is unchanged).
        """
        if not confirmed:
            raise ValueError(MESSAGE_GRIP_CONFIRM)
        with self.lock:
            problem = self.grip_problem()
            if problem is not None:
                raise Busy(problem)
            self.running = True
            self.run_id = new_run_id("grip", self.now())
            previous = self.seq.handle_contact_rad
        try:
            try:
                jaw, seq = self.measured_contact()
            except SequenceError as exc:
                self.event(
                    logging.WARNING,
                    f"grip not recorded: {exc}; nothing moved",
                    GRIP_STEP,
                    "refused",
                    reason=str(exc),
                    previous_rad=previous,
                )
                raise
            stamp = self.now().isoformat(timespec="seconds")
            file = None if self.grip_file is None else str(self.grip_file)
            try:
                self.write_grip(GripRecord(handle_contact_rad=jaw, recorded_at=datetime.fromisoformat(stamp)))
            except OSError as exc:
                self.event(
                    logging.ERROR,
                    f"grip not recorded: cannot write {file}: {exc}; nothing moved",
                    GRIP_STEP,
                    "error",
                    reason=str(exc),
                    measured_rad=jaw,
                    previous_rad=previous,
                    file=file,
                )
                raise SequenceError(f"cannot write the grip record {file}: {exc}") from exc
            with self.lock:
                self.seq = seq
                self.grip_recorded_at = stamp
                self.append_event(
                    logging.INFO,
                    f"grip recorded: handle contact {jaw:.4f} rad (was {previous:.4f} rad); nothing moved",
                    GRIP_STEP,
                    "recorded",
                    measured_rad=jaw,
                    previous_rad=previous,
                    grip_rad=seq.grip_rad,
                    file=file,
                    recorded_at=stamp,
                )
        finally:
            with self.lock:
                self.running = False
        return self.state()

    def measured_contact(self) -> tuple[float, ArmSequenceConfig]:
        """Measure the gripper (get_arm_state) and check it as a handle contact (nothing moves).

        Returns:
            tuple[float, ArmSequenceConfig]: The measured gripper angle (rad, as reported) and the arm sequence config
                with it as `handle_contact_rad`.

        Raises:
            SequenceError: get_arm_state fails or reports no gripper angle, the angle is outside GRIP_CONTACT_MIN_RAD
                to GRIP_CONTACT_MAX_RAD, or the arm sequence config refuses it.
        """
        try:
            jaw = self.measured().get(GRIPPER_JOINT)
        except StepFailed as exc:
            raise SequenceError(str(exc)) from exc
        if jaw is None:
            raise SequenceError("get_arm_state reports no gripper angle: cannot record the grip")
        if not GRIP_CONTACT_MIN_RAD <= jaw <= GRIP_CONTACT_MAX_RAD:
            raise SequenceError(
                f"the measured gripper angle {jaw:.4f} rad is outside {GRIP_CONTACT_MIN_RAD:g} to "
                f"{GRIP_CONTACT_MAX_RAD:g} rad: not a handle seated in the jaws"
            )
        try:
            return jaw, self.with_contact(jaw)
        except ValidationError as exc:
            raise SequenceError(f"the arm sequence config refuses a handle contact of {jaw:.4f} rad: {exc}") from exc

    def write_grip(self, record: GripRecord) -> None:
        """Write the grip record atomically (a temp file, then os.replace); no-op without a grip_file.

        Args:
            record (GripRecord): The record to write.

        Raises:
            OSError: The directory or the file cannot be written (the previous file, if any, is unchanged).
        """
        if self.grip_file is None:
            return
        self.grip_file.parent.mkdir(parents=True, exist_ok=True)
        temp = self.grip_file.with_name(self.grip_file.name + ".tmp")
        content = {
            "handle_contact_rad": record.handle_contact_rad,
            "recorded_at": record.recorded_at.isoformat(timespec="seconds"),
        }
        temp.write_text(json.dumps(content))
        os.replace(temp, self.grip_file)

    def reset_grip(self, confirmed: bool) -> dict[str, Any]:
        """Remove the grip record (a corrupt one too): the configured handle contact is active again. Nothing moves.

        Args:
            confirmed (bool): The user confirmed the reset.

        Returns:
            dict[str, Any]: The state() after the reset (also without a record: the reset is idempotent).

        Raises:
            ValueError: Not confirmed (nothing changes).
            Busy: A sequence, Home or grip measure runs, the worker thread is alive, or the phase is neither idle nor
                holding.
            SequenceError: The file cannot be removed (logged; the active contact is unchanged).
        """
        if not confirmed:
            raise ValueError(MESSAGE_GRIP_RESET_CONFIRM)
        with self.lock:
            problem = self.grip_problem()
            if problem is not None:
                raise Busy(problem)
            self.run_id = new_run_id("grip", self.now())
            previous = self.seq.handle_contact_rad
            had_record = self.grip_recorded_at is not None
            file = None if self.grip_file is None else str(self.grip_file)
            try:
                if self.grip_file is not None:
                    self.grip_file.unlink(missing_ok=True)
            except OSError as exc:
                self.append_event(
                    logging.ERROR, f"grip record not reset: {exc}", GRIP_STEP, "error", reason=str(exc), file=file
                )
                raise SequenceError(f"cannot remove the grip record {self.grip_file}: {exc}") from exc
            self.seq = self.config_seq
            self.grip_recorded_at = None
            self.append_event(
                logging.INFO,
                f"grip record reset: handle contact {self.seq.handle_contact_rad:.4f} rad from the config"
                f"{f' (was {previous:.4f} rad)' if had_record else ''}; nothing moved",
                GRIP_STEP,
                "reset",
                previous_rad=previous,
                had_record=had_record,
                handle_contact_rad=self.seq.handle_contact_rad,
                file=file,
            )
        return self.state()

    def record_board(self, run_id: str | None = None) -> None:
        """Write the board record (atomically); a failed write is logged, the in-memory state stands.

        Args:
            run_id (str | None): The run the log lines belong to (None: the current run).
        """
        if self.board_file is None:
            return
        mount = self.cfg.fit.board_mount
        record = {
            "session_id": self.session_id,
            "mount": {"xyz": list(mount.xyz_init), "rpy": list(mount.rpy_init)},
            "clipped_at": self.now().isoformat(timespec="seconds"),
        }
        try:
            self.board_file.parent.mkdir(parents=True, exist_ok=True)
            temp = self.board_file.with_name(self.board_file.name + ".tmp")
            temp.write_text(json.dumps(record))
            os.replace(temp, self.board_file)
        except OSError as exc:
            self.log(BOARD_RECORD_STEP, "error", logging.ERROR, f"cannot write: {exc}", run_id, file=self.board_file)
            return
        self.log(BOARD_RECORD_STEP, "written", run_id=run_id, file=self.board_file, **record)

    def clear_board(self, run_id: str | None = None) -> None:
        """Remove the board record.

        Args:
            run_id (str | None): The run the log line belongs to (None: the current run).
        """
        if self.board_file is None:
            return
        try:
            self.board_file.unlink(missing_ok=True)
        except OSError as exc:
            self.log(BOARD_RECORD_STEP, "error", logging.ERROR, f"cannot remove: {exc}", run_id, file=self.board_file)

    def log(
        self,
        step: str,
        outcome: str,
        level: int = logging.INFO,
        reason: str | None = None,
        run_id: str | None = None,
        **data: Any,
    ) -> dict[str, Any]:
        """Log one journal-only calib_event of this run (not a UI event).

        Args:
            step (str): The step.
            outcome (str): The outcome (ok, refused, converged, ...).
            level (int): Logging level.
            reason (str | None): Why it failed or was refused.
            run_id (str | None): The run the event belongs to; None takes the current `run_id` (a worker that logs
                after clearing `running` passes its own, as a new claim may have replaced it).
            **data (Any): Inputs and measured values.

        Returns:
            dict[str, Any]: The logged event.
        """
        return calib_event(LOG, FLOW, run_id or self.run_id, step, outcome, level, reason, **data)

    def timed(self, step: str, **data: Any) -> Any:
        """A timed worker step of this run (telemetry.step_timer on the injected clock; a stop logs `stopped`).

        Args:
            step (str): The step; it is also `current_step`, named by the sequence end event of a failure.
            **data (Any): The step's inputs.

        Returns:
            Any: The step_timer context manager; it yields the dict of fields logged with the end.
        """
        self.current_step = step
        return step_timer(LOG, FLOW, self.run_id, step, self.clock, (Stopped,), **data)

    def event(
        self,
        level: int,
        message: str,
        step: str,
        outcome: str,
        phase: str | None = None,
        reason: str | None = None,
        **data: Any,
    ) -> None:
        """Log one sequence or Home event and keep it in the recent events (the caller does not hold `lock`).

        Args:
            level (int): logging.INFO, logging.WARNING or logging.ERROR.
            message (str): What happened, with the reason of a failure.
            step (str): The step the event ends or reports.
            outcome (str): Its outcome (start, done, error, ...).
            phase (str | None): The phase the event belongs to; None takes the current phase.
            reason (str | None): Why it failed, was refused or stopped.
            **data (Any): Inputs and measured values for the journal.
        """
        with self.lock:
            self.append_event(level, message, step, outcome, phase, reason, **data)

    def append_event(
        self,
        level: int,
        message: str,
        step: str,
        outcome: str,
        phase: str | None = None,
        reason: str | None = None,
        **data: Any,
    ) -> None:
        """Log one event as a calib_event and keep it in the recent events: one call for the journal and the UI. The
        caller holds `lock` (an end of the worker records its event in the same update that clears `running`, so a
        poller that sees the end also sees why).

        Args:
            level (int): logging.INFO, logging.WARNING or logging.ERROR.
            message (str): What happened, with the reason of a failure (the UI text; also in the journal line).
            step (str): The step the event ends or reports.
            outcome (str): Its outcome (start, done, error, ...).
            phase (str | None): The phase the event belongs to; None takes the current phase.
            reason (str | None): Why it failed, was refused or stopped.
            **data (Any): Inputs and measured values for the journal.
        """
        phase = phase or self.phase
        calib_event(LOG, FLOW, self.run_id, step, outcome, level, reason, message=message, phase=phase, **data)
        stamp = self.now().isoformat(timespec="seconds")
        self.events.append({"time": stamp, "phase": phase, "level": EVENT_LEVELS[level], "message": message})

    def state(self) -> dict[str, Any]:
        """The sequence state for the HTTP API.

        Returns:
            {running, phase, session_id, tier, arm_pose, total_poses, captured, skipped, status, error, big_joints,
            waiting_since, board_on, board_record, events}: board_on is whether the board is on the gripper (attached
            or stated; a holding phase with it on may resume), arm_pose is the 0-based index over the whole pose set of the pose
            worked on (None before the first), skipped maps each skip reason to its count, big_joints names the joints
            tier 2 moves, waiting_since is the clock time a confirm wait began (None outside a wait), board_record is
            board_record_state() (the record file at any age, None without one), events
            are the newest EVENT_LIMIT `{time (ISO 8601), phase, level (info, warning, error), message}`, newest first.
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
                "board_on": self.board_on,
                "board_record": self.board_record_state(),
                "grip": self.grip_state(),
                "events": [dict(e) for e in reversed(self.events)],
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

        A Home in progress is not interrupted.

        Returns:
            The state() after the request.
        """
        self.stop_event.set()
        return self.state()

    def claim(self, phase: str, status: str, run_prefix: str, resume: bool = False, board_gripped: bool = False) -> str:
        """Mark a sequence or Home as running under a new run id, then check `blocked` (outside the lock: it reads
        other controllers).

        Args:
            phase: Phase to enter.
            status: Status line to show.
            run_prefix: Prefix of the new run id (seq, home); a refused claim keeps the run id of the running work.
            resume: A resume with the board on: claimed only while holding with the board on, or from idle with
                `board_gripped`.
            board_gripped (bool): The user states the board is gripped in the jaws (allows a resume from idle).

        Returns:
            str: The phase before the claim.

        Raises:
            Busy: Something runs already, the arm holds the board pose (a new base move), a resume outside holding with
                the board on (or idle with `board_gripped`), or `blocked` names a reason.
        """
        with self.lock:
            if self.running:
                raise Busy(f"a head-arm sequence is running (phase {self.phase}): stop it first")
            resumable = (self.phase == HOLDING and self.board_on) or (self.phase == IDLE and board_gripped)
            if resume and not resumable:
                raise Busy(f"{MESSAGE_RESUME_PHASE} (phase {self.phase}, board on {self.board_on})")
            if phase == MOVING_TO_BASE and self.phase == HOLDING and not resume:
                raise Busy(f"the arm holds the board pose: {MESSAGE_HOME} before a new sequence")
            previous = self.phase
            self.running = True
            self.run_id = new_run_id(run_prefix, self.now())
        reason = self.blocked()
        with self.lock:
            if reason:
                self.running = False
                self.log(phase, "refused", logging.WARNING, reason)
                raise Busy(reason)
            self.phase = phase
            self.status = status
            self.error = None
            self.waiting_since = None
        return previous

    def give_up(self, phase: str, error: str, event: str, step: str) -> None:
        """Undo a claim that failed before the worker started, and record the refusal as an error event.

        Args:
            phase (str): The phase to return to.
            error (str): Why the start was refused (the state error).
            event (str): The event message.
            step (str): The refused step (sequence or home).
        """
        with self.lock:
            self.append_event(logging.ERROR, event, step, "refused", reason=error)
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
            Busy: A sequence or Home runs, the arm holds the board pose, or `blocked` names a reason.
            SequenceError: The arm or head URDF is unusable (nothing is called).
        """
        if not confirmed:
            raise ValueError(
                "the base move needs confirmed=True: it moves " + ", ".join(BASE_JOINTS) + " (no board attached)"
            )
        self.claim(MOVING_TO_BASE, "preparing the pose check", "seq")
        try:
            plan = self.prepare()
        except SequenceError as exc:
            self.give_up(IDLE, str(exc), f"sequence start refused: {exc}", SEQUENCE_STEP)
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
        self.clear_board()
        self.stop_event.clear()
        seq = self.seq
        self.event(
            logging.INFO,
            f"sequence started: {len(plan.poses)} poses",
            SEQUENCE_STEP,
            "start",
            poses=len(plan.poses),
            tiers=Counter(tier for tier, _ in plan.poses),
            base_pose=seq.base_pose,
            insert_gripper_rad=seq.insert_gripper_rad,
            grip_rad=seq.grip_rad,
            handle_contact_rad=seq.handle_contact_rad,
            handle_squeeze_rad=seq.handle_squeeze_rad,
            grip_converge_tolerance_rad=seq.grip_converge_tolerance_rad,
            speed_scale=seq.speed_scale,
            head_offsets_deg=seq.head_offsets_deg,
            settle_s=seq.settle_s,
            confirm_timeout_s=seq.confirm_timeout_s,
            still_timeout_s=self.cfg.still.timeout_s,
            nominal_mount=nominal_mount(self.cfg),
            board_margin_m=seq.board_margin_m(nominal_mount(self.cfg)),
            board_mount={"xyz": self.cfg.fit.board_mount.xyz_init, "rpy": self.cfg.fit.board_mount.rpy_init},
            pose_check=seq.pose_check,
            camera_frame=self.cfg.camera_frame,
        )
        self.thread = threading.Thread(target=self.work, args=(plan, False), name="head-arm-sequence", daemon=True)
        self.thread.start()
        return self.state()

    def resume_with_board(self, confirmed: bool, board_gripped: bool = False) -> dict[str, Any]:
        """Resume the capture with the board on: a new run from holding (or idle) to the base pose, then the tiers.

        The gripper is commanded to its measured angle (never opened: the board may be taped into the jaws); the
        board confirm and the handle grip are skipped; the still session and the tiers run as after Board attached.
        From idle (a restarted node forgot the board) the user must also state the board is gripped in the jaws, and
        the measured jaw must be closed (`jaws_closed_on_board`) before board_on is set and anything moves.

        Args:
            confirmed (bool): The user confirmed the base move with the board on.
            board_gripped (bool): The user states the calibration board is gripped in the jaws (needed from idle).

        Returns:
            dict[str, Any]: The state() after the start.

        Raises:
            ValueError: Not confirmed, or from idle without `board_gripped` (nothing is called).
            Busy: A sequence or Home runs, the phase is neither holding with the board on nor idle with
                `board_gripped`, or `blocked` names a reason.
            SequenceError: The arm or head URDF is unusable, or (from idle) the measured jaw is open or not reported
                (nothing moved; the phase stays as it was).
        """
        if not confirmed:
            raise ValueError(MESSAGE_RESUME_CONFIRM)
        with self.lock:
            from_idle = not self.running and self.phase == IDLE
        if from_idle and not board_gripped:
            raise ValueError(MESSAGE_RESUME_GRIPPED)
        previous = self.claim(
            MOVING_TO_BASE, "preparing the pose check (board on)", "seq", resume=True, board_gripped=board_gripped
        )
        try:
            plan = self.prepare()
            jaw = self.jaws_closed_on_board() if previous == IDLE else None
        except SequenceError as exc:
            self.give_up(previous, str(exc), f"resume with the board on refused: {exc}", SEQUENCE_STEP)
            raise
        if jaw is not None:
            with self.lock:
                self.board_on = True
            self.event(
                logging.INFO,
                f"board confirmed gripped by the user (from idle, jaw {jaw:.4f} rad)",
                BOARD_CONFIRMED_STEP,
                "confirmed",
                from_phase=IDLE,
                by="user",
                jaw_rad=jaw,
                closed_max_rad=self.jaw_closed_max_rad(),
                restored_session_id=self.session_id,
                recorded_mount=self.recorded_mount,
            )
            self.record_board()
        with self.lock:
            previous_session = self.session_id
            self.recorded_mount = self.fitted_mount(previous_session) or self.recorded_mount
            self.session_id = None
            self.tier = self.arm_pose = None
            self.total_poses = len(plan.poses)
            self.captured = 0
            self.skipped = Counter()
            self.big_joints_allowed = None
            self.status = "taking the arm (board on)"
        self.stop_event.clear()
        self.event(
            logging.INFO,
            f"sequence resumed with the board on: {len(plan.poses)} poses",
            SEQUENCE_STEP,
            "start",
            resume=True,
            from_phase=previous,
            previous_session_id=previous_session,
            poses=len(plan.poses),
            tiers=Counter(tier for tier, _ in plan.poses),
            base_pose=self.seq.base_pose,
            speed_scale=self.seq.speed_scale,
            board_mount={"xyz": self.cfg.fit.board_mount.xyz_init, "rpy": self.cfg.fit.board_mount.rpy_init},
            recorded_mount=self.recorded_mount,
        )
        self.thread = threading.Thread(target=self.work, args=(plan, True), name="head-arm-sequence", daemon=True)
        self.thread.start()
        return self.state()

    def jaw_closed_max_rad(self) -> float:
        """The largest measured gripper angle that still counts as jaws closed on the board handle.

        Returns:
            float: `handle_contact_rad + grip_converge_tolerance_rad` (rad): the contact angle plus the mcp_server
                converge tolerance; above it the jaws are clearly open (no handle between them).
        """
        return self.seq.handle_contact_rad + self.seq.grip_converge_tolerance_rad

    def jaws_closed_on_board(self) -> float:
        """Measure the gripper (get_arm_state) and refuse open jaws before a resume from idle (nothing moves).

        Returns:
            float: The measured gripper angle (rad), at most `jaw_closed_max_rad()`.

        Raises:
            SequenceError: get_arm_state fails or reports no gripper angle, or the jaw is above `jaw_closed_max_rad()`.
        """
        limit = self.jaw_closed_max_rad()
        try:
            jaw = self.measured().get(GRIPPER_JOINT)
        except StepFailed as exc:
            raise SequenceError(str(exc)) from exc
        verdict = "not reported" if jaw is None else ("open" if jaw > limit else "closed")
        self.log(
            "gripper_measured",
            verdict,
            logging.INFO if verdict == "closed" else logging.WARNING,
            measured_rad=jaw,
            closed_max_rad=limit,
            handle_contact_rad=self.seq.handle_contact_rad,
            grip_converge_tolerance_rad=self.seq.grip_converge_tolerance_rad,
        )
        if jaw is None:
            raise SequenceError("get_arm_state reports no gripper angle: cannot tell the jaws grip the board")
        if jaw > limit:
            raise SequenceError(f"{MESSAGE_RESUME_OPEN_JAWS} (measured {jaw:.4f} rad, closed at most {limit:.4f} rad)")
        return jaw

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
        """The user seated the board handle in the jaws: close the grip, start the still session and tiers 0 and 1.

        Returns:
            The state() after the request.

        Raises:
            Busy: The sequence is not in phase waiting_board.
        """
        with self.lock:
            if not self.running or self.phase != WAITING_BOARD:
                raise Busy(f"Board attached is accepted in phase {WAITING_BOARD} only (phase {self.phase})")
            self.board_on = True
        self.event(logging.INFO, "board attached", "board_attached", "confirmed")
        self.record_board()
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
        self.event(
            logging.INFO,
            "tier 2 allowed" if confirmed else "tier 2 skipped by the user",
            "big_joints",
            "allowed" if confirmed else "declined",
            joints=BIG_JOINTS,
        )
        return self.state()

    def home(self, confirmed: bool, board_on: bool = False) -> dict[str, Any]:
        """Home the arm (mcp_server homes the arm, then the head), with the board on or off the gripper.

        Board not on (this sequence did not attach it and the user did not state it): the plain home, refused by
        mcp_server's carry reasons as ever. Board on (the sequence attached it, or `board_on`): whatever the load
        reads, the stored home is read from `arm_home_file`, the path to it is checked with the board outline and the
        sag margin, the arm moves there and mcp_server's internal board park finishes it (it checks the board again on
        its home legs). Any carry reason other than the holding load of the gripped board handle refuses.

        Args:
            confirmed: The user confirmed the home move.
            board_on: The user states, now, that the calibration board is on the gripper (a node that never saw the
                attach, or restarted since): recorded in the board file; the path checks still run.

        Returns:
            The state() after the start of the Home.

        Raises:
            ValueError: Not confirmed (nothing is called).
            Busy: A sequence or Home runs, or `blocked` names a reason.
            SequenceError: get_control_state fails, reports a carried object other than the board (its reasons are
                quoted), the stored home is missing or invalid (the file is named) or the board path check refuses
                (its rules are quoted); nothing moved.
        """
        if not confirmed:
            raise ValueError("Home needs confirmed=True: the arm moves to its home pose")
        previous = self.claim(HOMING, "checking what the gripper carries", "home")
        if board_on:
            with self.lock:
                stated = not self.board_on
                self.board_on = True
            if stated:
                self.record_board()
        params: dict[str, Any] | None = None
        home: dict[str, float] | None = None
        try:
            control = self.robot.control_state()
            reasons = carrying_reasons(control)
            with self.lock:
                board = self.board_on
            others = [r for r in reasons if not r.startswith(HOLDING_LOAD_PREFIX)]
            self.log(
                "control_state",
                "ok",
                reasons=reasons,
                other_reasons=others,
                board_on=board,
                board_on_stated=board_on,
                carrying=((control.data or {}).get("arm") or {}).get("carrying"),
            )
            if others or (reasons and not board):
                hint = "" if others else MESSAGE_STATE_BOARD
                raise SequenceError(f"Home refused: mcp_server reports carrying ({'; '.join(reasons)}){hint}")
            if board:
                with self.lock:
                    self.status = "checking the board's home path"
                try:
                    home = load_arm_home(Path(self.cfg.arm_home_file))
                except ArmHomeError as exc:
                    raise SequenceError(f"Home refused: {exc}") from exc
                params = self.check_board_home_path(home)
        except SequenceError as exc:
            self.give_up(previous, str(exc), str(exc), HOME_STEP)
            raise
        except Exception as exc:
            error = f"Home refused: {type(exc).__name__}: {exc}"
            self.give_up(previous, error, error, HOME_STEP)
            raise SequenceError(error) from exc
        with self.lock:
            self.status = "homing the arm, then the head"
        self.event(
            logging.INFO,
            f"home started (board {'on' if params is not None else 'off'})",
            HOME_STEP,
            "start",
            board_on=params is not None,
            mount=params,
            home_pose=home,
            speed_scale=self.seq.speed_scale,
            previous_phase=previous,
        )
        self.stop_event.clear()
        self.thread = threading.Thread(
            target=self.home_work, args=(previous, params, home), name="head-arm-home", daemon=True
        )
        self.thread.start()
        return self.state()

    def check_board_home_path(self, home: dict[str, float]) -> dict[str, Any]:
        """Check the path from the measured joints to the stored home with the board on the gripper.

        The board keeps `park_board_sag_margin_m` more than the sequence margin from the body and the head, and that
        much more than `min_board_height_m` above the floor (the board sags below FK under its weight).

        Args:
            home (dict[str, float]): The stored home, the five arm joints (rad, load_arm_home).

        Raises:
            SequenceError: A URDF is unusable, or the path check names a violated rule (nothing moved).
            StepFailed: get_arm_state fails or reports no measured joints.

        Returns:
            dict[str, Any]: The board mount the check used, `{"xyz", "rpy"}`, which is all mcp_server's own check of the
                home legs takes from the node (it owns the outline, the margins and the floor height).
        """
        seq = self.seq
        sag = seq.park_board_sag_margin_m
        fit = self.cfg.fit
        fitted = self.fitted_mount(self.session_id)
        used = fitted if fitted is not None else self.recorded_mount
        if used is not None:
            mount = fit.board_mount.model_copy(update={"xyz_init": tuple(used["xyz"]), "rpy_init": tuple(used["rpy"])})
            fit = fit.model_copy(update={"board_mount": mount})
        margin = seq.board_margin_m(fitted is None and nominal_mount(self.cfg)) + sag
        pose_check = seq.pose_check.model_copy(update={"min_board_height_m": seq.pose_check.min_board_height_m + sag})
        try:
            checker = PoseChecker(self.arm_urdf(), self.head_urdf(), fit, pose_check, margin)
        except (ValueError, KeyError, OSError, SyntaxError) as exc:
            raise SequenceError(f"cannot read the arm or head URDF: {type(exc).__name__}: {exc}") from exc
        start = self.measured()
        gripper = max(start.get(GRIPPER_JOINT, seq.grip_rad), seq.grip_rad)
        entries = checker.check_path(start, dict(home), gripper)
        self.log(
            "home_path_check",
            "refused" if entries else "ok",
            logging.WARNING if entries else logging.INFO,
            f"{len(entries)} violated entries" if entries else None,
            start=start,
            target=home,
            gripper_rad=gripper,
            mount_source="fitted" if fitted is not None else ("recorded" if used is not None else "configured"),
            mount={"xyz": fit.board_mount.xyz_init, "rpy": fit.board_mount.rpy_init},
            margin_m=margin,
            sag_margin_m=sag,
            min_board_height_m=pose_check.min_board_height_m,
            entries=entries,
        )
        if entries:
            quoted = "; ".join(entries[:MAX_QUOTED_ENTRIES])
            more = f" (+{len(entries) - MAX_QUOTED_ENTRIES} more)" if len(entries) > MAX_QUOTED_ENTRIES else ""
            raise SequenceError(
                f"Home refused by the path check with the board on, nothing was moved and the arm keeps holding: "
                f"{quoted}{more}"
            )
        return {"xyz": list(fit.board_mount.xyz_init), "rpy": list(fit.board_mount.rpy_init)}

    # worker thread

    def home_work(
        self, previous: str, board: dict[str, Any] | None = None, home: dict[str, float] | None = None
    ) -> None:
        """Worker: take the arm and release it with a home; a failure leaves the phase as it was.

        Args:
            previous (str): The phase before the Home.
            board (dict[str, Any] | None): The board is on the gripper (the mount check_board_home_path returned):
                move to `home` first (its path was checked), then finish through mcp_server's internal board park.
            home (dict[str, float] | None): The stored home the press read (set when `board` is).
        """
        run_id = self.run_id  # kept: a claim after the end may replace self.run_id
        error: str | None = None
        status = ""
        lease = False
        reset: dict[str, Any] = {}
        try:
            with self.timed("home_take_control"):
                self.call(self.robot.take_arm(), "take_control")
            lease = True
            if board is not None and home is not None:
                with self.timed("home_move", target=home, speed_scale=self.seq.speed_scale):
                    self.home_move(home)
            route = "park_calibration_board" if board is not None else "release_control (home)"
            with self.timed("home_release", route=route, mount=board) as fields:
                if board is not None:
                    result = self.call(self.robot.park_with_board(board), route)
                    data = (result.data or {}).get("result") or {}
                else:
                    result = self.call(self.robot.release_arm(keep_position=False), route)
                    data = result.data or {}
                lease = False
                reset = data.get("reset") or {}
                fields.update(reset=reset, result=data)
                if reset.get("arm") != HOMED:
                    raise StepFailed(f"the release did not home the arm: reset.arm {reset.get('arm')!r}")
            status = f"homed: arm {reset.get('arm')}, head {reset.get('head', 'n/a')}"
        except StepFailed as exc:
            error = str(exc)
        except Exception as exc:  # reported as the Home error; the phase stays as it was
            error = f"{type(exc).__name__}: {exc}"
        if lease:
            held = self.robot.release_arm(keep_position=True)
            if held.is_error:
                self.log("hold", "error", logging.ERROR, held.text)
            else:
                self.log("hold", "ok", release=held.data)
            if held.is_error:
                error = f"{error}; release_control (hold) failed: {held.text}"
        with self.lock:
            if error is None:
                self.append_event(logging.INFO, f"home done: {status}", HOME_STEP, "done", reset=reset)
            else:
                self.append_event(
                    logging.ERROR,
                    f"home failed: {error}",
                    HOME_STEP,
                    "error",
                    reason=error,
                    failed_step=self.current_step,
                    reset=reset,
                )
            self.running = False
            if error is None:
                self.phase = IDLE
                self.status = status
                self.error = None
                self.board_on = False
            else:
                self.phase = previous
                self.status = f"home failed: {error}"
                self.error = error
        if error is None:
            self.clear_board(run_id)

    def home_move(self, home: dict[str, float]) -> None:
        """Move to the stored home at the sequence speed scale (guarded by mcp_server); the path was checked.

        Args:
            home (dict[str, float]): The stored home, the five arm joints (rad).

        Raises:
            StepFailed: The move fails or does not converge.
        """
        result = self.call(
            self.robot.move_arm_joints(dict(home), self.seq.speed_scale),
            "move_arm_joints to the stored home",
        )
        status, message = self.log_motion("move_arm_joints", result, label="the stored home", targets=home)
        if status != CONVERGED:
            raise StepFailed(f"move_arm_joints to the stored home did not converge: {status}: {message}")

    def work(self, plan: Plan, resume: bool) -> None:
        """Worker: run the sequence, then always stop the capture and release the arm holding its position.

        Args:
            plan (Plan): The sequence plan.
            resume (bool): A resume with the board on (run_sequence).
        """
        run_id = self.run_id  # kept: a claim after the end may replace self.run_id
        error: str | None = None
        outcome = "done"
        try:
            self.run_sequence(plan, resume)
        except Stopped:
            outcome = REASON_STOPPED
        except StepFailed as exc:
            outcome, error = "error", str(exc)
        except Exception as exc:  # reported as the sequence error; the finally block still holds the arm
            outcome, error = "error", f"{type(exc).__name__}: {exc}"
            LOG.exception("head-arm sequence %s raised in step %s", self.run_id, self.current_step)
        finally:
            error = self.hold(error)
            with self.lock:
                ended_in = self.phase
                summary = {
                    "ended_in": ended_in,
                    "session_id": self.session_id,
                    "captured": self.captured,
                    "skipped": dict(self.skipped),
                    "big_joints_allowed": self.big_joints_allowed,
                    "board_on": self.board_on,
                }
                self.running = False
                self.phase = HOLDING if self.board_on else IDLE
                self.waiting_since = None
                self.error = error
                detail = f": {error}" if error else ""
                follow = f"; {MESSAGE_HOME}" if self.phase == HOLDING else ""
                note = "; tier 2 skipped" if self.big_joints_allowed is False else ""
                self.status = f"{outcome}{detail}{note}{follow}"
                if error is not None:
                    self.append_event(
                        logging.ERROR,
                        f"sequence error: {error}",
                        SEQUENCE_STEP,
                        "error",
                        ended_in,
                        error,
                        failed_step=self.current_step,
                        **summary,
                    )
                elif outcome == REASON_STOPPED:
                    self.append_event(
                        logging.WARNING,
                        "sequence stopped",
                        SEQUENCE_STEP,
                        REASON_STOPPED,
                        ended_in,
                        stopped_in_step=self.current_step,
                        **summary,
                    )
                else:
                    self.append_event(
                        logging.INFO, f"sequence done: {self.status}", SEQUENCE_STEP, "done", ended_in, **summary
                    )
        if self.board_on:
            self.record_board(run_id)  # now with the capture session id

    def hold(self, error: str | None) -> str | None:
        """Stop the still capture and release the arm keeping its position; failures are added to the error."""
        problems = [error] if error else []
        own: list[str] = []
        try:
            self.capture.stop()
        except Exception as exc:  # the arm is still released below
            own.append(f"stopping the still capture failed: {type(exc).__name__}: {exc}")
        result = self.robot.release_arm(keep_position=True)
        if result.is_error:
            own.append(f"release_control (hold) failed: {result.text}")
        self.log(
            "hold",
            "error" if own else "ok",
            logging.ERROR if own else logging.INFO,
            "; ".join(own) or None,
            release=result.data,
        )
        return "; ".join(problems + own) or None

    def run_sequence(self, plan: Plan, resume: bool = False) -> None:
        """Base pose, the board confirm, the handle grip, then capture_tiers.

        Args:
            plan (Plan): The sequence plan.
            resume (bool): Resume with the board on: the base move keeps the gripper at its measured angle, and the
                board confirm and the handle grip are skipped.
        """
        with self.lock:
            self.held_head = None  # never a band from an earlier run: the head may have moved since
        with self.timed("take_control"):
            self.call(self.robot.take_arm(), "take_control")
        self.check_alive()
        base = dict(self.seq.base_pose)
        if resume:
            self.resume_base_move(plan, base)
        else:
            with self.lock:
                self.status = "moving to the base pose (no board)"
            with self.timed("base_move", target=base, gripper_rad=self.seq.insert_gripper_rad):
                self.checked_move(plan, base, "the base pose", {**base, GRIPPER_JOINT: self.seq.insert_gripper_rad})
            self.event(logging.INFO, "base pose reached", "base_move", "reached")
            with self.timed("wait_board", timeout_s=self.seq.confirm_timeout_s):
                self.wait_confirm(WAITING_BOARD, MESSAGE_BOARD, lambda: self.board_on)
            with self.timed(
                "handle_grip",
                grip_rad=self.seq.grip_rad,
                handle_contact_rad=self.seq.handle_contact_rad,
                handle_squeeze_rad=self.seq.handle_squeeze_rad,
                grip_converge_tolerance_rad=self.seq.grip_converge_tolerance_rad,
            ):
                self.close_on_handle(plan)
        self.capture_tiers(plan, base)

    def resume_base_move(self, plan: Plan, base: dict[str, float]) -> None:
        """Move to the base pose with the board on, the gripper commanded to its measured angle (it never opens).

        Args:
            plan (Plan): The sequence plan.
            base (dict[str, float]): The base pose (arm joints).

        Raises:
            StepFailed: get_arm_state reports no gripper angle, or the checked move fails (nothing moved on a refusal).
        """
        with self.lock:
            self.status = "moving to the base pose (board on, gripper kept)"
        jaw = self.measured().get(GRIPPER_JOINT)
        if jaw is None:
            self.log("gripper_measured", "not reported", logging.WARNING, "resume needs the measured gripper angle")
            raise StepFailed("get_arm_state reports no gripper angle: cannot keep the jaws on the board; nothing moved")
        with self.timed("base_move", target=base, gripper_rad=jaw, resume=True):
            self.checked_move(plan, base, "the base pose (board on)", {**base, GRIPPER_JOINT: jaw})
        self.event(
            logging.INFO, f"base pose reached with the board on (jaw kept at {jaw:.4f} rad)", "base_move", "reached"
        )

    def capture_tiers(self, plan: Plan, base: dict[str, float]) -> None:
        """Still session, tiers 0 and 1, the big-joint confirm, tier 2, back to the base pose (the board is on).

        Args:
            plan (Plan): The sequence plan.
            base (dict[str, float]): The base pose to return to.
        """
        with self.timed("still_session") as fields:
            session_id = self.capture.start_still_session(None)
            fields["session_id"] = session_id
        with self.lock:
            self.session_id = session_id
            self.phase = CAPTURING
            self.status = "waiting for the colour camera_info"
        self.event(
            logging.INFO, f"still session {session_id} started", "still_session", "started", session_id=session_id
        )
        with self.timed("camera_info", topic=self.cfg.topics.rgb_info, timeout_s=self.cfg.still.timeout_s) as fields:
            intrinsics = self.camera_intrinsics()
            fields.update(
                width=intrinsics.width,
                height=intrinsics.height,
                fx=intrinsics.fx,
                fy=intrinsics.fy,
                cx=intrinsics.cx,
                cy=intrinsics.cy,
                distortion=intrinsics.distortion,
            )
        for tier in range(BIG_JOINT_TIER):
            self.capture_tier(plan, tier, intrinsics)
        big = ", ".join(BIG_JOINTS)
        with self.timed("wait_big_joints", joints=BIG_JOINTS, timeout_s=self.seq.confirm_timeout_s) as fields:
            self.wait_confirm(
                WAITING_BIG_JOINTS,
                f"Tier 2 moves {big}: allow these moves, or skip tier 2",
                lambda: self.big_joints_allowed is not None,
            )
            fields["allowed"] = self.big_joints_allowed
        with self.lock:
            self.phase = CAPTURING
        if self.big_joints_allowed:
            self.capture_tier(plan, BIG_JOINT_TIER, intrinsics)
        self.check_alive()
        with self.lock:
            self.phase = RETURNING
            self.tier = None
            self.status = "moving back to the base pose"
        with self.timed("return_to_base", target=base):
            self.checked_move(plan, base, "the base pose (return)", base)
        self.event(logging.INFO, "back at the base pose", "return_to_base", "reached")

    def close_on_handle(self, plan: Plan) -> None:
        """Close the gripper on the handle to `grip_rad`, then refuse an unseated handle or a slack grip.

        Args:
            plan: The sequence plan.

        Raises:
            StepFailed: The grip move fails or does not converge, no gripper angle is reported, the measured angle
                is below `handle_contact_rad - handle_squeeze_rad / 2` (the handle is not seated), or it is at or
                above `grip_rad + grip_converge_tolerance_rad` (mcp_server relaxes that residual: the grip goes slack).
        """
        with self.lock:
            self.phase = MOVING_TO_BASE
            self.status = "closing the gripper on the handle"
        base = dict(self.seq.base_pose)
        self.checked_move(plan, base, "the handle grip", {GRIPPER_JOINT: self.seq.grip_rad})
        measured = self.measured().get(GRIPPER_JOINT)
        limit = self.seq.handle_contact_rad - self.seq.handle_squeeze_rad / 2
        slack = self.seq.grip_rad + self.seq.grip_converge_tolerance_rad
        if measured is None:
            verdict = "not reported"
        elif measured < limit:
            verdict = "unseated"
        elif measured >= slack:
            verdict = "slack"
        else:
            verdict = "seated"
        self.log(
            "gripper_measured",
            verdict,
            logging.INFO if verdict == "seated" else logging.WARNING,
            measured_rad=measured,
            unseated_below_rad=limit,
            slack_from_rad=slack,
            grip_rad=self.seq.grip_rad,
        )
        if measured is None:
            raise StepFailed(
                "get_arm_state reports no gripper angle after the handle grip: cannot tell a seated handle"
            )
        if measured < limit:
            raise StepFailed(f"{MESSAGE_UNSEATED} (measured {measured:.4f} rad, limit {limit:.4f} rad)")
        if measured >= slack:
            raise StepFailed(f"{MESSAGE_SLACK_GRIP} (measured {measured:.4f} rad, limit {slack:.4f} rad)")
        self.event(logging.INFO, f"handle gripped: jaw {measured:.4f} rad", "handle_grip", "gripped", jaw_rad=measured)

    def camera_intrinsics(self) -> CameraIntrinsics:
        """The colour intrinsics of the still session, waiting up to `still.timeout_s` for the first camera_info.

        Raises:
            Stopped: stop() was called.
            StepFailed: No camera_info arrived in time, or it is not a usable pinhole (plumb_bob) camera_info.
        """
        began = self.clock()
        timeout = self.cfg.still.timeout_s
        while True:
            self.check_alive()
            info = self.capture.still_camera_info()
            if info is not None:
                try:
                    return CameraIntrinsics.from_camera_info(
                        info.width, info.height, info.k, info.d, info.distortion_model
                    )
                except ValueError as exc:
                    raise StepFailed(f"the colour camera_info is unusable: {exc}") from exc
            if self.clock() - began >= timeout:
                raise StepFailed(
                    f"no colour camera_info on {self.cfg.topics.rgb_info} within still.timeout_s {timeout:g} s"
                )
            self.sleep(POLL_S)

    def check_alive(self) -> None:
        """Raise Stopped after stop()."""
        if self.stop_event.is_set():
            raise Stopped

    def call(self, result: ToolResult, what: str) -> ToolResult:
        """The result of a tool call; a tool error is logged and stops the sequence quoting it."""
        if result.is_error:
            self.log("tool_error", "error", logging.ERROR, result.text, tool=what, unreachable=result.unreachable)
            raise StepFailed(f"{what} failed: {result.text}")
        return result

    def log_motion(self, tool: str, result: ToolResult, **inputs: Any) -> tuple[str | None, str]:
        """Log a motion tool result (move_arm_joints, set_head_angles) with its status, message and data.

        Args:
            tool (str): The tool name.
            result (ToolResult): The successful tool result.
            **inputs (Any): The command (label, targets, angles).

        Returns:
            tuple[str | None, str]: tool_status(result).
        """
        status, message = tool_status(result)
        converged = status == CONVERGED
        self.log(
            tool,
            status or "no status",
            logging.INFO if converged else logging.WARNING,
            None if converged else message,
            message=message,
            result=result.data,
            **inputs,
        )
        return status, message

    def measured(self) -> dict[str, float]:
        """Measured arm joints from get_arm_state (the gripper too when reported).

        Raises:
            StepFailed: The call failed, no positions were reported, or an arm joint is missing.
        """
        result = self.call(self.robot.arm_state(), "get_arm_state")
        positions = (result.data or {}).get("positions")
        self.log("get_arm_state", "ok" if isinstance(positions, dict) else "no positions", positions=positions)
        if not isinstance(positions, dict):
            raise StepFailed("get_arm_state reports no measured joints (stale joint states); nothing was moved")
        missing = [name for name in ARM_JOINT_NAMES if not isinstance(positions.get(name), int | float)]
        if missing:
            raise StepFailed(f"get_arm_state reports no measured {', '.join(missing)}; nothing was moved")
        return {name: float(positions[name]) for name in BASE_JOINTS if isinstance(positions.get(name), int | float)}

    def head_region(self, plan: Plan) -> HeadRegion | None:
        """The region the held head stays in during an arm move.

        Args:
            plan (Plan): The sequence plan (its checker's head limits and `head_band_deg`).

        Returns:
            HeadRegion | None: `head_band` about `held_head`, None (the full-range envelope) when the head is not held.
        """
        with self.lock:
            held = self.held_head
        return None if held is None else plan.checker.head_band(*held)

    def checked_move(self, plan: Plan, pose: dict[str, float], label: str, targets: dict[str, float]) -> None:
        """Check the path from the measured joints to `pose`, then command `targets`; refusals stop before it.

        The path is checked with the real head over the band about the held head (`head_region`), else with the
        full-range head envelope.

        Args:
            plan: The sequence plan.
            pose: Arm joint target (measured joint space) the path is checked to.
            label: The pose named in the messages.
            targets: What move_arm_joints is sent (`pose`, plus the gripper for the base move; the gripper alone for the
                handle grip).

        Raises:
            StepFailed: get_arm_state fails, the path check refuses, the move fails or does not converge.
        """
        start = self.measured()
        gripper = max(start.get(GRIPPER_JOINT, self.seq.grip_rad), self.seq.grip_rad)
        head = self.head_region(plan)
        entries = plan.checker.check_path(start, pose, gripper, head=head)
        self.log(
            "path_check",
            "refused" if entries else "ok",
            logging.WARNING if entries else logging.INFO,
            f"{len(entries)} violated entries" if entries else None,
            label=label,
            start=start,
            target=pose,
            gripper_rad=gripper,
            head_model="envelope" if head is None else "band",
            head_region=None if head is None else {"pan_rad": head.pan_rad, "tilt_rad": head.tilt_rad},
            entries=entries,
        )
        if entries:
            quoted = "; ".join(entries[:MAX_QUOTED_ENTRIES])
            more = f" (+{len(entries) - MAX_QUOTED_ENTRIES} more)" if len(entries) > MAX_QUOTED_ENTRIES else ""
            raise StepFailed(f"{label} refused by the path check, nothing was moved: {quoted}{more}")
        self.check_alive()
        result = self.call(self.robot.move_arm_joints(targets, self.seq.speed_scale), f"move_arm_joints to {label}")
        status, message = self.log_motion("move_arm_joints", result, label=label, targets=targets)
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

    def plan_views(self, plan: Plan, joints: dict[str, float], intrinsics: CameraIntrinsics) -> list[View]:
        """The views of an arm pose: the head aimed at the board centre on the applied chain, plus each head offset.

        Args:
            plan: The sequence plan.
            joints: Arm joints the board is placed by (the target before the move, the measured ones after it).
            intrinsics: Colour intrinsics of the still session.

        Returns:
            list[View]: One per head offset; a view outside the head limits, inside the D435 min-Z, outside the
                colour FOV or whose head cannot move about its aim at `joints` (`head_clearance`, the real head over
                `head_band(aim)`) carries its reason.
        """
        camera_frame = self.cfg.camera_frame
        gripper = max(joints.get(GRIPPER_JOINT, self.seq.grip_rad), self.seq.grip_rad)
        centre = (plan.checker.link_poses(joints)[BOARD_LINK] @ plan.board_centre)[:3]
        pan, tilt = aim_head(plan.head, centre, camera_frame)
        views = []
        for offset, (dpan, dtilt) in enumerate(self.seq.head_offsets_deg):
            pan_deg, tilt_deg = math.degrees(pan) + dpan, math.degrees(tilt) + dtilt
            detail = f"pan {pan_deg:.2f} deg, tilt {tilt_deg:.2f} deg"
            reason = outside_head_limits(plan.head, pan_deg, tilt_deg)
            if reason is None:
                aim = (math.radians(pan_deg), math.radians(tilt_deg))
                entries = plan.checker.check_view(joints, *aim, intrinsics, camera_frame)
                if not entries:
                    entries = plan.checker.check_head_move(joints, gripper, plan.checker.head_band(*aim))
                if entries:
                    reason = entries[0].partition(": ")[0]
                    detail = f"{detail}: {'; '.join(entries)}"
            views.append(View(offset, pan_deg, tilt_deg, reason, detail))
        return views

    def capture_tier(self, plan: Plan, tier: int, intrinsics: CameraIntrinsics) -> None:
        """Visit every pose of a tier: skip a pose whose every view is skipped at its target, else checked move, aim
        the head on the applied chain and capture each view that is not skipped at the measured joints."""
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
            where_pose = {"tier": tier, "pose": number, "arm_pose": index}
            planned = self.plan_views(plan, pose, intrinsics)
            if all(view.reason is not None for view in planned):
                for view in planned:
                    where = f"tier {tier} arm pose {index} head offset {view.offset}"
                    self.count_skipped(view.reason, f"{where} (pose not visited): {view.detail}", view)
                reasons = ", ".join(sorted({str(view.reason) for view in planned}))
                self.event(
                    logging.WARNING,
                    f"{label} not visited: every view skipped ({reasons})",
                    "pose",
                    "not_visited",
                    reason=reasons,
                    target=pose,
                    **where_pose,
                )
                continue
            with self.timed("pose", target=pose, **where_pose) as fields:
                self.checked_move(plan, pose, label, pose)
                with self.lock:
                    before = (self.captured, sum(self.skipped.values()))
                joints = self.measured()
                for view in self.plan_views(plan, joints, intrinsics):
                    self.check_alive()
                    where = f"tier {tier} arm pose {index} head offset {view.offset}"
                    if view.reason is not None:
                        self.count_skipped(view.reason, f"{where}: {view.detail}", view)
                        continue
                    self.view(plan, joints, index, tier, view.offset, view.pan_deg, view.tilt_deg)
                with self.lock:
                    captured, skipped = self.captured - before[0], sum(self.skipped.values()) - before[1]
                fields.update(captured=captured, skipped=skipped, **where_pose)
            self.event(
                logging.INFO,
                f"{label}: {captured} captured, {skipped} skipped",
                "pose",
                "visited",
                captured=captured,
                skipped=skipped,
                **where_pose,
            )

    def view(
        self,
        plan: Plan,
        joints: dict[str, float],
        index: int,
        tier: int,
        offset: int,
        pan_deg: float,
        tilt_deg: float,
    ) -> None:
        """One view that passed the checks: check the head move, aim, settle and capture a still.

        The head move from the held head to the aim is checked against the arm standing at `joints`
        (`check_head_move` over `head_span(held_head, aim)`, the full-range envelope when the head is not held); a
        refusal skips the view with REASON_HEAD_CLEARANCE and commands nothing. A converged move holds the head at its
        measured angles (the commanded ones when none are reported); any other outcome leaves it not held.

        Args:
            plan (Plan): The sequence plan.
            joints (dict[str, float]): Measured arm joints (the gripper too when reported) the views were planned at.
            index (int): Arm pose index.
            tier (int): Tier of the pose.
            offset (int): Head offset index.
            pan_deg (float): Commanded head_pan (deg).
            tilt_deg (float): Commanded head_tilt (deg).
        """
        where = f"tier {tier} arm pose {index} head offset {offset}"
        place = {"tier": tier, "arm_pose": index, "head_offset": offset}
        aim = (math.radians(pan_deg), math.radians(tilt_deg))
        with self.lock:
            held = self.held_head
        head = None if held is None else plan.checker.head_span(held, aim)
        gripper = max(joints.get(GRIPPER_JOINT, self.seq.grip_rad), self.seq.grip_rad)
        entries = plan.checker.check_head_move(joints, gripper, head)
        self.log(
            "head_move_check",
            "refused" if entries else "ok",
            logging.WARNING if entries else logging.INFO,
            f"{len(entries)} violated entries" if entries else None,
            joints=joints,
            gripper_rad=gripper,
            held_head_rad=held,
            pan_deg=pan_deg,
            tilt_deg=tilt_deg,
            head_model="envelope" if head is None else "span",
            head_region=None if head is None else {"pan_rad": head.pan_rad, "tilt_rad": head.tilt_rad},
            entries=entries,
            **place,
        )
        if entries:
            detail = f"{where}: pan {pan_deg:.2f} deg, tilt {tilt_deg:.2f} deg: {'; '.join(entries)}"
            self.count_skipped(REASON_HEAD_CLEARANCE, detail)
            return
        with self.lock:
            self.held_head = None  # unknown until the move converges (a tool error leaves it unknown)
        result = self.call(self.robot.set_head_angles(pan_deg, tilt_deg), f"set_head_angles at {where}")
        status, message = self.log_motion("set_head_angles", result, pan_deg=pan_deg, tilt_deg=tilt_deg, **place)
        if status != CONVERGED:
            self.count_skipped(f"head move {status}", f"{where}: {message}")
            return
        measured = (result.data or {}).get("measured")
        if isinstance(measured, dict) and all(
            isinstance(measured.get(k), int | float) for k in ("pan_deg", "tilt_deg")
        ):
            aim = (math.radians(measured["pan_deg"]), math.radians(measured["tilt_deg"]))
        with self.lock:
            self.held_head = aim
        self.sleep(self.seq.settle_s)
        self.check_alive()
        began = self.clock()
        outcome: StillOutcome = self.capture.capture_still(index, tier, offset)
        stored = outcome.stored
        self.log(
            "still",
            "stored" if stored else "skipped",
            logging.INFO if stored else logging.WARNING,
            None if stored else outcome.reason,
            pose_id=outcome.pose_id,
            waited_s=self.clock() - began,
            detail=outcome.detail,
            **place,
        )
        if stored:
            with self.lock:
                self.captured += 1
            return
        if outcome.reason == REASON_STOPPED:
            raise StepFailed(f"the still capture session was stopped at {where}")
        self.count_skipped(outcome.reason, where)

    def count_skipped(self, reason: str, detail: str, view: View | None = None) -> None:
        """Count a skipped view by its reason and log it.

        Args:
            reason (str): The skip reason (the count key).
            detail (str): Where, and the violated rules.
            view (View | None): The planned view (its head command is logged), None for a capture or head move skip.
        """
        with self.lock:
            self.skipped[reason] += 1
        head = {} if view is None else {"pan_deg": view.pan_deg, "tilt_deg": view.tilt_deg, "head_offset": view.offset}
        self.log("view", "skipped", logging.WARNING, reason, detail=detail, **head)
