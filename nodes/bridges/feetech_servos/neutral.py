"""Record a released group's hand-set pose and save it as each servo's EEPROM centre (no rclpy).

record(namespace): the group's neutral joints (position joints minus neutral_exclude_joints) must be released with a
verified torque 0. Their present_position is read (None retried) and kept as the capture; nothing is written.

save(namespace): refused without a capture, when the group is not released, or when a joint moved more than
NEUTRAL_MOVE_TOLERANCE_STEPS since the record (nothing written). Then every velocity joint of this bus is stopped
(zero goal_speed acknowledged): the save stalls this bridge's loop (~0.6 s per joint) and its velocity watchdog cannot
run meanwhile. Per joint, in config order (the sequence of scripts/calibrate_servos.py cmd_center): read the angle
limits, UnLockEprom, DefineMiddle, settle, LockEprom (always), verify present ~2048, restore the pre-centre limits if
DefineMiddle moved them, refresh the bridge command limits of EEPROM-limited joints. When every joint saved, torque is
enabled holding the present position (TorqueControl.enable). Any failure aborts the remaining joints and keeps torque
off on the whole group. A group with neutral_stale_marker gets the stale marker JSON for every joint whose centre
changed.

bridge.py runs both on the loop thread (the sole bus owner); tests drive them over an emulated bus.
"""

import json
import logging
import os
import tempfile
import time
from collections.abc import Callable
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from .command_mapping import STEP_CENTER
from .config import JointEntry, JointGroup
from .registers import get_register_entry_by_name, read_register, write_register
from .torque_control import PRESENT_REGISTER, TORQUE_OFF, TORQUE_REGISTER, TorqueControl, read_present_position

LOGGER = logging.getLogger(__name__)
NEUTRAL_MOVE_TOLERANCE_STEPS = 20
CENTER_TOLERANCE_STEPS = 3
CENTER_SETTLE_S = 0.5
CENTER_READ_ATTEMPTS = 10
CENTER_RETRY_SLEEP_S = 0.1
LIMIT_READ_ATTEMPTS = 3
LIMIT_VERIFY_ATTEMPTS = 3
LIMIT_SETTLE_S = 0.2
STEPS_PER_TURN = 4096
DEGREES_PER_TURN = 360.0
MIN_LIMIT_REGISTER = "min_angle_limit"
MAX_LIMIT_REGISTER = "max_angle_limit"
SPEED_REGISTER = "goal_speed"
MARKER_VERSION = 1
STALE_ITEMS = ("joint_offsets", "home")
NONE_TEXT = "none"


@dataclass
class NeutralResult:
    """Outcome of a record or save request.

    Attributes:
        success: True when the request did everything it was asked to.
        message: Human-readable summary; names every refused, failed or unattempted joint.
    """

    success: bool
    message: str


@dataclass
class CentreOutcome:
    """Outcome of centring one servo.

    Attributes:
        after_steps: Verified present_position after DefineMiddle; None when the centre was not verified.
        limits: Final (min, max) angle limit readback; None unless the joint fully saved.
        reason: Why the joint failed; None on success.
    """

    after_steps: int | None
    limits: tuple[int, int] | None
    reason: str | None


def utc_now_iso() -> str:
    """Return the current UTC time as ISO-8601 with seconds.

    Returns:
        str: e.g. "2026-10-20T12:00:00+00:00".
    """
    return datetime.now(UTC).isoformat(timespec="seconds")


def shift_text(name: str, steps: int) -> str:
    """Describe the neutral shift of one joint: present - 2048 in steps and degrees.

    Args:
        name (str): Joint name.
        steps (int): Recorded present_position steps.

    Returns:
        str: e.g. "elbow_flex +552 steps (+48.5 deg)".
    """
    delta = steps - STEP_CENTER
    return f"{name} {delta:+d} steps ({delta * DEGREES_PER_TURN / STEPS_PER_TURN:+.1f} deg)"


def merge_marker(
    existing: dict[str, Any] | None, group: str, saved_at: str, joints: dict[str, tuple[int, int]]
) -> dict[str, Any]:
    """Build the stale marker content, merging the joints of an existing marker.

    Args:
        existing (dict[str, Any] | None): Parsed existing marker; None or a malformed one starts afresh.
        group (str): Namespace of the saved group.
        saved_at (str): ISO-8601 UTC time of the save.
        joints (dict[str, tuple[int, int]]): joint name -> (before_steps, after_steps) of the joints whose centre changed.

    Returns:
        dict[str, Any]: {"version", "group", "saved_at", "joints", "stale"}; stale is reset to both STALE_ITEMS.
    """
    old_joints = existing.get("joints") if isinstance(existing, dict) else None
    merged = dict(old_joints) if isinstance(old_joints, dict) else {}
    for name, (before, after) in joints.items():
        merged[name] = {"before_steps": before, "after_steps": after}
    return {
        "version": MARKER_VERSION,
        "group": group,
        "saved_at": saved_at,
        "joints": merged,
        "stale": list(STALE_ITEMS),
    }


def write_stale_marker(path: Path, group: str, saved_at: str, joints: dict[str, tuple[int, int]]) -> None:
    """Write (merge) the stale marker atomically: temp file in the same directory, then os.replace.

    Missing parent directories are created. An existing marker that is not valid JSON is replaced.

    Args:
        path (Path): Marker path.
        group (str): Namespace of the saved group.
        saved_at (str): ISO-8601 UTC time of the save.
        joints (dict[str, tuple[int, int]]): joint name -> (before_steps, after_steps).

    Raises:
        OSError: When the directory, the temp file or the replace fails.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    existing = None
    if path.exists():
        try:
            existing = json.loads(path.read_text())
        except json.JSONDecodeError:
            LOGGER.warning("stale marker %s is not valid JSON; replacing it", path)
    content = json.dumps(merge_marker(existing, group, saved_at, joints), indent=2) + "\n"
    fd, temp_name = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.", suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_name, path)
    except BaseException:
        Path(temp_name).unlink(missing_ok=True)
        raise


def names_text(names: list[str]) -> str:
    """Join joint names for a message, or "none".

    Args:
        names (list[str]): Joint names.

    Returns:
        str: Comma-separated names or "none".
    """
    return ", ".join(names) if names else NONE_TEXT


class NeutralControl:
    """Record neutral and save it as the EEPROM centre per joint group, on the bridge's servo bus.

    Attributes:
        torque: The bus's torque control (gate, torque readings, register cache, enable).
        command_limits: servo_id -> (cmd_min, cmd_max) shared with BridgeCycle; refreshed after a save.
        sleep: Sleep function (seconds).
        now_iso: Returns the ISO-8601 UTC save time.
        log_info: Receives a line per record or save result.
        log_error: Receives a line per stale marker write failure.
        captures: namespace -> joint name -> recorded present steps (joints not saved yet).
        last_saves: namespace -> {"saved_at", "saved", "failed"} of the last save that reached the EEPROM step.
    """

    def __init__(
        self,
        torque: TorqueControl,
        command_limits: dict[int, tuple[int, int]],
        sleep: Callable[[float], Any] = time.sleep,
        now_iso: Callable[[], str] = utc_now_iso,
        log_info: Callable[[str], Any] = LOGGER.info,
        log_error: Callable[[str], Any] = LOGGER.error,
    ) -> None:
        """Create the neutral control for one bus.

        Args:
            torque (TorqueControl): Torque control of the bus.
            command_limits (dict[int, tuple[int, int]]): BridgeCycle command limits (updated in place).
            sleep (Callable[[float], Any]): Sleep function, seconds.
            now_iso (Callable[[], str]): ISO-8601 UTC clock for saved_at.
            log_info (Callable[[str], Any]): Info logger.
            log_error (Callable[[str], Any]): Error logger.
        """
        self.torque = torque
        self.command_limits = command_limits
        self.sleep = sleep
        self.now_iso = now_iso
        self.log_info = log_info
        self.log_error = log_error
        self.captures: dict[str, dict[str, int]] = {}
        self.last_saves: dict[str, dict[str, Any]] = {}
        self.present_entry = get_register_entry_by_name(PRESENT_REGISTER)
        self.min_entry = get_register_entry_by_name(MIN_LIMIT_REGISTER)
        self.max_entry = get_register_entry_by_name(MAX_LIMIT_REGISTER)

    def status(self, namespace: str) -> dict[str, Any]:
        """Neutral fields of a group's torque_state payload (fresh copies).

        Args:
            namespace (str): Group namespace.

        Returns:
            dict[str, Any]: {"neutral_capture": {name: {"steps", "delta_steps"}} | None, "last_save": dict | None}.
        """
        capture = self.captures.get(namespace)
        last_save = self.last_saves.get(namespace)
        return {
            "neutral_capture": (
                {name: {"steps": steps, "delta_steps": steps - STEP_CENTER} for name, steps in capture.items()}
                if capture
                else None
            ),
            "last_save": (
                {
                    "saved_at": last_save["saved_at"],
                    "saved": list(last_save["saved"]),
                    "failed": list(last_save["failed"]),
                }
                if last_save is not None
                else None
            ),
        }

    def record(self, namespace: str) -> NeutralResult:
        """Capture the present position of a released group's neutral joints; no bus write.

        Args:
            namespace (str): Group namespace.

        Returns:
            NeutralResult: success with the per-joint shift (steps and degrees), or the refusal.
        """
        group, refusal = self.checked_group(namespace)
        if group is None:
            return self.logged(NeutralResult(False, f"/{namespace}: {refusal}"))
        joints = group.neutral_joints
        present, unread = self.read_presents(joints)
        if unread:
            return self.logged(
                NeutralResult(False, f"/{namespace}: present_position read failed: {names_text(unread)}")
            )
        self.captures[namespace] = present
        shifts = ", ".join(shift_text(name, steps) for name, steps in present.items())
        return self.logged(NeutralResult(True, f"/{namespace}: neutral recorded: {shifts}"))

    def save(self, namespace: str) -> NeutralResult:
        """Write the recorded neutral into each joint's EEPROM centre, then enable torque holding present.

        Args:
            namespace (str): Group namespace.

        Returns:
            NeutralResult: success when every joint saved, torque is back on and the stale marker (if any) is written;
                otherwise the message names the saved, failed and unattempted joints.
        """
        group, refusal = self.checked_group(namespace)
        if group is None:
            return self.logged(NeutralResult(False, f"/{namespace}: {refusal}"))
        capture = self.captures.get(namespace)
        if not capture:
            return self.logged(NeutralResult(False, f"/{namespace}: record neutral first"))
        joints = [j for j in group.neutral_joints if j.name in capture]
        present, unread = self.read_presents(joints)
        if unread:
            message = f"/{namespace}: present_position read failed: {names_text(unread)}; nothing written"
            return self.logged(NeutralResult(False, message))
        moved = [j.name for j in joints if abs(present[j.name] - capture[j.name]) > NEUTRAL_MOVE_TOLERANCE_STEPS]
        if moved:
            return self.logged(
                NeutralResult(False, f"/{namespace}: moved since record: {', '.join(moved)}; record again")
            )
        unstopped = self.stop_wheels()
        if unstopped:
            message = f"/{namespace}: wheels not stopped: {names_text(unstopped)}; nothing written"
            return self.logged(NeutralResult(False, message))

        saved: list[str] = []
        centred: dict[str, tuple[int, int]] = {}
        failure: tuple[str, str] | None = None
        not_attempted: list[str] = []
        for index, joint in enumerate(joints):
            outcome = self.centre_joint(joint)
            if outcome.after_steps is not None:
                centred[joint.name] = (present[joint.name], outcome.after_steps)
            if outcome.reason is not None:
                failure = (joint.name, outcome.reason)
                not_attempted = [j.name for j in joints[index + 1 :]]
                break
            saved.append(joint.name)
            capture.pop(joint.name)
            if joint.command_min_steps is None or joint.command_max_steps is None:
                self.command_limits[joint.id] = outcome.limits

        saved_at = self.now_iso()
        self.last_saves[namespace] = {"saved_at": saved_at, "saved": saved, "failed": [failure[0]] if failure else []}
        marker_error = self.write_marker(group, saved_at, centred)
        if failure is not None:
            self.keep_released(group)
            message = (
                f"/{namespace}: neutral save failed, torque stays off; saved: {names_text(saved)}; "
                f"failed: {failure[0]} ({failure[1]}); not attempted: {names_text(not_attempted)}"
            )
            if marker_error is not None:
                message += f"; stale marker not written: {marker_error}"
            return self.logged(NeutralResult(False, message))

        del self.captures[namespace]
        enabled = self.torque.enable(namespace)
        if marker_error is not None:
            message = f"/{namespace}: neutral saved, stale marker not written: {marker_error}"
            if not enabled.success:
                message += f"; {enabled.message}"
            return self.logged(NeutralResult(False, message))
        message = f"/{namespace}: neutral saved for {names_text(saved)}; {enabled.message}"
        return self.logged(NeutralResult(enabled.success, message))

    def checked_group(self, namespace: str) -> tuple[JointGroup | None, str]:
        """Return the group for a record/save request, or None and the refusal reason.

        Args:
            namespace (str): Group namespace.

        Returns:
            tuple[JointGroup | None, str]: (group, "") or (None, reason).
        """
        group, refusal = self.torque.checked_group(namespace)
        if group is None:
            return None, refusal
        if not group.neutral_enabled:
            return None, "neutral not enabled for this group"
        if self.min_entry is None or self.max_entry is None:
            return None, "register map lacks min_angle_limit / max_angle_limit"
        not_released = [
            j.name
            for j in group.neutral_joints
            if j.id not in self.torque.released_ids or self.torque.joint_torque.get(j.id) != TORQUE_OFF
        ]
        if not_released:
            return None, f"release torque first ({names_text(not_released)})"
        return group, ""

    def read_presents(self, joints: list[JointEntry]) -> tuple[dict[str, int], list[str]]:
        """Read present_position of each joint (None retried).

        Args:
            joints (list[JointEntry]): Joints to read.

        Returns:
            tuple[dict[str, int], list[str]]: (joint name -> steps, names whose every read failed).
        """
        present: dict[str, int] = {}
        unread: list[str] = []
        for joint in joints:
            value = read_present_position(
                lambda sid=joint.id: read_register(self.torque.servo, sid, self.present_entry)
            )
            if value is None:
                unread.append(joint.name)
            else:
                present[joint.name] = value
        return present, unread

    def stop_wheels(self) -> list[str]:
        """Stop every velocity joint of this bus with an acknowledged zero goal_speed (cache bypassed).

        Returns:
            list[str]: Names of wheels whose stop was not acknowledged.
        """
        wheels = self.torque.cycle.velocity_joints
        for wheel in wheels:
            self.torque.last_written.setdefault(wheel.id, {}).pop(SPEED_REGISTER, None)
        self.torque.cycle.stop(wheels)
        return [w.name for w in wheels if SPEED_REGISTER not in self.torque.last_written[w.id]]

    def read_limits(self, servo_id: int) -> tuple[int, int] | None:
        """Read (min_angle_limit, max_angle_limit), each retried.

        Args:
            servo_id (int): Servo ID.

        Returns:
            tuple[int, int] | None: The limits, or None when a register never read.
        """
        servo = self.torque.servo
        low = read_present_position(lambda: read_register(servo, servo_id, self.min_entry), LIMIT_READ_ATTEMPTS)
        high = read_present_position(lambda: read_register(servo, servo_id, self.max_entry), LIMIT_READ_ATTEMPTS)
        if low is None or high is None:
            return None
        return low, high

    def read_limits_settled(
        self, servo_id: int, wanted: tuple[int, int]
    ) -> tuple[tuple[int, int] | None, tuple[int, int] | None]:
        """Read the limits up to LIMIT_VERIFY_ATTEMPTS times, LIMIT_SETTLE_S apart, until they equal wanted.

        A single None or mismatching read does not count: only the last attempt's result is reported.

        Args:
            servo_id (int): Servo ID.
            wanted (tuple[int, int]): Limits that end the loop when read.

        Returns:
            tuple[tuple[int, int] | None, tuple[int, int] | None]: The last non-None limits read (None when every
                attempt failed) and the first non-None limits read (None when none).
        """
        last: tuple[int, int] | None = None
        first: tuple[int, int] | None = None
        for attempt in range(LIMIT_VERIFY_ATTEMPTS):
            self.sleep(LIMIT_SETTLE_S)
            reading = self.read_limits(servo_id)
            if reading is not None:
                first = first or reading
                last = reading
                if reading == wanted:
                    break
        return last, first

    def centre_joint(self, joint: JointEntry) -> CentreOutcome:
        """Store one servo's present position as its EEPROM centre (2048), keeping its angle limits.

        Args:
            joint (JointEntry): Released position joint.

        Returns:
            CentreOutcome: Verified after-steps (when the centre changed), final limits (when saved), failure reason.
        """
        servo = self.torque.servo
        sid = joint.id
        before_limits = self.read_limits(sid)
        if before_limits is None:
            return CentreOutcome(None, None, "angle limit read failed, centre not changed")
        cache = self.torque.last_written.setdefault(sid, {})
        cache.pop(TORQUE_REGISTER, None)  # DefineMiddle writes torque_enable = 128 behind the cache
        servo.UnLockEprom(sid)
        try:
            try:
                defined = servo.DefineMiddle(sid)
            except (IndexError, OSError, ValueError) as exc:
                return CentreOutcome(None, None, f"DefineMiddle raised {type(exc).__name__}: {exc}")
            if not defined:
                return CentreOutcome(None, None, "DefineMiddle failed")
            self.sleep(CENTER_SETTLE_S)
        finally:
            servo.LockEprom(sid)

        after = self.verify_centre(sid)
        if after is None or abs(after - STEP_CENTER) > CENTER_TOLERANCE_STEPS:
            reading = "read failed" if after is None else f"reads {after}"
            reason = f"centre not verified: {reading}, expected {STEP_CENTER} +- {CENTER_TOLERANCE_STEPS}"
            return CentreOutcome(None, None, reason)

        after_limits, first_observed = self.read_limits_settled(sid, before_limits)
        if after_limits is None:
            return CentreOutcome(after, None, "centre changed, angle limits not restored: limit readback failed")
        if after_limits != before_limits:
            servo.UnLockEprom(sid)
            try:
                for entry, value in ((self.min_entry, before_limits[0]), (self.max_entry, before_limits[1])):
                    cache.pop(entry.name, None)
                    if not write_register(servo, sid, entry, value, cache):
                        return CentreOutcome(
                            after, None, f"centre changed, angle limits not restored: {entry.name} write failed"
                        )
            finally:
                servo.LockEprom(sid)
            after_limits, _ = self.read_limits_settled(sid, before_limits)
            if after_limits != before_limits:
                reason = (
                    f"centre changed, angle limits not restored: read back {after_limits}, wanted {before_limits}"
                    f" (first read after centre {first_observed})"
                )
                return CentreOutcome(after, None, reason)
        return CentreOutcome(after, after_limits, None)

    def verify_centre(self, servo_id: int) -> int | None:
        """Read present_position until it is within CENTER_TOLERANCE_STEPS of 2048 (None retried).

        Args:
            servo_id (int): Servo ID.

        Returns:
            int | None: The last successful reading (in tolerance when verified), or None when every read failed.
        """
        last: int | None = None
        for attempt in range(CENTER_READ_ATTEMPTS):
            reading = read_register(self.torque.servo, servo_id, self.present_entry)
            if reading is not None:
                last = reading
                if abs(reading - STEP_CENTER) <= CENTER_TOLERANCE_STEPS:
                    return reading
            if attempt < CENTER_READ_ATTEMPTS - 1:
                self.sleep(CENTER_RETRY_SLEEP_S)
        return last

    def keep_released(self, group: JointGroup) -> None:
        """After a failed save: keep the group's position joints gated and re-assert torque 0 (verified).

        Args:
            group (JointGroup): The group whose save failed.
        """
        position_ids = [j.id for j in group.joints if j.mode == "position"]
        self.torque.cycle.release(position_ids)
        failed = self.torque.set_torque(position_ids, TORQUE_OFF, {})
        if failed:
            self.log_error(f"/{group.namespace}: torque off not verified after a failed neutral save: {failed}")

    def write_marker(self, group: JointGroup, saved_at: str, centred: dict[str, tuple[int, int]]) -> str | None:
        """Write the group's stale marker for the joints whose centre changed.

        Args:
            group (JointGroup): The saved group.
            saved_at (str): ISO-8601 UTC save time.
            centred (dict[str, tuple[int, int]]): joint name -> (before_steps, after_steps).

        Returns:
            str | None: The error text when the marker could not be written; None when written or not needed.
        """
        if group.neutral_stale_marker is None or not centred:
            return None
        try:
            write_stale_marker(Path(group.neutral_stale_marker), group.namespace, saved_at, centred)
        except OSError as exc:
            error = f"{type(exc).__name__}: {exc}"
            self.log_error(f"/{group.namespace}: neutral saved, stale marker not written: {error}")
            return error
        return None

    def logged(self, result: NeutralResult) -> NeutralResult:
        """Log a result (info) and return it.

        Args:
            result (NeutralResult): The result.

        Returns:
            NeutralResult: The same result.
        """
        self.log_info(result.message)
        return result
