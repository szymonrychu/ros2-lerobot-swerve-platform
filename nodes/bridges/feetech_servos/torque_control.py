"""Per-group torque release and hold-present enable on the bridge's servo bus (no rclpy).

release(namespace) gates the group's position joints in BridgeCycle first (pending targets dropped, later commands
ignored, no goal refresh), then writes torque_enable = 0 to each one bypassing the register cache and verifies it by
readback. Velocity (wheel) joints are never released: they are stopped and their commands are dropped while the group
is released; their torque stays on.

enable(namespace) per position joint of the group: read present_position, write goal_position = present (cache
bypassed), then torque_enable = 1 (cache bypassed) verified by readback. A joint whose read, goal write or verify fails
stays gated and is named in the message, so no goal ever reaches a servo whose torque is not verified on.

bridge.py runs both on the loop thread (the sole bus owner); tests drive them over an emulated bus.
"""

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any

from .bridge_cycle import BridgeCycle
from .config import JointEntry, JointGroup
from .registers import get_register_entry_by_name, read_register, write_register
from .startup_torque import set_startup_torque_state

TORQUE_WRITE_ATTEMPTS = 8
TORQUE_VERIFY_SLEEP_S = 0.02
PRESENT_READ_ATTEMPTS = 3
TORQUE_REGISTER = "torque_enable"
GOAL_REGISTER = "goal_position"
PRESENT_REGISTER = "present_position"
TORQUE_OFF = 0
TORQUE_ON = 1


@dataclass
class TorqueResult:
    """Outcome of a release or enable request.

    Attributes:
        success: True when every joint of the group reached the wanted torque state (verified).
        message: Human-readable summary; names every failed joint and why.
        joints: joint name -> torque_enable value read back during this request (joints without a readback omitted).
    """

    success: bool
    message: str
    joints: dict[str, int] = field(default_factory=dict)


def read_present_position(read_once: Callable[[], int | None], attempts: int = PRESENT_READ_ATTEMPTS) -> int | None:
    """Read present_position, retrying a failed (None) read.

    Args:
        read_once (Callable[[], int | None]): One register read; None on failure.
        attempts (int): Maximum reads.

    Returns:
        int | None: Present position steps, or None when every read failed.
    """
    for _ in range(max(1, attempts)):
        value = read_once()
        if value is not None:
            return value
    return None


class TorqueControl:
    """Release and re-enable servo torque per joint group, gating the group's commands in BridgeCycle.

    Attributes:
        servo: ST3215-compatible bus, or None when no bus is open.
        groups: namespace -> joint group served by this bridge.
        last_written: servo_id -> register cache shared with write_register and BridgeCycle.
        cycle: The bridge's per-cycle logic holding the command gate.
        joint_torque: servo_id -> latest torque_enable value read back (verify readback or register dump); a servo
            with no reading is absent (never defaulted).
        attempts: Torque write + verify attempts per servo.
        verify_sleep_s: Delay between torque write attempts, seconds.
    """

    def __init__(
        self,
        servo: Any,
        groups: Sequence[JointGroup],
        last_written: dict[int, dict[str, int]],
        cycle: BridgeCycle,
        attempts: int = TORQUE_WRITE_ATTEMPTS,
        verify_sleep_s: float = TORQUE_VERIFY_SLEEP_S,
    ) -> None:
        """Create the torque control for one bus.

        Args:
            servo (Any): ST3215-compatible bus, or None when no bus is open.
            groups (Sequence[JointGroup]): Every joint group on this bus.
            last_written (dict[int, dict[str, int]]): Register cache shared with write_register (updated in place).
            cycle (BridgeCycle): The bridge's BridgeCycle (command gate, wheel stop).
            attempts (int): Torque write + verify attempts per servo.
            verify_sleep_s (float): Delay between torque write attempts, seconds.
        """
        self.servo = servo
        self.groups = {g.namespace: g for g in groups}
        self.last_written = last_written
        self.cycle = cycle
        self.attempts = attempts
        self.verify_sleep_s = verify_sleep_s
        self.joint_torque: dict[int, int] = {}
        self.position_ids = {j.id for g in groups for j in g.joints if j.mode == "position"}
        self.torque_entry = get_register_entry_by_name(TORQUE_REGISTER)
        self.goal_entry = get_register_entry_by_name(GOAL_REGISTER)
        self.present_entry = get_register_entry_by_name(PRESENT_REGISTER)

    @property
    def released_ids(self) -> set[int]:
        """Position-mode servo IDs that are released (gated, torque off or not verified on).

        Wheels of a released group are gated in BridgeCycle too but keep torque, so they are not listed.

        Returns:
            set[int]: Released servo IDs.
        """
        return self.cycle.released_ids & self.position_ids

    def note_register_dump(self, servo_id: int, registers: dict[str, int]) -> None:
        """Record the torque_enable value of a register dump (a servo may drop torque by itself).

        Args:
            servo_id (int): Servo the dump is for.
            registers (dict[str, int]): register name -> raw value; without torque_enable nothing is recorded.
        """
        if TORQUE_REGISTER in registers:
            self.joint_torque[servo_id] = registers[TORQUE_REGISTER]

    def release(self, namespace: str) -> TorqueResult:
        """Release torque of a group's position joints; stop and gate its wheels (their torque stays on).

        The joints are gated first, so no goal reaches them from now on; then torque_enable = 0 is written to each
        one bypassing the register cache and verified by readback. Releasing a released group writes torque 0 again.

        Args:
            namespace (str): Group namespace (e.g. "follower", "swerve_drive", "head").

        Returns:
            TorqueResult: success when every position joint reads back torque 0; the message names the others.
        """
        group, refusal = self.checked_group(namespace)
        if group is None:
            return TorqueResult(False, refusal)
        position_joints = [j for j in group.joints if j.mode == "position"]
        wheels = [j for j in group.joints if j.mode == "velocity"]
        self.cycle.release([j.id for j in position_joints + wheels])
        self.cycle.stop(wheels)
        readings: dict[int, int] = {}
        failed = set(self.set_torque([j.id for j in position_joints], TORQUE_OFF, readings))
        joints = {j.name: readings[j.id] for j in position_joints if j.id in readings}
        if failed:
            names = ", ".join(j.name for j in position_joints if j.id in failed)
            return TorqueResult(False, f"/{namespace}: torque off not verified for {names}", joints)
        return TorqueResult(True, f"/{namespace}: torque off for {len(position_joints)} joints", joints)

    def enable(self, namespace: str) -> TorqueResult:
        """Enable torque of a group's position joints holding their present position, then lift the gate.

        Per joint: present_position read (retried), goal_position = present (cache bypassed, so the cache ends
        holding present), torque_enable = 1 (cache bypassed, verified). The goal is always written before the
        torque. A joint failing any step stays gated (released) and is named in the message; the others are
        un-gated. The group's wheels accept commands again once no position joint of the group is released.

        Args:
            namespace (str): Group namespace.

        Returns:
            TorqueResult: success when every position joint holds its present position with torque 1 verified.
        """
        group, refusal = self.checked_group(namespace)
        if group is None:
            return TorqueResult(False, refusal)
        position_joints = [j for j in group.joints if j.mode == "position"]
        # Gate every joint for the duration: one that fails stays gated, also if it was not released before.
        self.cycle.release(j.id for j in position_joints)
        readings: dict[int, int] = {}
        failures: list[str] = []
        enabled: list[int] = []
        for joint in position_joints:
            reason = self.hold_and_enable(joint, readings)
            if reason is None:
                enabled.append(joint.id)
            else:
                failures.append(f"{joint.name} ({reason})")
        self.cycle.unrelease(enabled)
        if not self.cycle.released_ids & {j.id for j in position_joints}:
            self.cycle.unrelease(j.id for j in group.joints if j.mode == "velocity")
        joints = {j.name: readings[j.id] for j in position_joints if j.id in readings}
        if failures:
            return TorqueResult(False, f"/{namespace}: still released: {', '.join(failures)}", joints)
        return TorqueResult(True, f"/{namespace}: torque on for {len(position_joints)} joints", joints)

    def checked_group(self, namespace: str) -> tuple[JointGroup | None, str]:
        """Return the group for a request, or None and the refusal reason.

        Args:
            namespace (str): Group namespace.

        Returns:
            tuple[JointGroup | None, str]: (group, "") or (None, reason).
        """
        if self.servo is None:
            return None, "no servo bus"
        group = self.groups.get(namespace)
        if group is None:
            return None, f"unknown group {namespace!r}"
        if self.torque_entry is None or self.goal_entry is None or self.present_entry is None:
            return None, "register map lacks torque_enable / goal_position / present_position"
        return group, ""

    def hold_and_enable(self, joint: JointEntry, readings: dict[int, int]) -> str | None:
        """Write goal_position = present, then torque_enable = 1 verified, for one joint.

        Args:
            joint (JointEntry): Position-mode joint (gated by the caller).
            readings (dict[int, int]): servo_id -> torque readback, updated in place.

        Returns:
            str | None: None on success, else why the joint stays released.
        """
        sid = joint.id
        present = read_present_position(lambda: read_register(self.servo, sid, self.present_entry))
        if present is None:
            return "present_position read failed"
        cache = self.last_written.setdefault(sid, {})
        cache.pop(GOAL_REGISTER, None)  # a cached equal value would skip the write
        if not write_register(self.servo, sid, self.goal_entry, present, cache):
            return "goal_position write failed"
        if self.set_torque([sid], TORQUE_ON, readings):
            return "torque on not verified"
        return None

    def set_torque(self, servo_ids: list[int], value: int, readings: dict[int, int]) -> list[int]:
        """Write torque_enable bypassing the register cache on every attempt, and verify it by readback.

        Args:
            servo_ids (list[int]): Servos to write.
            value (int): TORQUE_OFF or TORQUE_ON.
            readings (dict[int, int]): servo_id -> torque readback, updated in place (also joint_torque).

        Returns:
            list[int]: Servo IDs whose readback did not match value.
        """

        def write_once(sid: int, wanted: int) -> bool:
            cache = self.last_written.setdefault(sid, {})
            cache.pop(TORQUE_REGISTER, None)  # the servo may have changed it by itself: never trust the cache
            return write_register(self.servo, sid, self.torque_entry, wanted, cache)

        def read_once(sid: int) -> int | None:
            readback = read_register(self.servo, sid, self.torque_entry)
            if readback is not None:
                readings[sid] = readback
                self.joint_torque[sid] = readback
            return readback

        return set_startup_torque_state(
            joint_ids=servo_ids,
            torque_value=value,
            write_once=write_once,
            read_once=read_once,
            attempts=self.attempts,
            verify_sleep_s=self.verify_sleep_s,
        )
