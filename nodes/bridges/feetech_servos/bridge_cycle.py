"""Per-cycle bridge logic without rclpy: command handling, velocity watchdog, write decisions, callback draining.

Command callbacks only record the latest finite target per joint (handle_command); the loop writes each pending
target once per iteration (write_pending_commands), so targets superseded within one cycle never reach the bus.

bridge.py wires this to ROS (subscriptions, executor, publishers); tests drive it with a fake servo and a
simulated message queue.
"""

import time
from collections.abc import Callable, Collection, Sequence
from dataclasses import dataclass
from typing import Any

from .command_mapping import is_finite_command, map_position_to_steps, position_to_raw_steps, velocity_to_speed_register
from .config import JointEntry, JointGroup
from .metrics import COMMANDS, CYCLE_DURATION
from .registers import RegisterEntry, write_register
from .velocity_watchdog import apply_velocity_command, expired_velocity_joints

MIN_STEPS = 0
MAX_STEPS = 4095
MAX_CALLBACKS_PER_CYCLE = 64


def drain_callbacks(spin_ready: Callable[[], bool], max_callbacks: int = MAX_CALLBACKS_PER_CYCLE) -> int:
    """Process all pending ROS callbacks for this loop iteration (bounded).

    Handling only one callback per iteration starves drive commands: the swerve controller publishes a steer
    and a drive message back to back, faster than the loop runs, so the KEEP_LAST queue is always full and its
    oldest entry (the one processed) is always a steer message; the watchdog then stops the wheels.

    Args:
        spin_ready: Processes at most one ready callback without blocking; returns True if one ran.
        max_callbacks: Upper bound on callbacks processed per call.

    Returns:
        int: Number of callbacks processed.
    """
    processed = 0
    while processed < max_callbacks and spin_ready():
        processed += 1
    return processed


def remaining_sleep_s(period_s: float, elapsed_s: float) -> float:
    """Return how long to sleep so one loop iteration lasts period_s.

    Args:
        period_s: Target loop period in seconds.
        elapsed_s: Time already spent in this iteration, seconds.

    Returns:
        float: Seconds to sleep (never negative).
    """
    return max(0.0, period_s - elapsed_s)


def end_cycle(period_s: float, elapsed_s: float) -> float:
    """Record the work time of one loop iteration and return how long to sleep.

    Args:
        period_s: Target loop period in seconds.
        elapsed_s: Work time of this iteration, seconds.

    Returns:
        float: Seconds to sleep (never negative).
    """
    CYCLE_DURATION.observe(elapsed_s)
    return remaining_sleep_s(period_s, elapsed_s)


@dataclass
class PendingTarget:
    """Latest finite command for one joint, not yet written to the bus.

    Attributes:
        joint: Joint the target is for.
        value: goal_position steps (position joints) or velocity in rad/s (velocity joints).
        received_at: Monotonic time the command was received, seconds (feeds the velocity watchdog).
    """

    joint: JointEntry
    value: float
    received_at: float


class BridgeCycle:
    """Command handling and velocity watchdog for one serial bus, independent of rclpy.

    Attributes:
        last_velocity_commands: servo_id -> (receive time of last drive command, applied rad/s) for the watchdog.
        pending_positions: servo_id -> latest unwritten position target (value in goal_position steps).
        pending_velocities: servo_id -> latest unwritten velocity target (value in rad/s).
    """

    def __init__(
        self,
        servo: Any,
        goal_entry: RegisterEntry | None,
        speed_goal_entry: RegisterEntry | None,
        velocity_joints: Sequence[JointEntry],
        command_limits: dict[int, tuple[int, int]],
        last_written: dict[int, dict[str, int]],
        velocity_command_timeout_s: float,
        clock: Callable[[], float] = time.monotonic,
        direct_command_sources: Collection[str] = (),
    ) -> None:
        """Create the per-cycle logic.

        Args:
            servo: ST3215-compatible servo bus (write1ByteTxRx / write2ByteTxRx), or None when no bus is open.
            goal_entry: goal_position register entry.
            speed_goal_entry: goal_speed register entry.
            velocity_joints: Joints running in wheel mode that accept velocity commands.
            command_limits: servo_id -> (cmd_min, cmd_max) step range for position joints.
            last_written: servo_id -> register cache shared with write_register (updated in place).
            velocity_command_timeout_s: A wheel is stopped when no drive command arrived for this long.
            clock: Monotonic time source in seconds.
            direct_command_sources: joint_commands header.frame_id values whose positions are follower joint
                radians: the source range mapping (source_min/max_steps) is skipped for them.
        """
        self.servo = servo
        self.goal_entry = goal_entry
        self.speed_goal_entry = speed_goal_entry
        self.velocity_joints = list(velocity_joints)
        self.velocity_ids = {j.id for j in self.velocity_joints}
        self.joint_by_id = {j.id: j for j in self.velocity_joints}
        self.command_limits = command_limits
        self.last_written = last_written
        self.velocity_command_timeout_s = velocity_command_timeout_s
        self.clock = clock
        self.direct_command_sources = frozenset(direct_command_sources)
        self.last_velocity_commands: dict[int, tuple[float, float]] = {}
        self.pending_positions: dict[int, PendingTarget] = {}
        self.pending_velocities: dict[int, PendingTarget] = {}

    def write_velocity(self, joint: JointEntry, velocity: float, received_at: float | None = None) -> bool:
        """Write goal_speed for a wheel joint and record it for the watchdog.

        Args:
            joint: Velocity-mode joint.
            velocity: Commanded velocity, rad/s (finite).
            received_at: Receive time of the drive command from joint_commands (feeds the watchdog even if the
                write fails); None for a watchdog / shutdown stop.

        Returns:
            bool: True if the write succeeded (or was skipped as unchanged).
        """
        raw = velocity_to_speed_register(velocity, joint.max_velocity_rad_s, inverted=joint.inverted)
        cache = self.last_written.setdefault(joint.id, {})
        return apply_velocity_command(
            lambda: write_register(self.servo, joint.id, self.speed_goal_entry, raw, cache),
            joint.id,
            velocity,
            received_at if received_at is not None else self.clock(),
            self.last_velocity_commands,
            received=received_at is not None,
        )

    def position_target_steps(self, joint: JointEntry, position: float, direct: bool = False) -> int:
        """Map a position command (radians) to goal_position steps within the joint's command range.

        Args:
            joint: Position-mode joint.
            position: Commanded position, centred radians.
            direct: True when the command already is a follower joint position (a direct command source):
                the source range mapping is skipped.

        Returns:
            int: Target goal_position steps.
        """
        cmd_min, cmd_max = self.command_limits.get(joint.id, (MIN_STEPS, MAX_STEPS))
        # Pass-through for direct sources and for joints without a source range (backward-compatible default).
        if direct or (joint.source_min_steps is None and joint.source_max_steps is None):
            return max(cmd_min, min(cmd_max, position_to_raw_steps(position, joint.inverted)))
        source_min = joint.source_min_steps if joint.source_min_steps is not None else MIN_STEPS
        source_max = joint.source_max_steps if joint.source_max_steps is not None else MAX_STEPS
        return map_position_to_steps(
            position,
            source_min,
            source_max,
            cmd_min,
            cmd_max,
            source_inverted=joint.source_inverted,
        )

    def handle_command(
        self,
        group: JointGroup,
        names: Sequence[str],
        positions: Sequence[float],
        velocities: Sequence[float],
        source: str = "",
    ) -> None:
        """Record one joint_commands message as the latest target per joint; never writes to the bus.

        Accepts separate steer-only / drive-only messages, position-only arm messages, and the combined format
        (all joints in one message, position NaN for velocity-mode joints, velocity NaN for position-mode
        joints). Non-finite entries are ignored and never replace a pending target. A newer finite target for a
        joint replaces the pending one, so superseded targets are never written. write_pending_commands writes them.

        Args:
            group: Joint group the message was received for.
            names: JointState.name.
            positions: JointState.position (drives position-mode joints).
            velocities: JointState.velocity (drives velocity-mode joints, rad/s).
            source: JointState.header.frame_id (command source tag set by filter_node, e.g. leader / autonomy).
        """
        if self.servo is None or self.goal_entry is None:
            return
        now = self.clock()
        direct = source in self.direct_command_sources
        for i, name in enumerate(names):
            joint = group.joint_entry_by_name(name)
            if joint is None:
                continue
            if joint.mode == "velocity":
                if joint.id in self.velocity_ids and i < len(velocities) and is_finite_command(velocities[i]):
                    self.pending_velocities[joint.id] = PendingTarget(joint, float(velocities[i]), now)
                continue
            if i >= len(positions) or not is_finite_command(positions[i]):
                continue
            target = self.position_target_steps(joint, float(positions[i]), direct)
            self.pending_positions[joint.id] = PendingTarget(joint, target, now)

    def write_pending_commands(self) -> int:
        """Write every pending target once (call once per loop iteration, after draining callbacks).

        write_register skips a value equal to the last written one, so only changed targets reach the bus. Pending
        targets are consumed whether or not the write succeeds (a failed write is not retried with a stale value;
        the next message brings a fresh one). Drive targets feed the watchdog with their receive time.

        Returns:
            int: Number of pending targets processed.
        """
        positions, self.pending_positions = self.pending_positions, {}
        velocities, self.pending_velocities = self.pending_velocities, {}
        if self.servo is None or self.goal_entry is None:
            return 0
        for target in positions.values():
            write_register(
                self.servo,
                target.joint.id,
                self.goal_entry,
                int(target.value),
                self.last_written.setdefault(target.joint.id, {}),
            )
        for target in velocities.values():
            self.write_velocity(target.joint, target.value, received_at=target.received_at)
        processed = len(positions) + len(velocities)
        COMMANDS.inc(processed)
        return processed

    def stop_expired(self) -> list[int]:
        """Velocity watchdog: stop wheels whose drive commands went stale.

        Returns:
            list[int]: Servo IDs a stop was issued for.
        """
        expired = expired_velocity_joints(self.last_velocity_commands, self.clock(), self.velocity_command_timeout_s)
        self.stop([self.joint_by_id[sid] for sid in expired if sid in self.joint_by_id])
        return expired

    def stop(self, joints: Sequence[JointEntry]) -> None:
        """Command zero velocity on the given wheel joints.

        Any pending (unwritten) velocity target for these joints is dropped so it cannot restart them.

        Args:
            joints: Velocity-mode joints to stop.
        """
        for joint in joints:
            self.pending_velocities.pop(joint.id, None)
            self.write_velocity(joint, 0.0)
