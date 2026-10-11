"""Tilt guard: base roll/pitch from the IMU orientation, OK/WARNING/ERROR levels, /tilt_guard/state JSON.

Pure logic (no rclpy imports, unit-testable). The node feeds every /imu/data orientation into TiltGuard and
publishes state_dict() on /tilt_guard/state. Levels:

- WARNING: |roll| > roll_warning_deg or |pitch| > pitch_warning_deg. Consumers (agent, web UI) stop automatic
  driving and tell the operator to drive back to level ground manually; the base still drives.
- ERROR: |roll| > roll_error_deg or |pitch| > pitch_error_deg. The base drive is inhibited (see the node).

Pitch (about the y axis) tips the robot over its longer front/rear edge, so its limits are larger than roll's.

A level is entered only after the tilt stayed beyond its limit for persistence_s (a bump does not trip it). A level
is left only after the tilt stayed below (limit - hysteresis_deg) for recovery_hold_s; the guard then drops to the
level those lowered limits give (ERROR can drop straight to OK). While the IMU is stale (no valid sample for
imu_timeout_s) the level is held: an ERROR never clears without fresh IMU data, and pending timers restart.
"""

import json
import math
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Literal, Self

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, NonNegativeFloat, PositiveFloat, model_validator

from .metrics import TILT_GUARD_LEVEL, TILT_PITCH_DEG, TILT_ROLL_DEG

# Mount offsets beyond this are a misconfiguration (or the IMU is not on the base).
MAX_LEVEL_REFERENCE_DEG = 30.0
TORQUE_ENABLE_REGISTER = "torque_enable"
# Key order of the /tilt_guard/state JSON object (the contract consumers parse).
STATE_KEYS = (
    "level",
    "level_code",
    "roll_deg",
    "pitch_deg",
    "axis",
    "base_drive_inhibited",
    "error_action",
    "imu_stale",
    "level_age_s",
    "limits",
    "message",
    "stamp_ns",
)

Axis = Literal["roll", "pitch"]


class TiltLevel(IntEnum):
    """Tilt guard level (the int value is the level_code and the tilt_guard_level gauge)."""

    OK = 0
    WARNING = 1
    ERROR = 2


class TiltGuardConfig(BaseModel):
    """tilt_guard block of the controller config.

    Attributes:
        enabled: False never leaves OK (roll/pitch are still measured and published).
        imu_topic: sensor_msgs/Imu input with a fused orientation (BNO055 NDOF).
        state_topic: std_msgs/String JSON output (reliable, transient_local).
        torque_topic: feetech bridge set_register topic used by error_action torque_off.
        roll_warning_deg: |roll| above this is a warning, deg.
        roll_error_deg: |roll| above this is an error, deg.
        pitch_warning_deg: |pitch| above this is a warning (longer axis: larger than roll), deg.
        pitch_error_deg: |pitch| above this is an error, deg.
        persistence_s: Tilt must stay beyond a limit this long before the level rises, s.
        hysteresis_deg: A level clears only below (limit - hysteresis_deg), deg.
        recovery_hold_s: Tilt must stay below the lowered limit this long before the level drops, s.
        imu_timeout_s: The IMU is stale when no valid sample arrived for this long, s.
        level_reference_roll_deg: IMU roll when the base stands level (mount offset), deg.
        level_reference_pitch_deg: IMU pitch when the base stands level (mount offset), deg.
        error_action: 'brake' (zero wheel speed, torque on, holds on a slope) or 'torque_off' (drive torque off,
            the robot can roll).
        state_publish_hz: Heartbeat rate of the state topic (transitions are published at once), Hz.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    enabled: bool = True
    imu_topic: str = "/imu/data"
    state_topic: str = "/tilt_guard/state"
    torque_topic: str = "/follower/set_register"
    roll_warning_deg: PositiveFloat = 8.0
    roll_error_deg: PositiveFloat = 15.0
    pitch_warning_deg: PositiveFloat = 12.0
    pitch_error_deg: PositiveFloat = 20.0
    persistence_s: NonNegativeFloat = 0.5
    hysteresis_deg: NonNegativeFloat = 2.0
    recovery_hold_s: NonNegativeFloat = 2.0
    imu_timeout_s: PositiveFloat = 0.5
    level_reference_roll_deg: float = Field(0.0, ge=-MAX_LEVEL_REFERENCE_DEG, le=MAX_LEVEL_REFERENCE_DEG)
    level_reference_pitch_deg: float = Field(0.0, ge=-MAX_LEVEL_REFERENCE_DEG, le=MAX_LEVEL_REFERENCE_DEG)
    error_action: Literal["brake", "torque_off"] = "brake"
    state_publish_hz: PositiveFloat = 2.0

    @model_validator(mode="after")
    def check_limits(self) -> Self:
        """Require warning < error on each axis and a hysteresis band narrower than the smallest warning limit.

        Returns:
            Self: The validated config.

        Raises:
            ValueError: When the limits are inconsistent.
        """
        if self.roll_warning_deg >= self.roll_error_deg:
            raise ValueError("roll_warning_deg must be below roll_error_deg")
        if self.pitch_warning_deg >= self.pitch_error_deg:
            raise ValueError("pitch_warning_deg must be below pitch_error_deg")
        if self.hysteresis_deg >= min(self.roll_warning_deg, self.pitch_warning_deg):
            raise ValueError("hysteresis_deg must be below both warning limits")
        return self

    def limit(self, axis: Axis, level: TiltLevel) -> float:
        """Return the limit of one axis for WARNING or ERROR.

        Args:
            axis: 'roll' or 'pitch'.
            level: TiltLevel.WARNING or TiltLevel.ERROR.

        Returns:
            float: Limit, deg.
        """
        if axis == "roll":
            return self.roll_error_deg if level is TiltLevel.ERROR else self.roll_warning_deg
        return self.pitch_error_deg if level is TiltLevel.ERROR else self.pitch_warning_deg


@dataclass(frozen=True)
class TiltTransition:
    """A level change, for logging and the immediate state publish.

    Attributes:
        previous: Level before the change.
        current: Level after the change.
        axis: Axis that caused the new level (None for OK).
        message: Human-readable description (same text as the state message).
    """

    previous: TiltLevel
    current: TiltLevel
    axis: Axis | None
    message: str


def quaternion_to_roll_pitch_deg(x: float, y: float, z: float, w: float) -> tuple[float, float] | None:
    """Convert an orientation quaternion to roll and pitch (ZYX Euler convention, REP 103).

    Args:
        x: Quaternion x.
        y: Quaternion y.
        z: Quaternion z.
        w: Quaternion w.

    Returns:
        tuple[float, float] | None: (roll, pitch) in degrees, or None for a non-finite or zero quaternion.
    """
    q = np.array([x, y, z, w], dtype=float)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < 1e-9:
        return None
    qx, qy, qz, qw = q / norm
    roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
    return float(np.degrees(roll)), float(np.degrees(pitch))


def drive_torque_payloads(drive_joints: list[str], enable: bool) -> list[str]:
    """Build feetech bridge set_register JSON payloads switching drive-motor torque.

    Args:
        drive_joints: Drive joint names (fl_drive, fr_drive, rl_drive, rr_drive).
        enable: True writes torque_enable 1, False writes 0.

    Returns:
        list[str]: One JSON payload per joint.
    """
    value = 1 if enable else 0
    return [json.dumps({"joint_name": j, "register": TORQUE_ENABLE_REGISTER, "value": value}) for j in drive_joints]


class TiltGuard:
    """Tilt level state machine fed with IMU orientation samples.

    Attributes:
        config: Tilt guard config.
        level: Current level.
        axis: Axis that caused the current level (None while OK).
        roll_deg: Last roll minus the level reference, deg (None before the first sample).
        pitch_deg: Last pitch minus the level reference, deg (None before the first sample).
        last_sample_s: Monotonic time of the last valid sample, s.
        level_since_s: Monotonic time the current level was entered, s.
    """

    def __init__(self, config: TiltGuardConfig) -> None:
        """Create the guard in OK with no sample.

        Args:
            config: Tilt guard config.
        """
        self.config = config
        self.level = TiltLevel.OK
        self.axis: Axis | None = None
        self.roll_deg: float | None = None
        self.pitch_deg: float | None = None
        self.last_sample_s: float | None = None
        self.level_since_s: float | None = None
        self.rise_since: dict[TiltLevel, float] = {}
        self.clear_since: float | None = None
        TILT_GUARD_LEVEL.set(int(self.level))

    @property
    def base_drive_inhibited(self) -> bool:
        """True while the level is ERROR: drive commands are ignored and the wheels are stopped.

        Returns:
            bool: Whether the base drive is inhibited.
        """
        return self.level is TiltLevel.ERROR

    def imu_stale(self, now: float) -> bool:
        """Return whether the IMU is stale.

        Args:
            now: Monotonic time, s.

        Returns:
            bool: True before the first valid sample or when the last one is older than imu_timeout_s.
        """
        return self.last_sample_s is None or now - self.last_sample_s > self.config.imu_timeout_s

    def worst_axis(self, level: TiltLevel, roll: float, pitch: float, margin_deg: float) -> tuple[float, Axis]:
        """Find the axis closest to (or furthest beyond) its limit for one level.

        Args:
            level: TiltLevel.WARNING or TiltLevel.ERROR.
            roll: Roll, deg.
            pitch: Pitch, deg.
            margin_deg: Subtracted from the limits (0 to rise, hysteresis_deg to clear).

        Returns:
            tuple[float, Axis]: (|tilt| / lowered limit, axis); a ratio above 1 means the limit is exceeded.
        """
        roll_ratio = abs(roll) / (self.config.limit("roll", level) - margin_deg)
        pitch_ratio = abs(pitch) / (self.config.limit("pitch", level) - margin_deg)
        return (roll_ratio, "roll") if roll_ratio >= pitch_ratio else (pitch_ratio, "pitch")

    def classify(self, roll: float, pitch: float, margin_deg: float) -> tuple[TiltLevel, Axis | None]:
        """Classify a tilt against the limits lowered by margin_deg.

        Args:
            roll: Roll, deg.
            pitch: Pitch, deg.
            margin_deg: Subtracted from every limit (0 to rise, hysteresis_deg to clear).

        Returns:
            tuple[TiltLevel, Axis | None]: Highest level exceeded and the axis exceeding it most, or (OK, None).
        """
        for level in (TiltLevel.ERROR, TiltLevel.WARNING):
            ratio, axis = self.worst_axis(level, roll, pitch, margin_deg)
            if ratio > 1.0:
                return level, axis
        return TiltLevel.OK, None

    def update_from_quaternion(self, x: float, y: float, z: float, w: float, now: float) -> TiltTransition | None:
        """Feed one IMU orientation; the level reference is subtracted from its roll and pitch.

        Args:
            x: Quaternion x.
            y: Quaternion y.
            z: Quaternion z.
            w: Quaternion w.
            now: Monotonic receive time, s.

        Returns:
            TiltTransition | None: The level change this sample caused, if any (None also for an invalid quaternion,
            which is not counted as a sample).
        """
        angles = quaternion_to_roll_pitch_deg(x, y, z, w)
        if angles is None:
            return None
        return self.update(
            angles[0] - self.config.level_reference_roll_deg,
            angles[1] - self.config.level_reference_pitch_deg,
            now,
        )

    def update(self, roll: float, pitch: float, now: float) -> TiltTransition | None:
        """Feed one roll/pitch sample (already relative to level) and advance the state machine.

        Args:
            roll: Roll, deg.
            pitch: Pitch, deg.
            now: Monotonic receive time, s.

        Returns:
            TiltTransition | None: The level change, if any.
        """
        if not (math.isfinite(roll) and math.isfinite(pitch)):
            return None
        if self.imu_stale(now):
            # A gap in the data: time before it must not count towards persistence or the recovery hold.
            self.rise_since.clear()
            self.clear_since = None
        self.roll_deg, self.pitch_deg, self.last_sample_s = roll, pitch, now
        TILT_ROLL_DEG.set(roll)
        TILT_PITCH_DEG.set(pitch)
        if not self.config.enabled:
            return None

        raw, _ = self.classify(roll, pitch, 0.0)
        for level in (TiltLevel.WARNING, TiltLevel.ERROR):
            if raw >= level:
                self.rise_since.setdefault(level, now)
            else:
                self.rise_since.pop(level, None)
        for level in (TiltLevel.ERROR, TiltLevel.WARNING):
            since = self.rise_since.get(level)
            if level > self.level and since is not None and now - since >= self.config.persistence_s:
                return self.enter(level, self.worst_axis(level, roll, pitch, 0.0)[1], now)

        cleared, cleared_axis = self.classify(roll, pitch, self.config.hysteresis_deg)
        if cleared >= self.level:
            self.clear_since = None
            return None
        if self.clear_since is None:
            self.clear_since = now
        if now - self.clear_since >= self.config.recovery_hold_s:
            return self.enter(cleared, cleared_axis, now)
        return None

    def enter(self, level: TiltLevel, axis: Axis | None, now: float) -> TiltTransition:
        """Switch to a new level.

        Args:
            level: New level.
            axis: Axis that caused it (None for OK).
            now: Monotonic time, s.

        Returns:
            TiltTransition: The change.
        """
        previous = self.level
        self.level, self.level_since_s, self.clear_since = level, now, None
        self.axis = axis
        TILT_GUARD_LEVEL.set(int(level))
        return TiltTransition(previous, level, self.axis, self.message())

    def message(self) -> str:
        """Describe the current level for operators and the agent.

        Returns:
            str: Message text.
        """
        cfg = self.config
        roll = self.roll_deg if self.roll_deg is not None else math.nan
        pitch = self.pitch_deg if self.pitch_deg is not None else math.nan
        if self.level is TiltLevel.OK or self.axis is None:
            return f"level: roll {roll:.1f} deg, pitch {pitch:.1f} deg within limits"
        value = abs(roll if self.axis == "roll" else pitch)
        limit = cfg.limit(self.axis, self.level)
        if self.level is TiltLevel.WARNING:
            return (
                f"tilt warning: {self.axis} {value:.1f} deg beyond the {limit:.1f} deg warning limit; stop automatic "
                "driving and drive back to level ground manually"
            )
        return (
            f"tilt error: {self.axis} {value:.1f} deg beyond the {limit:.1f} deg error limit; base drive motors "
            f"disabled ({cfg.error_action}) until the IMU reads below {limit - cfg.hysteresis_deg:.1f} deg for "
            f"{cfg.recovery_hold_s:.1f} s"
        )

    def state_dict(self, now: float, stamp_ns: int) -> dict[str, Any] | None:
        """Build the /tilt_guard/state JSON object.

        Args:
            now: Monotonic time, s.
            stamp_ns: ROS clock time of the publish, ns.

        Returns:
            dict[str, Any] | None: State with the keys in STATE_KEYS order, or None before the first IMU sample
            (nothing is published without real data).
        """
        if self.last_sample_s is None:
            return None
        cfg = self.config
        stale = self.imu_stale(now)
        message = self.message()
        if stale:
            message += f" (IMU stale: last sample {now - self.last_sample_s:.1f} s ago)"
        return {
            "level": self.level.name.lower(),
            "level_code": int(self.level),
            "roll_deg": self.roll_deg,
            "pitch_deg": self.pitch_deg,
            "axis": self.axis,
            "base_drive_inhibited": self.base_drive_inhibited,
            "error_action": cfg.error_action,
            "imu_stale": stale,
            "level_age_s": None if self.level_since_s is None else now - self.level_since_s,
            "limits": {
                "roll_warning_deg": cfg.roll_warning_deg,
                "roll_error_deg": cfg.roll_error_deg,
                "pitch_warning_deg": cfg.pitch_warning_deg,
                "pitch_error_deg": cfg.pitch_error_deg,
                "hysteresis_deg": cfg.hysteresis_deg,
            },
            "message": message,
            "stamp_ns": stamp_ns,
        }
