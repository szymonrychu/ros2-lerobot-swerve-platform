"""Pure servo calibration planning (no rclpy, no bus): range tracking and the per-joint offset and limit plan.

The home offset change makes the hand-posed joint read its target (spec 3.2); the captured range, expressed relative
to home and therefore wrap-safe, gives the new angle limits in the new frame (spec 3.4).
"""

from dataclasses import dataclass, field
from typing import Any

from feetech_mapping.command_mapping import STEPS_PER_REVOLUTION, position_to_raw_steps, steps_to_radians

from .config import JointEntry, ServoCalibrationConfig
from .servo_offsets import (
    DEGREES_PER_TURN,
    HALF_TURN_STEPS,
    OFFSET_MAX_STEPS,
    offset_delta_steps,
    steps_to_degrees,
    wrap_offset_steps,
)

MAX_STEP = STEPS_PER_REVOLUTION - 1
FULL_LIMITS = (0, MAX_STEP)


@dataclass
class RangeTracker:
    """Wrap-safe min/max of one joint's travel, unwrapped relative to the home reading.

    Attributes:
        home_steps: Recorded home reading (0..4095, old frame).
        last_steps: Last folded reading, None before the first sample.
        rel: Current position relative to home (steps, unwrapped, may exceed one turn).
        min_rel: Smallest rel seen, None before the first sample.
        max_rel: Largest rel seen, None before the first sample.
        samples: Number of folded readings.
    """

    home_steps: int
    last_steps: int | None = None
    rel: int = 0
    min_rel: int | None = None
    max_rel: int | None = None
    samples: int = 0

    def fold(self, steps: int) -> None:
        """Fold one present_position reading into the tracked range.

        Args:
            steps (int): Reading, 0..4095 (old frame).

        Returns:
            None
        """
        if self.last_steps is None:
            self.rel = offset_delta_steps(steps, self.home_steps)
        else:
            self.rel += offset_delta_steps(steps, self.last_steps)
        self.min_rel = self.rel if self.min_rel is None else min(self.min_rel, self.rel)
        self.max_rel = self.rel if self.max_rel is None else max(self.max_rel, self.rel)
        self.samples += 1
        self.last_steps = steps


@dataclass
class JointCalibrationPlan:
    """Planned offset change and angle limits of one joint.

    Attributes:
        name: Joint name.
        id: Servo ID.
        inverted: Joint direction is opposite to the servo's.
        home_steps: Recorded home reading (old frame).
        target_steps: Wanted reading at home (new frame).
        offset_before_steps: Offset register before (steps).
        offset_change_steps: Offset change, home_steps - target_steps (short way round).
        new_offset_steps: Offset after, wrapped to -2048..2047 (-2048 is a problem).
        limits_before: (min, max) angle limits before.
        limits_after: (min, max) angle limits after, None when they cannot be planned.
        range_deg: Captured range in degrees, None without a capture.
        full_turn: The joint keeps 0..4095 as a full-turn joint.
        exceeds_max_change: |change| is above max_change_deg (needs the confirm).
        narrower_than_required: limits_after are narrower than required_travel_rad (needs the confirm).
        problems: Reasons the store is refused; empty when the plan is storable.
    """

    name: str
    id: int  # noqa: A003
    inverted: bool
    home_steps: int
    target_steps: int
    offset_before_steps: int
    offset_change_steps: int
    new_offset_steps: int
    limits_before: tuple[int, int]
    limits_after: tuple[int, int] | None
    range_deg: float | None
    full_turn: bool
    exceeds_max_change: bool
    narrower_than_required: bool
    problems: list[str] = field(default_factory=list)

    def payload(self) -> dict[str, Any]:
        """Return the plan as JSON-serialisable values (for the status topic and the audit record).

        Returns:
            dict[str, Any]: All fields (tuples as lists) plus home_rad, target_rad and change_deg.
        """
        return {
            "name": self.name,
            "id": self.id,
            "inverted": self.inverted,
            "home_steps": self.home_steps,
            "target_steps": self.target_steps,
            "home_rad": steps_to_radians(self.home_steps, self.inverted),
            "target_rad": steps_to_radians(self.target_steps, self.inverted),
            "offset_before_steps": self.offset_before_steps,
            "offset_change_steps": self.offset_change_steps,
            "change_deg": steps_to_degrees(self.offset_change_steps),
            "new_offset_steps": self.new_offset_steps,
            "limits_before": list(self.limits_before),
            "limits_after": None if self.limits_after is None else list(self.limits_after),
            "range_deg": self.range_deg,
            "full_turn": self.full_turn,
            "exceeds_max_change": self.exceeds_max_change,
            "narrower_than_required": self.narrower_than_required,
            "problems": list(self.problems),
        }


def degrees_to_steps(deg: float) -> int:
    """Convert degrees to the nearest whole number of servo steps.

    Args:
        deg (float): Angle in degrees.

    Returns:
        int: Steps (4096 per turn), rounded.
    """
    return round(deg * STEPS_PER_REVOLUTION / DEGREES_PER_TURN)


def plan_joint(
    joint: JointEntry,
    config: ServoCalibrationConfig,
    home_steps: int,
    offset_before: int,
    limits_before: tuple[int, int],
    tracker: RangeTracker | None,
) -> JointCalibrationPlan:
    """Plan the offset change and angle limits of one joint.

    Args:
        joint (JointEntry): The joint.
        config (ServoCalibrationConfig): Calibration targets and thresholds.
        home_steps (int): Recorded home reading (0..4095).
        offset_before (int): Offset register now (steps).
        limits_before (tuple[int, int]): (min, max) angle limits now.
        tracker (RangeTracker | None): Captured range, None for a home-only store.

    Returns:
        JointCalibrationPlan: The plan; problems lists every reason it must not be stored.
    """
    problems: list[str] = []
    excluded = joint.name in config.offset_exclude_joints
    target = home_steps if excluded else position_to_raw_steps(config.home_reference_rad[joint.name], joint.inverted)
    change = offset_delta_steps(home_steps, target)
    new_offset = wrap_offset_steps(offset_before + change)
    if new_offset == -HALF_TURN_STEPS:
        problems.append(
            f"new offset {offset_before} + {change} wraps to -2048, which cannot be encoded "
            f"(|offset| <= {OFFSET_MAX_STEPS}); move it a little and record home again"
        )
    range_deg: float | None = None
    full_turn = False
    limits: tuple[int, int] | None
    if tracker is None or tracker.min_rel is None or tracker.max_rel is None:
        limits, shift_problem = shifted_limits(limits_before, change)
        if shift_problem:
            problems.append(shift_problem)
    else:
        span = tracker.max_rel - tracker.min_rel
        range_deg = steps_to_degrees(span)
        full_turn = joint.name in config.full_turn_joints and range_deg >= config.full_turn_min_deg
        if full_turn:
            limits = FULL_LIMITS
        else:
            lo = target + tracker.min_rel
            hi = target + tracker.max_rel
            guard = degrees_to_steps(config.wrap_guard_deg)
            margin = degrees_to_steps(config.margin_deg)
            found = len(problems)
            if range_deg < config.min_range_deg:
                problems.append(f"captured range {range_deg:.1f} deg is below {config.min_range_deg:g} deg")
            if not tracker.min_rel <= 0 <= tracker.max_rel:
                problems.append("home is outside the captured range (move the joint through home while recording)")
            if lo < guard or hi > MAX_STEP - guard:
                problems.append(
                    f"range {lo}..{hi} in the new frame is within {config.wrap_guard_deg:g} deg of the 0/4095 wrap"
                )
            limits = (max(0, lo + margin), min(MAX_STEP, hi - margin))
            if len(problems) == found and limits[0] >= limits[1]:
                problems.append(f"limits {limits[0]}..{limits[1]} leave no travel")
            if len(problems) != found:
                limits = None
    narrower = False
    if limits is not None and joint.name in config.required_travel_rad:
        ends = sorted(position_to_raw_steps(rad, joint.inverted) for rad in config.required_travel_rad[joint.name])
        narrower = limits[0] > ends[0] or limits[1] < ends[1]
    return JointCalibrationPlan(
        name=joint.name,
        id=joint.id,
        inverted=joint.inverted,
        home_steps=home_steps,
        target_steps=target,
        offset_before_steps=offset_before,
        offset_change_steps=change,
        new_offset_steps=new_offset,
        limits_before=limits_before,
        limits_after=limits,
        range_deg=range_deg,
        full_turn=full_turn,
        exceeds_max_change=abs(steps_to_degrees(change)) > config.max_change_deg,
        narrower_than_required=narrower,
        problems=problems,
    )


def shifted_limits(limits_before: tuple[int, int], change: int) -> tuple[tuple[int, int] | None, str]:
    """Shift existing limits by -change so they keep the same physical travel (home-only store).

    Args:
        limits_before (tuple[int, int]): (min, max) angle limits now.
        change (int): Offset change in steps.

    Returns:
        tuple[tuple[int, int] | None, str]: (new limits, "") or (None, problem text). 0..4095 stays 0..4095.
    """
    if limits_before == FULL_LIMITS:
        return FULL_LIMITS, ""
    shifted = (limits_before[0] - change, limits_before[1] - change)
    if shifted[0] < 0 or shifted[1] > MAX_STEP:
        return None, f"shifting limits {limits_before[0]}..{limits_before[1]} by {-change} leaves 0..{MAX_STEP}"
    return shifted, ""
