"""Pure servo calibration planning: range tracker and per-joint plan (calibration_plan)."""

import json
import math

import pytest

from feetech_servos.calibration_plan import (
    RangeTracker,
    degrees_to_steps,
    plan_joint,
)
from feetech_servos.config import JointEntry, ServoCalibrationConfig
from feetech_servos.servo_offsets import steps_to_degrees

FULL_RANGE = (0, 4095)
MARGIN_STEPS = 23  # 2 deg
GUARD_STEPS = 57  # 5 deg


def make_config(**overrides: object) -> ServoCalibrationConfig:
    values: dict[str, object] = {
        "home_reference_rad": {"shoulder_lift": -math.pi / 2, "wrist_roll": 0.0, "steer": 0.0},
        "offset_exclude_joints": ["gripper"],
        "record_dir": "/tmp/records",
        "full_turn_joints": ["wrist_roll"],
    }
    values.update(overrides)
    return ServoCalibrationConfig(**values)


def make_tracker(home: int, min_rel: int, max_rel: int) -> RangeTracker:
    return RangeTracker(home_steps=home, min_rel=min_rel, max_rel=max_rel, samples=2)


def plan(
    name: str = "steer",
    home: int = 2048,
    offset: int = 0,
    limits: tuple[int, int] = FULL_RANGE,
    tracker: RangeTracker | None = None,
    inverted: bool = False,
    **config: object,
):
    joint = JointEntry(name=name, id=3, inverted=inverted)
    return plan_joint(joint, make_config(**config), home, offset, limits, tracker)


def test_degrees_to_steps_rounds() -> None:
    assert degrees_to_steps(2) == 23
    assert degrees_to_steps(5) == 57


def test_tracker_folds_min_max_relative_to_home() -> None:
    tracker = RangeTracker(home_steps=2048)
    for steps in (2000, 2300, 1900):
        tracker.fold(steps)
    assert (tracker.min_rel, tracker.max_rel, tracker.samples) == (-148, 252, 3)
    assert tracker.last_steps == 1900


def test_tracker_unwraps_through_the_wrap() -> None:
    tracker = RangeTracker(home_steps=4000)
    for steps in (4090, 10, 100):
        tracker.fold(steps)
    assert (tracker.min_rel, tracker.max_rel) == (90, 196)
    assert tracker.rel == 196


def test_plan_arm_shoulder_lift_target_minus_half_pi() -> None:
    result = plan("shoulder_lift", home=1100)
    assert result.target_steps == 1024
    assert result.offset_change_steps == 76
    assert result.new_offset_steps == 76
    assert result.problems == []
    assert result.limits_after == FULL_RANGE
    assert result.range_deg is None


def test_plan_wraps_new_offset_from_current_offset() -> None:
    result = plan(home=2148, offset=2000)
    assert result.offset_change_steps == 100
    assert result.new_offset_steps == -1996
    assert result.problems == []


def test_plan_refuses_unencodable_offset() -> None:
    result = plan(home=2096, offset=2000)
    assert result.offset_change_steps == 48
    assert any("-2048" in p for p in result.problems)


def test_plan_excluded_joint_keeps_offset() -> None:
    result = plan("gripper", home=2500, offset=10)
    assert result.target_steps == 2500
    assert result.offset_change_steps == 0
    assert result.new_offset_steps == 10


def test_plan_limits_are_extremes_minus_inward_margin() -> None:
    result = plan(tracker=make_tracker(2048, -500, 600))
    assert result.limits_after == (1571, 2625)
    assert result.problems == []
    assert result.range_deg == pytest.approx(steps_to_degrees(1100))


def test_plan_inverted_joint_target() -> None:
    result = plan("shoulder_lift", home=3000, inverted=True)
    assert result.target_steps == 3072


def test_plan_refuses_small_range() -> None:
    result = plan(tracker=make_tracker(2048, -85, 85))  # 170 steps = 14.9 deg
    assert any("range" in p for p in result.problems)
    assert result.limits_after is None


def test_plan_refuses_home_outside_range() -> None:
    result = plan(tracker=make_tracker(2048, 10, 600))
    assert any("home" in p for p in result.problems)


def test_plan_refuses_wrap_crossing_in_new_frame() -> None:
    result = plan("shoulder_lift", home=1024, tracker=make_tracker(1024, -1000, 500))
    assert any("wrap" in p for p in result.problems)


def test_plan_refuses_wrap_crossing_at_the_top() -> None:
    result = plan(tracker=make_tracker(2048, -500, 2040))
    assert any("wrap" in p for p in result.problems)


def test_plan_full_turn_joint_keeps_full_range() -> None:
    result = plan("wrist_roll", tracker=make_tracker(2048, -2000, 1982))
    assert result.full_turn
    assert result.range_deg == pytest.approx(350, abs=0.1)
    assert result.limits_after == FULL_RANGE
    assert result.problems == []


def test_plan_full_turn_joint_with_small_range_gets_limits() -> None:
    result = plan("wrist_roll", tracker=make_tracker(2048, -900, 1000))  # 167 deg
    assert not result.full_turn
    assert result.limits_after == (2048 - 900 + MARGIN_STEPS, 2048 + 1000 - MARGIN_STEPS)
    wide = plan("wrist_roll", tracker=make_tracker(2048, -1100, 1100))  # 193 deg
    assert not wide.full_turn
    assert wide.limits_after == (948 + MARGIN_STEPS, 3148 - MARGIN_STEPS)


def test_plan_home_only_shifts_existing_limits() -> None:
    result = plan(home=2148, limits=(1000, 3000))
    assert result.offset_change_steps == 100
    assert result.limits_after == (900, 2900)
    assert result.problems == []


def test_plan_home_only_keeps_full_range() -> None:
    result = plan(home=2148)
    assert result.limits_after == FULL_RANGE


def test_plan_home_only_refuses_shift_out_of_range() -> None:
    result = plan(home=2148, limits=(50, 3000))
    assert result.problems
    assert result.limits_after is None


def test_plan_flags_large_change() -> None:
    change = degrees_to_steps(15.1)
    result = plan(home=2048 + change)
    assert result.exceeds_max_change
    assert result.problems == []
    assert not plan(home=2048 + 100).exceeds_max_change


def test_plan_flags_narrower_than_required_travel() -> None:
    config = {"required_travel_rad": {"steer": (-1.5708, 1.5708)}}
    narrow = plan(limits=(1100, 3000), **config)
    assert narrow.narrower_than_required
    assert narrow.problems == []
    assert not plan(limits=(1000, 3100), **config).narrower_than_required
    assert not plan(limits=FULL_RANGE, **config).narrower_than_required


def test_plan_required_travel_inverted_sorts_ends() -> None:
    config = {"required_travel_rad": {"steer": (-0.5, 1.0)}}
    result = plan(inverted=True, limits=(900, 2600), **config)
    assert not result.narrower_than_required


def test_payload_is_json_serialisable() -> None:
    payload = plan("shoulder_lift", home=1100, tracker=make_tracker(1024, -300, 300)).payload()
    json.dumps(payload)
    assert payload["name"] == "shoulder_lift"
    assert payload["change_deg"] == pytest.approx(steps_to_degrees(76))
    assert payload["target_rad"] == pytest.approx(-math.pi / 2, abs=1e-3)
    assert payload["home_rad"] == pytest.approx((1100 - 2048) * 2 * math.pi / 4096)
