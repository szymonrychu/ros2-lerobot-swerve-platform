"""Replay plan parsing, label lookup and joint interpolation."""

import json

import pytest

from grasp_sim.plan import JOINT_ORDER, LABELS, interpolate_joints, label_at, parse_plan

HOME = dict.fromkeys(JOINT_ORDER, 0.0)


def sample(t: float, label: str | None = None, **joints: float) -> dict:
    out: dict = {"t": t, "joints": {**HOME, **joints}}
    if label:
        out["label"] = label
    return out


def test_labels_are_the_documented_set() -> None:
    assert LABELS == ("open", "pre_grasp", "approach", "grasp", "close", "lift", "retreat")


def test_parse_plan_accepts_json_string_list_and_dict() -> None:
    raw = [sample(0.0), sample(1.0, shoulder_pan=0.5)]
    for payload in (json.dumps(raw), raw, {"samples": raw}):
        plan = parse_plan(payload)
        assert [s.t for s in plan] == [0.0, 1.0]
        assert plan[1].joints["shoulder_pan"] == 0.5


def test_parse_plan_rejects_unsorted_time_unknown_label_and_unknown_joint() -> None:
    with pytest.raises(ValueError, match="strictly increasing"):
        parse_plan([sample(1.0), sample(0.5)])
    with pytest.raises(ValueError, match="label"):
        parse_plan([sample(0.0, "teleport"), sample(1.0)])
    with pytest.raises(ValueError, match="joint"):
        parse_plan([{"t": 0.0, "joints": {**HOME, "wing": 1.0}}, sample(1.0)])


def test_parse_plan_needs_two_samples_and_a_full_first_sample() -> None:
    with pytest.raises(ValueError, match="at least two"):
        parse_plan([sample(0.0)])
    with pytest.raises(ValueError, match="first sample"):
        parse_plan([{"t": 0.0, "joints": {"shoulder_pan": 0.0}}, sample(1.0)])


def test_partial_later_samples_hold_previous_values() -> None:
    plan = parse_plan([sample(0.0, elbow_flex=-1.0), {"t": 1.0, "joints": {"shoulder_pan": 0.3}}])
    assert plan[1].joints["elbow_flex"] == -1.0
    assert plan[1].joints["shoulder_pan"] == 0.3


def test_label_applies_until_next_labelled_sample_and_unlabelled_start_is_none() -> None:
    plan = parse_plan([sample(0.0), sample(1.0, "approach"), sample(2.0), sample(3.0, "grasp")])
    assert label_at(plan, 0.5) is None
    assert label_at(plan, 1.0) == "approach"
    assert label_at(plan, 2.5) == "approach"
    assert label_at(plan, 3.0) == "grasp"
    assert label_at(plan, 99.0) == "grasp"


def test_interpolate_joints_is_linear_and_clamped() -> None:
    plan = parse_plan([sample(0.0), sample(2.0, shoulder_pan=1.0)])
    assert interpolate_joints(plan, 1.0)["shoulder_pan"] == pytest.approx(0.5)
    assert interpolate_joints(plan, -1.0)["shoulder_pan"] == 0.0
    assert interpolate_joints(plan, 5.0)["shoulder_pan"] == 1.0
