"""The tolerant GraspPlan -> replay plan adapter."""

import json
import math

import pytest

from grasp_sim.adapter import grasp_plan_to_replay
from grasp_sim.plan import JOINT_ORDER, parse_plan

POSE = dict.fromkeys(JOINT_ORDER, 0.1)


def test_waypoints_with_durations_become_cumulative_timed_samples() -> None:
    plan = {
        "waypoints": [
            {"label": "pre_grasp", "joints": POSE},
            {"label": "approach", "duration_s": 2.0, "joints": {**POSE, "shoulder_pan": 0.5}},
            {"label": "grasp", "duration_s": 1.0, "joints": {**POSE, "shoulder_pan": 0.5}},
        ]
    }
    out = grasp_plan_to_replay(plan)
    assert [s["t"] for s in out] == [0.0, 2.0, 3.0]
    assert [s["label"] for s in out] == ["pre_grasp", "approach", "grasp"]
    assert out[1]["joints"]["shoulder_pan"] == 0.5
    assert len(parse_plan(out)) == 3


def test_joint_samples_per_waypoint_keep_their_times_and_label_only_the_first() -> None:
    plan = {
        "waypoints": [
            {
                "label": "approach",
                "joint_samples": [
                    {"t": 0.0, "joints": POSE},
                    {"t": 0.5, "joints": {**POSE, "elbow_flex": 0.3}},
                ],
            },
            {"label": "lift", "joint_samples": [{"t": 0.0, "joints": POSE}, {"t": 0.4, "joints": POSE}]},
        ]
    }
    out = grasp_plan_to_replay(plan)
    assert [s["t"] for s in out] == pytest.approx([0.0, 0.5, 0.5 + 0.0 + 1e-3, 0.9], abs=2e-3)
    assert [s.get("label") for s in out] == ["approach", None, "lift", None]


def test_accepts_alternate_keys_aliases_lists_and_degrees() -> None:
    plan = {
        "units": "deg",
        "joint_names": list(JOINT_ORDER),
        "trajectory": [
            {"phase": "Pre-Grasp", "time_from_start": 0.0, "positions": [90.0, 0, 0, 0, 0, 0]},
            {
                "phase": "descend",
                "time_from_start": {"sec": 1, "nanosec": 500_000_000},
                "positions": [90, 0, 0, 0, 0, 0],
            },
        ],
    }
    out = grasp_plan_to_replay(plan)
    assert out[0]["joints"]["shoulder_pan"] == pytest.approx(math.pi / 2)
    assert out[0]["label"] == "pre_grasp"
    assert out[1]["label"] == "approach"
    assert out[1]["t"] == pytest.approx(1.5)


def test_separate_gripper_key_joint_suffixes_json_text_and_bare_list() -> None:
    arm = {f"{j}_joint": 0.2 for j in JOINT_ORDER[:5]}
    raw = [{"label": "open", "joints": arm, "gripper": 0.9}, {"duration": 1.0, "joints": arm, "gripper_rad": 0.0}]
    out = grasp_plan_to_replay(json.dumps(raw))
    assert out[0]["joints"]["gripper"] == 0.9
    assert out[0]["joints"]["shoulder_pan"] == 0.2
    assert out[1]["joints"]["gripper"] == 0.0


def test_unknown_label_is_dropped_not_fatal() -> None:
    raw = {"waypoints": [{"label": "dance", "joints": POSE}, {"label": "grasp", "duration_s": 1, "joints": POSE}]}
    out = grasp_plan_to_replay(raw)
    assert "label" not in out[0]
    assert out[1]["label"] == "grasp"


def test_missing_times_default_to_one_second_spacing() -> None:
    raw = {"waypoints": [{"joints": POSE}, {"joints": POSE}, {"joints": POSE}]}
    assert [s["t"] for s in grasp_plan_to_replay(raw)] == [0.0, 1.0, 2.0]


@pytest.mark.parametrize(
    "bad",
    [
        {"waypoints": []},
        {"nothing": 1},
        {"waypoints": [{"joints": {"shoulder_pan": 0.0}}, {"joints": POSE}]},
        {"waypoints": [{"label": "grasp"}, {"joints": POSE}]},
    ],
)
def test_unusable_plans_raise_value_error(bad: dict) -> None:
    with pytest.raises(ValueError):
        grasp_plan_to_replay(bad)
