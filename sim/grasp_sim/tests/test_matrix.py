"""Planner scenario matrix: executor-timed replay of mcp_server GraspPlans and the summary table."""

import json
import math
from pathlib import Path

import pytest

from grasp_sim.cli import main
from grasp_sim.matrix import (
    ExecutorTiming,
    MatrixEntry,
    MatrixIndex,
    executor_samples,
    path_samples,
    scene_for,
    summary_table,
)

ARM = {"shoulder_pan": 0.0, "shoulder_lift": -0.3, "elbow_flex": 0.6, "wrist_flex": 0.9, "wrist_roll": -1.57}


def pose(**changes: float) -> dict[str, float]:
    return ARM | changes


def waypoint(label: str, joints: dict[str, float], gripper: float | None, speed: float, linear: bool) -> dict:
    return {
        "label": label,
        "x": 0.2,
        "y": 0.0,
        "z": 0.0,
        "pitch": 1.57,
        "roll": -1.57,
        "gripper": gripper,
        "gripper_action": "close" if label == "close" else "set",
        "speed_scale": speed,
        "linear": linear,
        "joints": joints,
    }


def planner_plan(lift_speed: float = 0.15) -> dict:
    """A small GraspPlan.model_dump()-shaped plan (arm stays high, nothing to grasp)."""
    pre, low, lifted = pose(), pose(shoulder_lift=-0.2), pose(shoulder_lift=-0.4)
    return {
        "strategy": "top_down",
        "feasible": True,
        "waypoints": [
            waypoint("pre_grasp", pre, 0.8, 0.5, False),
            waypoint("open", pre, 1.2, 0.5, False),
            waypoint("approach", pre, 1.2, 0.15, True),
            waypoint("grasp", low, 1.2, 0.15, True),
            waypoint("close", low, 0.1, 0.15, False),
            waypoint("lift", lifted, 0.1, lift_speed, True),
            waypoint("retreat", lifted, 0.1, 0.15, True),
        ],
        "segments": {
            "approach": [pre | {"gripper": 1.2}],
            "grasp": [pose(shoulder_lift=-0.25) | {"gripper": 1.2}, low | {"gripper": 1.2}],
            "lift": [pose(shoulder_lift=-0.3) | {"gripper": 0.1}, lifted | {"gripper": 0.1}],
            "retreat": [lifted | {"gripper": 0.1}],
        },
    }


def test_path_samples_follow_one_quintic_profile_within_the_velocity_cap() -> None:
    start = {"a": 0.0}
    path = [{"a": 0.5}, {"a": 1.0}]
    samples = path_samples(start, path, max_velocity=0.5, rate_hz=25.0)
    assert samples[-1] == {"a": 1.0}
    assert len(samples) == math.ceil(15.0 / 8.0 * 1.0 / 0.5 * 25.0)
    speeds = [abs(b["a"] - a["a"]) * 25.0 for a, b in zip([start, *samples], samples, strict=False)]
    assert max(speeds) <= 0.5 + 1e-6


def test_executor_samples_run_the_phases_in_order_and_hold_closed_from_the_close() -> None:
    timing = ExecutorTiming()
    samples = executor_samples(planner_plan(), timing)
    labels = [s["label"] for s in samples if "label" in s]
    assert labels == ["pre_grasp", "open", "approach", "grasp", "close", "lift", "retreat"]
    times = [s["t"] for s in samples]
    assert times == sorted(times) and len(set(times)) == len(times)
    assert all(set(s["joints"]) == {*ARM, "gripper"} for s in samples)
    lift_index = next(i for i, s in enumerate(samples) if s.get("label") == "lift")
    assert samples[lift_index - 1]["joints"]["gripper"] == pytest.approx(timing.gripper_closed_rad)
    assert all(s["joints"]["gripper"] == pytest.approx(timing.gripper_closed_rad) for s in samples[lift_index:])
    assert samples[0]["joints"]["gripper"] == pytest.approx(0.8)


def test_slower_lift_speed_scale_takes_longer() -> None:
    def lift_duration(speed: float) -> float:
        samples = executor_samples(planner_plan(lift_speed=speed), ExecutorTiming())
        start = next(s["t"] for s in samples if s.get("label") == "lift")
        end = next(s["t"] for s in samples if s.get("label") == "retreat")
        return end - start

    assert lift_duration(0.05) > 2.5 * lift_duration(0.15)


def entry(**changes: object) -> MatrixEntry:
    base = {
        "key": "4x4x4_r20_ledge-0.08_top_down",
        "box": "4x4x4",
        "position": "r20",
        "support": "ledge-0.08",
        "strategy": "top_down",
        "x": 0.2,
        "y": 0.0,
        "support_z": -0.08,
        "surface_z": -0.08,
        "size_m": (0.04, 0.04, 0.04),
        "gap_below_m": 0.0,
        "feasible": True,
        "reasons": [],
        "chosen": "top_down",
        "plan_file": "plan.json",
    }
    return MatrixEntry.model_validate(base | changes)


def test_scene_for_an_entry_uses_the_calibrated_jaws_and_faces_the_object_radially() -> None:
    index = MatrixIndex(base_height_m=0.15, floor_z_m=-0.15, tool_offset_m=(0.0010, -0.0056, -0.0014), entries=[])
    scene = scene_for(entry(x=0.2, y=0.1), index, stock_jaws=False)
    assert scene.tool_offset_m == pytest.approx((0.0010, -0.0056, -0.0014))
    assert scene.support_z_m == pytest.approx(-0.08)
    assert scene.object is not None and scene.object.yaw_rad == pytest.approx(math.atan2(0.1, 0.2))
    assert scene_for(entry(), index, stock_jaws=True).tool_offset_m is None


def test_scene_for_a_gap_entry_puts_the_rails_on_the_surface_under_the_object() -> None:
    index = MatrixIndex(base_height_m=0.15, floor_z_m=-0.15, tool_offset_m=(0.0, 0.0, 0.0), entries=[])
    scene = scene_for(
        entry(support="floor", support_z=-0.13, surface_z=-0.15, gap_below_m=0.02), index, stock_jaws=False
    )
    assert scene.support_kind == "floor"
    assert scene.object is not None and scene.object.gap_below_m == pytest.approx(0.02)


def test_scene_for_a_stair_entry_puts_the_step_edge_where_the_planner_was_told() -> None:
    index = MatrixIndex(base_height_m=0.1, floor_z_m=-0.1, tool_offset_m=(0.0, 0.0, 0.0), entries=[])
    stair = entry(support="stair-0.10", support_z=-0.2, surface_z=-0.2, x=0.3, support_edge_x=0.25)
    scene = scene_for(stair, index, stock_jaws=False)
    assert scene.support_kind == "stair" and scene.support_start_x == pytest.approx(0.25)
    assert scene_for(entry(x=0.3), index, stock_jaws=False).support_start_x == pytest.approx(0.22)  # default margin


def test_summary_table_counts_lifts_per_strategy_and_box() -> None:
    results = [
        {"key": "a", "strategy": "top_down", "box": "4x4x4", "support": "floor", "feasible": True, "lifted": True},
        {"key": "b", "strategy": "top_down", "box": "4x4x4", "support": "floor", "feasible": True, "lifted": False},
        {"key": "c", "strategy": "top_down", "box": "4x4x4", "support": "floor", "feasible": False, "lifted": False},
    ]
    table = summary_table(results)
    assert "top_down" in table and "1/2" in table


def test_matrix_command_replays_feasible_plans_and_writes_results(tmp_path: Path) -> None:
    (tmp_path / "plan.json").write_text(json.dumps(planner_plan()))
    index = MatrixIndex(
        base_height_m=0.15,
        floor_z_m=-0.15,
        tool_offset_m=(0.0010, -0.0056, -0.0014),
        entries=[entry(), entry(key="skipped", feasible=False, reasons=["unreachable"])],
    )
    (tmp_path / "index.json").write_text(index.model_dump_json())
    out = tmp_path / "results.json"
    assert main(["matrix", str(tmp_path), "--out", str(out), "--workers", "1"]) == 0
    results = {r["key"]: r for r in json.loads(out.read_text())}
    assert results["skipped"]["feasible"] is False and results["skipped"]["lifted"] is False
    replayed = results["4x4x4_r20_ledge-0.08_top_down"]
    assert replayed["feasible"] is True
    assert replayed["lifted"] is False
    assert "arm_contact" in replayed and "reasons" in replayed
