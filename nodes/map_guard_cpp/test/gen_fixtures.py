"""Golden fixtures of map_guard_cpp: run the Python map_guard functions on the inputs of nodes/map_guard/tests.

The gtests of map_guard_cpp read test/fixtures/*.json and assert that the C++ port gives the same outputs: agreement
ratios and counts, detector firings per step, guard state timelines with their service calls, events and logs, backup
names and pruning, agent events and config verdicts. The inputs come from the Python tests' own helpers (room_grid,
room_scan, tilt_quaternion, the guard Harness), so a fixture is the Python behaviour on exactly the tested inputs.

Run from nodes/map_guard (its uv environment has numpy, pydantic and the map_guard package):
    uv run --frozen python ../map_guard_cpp/test/gen_fixtures.py          # write the fixtures
    uv run --frozen python ../map_guard_cpp/test/gen_fixtures.py --check  # exit 1 when a fixture is stale
"""

import argparse
import json
import math
import sys
import tempfile
from collections.abc import Callable
from datetime import UTC, datetime, timedelta, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from prometheus_client import REGISTRY
from pydantic import ValidationError

FIXTURE_DIR = Path(__file__).resolve().parent / "fixtures"
PYTHON_NODE_DIR = Path(__file__).resolve().parents[2] / "map_guard"
BACKUP_DIR_TOKEN = "@BACKUP_DIR@"
DELETED_PREFIX = "map_guard: deleted old backup "
sys.path.insert(0, str(PYTHON_NODE_DIR))

from map_guard import agent as py_agent  # noqa: E402
from map_guard.agreement import (  # noqa: E402
    UNKNOWN,
    GridMap,
    MismatchMonitor,
    agreement,
    dilate,
    prepare_map,
    scan_endpoints,
    transform_points,
    yaw_quaternion,
)
from map_guard.backup import backup_name, prune_backups  # noqa: E402
from map_guard.config import MapGuardConfig  # noqa: E402
from map_guard.detectors import (  # noqa: E402
    Disturbances,
    angle_diff_deg,
    odom_speed,
    roll_pitch_deg,
    world_vector,
)
from map_guard.guard import State  # noqa: E402
from map_guard.timing import HoldTimer, RateLimiter  # noqa: E402
from tests.test_agreement import HALF_ROOM_M, RESOLUTION, ROBOT, room_grid, room_scan  # noqa: E402
from tests.test_detectors import DT, GRAVITY, LEVEL, STILL, tilt_quaternion  # noqa: E402
from tests.test_guard import BAD, FIXED_UTC, HEALTHY, FakeSlam, Harness  # noqa: E402

SCAN_RANGE_MIN = 0.15
SCAN_RANGE_MAX = 12.0
MAX_RANGE_M = 8.0
OCCUPIED_THRESHOLD = 65
HIT_RADIUS = 1
MIN_POINTS = 40


# ---------------------------------------------------------------------------
# Encoding helpers
# ---------------------------------------------------------------------------


def num(value: float) -> float | str:
    """JSON-safe float: non-finite values become "inf", "-inf" or "nan".

    Args:
        value (float): Number.

    Returns:
        float | str: The number, or its name when not finite.
    """
    value = float(value)
    if math.isnan(value):
        return "nan"
    if math.isinf(value):
        return "inf" if value > 0 else "-inf"
    return value


def rle(cells: np.ndarray) -> list[list[int]]:
    """Run-length encode a 2D int array in row-major order.

    Args:
        cells (np.ndarray): Array.

    Returns:
        list[list[int]]: [value, count] runs.
    """
    out: list[list[int]] = []
    for value in cells.reshape(-1).tolist():
        if out and out[-1][0] == int(value):
            out[-1][1] += 1
        else:
            out.append([int(value), 1])
    return out


def mask_rows(mask: np.ndarray) -> list[str]:
    """A boolean mask as rows of "0"/"1".

    Args:
        mask (np.ndarray): 2D boolean mask.

    Returns:
        list[str]: One string per row.
    """
    return ["".join("1" if v else "0" for v in row) for row in mask.tolist()]


def grid_json(grid: GridMap) -> dict[str, Any]:
    """An occupancy grid as JSON.

    Args:
        grid (GridMap): Grid.

    Returns:
        dict[str, Any]: resolution, origin, size and RLE cells.
    """
    height, width = grid.cells.shape
    return {
        "resolution": grid.resolution,
        "origin_x": grid.origin_x,
        "origin_y": grid.origin_y,
        "origin_yaw": grid.origin_yaw,
        "width": width,
        "height": height,
        "cells_rle": rle(grid.cells),
    }


def result_json(result: Any) -> dict[str, Any] | None:
    """An AgreementResult as JSON.

    Args:
        result (Any): AgreementResult or None.

    Returns:
        dict[str, Any] | None: ratio, evaluated, agreeing.
    """
    if result is None:
        return None
    return {"ratio": result.ratio, "evaluated": result.evaluated, "agreeing": result.agreeing}


# ---------------------------------------------------------------------------
# agreement.json (test_agreement.py)
# ---------------------------------------------------------------------------


def score_case(
    name: str,
    tf: tuple[float, float, float],
    grid: GridMap,
    ranges: np.ndarray | None = None,
    scan_pose: tuple[float, float, float] = ROBOT,
) -> dict[str, Any]:
    """One scored scan: inputs, Python endpoints in the map frame and the agreement result.

    Args:
        name (str): Case name (the Python test it mirrors).
        tf (tuple[float, float, float]): x, y, yaw of the transform the scan is projected with.
        grid (GridMap): Map.
        ranges (np.ndarray | None): Ranges; the room scan at scan_pose when None.
        scan_pose (tuple[float, float, float]): Pose the room scan is taken from.

    Returns:
        dict[str, Any]: The case.
    """
    if ranges is None:
        ranges, angle_min, inc = room_scan(*scan_pose)
    else:
        angle_min, inc = -math.pi, 2 * math.pi / len(ranges)
    points = scan_endpoints(ranges, angle_min, inc, SCAN_RANGE_MIN, SCAN_RANGE_MAX, MAX_RANGE_M)
    rotation = yaw_quaternion(tf[2])
    in_map = transform_points(points, (tf[0], tf[1], 0.0), rotation)
    prepared = prepare_map(grid, OCCUPIED_THRESHOLD, HIT_RADIUS)
    return {
        "name": name,
        "grid": grid_json(grid),
        "scan": {
            "ranges": [num(r) for r in ranges],
            "angle_min": angle_min,
            "angle_increment": inc,
            "range_min": SCAN_RANGE_MIN,
            "range_max": SCAN_RANGE_MAX,
            "max_range_m": MAX_RANGE_M,
        },
        "translation": [tf[0], tf[1], 0.0],
        "rotation": list(rotation),
        "occupied_threshold": OCCUPIED_THRESHOLD,
        "hit_radius_cells": HIT_RADIUS,
        "min_points": MIN_POINTS,
        "points_map": in_map.tolist(),
        "known_cells": prepared.known_cells,
        "result": result_json(agreement(in_map, prepared, MIN_POINTS)),
    }


def monitor_scenarios() -> list[dict[str, Any]]:
    """MismatchMonitor update sequences of test_agreement.py and their outputs.

    Returns:
        list[dict[str, Any]]: Scenarios with updates [t, ratio, known] or "reset", outputs and final state.
    """

    def feed(start: float, end: float, ratio: float, known: int = 5000) -> list[list[float]]:
        out = []
        t = start
        while t < end - 1e-9:
            out.append([t, ratio, known])
            t += 0.5
        return out

    alternating = [[5.0 + i * 0.5, 0.1 if i % 3 == 0 else 0.8, 5000] for i in range(40)]
    scenarios = {
        "fresh_map_never_triggers": feed(0.0, 30.0, 0.1),
        "small_map_never_triggers": feed(0.0, 5.0, 0.9, 500) + feed(5.0, 30.0, 0.1, 500),
        "mismatch_after_healthy_triggers_after_mismatch_s": feed(0.0, 5.0, 0.9) + feed(5.0, 11.5, 0.1),
        "single_low_samples_do_not_trigger": feed(0.0, 5.0, 0.9) + alternating,
        "reset_disarms": feed(0.0, 5.0, 0.9) + ["reset"] + feed(5.0, 30.0, 0.1),
        "median_tracks_window_0_9": feed(0.0, 5.0, 0.9),
        "median_tracks_window_0_45": feed(0.0, 5.0, 0.45),
        "even_window_median": [[0.0, 0.2, 5000], [0.5, 0.9, 5000]],
    }
    out = []
    for name, updates in scenarios.items():
        mon = MismatchMonitor(mismatch_ratio=0.3, healthy_ratio=0.6, mismatch_s=5.0, min_map_cells_known=1000, window=5)
        fired = []
        for update in updates:
            if update == "reset":
                mon.reset()
                fired.append(None)
                continue
            fired.append(mon.update(update[0], update[1], int(update[2])))
        out.append(
            {
                "name": name,
                "updates": updates,
                "fired": fired,
                "armed": mon.armed,
                "median": mon.median,
            }
        )
    return out


def agreement_fixture() -> dict[str, Any]:
    """Every agreement input of test_agreement.py with the Python outputs.

    Returns:
        dict[str, Any]: The fixture.
    """
    room = room_grid()
    empty = GridMap(room.resolution, room.origin_x, room.origin_y, 0.0, np.full_like(room.cells, UNKNOWN))
    rotated = GridMap(room.resolution, 5.0, -5.0, math.pi / 2, room.cells)
    few = room_scan(*ROBOT)[0]
    few[30:] = math.inf

    single = np.zeros((5, 5), dtype=bool)
    single[2, 2] = True
    rng = np.random.default_rng(7)
    random_mask = rng.random((7, 9)) < 0.15
    dilate_cases = []
    for mask_name, mask in (("single", single), ("random", random_mask)):
        for radius in (0, 1, 2):
            dilate_cases.append(
                {
                    "name": f"{mask_name}_r{radius}",
                    "mask": mask_rows(mask),
                    "radius": radius,
                    "out": mask_rows(dilate(mask, radius)),
                }
            )

    endpoint_cases = []
    for name, args in (
        ("filter_ranges", ([1.0, math.inf, math.nan, 0.05, 9.0, 2.0], 0.0, math.pi / 2, 0.15, 12.0, 8.0)),
        ("range_max_below_max_range", ([5.0, 3.0], 0.0, 0.1, 0.15, 4.0, 8.0)),
        ("negative_inf_and_bounds", ([-math.inf, 0.15, 8.0, 8.0000001, 12.0], -1.0, 0.25, 0.15, 12.0, 8.0)),
    ):
        ranges = np.array(args[0])
        endpoint_cases.append(
            {
                "name": name,
                "ranges": [num(r) for r in ranges],
                "angle_min": args[1],
                "angle_increment": args[2],
                "range_min": args[3],
                "range_max": args[4],
                "max_range_m": args[5],
                "points": scan_endpoints(ranges, *args[1:]).tolist(),
            }
        )

    transform_cases = []
    for name, points, translation, rotation in (
        ("rotates_then_translates", [[1.0, 0.0]], (2.0, 3.0, 0.0), yaw_quaternion(math.pi / 2)),
        ("upside_down_mount", [[1.0, 1.0]], (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
        ("general", [[0.3, -1.2], [2.0, 0.5]], (0.4, -0.7, 0.2), (0.1, -0.2, 0.3, 0.927361849549570)),
    ):
        transform_cases.append(
            {
                "name": name,
                "points": points,
                "translation": list(translation),
                "rotation": list(rotation),
                "out": transform_points(np.array(points), translation, rotation).tolist(),
            }
        )

    scores = [
        score_case("perfect_alignment_scores_high", ROBOT, room),
        score_case("shifted_transform_scores_low", (ROBOT[0] + 0.4, ROBOT[1] + 0.4, ROBOT[2]), room),
        score_case("rotated_transform_scores_low", (ROBOT[0], ROBOT[1], ROBOT[2] + 0.35), room),
        score_case("hit_radius_one_cell", (ROBOT[0] + RESOLUTION * 0.9, ROBOT[1], ROBOT[2]), room),
        score_case("hit_radius_three_cells", (ROBOT[0] + RESOLUTION * 3, ROBOT[1] + RESOLUTION * 3, ROBOT[2]), room),
        score_case("empty_map_is_skipped", ROBOT, empty),
        score_case("few_points_are_skipped", ROBOT, room, ranges=few),
        score_case("rotated_map_origin", ROBOT, rotated),
    ]
    far = np.array([[100.0, 100.0], [4.5, 4.5]] * 30)
    prepared_room = prepare_map(room, OCCUPIED_THRESHOLD, HIT_RADIUS)
    return {
        "half_room_m": HALF_ROOM_M,
        "room": grid_json(room),
        "room_known_cells": prepared_room.known_cells,
        "dilate": dilate_cases,
        "scan_endpoints": endpoint_cases,
        "transform": transform_cases,
        "yaw_quaternion": [{"yaw": yaw, "q": list(yaw_quaternion(yaw))} for yaw in (0.0, 0.3, math.pi / 2, -2.5)],
        "scores": scores,
        "outside_points": {"points_map": far.tolist(), "result": result_json(agreement(far, prepared_room, 40))},
        "monitor": monitor_scenarios(),
    }


# ---------------------------------------------------------------------------
# detectors.json (test_detectors.py)
# ---------------------------------------------------------------------------


def segment_inputs(segments: list[dict[str, Any]]) -> Callable[[float], dict]:
    """The inputs(t) function of a segment list: the first segment whose "until" is above t (None: always).

    Args:
        segments (list[dict[str, Any]]): [{"until": float | None, "inputs": {...}}].

    Returns:
        Callable[[float], dict]: inputs(t) as test_detectors.run expects it (tuples for vectors).
    """

    def inputs(t: float) -> dict:
        for segment in segments:
            if segment["until"] is None or t < segment["until"]:
                return {k: tuple(v) if isinstance(v, list) else v for k, v in segment["inputs"].items()}
        return {}

    return inputs


def report_json(report: Any) -> dict[str, Any]:
    """A DisturbanceReport as JSON.

    Args:
        report (Any): DisturbanceReport.

    Returns:
        dict[str, Any]: Sorted reasons and the numbers.
    """
    return {
        "reasons": sorted(report.reasons),
        "tilt_deg": report.tilt_deg,
        "unexplained_speed_mps": report.unexplained_speed_mps,
        "imu_yaw_rate": report.imu_yaw_rate,
    }


def detector_scenarios() -> list[dict[str, Any]]:
    """The detector scenarios of test_detectors.py as segment lists, with the Python reasons at every step.

    Returns:
        list[dict[str, Any]]: Scenarios: runs (start, end, segments), optional pre-calls, per-step reasons and the
            report of a final evaluate.
    """
    level = {"until": None, "inputs": {"lidar": 0.0}}
    start = 4.0

    def tilt(deg: float) -> list[float]:
        return list(tilt_quaternion(deg))

    specs: list[dict[str, Any]] = [
        {"name": "sustained_tilt_triggers_lifted_after_lift_s", "runs": [
            (start, start + 2.0, [{"until": None, "inputs": {"orientation": tilt(20.0), "lidar": 0.0}}])]},
        {"name": "tilt_spike_of_0_2_s_never_triggers", "runs": [(start, start + 3.0, [
            {"until": start + 0.2, "inputs": {"orientation": tilt(25.0), "lidar": 0.0}},
            {"until": None, "inputs": {"orientation": tilt(0.0), "lidar": 0.0}}])]},
        {"name": "no_tilt_trigger_without_a_level_baseline", "level": False, "runs": [
            (0.0, 0.5, [{"until": None, "inputs": {"orientation": tilt(30.0)}}])]},
        {"name": "small_tilt_below_threshold_never_triggers", "runs": [
            (start, start + 3.0, [{"until": None, "inputs": {"orientation": tilt(8.0), "lidar": 0.0}}])]},
        {"name": "baseline_follows_a_level_but_inclined_floor", "level": False, "runs": [
            (0.0, 4.0, [{"until": None, "inputs": {"orientation": tilt(5.0), "lidar": 0.0}}]),
            (4.0, 7.0, [{"until": None, "inputs": {"orientation": tilt(15.0), "lidar": 0.0}}])]},
        {"name": "rebaseline_after_a_long_steady_tilt", "runs": [
            (start, start + 40.0, [{"until": None, "inputs": {"orientation": tilt(20.0), "lidar": 0.0}}])]},
        {"name": "vertical_acceleration_then_tilt_triggers_lifted", "runs": [(start, start + 1.0, [
            {"until": start + 0.6, "inputs": {"accel": [0.0, 0.0, 3.0], "lidar": 0.0}},
            {"until": None, "inputs": {"orientation": tilt(6.0), "lidar": 0.0}}])]},
        {"name": "vertical_acceleration_then_lidar_motion_triggers_lifted", "runs": [(start, start + 0.8, [
            {"until": start + 0.6, "inputs": {"accel": [0.0, 0.0, -2.5], "lidar": 0.0}},
            {"until": None, "inputs": {"lidar": 0.15}}])]},
        {"name": "vertical_acceleration_alone_never_triggers", "runs": [
            (start, start + 3.0, [{"until": None, "inputs": {"accel": [0.0, 0.0, 3.0], "lidar": 0.0}}])]},
        {"name": "vertical_acceleration_spike_then_tilt_never_triggers", "runs": [(start, start + 0.9, [
            {"until": start + 0.2, "inputs": {"accel": [0.0, 0.0, 4.0], "lidar": 0.0}},
            {"until": None, "inputs": {"orientation": tilt(6.0), "lidar": 0.0}}])]},
        {"name": "gravity_included_fallback_samples_are_ignored", "runs": [(start, start + 1.5, [
            {"until": start + 1.0, "inputs": {"accel": [0.0, 0.0, GRAVITY], "lidar": 0.0}},
            {"until": None, "inputs": {"orientation": tilt(6.0), "lidar": 0.0}}])]},
        {"name": "sustained_unexplained_lidar_motion_triggers_carried", "runs": [
            (start, start + 1.5, [{"until": None, "inputs": {"lidar": 0.2}}])]},
        {"name": "lidar_motion_spike_never_triggers_carried", "runs": [(start, start + 3.0, [
            {"until": start + 0.2, "inputs": {"lidar": 0.3}}, {"until": None, "inputs": {"lidar": 0.0}}])]},
        {"name": "commanded_motion_suppresses_carried", "runs": [
            (start, start + 3.0, [{"until": None, "inputs": {"lidar": 0.2, "cmd": [0.2, 0.0]}}])]},
        {"name": "command_older_than_cmd_timeout_does_not_suppress", "runs": [(start, start + 2.0, [
            {"until": start + 0.1, "inputs": {"lidar": 0.2, "cmd": [0.2, 0.0]}},
            {"until": None, "inputs": {"lidar": 0.2}}])]},
        {"name": "zero_command_does_not_suppress", "runs": [
            (start, start + 2.0, [{"until": None, "inputs": {"lidar": 0.2, "cmd": [0.0, 0.0]}}])]},
        {"name": "moving_wheels_suppress_carried", "runs": [
            (start, start + 3.0, [{"until": None, "inputs": {"lidar": 0.2, "wheel": [0.18, 0.0]}}])]},
        {"name": "stale_lidar_odometry_never_triggers_carried", "lidar_at_start": 0.3, "runs": [
            (start, start + 3.0, [{"until": None, "inputs": {}}])]},
        {"name": "sustained_unexplained_yaw_rate_triggers_rotated", "runs": [
            (start, start + 1.5, [{"until": None, "inputs": {"gyro": [0.0, 0.0, 0.5], "lidar": 0.0}}])]},
        {"name": "yaw_rate_spike_never_triggers_rotated", "runs": [(start, start + 3.0, [
            {"until": start + 0.2, "inputs": {"gyro": [0.0, 0.0, 1.0]}},
            {"until": None, "inputs": {"gyro": [0.0, 0.0, 0.0]}}])]},
        {"name": "commanded_rotation_suppresses_rotated", "runs": [
            (start, start + 3.0, [{"until": None, "inputs": {"gyro": [0.0, 0.0, 0.5], "cmd": [0.0, 0.5]}}])]},
        {"name": "wheel_yaw_rate_suppresses_rotated", "runs": [
            (start, start + 3.0, [{"until": None, "inputs": {"gyro": [0.0, 0.0, 0.5], "wheel": [0.0, 0.45]}}])]},
        {"name": "quiet_robot_reports_nothing", "level": False, "runs": [
            (0.0, 6.0, [{"until": None, "inputs": {"lidar": 0.01}}])]},
    ]  # fmt: skip
    out = []
    for spec in specs:
        det = Disturbances(MapGuardConfig())
        runs = ([(0.0, start, [level])] if spec.get("level", True) else []) + spec["runs"]
        steps: list[list[str]] = []
        for index, (run_start, run_end, segments) in enumerate(runs):
            if index == len(runs) - len(spec["runs"]) and "lidar_at_start" in spec:
                det.on_lidar_odom(run_start, spec["lidar_at_start"])
            inputs = segment_inputs(segments)
            for i in range(int(round((run_end - run_start) / DT))):
                t = run_start + i * DT
                values = inputs(t)
                det.on_imu(t, values.get("orientation", LEVEL), values.get("gyro", STILL), values.get("accel", STILL))
                if "lidar" in values:
                    det.on_lidar_odom(t, values["lidar"])
                det.on_wheel_odom(t, *values.get("wheel", (0.0, 0.0)))
                if "cmd" in values:
                    det.on_cmd(t, *values["cmd"])
                steps.append([t, ",".join(sorted(det.evaluate(t).reasons))])
        final_t = runs[-1][1]
        out.append(
            {
                "name": spec["name"],
                "runs": [{"start": s, "end": e, "segments": seg} for s, e, seg in runs],
                "lidar_at_start": spec.get("lidar_at_start"),
                "lidar_at_run": len(runs) - len(spec["runs"]),
                "steps": steps,
                "final_t": final_t,
                "final": report_json(det.evaluate(final_t)),
                "baseline": None if det.tilt_tracker.baseline is None else list(det.tilt_tracker.baseline),
            }
        )
    return out


def detectors_fixture() -> dict[str, Any]:
    """Helper outputs and detector scenarios of test_detectors.py.

    Returns:
        dict[str, Any]: The fixture.
    """
    quaternions = [list(tilt_quaternion(20.0)), list(tilt_quaternion(-35.0)), [0.1, -0.2, 0.3, 0.927361849549570]]
    quaternions.append([0.0, 0.7071067811865476, 0.0, 0.7071067811865476])  # pitch 90: asin clamp
    return {
        "dt": DT,
        "gravity": GRAVITY,
        "roll_pitch": [{"q": q, "out": list(roll_pitch_deg(tuple(q)))} for q in quaternions],
        "angle_diff": [
            {"a": a, "b": b, "out": angle_diff_deg(a, b)}
            for a, b in ((179.0, -179.0), (10.0, 0.0), (-170.0, 170.0), (180.0, 0.0), (-180.0, 0.0), (540.5, 0.25))
        ],
        "world_vector": [
            {"q": q, "v": v, "out": list(world_vector(tuple(q), tuple(v)))}
            for q, v in (
                (list(tilt_quaternion(90.0)), [0.0, 1.0, 0.0]),
                ([0.1, -0.2, 0.3, 0.927361849549570], [0.5, -1.0, 2.0]),
            )
        ],
        "odom_speed": [
            {
                "twist": list(tw),
                "previous": None if prev is None else list(prev),
                "current": list(cur),
                "out": odom_speed(tw, prev, cur),
            }
            for tw, prev, cur in (
                ((0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.05, 0.5)),
                ((0.3, 0.0), (0.0, 0.0, 0.0), (0.0, 0.05, 0.5)),
                ((0.0, 0.0), None, (0.0, 0.05, 0.5)),
                ((0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 1.0)),
                ((0.1, 0.1), (0.0, 0.0, 2.0), (0.3, 0.4, 1.0)),
            )
        ],  # fmt: skip
        "scenarios": detector_scenarios(),
    }


# ---------------------------------------------------------------------------
# guard.json (test_guard.py)
# ---------------------------------------------------------------------------


def sample_value(registry_name: str, labels: dict[str, str] | None = None) -> float:
    """Current value of a default-registry sample (0 when absent).

    Args:
        registry_name (str): Sample name.
        labels (dict[str, str] | None): Labels.

    Returns:
        float: Value.
    """
    return REGISTRY.get_sample_value(registry_name, labels or {}) or 0.0


COUNTERS = [
    ("map_guard_pauses_total", {}),
    ("map_guard_resumes_total", {}),
    ("map_guard_resets_total", {"dry_run": "true"}),
    ("map_guard_resets_total", {"dry_run": "false"}),
] + [("map_guard_triggers_total", {"reason": r}) for r in ("mismatch", "lifted", "carried", "rotated")]


class NoParamSlam(FakeSlam):
    """slam_toolbox whose parameter cannot be read (test_unknown_parameter_state_falls_back_to_tracking)."""

    def query_paused(self) -> bool | None:
        self.calls.append("query")
        return None


def tokenize(value: Any, backup_dir: str) -> Any:
    """Replace the scenario's temporary backup directory with BACKUP_DIR_TOKEN, recursively.

    Args:
        value (Any): String, list, tuple or dict.
        backup_dir (str): Temporary directory.

    Returns:
        Any: The value with every occurrence replaced.
    """
    if isinstance(value, str):
        return value.replace(backup_dir, BACKUP_DIR_TOKEN)
    if isinstance(value, (list, tuple)):
        return [tokenize(v, backup_dir) for v in value]
    if isinstance(value, dict):
        return {k: tokenize(v, backup_dir) for k, v in value.items()}
    return value


def sort_deleted_runs(logs: list[str]) -> list[str]:
    """Sort each run of consecutive "deleted old backup" lines (their order is the directory listing order).

    Args:
        logs (list[str]): Log lines.

    Returns:
        list[str]: The lines, each run of deleted-backup lines sorted by name.
    """
    out: list[str] = []
    run: list[str] = []
    for line in logs + [""]:
        if line.startswith(DELETED_PREFIX):
            run.append(line)
            continue
        out += sorted(run)
        run = []
        out.append(line)
    return out[:-1]


def run_guard_scenario(spec: dict[str, Any]) -> dict[str, Any]:
    """Run one guard scenario through the Python Harness of test_guard.py and record everything observable.

    Args:
        spec (dict[str, Any]): name, overrides, slam options, existing backups and steps.

    Returns:
        dict[str, Any]: The spec plus the per-tick state runs, slam calls, events, logs, publishes, metric deltas,
            final paused flag and the backup stems left on disk.
    """
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        for stem in spec.get("existing_backups", []):
            (tmp_path / f"{stem}.posegraph").write_text("g")
            (tmp_path / f"{stem}.data").write_text("d")
        slam_spec = spec.get("slam", {})
        slam_cls = NoParamSlam if slam_spec.get("query_none") else FakeSlam
        slam = slam_cls(
            paused=slam_spec.get("paused", False),
            toggle_ok=slam_spec.get("toggle_ok", True),
            toggle_works=slam_spec.get("toggle_works", True),
        )
        slam.serialize_ok = slam_spec.get("serialize_ok", True)
        slam.reset_ok = slam_spec.get("reset_ok", True)
        before = [sample_value(name, labels) for name, labels in COUNTERS]
        h = Harness(tmp_path, slam=slam, **spec.get("overrides", {}))
        states: list[list[Any]] = []
        ticks = 0

        def record() -> None:
            state = h.guard.state.value
            if states and states[-1][0] == state:
                states[-1][1] += 1
            else:
                states.append([state, 1])

        for step in spec["steps"]:
            answer = step.get("answer")
            if answer:
                # A map_manager result, applied before the step's ticks: request "last" answers the newest request,
                # "first" the oldest, any other text is a foreign request id.
                requests = h.relocalise_requests
                ids = {"last": requests[-1][0] if requests else "", "first": requests[0][0] if requests else ""}
                request_id = ids.get(answer.get("request", "last"), answer.get("request"))
                h.guard.on_relocalise_result(request_id, answer["ok"], answer["state"], answer["discarded_pois"])
            sample = step.get("sample")
            for _ in range(int(round(step["seconds"] / 0.1))):
                h.run(0.1, reasons=step.get("reasons", []), sample=tuple(sample) if sample else None)
                ticks += 1
                record()
        after = [sample_value(name, labels) for name, labels in COUNTERS]
        counters = [
            {"name": name, "labels": labels, "delta": a - b}
            for (name, labels), a, b in zip(COUNTERS, after, before, strict=True)
        ]
        result = {
            **spec,
            "ticks": ticks,
            "states": states,
            "slam_calls": tokenize([list(c) if isinstance(c, tuple) else c for c in slam.calls], tmp),
            "events": tokenize([list(e) for e in h.notifier.events], tmp),
            "logs": sort_deleted_runs(tokenize(h.logs, tmp)),
            "resets_published": h.resets_published,
            "relocalise_requests": [list(r) for r in h.relocalise_requests],
            "paused": h.guard.paused,
            "slam_paused": slam.paused,
            "counters": counters,
            "state_gauge": {s.value: sample_value("map_guard_state", {"state": s.value}) for s in State},
            "dry_run_gauge": sample_value("map_guard_dry_run"),
            "backup_stems": sorted({p.name.split(".", 1)[0] for p in tmp_path.iterdir()}),
        }
    return result


def guard_fixture() -> dict[str, Any]:
    """Every scenario of test_guard.py with the Python state timeline and side effects.

    Returns:
        dict[str, Any]: The fixture.
    """
    healthy, bad = list(HEALTHY), list(BAD)
    lifted, carried = ["lifted"], ["carried"]
    fail = {"ok": False, "state": "not_localised", "discarded_pois": 0}
    found = {"ok": True, "state": "localised", "discarded_pois": 0}
    answer_fail = {"seconds": 0.5, "sample": bad, "answer": fail}
    specs: list[dict[str, Any]] = [
        {"name": "starts_ok", "steps": [{"seconds": 1.0, "sample": healthy}]},
        {"name": "lifted_pauses_settles_and_resumes", "steps": [
            {"seconds": 1.0, "sample": healthy}, {"seconds": 0.1, "reasons": lifted, "sample": healthy},
            {"seconds": 2.0, "reasons": lifted, "sample": healthy}, {"seconds": 0.1, "sample": healthy},
            {"seconds": 2.8, "sample": healthy}, {"seconds": 0.4, "sample": healthy},
            {"seconds": 4.1, "sample": healthy}, {"seconds": 0.2, "sample": healthy}]},
        {"name": "new_trigger_while_settling_goes_back_to_disturbed", "steps": [
            {"seconds": 0.1, "reasons": lifted}, {"seconds": 1.0}, {"seconds": 0.1, "reasons": carried}]},
        {"name": "trigger_while_evaluating_goes_back_to_disturbed", "steps": [
            {"seconds": 0.1, "reasons": lifted}, {"seconds": 3.2, "sample": healthy},
            {"seconds": 0.1, "reasons": carried, "sample": healthy}]},
        {"name": "low_agreement_after_settling_backs_up_and_resets", "steps": [
            {"seconds": 0.1, "reasons": carried}, {"seconds": 3.2, "sample": bad}, {"seconds": 4.2, "sample": bad},
            answer_fail, {"seconds": 0.2, "sample": bad}]},
        {"name": "reset_prunes_old_backups", "overrides": {"keep_backups": 3},
         "existing_backups": [f"map_guard_backup_2026010{i}T000000Z" for i in range(6)],
         "steps": [{"seconds": 0.1, "reasons": carried}, {"seconds": 8.0, "sample": bad}, answer_fail]},
        {"name": "failed_backup_skips_the_reset", "slam": {"serialize_ok": False},
         "steps": [{"seconds": 0.1, "reasons": carried}, {"seconds": 8.0, "sample": bad}, answer_fail]},
        {"name": "failed_reset_does_not_publish", "slam": {"reset_ok": False},
         "steps": [{"seconds": 0.1, "reasons": carried}, {"seconds": 8.0, "sample": bad}, answer_fail]},
        {"name": "no_agreement_samples_during_evaluation_resumes",
         "steps": [{"seconds": 0.1, "reasons": lifted}, {"seconds": 8.0}]},
        {"name": "mismatch_alone_pauses_and_evaluates_without_settling", "steps": [
            {"seconds": 5.0, "sample": healthy}, {"seconds": 7.5, "sample": bad}, {"seconds": 4.5, "sample": bad},
            answer_fail]},
        {"name": "cooldown_blocks_mismatch_retrigger", "overrides": {"cooldown_s": 30.0}, "steps": [
            {"seconds": 5.0, "sample": healthy}, {"seconds": 12.5, "sample": bad}, answer_fail,
            {"seconds": 4.0, "sample": healthy}, {"seconds": 10.0, "sample": bad}, {"seconds": 25.0, "sample": bad},
            answer_fail]},
        {"name": "cooldown_does_not_block_physical_triggers", "steps": [
            {"seconds": 0.1, "reasons": carried}, {"seconds": 8.0, "sample": bad}, answer_fail,
            {"seconds": 0.1, "reasons": lifted}]},
        {"name": "disabled_reasons_are_ignored", "overrides": {"reasons": ["mismatch"]},
         "steps": [{"seconds": 3.0, "reasons": ["lifted", "carried"]}]},
        {"name": "disabled_mismatch_is_ignored", "overrides": {"reasons": ["lifted"]},
         "steps": [{"seconds": 5.0, "sample": healthy}, {"seconds": 15.0, "sample": bad}]},
        {"name": "already_paused_is_not_toggled", "slam": {"paused": True},
         "steps": [{"seconds": 0.1, "reasons": lifted}]},
        {"name": "failed_toggle_keeps_the_tracked_state", "slam": {"toggle_ok": False},
         "steps": [{"seconds": 0.1, "reasons": lifted}]},
        {"name": "toggle_that_did_not_take_effect_is_detected", "slam": {"toggle_works": False},
         "steps": [{"seconds": 0.1, "reasons": lifted}]},
        {"name": "unknown_parameter_state_falls_back_to_tracking", "slam": {"query_none": True}, "steps": [
            {"seconds": 0.1, "reasons": lifted}, {"seconds": 0.1, "reasons": carried}, {"seconds": 1.0},
            {"seconds": 0.1, "reasons": lifted}, {"seconds": 8.0, "sample": healthy}]},
        {"name": "dry_run_never_calls_services_or_publishes", "overrides": {"dry_run": True},
         "steps": [{"seconds": 0.1, "reasons": carried}, {"seconds": 8.0, "sample": bad}]},
        {"name": "dry_run_mismatch_reset", "overrides": {"dry_run": True}, "steps": [
            {"seconds": 5.0, "sample": healthy}, {"seconds": 12.5, "sample": bad}]},
        {"name": "rotated_and_mismatch_episode", "steps": [
            {"seconds": 5.0, "sample": healthy}, {"seconds": 0.3, "reasons": ["rotated"], "sample": bad},
            {"seconds": 0.2, "reasons": ["rotated", "lifted"], "sample": bad}, {"seconds": 8.0, "sample": bad},
            answer_fail]},
        {"name": "poor_median_requests_relocalise_and_stays_paused", "steps": [
            {"seconds": 0.1, "reasons": carried}, {"seconds": 8.0, "sample": bad}, {"seconds": 5.0, "sample": bad}]},
        {"name": "localised_result_resumes_without_deleting", "steps": [
            {"seconds": 0.1, "reasons": carried}, {"seconds": 8.0, "sample": bad},
            {"seconds": 0.5, "sample": healthy, "answer": found}, {"seconds": 0.5, "sample": healthy}]},
        {"name": "failed_result_resets_once_with_discarded_pois", "steps": [
            {"seconds": 0.1, "reasons": carried}, {"seconds": 8.0, "sample": bad},
            {"seconds": 0.5, "sample": bad, "answer": {"ok": False, "state": "not_localised", "discarded_pois": 7}},
            {"seconds": 0.5, "sample": bad},
            {"seconds": 0.1, "sample": bad, "answer": {"ok": False, "state": "not_localised", "discarded_pois": 7}}]},
        {"name": "ok_result_with_other_state_is_unrecoverable", "steps": [
            {"seconds": 0.1, "reasons": carried}, {"seconds": 8.0, "sample": bad},
            {"seconds": 0.5, "sample": bad, "answer": {"ok": True, "state": "ambiguous", "discarded_pois": 0}}]},
        {"name": "timeout_is_unrecoverable", "overrides": {"relocalise_timeout_s": 10.0}, "steps": [
            {"seconds": 0.1, "reasons": carried}, {"seconds": 8.0, "sample": bad}, {"seconds": 9.0, "sample": bad},
            {"seconds": 1.5, "sample": bad}]},
        {"name": "foreign_request_id_is_ignored", "steps": [
            {"seconds": 0.1, "reasons": carried}, {"seconds": 8.0, "sample": bad},
            {"seconds": 0.5, "sample": bad, "answer": {**found, "request": "someone-else-1"}},
            {"seconds": 0.5, "sample": bad,
             "answer": {"ok": False, "state": "not_localised", "discarded_pois": 3, "request": "someone-else-2"}}]},
        {"name": "result_without_pending_request_is_ignored", "steps": [
            {"seconds": 0.5, "sample": healthy, "answer": {**fail, "request": "map_guard-1"}}]},
        {"name": "new_request_id_per_episode", "overrides": {"cooldown_s": 1.0}, "steps": [
            {"seconds": 0.1, "reasons": carried}, {"seconds": 8.0, "sample": bad}, answer_fail,
            {"seconds": 2.0, "sample": healthy}, {"seconds": 0.1, "reasons": carried},
            {"seconds": 8.0, "sample": bad}]},
        {"name": "dry_run_requests_nothing_and_resets_nothing", "overrides": {"dry_run": True, "relocalise_timeout_s": 5.0},
         "steps": [{"seconds": 0.1, "reasons": carried}, {"seconds": 8.0, "sample": bad}, {"seconds": 6.0, "sample": bad}]},
    ]  # fmt: skip
    report_gauges = None
    with tempfile.TemporaryDirectory() as tmp:
        from map_guard.detectors import DisturbanceReport

        h = Harness(Path(tmp))
        h.guard.tick(0.0, DisturbanceReport(frozenset(), 3.5, 0.04, 0.0), [(0.77, 5000)])
        report_gauges = {
            "tilt_deg": sample_value("map_guard_tilt_deg"),
            "unexplained_speed_mps": sample_value("map_guard_unexplained_speed_mps"),
            "agreement_ratio": sample_value("map_guard_agreement_ratio"),
        }
    return {
        "tick_s": 0.1,
        "fixed_utc_epoch_s": FIXED_UTC.timestamp(),
        "backup_dir_token": BACKUP_DIR_TOKEN,
        "scenarios": [run_guard_scenario(spec) for spec in specs],
        "report_numbers_reach_the_gauges": report_gauges,
    }


# ---------------------------------------------------------------------------
# timing.json, backup.json, agent.json, config.json
# ---------------------------------------------------------------------------


def timing_fixture() -> dict[str, Any]:
    """HoldTimer and RateLimiter sequences of test_timing.py.

    Returns:
        dict[str, Any]: The fixture: hold sequences ([t, condition] or "reset") and limiter calls with outputs.
    """
    holds = {
        "needs_the_full_duration": (1.0, [[0.0, True], [0.5, True], [0.99, True], [1.0, True], [3.0, True]]),
        "resets_on_a_false_sample": (1.0, [[0.0, True], [0.8, False], [1.2, True], [2.2, True]]),
        "short_spike_never_fires": (0.4, [[t / 10, 1.0 <= t / 10 < 1.2] for t in range(40)]),
        "reset": (0.5, [[0.0, True], "reset", [0.6, True]]),
        "float_tolerance": (0.3, [[0.1, True], [0.2, True], [0.4, True]]),
    }
    hold_out = []
    for name, (duration, seq) in holds.items():
        timer = HoldTimer(duration)
        outs = []
        for item in seq:
            if item == "reset":
                timer.reset()
                outs.append(None)
            else:
                outs.append(timer.update(item[0], item[1]))
        hold_out.append({"name": name, "duration_s": duration, "sequence": seq, "out": outs})
    limiters = []
    for rate, times in ((2.0, [0.0, 0.3, 0.5, 0.9, 1.01]), (3.0, [0.0, 1 / 3, 0.5, 2 / 3, 1.0, 1.3])):
        limiter = RateLimiter(rate)
        limiters.append({"rate_hz": rate, "times": times, "out": [limiter.ready(t) for t in times]})
    return {"hold": hold_out, "rate_limiter": limiters}


def backup_fixture() -> dict[str, Any]:
    """backup_name and prune_backups cases of test_backup.py.

    Returns:
        dict[str, Any]: The fixture.
    """
    names = []
    for when in (
        datetime(2026, 10, 10, 7, 5, 3, tzinfo=UTC),
        datetime(2026, 10, 10, 9, 0, 0, tzinfo=timezone(timedelta(hours=2))),
        datetime(1999, 12, 31, 23, 59, 59, 999000, tzinfo=UTC),
    ):
        names.append(
            {"prefix": "map_guard_backup", "epoch_s": when.timestamp(), "name": backup_name("map_guard_backup", when)}
        )
    base = datetime(2026, 1, 1, tzinfo=UTC)
    prune_cases = []
    for name, count, extra, keep in (
        ("keeps_the_newest", 8, ["slam_map.posegraph", "slam_map.data"], 5),
        ("fewer_backups_than_keep", 2, [], 5),
        ("ignores_other_prefixes", 6, ["map_guard_backup_old_manual.txt", "other_20260101T000000Z.data"], 5),
        ("keep_one", 3, ["map_guard_backup_20260101T000000Z.extra.tar"], 1),
    ):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            files = []
            for i in range(count):
                stem = backup_name("map_guard_backup", base + timedelta(minutes=i))
                files += [f"{stem}.posegraph", f"{stem}.data"]
            files += extra
            for f in files:
                (directory / f).write_text("x")
            deleted = sorted(p.name for p in prune_backups(directory, "map_guard_backup", keep))
            remaining = sorted(p.name for p in directory.iterdir())
        prune_cases.append({"name": name, "files": files, "keep": keep, "deleted": deleted, "remaining": remaining})
    return {"names": names, "prune": prune_cases}


def agent_fixture() -> dict[str, Any]:
    """fetch_busy bodies, event severities and built events of test_agent.py.

    Returns:
        dict[str, Any]: The fixture.
    """

    class Body:
        def __init__(self, raw: bytes) -> None:
            self.raw = raw

        def __enter__(self) -> "Body":
            return self

        def __exit__(self, *_exc: object) -> None:
            return None

        def read(self) -> bytes:
            return self.raw

    bodies = [
        '{"busy": true, "session": "x"}', '{"busy": false}', '{"state": "idle"}', '{"busy": "yes"}', "[1, 2]",
        "not json", '{"busy": 1}', '{"busy": true}', "", "null", '{"busy": null}',
    ]  # fmt: skip
    busy = [
        {"body": b, "busy": py_agent.fetch_busy("u", 0.5, lambda _u, timeout, b=b: Body(b.encode()))} for b in bodies
    ]
    severities = []
    for dry_run in (False, True):
        for kind in (py_agent.EVENT_DISTURBED, py_agent.EVENT_RESUMED, py_agent.EVENT_RESET):
            for reason in ("mismatch", "lifted", "carried", "rotated"):
                severities.append(
                    {"kind": kind, "reason": reason, "dry_run": dry_run,
                     "severity": py_agent.event_severity(kind, reason, dry_run)}
                )  # fmt: skip
    events = []
    for seq, ts, kind, reason, ratio, dry_run, data in (
        (7, 1700000000.5, py_agent.EVENT_RESET, "carried", 0.123456, False, {"backup": "/x"}),
        (1, 1.0, py_agent.EVENT_DISTURBED, "lifted", None, False, None),
        (2, 12.25, py_agent.EVENT_RESUMED, "rotated", 0.9, True, None),
        (3, 99.0, py_agent.EVENT_RESET, "mismatch", 0.2, True, {"backup": "/var/lib/ros2/maps/b"}),
        (4, 5.5, py_agent.EVENT_DISTURBED, "mismatch", 0.0049, False, None),
        (5, 5.5, py_agent.EVENT_DISTURBED, "carried", 0.6666, True, None),
    ):
        events.append(
            {"seq": seq, "ts": ts, "kind": kind, "reason": reason, "ratio": ratio, "dry_run": dry_run, "data": data,
             "event": py_agent.build_event(seq, ts, kind, reason, ratio, dry_run, data)}
        )  # fmt: skip
    return {
        "event_types": [py_agent.EVENT_DISTURBED, py_agent.EVENT_RESET, py_agent.EVENT_RESUMED],
        "busy": busy,
        "severity": severities,
        "events": events,
    }


CONFIG_YAML_CASES = [
    # Every field with a valid value.
    "dry_run: false\nreasons: [lifted]\nmetrics_port: 19999\nsettle_s: 2.5\n",
    "{}\n",
    "metrics_port: null\n",
    "metrics_port: 1\n",
    "reasons: []\n",
    "reasons: [mismatch, lifted, carried, rotated]\n",
    "hit_radius_cells: 0\noccupied_threshold: 1\nmin_points: 1\nmedian_window: 1\nmin_map_cells_known: 0\n",
    "occupied_threshold: 100\nkeep_backups: 1\n",
    "mismatch_ratio: 0.0\nhealthy_ratio: 1.0\n",
    "cooldown_s: 12\n",
    "agreement_hz: 1\ntick_hz: 20\n",
    "hit_radius_cells: 2.0\n",
    "scan_topic: /scan\nmap_topic: /map2\nimu_topic: /imu\nwheel_odom_topic: /o\nlidar_odom_topic: /l\n"
    "cmd_vel_topic: /c\nmap_frame: odom\nreset_topic: /r\nevents_topic: /e\npause_service: /p\n"
    "reset_service: /rs\nserialize_service: /s\nparameters_service: /g\nagent_state_url: http://h:1/x\n"
    "backup_dir: /tmp/b\nbackup_prefix: pre\n",
    "service_timeout_s: 1.5\nagent_poll_s: 0.1\nagent_timeout_s: 0.2\nmax_range_m: 4\nmismatch_s: 1\n"
    "lift_tilt_deg: 1\nlift_s: 1\nlevel_window_s: 1\nlevel_tolerance_deg: 1\nlevel_rebaseline_s: 1\n"
    "lift_accel_z: 1\nlift_accel_s: 1\nlift_confirm_s: 1\nlift_confirm_tilt_deg: 1\ngravity_band: 1\n"
    "carried_speed_mps: 1\nwheel_still_mps: 1\ncmd_timeout_s: 1\ncmd_eps: 1\ncarried_s: 1\ninput_timeout_s: 1\n"
    "rotate_rate_rad_s: 1\nwheel_still_rad_s: 1\nrotate_s: 1\nsettle_s: 1\nevaluate_s: 1\ncooldown_s: 1\n",
    "dry_run: true\n",
    "dry_run: 'true'\n",
    "dry_run: yes\n",
    "dry_run: 1\n",
    "lift_s: '2.5'\n",
    # pydantic lax-mode coercions (bool from 0/1 and words, numbers from bools and strings).
    "dry_run: 0.0\n",
    "dry_run: 'Off'\n",
    "dry_run: T\n",
    "lift_s: true\n",
    "lift_s: ' 2.5 '\n",
    "lift_s: 1e3\n",
    "lift_s: '1_000'\n",
    "lift_s: .inf\n",
    "mismatch_ratio: false\nhealthy_ratio: true\n",
    "hit_radius_cells: true\n",
    "hit_radius_cells: '5.0'\n",
    "hit_radius_cells: ' 5 '\n",
    "hit_radius_cells: 0x10\n",
    "metrics_port: 2.0\n",
    "metrics_port: '19'\n",
    # Rejections: lax-mode limits.
    "dry_run: 0.5\n",
    "dry_run: ' true'\n",
    "lift_s: .nan\n",
    "hit_radius_cells: .inf\n",
    "hit_radius_cells: '5.5'\n",
    "reasons: [1]\n",
    "reasons: {a: 1}\n",
    # Rejections: ranges, ratio cross-check, literals, unknown keys, types.
    "lift_s: 0.0\n",
    "settle_s: -1.0\n",
    "agreement_hz: 0.0\n",
    "mismatch_ratio: 1.5\n",
    "healthy_ratio: -0.1\n",
    "healthy_ratio: 0.3\nmismatch_ratio: 0.3\n",
    "healthy_ratio: 0.2\nmismatch_ratio: 0.3\n",
    "reasons: [bumped]\n",
    "reasons: lifted\n",
    "reasons: [Lifted]\n",
    "keep_backups: 0\n",
    "hit_radius_cells: -1\n",
    "occupied_threshold: 101\n",
    "occupied_threshold: 0\n",
    "min_points: 0\n",
    "median_window: 0\n",
    "min_map_cells_known: -1\n",
    "no_such_key: 1\n",
    "dry_run: maybe\n",
    "dry_run: 2\n",
    "dry_run: null\n",
    "scan_topic: 5\n",
    "scan_topic: [a]\n",
    "scan_topic: null\n",
    "map_frame: true\n",
    "tick_hz: abc\n",
    "tick_hz: null\n",
    "tick_hz: [1]\n",
    "tick_hz: -0.5\n",
    "hit_radius_cells: 1.5\n",
    "hit_radius_cells: abc\n",
    "metrics_port: abc\n",
    "metrics_port: 1.5\n",
    "service_timeout_s: 0\n",
    "agent_poll_s: -1\n",
    "agent_timeout_s: 0\n",
    "max_range_m: 0\n",
    "mismatch_s: 0\n",
    "lift_tilt_deg: 0\n",
    "level_window_s: 0\n",
    "level_tolerance_deg: 0\n",
    "level_rebaseline_s: 0\n",
    "lift_accel_z: 0\n",
    "lift_accel_s: 0\n",
    "lift_confirm_s: 0\n",
    "lift_confirm_tilt_deg: 0\n",
    "gravity_band: 0\n",
    "carried_speed_mps: 0\n",
    "wheel_still_mps: 0\n",
    "cmd_timeout_s: 0\n",
    "cmd_eps: 0\n",
    "carried_s: 0\n",
    "input_timeout_s: 0\n",
    "rotate_rate_rad_s: 0\n",
    "wheel_still_rad_s: 0\n",
    "rotate_s: 0\n",
    "evaluate_s: 0\n",
    "cooldown_s: 0\n",
    "backup_dir: 3\n",
    "backup_prefix: [x]\n",
    "agent_state_url: 1.5\n",
]


def config_fixture() -> dict[str, Any]:
    """Defaults of MapGuardConfig and the pydantic verdict on YAML documents.

    Returns:
        dict[str, Any]: defaults (model_dump) and cases {yaml, valid, values (model_dump when valid)}.
    """
    cases = []
    for text in CONFIG_YAML_CASES:
        data = yaml.safe_load(text)
        try:
            config = MapGuardConfig(**data)
        except ValidationError as exc:
            cases.append(
                {
                    "yaml": text,
                    "valid": False,
                    "error_fields": sorted({str(e["loc"][0]) for e in exc.errors() if e["loc"]}),
                }
            )
            continue
        values = {k: num(v) if isinstance(v, float) else v for k, v in config.model_dump().items()}
        cases.append({"yaml": text, "valid": True, "values": values})
    return {"defaults": MapGuardConfig().model_dump(), "cases": cases}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

FIXTURES: dict[str, Callable[[], dict[str, Any]]] = {
    "agreement.json": agreement_fixture,
    "detectors.json": detectors_fixture,
    "guard.json": guard_fixture,
    "timing.json": timing_fixture,
    "backup.json": backup_fixture,
    "agent.json": agent_fixture,
    "config.json": config_fixture,
}


def render(data: dict[str, Any]) -> str:
    """Deterministic JSON text of a fixture.

    Args:
        data (dict[str, Any]): Fixture.

    Returns:
        str: JSON with sorted keys and a trailing newline.
    """
    return json.dumps(data, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n"


def main(argv: list[str] | None = None) -> int:
    """Write the fixtures, or with --check compare them with the files.

    Args:
        argv (list[str] | None): Command line (sys.argv[1:] when None).

    Returns:
        int: 0 when written or up to date; 1 when --check found a stale or missing fixture.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="exit 1 when a fixture differs from the Python output")
    args = parser.parse_args(argv)
    stale = []
    FIXTURE_DIR.mkdir(parents=True, exist_ok=True)
    for name, build in FIXTURES.items():
        text = render(build())
        path = FIXTURE_DIR / name
        if args.check:
            if not path.exists() or path.read_text() != text:
                stale.append(name)
        else:
            path.write_text(text)
            print(f"wrote {path}")
    if stale:
        print(f"stale fixtures (run gen_fixtures.py from nodes/map_guard): {', '.join(stale)}", file=sys.stderr)
        return 1
    if args.check:
        print(f"fixtures up to date: {', '.join(FIXTURES)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
