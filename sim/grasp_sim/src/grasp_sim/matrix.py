"""Planner scenario matrix: replay mcp_server GraspPlans (made by scripts/plan_matrix.py) with the executor's timing.

The planner runs in the nodes/mcp_server uv environment (ikpy, the node's own pydantic models) and writes
index.json plus one GraspPlan JSON per scenario. This module, in the sim environment, turns each feasible plan into
the setpoint stream GraspExecutor.execute would send (nodes/mcp_server/mcp_server/grasp_tools.py, trajectory.py) and
judges it with grasp_sim.replay.simulate on a scene with the calibrated jaws.
"""

import json
import math
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

from pydantic import BaseModel, ConfigDict

from grasp_sim.config import BoxObjectConfig, ObjectShape, SceneConfig, SimConfig
from grasp_sim.plan import JOINT_ORDER
from grasp_sim.replay import simulate
from grasp_sim.report import SegmentReport

ARM_JOINTS = JOINT_ORDER[:5]
GRIPPER = "gripper"
QUINTIC_PEAK_VELOCITY_FACTOR = 15.0 / 8.0
FLOOR_TOLERANCE_M = 1e-6
ARM_CONTACT_KINDS = ("arm_floor", "arm_support")
INDEX_FILE = "index.json"
DEFAULT_RESULTS_FILE = "results.json"
DESCENT_LABELS = ("approach", "grasp")  # the open jaws travel down onto the object


class StrictModel(BaseModel):
    """Base model that rejects unknown keys."""

    model_config = ConfigDict(extra="forbid")


class ExecutorTiming(StrictModel):
    """mcp_server limits that set the executor's timing (defaults: nodes/mcp_server/mcp_server/config.py)."""

    rate_hz: float = 25.0
    arm_max_joint_velocity_rps: float = 1.0
    arm_max_speed_scale: float = 0.5
    gripper_velocity_rps: float = 0.5
    gripper_closed_rad: float = -0.165
    close_hold_s: float = 0.3

    def arm_velocity(self, speed_scale: float | None) -> float:
        """Per-joint velocity cap of a speed scale (ArmController.velocity_for).

        Args:
            speed_scale (float | None): Scale in (0, arm_max_speed_scale]; None = the maximum.

        Returns:
            float: rad/s.
        """
        scale = self.arm_max_speed_scale if speed_scale is None else min(speed_scale, self.arm_max_speed_scale)
        return self.arm_max_joint_velocity_rps * scale / self.arm_max_speed_scale


class MatrixEntry(StrictModel):
    """One planned scenario (written by scripts/plan_matrix.py)."""

    key: str
    box: str
    position: str
    support: str
    strategy: str
    x: float
    y: float
    support_z: float  # object bottom (arm frame, m), the planner's ObjectSpec.support_z
    surface_z: float  # top of the slab under the object (arm frame, m); support_z - gap_below_m
    size_m: tuple[float, float, float]  # depth (radial), width (across the jaws), height
    gap_below_m: float = 0.0
    shape: ObjectShape = "box"  # cylinder: upright, diameter size_m[0]
    mass_kg: float | None = None  # None = the scene default (0.05 kg)
    # Where the sim puts the object relative to where the planner was told (arm frame x, y, m): a perception error.
    sim_offset_xy_m: tuple[float, float] = (0.0, 0.0)
    support_edge_x: float | None = None  # where the ledge/stair starts (arm x, m); None = the scene default
    surfaces: list[dict[str, Any]] | None = None  # surface regions the planner was given (the stair step)
    feasible: bool
    reasons: list[str]
    chosen: str | None = None
    approach_pitch_deg: float | None = None
    plan_file: str


class MatrixIndex(StrictModel):
    """index.json: the scene constants of the run and every scenario."""

    base_height_m: float
    floor_z_m: float
    tool_offset_m: tuple[float, float, float]
    # Shoulder pan axis (arm frame x, y, m): the planner puts the jaws across the heading from it (yaw omitted), so the
    # box faces it. (0, 0) for indexes written before 2026-10-10 evening (the box then faced the arm base origin).
    pan_axis_xy: tuple[float, float] = (0.0, 0.0)
    timing: ExecutorTiming = ExecutorTiming()
    planner_params: dict[str, Any] = {}
    entries: list[MatrixEntry]


def quintic(s: float) -> float:
    """Quintic time scaling 10 s^3 - 15 s^4 + 6 s^5 (trajectory.quintic)."""
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s))


def path_samples(
    start: dict[str, float], path: list[dict[str, float]], max_velocity: float, rate_hz: float
) -> list[dict[str, float]]:
    """Setpoints along a joint polyline with one quintic profile over its whole length (trajectory.path_trajectory).

    Args:
        start (dict[str, float]): Pose before the first path sample.
        path (list[dict[str, float]]): Samples; the last one is the goal.
        max_velocity (float): Per-joint velocity cap (rad/s).
        rate_hz (float): Setpoint rate.

    Returns:
        list[dict[str, float]]: Setpoints (start excluded), the last equal to path[-1].
    """
    nodes = [start, *path]
    legs = [max(abs(b[j] - a[j]) for j in b) for a, b in zip(nodes, nodes[1:], strict=False)]
    total = sum(legs)
    if total <= 0.0:
        return [dict(path[-1])]
    steps = max(1, math.ceil(QUINTIC_PEAK_VELOCITY_FACTOR * total / max_velocity * rate_hz))
    points: list[dict[str, float]] = []
    leg, covered = 0, 0.0
    for i in range(1, steps):
        distance = quintic(i / steps) * total
        while leg < len(legs) - 1 and covered + legs[leg] < distance:
            covered += legs[leg]
            leg += 1
        a, b = nodes[leg], nodes[leg + 1]
        f = 0.0 if legs[leg] <= 0.0 else min(1.0, (distance - covered) / legs[leg])
        points.append({j: a[j] + (b[j] - a[j]) * f for j in b})
    points.append(dict(path[-1]))
    return points


def executor_samples(plan: dict[str, Any], timing: ExecutorTiming) -> list[dict[str, Any]]:
    """Replay samples of GraspExecutor.execute for a feasible GraspPlan (model_dump()).

    Starts at the pre-grasp with the wrist already rolled and the gripper at the pre-grasp opening (the half-open and
    roll steps before it happen high above the object). Then: open (gripper only), approach and grasp along the
    planned straight-line samples, close to gripper_closed_rad (the sim servo stalls on the object at its force limit),
    lift and retreat along their samples, each linear step at its waypoint speed_scale.

    Args:
        plan (dict[str, Any]): GraspPlan JSON.
        timing (ExecutorTiming): Velocity caps and rate.

    Returns:
        list[dict[str, Any]]: Samples {"t", "joints", "label"?} for grasp_sim.replay.simulate.
    """
    wp = {w["label"]: w for w in plan["waypoints"]}
    dt = 1.0 / timing.rate_hz
    pre = wp["pre_grasp"]
    current = {j: float(pre["joints"][j]) for j in ARM_JOINTS} | {GRIPPER: float(pre["gripper"])}
    out: list[dict[str, Any]] = [{"t": 0.0, "joints": dict(current), "label": "pre_grasp"}]

    def emit(label: str, setpoints: list[dict[str, float]]) -> None:
        nonlocal current
        for k, point in enumerate(setpoints):
            current = current | point
            sample: dict[str, Any] = {"t": round(out[-1]["t"] + dt, 6), "joints": dict(current)}
            if k == 0:
                sample["label"] = label
            out.append(sample)

    def gripper_to(label: str, target: float, hold_s: float = 0.0) -> None:
        moves = path_samples({GRIPPER: current[GRIPPER]}, [{GRIPPER: target}], timing.gripper_velocity_rps, 1 / dt)
        emit(label, moves + [{GRIPPER: target}] * math.ceil(hold_s / dt))

    gripper_to("open", float(wp["open"]["gripper"]))
    for label in ("approach", "grasp"):
        path = [{j: float(s[j]) for j in ARM_JOINTS} for s in plan["segments"][label]]
        start = {j: current[j] for j in ARM_JOINTS}
        emit(label, path_samples(start, path, timing.arm_velocity(wp[label]["speed_scale"]), timing.rate_hz))
    gripper_to("close", timing.gripper_closed_rad, timing.close_hold_s)
    for label in ("lift", "retreat"):
        path = [{j: float(s[j]) for j in ARM_JOINTS} for s in plan["segments"][label]]
        start = {j: current[j] for j in ARM_JOINTS}
        emit(label, path_samples(start, path, timing.arm_velocity(wp[label]["speed_scale"]), timing.rate_hz))
    return out


def scene_for(entry: MatrixEntry, index: MatrixIndex, stock_jaws: bool) -> SceneConfig:
    """Scene of one scenario: the slab at surface_z, the box (rails when gap_below_m > 0) facing the shoulder pan
    axis radially, square to the jaws the planner aligned across that heading.

    Args:
        entry (MatrixEntry): Scenario.
        index (MatrixIndex): Run constants (mount height, floor, tool offset).
        stock_jaws (bool): True to keep the uncalibrated Menagerie jaws.

    Returns:
        SceneConfig: Scene.
    """
    on_floor = abs(entry.surface_z - index.floor_z_m) < FLOOR_TOLERANCE_M
    return SceneConfig(
        base_height_m=index.base_height_m,
        support_z_m=None if on_floor else entry.surface_z,
        support_edge_x_m=None if on_floor else entry.support_edge_x,
        tool_offset_m=None if stock_jaws else index.tool_offset_m,
        object=BoxObjectConfig(
            shape=entry.shape,
            size_m=entry.size_m,
            **({} if entry.mass_kg is None else {"mass_kg": entry.mass_kg}),
            x_m=entry.x + entry.sim_offset_xy_m[0],
            y_m=entry.y + entry.sim_offset_xy_m[1],
            yaw_rad=math.atan2(entry.y - index.pan_axis_xy[1], entry.x - index.pan_axis_xy[0]),
            gap_below_m=entry.gap_below_m,
        ),
    )


def base_result(entry: MatrixEntry) -> dict[str, Any]:
    """Result fields every scenario carries, replayed or not."""
    return {
        "key": entry.key,
        "box": entry.box,
        "position": entry.position,
        "support": entry.support,
        "strategy": entry.strategy,
        "chosen": entry.chosen,
        "approach_pitch_deg": entry.approach_pitch_deg,
        "feasible": entry.feasible,
        "plan_reasons": entry.reasons,
        "lifted": False,
        "arm_contact": False,
        "reasons": [],
    }


def run_entry(job: tuple[MatrixEntry, MatrixIndex, Path, bool]) -> dict[str, Any]:
    """Replay one scenario (feasible plans only) and summarise the SimReport.

    Args:
        job (tuple[MatrixEntry, MatrixIndex, Path, bool]): Entry, index, run directory, stock_jaws.

    Returns:
        dict[str, Any]: Result row (see base_result) plus the replay verdict.
    """
    entry, index, root, stock_jaws = job
    result = base_result(entry)
    if not entry.feasible:
        return result
    plan = json.loads((root / entry.plan_file).read_text())
    report = simulate(executor_samples(plan, index.timing), scene_for(entry, index, stock_jaws), SimConfig())
    arm_contacts = sum(report.event_counts.get(k, 0) for k in ARM_CONTACT_KINDS)
    clearances = {
        key: min(
            (getattr(s.min_clearance, key) for s in report.segments if getattr(s.min_clearance, key) is not None),
            default=None,
        )
        for key in ("jaws_floor", "jaws_support")
    }
    obj = report.object
    descent = descent_motion(report.segments)
    result.update(
        {
            "lifted": bool(report.grasp_success) and arm_contacts == 0,
            "arm_contact": arm_contacts > 0,
            "jaw_surface_contacts": report.event_counts.get("jaw_floor", 0) + report.event_counts.get("jaw_support", 0),
            "reasons": report.reasons,
            "min_clearance_m": clearances,
            "approach_tilt_deg": None if obj is None else round(obj.approach_max_tilt_deg, 1),
            "approach_push_m": None if obj is None else round(obj.approach_max_displacement_m, 4),
            "descent_tilt_deg": round(descent[0], 1),
            "descent_push_m": round(descent[1], 4),
            "lift_height_m": None if obj is None else round(obj.lift_height_m, 4),
            "saturated": report.saturated_actuators,
        }
    )
    return result


def descent_motion(segments: list[SegmentReport]) -> tuple[float, float]:
    """Worst object tilt and push while the open jaws come down onto the object (approach and grasp slide).

    The slide into the grasp is labelled grasp, where contact is allowed, so the SimReport approach_* figures leave it
    out; a fixed jaw landing on the object rim (the 2026-10-10 jar tip-over) shows here.

    Args:
        segments (list[SegmentReport]): SimReport segments.

    Returns:
        tuple[float, float]: (max tilt in deg, max horizontal displacement in m); zeros without such segments.
    """
    picked = [s for s in segments if s.label in DESCENT_LABELS]
    tilt = max((s.max_object_tilt_deg or 0.0 for s in picked), default=0.0)
    push = max((s.max_object_displacement_m or 0.0 for s in picked), default=0.0)
    return tilt, push


def failure_mode(result: dict[str, Any]) -> str:
    """Short failure class of a result row."""
    if not result["feasible"]:
        return "infeasible"
    if result["lifted"]:
        return "lifted"
    if result.get("arm_contact"):
        return "arm contact"
    text = " ".join(result.get("reasons", []))
    if "tipped" in text:
        return "tipped"
    if "pushed" in text:
        return "pushed"
    return "not lifted (slip/miss)"


def summary_table(results: list[dict[str, Any]]) -> str:
    """Lifted / feasible counts per strategy and box, plus failure modes per strategy.

    Args:
        results (list[dict[str, Any]]): Result rows.

    Returns:
        str: Plain-text table.
    """
    boxes = sorted({r["box"] for r in results})
    strategies = list(dict.fromkeys(r["strategy"] for r in results))
    lines = ["lifted/feasible (planned scenarios)", f"{'strategy':<12}" + "".join(f"{b:>12}" for b in boxes)]
    for strategy in strategies:
        cells = []
        for box in boxes:
            rows = [r for r in results if r["strategy"] == strategy and r["box"] == box]
            feasible = sum(r["feasible"] for r in rows)
            lifted = sum(r["lifted"] for r in rows)
            cells.append(f"{lifted}/{feasible} of {len(rows)}")
        lines.append(f"{strategy:<12}" + "".join(f"{c:>12}" for c in cells))
    lines.append("failure modes")
    for strategy in strategies:
        modes = Counter(failure_mode(r) for r in results if r["strategy"] == strategy)
        lines.append(f"{strategy:<12}" + ", ".join(f"{k} {v}" for k, v in sorted(modes.items())))
    return "\n".join(lines)


def run_matrix(root: Path, workers: int, stock_jaws: bool) -> list[dict[str, Any]]:
    """Replay every scenario of root/index.json.

    Args:
        root (Path): Directory with index.json and the plan files.
        workers (int): Parallel processes (1 = in process).
        stock_jaws (bool): Keep the uncalibrated jaws.

    Returns:
        list[dict[str, Any]]: One result row per entry, in index order.
    """
    index = MatrixIndex.model_validate_json((root / INDEX_FILE).read_text())
    jobs = [(e, index, root, stock_jaws) for e in index.entries]
    if workers <= 1:
        return [run_entry(job) for job in jobs]
    with ProcessPoolExecutor(workers) as pool:
        return list(pool.map(run_entry, jobs))
