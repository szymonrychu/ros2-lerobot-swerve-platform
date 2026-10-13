"""Python references of the grasp kernel family (registry read by tests/regenerate_core_golden.py).

grasp.plan_auto (bench/REPORT.md) spends its time in the waypoint IK of GraspPlanner.realize: solve_point (one
waypoint) and line (a straight tool-point line, one IK per interpolation step) on ArmKinematics.inverse. That waypoint
IK is the C++ kernel mcp_core_cpp.grasp.solve_line; the original Python bodies of GraspPlanner.solve_point,
GraspPlanner.pose_problem and GraspPlanner.line live here (moved verbatim, ``self`` becoming the kinematics argument)
and run only to write the golden. ArmKinematics (ikpy and scipy) is the IK they call, rebuilt from the same
solver_geometry the C++ kernel reads.
"""

import math
from collections import OrderedDict
from types import SimpleNamespace
from typing import Any

import ikpy.chain
import ikpy.link
import numpy as np

from mcp_server.grasp import IkBudget, is_stall_pose
from mcp_server.ik import KINEMATICS, ArmKinematics, UnreachableError
from mcp_server.reference import ReferenceKernel, Tolerance

SAMPLES_ATOL_RAD = 1e-9  # joint samples: the C++ TRF follows scipy's steps, only float rounding differs
HEADING_BIAS_ATOL_RAD = 1e-9
# The heading refinement is a fixed-point iteration: one implementation can land bit-exactly on an earlier query (a
# cache hit) where the other is an ulp off (another ikpy run). Samples and reasons are the same; only the effort count
# the IK budget sees moves by a run.
SOLVES_ATOL = 1


def kinematics_from_geometry(geometry: dict[str, Any]) -> ArmKinematics:
    """An ArmKinematics on the ikpy chain described by solver_geometry (no URDF file needed).

    Args:
        geometry (dict[str, Any]): mcp_server.grasp.solver_geometry output.

    Returns:
        ArmKinematics: Kinematics with a cold solution cache and solve_count 0.
    """
    links: list[ikpy.link.Link] = [ikpy.link.OriginLink()]
    for spec in geometry["links"]:
        revolute = spec["joint_type"] == "revolute"
        links.append(
            ikpy.link.URDFLink(
                name=spec["name"],
                origin_translation=np.array(spec["translation"]),
                origin_orientation=np.array(spec["orientation"]),
                rotation=np.array(spec["axis"]) if revolute else None,
                bounds=tuple(spec["bounds"]) if revolute else None,
                joint_type=spec["joint_type"],
            )
        )
    mask = [link.name in geometry["active"] for link in links]
    kin = ArmKinematics.__new__(ArmKinematics)
    kin.chain = ikpy.chain.Chain(links, active_links_mask=mask, name="so101")
    kin.limits = {j: (float(v[0]), float(v[1])) for j, v in geometry["limits"].items()}
    kin.margin = float(geometry["margin"])
    kin.tool_offset = np.array(geometry["tool_offset"], dtype=np.float64)
    kin.offsets = {j: float(v) for j, v in geometry["offsets"].items()}
    kin.index = {j: int(v) for j, v in geometry["index"].items()}
    kin.joint_names = tuple(geometry["joints"])
    kin.max_reach_m = float(geometry["max_reach_m"])
    kin.pan_axis_xy = (float(geometry["pan_axis_xy"][0]), float(geometry["pan_axis_xy"][1]))
    kin.solve_count = 0
    kin.last_heading_bias = 0.0
    kin.solutions = OrderedDict()
    kin.core = None  # core_model() builds the C++ ArmModel on demand (the reference itself never does)
    kin.core_released = False
    KINEMATICS.add(kin)  # so release_core_models frees a model the C++ side of the golden built on it
    return kin


def solve_point(
    kin: ArmKinematics,
    label: str,
    point: np.ndarray,
    pitch: float | None,
    seed: dict[str, float],
    extra: tuple[float, float, float] | None,
    params: Any,
    budget: IkBudget | None = None,
) -> tuple[list[dict[str, float]], str | None]:
    """IK of one waypoint with the pose checks (the original GraspPlanner.solve_point).

    Args:
        kin (ArmKinematics): Kinematics.
        label (str): Waypoint label (for reasons).
        point (np.ndarray): Target (arm frame).
        pitch (float | None): Approach pitch (rad).
        seed (dict[str, float]): IK seed (its wrist_roll is kept).
        extra (tuple[float, float, float] | None): Centring tool offset.
        params (Any): Thresholds (GraspParams attributes).
        budget (IkBudget | None): Effort cap; a spent budget is a reason instead of another search.

    Returns:
        tuple[list[dict[str, float]], str | None]: [solution] and None, or ([], reason).
    """
    if budget is not None and budget.spent():
        return [], f"{label}: IK budget of {budget.limit} solves spent (planning effort cap)"
    try:
        sol = kin.inverse(*point, pitch, seed, extra, kin.last_heading_bias)
    except UnreachableError as exc:
        return [], f"{label} unreachable: {exc}"
    problem = pose_problem(kin, label, sol, params)
    return ([sol], None) if problem is None else ([], problem)


def pose_problem(kin: ArmKinematics, label: str, joints: dict[str, float], params: Any) -> str | None:
    """Limit-margin and shoulder-stall checks of one pose (the original GraspPlanner.pose_problem).

    Args:
        kin (ArmKinematics): Kinematics.
        label (str): Waypoint label.
        joints (dict[str, float]): Measured joint angles.
        params (Any): Thresholds (GraspParams attributes).

    Returns:
        str | None: Reason, or None when the pose is fine.
    """
    if not kin.within_limits(kin.to_urdf(joints)):
        return f"{label}: joints outside the limits minus the {kin.margin} rad margin"
    if is_stall_pose(joints, params):
        return (
            f"{label}: stretched arm (elbow_flex {joints['elbow_flex']:.2f} rad) with shoulder_lift "
            f"{joints['shoulder_lift']:.2f} rad above {params.stall_shoulder_lift_rad} rad would stall the shoulder"
        )
    return None


def line(
    kin: ArmKinematics,
    label: str,
    start: np.ndarray,
    end: np.ndarray,
    pitch: float | None,
    seed: dict[str, float],
    extra: tuple[float, float, float] | None,
    params: Any,
    budget: IkBudget | None = None,
) -> tuple[list[dict[str, float]], str | None]:
    """Straight tool-point line sampled every interpolation_step_m (the original GraspPlanner.line).

    Args:
        kin (ArmKinematics): Kinematics.
        label (str): Waypoint label.
        start (np.ndarray): Line start (arm frame).
        end (np.ndarray): Line end.
        pitch (float | None): Constant approach pitch (rad).
        seed (dict[str, float]): Pose at the line start.
        extra (tuple[float, float, float] | None): Centring tool offset.
        params (Any): Step, jump and stall thresholds (GraspParams attributes).
        budget (IkBudget | None): Effort cap shared with the other IK of the candidate.

    Returns:
        tuple[list[dict[str, float]], str | None]: Samples (end included) and None, or ([], reason).
    """
    n = max(1, math.ceil(float(np.linalg.norm(end - start)) / params.interpolation_step_m))
    samples: list[dict[str, float]] = []
    prev = seed
    for i in range(1, n + 1):
        point = start + (end - start) * i / n
        solved, problem = solve_point(kin, label, point, pitch, prev, extra, params, budget)
        if problem is not None:
            return [], problem
        sol = solved[0]
        jump = max(abs(sol[j] - prev[j]) for j in kin.joint_names)
        if jump > params.max_joint_jump_rad:
            return [], (
                f"{label}: joint jump {jump:.2f} rad between straight-line samples at "
                f"({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}) exceeds max_joint_jump_rad {params.max_joint_jump_rad}"
            )
        samples.append(sol)
        prev = sol
    return samples, None


def solve_line_reference(
    geometry: dict[str, Any],
    label: str,
    start: list[float] | None,
    end: list[float],
    pitch: float,
    seed: dict[str, float],
    extra: list[float] | None,
    heading_bias: float,
    params: dict[str, float],
    budget_limit: int | None,
    budget_used: int,
) -> dict[str, Any]:
    """The reference of mcp_core_cpp.grasp.solve_line: line (start given) or solve_point (start None).

    Args:
        geometry (dict[str, Any]): solver_geometry of the arm.
        label (str): Waypoint label.
        start (list[float] | None): Line start, or None for the single waypoint end.
        end (list[float]): Line end / waypoint (arm frame, m).
        pitch (float): Approach pitch (rad).
        seed (dict[str, float]): Pose at the start (measured, the five chain joints).
        extra (list[float] | None): Centring tool offset (m, gripper_frame_link).
        heading_bias (float): ArmKinematics.last_heading_bias before the call.
        params (dict[str, float]): kernel_params of the grasp parameters.
        budget_limit (int | None): IkBudget.limit, None without a budget.
        budget_used (int): IK solves the budget has already counted.

    Returns:
        dict[str, Any]: samples (list of joint dicts, empty on a problem), problem (str | None), solves (ikpy runs
            of this call) and heading_bias (last_heading_bias after the call).
    """
    kin = kinematics_from_geometry(geometry)
    kin.last_heading_bias = heading_bias
    budget = None
    if budget_limit is not None:
        budget = IkBudget(kin, budget_limit)
        budget.start = -budget_used
    offset = None if extra is None else (float(extra[0]), float(extra[1]), float(extra[2]))
    thresholds = SimpleNamespace(**params)
    if start is None:
        samples, problem = solve_point(kin, label, np.array(end), pitch, dict(seed), offset, thresholds, budget)
    else:
        samples, problem = line(
            kin, label, np.array(start), np.array(end), pitch, dict(seed), offset, thresholds, budget
        )
    return {"samples": samples, "problem": problem, "solves": kin.solve_count, "heading_bias": kin.last_heading_bias}


# Kernel name -> reference; a port of a grasp kernel to mcp_core_cpp.grasp adds its entry here.
KERNELS: dict[str, ReferenceKernel] = {
    "solve_line": ReferenceKernel(
        run=solve_line_reference,
        inputs="grasp_solve_line_cases",
        tolerances={
            "samples": Tolerance(atol=SAMPLES_ATOL_RAD),
            "heading_bias": Tolerance(atol=HEADING_BIAS_ATOL_RAD),
            "solves": Tolerance(atol=SOLVES_ATOL),
        },
    ),
}
