"""Python references of the ik_reach kernel family (registry read by tests/regenerate_core_golden.py).

Ported to mcp_core_cpp.ik_reach (bench/report.json verdict 'port'):

- ``point_reach``: the original body of arm_reach.point_reach (ikpy IK per approach pitch, pruned body clearance of
  each solution). Golden outputs: the point_reach result plus ``solutions`` (the IK solution of every pitch, None
  where the solver found none).
- ``reach_map_build``: the original body of reach_map.build (forward-sampled reach map). Golden outputs: pitch mask,
  clearance, counts, grid and key of the map.

Each golden case carries the robot model it was computed for (``robot_inputs``: URDF, joint offsets and limits, tool
point, jaw, arm mount, body obstacles and margins), so a later change of the deployed config does not invalidate it.
mcp_server never calls these bodies (it calls the C++ kernels); the golden generator and the tests do.
"""

import datetime
import functools
import json
import logging
import math
import time
from typing import Any

import numpy as np

from mcp_server.arm_reach import ENVELOPE_BATCH_SAMPLES, POINT_PITCHES_DEG, SHOULDER_PAN, ReachEnvelope, joint_range
from mcp_server.body_model import BodyModel
from mcp_server.config import (
    REPO_ROOT,
    ArmBaseOffset,
    BodyCheckSettings,
    BodySettings,
    McpServerConfig,
    ReachMapSettings,
)
from mcp_server.floor_guard import JawModel, arm_to_base_link, base_link_to_arm
from mcp_server.ik import ArmKinematics, UnreachableError
from mcp_server.reach_map import (
    GRID_DECIMALS,
    PITCH_HALF_SPACING_DEG,
    PLANAR_JOINTS,
    REACH_MAP_VERSION,
    ReachMap,
    body_clearance,
    grid_values,
    heading_grid,
    pan_geometry,
    rank_in_group,
    reach_map_key,
    tool_points,
    wrap_deg,
)
from mcp_server.reference import ReferenceKernel, Tolerance
from mcp_server.self_collision import SelfCollisionChecker

# IK joint solutions agree within this (rad) wherever the Python solver converges.
SOLUTION_TOLERANCE_RAD = 1e-6
# The map stores clearance as float16: one float16 step at the soft margin (0.03 m) is 1.5e-5 m.
CLEARANCE_TOLERANCE_M = 2e-5

log = logging.getLogger(__name__)


def robot_inputs(config: McpServerConfig) -> dict[str, Any]:
    """The robot model inputs of the reach kernels, JSON-ready (stored in every golden case).

    Args:
        config (McpServerConfig): Node configuration.

    Returns:
        dict[str, Any]: urdf (repo-relative), margin, joint_offsets, tool_offset, limit_overrides, jaw_open_axis,
            gripper_closed_rad, gripper_joint, mount, body, body_check.
    """
    arm = config.arm
    mount = arm.base_in_base_link or ArmBaseOffset(z=arm.arm_base_height_m)
    return {
        "urdf": str(arm.urdf_path.resolve().relative_to(REPO_ROOT)),
        "margin": config.limits.arm_limit_margin_rad,
        "joint_offsets": arm.joint_offsets_rad.model_dump(),
        "tool_offset": list(arm.tool_offset_m.model_dump().values()),
        "limit_overrides": {k: list(v) for k, v in arm.joint_limit_overrides_rad.items()},
        "jaw_open_axis": list(arm.jaw_open_axis),
        "gripper_closed_rad": arm.gripper_closed_rad,
        "gripper_joint": arm.gripper_joint,
        "mount": mount.model_dump(),
        "body": config.body.model_dump(mode="json"),
        "body_check": config.body_check.model_dump(mode="json", exclude={"reach_cache_file"}),
    }


@functools.cache
def cached_checker(robot_json: str) -> SelfCollisionChecker:
    """The body checker of a robot_inputs JSON (built once per model).

    Args:
        robot_json (str): json.dumps(robot_inputs(...), sort_keys=True).

    Returns:
        SelfCollisionChecker: Kinematics, jaw, body, margins and mount as the node builds them.
    """
    robot = json.loads(robot_json)
    kin = ArmKinematics(
        REPO_ROOT / robot["urdf"],
        margin=robot["margin"],
        joint_offsets=robot["joint_offsets"],
        tool_offset=tuple(robot["tool_offset"]),
        limit_overrides={k: (v[0], v[1]) for k, v in robot["limit_overrides"].items()},
    )
    jaw = JawModel(kin, tuple(robot["jaw_open_axis"]), robot["gripper_closed_rad"])
    return SelfCollisionChecker(
        kin,
        BodyModel.from_settings(BodySettings.model_validate(robot["body"])),
        BodyCheckSettings.model_validate(robot["body_check"]),
        ArmBaseOffset(**robot["mount"]),
        jaw,
        robot["gripper_joint"],
    )


def checker_for(robot: dict[str, Any]) -> SelfCollisionChecker:
    """The body checker of robot_inputs.

    Args:
        robot (dict[str, Any]): robot_inputs output.

    Returns:
        SelfCollisionChecker: The checker (shared per model).
    """
    return cached_checker(json.dumps(robot, sort_keys=True))


def point_reach(
    checker: SelfCollisionChecker,
    point: dict[str, float | None],
    seed: dict[str, float],
    envelope: ReachEnvelope,
) -> dict[str, Any]:
    """Reach and body clearance of one base_link tool point (see the module docstring).

    Args:
        checker (SelfCollisionChecker): Body checker.
        point (dict[str, float | None]): x, y, z (base_link, m) and pitch (rad, + down) or None.
        seed (dict[str, float]): IK seed (current arm pose).
        envelope (ReachEnvelope): Envelope for the suggestion of an unreachable point.

    Returns:
        dict[str, Any]: point, reachable, collision_free, pitch_range_deg, pitches_deg, slowed, limiting, suggestion.
    """
    cfg = checker.settings
    kin = checker.kin
    x, y, z = float(point["x"] or 0.0), float(point["y"] or 0.0), float(point["z"] or 0.0)
    target = base_link_to_arm((x, y, z), checker.mount)
    pitch = point.get("pitch")
    pitches = [float(pitch)] if pitch is not None else [math.radians(p) for p in POINT_PITCHES_DEG]
    reachable: list[float] = []
    working: list[tuple[float, float]] = []
    limiting: dict[str, Any] | None = None
    solutions: list[dict[str, float]] = []
    for p in pitches:
        try:
            solutions.append(kin.inverse(*target, p, seed=seed))
        except UnreachableError:
            continue
        reachable.append(p)
    for p, (clearance, obstacle, part) in zip(reachable, checker.closest(solutions), strict=True):
        if clearance >= cfg.hard_margin_m:
            working.append((p, clearance))
        elif limiting is None or clearance > limiting["clearance_m"]:
            limiting = {"obstacle": obstacle, "link": part, "clearance_m": round(clearance, 4)}
    pan_axis = arm_to_base_link((*kin.pan_axis_xy, 0.0), checker.mount)
    heading = math.degrees(math.atan2(y - pan_axis[1], x - pan_axis[0]))
    r = math.hypot(x - pan_axis[0], y - pan_axis[1])
    degs = [round(math.degrees(p), 1) for p, _ in working]
    out: dict[str, Any] = {
        "point": {"x": x, "y": y, "z": z, "pitch": pitch},
        "reachable": bool(reachable),
        "collision_free": bool(working),
        "pitch_range_deg": [min(degs), max(degs)] if degs else None,
        "pitches_deg": degs,
        "slowed": bool(working) and all(c < cfg.soft_margin_m for _, c in working),
        "limiting": limiting,
        "heading_deg": round(heading, 1),
        "r_m": round(r, 3),
    }
    if working:
        out["suggestion"] = f"reachable without touching the body at pitch {degs[0]:g}..{degs[-1]:g} deg" + (
            " (inside the soft margin: the move runs slowed)" if out["slowed"] else ""
        )
    elif reachable:
        assert limiting is not None
        obstacle = checker.body.by_name(limiting["obstacle"])
        out["suggestion"] = (
            f"every reachable pitch brings the {limiting['link']} within {cfg.hard_margin_m:g} m of "
            f"{limiting['obstacle']} (best clearance {limiting['clearance_m']:.3f} m): the move would be rejected. "
            f"Raise the point above z {obstacle.top_z + cfg.soft_margin_m:.2f} m, move it away from "
            f"{limiting['obstacle']}, or turn the base so the arm reaches it from another side"
        )
    else:
        cell = envelope.cell(heading, z)
        if cell is not None and cell["max_r_m"] is not None:
            out["suggestion"] = (
                f"out of reach{'' if pitch is None else ' at this pitch'}: at heading {cell['heading_deg']:g} deg and "
                f"height {cell['z_m']:g} m the collision-free envelope spans {cell['min_r_m']:.2f}..{cell['max_r_m']:.2f} "
                f"m from the pan axis; the point is {r:.2f} m away. Drive the base closer or pick a point inside"
            )
        else:
            out["suggestion"] = (
                f"out of reach{'' if pitch is None else ' at this pitch'}: the point is {r:.2f} m from the pan axis at "
                f"height {z:g} m; drive the base closer or try another pitch"
            )
    return out


def build(checker: SelfCollisionChecker, settings: ReachMapSettings) -> ReachMap:
    """Forward-sample the map (see the reach_map module docstring).

    Args:
        checker (SelfCollisionChecker): Body checker of the robot.
        settings (ReachMapSettings): Grid and sampling density.

    Returns:
        ReachMap: The map (not saved).
    """
    started = time.monotonic()
    pan = pan_geometry(checker)
    pan_lo, pan_hi = joint_range(checker, SHOULDER_PAN)
    headings = heading_grid(pan, pan_lo, pan_hi, settings.heading_step_deg)
    radii = grid_values(settings.r_step_m, settings.r_max_m)
    heights = grid_values(settings.z_step_m, settings.z_max_m)
    pitches = np.array(POINT_PITCHES_DEG)
    shape = (len(headings), len(radii), len(heights))
    joint_grids = [
        np.linspace(*joint_range(checker, j), n) for j, n in zip(PLANAR_JOINTS, settings.planar_samples, strict=True)
    ]

    def planar(index: int) -> dict[str, float]:
        a, b, c = np.unravel_index(index, settings.planar_samples)
        values = (joint_grids[0][a], joint_grids[1][b], joint_grids[2][c])
        return dict(zip(PLANAR_JOINTS, (float(v) for v in values), strict=True)) | {"wrist_roll": 0.0}

    total = math.prod(settings.planar_samples)
    points = np.empty((total, 3))
    pitch_rad = np.empty(total)
    for start in range(0, total, ENVELOPE_BATCH_SAMPLES * 64):
        chunk = [planar(i) | {SHOULDER_PAN: 0.0} for i in range(start, min(start + ENVELOPE_BATCH_SAMPLES * 64, total))]
        points[start : start + len(chunk)], pitch_rad[start : start + len(chunk)] = tool_points(checker, chunk)
    rel = points - pan.axis
    r = np.hypot(rel[:, 0], rel[:, 1])
    bearing = np.degrees(np.arctan2(rel[:, 1], rel[:, 0]))
    ri = np.rint(r / settings.r_step_m).astype(np.int64) - 1
    zi = np.rint(points[:, 2] / settings.z_step_m).astype(np.int64) - 1
    gap = np.abs(np.degrees(pitch_rad)[:, None] - pitches[None, :])
    pi = gap.argmin(axis=1)
    keep = (
        (ri >= 0)
        & (ri < len(radii))
        & (zi >= 0)
        & (zi < len(heights))
        & (gap.min(axis=1) <= PITCH_HALF_SPACING_DEG + 1e-9)
    )
    index = np.nonzero(keep)[0]
    ri, zi, pi, bearing = ri[index], zi[index], pi[index], bearing[index]
    distance = np.hypot(r[index] - np.array(radii)[ri], points[index, 2] - np.array(heights)[zi])
    cell = (ri * len(heights) + zi) * len(pitches) + pi
    order = np.lexsort((index, distance, cell))
    del points, rel, r, gap, pitch_rad
    mask = np.zeros(shape, dtype=np.uint16)
    best = np.full(shape, np.nan)
    checked = clear_count = 0
    pan_lo_deg, pan_hi_deg = math.degrees(pan_lo), math.degrees(pan_hi)
    for h, heading in enumerate(headings):
        pan_deg = wrap_deg(pan.sign * (heading - bearing))
        allowed = (pan_deg >= pan_lo_deg) & (pan_deg <= pan_hi_deg)
        candidates = order[allowed[order]]
        chosen = candidates[rank_in_group(cell[candidates]) < settings.samples_per_cell]
        for start in range(0, len(chosen), ENVELOPE_BATCH_SAMPLES):
            batch = chosen[start : start + ENVELOPE_BATCH_SAMPLES]
            samples = [planar(int(index[k])) | {SHOULDER_PAN: math.radians(float(pan_deg[k]))} for k in batch]
            clearance = body_clearance(checker, checker.geometry(samples))
            clear = clearance >= checker.settings.hard_margin_m
            flat = np.ravel_multi_index((np.full(int(clear.sum()), h), ri[batch][clear], zi[batch][clear]), shape)
            np.bitwise_or.at(mask.reshape(-1), flat, (1 << pi[batch][clear]).astype(np.uint16))
            np.fmax.at(best.reshape(-1), flat, clearance[clear])
            checked += len(batch)
            clear_count += int(clear.sum())
        log.info("reach map heading %g deg: %d samples checked", heading, len(chosen))
    bits = np.unpackbits(mask.view(np.uint8)).sum()
    meta = {
        "key": reach_map_key(checker, settings),
        "version": REACH_MAP_VERSION,
        "grid": {
            "heading_step_deg": settings.heading_step_deg,
            "headings_deg": headings,
            "r_step_m": settings.r_step_m,
            "r_m": radii,
            "z_step_m": settings.z_step_m,
            "z_m": heights,
            "pitches_deg": list(POINT_PITCHES_DEG),
            "pan_axis_base_link": [round(float(v), GRID_DECIMALS) for v in pan.axis],
            "pan_sign": pan.sign,
            "pan_range_deg": [round(pan_lo_deg, 3), round(pan_hi_deg, 3)],
        },
        "planar_samples": list(settings.planar_samples),
        "samples_per_cell": settings.samples_per_cell,
        "compute_s": round(time.monotonic() - started, 3),
        "created_at": datetime.datetime.now(datetime.UTC).isoformat(timespec="seconds"),
        "counts": {
            "planar_samples": total,
            "binned_samples": len(index),
            "checked_samples": checked,
            "clear_samples": clear_count,
            "reachable_cells": int((mask > 0).sum()),
            "reachable_cell_pitches": int(bits),
            "cells": int(mask.size),
        },
    }
    return ReachMap(mask, best.astype(np.float16), meta)


def point_solutions(
    checker: SelfCollisionChecker, point: dict[str, float | None], seed: dict[str, float]
) -> list[dict[str, float] | None]:
    """The Python IK solution of every pitch point_reach tries (None where it raises UnreachableError).

    Args:
        checker (SelfCollisionChecker): Body checker (kinematics, mount).
        point (dict[str, float | None]): x, y, z (base_link, m) and pitch (rad) or None for every POINT_PITCHES_DEG.
        seed (dict[str, float]): IK seed.

    Returns:
        list[dict[str, float] | None]: Measured joint solutions, one per pitch.
    """
    target = base_link_to_arm(
        (float(point["x"] or 0.0), float(point["y"] or 0.0), float(point["z"] or 0.0)), checker.mount
    )
    pitch = point.get("pitch")
    pitches = [float(pitch)] if pitch is not None else [math.radians(p) for p in POINT_PITCHES_DEG]
    out: list[dict[str, float] | None] = []
    for p in pitches:
        try:
            out.append(checker.kin.inverse(*target, p, seed=seed))
        except UnreachableError:
            out.append(None)
    return out


def run_point_reach(robot: dict[str, Any], point: dict[str, float | None], seed: dict[str, float]) -> dict[str, Any]:
    """Golden outputs of point_reach: its result plus the IK solution of every pitch.

    Args:
        robot (dict[str, Any]): robot_inputs.
        point (dict[str, float | None]): Tool point (base_link) and optional pitch.
        seed (dict[str, float]): IK seed.

    Returns:
        dict[str, Any]: The point_reach result with 'solutions'.
    """
    checker = checker_for(robot)
    out = point_reach(checker, point, seed, ReachEnvelope(checker, checker.settings))
    return out | {"solutions": point_solutions(checker, point, seed)}


def reach_map_outputs(rmap: ReachMap) -> dict[str, Any]:
    """The compared outputs of a reach map (everything but its timing and creation time).

    Args:
        rmap (ReachMap): Built map.

    Returns:
        dict[str, Any]: pitch_mask, clearance, counts, grid, key.
    """
    meta = rmap.meta
    return {
        "pitch_mask": rmap.pitch_mask,
        "clearance": rmap.clearance,
        "counts": meta["counts"],
        "grid": meta["grid"],
        "key": meta["key"],
    }


def run_reach_map_build(robot: dict[str, Any], settings: dict[str, Any]) -> dict[str, Any]:
    """Golden outputs of reach_map.build.

    Args:
        robot (dict[str, Any]): robot_inputs.
        settings (dict[str, Any]): ReachMapSettings fields (file left at its default).

    Returns:
        dict[str, Any]: reach_map_outputs of the map.
    """
    return reach_map_outputs(build(checker_for(robot), ReachMapSettings(**settings)))


# Kernel name -> reference.
KERNELS: dict[str, ReferenceKernel] = {
    "point_reach": ReferenceKernel(
        run=run_point_reach,
        inputs="point_reach_cases",
        tolerances={"solutions": Tolerance(atol=SOLUTION_TOLERANCE_RAD)},
    ),
    "reach_map_build": ReferenceKernel(
        run=run_reach_map_build,
        inputs="reach_map_cases",
        tolerances={"clearance": Tolerance(atol=CLEARANCE_TOLERANCE_M)},
    ),
}
