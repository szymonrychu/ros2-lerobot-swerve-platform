"""Python references of the collision kernel family (registry read by tests/regenerate_core_golden.py).

Ported kernels (bench/report.json verdict 'port'): self_collision.path (SelfCollisionChecker.path) and
self_collision.pose (SelfCollisionChecker.pose). Their original Python bodies live here, moved from
mcp_server/self_collision.py with the clearance computations they run (capsule_distances, clearances_of,
clearance_matrix, bounded_clearances, pruned_path_clearances, hard_limit, moving_soft_clearances), each taking the
checker as its first argument instead of self. mcp_server calls the C++ kernels (mcp_core_cpp.collision.BodyChecker);
these functions are used only to write the goldens.
"""

import math
from collections.abc import Mapping
from typing import Any

import numpy as np

import bench.inputs
from mcp_server.body_model import Obstacle, capsule_signed_distance
from mcp_server.reference import ReferenceKernel, Tolerance
from mcp_server.self_collision import (
    CAPSULES,
    CHECKED_PARTS,
    ESCAPE_TOLERANCE_M,
    JAW_POINTS,
    LINK_BLOCKS,
    NORMAL_SCALE,
    PRUNE_SLACK_M,
    BodyReport,
    PoseClearance,
    SampleGeometry,
    SelfCollisionChecker,
    contact_key,
    obstacle_hint,
    rigid,
)

# Clearances are float sums of the same terms in another order (C++ vs numpy): 1 nm is far below any margin (mm).
CLEARANCE_TOLERANCE = Tolerance(atol=1e-9)
# summary / rejection hold clearances rounded to ROUND_M decimals: one unit of the last decimal covers a rounding tie.
ROUNDED_TOLERANCE = Tolerance(atol=1e-4)


def capsule_distances(obstacle: Obstacle, ends: list[np.ndarray]) -> list[np.ndarray]:
    """Signed distances of the CAPSULES to one obstacle in a single vectorised axis search (same values as one
    capsule_signed_distance call per capsule: the rows are searched independently).

    Args:
        obstacle (Obstacle): Convex obstacle.
        ends (list[np.ndarray]): Per capsule of CAPSULES, (R, >= 2, 3) points whose first two are its axis ends.

    Returns:
        list[np.ndarray]: Per capsule, (R,) distances (m).
    """
    axis = capsule_signed_distance(
        obstacle, np.concatenate([e[:, 0] for e in ends]), np.concatenate([e[:, 1] for e in ends]), 0.0
    )
    out, start = [], 0
    for e, (*_name, radius) in zip(ends, CAPSULES, strict=True):
        out.append(axis[start : start + len(e)] - radius)
        start += len(e)
    return out


def clearances_of(
    checker: SelfCollisionChecker, stacked: dict[str, np.ndarray]
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Clearance of stacked part points to every obstacle and the closest part.

    The hard check's clearance of a contact-only box is its signed distance plus hard_margin_m (it breaks the
    hard margin exactly when it touches); the soft clearance leaves those boxes out (they never slow a motion).

    Args:
        checker (SelfCollisionChecker): The checker.
        stacked (dict[str, np.ndarray]): stacked_points output.

    Returns:
        tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: (N, M) clearances (m) and (N, M) indices into
            CHECKED_PARTS of the closest part as the hard check sees them, then the same two without the
            contact-only boxes (the soft clearances the slow band uses).
    """
    n, m = stacked[CHECKED_PARTS[0]].shape[0], len(checker.body.obstacles)
    out = np.empty((n, m, len(CHECKED_PARTS)))
    for k, obstacle in enumerate(checker.body.obstacles):
        capsules = capsule_distances(obstacle, [stacked[name] for name, *_rest in CAPSULES])
        for p, (name, *_rest) in enumerate(CAPSULES):
            ends = stacked[name]
            out[:, k, p] = capsules[p]
            if ends.shape[1] > 2:
                out[:, k, p] = np.minimum(out[:, k, p], obstacle.signed_distance(ends[:, 2:]).min(axis=1))
        for p in range(len(CAPSULES), len(CHECKED_PARTS)):
            out[:, k, p] = obstacle.signed_distance(stacked[CHECKED_PARTS[p]]).min(axis=1)
    soft, soft_parts = out.min(axis=2), out.argmin(axis=2)
    for p, (name, *_rest) in enumerate(CAPSULES):
        if contact_key(name) in stacked:
            contact = np.stack(
                [o.signed_distance(stacked[contact_key(name)]).min(axis=1) for o in checker.body.obstacles], axis=1
            )
            out[:, :, p] = np.minimum(out[:, :, p], contact + checker.settings.hard_margin_m)
    return out.min(axis=2), out.argmin(axis=2), soft, soft_parts


def clearance_matrix(
    checker: SelfCollisionChecker, samples: list[dict[str, float]]
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Clearance of every sample to every obstacle and the closest part.

    Args:
        checker (SelfCollisionChecker): The checker.
        samples (list[dict[str, float]]): Joint configurations.

    Returns:
        tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: clearances_of output.
    """
    n, m = len(samples), len(checker.body.obstacles)
    if n == 0 or m == 0:
        empty, parts = np.zeros((n, m)), np.zeros((n, m), dtype=int)
        return empty, parts, empty, parts
    return clearances_of(checker, checker.stacked_points(samples))


def bounded_clearances(
    checker: SelfCollisionChecker, geometry: SampleGeometry, hard_limit: np.ndarray | None = None, soft: bool = True
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """clearance_matrix with bounding-sphere pruning, exact wherever a verdict or a reported value depends on it.

    Every capsule and every block of LINK_BLOCKS has a lower bound on its distance to an obstacle (bounding sphere:
    centre distance minus radius) and an upper bound (centre distance plus radius, a capsule axis end). Per sample
    the smallest upper bound bounds its smallest soft clearance from above. Only the block centres are placed in
    base_link for every sample; a capsule or block whose lower bound is above every threshold it could matter for
    is neither placed point by point nor evaluated.

    Args:
        checker (SelfCollisionChecker): The checker.
        geometry (SampleGeometry): geometry(samples) of at least one sample (the body has obstacles).
        hard_limit (np.ndarray | None): (N, M) hard clearance per sample and obstacle below which the hard values
            must be exact (the path limit); None for each sample's smallest hard clearance.
        soft (bool): Keep the soft band guarantee (the per-sample smallest soft clearance exact).

    Returns:
        tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: (N, M) hard clearances (m), (N, M) indices into
            CHECKED_PARTS, (N, M) soft clearances, (N, M) soft part indices.
    """
    obstacles = checker.body.obstacles
    n, m = geometry.n, len(obstacles)
    hm = checker.settings.hard_margin_m
    fixed, poses = geometry.fixed, geometry.poses
    centres = [
        checker.to_base_link(rigid(poses[b.link][0][:, None], poses[b.link][1][:, None], b.centres))
        for b in LINK_BLOCKS
    ]
    jaw_parts = range(len(CAPSULES) + 1, len(CHECKED_PARTS))
    out = np.full((n, m, len(CHECKED_PARTS)), np.inf)
    # Pass 1: bounds (and the jaw points, single points evaluated exactly).
    cap_lo = np.empty((m, len(CAPSULES), n))
    block_lo: list[list[np.ndarray]] = [[] for _ in range(m)]
    soft_up = np.full(n, np.inf)
    hard_up = np.full(n, np.inf)
    for k, obstacle in enumerate(obstacles):
        for p, (name, *_r, radius) in enumerate(CAPSULES):
            a, b = fixed[name][:, 0], fixed[name][:, 1]
            half = np.linalg.norm(b - a, axis=-1) / 2.0
            cap_lo[k, p] = obstacle.signed_distance((a + b) / 2.0) - half - radius
            soft_up = np.minimum(soft_up, np.minimum(obstacle.signed_distance(a), obstacle.signed_distance(b)) - radius)
        for blocks, centre in zip(LINK_BLOCKS, centres, strict=True):
            at = obstacle.signed_distance(centre)
            block_lo[k].append(at - blocks.radii)
            upper = (at + blocks.radii).min(axis=1)
            if blocks.contact_only:
                hard_up = np.minimum(hard_up, upper + hm)
            else:
                soft_up = np.minimum(soft_up, upper)
        for p in jaw_parts:
            out[:, k, p] = obstacle.signed_distance(fixed[CHECKED_PARTS[p]]).min(axis=1)
            soft_up = np.minimum(soft_up, out[:, k, p])
    hard_up = np.minimum(hard_up, soft_up)
    soft_t = soft_up + PRUNE_SLACK_M if soft else np.full(n, -np.inf)
    limit = hard_up[:, None] if hard_limit is None else hard_limit
    hard_t = np.broadcast_to(limit + PRUNE_SLACK_M, (n, m))
    need = np.maximum(soft_t[:, None], hard_t)  # (N, M): held parts can matter below this
    # Pass 2: exact distances of what can matter, the three capsules of an obstacle in one axis search.
    for k, obstacle in enumerate(obstacles):
        rows = [np.nonzero(cap_lo[k, p] <= need[:, k])[0] for p in range(len(CAPSULES))]
        if sum(len(r) for r in rows):
            found = capsule_distances(obstacle, [fixed[name][r] for r, (name, *_r) in zip(rows, CAPSULES, strict=True)])
            for p, (r, distances) in enumerate(zip(rows, found, strict=True)):
                out[r, k, p] = distances
    contact_min = np.full((n, m, len(CAPSULES)), np.inf)
    for g, blocks in enumerate(LINK_BLOCKS):
        required = hard_t - hm if blocks.contact_only else need
        near = np.stack([block_lo[k][g] <= required[:, k, None] for k in range(m)])  # (M, N, B)
        ni, bi = np.nonzero(near.any(axis=0))
        if not len(ni):
            continue
        rotation, translation = poses[blocks.link]
        placed = checker.to_base_link(rigid(rotation[ni][:, None], translation[ni][:, None], blocks.points[bi]))
        for k, obstacle in enumerate(obstacles):
            pick = near[k, ni, bi]
            if not pick.any():
                continue
            found = np.full(near.shape[1:], np.inf)
            found[ni[pick], bi[pick]] = obstacle.signed_distance(placed[pick]).min(axis=1)
            target = contact_min[:, k] if blocks.contact_only else out[:, k]
            target[:, blocks.part] = np.minimum(target[:, blocks.part], found.min(axis=1))
    soft_values, soft_parts = out.min(axis=2), out.argmin(axis=2)
    out[:, :, : len(CAPSULES)] = np.minimum(out[:, :, : len(CAPSULES)], contact_min + hm)
    return out.min(axis=2), out.argmin(axis=2), soft_values, soft_parts


def hard_limit(checker: SelfCollisionChecker, start: np.ndarray, allow_escape: bool) -> np.ndarray:
    """Per-obstacle hard limit of a path: the hard margin, or for a start already inside it (when it may escape)
    the start clearance minus ESCAPE_TOLERANCE_M.

    Args:
        checker (SelfCollisionChecker): The checker.
        start (np.ndarray): (M,) hard clearances of the start pose (m).
        allow_escape (bool): Whether a start inside the hard margin may move out.

    Returns:
        np.ndarray: (M,) limits (m).
    """
    cfg = checker.settings
    return np.where((start >= cfg.hard_margin_m) | (not allow_escape), cfg.hard_margin_m, start - ESCAPE_TOLERANCE_M)


def pruned_path_clearances(
    checker: SelfCollisionChecker,
    samples: list[dict[str, float]],
    allow_escape: bool,
    geometry: SampleGeometry | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """clearance_matrix of a path through bounded_clearances: first the start (exact below the hard margin, which
    is all its escape limits need), then the rest to those limits.

    Args:
        checker (SelfCollisionChecker): The checker.
        samples (list[dict[str, float]]): Joint samples, start first (at least one; the body has obstacles).
        allow_escape (bool): Whether a start inside the hard margin may move out (sets the limits).
        geometry (SampleGeometry | None): geometry(samples) when the caller has it already.

    Returns:
        tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: clearances_of output, exact where path() reads it.
    """
    m = len(checker.body.obstacles)
    geometry = geometry or checker.geometry(samples)
    start = bounded_clearances(checker, geometry.rows(slice(0, 1)), np.full((1, m), checker.settings.hard_margin_m))
    if len(samples) == 1:
        return start
    limit = hard_limit(checker, start[0][0], allow_escape)
    rest = bounded_clearances(checker, geometry.rows(slice(1, None)), np.broadcast_to(limit, (len(samples) - 1, m)))
    values, parts, soft, soft_parts = (np.concatenate([a, b]) for a, b in zip(start, rest, strict=True))
    return values, parts, soft, soft_parts


def moving_soft_clearances(
    checker: SelfCollisionChecker, geometry: SampleGeometry, moving_parts: tuple[str, ...]
) -> tuple[np.ndarray, np.ndarray]:
    """Soft clearances of the given single-point parts only (the parts a gripper-only motion moves).

    Args:
        checker (SelfCollisionChecker): The checker.
        geometry (SampleGeometry): geometry(samples).
        moving_parts (tuple[str, ...]): Jaw point parts (JAW_POINTS).

    Returns:
        tuple[np.ndarray, np.ndarray]: (N, M) clearances (m) and (N, M) indices into CHECKED_PARTS of the closest
            moving part.
    """
    n, m = geometry.n, len(checker.body.obstacles)
    out = np.empty((n, m, len(moving_parts)))
    for k, obstacle in enumerate(checker.body.obstacles):
        for p, part in enumerate(moving_parts):
            out[:, k, p] = obstacle.signed_distance(geometry.fixed[part]).min(axis=1)
    index = np.array([CHECKED_PARTS.index(part) for part in moving_parts])
    return out.min(axis=2), index[out.argmin(axis=2)]


def path(
    checker: SelfCollisionChecker,
    samples: list[dict[str, float]],
    allow_escape: bool = True,
    prune: bool = True,
    moving_parts: tuple[str, ...] | None = None,
) -> BodyReport:
    """Speed scale of every sample and the hard verdict of a trajectory (samples[0] = the start pose).

    Args:
        checker (SelfCollisionChecker): The checker.
        samples (list[dict[str, float]]): Joint samples the arm will follow, start first.
        allow_escape (bool): A start inside the hard margin may move out (the clearance must not get worse);
            False holds every sample to the hard margin (the planner's own waypoints).
        prune (bool): Bounding-sphere pruning (False: every distance in full).
        moving_parts (tuple[str, ...] | None): Single-point parts (JAW_POINTS) that move; None = every part moves.

    Returns:
        BodyReport: Scales, clearances, limiting obstacle/part per sample and the verdict.

    Raises:
        ValueError: When moving_parts names a part that is not a single jaw point.
    """
    if moving_parts is not None and not set(moving_parts) <= set(JAW_POINTS):
        raise ValueError(f"moving_parts must be jaw points {JAW_POINTS}, got {moving_parts}")
    cfg = checker.settings
    n = len(samples)
    if not cfg.enabled or n == 0:
        return BodyReport([NORMAL_SCALE] * n, [math.inf] * n, [""] * n, [""] * n, cfg, enabled=False)
    geometry = checker.geometry(samples) if checker.body.obstacles and (prune or moving_parts) else None
    if prune and geometry is not None:
        values, parts, soft, soft_parts = pruned_path_clearances(checker, samples, allow_escape, geometry)
    else:
        values, parts, soft, soft_parts = clearance_matrix(checker, samples)
    if moving_parts and geometry is not None:
        soft, soft_parts = moving_soft_clearances(checker, geometry, moving_parts)
    names = [o.name for o in checker.body.obstacles]
    worst = soft.argmin(axis=1)
    clearances = [float(soft[i, worst[i]]) for i in range(n)]
    report = BodyReport(
        scales=[cfg.slow_speed_scale if c < cfg.soft_margin_m else NORMAL_SCALE for c in clearances],
        clearances=clearances,
        obstacles=[names[int(worst[i])] for i in range(n)],
        parts=[CHECKED_PARTS[int(soft_parts[i, worst[i]])] for i in range(n)],
        settings=cfg,
        hints={o.name: obstacle_hint(o, "arm", cfg) for o in checker.body.obstacles},
    )
    limit = hard_limit(checker, values[0], allow_escape)
    broken = values < limit[None, :]
    if broken.any():
        i = int(np.argmax(broken.any(axis=1)))
        k = int(np.argmin(np.where(broken[i], values[i], np.inf)))
        part = CHECKED_PARTS[int(parts[i, k])]
        report.violation = i
        report.violation_obstacle = names[k]
        report.violation_part = part
        report.violation_clearance = float(values[i, k])
        report.hints[names[k]] = obstacle_hint(checker.body.obstacles[k], part, cfg)
    return report


def pose(checker: SelfCollisionChecker, joints: dict[str, float]) -> PoseClearance:
    """Clearance of one configuration to every obstacle (computed even when the check is disabled).

    Args:
        checker (SelfCollisionChecker): The checker.
        joints (dict[str, float]): Joint configuration.

    Returns:
        PoseClearance: Per obstacle (clearance, closest part).
    """
    values, parts, soft, _soft_parts = clearance_matrix(checker, [joints])
    obstacles = checker.body.obstacles
    return PoseClearance(
        {o.name: (float(values[0, k]), CHECKED_PARTS[int(parts[0, k])]) for k, o in enumerate(obstacles)},
        {o.name: float(soft[0, k]) for k, o in enumerate(obstacles)},
    )


def report_outputs(report: BodyReport) -> dict[str, Any]:
    """Golden outputs of a path report: every field but the hints (rejection carries the one that matters), the verdicts
    (rejected, slowed per sample) and both summaries.

    Args:
        report (BodyReport): A path() report.

    Returns:
        dict[str, Any]: Named outputs.
    """
    return {
        "scales": report.scales,
        "clearances": report.clearances,
        "obstacles": report.obstacles,
        "parts": report.parts,
        "enabled": report.enabled,
        "rejected": report.rejected,
        "slowed": [s < NORMAL_SCALE for s in report.scales],
        "violation": report.violation,
        "violation_obstacle": report.violation_obstacle,
        "violation_part": report.violation_part,
        "violation_clearance": report.violation_clearance,
        "summary": report.summary(),
        "rejection": report.rejection(),
    }


def pose_outputs(clearance: PoseClearance) -> dict[str, Any]:
    """Golden outputs of a pose clearance: per obstacle (clearance, part), soft clearances, the limiting obstacle and
    part, and the verdicts against the deployed margins.

    Args:
        clearance (PoseClearance): A pose() result.

    Returns:
        dict[str, Any]: Named outputs.
    """
    settings = bench.inputs.config().body_check
    return {
        "per_obstacle": {name: list(value) for name, value in clearance.per_obstacle.items()},
        "soft": clearance.soft,
        "min_clearance": clearance.min_clearance,
        "min_soft_clearance": clearance.min_soft_clearance,
        "obstacle": clearance.obstacle,
        "part": clearance.part,
        "hard_clear": clearance.min_clearance >= settings.hard_margin_m,
        "soft_clear": clearance.min_soft_clearance >= settings.soft_margin_m,
    }


def path_reference(
    samples: list[dict[str, float]], allow_escape: bool, prune: bool, moving_parts: list[str] | None
) -> Mapping[str, Any]:
    """Golden run of self_collision.path: the original path() on the deployed checker.

    Args:
        samples (list[dict[str, float]]): Joint samples, start first.
        allow_escape (bool): path() allow_escape.
        prune (bool): path() prune.
        moving_parts (list[str] | None): path() moving_parts (a list in the golden).

    Returns:
        Mapping[str, Any]: report_outputs.
    """
    moving = None if moving_parts is None else tuple(moving_parts)
    return report_outputs(path(bench.inputs.checker(), samples, allow_escape, prune, moving))


def pose_reference(joints: dict[str, float]) -> Mapping[str, Any]:
    """Golden run of self_collision.pose: the original pose() on the deployed checker.

    Args:
        joints (dict[str, float]): Joint configuration.

    Returns:
        Mapping[str, Any]: pose_outputs.
    """
    return pose_outputs(pose(bench.inputs.checker(), joints))


# Kernel name -> reference; a port of a collision kernel to mcp_core_cpp.collision adds its entry here.
KERNELS: dict[str, ReferenceKernel] = {
    "self_collision.path": ReferenceKernel(
        run=path_reference,
        inputs="collision_path_cases",
        tolerances={
            "clearances": CLEARANCE_TOLERANCE,
            "violation_clearance": CLEARANCE_TOLERANCE,
            "summary": ROUNDED_TOLERANCE,
            "rejection": ROUNDED_TOLERANCE,
        },
    ),
    "self_collision.pose": ReferenceKernel(
        run=pose_reference,
        inputs="collision_pose_cases",
        tolerances={
            "per_obstacle": CLEARANCE_TOLERANCE,
            "soft": CLEARANCE_TOLERANCE,
            "min_clearance": CLEARANCE_TOLERANCE,
            "min_soft_clearance": CLEARANCE_TOLERANCE,
        },
    ),
}
