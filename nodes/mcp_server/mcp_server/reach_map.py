"""Offline inverse reachability map of the arm with the robot body (forward sampled, RM4D-style).

Grid (ReachMapSettings): tool point bearing from the shoulder_pan axis in base_link ("heading", every heading_step_deg
inside the bearings the pan range minus its limit margin covers) x horizontal distance from the pan axis (r_step_m ..
r_max_m) x tool height in robot base_link (floor = 0; z_step_m .. z_max_m) x approach pitch (POINT_PITCHES_DEG).

Build: forward kinematics of an even planar_samples grid of (shoulder_lift, elbow_flex, wrist_flex) inside their
limits (minus the margin) at pan 0, wrist_roll 0, gripper closed. The pan axis is vertical (pan_geometry checks it), so
turning the pan only rotates a sample about the axis: its r, z and pitch stay, its bearing moves by sign * pan. A sample
belongs to the cell nearest its (r, z) and to the nearest pitch within half the pitch spacing. Per heading, every sample
whose required pan lies inside the pan range is a candidate; the samples_per_cell nearest each cell centre and pitch are
body checked at that pan (ENVELOPE_BATCH_SAMPLES at a time). A cell's pitch bit is set when one of its checked samples
keeps the hard margin (SelfCollisionChecker.clear_of's verdict); clearance holds the best body clearance of those samples,
saturated at soft_margin_m. No numeric IK is used: a cell is reachable only where a real FK sample landed.

Stored as .npz (pitch_mask uint16 [H, R, Z], clearance float16 [H, R, Z], meta JSON) under a key of every input
(reach_inputs, tool point, jaw, margins, grid, REACH_MAP_VERSION): ReachMap.load refuses a file built for other inputs.

CLI: ``python -m mcp_server.reach_map build --client-yml ansible/group_vars/client.yml --out <npz>`` builds the map from
the mcp_server node config of that file; ``... check --client-yml <yml> [--file <npz>]`` exits 1 when the file is
missing or stale.
"""

import argparse
import datetime
import hashlib
import json
import logging
import math
import os
import sys
import time
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .arm_reach import ENVELOPE_BATCH_SAMPLES, POINT_PITCHES_DEG, SHOULDER_PAN, joint_range, reach_inputs
from .config import REPO_ROOT, McpServerConfig, ReachMapSettings
from .floor_guard import JawModel
from .ik import ArmKinematics
from .self_collision import SampleGeometry, SelfCollisionChecker

REACH_MAP_VERSION = 1
PLANAR_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")
PITCH_HALF_SPACING_DEG = (POINT_PITCHES_DEG[1] - POINT_PITCHES_DEG[0]) / 2.0
# Pan axis checks: FK at a pan angle equals the pan 0 FK rotated about the pan joint axis within this (m), and the axis
# is vertical within PAN_TILT_TOLERANCE_RAD. The URDF's rpy 3.14159 (pi to 5 decimals) tilts it 2.65e-6 rad: 1.8 um at
# full reach when the map rotates samples about the exactly vertical line instead, far below a grid cell.
PAN_ROTATION_TOLERANCE_M = 1e-6
PAN_TILT_TOLERANCE_RAD = 1e-4
PAN_PROBE_ANGLES_RAD = (-1.0, -0.4, 0.5, 1.2)
# (shoulder_lift, elbow_flex, wrist_flex) probe poses, measured rad: tool point away from the pan axis.
PAN_PROBE_POSES = ((0.0, 0.0, 0.0), (0.5, 0.6, -0.3), (-0.8, 1.2, 0.4))
GRID_DECIMALS = 6
MCP_SERVER_NODE = "mcp_server"
DEFAULT_CLIENT_YML = Path("ansible/group_vars/client.yml")
REGENERATE_COMMAND = (
    "uv run --project nodes/mcp_server python -m mcp_server.reach_map build "
    "--client-yml ansible/group_vars/client.yml --out nodes/mcp_server/mcp_server/data/reach_map.npz"
)

log = logging.getLogger(__name__)


class ReachMapMissing(RuntimeError):
    """Raised when the reach map file does not exist or cannot be read."""


class ReachMapStale(RuntimeError):
    """Raised when the reach map file was built for other inputs than the current config."""

    def __init__(self, expected: str, found: str) -> None:
        """Keep both keys.

        Args:
            expected (str): Key of the current inputs.
            found (str): Key stored in the file.
        """
        super().__init__(
            f"reach map is stale: expected key {expected}, found {found}; regenerate it: {REGENERATE_COMMAND}"
        )
        self.expected = expected
        self.found = found


@dataclass(frozen=True)
class PanGeometry:
    """The shoulder_pan axis in robot base_link.

    Attributes:
        axis (np.ndarray): A point on the pan axis, base_link (m).
        direction (np.ndarray): Unit direction of the pan joint axis in base_link (vertical up to the URDF rounding).
        sign (float): +1 when a positive measured pan turns the arm counter-clockwise seen from above, else -1.
        ref_bearing_deg (float): Bearing of the tool point of the zero planar pose at pan 0 (deg, base_link).
    """

    axis: np.ndarray
    direction: np.ndarray
    sign: float
    ref_bearing_deg: float


@dataclass(frozen=True)
class CellSet:
    """Reachable cells of one height slice (ReachMap.cells); arrays are parallel, one entry per (heading, r) cell.

    Attributes:
        z_m (float): Height of the slice (base_link z, m).
        heading_deg (np.ndarray): Tool bearing from the pan axis (deg, base_link).
        r_m (np.ndarray): Horizontal distance from the pan axis (m).
        pitches (list[list[float]]): Reachable, collision-free approach pitches of each cell (deg, + down).
        clearance (np.ndarray): Best body clearance of the cell (m, saturated at soft_margin_m).
        margin_m (np.ndarray): Horizontal distance from the cell centre to the nearest unreachable cell centre (m).
    """

    z_m: float
    heading_deg: np.ndarray
    r_m: np.ndarray
    pitches: list[list[float]]
    clearance: np.ndarray
    margin_m: np.ndarray


def checker_from_config(config: McpServerConfig) -> SelfCollisionChecker:
    """Kinematics, jaw and body checker exactly as the node builds them (URDF, margin, offsets, tool point, limits).

    Args:
        config (McpServerConfig): Node configuration.

    Returns:
        SelfCollisionChecker: The checker.
    """
    kin = ArmKinematics(
        config.arm.urdf_path,
        margin=config.limits.arm_limit_margin_rad,
        joint_offsets=config.arm.joint_offsets_rad.model_dump(),
        tool_offset=tuple(config.arm.tool_offset_m.model_dump().values()),
        limit_overrides=config.arm.joint_limit_overrides_rad,
    )
    jaw = JawModel(kin, config.arm.jaw_open_axis, config.arm.gripper_closed_rad)
    return SelfCollisionChecker.from_config(kin, config, jaw)


def config_from_client_yml(path: Path) -> McpServerConfig:
    """The mcp_server node config of an ansible group_vars file (ros2_nodes entry named mcp_server).

    Args:
        path (Path): group_vars YAML (e.g. ansible/group_vars/client.yml).

    Returns:
        McpServerConfig: Validated config.

    Raises:
        ValueError: When the file has no mcp_server node.
    """
    doc = yaml.safe_load(path.read_text()) or {}
    node = next((n for n in doc.get("ros2_nodes") or [] if n.get("name") == MCP_SERVER_NODE), None)
    if node is None:
        raise ValueError(f"{path} has no ros2_nodes entry named {MCP_SERVER_NODE}")
    raw = node.get("config") or {}
    data = yaml.safe_load(raw) if isinstance(raw, str) else raw
    return McpServerConfig.model_validate(data or {})


def reach_map_key(checker: SelfCollisionChecker, settings: ReachMapSettings) -> str:
    """Hash of every input of the map: a change of any of them makes a stored map stale.

    Args:
        checker (SelfCollisionChecker): Body checker (body, mount, kinematics, jaw, margins).
        settings (ReachMapSettings): Grid (file is not part of the key).

    Returns:
        str: Hex sha256 digest.
    """
    inputs = {
        "version": REACH_MAP_VERSION,
        "inputs": reach_inputs(checker),
        "tool_offset": [float(v) for v in checker.kin.tool_offset],
        "jaw": {"closed_rad": checker.jaw.closed_rad, "open_axis": [float(v) for v in checker.jaw.open_axis]},
        "margins": checker.settings.model_dump(include={"hard_margin_m", "soft_margin_m"}),
        "grid": settings.model_dump(exclude={"file"}),
        "pitches_deg": list(POINT_PITCHES_DEG),
    }
    return hashlib.sha256(json.dumps(inputs, sort_keys=True, default=str).encode()).hexdigest()


def tool_points(checker: SelfCollisionChecker, samples: list[dict[str, float]]) -> tuple[np.ndarray, np.ndarray]:
    """Tool points (base_link) and approach pitches of joint configurations.

    Args:
        checker (SelfCollisionChecker): Checker (kinematics and mount).
        samples (list[dict[str, float]]): Joint configurations (measured rad).

    Returns:
        tuple[np.ndarray, np.ndarray]: (N, 3) tool points (m) and (N,) pitches (rad, + down).
    """
    poses = [checker.kin.forward(s) for s in samples]
    arm = np.array([(p.x, p.y, p.z) for p in poses], dtype=np.float64).reshape(-1, 3)
    return arm @ checker.rotation.T + checker.offset, np.array([p.pitch for p in poses], dtype=np.float64)


def wrap_deg(angle: np.ndarray) -> np.ndarray:
    """Angles wrapped to [-180, 180).

    Args:
        angle (np.ndarray): Angles (deg).

    Returns:
        np.ndarray: Wrapped angles (deg).
    """
    return (np.asarray(angle) + 180.0) % 360.0 - 180.0


def rotate_about(points: np.ndarray, point: np.ndarray, direction: np.ndarray, angle: float) -> np.ndarray:
    """Rotate points about an axis line (Rodrigues).

    Args:
        points (np.ndarray): (N, 3) points.
        point (np.ndarray): A point on the axis.
        direction (np.ndarray): Unit axis direction.
        angle (float): Right-handed rotation about direction (rad).

    Returns:
        np.ndarray: (N, 3) rotated points.
    """
    rel = np.asarray(points, dtype=np.float64) - point
    c, s = math.cos(angle), math.sin(angle)
    turned = rel * c + np.cross(direction, rel) * s + np.outer(rel @ direction, direction) * (1.0 - c)
    return turned + point


def pan_geometry(checker: SelfCollisionChecker) -> PanGeometry:
    """The pan axis in base_link and its turning sense, checked to be a vertical rotation axis.

    Args:
        checker (SelfCollisionChecker): Checker (kinematics and mount).

    Returns:
        PanGeometry: Axis point and direction, sign and reference bearing.

    Raises:
        ValueError: When the axis tilts more than PAN_TILT_TOLERANCE_RAD from vertical, or FK at a pan angle is not
            the pan 0 FK rotated about the axis (within PAN_ROTATION_TOLERANCE_M): the forward-sampled map would be wrong.
    """
    kin = checker.kin
    frame = kin.chain.forward_kinematics(kin.to_vector({}), full_kinematics=True)[kin.index[SHOULDER_PAN]]
    direction = checker.rotation @ np.asarray(frame[:3, 2], dtype=np.float64)
    direction /= np.linalg.norm(direction)
    tilt = math.acos(min(1.0, abs(float(direction[2]))))
    if tilt > PAN_TILT_TOLERANCE_RAD:
        raise ValueError(
            f"the shoulder_pan axis is not vertical in base_link (tilted {math.degrees(tilt):.4f} deg): "
            "the reach map needs a vertical pan axis"
        )
    axis = checker.rotation @ np.array([*kin.pan_axis_xy, 0.0]) + checker.offset
    zero = [
        dict(zip(PLANAR_JOINTS, pose, strict=True)) | {SHOULDER_PAN: 0.0, "wrist_roll": 0.0} for pose in PAN_PROBE_POSES
    ]
    base, _ = tool_points(checker, zero)
    for angle in PAN_PROBE_ANGLES_RAD:
        points, _ = tool_points(checker, [pose | {SHOULDER_PAN: angle} for pose in zero])
        error = float(np.linalg.norm(points - rotate_about(base, axis, direction, angle), axis=1).max())
        if error > PAN_ROTATION_TOLERANCE_M:
            raise ValueError(
                f"the shoulder_pan joint is not a rotation about its axis in base_link (pan {angle:+.2f} rad is "
                f"{error * 1000:.3f} mm off): the reach map needs a vertical pan axis"
            )
    rel = base[0] - axis
    return PanGeometry(
        axis=axis,
        direction=direction,
        sign=1.0 if direction[2] > 0.0 else -1.0,
        ref_bearing_deg=math.degrees(math.atan2(rel[1], rel[0])),
    )


def grid_values(step: float, maximum: float) -> list[float]:
    """Cell centres step, 2 * step, ... up to maximum.

    Args:
        step (float): Spacing.
        maximum (float): Largest centre.

    Returns:
        list[float]: Centres, rounded to GRID_DECIMALS.
    """
    return [round(k * step, GRID_DECIMALS) for k in range(1, math.floor(maximum / step + 1e-9) + 1)]


def heading_grid(pan: PanGeometry, pan_lo: float, pan_hi: float, step_deg: float) -> list[float]:
    """Headings (tool bearings from the pan axis, base_link) every step_deg inside what the pan range covers.

    Args:
        pan (PanGeometry): Pan axis geometry.
        pan_lo (float): Lowest usable measured pan (rad).
        pan_hi (float): Highest usable measured pan (rad).
        step_deg (float): Heading spacing (deg).

    Returns:
        list[float]: Headings (deg), multiples of step_deg.
    """
    ends = sorted(pan.ref_bearing_deg + pan.sign * math.degrees(v) for v in (pan_lo, pan_hi))
    first, last = math.ceil(ends[0] / step_deg - 1e-9), math.floor(ends[1] / step_deg + 1e-9)
    return [round(k * step_deg, GRID_DECIMALS) for k in range(first, last + 1)]


def body_clearance(checker: SelfCollisionChecker, geometry: SampleGeometry) -> np.ndarray:
    """Smallest hard body clearance of each sample, saturated at soft_margin_m.

    One pruned pass (bounded_clearances to soft_margin_m): every value up to the soft margin is exact, so the verdict
    clearance >= hard_margin_m is exactly SelfCollisionChecker.clear_of(geometry, hard_margin_m) (clear_of is the same
    pass bounded at the hard margin), and the clearance comes at no second pass.

    Args:
        checker (SelfCollisionChecker): Body checker.
        geometry (SampleGeometry): checker.geometry(samples).

    Returns:
        np.ndarray: (N,) clearance (m), at most soft_margin_m.
    """
    cap = checker.settings.soft_margin_m
    if not checker.body.obstacles:
        return np.full(geometry.n, cap)
    limit = np.full((geometry.n, len(checker.body.obstacles)), cap)
    return np.minimum(checker.bounded_clearances(geometry, limit, soft=False)[0].min(axis=1), cap)


def rank_in_group(groups: np.ndarray) -> np.ndarray:
    """Position of each entry within its run of equal values (groups sorted).

    Args:
        groups (np.ndarray): (N,) sorted group ids.

    Returns:
        np.ndarray: (N,) 0 for the first entry of a group, 1 for the next, ...
    """
    if not len(groups):
        return np.zeros(0, dtype=np.int64)
    index = np.arange(len(groups))
    starts = np.r_[True, groups[1:] != groups[:-1]]
    return index - np.maximum.accumulate(np.where(starts, index, 0))


class ReachMap:
    """Collision-free reachable cells of the arm (see the module docstring)."""

    def __init__(self, pitch_mask: np.ndarray, clearance: np.ndarray, meta: dict[str, Any]) -> None:
        """Hold the arrays.

        Args:
            pitch_mask (np.ndarray): uint16 [H, R, Z], bit k = POINT_PITCHES_DEG[k] reachable and collision-free.
            clearance (np.ndarray): float16 [H, R, Z], best body clearance (m), NaN where no pitch works.
            meta (dict[str, Any]): key, version, grid (headings_deg, r_m, z_m, r_step_m, z_step_m, heading_step_deg,
                pitches_deg, pan_axis_base_link), planar_samples, samples_per_cell, compute_s, created_at, counts.
        """
        self.pitch_mask = pitch_mask
        self.clearance = clearance
        self.meta = meta

    @property
    def key(self) -> str:
        """Key of the inputs the map was built for.

        Returns:
            str: Hex digest.
        """
        return str(self.meta["key"])

    @property
    def grid(self) -> dict[str, Any]:
        """The grid description.

        Returns:
            dict[str, Any]: meta["grid"].
        """
        return self.meta["grid"]

    def save(self, path: Path) -> None:
        """Write the map as compressed .npz atomically (temporary file, then rename).

        Args:
            path (Path): Target file.
        """
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(path.name + ".tmp")
        with tmp.open("wb") as handle:
            np.savez_compressed(
                handle,
                pitch_mask=self.pitch_mask,
                clearance=self.clearance,
                meta=np.array(json.dumps(self.meta, sort_keys=True)),
            )
        os.replace(tmp, path)

    @classmethod
    def load(cls, path: Path, checker: SelfCollisionChecker, settings: ReachMapSettings) -> "ReachMap":
        """Read a map and require it to match the current inputs.

        Args:
            path (Path): .npz file.
            checker (SelfCollisionChecker): Current body checker.
            settings (ReachMapSettings): Current grid.

        Returns:
            ReachMap: The map.

        Raises:
            ReachMapMissing: When the file is absent or unreadable.
            ReachMapStale: When its key differs from reach_map_key(checker, settings).
        """
        if not path.exists():
            raise ReachMapMissing(f"no reach map at {path}; generate it: {REGENERATE_COMMAND}")
        try:
            with np.load(path, allow_pickle=False) as data:
                rmap = cls(data["pitch_mask"], data["clearance"], json.loads(str(data["meta"])))
        except (OSError, ValueError, KeyError, zipfile.BadZipFile) as exc:
            raise ReachMapMissing(f"unreadable reach map {path} ({exc}); regenerate it: {REGENERATE_COMMAND}") from exc
        expected = reach_map_key(checker, settings)
        if rmap.meta.get("key") != expected:
            raise ReachMapStale(expected, str(rmap.meta.get("key")))
        return rmap

    def cells(self, z: float, pitch_deg: float | None) -> CellSet:
        """Reachable cells at a height, optionally for one approach pitch, with their margins.

        Args:
            z (float): Tool height (base_link z, m); the nearest slice within half a z step, else nothing is reachable.
            pitch_deg (float | None): Approach pitch (deg, + down), matched to the nearest POINT_PITCHES_DEG within
                half their spacing; None for any pitch.

        Returns:
            CellSet: The reachable cells of the slice (empty outside the grid).

        Raises:
            ValueError: For a pitch farther than half the spacing from every grid pitch.
        """
        grid = self.grid
        headings = np.array(grid["headings_deg"], dtype=np.float64)
        radii = np.array(grid["r_m"], dtype=np.float64)
        heights = np.array(grid["z_m"], dtype=np.float64)
        pitches = list(grid["pitches_deg"])
        bit: int | None = None
        if pitch_deg is not None:
            k = int(np.argmin(np.abs(np.array(pitches) - pitch_deg)))
            if abs(pitches[k] - pitch_deg) > PITCH_HALF_SPACING_DEG + 1e-9:
                raise ValueError(f"pitch {pitch_deg:g} deg is outside the map pitches {pitches}")
            bit = 1 << k
        zi = int(np.argmin(np.abs(heights - z)))
        if abs(heights[zi] - z) > grid["z_step_m"] / 2.0 + 1e-9:
            empty = np.zeros(0)
            return CellSet(float(z), empty, empty, [], empty, empty)
        masks = self.pitch_mask[:, :, zi].astype(np.int64)
        ok = masks > 0 if bit is None else (masks & bit) > 0
        hi, ri = np.nonzero(ok)
        bad_h, bad_r = np.nonzero(~ok)
        unreachable = [np.column_stack([headings[bad_h], radii[bad_r]])]
        # Outside the grid nothing is reachable: the ring one r step beyond r_max and, unless the headings close the
        # circle, the rows one heading step beyond each end.
        step_h, outer = float(grid["heading_step_deg"]), radii[-1] + grid["r_step_m"]
        rows = [headings]
        if headings[-1] - headings[0] + step_h < 360.0 - 1e-9:
            extra = np.array([headings[0] - step_h, headings[-1] + step_h])
            unreachable.append(np.array([(h, r) for h in extra for r in radii]))
            rows.append(extra)
        unreachable.append(np.column_stack([np.concatenate(rows), np.full(sum(len(r) for r in rows), outer)]))
        bad = np.concatenate(unreachable)
        good_xy = polar(headings[hi], radii[ri])
        bad_xy = polar(bad[:, 0], bad[:, 1])
        margin = np.sqrt(((good_xy[:, None, :] - bad_xy[None, :, :]) ** 2).sum(axis=2)).min(axis=1)
        return CellSet(
            z_m=float(heights[zi]),
            heading_deg=headings[hi],
            r_m=radii[ri],
            pitches=[[p for k, p in enumerate(pitches) if m >> k & 1] for m in masks[hi, ri]],
            clearance=self.clearance[hi, ri, zi].astype(np.float64),
            margin_m=margin,
        )

    def describe(self) -> dict[str, Any]:
        """JSON summary for tools and the CLI.

        Returns:
            dict[str, Any]: key, version, created_at, compute_s, planar_samples, samples_per_cell, grid, counts.
        """
        fields = ("key", "version", "created_at", "compute_s", "planar_samples", "samples_per_cell", "grid", "counts")
        return {f: self.meta.get(f) for f in fields}


def polar(heading_deg: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Horizontal cell centres relative to the pan axis.

    Args:
        heading_deg (np.ndarray): Bearings (deg).
        r (np.ndarray): Distances (m).

    Returns:
        np.ndarray: (N, 2) x, y (m).
    """
    angle = np.radians(heading_deg)
    return np.column_stack([r * np.cos(angle), r * np.sin(angle)])


def build(checker: SelfCollisionChecker, settings: ReachMapSettings) -> ReachMap:
    """Forward-sample the map (see the module docstring).

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


def main(argv: list[str] | None = None) -> int:
    """CLI: build or check the map for the mcp_server config of a group_vars file.

    Args:
        argv (list[str] | None): Arguments (None: sys.argv).

    Returns:
        int: Exit code (check: 1 when the file is missing or stale).
    """
    parser = argparse.ArgumentParser(prog="python -m mcp_server.reach_map", description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    for name, target, text in (("build", "--out", "map file to write"), ("check", "--file", "map file to check")):
        cmd = sub.add_parser(name)
        cmd.add_argument("--client-yml", type=Path, default=REPO_ROOT / DEFAULT_CLIENT_YML)
        cmd.add_argument(target, type=Path, default=None, help=f"{text} (default: the config's reach_map.file)")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    config = config_from_client_yml(args.client_yml)
    checker = checker_from_config(config)
    settings = config.reach_map
    if args.command == "build":
        out = args.out or settings.file
        rmap = build(checker, settings)
        rmap.save(out)
        print(json.dumps(rmap.describe() | {"file": str(out)}, indent=2))
        return 0
    path = args.file or settings.file
    try:
        rmap = ReachMap.load(path, checker, settings)
    except (ReachMapMissing, ReachMapStale) as exc:
        print(f"{path}: {exc}")
        return 1
    print(f"{path}: up to date (key {rmap.key}, created {rmap.meta.get('created_at')})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
