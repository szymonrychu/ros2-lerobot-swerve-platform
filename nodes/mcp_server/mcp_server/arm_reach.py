"""Arm reach with the robot body: the collision-free reach envelope and per-point reach checks (get_arm_reach).

Envelope grid (BodyCheckSettings): shoulder_pan headings every reach_heading_step_deg across the pan limits (minus the
limit margin) x tool heights 0 .. reach_height_max_m every reach_height_step_m (base_link z). At every heading the
arm joints shoulder_lift x elbow_flex x wrist_flex are sampled on an even grid inside their limits (minus margin;
wrist_roll 0, gripper closed): ENVELOPE_SAMPLES_PER_JOINT, ENVELOPE_WRIST_SAMPLES. Every sample whose body clearance
is at least hard_margin_m (not rejected) and whose tool point lies within half a height step of a grid height gives
that cell's horizontal tool distance from the pan axis; a cell reports the min / max of them (null = no collision-free
pose there). It is a sampled approximation (about a centimetre at full reach), computed once in a background thread
started when the MCP server is built (ENVELOPE_BATCH_SAMPLES poses at a time to bound memory; minutes on the busy RPi5) and cached in memory and, with
BodyCheckSettings.reach_cache_file, on disk under a key of every input, so a restart loads it instead of recomputing.

Per point (base_link x, y, z and optional pitch): IK within the joint limits for the given pitch, or for every pitch
of POINT_PITCHES_DEG, the body clearance of each solution, the pitches that are reachable and collision-free, the
limiting obstacle/link and a suggestion when the point does not work. The IK and the body clearance run in the C++
kernel mcp_core_cpp.ik_reach.point_reach (the same solver, seeds and pruned clearance pass as ArmKinematics.inverse and
SelfCollisionChecker.closest, proven equal by tests/test_core_parity_ik_reach.py; the Python original is
mcp_server/reference/ik_reach.py).
"""

import atexit
import hashlib
import json
import logging
import math
import os
import threading
import time
import weakref
from pathlib import Path
from typing import Any

import numpy as np

import mcp_core_cpp

from .arm_geometry import LINK_ORIGINS
from .body_model import BoxObstacle, CylinderObstacle
from .config import BodyCheckSettings
from .floor_guard import GRIPPER_LINK, TOOL_FRAME_LINK, arm_to_base_link, base_link_to_arm
from .ik import ARM_CHAIN_JOINTS
from .self_collision import CAPSULES, CHECKED_PARTS, LINK_BLOCKS, SelfCollisionChecker, link_model_description

SHOULDER_PAN = "shoulder_pan"
# How long interpreter exit waits for a running envelope computation to stop (it stops at its next batch).
EXIT_STOP_TIMEOUT_S = 10.0
ENVELOPE_JOINTS = ("shoulder_lift", "elbow_flex")
ENVELOPE_SAMPLES_PER_JOINT = 13
ENVELOPE_WRIST_SAMPLES = 7
# Poses stacked and checked at once: a whole heading (1183 poses) peaked at about 2.4 GB against the unit's memory cap;
# 32 adds about 70 MB RSS (64: 145 MB, 16: 60 MB but 1.7x slower).
ENVELOPE_BATCH_SAMPLES = 32
POINT_PITCHES_DEG = (-30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0)
DEFAULT_SEED = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 1.2, "wrist_flex": 0.3, "wrist_roll": 0.0}
ROUND_M = 3
CACHE_VERSION = 1

log = logging.getLogger(__name__)
# The C++ body model of each checker (core_body) with the parameters it was built from.
CORE_BODIES: "weakref.WeakKeyDictionary[SelfCollisionChecker, tuple[Any, Any]]" = weakref.WeakKeyDictionary()


def joint_range(checker: SelfCollisionChecker, joint: str) -> tuple[float, float]:
    """Usable range of a chain joint in measured space (URDF limits minus the margin).

    Args:
        checker (SelfCollisionChecker): Checker (its kinematics).
        joint (str): Chain joint.

    Returns:
        tuple[float, float]: (low, high) rad.
    """
    kin = checker.kin
    lo, hi = kin.limits[joint]
    offset = kin.offsets[joint]
    return lo - offset + kin.margin, hi - offset - kin.margin


def reach_inputs(checker: SelfCollisionChecker) -> dict[str, Any]:
    """The arm and body inputs every reach computation depends on (envelope and reach map keys hash them).

    Args:
        checker (SelfCollisionChecker): Checker (body, mount, kinematics).

    Returns:
        dict[str, Any]: body, links (link collision model), mount, limits (URDF, rad), offsets (joint zero offsets,
            rad), margin (joint limit margin, rad); JSON-serialisable.
    """
    kin = checker.kin
    return {
        "body": checker.body.describe(),
        "links": link_model_description(),
        "mount": checker.mount.model_dump(),
        "limits": {k: list(v) for k, v in sorted(kin.limits.items())},
        "offsets": dict(sorted(kin.offsets.items())),
        "margin": kin.margin,
    }


class ReachEnvelope:
    """Lazily computed, cached collision-free reach envelope (see the module docstring)."""

    def __init__(
        self, checker: SelfCollisionChecker, settings: BodyCheckSettings, cache_file: Path | None = None
    ) -> None:
        """Bind to the checker; nothing is computed yet.

        Args:
            checker (SelfCollisionChecker): Body checker (kinematics, body, mount).
            settings (BodyCheckSettings): Grid and margins.
            cache_file (Path | None): JSON cache of the envelope on disk; None keeps it in memory only.
        """
        self.checker = checker
        self.settings = settings
        self.cache_file = cache_file
        self.compute_count = 0
        self._result: dict[str, Any] | None = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()

    def headings_deg(self) -> list[float]:
        """Pan headings of the grid (deg, measured shoulder_pan), symmetric about 0 within the pan range.

        Returns:
            list[float]: Headings.
        """
        lo, hi = joint_range(self.checker, SHOULDER_PAN)
        step = self.settings.reach_heading_step_deg
        n_lo, n_hi = math.ceil(math.degrees(lo) / step - 1e-9), math.floor(math.degrees(hi) / step + 1e-9)
        return [k * step for k in range(n_lo, n_hi + 1)]

    def heights_m(self) -> list[float]:
        """Tool heights of the grid (base_link z, m).

        Returns:
            list[float]: Heights from 0 to reach_height_max_m.
        """
        step = self.settings.reach_height_step_m
        n = math.floor(self.settings.reach_height_max_m / step + 1e-9)
        return [round(k * step, 6) for k in range(n + 1)]

    def cache_key(self) -> str:
        """Hash of every input of the envelope: a change of any of them invalidates the disk cache.

        Returns:
            str: Hex digest.
        """
        inputs = {
            "version": CACHE_VERSION,
            **reach_inputs(self.checker),
            "settings": self.settings.model_dump(
                include={"hard_margin_m", "reach_heading_step_deg", "reach_height_max_m", "reach_height_step_m"}
            ),
            "samples": [ENVELOPE_SAMPLES_PER_JOINT, ENVELOPE_WRIST_SAMPLES],
        }
        return hashlib.sha256(json.dumps(inputs, sort_keys=True, default=str).encode()).hexdigest()

    def load_cache(self) -> dict[str, Any] | None:
        """The envelope from the disk cache when its key matches the current inputs.

        Returns:
            dict[str, Any] | None: The cached envelope, or None (no file, other key, unreadable).
        """
        if self.cache_file is None or not self.cache_file.exists():
            return None
        try:
            data = json.loads(self.cache_file.read_text())
        except (OSError, ValueError) as exc:
            log.warning("ignoring the reach envelope cache %s: %s", self.cache_file, exc)
            return None
        if not isinstance(data, dict) or data.get("key") != self.cache_key():
            return None
        envelope = data.get("envelope")
        return envelope if isinstance(envelope, dict) else None

    def save_cache(self, envelope: dict[str, Any]) -> None:
        """Write the envelope to the disk cache atomically; a failure is logged, the in-memory result stays.

        Args:
            envelope (dict[str, Any]): Computed envelope.
        """
        if self.cache_file is None:
            return
        try:
            self.cache_file.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.cache_file.with_suffix(self.cache_file.suffix + ".tmp")
            tmp.write_text(json.dumps({"key": self.cache_key(), "envelope": envelope}))
            os.replace(tmp, self.cache_file)
        except OSError as exc:
            log.warning("could not write the reach envelope cache %s: %s", self.cache_file, exc)

    def start(self) -> None:
        """Load the envelope from the disk cache, or start computing it in a background thread (idempotent)."""
        with self._lock:
            if self._result is not None or self._thread is not None:
                return
            cached = self.load_cache()
            if cached is not None:
                self._result = cached
                return
            self._thread = threading.Thread(target=self.compute, name="arm-reach-envelope", daemon=True)
            RUNNING.add(self)
            self._thread.start()

    def stop(self, timeout_s: float) -> None:
        """Stop a running computation at its next batch and wait for its thread (no result is stored or cached).

        Args:
            timeout_s (float): How long to wait for the thread (s).
        """
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout_s)

    def get(self, wait_s: float) -> dict[str, Any] | None:
        """The envelope, starting its computation in a background thread on the first call.

        Args:
            wait_s (float): How long to wait for a running computation (s).

        Returns:
            dict[str, Any] | None: The envelope, or None while it is still being computed.
        """
        self.start()
        thread = self._thread
        if self._result is None and thread is not None:
            thread.join(wait_s)
        return self._result

    def compute(self) -> None:
        """Compute and cache the envelope (runs once)."""
        started = time.monotonic()
        checker, cfg = self.checker, self.settings
        grids = [np.linspace(*joint_range(checker, j), ENVELOPE_SAMPLES_PER_JOINT) for j in ENVELOPE_JOINTS]
        wrist = np.linspace(*joint_range(checker, "wrist_flex"), ENVELOPE_WRIST_SAMPLES)
        heights = self.heights_m()
        half = cfg.reach_height_step_m / 2.0
        pan_axis = arm_to_base_link((*checker.kin.pan_axis_xy, 0.0), checker.mount)
        rows = []
        for heading in self.headings_deg():
            samples = [
                {
                    SHOULDER_PAN: math.radians(heading),
                    "shoulder_lift": a,
                    "elbow_flex": b,
                    "wrist_flex": c,
                    "wrist_roll": 0.0,
                }
                for a, b, c in np.array(np.meshgrid(grids[0], grids[1], wrist)).reshape(3, -1).T
            ]
            tip_parts, clear_parts = [], []
            for start in range(0, len(samples), ENVELOPE_BATCH_SAMPLES):
                if self._stop.is_set():
                    return
                batch = samples[start : start + ENVELOPE_BATCH_SAMPLES]
                geometry = checker.geometry(batch)
                tip_parts.append(geometry.fixed["tool_point"][:, 0])
                clear_parts.append(checker.clear_of(geometry, cfg.hard_margin_m))
                del geometry
            tips = np.concatenate(tip_parts)
            clear = np.concatenate(clear_parts)
            radius = np.hypot(tips[:, 0] - pan_axis[0], tips[:, 1] - pan_axis[1])
            cells = []
            for z in heights:
                pick = clear & (np.abs(tips[:, 2] - z) <= half)
                cells.append(
                    {
                        "z_m": z,
                        "min_r_m": round(float(radius[pick].min()), ROUND_M) if pick.any() else None,
                        "max_r_m": round(float(radius[pick].max()), ROUND_M) if pick.any() else None,
                    }
                )
            rows.append({"heading_deg": heading, "cells": cells})
        result = {
            "status": "ready",
            "grid": {
                "heading_step_deg": cfg.reach_heading_step_deg,
                "headings_deg": self.headings_deg(),
                "heights_m": heights,
                "height_band_m": half,
                "joint_samples": [ENVELOPE_SAMPLES_PER_JOINT, ENVELOPE_SAMPLES_PER_JOINT, ENVELOPE_WRIST_SAMPLES],
            },
            "pan_axis_base_link": [round(float(v), 4) for v in pan_axis],
            "rows": rows,
            "compute_s": round(time.monotonic() - started, 2),
        }
        self.compute_count += 1
        self._result = result
        self.save_cache(result)

    def cell(self, heading_deg: float, z: float) -> dict[str, Any] | None:
        """Envelope cell nearest a heading and height (None until computed).

        Args:
            heading_deg (float): Pan heading (deg).
            z (float): Height (base_link z, m).

        Returns:
            dict[str, Any] | None: The cell with its heading.
        """
        if self._result is None:
            return None
        row = min(self._result["rows"], key=lambda r: abs(r["heading_deg"] - heading_deg))
        cell = min(row["cells"], key=lambda c: abs(c["z_m"] - z))
        return cell | {"heading_deg": row["heading_deg"]}


def obstacle_spec(obstacle: Any) -> tuple[str, list[float]]:
    """An obstacle as mcp_core_cpp.ik_reach.BodyModel takes it.

    Args:
        obstacle (Any): BoxObstacle or CylinderObstacle.

    Returns:
        tuple[str, list[float]]: ('box', lo + hi) or ('cylinder', [x, y, radius, z_min, z_max]).

    Raises:
        TypeError: For another obstacle type.
    """
    if isinstance(obstacle, BoxObstacle):
        return "box", [*map(float, obstacle.lo), *map(float, obstacle.hi)]
    if isinstance(obstacle, CylinderObstacle):
        x, y = obstacle.center_xy
        return "cylinder", [x, y, obstacle.radius, obstacle.z_min, obstacle.z_max]
    raise TypeError(f"the C++ body check knows boxes and vertical cylinders, not {type(obstacle).__name__}")


def core_body(checker: SelfCollisionChecker) -> Any:
    """The C++ body model of a checker (mcp_core_cpp.ik_reach.BodyModel: obstacles, mount, jaw, link capsules and
    point blocks, margins), rebuilt when the body, mount, jaw or margins changed since.

    Args:
        checker (SelfCollisionChecker): Body checker.

    Returns:
        Any: mcp_core_cpp.ik_reach.BodyModel.
    """
    kin, jaw = checker.kin, checker.jaw
    obstacles = [obstacle_spec(o) for o in checker.body.obstacles]
    key = (
        tuple(checker.rotation.ravel().tolist()),
        tuple(checker.offset.tolist()),
        tuple(jaw.closed_tip.tolist()),
        tuple(jaw.pivot.tolist()),
        tuple(jaw.axis.tolist()),
        jaw.sign,
        jaw.closed_rad,
        tuple((kind, tuple(values)) for kind, values in obstacles),
        checker.settings.hard_margin_m,
        checker.settings.soft_margin_m,
    )
    cached = CORE_BODIES.get(checker)
    if cached is not None and cached[0] == key:
        return cached[1]
    frame = kin.link_index
    model = mcp_core_cpp.ik_reach.BodyModel(
        rotation=checker.rotation.ravel().tolist(),
        offset=checker.offset.tolist(),
        jaw_closed_tip=jaw.closed_tip.tolist(),
        jaw_pivot=jaw.pivot.tolist(),
        jaw_axis=jaw.axis.tolist(),
        jaw_sign=jaw.sign,
        jaw_closed_rad=jaw.closed_rad,
        tool_frame=frame[TOOL_FRAME_LINK],
        gripper_frame=frame[GRIPPER_LINK],
        capsules=[(frame[LINK_ORIGINS[a]], frame[LINK_ORIGINS[b]], radius) for _name, a, b, radius in CAPSULES],
        groups=[
            (
                b.part,
                frame[b.link],
                b.contact_only,
                b.points.shape[0],
                b.points.shape[1],
                b.points.ravel().tolist(),
                b.centres.ravel().tolist(),
                b.radii.tolist(),
            )
            for b in LINK_BLOCKS
        ],
        obstacles=obstacles,
        hard_margin_m=checker.settings.hard_margin_m,
        soft_margin_m=checker.settings.soft_margin_m,
    )
    CORE_BODIES[checker] = (key, model)
    return model


def reach_pitches(pitch: float | None) -> list[float]:
    """The approach pitches point_reach tries.

    Args:
        pitch (float | None): One pitch (rad), or None for every POINT_PITCHES_DEG.

    Returns:
        list[float]: Pitches (rad).
    """
    return [float(pitch)] if pitch is not None else [math.radians(p) for p in POINT_PITCHES_DEG]


def reach_kernel(
    checker: SelfCollisionChecker, target: np.ndarray, pitches: list[float], seed: dict[str, float]
) -> dict[str, Any]:
    """Run the C++ point_reach kernel: ArmKinematics.inverse per pitch and SelfCollisionChecker.closest of the
    solutions found, keeping the kinematics' heading bias and solve count as the Python IK does.

    Args:
        checker (SelfCollisionChecker): Body checker.
        target (np.ndarray): Tool point in the arm base frame (m).
        pitches (list[float]): Approach pitches (rad).
        seed (dict[str, float]): IK seed (measured).

    Returns:
        dict[str, Any]: solutions (measured joint maps or None, per pitch) and closest ((clearance, obstacle, part)
            per solution found).
    """
    kin = checker.kin
    out = mcp_core_cpp.ik_reach.point_reach(
        kin.core_model(),
        core_body(checker),
        [float(v) for v in target],
        pitches,
        kin.urdf_seed(seed),
        kin.last_heading_bias,
    )
    kin.last_heading_bias = out["last_heading_bias"]
    kin.solve_count += out["solves"]
    names = [o.name for o in checker.body.obstacles]
    return {
        "solutions": [None if s is None else dict(zip(ARM_CHAIN_JOINTS, s, strict=True)) for s in out["solutions"]],
        "closest": [
            (value, names[k], CHECKED_PARTS[part]) if k >= 0 else (value, "", "") for value, k, part in out["closest"]
        ],
    }


def point_solutions(
    checker: SelfCollisionChecker, point: dict[str, float | None], seed: dict[str, float]
) -> list[dict[str, float] | None]:
    """The IK solution of every pitch point_reach tries (None where none is found).

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
    return reach_kernel(checker, target, reach_pitches(point.get("pitch")), seed)["solutions"]


# Envelopes whose computation thread was started; stopped at interpreter exit (stop_running_envelopes).
RUNNING: weakref.WeakSet[ReachEnvelope] = weakref.WeakSet()


@atexit.register
def stop_running_envelopes() -> None:
    """Stop every envelope computation still running at interpreter exit, so its daemon thread does not hold the C++
    body kernel (mcp_core_cpp.collision.BodyChecker) through shutdown, which nanobind reports as a leak."""
    for envelope in list(RUNNING):
        envelope.stop(EXIT_STOP_TIMEOUT_S)


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
    pitches = reach_pitches(pitch)
    found = reach_kernel(checker, target, pitches, seed)
    reachable = [p for p, s in zip(pitches, found["solutions"], strict=True) if s is not None]
    working: list[tuple[float, float]] = []
    limiting: dict[str, Any] | None = None
    for p, (clearance, obstacle, part) in zip(reachable, found["closest"], strict=True):
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
