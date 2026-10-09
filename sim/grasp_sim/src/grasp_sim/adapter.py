"""Tolerant adapter from a (future) mcp_server GraspPlan JSON to the replay plan format.

The expected schema is documented in sim/README.md. The adapter accepts the documented keys plus common
aliases, so small differences in the planner's output do not need code changes here.
"""

import json
import math
from pathlib import Path
from typing import Any

from grasp_sim.plan import JOINT_ORDER, LABELS

WAYPOINT_KEYS = ("waypoints", "segments", "trajectory", "samples", "phases", "steps")
SAMPLE_KEYS = ("joint_samples", "samples", "points", "trajectory")
LABEL_KEYS = ("label", "phase", "name", "type")
ABSOLUTE_TIME_KEYS = ("t", "time", "t_s", "time_s", "time_from_start", "time_from_start_s")
DURATION_KEYS = ("duration_s", "duration", "dt", "dt_s")
JOINT_VALUE_KEYS = ("joints", "joint_positions", "positions", "position", "q")
GRIPPER_KEYS = ("gripper", "gripper_rad")
JOINT_NAMES_KEYS = ("joint_names", "names")
JOINT_SUFFIXES = ("_joint", "_rad")
LABEL_ALIASES = {
    "open_gripper": "open",
    "pregrasp": "pre_grasp",
    "descend": "approach",
    "grasping": "grasp",
    "close_gripper": "close",
    "lifting": "lift",
    "retract": "retreat",
}
DEFAULT_SEGMENT_S = 1.0
RELATIVE_START_GAP_S = 1e-3


def first_key(mapping: dict[str, Any], keys: tuple[str, ...]) -> Any:
    """Value of the first of keys present in mapping, or None."""
    for key in keys:
        if key in mapping and mapping[key] is not None:
            return mapping[key]
    return None


def normalise_label(raw: Any) -> str | None:
    """Map a free-form phase name to a replay label, or None when it is not one.

    Args:
        raw (Any): Label text from the plan.

    Returns:
        str | None: One of grasp_sim.plan.LABELS, or None.
    """
    if not isinstance(raw, str):
        return None
    text = raw.strip().lower().replace("-", "_").replace(" ", "_")
    text = LABEL_ALIASES.get(text, text)
    return text if text in LABELS else None


def to_seconds(value: Any) -> float:
    """Convert a time value (number, or {"sec", "nanosec"}) to seconds.

    Args:
        value (Any): Time in s, or a ROS-style duration dict.

    Returns:
        float: Seconds.
    """
    if isinstance(value, dict):
        return float(value.get("sec", value.get("secs", 0))) + float(value.get("nanosec", value.get("nsecs", 0))) * 1e-9
    return float(value)


def clean_joint_name(name: str) -> str:
    """Strip _joint/_rad suffixes from a joint key."""
    for suffix in JOINT_SUFFIXES:
        if name.endswith(suffix) and name[: -len(suffix)] in JOINT_ORDER:
            return name[: -len(suffix)]
    return name


def read_joints(entry: dict[str, Any], names: list[str], scale: float) -> dict[str, float]:
    """Extract a joint-name -> rad mapping from a waypoint/sample dict.

    Args:
        entry (dict[str, Any]): Waypoint or sample.
        names (list[str]): Joint order for list-valued positions.
        scale (float): Multiplier to radians.

    Returns:
        dict[str, float]: Joints found (possibly partial, possibly empty).
    """
    raw = first_key(entry, JOINT_VALUE_KEYS)
    joints: dict[str, float] = {}
    if isinstance(raw, dict):
        joints = {clean_joint_name(k): float(v) * scale for k, v in raw.items()}
    elif isinstance(raw, list):
        joints = {clean_joint_name(n): float(v) * scale for n, v in zip(names, raw, strict=False)}
    gripper = first_key(entry, GRIPPER_KEYS)
    if gripper is not None:
        joints["gripper"] = float(gripper) * scale
    return joints


def load_raw(grasp_plan: str | Path | dict[str, Any] | list[Any]) -> Any:
    """Load JSON text, a JSON file path or an already parsed plan."""
    if isinstance(grasp_plan, Path):
        return json.loads(grasp_plan.read_text())
    if isinstance(grasp_plan, str):
        return json.loads(grasp_plan)
    return grasp_plan


def grasp_plan_to_replay(grasp_plan: str | Path | dict[str, Any] | list[Any]) -> list[dict[str, Any]]:
    """Convert a GraspPlan JSON into replay samples [{"t", "joints", "label"?}] in the measured joint space.

    Args:
        grasp_plan (str | Path | dict | list): JSON text, file path, or parsed plan: a list of waypoints or a dict
            holding one under waypoints/segments/trajectory/samples/phases/steps. Each waypoint has a label (label,
            phase, name or type), joints (dict of joint -> value or a list in joint_names order, plus an optional
            separate gripper), and a time (absolute t/time/time_from_start, or relative duration_s). A waypoint may
            instead hold joint_samples, each with its own t (absolute, or relative to the waypoint start when it
            restarts below the previous time). Optional top-level "units": "rad" (default) or "deg".

    Returns:
        list[dict[str, Any]]: Replay samples; the waypoint label sits on its first sample, unknown labels are dropped.

    Raises:
        ValueError: If no waypoints are found, the first sample lacks joints, or a waypoint has no joints.
    """
    raw = load_raw(grasp_plan)
    meta: dict[str, Any] = raw if isinstance(raw, dict) else {}
    waypoints = raw if isinstance(raw, list) else first_key(raw, WAYPOINT_KEYS)
    if not waypoints:
        raise ValueError("no waypoints found in the grasp plan")
    scale = math.pi / 180.0 if str(meta.get("units", "rad")).lower().startswith("deg") else 1.0
    default_names = [clean_joint_name(n) for n in (first_key(meta, JOINT_NAMES_KEYS) or JOINT_ORDER)]
    out: list[dict[str, Any]] = []
    held: dict[str, float] = {}
    end = 0.0
    for index, waypoint in enumerate(waypoints):
        names = [clean_joint_name(n) for n in (first_key(waypoint, JOINT_NAMES_KEYS) or default_names)]
        label = normalise_label(first_key(waypoint, LABEL_KEYS))
        sub = first_key(waypoint, SAMPLE_KEYS)
        entries = sub if isinstance(sub, list) and sub else [waypoint]
        for k, entry in enumerate(entries):
            joints = read_joints(entry, names, scale) or (
                read_joints(waypoint, names, scale) if entry is not waypoint else {}
            )
            if not joints:
                raise ValueError(f"waypoint {index} has no joint values")
            held = {**held, **joints}
            t = sample_time(entry, waypoint, index, end, entry is waypoint, not out)
            if out and t <= out[-1]["t"]:
                t = out[-1]["t"] + RELATIVE_START_GAP_S
            sample: dict[str, Any] = {"t": round(t, 6), "joints": dict(held) if out else dict(joints)}
            if k == 0 and label is not None:
                sample["label"] = label
            out.append(sample)
        end = out[-1]["t"]
    if set(out[0]["joints"]) != set(JOINT_ORDER):
        raise ValueError(f"first sample must give all joints {JOINT_ORDER}, got {sorted(out[0]['joints'])}")
    return out


def sample_time(
    entry: dict[str, Any], waypoint: dict[str, Any], index: int, end: float, whole: bool, first: bool
) -> float:
    """Absolute time of a sample.

    Args:
        entry (dict[str, Any]): The sample (or the waypoint itself when it has no joint_samples).
        waypoint (dict[str, Any]): Owning waypoint.
        index (int): Waypoint index.
        end (float): Time of the previous sample.
        whole (bool): True when the entry is the waypoint itself.
        first (bool): True for the very first sample of the plan.

    Returns:
        float: Time in s.
    """
    absolute = first_key(entry, ABSOLUTE_TIME_KEYS)
    if absolute is not None:
        t = to_seconds(absolute)
        # A sub-sample list that restarts below the previous time is relative to the waypoint start.
        if not whole and index > 0 and t < end:
            return end + RELATIVE_START_GAP_S + t
        return t
    duration = first_key(waypoint, DURATION_KEYS) if whole else first_key(entry, DURATION_KEYS)
    if duration is not None:
        return end + to_seconds(duration)
    return 0.0 if first else end + DEFAULT_SEGMENT_S
