"""Replay plan format: timed joint samples (measured joint space, rad) with optional segment labels."""

import json
from bisect import bisect_right
from pathlib import Path
from typing import Any

from pydantic import BaseModel, ConfigDict, field_validator

JOINT_ORDER = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
LABELS = ("open", "pre_grasp", "approach", "grasp", "close", "lift", "retreat")


class JointSample(BaseModel):
    """One timed joint target. The label (if any) applies from this sample until the next labelled one."""

    model_config = ConfigDict(extra="forbid")

    t: float
    joints: dict[str, float]
    label: str | None = None

    @field_validator("label")
    @classmethod
    def check_label(cls, value: str | None) -> str | None:
        """Reject labels outside LABELS."""
        if value is not None and value not in LABELS:
            raise ValueError(f"unknown label {value!r}, expected one of {LABELS}")
        return value

    @field_validator("joints")
    @classmethod
    def check_joint_names(cls, value: dict[str, float]) -> dict[str, float]:
        """Reject joint names outside JOINT_ORDER."""
        unknown = set(value) - set(JOINT_ORDER)
        if unknown:
            raise ValueError(f"unknown joint names {sorted(unknown)}, expected {JOINT_ORDER}")
        return value


def parse_plan(plan_json: str | Path | list[Any] | dict[str, Any]) -> list[JointSample]:
    """Parse and validate a replay plan.

    Args:
        plan_json (str | Path | list | dict): JSON text, a path to a JSON file, a list of samples or a dict with a
            "samples" list. Samples after the first may omit joints, which then hold their previous value.

    Returns:
        list[JointSample]: Validated samples with every joint filled in, strictly increasing in t.

    Raises:
        ValueError: On fewer than two samples, a partial first sample, non-increasing t, unknown labels or joints.
    """
    if isinstance(plan_json, Path):
        raw: Any = json.loads(plan_json.read_text())
    elif isinstance(plan_json, str):
        raw = json.loads(plan_json)
    else:
        raw = plan_json
    items = raw["samples"] if isinstance(raw, dict) else raw
    if len(items) < 2:
        raise ValueError("a plan needs at least two samples")
    samples: list[JointSample] = []
    held: dict[str, float] = {}
    for index, item in enumerate(items):
        sample = JointSample.model_validate(item)
        if index == 0 and set(sample.joints) != set(JOINT_ORDER):
            raise ValueError(f"first sample must give all joints {JOINT_ORDER}")
        if samples and sample.t <= samples[-1].t:
            raise ValueError("sample times must be strictly increasing")
        held = {**held, **sample.joints}
        samples.append(sample.model_copy(update={"joints": dict(held)}))
    return samples


def label_at(plan: list[JointSample], t: float) -> str | None:
    """Segment label active at time t (None before the first labelled sample).

    Args:
        plan (list[JointSample]): Parsed plan.
        t (float): Time in s.

    Returns:
        str | None: The label of the latest labelled sample at or before t.
    """
    active: str | None = None
    for sample in plan:
        if sample.t > t:
            break
        if sample.label is not None:
            active = sample.label
    return active


def interpolate_joints(plan: list[JointSample], t: float) -> dict[str, float]:
    """Linearly interpolate the joint targets at time t, clamped to the plan ends.

    Args:
        plan (list[JointSample]): Parsed plan.
        t (float): Time in s.

    Returns:
        dict[str, float]: Joint name -> measured angle in rad.
    """
    times = [s.t for s in plan]
    index = bisect_right(times, t)
    if index == 0:
        return dict(plan[0].joints)
    if index == len(plan):
        return dict(plan[-1].joints)
    before, after = plan[index - 1], plan[index]
    w = (t - before.t) / (after.t - before.t)
    return {j: (1 - w) * before.joints[j] + w * after.joints[j] for j in JOINT_ORDER}
