"""Traversability classes shared by the planner and the 3D map (task T3), the Python mirror of ground_estimator_cpp.

ground_estimator_cpp (include/ground_estimator_cpp/ground_classes.hpp) writes one class per measured point on
/ground/classified_points: x, y, z float32 at offsets 0, 4, 8 and `class` uint8 at offset 12 (16 bytes per point), in
the fixed frame (odom). tests/test_ground_classes_contract.py pins the values, names and costs below to the C++ enum,
and the limits to the ground estimator config. Readers: mcp_server map3d (class voxels), web_ui (Ground layer colours).
"""

from collections.abc import Mapping, Sequence
from enum import IntEnum
from typing import Any, Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

CLASSIFIED_POINT_STEP = 16  # bytes per point of /ground/classified_points
CLASS_FIELD = "class"
XYZ_FIELDS = ("x", "y", "z")
STEP_UNIT_M = 2.0  # a step fraction is of the wheel diameter: 2 x wheel_radius_m

ClassCost = Literal["unknown", "free", "slow", "lethal", "ignored"]


class GroundClass(IntEnum):
    """Class of a surface point; the value is the wire format of the `class` field."""

    unknown = 0  # measured, no class settled (cell not stable, spread-gated, occlusion shadow)
    floor = 1  # within the floor band
    step_up = 2  # climbable step up (slow down)
    step_down = 3  # climbable step down (slow down)
    ramp = 4  # incline up to the max slope (slow down)
    obstacle = 5  # too high a step or too steep (lethal)
    wall = 6  # a lethal cell taller than the tallest step (lethal)
    drop = 7  # deeper than the step down limit (lethal, no-go)
    overhang = 8  # above the robot height over a cell that is not blocked: ignored for driving


COSTS: dict[GroundClass, ClassCost] = {
    GroundClass.unknown: "unknown",
    GroundClass.floor: "free",
    GroundClass.step_up: "slow",
    GroundClass.step_down: "slow",
    GroundClass.ramp: "slow",
    GroundClass.obstacle: "lethal",
    GroundClass.wall: "lethal",
    GroundClass.drop: "lethal",
    GroundClass.overhang: "ignored",
}
LETHAL_CLASSES = frozenset(c for c, cost in COSTS.items() if cost == "lethal")
CLASS_COUNT = len(GroundClass)


class ClassLimits(BaseModel):
    """The limits of the classes, all from one ground estimator config (bands, traversability, edges)."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    floor_band_m: float = Field(gt=0.0)  # bands.floor_band_m
    max_step_up_m: float = Field(gt=0.0)  # 2 x wheel_radius_m x max_step_up_fraction
    max_step_down_m: float = Field(gt=0.0)  # 2 x wheel_radius_m x max_step_down_fraction
    max_slope_deg: float = Field(gt=0.0)  # traversability.max_slope_deg, in every direction
    wall_min_height_m: float = Field(gt=0.0)  # edges.step_max_height_m: the tallest step-like object
    robot_height_m: float = Field(gt=0.0)  # traversability.robot_height_m: higher is an overhang
    chassis_clearance_m: float = Field(gt=0.0)  # traversability.chassis_clearance_m


def limits_from_ground_config(config: Mapping[str, Any]) -> ClassLimits:
    """The class limits of a parsed ground estimator config (the repo yaml or the client.yml `config: |` block).

    Args:
        config (Mapping[str, Any]): The config document; bands, traversability and edges must hold the keys used.

    Returns:
        ClassLimits: The limits.
    """
    trav = config["traversability"]
    diameter_m = STEP_UNIT_M * float(trav["wheel_radius_m"])
    return ClassLimits(
        floor_band_m=float(config["bands"]["floor_band_m"]),
        max_step_up_m=diameter_m * float(trav["max_step_up_fraction"]),
        max_step_down_m=diameter_m * float(trav["max_step_down_fraction"]),
        max_slope_deg=float(trav["max_slope_deg"]),
        wall_min_height_m=float(config["edges"]["step_max_height_m"]),
        robot_height_m=float(trav["robot_height_m"]),
        chassis_clearance_m=float(trav["chassis_clearance_m"]),
    )


def class_name(value: int) -> str:
    """Name of a class value; an out-of-range value is "unknown" (never a guess).

    Args:
        value (int): Wire value.

    Returns:
        str: The class name.
    """
    return GroundClass(value).name if 0 <= value < CLASS_COUNT else GroundClass.unknown.name


def decode_classified(
    data: bytes, count: int, point_step: int, fields: Sequence[tuple[str, int]], is_bigendian: bool = False
) -> tuple[np.ndarray, np.ndarray]:
    """Points and classes of a /ground/classified_points message body.

    Args:
        data (bytes): PointCloud2 data.
        count (int): width x height.
        point_step (int): Bytes per point.
        fields (Sequence[tuple[str, int]]): (name, offset) of the message's fields.
        is_bigendian (bool): Byte order of the floats.

    Returns:
        tuple[np.ndarray, np.ndarray]: N x 3 float64 xyz and N uint8 classes, points with a non-finite coordinate or a
            class value outside the enum dropped.

    Raises:
        ValueError: A field of x, y, z or class is missing.
    """
    offsets = dict(fields)
    missing = [n for n in (*XYZ_FIELDS, CLASS_FIELD) if n not in offsets]
    if missing:
        raise ValueError(f"classified cloud without fields {missing}")
    order = ">" if is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": [*XYZ_FIELDS, CLASS_FIELD],
            "formats": [f"{order}f4"] * 3 + ["u1"],
            "offsets": [offsets[n] for n in (*XYZ_FIELDS, CLASS_FIELD)],
            "itemsize": point_step,
        }
    )
    raw = np.frombuffer(bytes(data), dtype=dtype, count=count)
    xyz = np.stack([raw[n] for n in XYZ_FIELDS], axis=1).astype(np.float64)
    classes = raw[CLASS_FIELD].astype(np.uint8)
    keep = np.isfinite(xyz).all(axis=1) & (classes < CLASS_COUNT)
    return xyz[keep], classes[keep]
