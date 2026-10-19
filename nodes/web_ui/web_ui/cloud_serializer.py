"""Serialize the RTAB-Map point cloud (/map3d/cloud, sensor_msgs/PointCloud2) into a compact browser payload."""

from __future__ import annotations

import base64
from typing import Any

import numpy as np

from web_ui.msg_serializer import FLOAT32_DATATYPE, UINT32_DATATYPE, cloud_columns, stamp_to_seconds

# Defaults of the map_nav tab fields map3d_max_points and map3d_voxel_m.
DEFAULT_MAX_POINTS = 60000
DEFAULT_VOXEL_M = 0.05
# Wire format: int16 centimetres, so +-327.67 m is the representable range.
CM_PER_M = 100.0
CM_MAX = 32767
# Factor the voxel grows by while the cloud still exceeds max_points.
VOXEL_GROWTH = 1.5
COLOR_FIELDS = ("rgb", "rgba")
XYZ_FIELDS = ("x", "y", "z")
COLOR_DATATYPES = frozenset({UINT32_DATATYPE, FLOAT32_DATATYPE})


def voxel_first_indices(xyz: np.ndarray, voxel_m: float) -> np.ndarray:
    """Indices of the first point in each voxel cell, in input order.

    Args:
        xyz (np.ndarray): (n, 3) points in metres, all within the representable range.
        voxel_m (float): Cell edge in metres.

    Returns:
        np.ndarray: Sorted indices of one point per occupied cell.
    """
    cells = np.floor(xyz / voxel_m).astype(np.int64)
    cells -= cells.min(axis=0)
    span = cells.max(axis=0) + 1
    keys = (cells[:, 0] * span[1] + cells[:, 1]) * span[2] + cells[:, 2]
    _, first = np.unique(keys, return_index=True)
    return np.sort(first)


def serialize_map3d_cloud(msg: Any, max_points: int, voxel_m: float) -> dict[str, Any]:
    """Serialize a sensor_msgs/PointCloud2 to int16 centimetre points with optional colours.

    Non-finite points are dropped, the rest voxel-downsampled (first point of a cell wins, colour included); while
    more than max_points remain the voxel grows by VOXEL_GROWTH. Points beyond +-327.67 m are dropped and counted.

    Args:
        msg (Any): sensor_msgs/PointCloud2 with float32 x, y, z and optionally a packed rgb/rgba field.
        max_points (int): Maximum points in the payload.
        voxel_m (float): Starting voxel edge in metres.

    Returns:
        dict[str, Any]: points_b64 (little-endian int16 cm x, y, z), colors_b64 (uint8 r, g, b per point; omitted
        without a colour field), count, source_count, voxel_m (the one used), dropped_out_of_range, frame_id, stamp.

    Raises:
        ValueError: If the layout is unsupported (the message names the fields) or the data is truncated.
    """
    source_count = msg.width * msg.height
    payload: dict[str, Any] = {
        "frame_id": msg.header.frame_id,
        "stamp": stamp_to_seconds(msg.header.stamp),
        "source_count": source_count,
        "voxel_m": voxel_m,
        "dropped_out_of_range": 0,
    }
    if source_count == 0 and len(msg.data) == 0:
        return {**payload, "count": 0, "points_b64": ""}

    color_name = next((f.name for f in msg.fields if f.name in COLOR_FIELDS), None)
    names = (*XYZ_FIELDS, *(color_name,) * (color_name is not None))
    datatypes = {
        **dict.fromkeys(XYZ_FIELDS, frozenset({FLOAT32_DATATYPE})),
        **dict.fromkeys(names[3:], COLOR_DATATYPES),
    }
    columns = cloud_columns(msg, names, datatypes)
    xyz = np.stack([columns[n] for n in XYZ_FIELDS], axis=1).astype(np.float64)
    packed = None
    if color_name is not None:
        column = columns[color_name]
        # A float32 colour carries the packed uint32 bit pattern; a uint32 one is the value itself.
        packed = column.astype(np.float32).view(np.uint32) if column.dtype.kind == "f" else column.astype(np.uint32)

    keep = np.isfinite(xyz).all(axis=1)
    xyz = xyz[keep]
    packed = packed[keep] if packed is not None else None
    in_range = (np.abs(np.rint(xyz * CM_PER_M)) <= CM_MAX).all(axis=1)
    payload["dropped_out_of_range"] = int((~in_range).sum())
    xyz = xyz[in_range]
    packed = packed[in_range] if packed is not None else None

    voxel = voxel_m
    if len(xyz):
        picked = voxel_first_indices(xyz, voxel)
        while len(picked) > max(max_points, 1):
            voxel = voxel * VOXEL_GROWTH
            picked = voxel_first_indices(xyz, voxel)
        xyz = xyz[picked]
        packed = packed[picked] if packed is not None else None

    payload["voxel_m"] = voxel
    payload["count"] = len(xyz)
    payload["points_b64"] = base64.b64encode(np.rint(xyz * CM_PER_M).astype("<i2").tobytes()).decode("ascii")
    if packed is not None:
        rgb = np.stack([(packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF], axis=1).astype(np.uint8)
        payload["colors_b64"] = base64.b64encode(rgb.tobytes()).decode("ascii")
    return payload
