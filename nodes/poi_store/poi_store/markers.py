"""Pure merge of a marker sighting into the stored marker POI (design D12). No ROS imports."""

import math

import numpy as np
from numpy.typing import ArrayLike

from .config import MarkerMergeConfig
from .models import Poi

MIN_NORM = 1e-12


def unit(q: ArrayLike) -> np.ndarray:
    """Normalise a quaternion.

    Args:
        q: [qx, qy, qz, qw].

    Returns:
        np.ndarray: Unit quaternion.
    """
    arr = np.asarray(q, dtype=float)
    return arr / max(float(np.linalg.norm(arr)), MIN_NORM)


def angle_between_deg(q1: ArrayLike, q2: ArrayLike) -> float:
    """Rotation angle between two orientations (sign of a quaternion does not matter).

    Args:
        q1: [qx, qy, qz, qw].
        q2: [qx, qy, qz, qw].

    Returns:
        float: Angle in degrees, 0..180.
    """
    dot = min(1.0, abs(float(np.dot(unit(q1), unit(q2)))))
    return math.degrees(2.0 * math.acos(dot))


def merge_marker_sighting(existing: Poi, sighting: Poi, cfg: MarkerMergeConfig, now: float) -> Poi:
    """Merge a sighting into the existing marker POI.

    Within cfg.relocate_m and cfg.relocate_deg the position and the sign-aligned quaternion are weighted means (old
    weight min(pose_samples, max_pose_samples) * confidence, new weight the sighting confidence) and pose_samples
    grows; beyond either the pose and confidence are replaced and pose_samples is 1. Name, note, status, created_by,
    created_at and first_seen stay; marker size/params/payload/version come from the sighting.

    Args:
        existing: Stored marker POI.
        sighting: New sighting (a marker POI).
        cfg: Merge settings.
        now: Unix seconds of the sighting.

    Returns:
        Poi: The merged POI (a new object).
    """
    assert existing.marker is not None and sighting.marker is not None and sighting.orientation is not None
    assert existing.orientation is not None
    old_q, new_q = unit(existing.orientation), unit(sighting.orientation)
    old_p = np.array([existing.x, existing.y, existing.z])
    new_p = np.array([sighting.x, sighting.y, sighting.z])
    moved = float(np.linalg.norm(new_p - old_p)) > cfg.relocate_m
    turned = angle_between_deg(old_q, new_q) > cfg.relocate_deg
    if moved or turned:
        pos, quat, conf, samples = new_p, new_q, sighting.confidence, 1
    else:
        w_old = min(existing.marker.pose_samples, cfg.max_pose_samples) * existing.confidence
        w_new = sighting.confidence
        total = w_old + w_new
        if total <= 0.0:
            w_old, w_new, total = 1.0, 1.0, 2.0
        pos = (old_p * w_old + new_p * w_new) / total
        aligned = new_q if float(np.dot(old_q, new_q)) >= 0.0 else -new_q
        quat = unit((old_q * w_old + aligned * w_new) / total)
        conf = (existing.confidence * w_old + sighting.confidence * w_new) / total
        samples = existing.marker.pose_samples + 1
    marker = sighting.marker.model_copy(update={"pose_samples": samples})
    return existing.model_copy(
        update={
            "x": float(pos[0]),
            "y": float(pos[1]),
            "z": float(pos[2]),
            "orientation": tuple(float(c) for c in quat),
            "confidence": min(1.0, max(0.0, float(conf))),
            "marker": marker,
            "times_seen": existing.times_seen + 1,
            "last_seen": now,
        }
    )
