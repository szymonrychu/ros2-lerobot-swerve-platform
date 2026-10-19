"""Marker geometry: map composition, zoom-still intrinsics, zoom requests and the POI / report JSON (no ROS)."""

import json

import numpy as np
from ros2_common.marker_payload import marker_info
from scipy.spatial.transform import Rotation

from marker_detector.config import ZoomConfig
from marker_detector.decode import Candidate
from marker_detector.pose import PoseResult
from marker_detector.zoom import UNDECODED_ZOOM, ZOOM_STEPS

REPORT_SOURCES = frozenset({"stream", "zoom"})


def marker_in_map(
    rvec: np.ndarray, tvec: np.ndarray, map_from_camera: tuple[np.ndarray, np.ndarray]
) -> tuple[np.ndarray, np.ndarray]:
    """Compose map <- camera optical <- marker.

    Args:
        rvec: (3,) Rodrigues rotation of the marker in the camera optical frame.
        tvec: (3,) marker position in the camera optical frame, metres.
        map_from_camera: (translation (3,) metres, quaternion xyzw) of the camera optical frame in the map frame.
    Returns:
        (position (3,) metres in map, quaternion xyzw of the marker in map).
    """
    translation, quat = map_from_camera
    cam_rot = Rotation.from_quat(np.asarray(quat, dtype=float))
    position = np.asarray(translation, dtype=float) + cam_rot.apply(np.asarray(tvec, dtype=float).reshape(3))
    marker_rot = cam_rot * Rotation.from_rotvec(np.asarray(rvec, dtype=float).reshape(3))
    return position, marker_rot.as_quat()


def zoom_intrinsics(
    stream_k: np.ndarray,
    stream_size: tuple[int, int],
    full_size: tuple[int, int],
    crop: tuple[int, int, int, int],
    out_size: tuple[int, int],
) -> np.ndarray:
    """Intrinsics of a zoom still: scale to the full sensor, shift by the crop origin, scale by out/crop.

    Args:
        stream_k: 3x3 intrinsics of the stream image.
        stream_size: (width, height) of the stream image.
        full_size: (width, height) of the full sensor image.
        crop: (x, y, width, height) of the crop in full-sensor pixels.
        out_size: (width, height) of the delivered still.
    Returns:
        3x3 intrinsics of the zoom still.
    """
    k = np.array(stream_k, dtype=float)
    k[0] *= full_size[0] / stream_size[0]
    k[1] *= full_size[1] / stream_size[1]
    k[0, 2] -= crop[0]
    k[1, 2] -= crop[1]
    k[0] *= out_size[0] / crop[2]
    k[1] *= out_size[1] / crop[3]
    return k


def zoom_factor(module_px: float | None, cfg: ZoomConfig) -> int:
    """Smallest zoom of 2/4/8 reaching ``cfg.target_module_px``, else 8 (4 for an undecoded quad).

    Args:
        module_px: Decoded module size in pixels, or None / non-positive when unknown.
        cfg: Zoom settings.
    Returns:
        The zoom factor.
    """
    if module_px is None or module_px <= 0:
        return UNDECODED_ZOOM
    needed = cfg.target_module_px / module_px
    return next((z for z in ZOOM_STEPS if z >= needed), ZOOM_STEPS[-1])


def zoom_request(u: float, v: float, module_px: float | None, cfg: ZoomConfig, request_id: str) -> dict:
    """Build a zoom command (front camera epic D4).

    Args:
        u: Normalised x of the target in the default view, top-left origin.
        v: Normalised y of the target.
        module_px: Module size in pixels (None for an undecoded quad).
        cfg: Zoom settings.
        request_id: Id echoed in the result.
    Returns:
        Dict with request_id, u, v, zoom, max_px and af.
    """
    return {
        "request_id": request_id,
        "u": float(u),
        "v": float(v),
        "zoom": zoom_factor(module_px, cfg),
        "max_px": cfg.max_px,
        "af": True,
    }


def sighting_command(
    candidate: Candidate, pose: PoseResult, position: np.ndarray, quaternion: np.ndarray, request_id: str
) -> dict:
    """Build the ``/poi/command`` upsert_marker dict (contract owned by poi_store).

    Args:
        candidate: The decoded marker.
        pose: Accepted pose result (its confidence is used).
        position: (3,) map position in metres.
        quaternion: (4,) map orientation, xyzw.
        request_id: Command id.
    Returns:
        JSON-serialisable dict.
    """
    x, y, z = (float(c) for c in position)
    return {
        "op": "upsert_marker",
        "request_id": request_id,
        "poi": {
            "kind": "marker",
            "x": x,
            "y": y,
            "z": z,
            "orientation": [float(c) for c in quaternion],
            "marker": marker_info(candidate.payload).model_dump(),
            "confidence": float(pose.confidence),
        },
    }


def detection_report(stamp_ns: int, source: str, items: list[dict]) -> str:
    """Serialise the ``/markers/detections`` JSON.

    Args:
        stamp_ns: Image stamp in nanoseconds.
        source: ``stream`` or ``zoom``.
        items: Per marker dicts (key, payload, position, quaternion, rms_px, ratio, confidence, module_px,
            chooser ``reprojection`` or ``tof``).
    Returns:
        JSON text.
    Raises:
        ValueError: if ``source`` is not stream or zoom.
    """
    if source not in REPORT_SOURCES:
        raise ValueError(f"source must be one of {sorted(REPORT_SOURCES)}, got {source!r}")
    return json.dumps({"stamp_ns": stamp_ns, "source": source, "markers": items})
