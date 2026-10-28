"""Marker geometry: map composition and the POI / report JSON (no ROS)."""

import json

import numpy as np
from ros2_common.marker_payload import marker_info
from scipy.spatial.transform import Rotation

from marker_detector.decode import Candidate
from marker_detector.pose import PoseResult

REPORT_SOURCE = "stream"


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


def detection_report(stamp_ns: int, items: list[dict]) -> str:
    """Serialise the ``/markers/detections`` JSON of one stream frame (``source`` is always ``stream``).

    Args:
        stamp_ns: Image stamp in nanoseconds.
        items: Per marker dicts (key, payload, position, quaternion, rms_px, ratio, confidence, module_px,
            chooser ``reprojection`` or ``depth``).
    Returns:
        JSON text.
    """
    return json.dumps({"stamp_ns": stamp_ns, "source": REPORT_SOURCE, "markers": items})
