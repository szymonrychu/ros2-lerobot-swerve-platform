"""Tests for marker_detector.geometry (map composition, POI command and report JSON)."""

import json

import numpy as np
import pytest
from ros2_common.marker_payload import MarkerInfo, decode_payload
from scipy.spatial.transform import Rotation

from marker_detector.decode import Candidate
from marker_detector.geometry import detection_report, marker_in_map, sighting_command
from marker_detector.pose import PoseResult

PAYLOAD_TEXT = "P1/K:BASE/N:HOME/S:150"
IDENTITY_Q = np.array([0.0, 0.0, 0.0, 1.0])


def candidate(module_px: float = 8.0) -> Candidate:
    corners = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])
    return Candidate(decode_payload(PAYLOAD_TEXT), PAYLOAD_TEXT, corners, 1, module_px)


def test_identity_tf_returns_camera_pose() -> None:
    rvec = Rotation.from_euler("z", 30, degrees=True).as_rotvec()
    tvec = np.array([0.1, -0.2, 1.5])
    pos, quat = marker_in_map(rvec, tvec, (np.zeros(3), IDENTITY_Q))
    assert pos == pytest.approx(tvec)
    assert Rotation.from_quat(quat).as_rotvec() == pytest.approx(rvec)


def test_known_tf_maps_marker_ahead() -> None:
    # camera at (1, 2, 0.5) rotated 90 deg yaw: camera +z (forward in optical) mapped to map +y here.
    rot = Rotation.from_euler("z", 90, degrees=True)
    tf = (np.array([1.0, 2.0, 0.5]), rot.as_quat())
    pos, quat = marker_in_map(np.zeros(3), np.array([0.0, 0.0, 1.0]), tf)
    assert pos == pytest.approx([1.0, 2.0, 1.5])
    assert Rotation.from_quat(quat).as_matrix() == pytest.approx(rot.as_matrix())
    pos, _ = marker_in_map(np.zeros(3), np.array([1.0, 0.0, 0.0]), tf)
    assert pos == pytest.approx([1.0, 3.0, 0.5])
    assert np.linalg.norm(quat) == pytest.approx(1.0)


def test_sighting_command_matches_poi_contract() -> None:
    quat = Rotation.from_euler("z", 40, degrees=True).as_quat()
    pose = PoseResult(ok=True, confidence=0.8)
    cmd = sighting_command(candidate(), pose, np.array([1.0, 2.0, 0.5]), quat, "rid")
    assert set(cmd) == {"op", "request_id", "poi"}
    assert cmd["op"] == "upsert_marker"
    assert cmd["request_id"] == "rid"
    poi = cmd["poi"]
    assert set(poi) == {"kind", "x", "y", "z", "orientation", "marker", "confidence"}
    assert poi["kind"] == "marker"
    assert (poi["x"], poi["y"], poi["z"]) == (1.0, 2.0, 0.5)
    assert len(poi["orientation"]) == 4
    assert np.linalg.norm(poi["orientation"]) == pytest.approx(1.0)
    assert poi["orientation"] == pytest.approx(list(quat))
    info = MarkerInfo.model_validate(poi["marker"])
    assert info.key == "base/HOME"
    assert info.payload == PAYLOAD_TEXT
    assert poi["confidence"] == 0.8
    assert json.loads(json.dumps(cmd)) == cmd


def test_detection_report_round_trips() -> None:
    item = {
        "key": "base/HOME",
        "payload": PAYLOAD_TEXT,
        "position": [1.0, 2.0, 0.5],
        "quaternion": [0.0, 0.0, 0.0, 1.0],
        "rms_px": 0.4,
        "ratio": 3.1,
        "confidence": 0.8,
        "module_px": 8.0,
        "chooser": "reprojection",
    }
    text = detection_report(123, [item])
    assert json.loads(text) == {"stamp_ns": 123, "source": "stream", "markers": [item]}
