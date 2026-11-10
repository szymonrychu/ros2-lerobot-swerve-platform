"""GET /api/arm/calibration (web_ui.arm_calibration): the accepted arm calibration for the 3D arm view, re-read only
when applied/applied.json changes; null while nothing is accepted (the view keeps the tab's configured mount)."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

from fastapi.testclient import TestClient
from ros2_common.head_rgbd_calibration.arm_calibration_file import (
    ARM_CALIBRATION_FILE,
    ARM_CALIBRATION_KEY,
    read_arm_calibration,
)
from ros2_common.head_rgbd_calibration.urdf_text import APPLIED_MARKER_FILE

from web_ui.arm_calibration import ArmCalibrationCache, calibration_payload
from web_ui.config import AppConfig, TabConfig

SESSION = "calib-20261109-120000"
OFFSETS = {"shoulder_pan": 0.03, "shoulder_lift": -0.02, "elbow_flex": -0.09, "wrist_flex": 0.08, "wrist_roll": -0.07}
BASE = {"x": 0.061, "y": -0.048, "z": 0.102, "yaw": 0.01}
ACCEPTED = {
    "session": SESSION,
    "result": "r-1",
    "accepted_at": "2026-11-09T12:20:00Z",
    "joint_offsets_rad": OFFSETS,
    "base_in_base_link": BASE,
}


def accept(applied: Path, result: str = "r-1") -> None:
    """Write arm_calibration.json, then the applied.json record, as the head_rgbd_calibration accept does."""
    applied.mkdir(parents=True, exist_ok=True)
    body = {
        "session": SESSION,
        "result": result,
        "accepted_at": "2026-11-09T12:20:00+00:00",
        "captured_from": "2026-11-09T12:01:00+00:00",
        "score": {"rms_px": 0.9},
        "joint_offsets_rad": OFFSETS,
        "base_in_base_link": BASE,
        "tool_offset_m": [0.001, -0.0056, -0.0014],
    }
    (applied / ARM_CALIBRATION_FILE).write_text(json.dumps(body))
    record = {"session": SESSION, "result": result, "accepted_at": body["accepted_at"]}
    write_marker(applied, {ARM_CALIBRATION_KEY: record})


def write_marker(applied: Path, marker: dict[str, Any]) -> None:
    """applied.json with an mtime one second past the previous one (coarse clocks)."""
    path = applied / APPLIED_MARKER_FILE
    old = path.stat().st_mtime_ns if path.exists() else 0
    path.write_text(json.dumps(marker))
    stat = path.stat()
    os.utime(path, ns=(stat.st_atime_ns, max(stat.st_mtime_ns, old + 1_000_000_000)))


def client(applied_dir: Path | None, tmp_path: Path) -> TestClient:
    """App with an arm tab and the given arm_applied_calibration_dir."""
    from web_ui.server import build_app

    config = AppConfig(
        tabs=[TabConfig(id="arm", type="arm", label="Arm")],
        arm_applied_calibration_dir=None if applied_dir is None else str(applied_dir),
    )
    return TestClient(build_app(config=config, urdf_dir=tmp_path, static_dir=tmp_path / "none"))


# --- cache --------------------------------------------------------------------------------------------------------


def test_nothing_accepted_is_null(tmp_path: Path) -> None:
    assert ArmCalibrationCache(tmp_path).get() == {"accepted": None, "error": None}
    assert ArmCalibrationCache(None).get() == {"accepted": None, "error": None}


def test_an_accepted_calibration_is_served(tmp_path: Path) -> None:
    accept(tmp_path)
    assert ArmCalibrationCache(tmp_path).get() == {"accepted": ACCEPTED, "error": None}


def test_the_file_is_read_only_after_an_applied_json_change(tmp_path: Path) -> None:
    accept(tmp_path)
    cache = ArmCalibrationCache(tmp_path)
    assert cache.get()["accepted"] == ACCEPTED
    (tmp_path / ARM_CALIBRATION_FILE).write_text("{broken, but applied.json did not change")
    assert cache.get() == {"accepted": ACCEPTED, "error": None}
    accept(tmp_path, result="r-2")
    assert cache.get()["accepted"]["result"] == "r-2"


def test_an_invalid_file_keeps_the_last_good_calibration_and_reports_the_error(tmp_path: Path) -> None:
    accept(tmp_path)
    cache = ArmCalibrationCache(tmp_path)
    cache.get()
    (tmp_path / ARM_CALIBRATION_FILE).write_text("{not json")
    write_marker(tmp_path, json.loads((tmp_path / APPLIED_MARKER_FILE).read_text()) | {"touched": True})
    body = cache.get()
    assert body["accepted"] == ACCEPTED and ARM_CALIBRATION_FILE in body["error"]
    accept(tmp_path, result="r-3")
    body = cache.get()
    assert body["accepted"]["result"] == "r-3" and body["error"] is None


def test_an_invalid_file_with_nothing_good_before_serves_null_and_the_error(tmp_path: Path) -> None:
    accept(tmp_path)
    (tmp_path / ARM_CALIBRATION_FILE).write_text("{not json")
    body = ArmCalibrationCache(tmp_path).get()
    assert body["accepted"] is None and ARM_CALIBRATION_FILE in body["error"]


def test_a_removed_record_goes_back_to_null(tmp_path: Path) -> None:
    accept(tmp_path)
    cache = ArmCalibrationCache(tmp_path)
    cache.get()
    write_marker(tmp_path, {"head_chain": {"session": "s", "result": "r"}})
    assert cache.get() == {"accepted": None, "error": None}


def test_the_payload_carries_only_what_the_view_draws(tmp_path: Path) -> None:
    """Identity, joint offsets and base pose; no capture time, score or tool point (the tool point is not fitted)."""
    accept(tmp_path)
    assert calibration_payload(read_arm_calibration(tmp_path)) == ACCEPTED
    assert calibration_payload(None) is None


# --- route --------------------------------------------------------------------------------------------------------


def test_the_route_serves_the_accepted_calibration_uncached(tmp_path: Path) -> None:
    applied = tmp_path / "applied"
    accept(applied)
    response = client(applied, tmp_path).get("/api/arm/calibration")
    assert response.status_code == 200
    assert response.json() == {"accepted": ACCEPTED, "error": None}
    assert response.headers["cache-control"] == "no-store"


def test_the_route_follows_a_new_accept_without_a_restart(tmp_path: Path) -> None:
    applied = tmp_path / "applied"
    http = client(applied, tmp_path)
    assert http.get("/api/arm/calibration").json() == {"accepted": None, "error": None}
    accept(applied)
    assert http.get("/api/arm/calibration").json()["accepted"] == ACCEPTED


def test_without_an_applied_dir_the_route_answers_null(tmp_path: Path) -> None:
    accept(tmp_path / "applied")
    response = client(None, tmp_path).get("/api/arm/calibration")
    assert response.status_code == 200 and response.json() == {"accepted": None, "error": None}


def test_the_config_key_defaults_to_none() -> None:
    assert AppConfig().arm_applied_calibration_dir is None
