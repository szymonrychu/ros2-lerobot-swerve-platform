"""GET /api/arm/calibration (web_ui.arm_calibration): the arm calibration the 3D arm views draw with, the accepted one
(re-read only when applied/applied.json changes) over the configured client.yml values (source "configured"), the
baseline mcp_server and filter_node use while nothing is accepted."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.applied_arm_calibration import ArmCalibrationSettings, calibrated_settings
from ros2_common.head_rgbd_calibration.arm_calibration_file import (
    ARM_CALIBRATION_FILE,
    ARM_CALIBRATION_KEY,
    read_arm_calibration,
)
from ros2_common.head_rgbd_calibration.urdf_text import APPLIED_MARKER_FILE

from web_ui.arm_calibration import ArmCalibrationCache, calibration_payload
from web_ui.config import AppConfig, ArmCalibrationConfig, TabConfig

# Route bodies shared with the frontend tests (frontend/src/arm/armCalibrationFixtures.ts).
FIXTURES = Path(__file__).parent / "fixtures" / "api"
SESSION = "calib-20261109-120000"
OFFSETS = {"shoulder_pan": 0.03, "shoulder_lift": -0.02, "elbow_flex": -0.09, "wrist_flex": 0.08, "wrist_roll": -0.07}
BASE = {"x": 0.061, "y": -0.048, "z": 0.102, "yaw": 0.01}
ACCEPTED = {
    "source": "accepted",
    "accepted": {"session": SESSION, "result": "r-1", "accepted_at": "2026-11-09T12:20:00Z"},
    "joint_offsets_rad": OFFSETS,
    "base_in_base_link": BASE,
}
# The configured values (client.yml web_ui arm_calibration, the mcp_server arm values).
CONFIGURED_OFFSETS = {
    "shoulder_pan": 0.0182,
    "shoulder_lift": -0.0345,
    "elbow_flex": -0.107,
    "wrist_flex": 0.0988,
    "wrist_roll": -0.071,
}
CONFIGURED_BASE = {"x": 0.0592, "y": -0.05, "z": 0.1, "yaw": 0.0}
BASELINE = ArmCalibrationSettings(
    joint_offsets_rad=CONFIGURED_OFFSETS, arm_base_height_m=0.1, base_in_base_link=CONFIGURED_BASE
)
CONFIGURED = {
    "source": "configured",
    "accepted": None,
    "joint_offsets_rad": CONFIGURED_OFFSETS,
    "base_in_base_link": CONFIGURED_BASE,
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


def arm_calibration_config(applied_dir: Path | None) -> ArmCalibrationConfig:
    """web_ui arm_calibration with the configured values and the given applied directory."""
    return ArmCalibrationConfig(applied_dir=None if applied_dir is None else str(applied_dir), **BASELINE.model_dump())


def client(applied_dir: Path | None, tmp_path: Path, configured: bool = True) -> TestClient:
    """App with an arm tab and arm_calibration (the configured values over applied_dir), or none."""
    from web_ui.server import build_app

    config = AppConfig(
        tabs=[TabConfig(id="arm", type="arm", label="Arm")],
        arm_calibration=arm_calibration_config(applied_dir) if configured else None,
    )
    return TestClient(build_app(config=config, urdf_dir=tmp_path, static_dir=tmp_path / "none"))


# --- cache --------------------------------------------------------------------------------------------------------


def test_nothing_accepted_serves_the_configured_values(tmp_path: Path) -> None:
    assert ArmCalibrationCache(BASELINE, tmp_path).get() == {"calibration": CONFIGURED, "error": None}
    assert ArmCalibrationCache(BASELINE, None).get() == {"calibration": CONFIGURED, "error": None}


def test_an_accepted_calibration_is_served(tmp_path: Path) -> None:
    accept(tmp_path)
    assert ArmCalibrationCache(BASELINE, tmp_path).get() == {"calibration": ACCEPTED, "error": None}


def test_without_an_applied_dir_an_accepted_calibration_is_not_read(tmp_path: Path) -> None:
    accept(tmp_path)
    assert ArmCalibrationCache(BASELINE, None).get() == {"calibration": CONFIGURED, "error": None}


def test_the_file_is_read_only_after_an_applied_json_change(tmp_path: Path) -> None:
    accept(tmp_path)
    cache = ArmCalibrationCache(BASELINE, tmp_path)
    assert cache.get()["calibration"] == ACCEPTED
    (tmp_path / ARM_CALIBRATION_FILE).write_text("{broken, but applied.json did not change")
    assert cache.get() == {"calibration": ACCEPTED, "error": None}
    accept(tmp_path, result="r-2")
    assert cache.get()["calibration"]["accepted"]["result"] == "r-2"


def test_an_invalid_file_keeps_the_last_good_calibration_and_reports_the_error(tmp_path: Path) -> None:
    accept(tmp_path)
    cache = ArmCalibrationCache(BASELINE, tmp_path)
    cache.get()
    (tmp_path / ARM_CALIBRATION_FILE).write_text("{not json")
    write_marker(tmp_path, json.loads((tmp_path / APPLIED_MARKER_FILE).read_text()) | {"touched": True})
    body = cache.get()
    assert body["calibration"] == ACCEPTED and ARM_CALIBRATION_FILE in body["error"]
    accept(tmp_path, result="r-3")
    body = cache.get()
    assert body["calibration"]["accepted"]["result"] == "r-3" and body["error"] is None


def test_an_invalid_file_with_nothing_good_before_serves_the_configured_values_and_the_error(tmp_path: Path) -> None:
    accept(tmp_path)
    (tmp_path / ARM_CALIBRATION_FILE).write_text("{not json")
    body = ArmCalibrationCache(BASELINE, tmp_path).get()
    assert body["calibration"] == CONFIGURED and ARM_CALIBRATION_FILE in body["error"]


def test_a_calibration_the_baseline_model_refuses_keeps_the_values_in_use(tmp_path: Path) -> None:
    """A base z of 1.5 m is above the 1 m mount height limit: the merge fails, the configured values stay."""
    applied = tmp_path / "applied"
    accept(applied)
    calibration = json.loads((applied / ARM_CALIBRATION_FILE).read_text())
    calibration["base_in_base_link"]["z"] = 1.5
    (applied / ARM_CALIBRATION_FILE).write_text(json.dumps(calibration))
    body = ArmCalibrationCache(BASELINE, applied).get()
    assert body["calibration"] == CONFIGURED and "arm_base_height_m" in body["error"]


def test_a_removed_record_goes_back_to_the_configured_values(tmp_path: Path) -> None:
    accept(tmp_path)
    cache = ArmCalibrationCache(BASELINE, tmp_path)
    cache.get()
    write_marker(tmp_path, {"head_chain": {"session": "s", "result": "r"}})
    assert cache.get() == {"calibration": CONFIGURED, "error": None}


def test_the_payload_carries_only_what_the_view_draws(tmp_path: Path) -> None:
    """Source, identity, joint offsets and base pose; no capture time, score or tool point (the tool point is not
    fitted); the mount height is the base z."""
    accept(tmp_path)
    calibration = read_arm_calibration(tmp_path)
    assert calibration_payload(calibrated_settings(BASELINE, calibration), calibration) == ACCEPTED
    assert calibration_payload(BASELINE, None) == CONFIGURED


# --- config -------------------------------------------------------------------------------------------------------


def test_the_config_key_defaults_to_none() -> None:
    assert AppConfig().arm_calibration is None


def test_the_config_validates_the_configured_values_as_the_shared_baseline() -> None:
    config = arm_calibration_config(Path("/var/lib/ros2/head_rgbd_calibration/applied"))
    assert config.applied_dir == "/var/lib/ros2/head_rgbd_calibration/applied"
    assert ArmCalibrationSettings.model_validate(config.model_dump(exclude={"applied_dir"})) == BASELINE
    with pytest.raises(ValidationError):
        ArmCalibrationConfig(**{**BASELINE.model_dump(), "arm_base_height_m": 0.12})
    with pytest.raises(ValidationError):
        ArmCalibrationConfig(applied_dir=None, joint_offsets_rad=CONFIGURED_OFFSETS, arm_base_height_m=0.1)


def test_the_arm_tab_has_no_arm_offset_of_its_own() -> None:
    """The arm mount comes only from arm_calibration (GET /api/arm/calibration)."""
    with pytest.raises(ValidationError):
        TabConfig(id="arm", type="arm", label="Arm", arm_offset=(0.0592, -0.05, 0.1))


# --- route --------------------------------------------------------------------------------------------------------


def test_the_route_serves_the_accepted_calibration_uncached(tmp_path: Path) -> None:
    applied = tmp_path / "applied"
    accept(applied)
    response = client(applied, tmp_path).get("/api/arm/calibration")
    assert response.status_code == 200
    assert response.json() == {"calibration": ACCEPTED, "error": None}
    assert response.headers["cache-control"] == "no-store"


def test_the_route_bodies_are_the_frontend_fixtures(tmp_path: Path) -> None:
    """The frontend tests read these files: a contract change here shows up there."""
    applied = tmp_path / "applied"
    http = client(applied, tmp_path)
    assert http.get("/api/arm/calibration").json() == json.loads(
        (FIXTURES / "arm_calibration_configured.json").read_text()
    )
    accept(applied)
    assert http.get("/api/arm/calibration").json() == json.loads(
        (FIXTURES / "arm_calibration_accepted.json").read_text()
    )


def test_the_route_follows_a_new_accept_without_a_restart(tmp_path: Path) -> None:
    applied = tmp_path / "applied"
    http = client(applied, tmp_path)
    assert http.get("/api/arm/calibration").json() == {"calibration": CONFIGURED, "error": None}
    accept(applied)
    assert http.get("/api/arm/calibration").json()["calibration"] == ACCEPTED


def test_without_an_applied_dir_the_route_serves_the_configured_values(tmp_path: Path) -> None:
    accept(tmp_path / "applied")
    response = client(None, tmp_path).get("/api/arm/calibration")
    assert response.status_code == 200 and response.json() == {"calibration": CONFIGURED, "error": None}


def test_without_arm_calibration_the_route_answers_null(tmp_path: Path) -> None:
    accept(tmp_path / "applied")
    response = client(tmp_path / "applied", tmp_path, configured=False).get("/api/arm/calibration")
    assert response.status_code == 200 and response.json() == {"calibration": None, "error": None}
