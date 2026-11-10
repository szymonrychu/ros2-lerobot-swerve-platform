"""The accepted arm calibration at runtime (ros2_common.head_rgbd_calibration.applied_arm_calibration): the applied.json
identity, the merge over a configured arm model and the reload state shared by mcp_server, filter_node and web_ui."""

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest
from arm_collision.settings import MOUNT_HEIGHT_TOLERANCE_M as ARM_MODEL_MOUNT_HEIGHT_TOLERANCE_M
from arm_collision.settings import ArmBaseOffset, ArmModelSettings
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.applied_arm_calibration import (
    MOUNT_HEIGHT_TOLERANCE_M,
    AppliedArmCalibration,
    ArmCalibrationSettings,
    applied_marker_identity,
    assign_calibration,
    calibrated_settings,
)
from ros2_common.head_rgbd_calibration.arm_calibration_file import (
    ARM_CALIBRATION_FILE,
    ARM_CALIBRATION_KEY,
    ARM_SETTINGS_FIELDS,
    ArmCalibrationError,
    ArmCalibrationFile,
)
from ros2_common.head_rgbd_calibration.urdf_text import APPLIED_MARKER_FILE

SESSION = "calib-20261109-120000"
OFFSETS = {"shoulder_pan": 0.03, "shoulder_lift": -0.02, "elbow_flex": -0.09, "wrist_flex": 0.08, "wrist_roll": -0.07}
BASE = {"x": 0.061, "y": -0.048, "z": 0.102, "yaw": 0.01}
CONFIGURED = ArmModelSettings(
    joint_offsets_rad={"shoulder_pan": 0.0182, "elbow_flex": -0.107},
    tool_offset_m={"x": 0.001, "y": -0.0056, "z": -0.0014},
)


def body(result: str = "r-1", **base: float) -> dict[str, Any]:
    """An arm_calibration.json body as the accept writes it."""
    return {
        "session": SESSION,
        "result": result,
        "accepted_at": "2026-11-09T12:20:00+00:00",
        "captured_from": "2026-11-09T12:01:00+00:00",
        "score": {"rms_px": 0.9},
        "joint_offsets_rad": dict(OFFSETS),
        "base_in_base_link": {**BASE, **base},
        "tool_offset_m": [0.5, 0.5, 0.5],
    }


def accept(applied: Path, result: str = "r-1", **base: float) -> None:
    """Write arm_calibration.json, then the applied.json record (distinct mtime), as the accept does."""
    applied.mkdir(parents=True, exist_ok=True)
    (applied / ARM_CALIBRATION_FILE).write_text(json.dumps(body(result, **base)))
    record = {"session": SESSION, "result": result, "accepted_at": "2026-11-09T12:20:00+00:00"}
    write_marker(applied, {ARM_CALIBRATION_KEY: record})


def write_marker(applied: Path, marker: dict[str, Any]) -> None:
    """applied.json with an mtime one second past the previous one (coarse clocks)."""
    path = applied / APPLIED_MARKER_FILE
    old = path.stat().st_mtime_ns if path.exists() else 0
    path.write_text(json.dumps(marker))
    stat = path.stat()
    os.utime(path, ns=(stat.st_atime_ns, max(stat.st_mtime_ns, old + 1_000_000_000)))


# --- applied_marker_identity --------------------------------------------------------------------------------------


def test_the_identity_is_mtime_size_and_inode(tmp_path: Path) -> None:
    accept(tmp_path)
    st = (tmp_path / APPLIED_MARKER_FILE).stat()
    assert applied_marker_identity(tmp_path) == (st.st_mtime_ns, st.st_size, st.st_ino)


def test_no_dir_or_no_marker_has_no_identity(tmp_path: Path) -> None:
    assert applied_marker_identity(None) is None
    assert applied_marker_identity(tmp_path) is None
    assert applied_marker_identity(tmp_path / "missing") is None


# --- calibrated_settings / assign_calibration ---------------------------------------------------------------------


def test_without_a_calibration_the_baseline_is_copied() -> None:
    merged = calibrated_settings(CONFIGURED, None)
    assert merged == CONFIGURED and merged is not CONFIGURED
    assert merged.joint_offsets_rad is not CONFIGURED.joint_offsets_rad


def test_the_calibration_sets_exactly_the_fitted_fields_and_keeps_the_type() -> None:
    merged = calibrated_settings(CONFIGURED, ArmCalibrationFile.model_validate(body()))
    assert type(merged) is ArmModelSettings
    assert merged.joint_offsets_rad.model_dump() == OFFSETS
    assert merged.base_in_base_link == ArmBaseOffset(**BASE) and merged.arm_base_height_m == BASE["z"]
    untouched = set(ARM_SETTINGS_FIELDS)
    assert merged.model_dump(exclude=untouched) == CONFIGURED.model_dump(exclude=untouched)  # tool point stays


def test_a_calibration_the_arm_model_refuses_raises() -> None:
    """Base z 1.5 m is above ArmModelSettings' 1 m mount height limit."""
    with pytest.raises(ValidationError):
        calibrated_settings(CONFIGURED, ArmCalibrationFile.model_validate(body(z=1.5)))


def test_assign_copies_the_fitted_fields_in_place() -> None:
    target = CONFIGURED.model_copy(deep=True)
    source = calibrated_settings(CONFIGURED, ArmCalibrationFile.model_validate(body()))
    assign_calibration(target, source)
    assert target == source
    assert target.base_in_base_link is not source.base_in_base_link
    assert target.joint_offsets_rad is not source.joint_offsets_rad


# --- ArmCalibrationSettings ---------------------------------------------------------------------------------------

CONFIGURED_VALUES = {
    "joint_offsets_rad": {
        "shoulder_pan": 0.0182,
        "shoulder_lift": -0.0345,
        "elbow_flex": -0.107,
        "wrist_flex": 0.0988,
        "wrist_roll": -0.071,
    },
    "arm_base_height_m": 0.1,
    "base_in_base_link": {"x": 0.0592, "y": -0.05, "z": 0.1, "yaw": 0.0},
}


def test_the_calibrated_values_validate_as_a_baseline() -> None:
    settings = ArmCalibrationSettings.model_validate(CONFIGURED_VALUES)
    assert settings.model_dump() == CONFIGURED_VALUES
    assert set(ArmCalibrationSettings.model_fields) == set(ARM_SETTINGS_FIELDS)


@pytest.mark.parametrize(
    "change",
    [
        {"joint_offsets_rad": {"shoulder_pan": 0.0}},
        {"joint_offsets_rad": {**CONFIGURED_VALUES["joint_offsets_rad"], "gripper": 0.0}},
        {"joint_offsets_rad": {**CONFIGURED_VALUES["joint_offsets_rad"], "wrist_roll": float("nan")}},
        {"arm_base_height_m": 0.12},
        {"arm_base_height_m": 0.0, "base_in_base_link": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}},
        {"tool_offset_m": {"x": 0.0, "y": 0.0, "z": 0.0}},
    ],
    ids=["missing-joint", "extra-joint", "nan-offset", "z-not-mount-height", "mount-on-the-floor", "unknown-key"],
)
def test_the_baseline_refuses_what_the_arm_models_refuse(change: dict[str, Any]) -> None:
    with pytest.raises(ValidationError):
        ArmCalibrationSettings.model_validate({**CONFIGURED_VALUES, **change})


def test_the_mount_height_tolerance_is_the_arm_models() -> None:
    assert MOUNT_HEIGHT_TOLERANCE_M == ARM_MODEL_MOUNT_HEIGHT_TOLERANCE_M


def test_an_accepted_calibration_merges_over_the_values_baseline() -> None:
    baseline = ArmCalibrationSettings.model_validate(CONFIGURED_VALUES)
    merged = calibrated_settings(baseline, ArmCalibrationFile.model_validate(body()))
    assert type(merged) is ArmCalibrationSettings
    expected = {"joint_offsets_rad": OFFSETS, "arm_base_height_m": BASE["z"], "base_in_base_link": BASE}
    assert merged.model_dump() == expected


# --- AppliedArmCalibration ----------------------------------------------------------------------------------------


def test_the_baseline_is_a_snapshot_in_use_until_a_calibration_is_used(tmp_path: Path) -> None:
    configured = CONFIGURED.model_copy(deep=True)
    applied = AppliedArmCalibration(configured, tmp_path)
    configured.arm_base_height_m = 0.2
    configured.base_in_base_link = None
    assert applied.baseline == CONFIGURED and applied.current == CONFIGURED
    assert applied.loaded is None and applied.stamp is None


def test_changed_follows_the_marker_identity(tmp_path: Path) -> None:
    applied = AppliedArmCalibration(CONFIGURED, tmp_path)
    assert not applied.changed()  # no applied.json, as before the first call
    accept(tmp_path)
    assert applied.changed() and applied.stamp == applied_marker_identity(tmp_path)
    assert not applied.changed()
    write_marker(tmp_path, {"head_chain": {"session": "s", "result": "r"}})
    assert applied.changed()
    (tmp_path / APPLIED_MARKER_FILE).unlink()
    assert applied.changed() and applied.stamp is None


def test_an_atomic_replace_within_one_mtime_tick_is_a_change(tmp_path: Path) -> None:
    accept(tmp_path)
    applied = AppliedArmCalibration(CONFIGURED, tmp_path)
    assert applied.changed()
    marker = tmp_path / APPLIED_MARKER_FILE
    old = marker.stat()
    replacement = tmp_path / "applied.json.tmp"
    replacement.write_text(marker.read_text())
    os.replace(replacement, marker)
    os.utime(marker, ns=(old.st_atime_ns, old.st_mtime_ns))
    assert marker.stat().st_mtime_ns == old.st_mtime_ns
    assert applied.changed()


def test_read_merges_the_accepted_calibration(tmp_path: Path) -> None:
    accept(tmp_path)
    settings, calibration = AppliedArmCalibration(CONFIGURED, tmp_path).read()
    assert calibration is not None and calibration.result == "r-1"
    assert settings.joint_offsets_rad.model_dump() == OFFSETS


@pytest.mark.parametrize("applied_dir", [None, "empty"], ids=["no-applied-dir", "nothing-accepted"])
def test_read_without_an_accepted_calibration_gives_the_baseline(tmp_path: Path, applied_dir: str | None) -> None:
    settings, calibration = AppliedArmCalibration(CONFIGURED, tmp_path / applied_dir if applied_dir else None).read()
    assert settings == CONFIGURED and calibration is None


def test_read_of_a_broken_file_raises(tmp_path: Path) -> None:
    accept(tmp_path)
    (tmp_path / ARM_CALIBRATION_FILE).write_text("{not json")
    with pytest.raises(ArmCalibrationError, match=ARM_CALIBRATION_FILE):
        AppliedArmCalibration(CONFIGURED, tmp_path).read()


def test_same_and_use_track_the_calibration_in_use(tmp_path: Path) -> None:
    accept(tmp_path)
    applied = AppliedArmCalibration(CONFIGURED, tmp_path)
    assert applied.same(None)
    settings, calibration = applied.read()
    assert not applied.same(calibration)
    assert applied.use(settings, calibration) == CONFIGURED
    assert applied.current is settings and applied.loaded is calibration
    assert applied.same(ArmCalibrationFile.model_validate(body()))
    assert not applied.same(ArmCalibrationFile.model_validate(body("r-2")))


def test_the_consumer_modules_import_no_opencv() -> None:
    """filter_node and web_ui import these modules; OpenCV and scipy stay out of their processes."""
    code = (
        "import sys\n"
        "import ros2_common.head_rgbd_calibration.applied_arm_calibration\n"
        "import ros2_common.head_rgbd_calibration.arm_calibration_file\n"
        "print(sorted(m for m in ('cv2', 'scipy') if m in sys.modules))\n"
    )
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True).stdout
    assert out.strip() == "[]"
