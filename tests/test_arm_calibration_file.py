"""applied/arm_calibration.json (ros2_common.head_rgbd_calibration.arm_calibration_file): the accepted arm to camera
calibration head_rgbd_calibration writes and mcp_server applies, and the `arm:` fields it overrides."""

import json
import math
from pathlib import Path
from typing import Any

import pytest
import yaml
from arm_collision.settings import ArmModelSettings
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.arm_calibration_file import (
    ARM_CALIBRATION_FILE,
    ARM_CALIBRATION_KEY,
    ARM_SETTINGS_FIELDS,
    ArmCalibrationError,
    ArmCalibrationFile,
    read_arm_calibration,
)
from ros2_common.head_rgbd_calibration.arm_camera_fit import calibration_snippet_yaml
from ros2_common.head_rgbd_calibration.urdf_text import APPLIED_MARKER_FILE

OFFSETS = {"shoulder_pan": 0.02, "shoulder_lift": -0.03, "elbow_flex": -0.1, "wrist_flex": 0.1, "wrist_roll": -0.07}
BASE = {"x": 0.061, "y": -0.048, "z": 0.102, "yaw": 0.0}


def body(**changes: Any) -> dict[str, Any]:
    """An arm_calibration.json body as the accept writes it (with the fields mcp_server does not read)."""
    return {
        "session": "calib-20261109-120000",
        "result": "r-20261109-121000",
        "accepted_at": "2026-11-09T12:20:00+00:00",
        "captured_from": "2026-11-09T12:01:00+00:00",
        "score": {"rms_px": 0.9},
        "joint_offsets_rad": dict(OFFSETS),
        "base_in_base_link": dict(BASE),
        "tool_offset_m": [0.001, -0.0056, -0.0014],
        "snippet": "...",
        "head_chain": {"source": "head_arm", "session": "calib-1", "result": "r-1"},
        "accuracy_examples": [],
        **changes,
    }


def write(applied: Path, data: dict[str, Any] | None, record: dict[str, Any] | None) -> None:
    """applied/arm_calibration.json (None: none) and applied.json with the arm_calibration record (None: none)."""
    applied.mkdir(parents=True, exist_ok=True)
    if data is not None:
        (applied / ARM_CALIBRATION_FILE).write_text(json.dumps(data))
    marker: dict[str, Any] = {"head_chain": {"session": "calib-1", "result": "r-1"}}
    if record is not None:
        marker[ARM_CALIBRATION_KEY] = record
    (applied / APPLIED_MARKER_FILE).write_text(json.dumps(marker))


RECORD = {"session": "calib-20261109-120000", "result": "r-20261109-121000", "accepted_at": "x", "score": {}}


def test_the_accept_body_validates_and_ignores_the_report_fields() -> None:
    cal = ArmCalibrationFile.model_validate(body())
    assert (cal.session, cal.result) == ("calib-20261109-120000", "r-20261109-121000")
    assert cal.joint_offsets_rad == OFFSETS and cal.base_in_base_link.model_dump() == BASE
    assert cal.captured_from.tzinfo is not None


@pytest.mark.parametrize(
    "changes",
    [
        {"joint_offsets_rad": {k: v for k, v in OFFSETS.items() if k != "wrist_roll"}},
        {"joint_offsets_rad": {**OFFSETS, "gripper": 0.0}},
        {"joint_offsets_rad": {**OFFSETS, "elbow_flex": math.nan}},
        {"base_in_base_link": {**BASE, "z": math.inf}},
        {"base_in_base_link": {"x": 0.06, "y": -0.05, "z": 0.1}},
        {"captured_from": "2026-11-09T12:01:00"},
        {"session": ""},
    ],
)
def test_an_invalid_body_is_refused(changes: dict[str, Any]) -> None:
    with pytest.raises(ValidationError):
        ArmCalibrationFile.model_validate(body(**changes))


def test_arm_settings_update_names_exactly_the_overridden_fields() -> None:
    update = ArmCalibrationFile.model_validate(body()).arm_settings_update()
    assert tuple(update) == ARM_SETTINGS_FIELDS
    assert update == {"joint_offsets_rad": OFFSETS, "arm_base_height_m": BASE["z"], "base_in_base_link": BASE}


def test_the_update_validates_as_the_mcp_server_arm_model() -> None:
    update = ArmCalibrationFile.model_validate(body()).arm_settings_update()
    arm = ArmModelSettings.model_validate({**ArmModelSettings().model_dump(), **update})
    assert arm.joint_offsets_rad.model_dump() == OFFSETS
    assert arm.base_in_base_link is not None and arm.base_in_base_link.model_dump() == BASE
    assert arm.arm_base_height_m == BASE["z"]


def test_the_reviewed_snippet_sets_the_same_fields() -> None:
    """The client.yml snippet of a fit and the runtime override set the same `arm:` keys (one source of truth)."""

    class Fit:
        offsets = OFFSETS
        base = BASE
        rms_px = 0.9
        holdout_tool_mm = 1.2

    snippet = yaml.safe_load(calibration_snippet_yaml(Fit(), "s", "r", "2026-11-09"))  # type: ignore[arg-type]
    assert set(snippet) == set(ARM_SETTINGS_FIELDS)


def test_nothing_accepted_reads_none(tmp_path: Path) -> None:
    assert read_arm_calibration(tmp_path) is None
    write(tmp_path, body(), None)  # a file without its applied.json record is not in effect
    assert read_arm_calibration(tmp_path) is None


def test_the_accepted_calibration_is_read(tmp_path: Path) -> None:
    write(tmp_path, body(), RECORD)
    cal = read_arm_calibration(tmp_path)
    assert cal is not None and cal.result == "r-20261109-121000" and cal.joint_offsets_rad == OFFSETS


def test_a_record_naming_another_result_is_an_error(tmp_path: Path) -> None:
    write(tmp_path, body(result="r-old"), RECORD)
    with pytest.raises(ArmCalibrationError, match="r-old"):
        read_arm_calibration(tmp_path)


def test_a_missing_or_unreadable_file_is_an_error(tmp_path: Path) -> None:
    write(tmp_path, None, RECORD)
    with pytest.raises(ArmCalibrationError, match=ARM_CALIBRATION_FILE):
        read_arm_calibration(tmp_path)
    (tmp_path / ARM_CALIBRATION_FILE).write_text("{not json")
    with pytest.raises(ArmCalibrationError, match=ARM_CALIBRATION_FILE):
        read_arm_calibration(tmp_path)


def test_an_invalid_file_is_an_error(tmp_path: Path) -> None:
    write(tmp_path, body(joint_offsets_rad={"shoulder_pan": 0.0}), RECORD)
    with pytest.raises(ArmCalibrationError, match="joint_offsets_rad"):
        read_arm_calibration(tmp_path)


def test_an_unreadable_marker_is_an_error(tmp_path: Path) -> None:
    write(tmp_path, body(), RECORD)
    (tmp_path / APPLIED_MARKER_FILE).write_text("[1, 2")
    with pytest.raises(ArmCalibrationError, match=APPLIED_MARKER_FILE):
        read_arm_calibration(tmp_path)
