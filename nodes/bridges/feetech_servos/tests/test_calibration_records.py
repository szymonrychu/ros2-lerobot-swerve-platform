"""Unit tests for calibration_records: audit and latest record files, rest reference conversion, home readings."""

import json
import math
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import pytest

from feetech_servos import calibration_records as records_module
from feetech_servos.calibration_plan import JointCalibrationPlan
from feetech_servos.calibration_records import (
    audit_path,
    convert_rest_reference,
    home_rad,
    record_payload,
    write_records,
)

CREATED_AT = "2026-11-10T12:00:00+00:00"
STAMP = datetime(2026, 11, 10, 12, 0, 0, tzinfo=UTC)
AUDIT_NAME = "servo_calibration-20261110T120000Z-run1.json"
LATEST_NAME = "servo_calibration-arm-latest.json"


def make_payload(failure: tuple[str, str] | None = None) -> dict[str, Any]:
    return record_payload(
        "arm", "run1", "plan1", CREATED_AT, False, {"elbow_flex": {"ok": True}}, failure, [], {"elbow_flex": 1.0}, []
    )


def make_plan(name: str, home: int, target: int, inverted: bool = False) -> JointCalibrationPlan:
    return JointCalibrationPlan(
        name=name,
        id=1,
        inverted=inverted,
        home_steps=home,
        target_steps=target,
        offset_before_steps=0,
        offset_change_steps=home - target,
        new_offset_steps=home - target,
        limits_before=(0, 4095),
        limits_after=None,
        range_deg=None,
        full_turn=False,
        exceeds_max_change=False,
        narrower_than_required=False,
    )


def test_audit_path_names_utc_stamp_and_run_id(tmp_path: Path) -> None:
    """The audit file name carries the run time in UTC and the run id."""
    stamp = datetime(2026, 11, 10, 12, 0, 5, tzinfo=UTC)
    assert audit_path(tmp_path, stamp, "abc123") == tmp_path / "servo_calibration-20261110T120005Z-abc123.json"


def test_write_records_writes_audit_and_latest_on_success(tmp_path: Path) -> None:
    payload = make_payload()
    assert write_records(tmp_path, "arm", STAMP, payload) == []
    assert sorted(p.name for p in tmp_path.iterdir()) == [AUDIT_NAME, LATEST_NAME]
    assert json.loads((tmp_path / LATEST_NAME).read_text()) == payload
    assert json.loads((tmp_path / AUDIT_NAME).read_text())["success"] is True


def test_write_records_skips_latest_on_failure(tmp_path: Path) -> None:
    payload = make_payload(failure=("elbow_flex", "verify failed"))
    assert payload["success"] is False
    assert payload["failure"] == ["elbow_flex", "verify failed"]
    assert write_records(tmp_path, "arm", STAMP, payload) == []
    assert [p.name for p in tmp_path.iterdir()] == [AUDIT_NAME]


def test_write_records_reports_write_errors(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    def broken(*args: Any, **kwargs: Any) -> None:
        raise OSError("disk full")

    monkeypatch.setattr(records_module, "write_json_atomic", broken)
    errors = write_records(tmp_path, "arm", STAMP, make_payload())
    assert len(errors) == 2
    assert all("not written" in e and "disk full" in e for e in errors)


def test_record_changed_at_equals_created_at() -> None:
    payload = make_payload()
    assert payload["changed_at"] == payload["created_at"] == CREATED_AT
    assert payload["version"] == 1
    assert payload["group"] == "arm"


def write_reference(path: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "version": 1,
                "group": "arm",
                "saved_at": "2026-01-01T00:00:00+00:00",
                "joints": {"elbow_flex": 0.0, "wrist_flex": 0.5},
                "steps": {"elbow_flex": 3150, "wrist_flex": 2500},
            }
        )
    )


def test_convert_rest_reference_shifts_steps_and_rad(tmp_path: Path) -> None:
    path = tmp_path / "rest_reference.json"
    write_reference(path)
    assert convert_rest_reference(path, {"elbow_flex": (78, False)}, CREATED_AT) is None
    data = json.loads(path.read_text())
    assert data["steps"] == {"elbow_flex": 3072, "wrist_flex": 2500}
    assert data["joints"]["elbow_flex"] == pytest.approx(math.pi / 2)
    assert data["joints"]["wrist_flex"] == 0.5
    assert data["saved_at"] == CREATED_AT
    assert data["group"] == "arm"
    assert data["version"] == 1


def test_convert_rest_reference_inverted_and_wraps(tmp_path: Path) -> None:
    path = tmp_path / "rest_reference.json"
    write_reference(path)
    assert convert_rest_reference(path, {"elbow_flex": (-1000, True)}, CREATED_AT) is None
    data = json.loads(path.read_text())
    assert data["steps"]["elbow_flex"] == (3150 + 1000) % 4096
    assert data["joints"]["elbow_flex"] == pytest.approx(-(54 - 2048) * 2 * math.pi / 4096)


def test_convert_rest_reference_absent_file_is_noop(tmp_path: Path) -> None:
    path = tmp_path / "rest_reference.json"
    assert convert_rest_reference(path, {"elbow_flex": (78, False)}, CREATED_AT) is None
    assert list(tmp_path.iterdir()) == []


def test_convert_rest_reference_invalid_file_returns_error(tmp_path: Path) -> None:
    path = tmp_path / "rest_reference.json"
    path.write_text("{not json")
    error = convert_rest_reference(path, {"elbow_flex": (78, False)}, CREATED_AT)
    assert error is not None
    assert "rest reference" in error
    assert path.read_text() == "{not json"


def test_home_rad_uses_targets_and_inversion() -> None:
    plans = {
        "elbow_flex": make_plan("elbow_flex", 3150, 3072),
        "wrist_flex": make_plan("wrist_flex", 100, 3072, inverted=True),
        "gripper": make_plan("gripper", 2048, 2048),
    }
    result = home_rad(plans)
    assert result["elbow_flex"] == pytest.approx(math.pi / 2)
    assert result["wrist_flex"] == pytest.approx(-math.pi / 2)
    assert result["gripper"] == 0.0
