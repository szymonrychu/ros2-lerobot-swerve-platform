"""The guard's accepted arm calibration (filter_node.arm_calibration): loaded before the guard is built, re-read when
applied/applied.json changes and swapped into the running checker; client.yml's arm values while nothing is accepted."""

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import pytest
from arm_collision.ik import ArmKinematics
from arm_collision.jaw import JawModel
from arm_collision.self_collision import SelfCollisionChecker
from arm_collision.settings import ArmBaseOffset, ArmModelSettings, BodyCheckSettings, load_robot_body
from ros2_common.head_rgbd_calibration.arm_calibration_file import ARM_CALIBRATION_FILE, ARM_CALIBRATION_KEY
from ros2_common.head_rgbd_calibration.urdf_text import APPLIED_MARKER_FILE

from filter_node.arm_calibration import GuardCalibration, swap_checker_calibration

from .arm_fixture import ARM, DRIVE_POSE, HARD_MARGIN_M, PARKED_HOME, WHEEL_FR_INSIDE

SESSION = "calib-20261109-120000"
OFFSETS = {"shoulder_pan": 0.03, "shoulder_lift": -0.02, "elbow_flex": -0.09, "wrist_flex": 0.08, "wrist_roll": -0.07}
BASE = {"x": 0.071, "y": -0.04, "z": 0.104, "yaw": 0.02}


def accept(applied: Path, result: str = "r-1", offsets: dict[str, float] | None = None, **base: float) -> None:
    """Write arm_calibration.json, then the applied.json record, as the head_rgbd_calibration accept does.

    Args:
        applied (Path): Applied directory.
        result (str): Result id.
        offsets (dict[str, float] | None): Joint offsets; None for OFFSETS.
        **base (float): Base pose keys replacing BASE's.
    """
    applied.mkdir(parents=True, exist_ok=True)
    body = {
        "session": SESSION,
        "result": result,
        "accepted_at": "2026-11-09T12:20:00+00:00",
        "captured_from": "2026-11-09T12:01:00+00:00",
        "score": {"rms_px": 0.9},
        "joint_offsets_rad": offsets or OFFSETS,
        "base_in_base_link": {**BASE, **base},
        "tool_offset_m": [0.5, 0.5, 0.5],
    }
    (applied / ARM_CALIBRATION_FILE).write_text(json.dumps(body))
    marker_path = applied / APPLIED_MARKER_FILE
    marker = json.loads(marker_path.read_text()) if marker_path.exists() else {}
    marker[ARM_CALIBRATION_KEY] = {"session": SESSION, "result": result, "accepted_at": body["accepted_at"]}
    write_marker(applied, marker)


def write_marker(applied: Path, marker: dict[str, Any]) -> None:
    """applied.json with an mtime one second past the previous one (coarse clocks).

    Args:
        applied (Path): Applied directory.
        marker (dict[str, Any]): applied.json content.
    """
    path = applied / APPLIED_MARKER_FILE
    old = path.stat().st_mtime_ns if path.exists() else 0
    path.write_text(json.dumps(marker))
    stat = path.stat()
    os.utime(path, ns=(stat.st_atime_ns, max(stat.st_mtime_ns, old + 1_000_000_000)))


def accepted_arm(**base: float) -> ArmModelSettings:
    """ARM with OFFSETS and BASE (the merged settings an accept gives).

    Args:
        **base (float): Base pose keys replacing BASE's.

    Returns:
        ArmModelSettings: The settings.
    """
    mount = {**BASE, **base}
    return ARM.model_copy(
        update={
            "joint_offsets_rad": ARM.joint_offsets_rad.model_validate(OFFSETS),
            "base_in_base_link": ArmBaseOffset(**mount),
            "arm_base_height_m": mount["z"],
        }
    )


def checker_for(arm: ArmModelSettings) -> SelfCollisionChecker:
    """A fresh guard checker of an arm (as filter_node.node.build_arm_guard builds it).

    Args:
        arm (ArmModelSettings): Arm settings.

    Returns:
        SelfCollisionChecker: The checker.
    """
    kin = ArmKinematics(
        arm.urdf_path,
        margin=0.0,
        joint_offsets=arm.joint_offsets_rad.model_dump(),
        tool_offset=tuple(arm.tool_offset_m.model_dump().values()),
        limit_overrides=arm.joint_limit_overrides_rad,
    )
    jaw = JawModel(kin, arm.jaw_open_axis, arm.gripper_closed_rad)
    return SelfCollisionChecker.from_settings(
        kin, load_robot_body(), BodyCheckSettings(hard_margin_m=HARD_MARGIN_M), arm, jaw
    )


def info_lines(log: MagicMock) -> list[str]:
    return [str(c.args[0]) for c in log.info.call_args_list]


def error_lines(log: MagicMock) -> list[str]:
    return [str(c.args[0]) for c in log.error.call_args_list]


# --- startup ------------------------------------------------------------------------------------------------------


def test_startup_without_an_accepted_calibration_uses_the_configured_arm(tmp_path: Path) -> None:
    log = MagicMock()
    calibration = GuardCalibration(ARM, tmp_path, log)
    assert calibration.startup() == ARM and calibration.loaded is None
    assert any("client.yml" in line for line in info_lines(log))


def test_startup_merges_the_accepted_calibration(tmp_path: Path) -> None:
    accept(tmp_path)
    log = MagicMock()
    calibration = GuardCalibration(ARM, tmp_path, log)
    arm = calibration.startup()
    assert arm == accepted_arm() and arm.tool_offset_m == ARM.tool_offset_m
    assert calibration.loaded is not None and calibration.loaded.result == "r-1"
    assert any(SESSION in line and "r-1" in line for line in info_lines(log))


def test_startup_with_an_invalid_file_uses_the_configured_arm_and_logs_an_error(tmp_path: Path) -> None:
    accept(tmp_path, offsets={"shoulder_pan": 0.1})
    log = MagicMock()
    calibration = GuardCalibration(ARM, tmp_path, log)
    assert calibration.startup() == ARM and calibration.loaded is None
    (line,) = error_lines(log)
    assert "joint_offsets_rad" in line and "client.yml" in line


def test_no_applied_dir_uses_the_configured_arm(tmp_path: Path) -> None:
    accept(tmp_path)
    calibration = GuardCalibration(ARM, None, MagicMock())
    assert calibration.startup() == ARM
    apply = MagicMock()
    assert not calibration.tick(apply)
    apply.assert_not_called()


# --- tick ---------------------------------------------------------------------------------------------------------


def test_tick_without_a_change_applies_nothing(tmp_path: Path) -> None:
    accept(tmp_path)
    calibration = GuardCalibration(ARM, tmp_path, MagicMock())
    calibration.startup()
    (tmp_path / ARM_CALIBRATION_FILE).write_text("{broken, but applied.json did not change")
    apply = MagicMock()
    assert not calibration.tick(apply)
    apply.assert_not_called()


def test_an_accept_after_startup_is_applied(tmp_path: Path) -> None:
    log = MagicMock()
    calibration = GuardCalibration(ARM, tmp_path, log)
    calibration.startup()
    accept(tmp_path)
    apply = MagicMock()
    assert calibration.tick(apply)
    apply.assert_called_once_with(accepted_arm())
    assert calibration.current == accepted_arm() and calibration.loaded is not None
    assert any("r-1" in line for line in info_lines(log))


def test_a_new_accept_replaces_the_previous_one(tmp_path: Path) -> None:
    accept(tmp_path)
    calibration = GuardCalibration(ARM, tmp_path, MagicMock())
    calibration.startup()
    accept(tmp_path, result="r-2", x=0.065)
    apply = MagicMock()
    assert calibration.tick(apply)
    assert apply.call_args.args[0].base_in_base_link.x == 0.065
    assert calibration.loaded is not None and calibration.loaded.result == "r-2"


def test_a_head_chain_apply_alone_applies_nothing(tmp_path: Path) -> None:
    accept(tmp_path)
    calibration = GuardCalibration(ARM, tmp_path, MagicMock())
    calibration.startup()
    marker = json.loads((tmp_path / APPLIED_MARKER_FILE).read_text())
    write_marker(tmp_path, {**marker, "head_chain": {"session": "s2", "result": "r2"}})
    apply = MagicMock()
    assert not calibration.tick(apply)
    apply.assert_not_called()


def test_an_invalid_reload_keeps_the_values_in_use(tmp_path: Path) -> None:
    accept(tmp_path)
    log = MagicMock()
    calibration = GuardCalibration(ARM, tmp_path, log)
    calibration.startup()
    (tmp_path / ARM_CALIBRATION_FILE).write_text("{not json")
    write_marker(tmp_path, json.loads((tmp_path / APPLIED_MARKER_FILE).read_text()) | {"touched": True})
    apply = MagicMock()
    assert not calibration.tick(apply)
    apply.assert_not_called()
    assert calibration.current == accepted_arm() and calibration.loaded is not None
    (line,) = error_lines(log)
    assert ARM_CALIBRATION_FILE in line and "r-1" in line


def test_a_removed_record_falls_back_to_the_configured_arm(tmp_path: Path) -> None:
    accept(tmp_path)
    calibration = GuardCalibration(ARM, tmp_path, MagicMock())
    calibration.startup()
    write_marker(tmp_path, {"head_chain": {"session": "s", "result": "r"}})
    apply = MagicMock()
    assert calibration.tick(apply)
    apply.assert_called_once_with(ARM)
    assert calibration.loaded is None


def test_a_failing_swap_keeps_the_values_in_use_and_never_raises(tmp_path: Path) -> None:
    log = MagicMock()
    calibration = GuardCalibration(ARM, tmp_path, log)
    calibration.startup()
    accept(tmp_path)
    apply = MagicMock(side_effect=RuntimeError("kernel build failed"))
    assert not calibration.tick(apply)
    assert not calibration.tick(apply)  # no retry until applied.json changes again
    assert apply.call_count == 1
    assert calibration.loaded is None and calibration.current == ARM
    (line,) = error_lines(log)
    assert "kernel build failed" in line


def test_an_unexpected_read_error_keeps_the_values_in_use_and_never_raises(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    accept(tmp_path)
    log = MagicMock()
    calibration = GuardCalibration(ARM, tmp_path, log)
    calibration.startup()
    accept(tmp_path, result="r-2", x=0.065)
    monkeypatch.setattr(calibration, "read", MagicMock(side_effect=OSError("disk vanished")))
    apply = MagicMock()
    assert not calibration.tick(apply)
    apply.assert_not_called()
    assert calibration.current == accepted_arm() and calibration.loaded is not None
    assert calibration.loaded.result == "r-1"
    (line,) = error_lines(log)
    assert "disk vanished" in line and "r-1" in line


# --- swap_checker_calibration -------------------------------------------------------------------------------------


@pytest.mark.parametrize("pose", [PARKED_HOME, DRIVE_POSE, WHEEL_FR_INSIDE], ids=["home", "drive", "wheel"])
def test_the_swapped_checker_checks_like_one_built_with_the_calibration(pose: dict[str, float]) -> None:
    checker = checker_for(ARM)
    swap_checker_calibration(checker, accepted_arm())
    reference = checker_for(accepted_arm())
    assert checker.mount == ArmBaseOffset(**BASE)
    assert checker.kin.offsets == OFFSETS
    assert checker.pose(pose).min_clearance == pytest.approx(reference.pose(pose).min_clearance, abs=1e-9)
    assert checker.pose(pose).min_clearance != pytest.approx(checker_for(ARM).pose(pose).min_clearance, abs=1e-6)


def test_no_mount_swaps_to_the_mount_height_above_base_link() -> None:
    checker = checker_for(ARM)
    swap_checker_calibration(checker, ARM.model_copy(update={"base_in_base_link": None, "arm_base_height_m": 0.11}))
    assert checker.mount == ArmBaseOffset(z=0.11)


def test_the_guard_calibration_imports_no_opencv() -> None:
    """ros2-common is a dependency for its OpenCV-free modules only: cv2 stays out of the teleop process."""
    code = "import sys\nimport filter_node.arm_calibration\nprint('cv2' in sys.modules)\n"
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True).stdout
    assert out.strip() == "False"
