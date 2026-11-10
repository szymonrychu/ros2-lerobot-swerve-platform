"""calibrate_servos zero changes write the bridge's zero change record and stale marker (fake bus, never hardware).

Every CLI path that changes a servo zero (calibrate and center send DefineMiddle; set, write and load-config can write
the offset register or torque_enable = 128) resolves the servo through --bridge-config and writes
zero_record.write_zero_change for a joint of a group with neutral_stale_marker, as the bridge's neutral save does.
"""

import json
import logging
import os
import sys
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock, patch

import pytest

scripts_dir = Path(__file__).resolve().parent.parent / "scripts"
if str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))

import calibrate_servos as m  # noqa: E402

DEVICE = "/dev/tty"
MARKER_NAME = "calibration_stale.json"
RECORD_NAME = "zero_changed.json"
POSITION = "present_position"


class FakeBus:
    """A servo bus double: per-servo present_position, the DefineMiddle effect and every write, in order.

    Attributes:
        present (dict[int, int]): servo id -> present_position steps.
        writes (list[tuple[int, str, int]]): (servo id, register, value) of every register write.
        middles (list[int]): servo ids DefineMiddle was sent to.
        middle_result (bool): What DefineMiddle returns.
        middle_raises (bool): DefineMiddle raises OSError instead (the command may still have landed).
        centre_to (int): present_position a centred servo reads afterwards.
    """

    def __init__(self, present: dict[int, int]) -> None:
        self.present = dict(present)
        self.writes: list[tuple[int, str, int]] = []
        self.middles: list[int] = []
        self.middle_result = True
        self.middle_raises = False
        self.centre_to = 2048
        self.servo = MagicMock()
        self.servo.DefineMiddle.side_effect = self.define_middle
        self.servo.ReadPosition.side_effect = lambda sid: self.present.get(sid)
        self.servo.ListServos.return_value = sorted(self.present)

    def define_middle(self, sid: int) -> bool:
        """Record DefineMiddle and centre the servo (also when it reports failure or raises)."""
        self.middles.append(sid)
        self.present[sid] = self.centre_to
        if self.middle_raises:
            raise OSError("serial timeout")
        return self.middle_result

    def read_register(self, _servo: Any, sid: int, entry: Any) -> int | None:
        """read_register double: present_position from the bus state, other registers 0."""
        return self.present.get(sid) if entry.name == POSITION else 0

    def write_register(self, _servo: Any, sid: int, entry: Any, value: int, _cache: dict[str, int]) -> bool:
        """write_register double: an offset write moves present_position by the offset change; 128 centres."""
        self.writes.append((sid, entry.name, value))
        if entry.name == "offset":
            self.present[sid] = self.present.get(sid, 0) - value
        if entry.name == "torque_enable" and value == 128:
            self.present[sid] = self.centre_to
        return True


def bridge_config(tmp_path: Path, device: str = DEVICE, marker: Path | None = None) -> Path:
    """Write a bridge config: follower arm (ids 1, 3, 6) with the stale marker, swerve_drive (id 33) without.

    Args:
        tmp_path (Path): Test directory.
        device (str): The config's serial device.
        marker (Path | None): Stale marker path; default tmp_path/arm/calibration_stale.json.

    Returns:
        Path: The config file.
    """
    marker = marker or tmp_path / "arm" / MARKER_NAME
    path = tmp_path / "config.yaml"
    path.write_text(
        f"namespace: follower\ndevice: {device}\nneutral_stale_marker: {marker}\n"
        "joint_names:\n"
        "  - name: shoulder_pan\n    id: 1\n"
        "  - name: elbow_flex\n    id: 3\n"
        "  - name: gripper\n    id: 6\n"
        "extra_groups:\n"
        "  - namespace: swerve_drive\n"
        "    joint_names:\n      - name: fl_steer\n        id: 33\n"
    )
    return path


def run(bus: FakeBus, argv: list[str]) -> int:
    """Run calibrate_servos main over the fake bus (no sleeps).

    Args:
        bus (FakeBus): The bus double.
        argv (list[str]): Arguments after the program name.

    Returns:
        int: The exit code.
    """
    with (
        patch.object(m, "ST3215", return_value=bus.servo),
        patch.object(m, "read_register", bus.read_register),
        patch.object(m, "write_register", bus.write_register),
        patch.object(m, "time") as fake_time,
        patch("builtins.input", return_value=""),
    ):
        fake_time.monotonic.side_effect = [0.0, 0.0, 99.0] * 10  # one range sample per joint in calibrate
        fake_time.sleep = MagicMock()
        with patch.object(sys, "argv", ["calibrate_servos", *argv]):
            return m.main()


def record(tmp_path: Path) -> dict[str, Any]:
    """The zero change record next to the default marker."""
    return json.loads((tmp_path / "arm" / RECORD_NAME).read_text())


def marker(tmp_path: Path) -> dict[str, Any]:
    """The default stale marker."""
    return json.loads((tmp_path / "arm" / MARKER_NAME).read_text())


# --- center --------------------------------------------------------------------------------------------------------


def test_center_of_an_arm_joint_writes_the_record_and_the_marker(tmp_path: Path) -> None:
    bus = FakeBus({3: 2600})
    config = bridge_config(tmp_path)
    assert run(bus, ["center", "--device", DEVICE, "--id", "3", "--bridge-config", str(config)]) == 0
    assert record(tmp_path)["group"] == "follower" and record(tmp_path)["version"] == 1
    content = marker(tmp_path)
    assert content["joints"] == {"elbow_flex": {"before_steps": 2600, "after_steps": 2048}}
    assert content["stale"] == ["joint_offsets", "home"]
    assert content["saved_at"] == record(tmp_path)["changed_at"]


def test_center_not_verified_writes_only_the_record(tmp_path: Path) -> None:
    bus = FakeBus({3: 2600})
    bus.centre_to = 500
    assert run(bus, ["center", "--device", DEVICE, "--id", "3", "--bridge-config", str(bridge_config(tmp_path))]) == 1
    assert record(tmp_path)["group"] == "follower"
    assert not (tmp_path / "arm" / MARKER_NAME).exists()
    assert bus.writes == []  # no limits


def test_center_records_a_define_middle_that_failed_or_raised(tmp_path: Path) -> None:
    for attribute in ("middle_result", "middle_raises"):
        bus = FakeBus({3: 2600})
        setattr(bus, attribute, attribute == "middle_raises")
        config = bridge_config(tmp_path)
        (tmp_path / "arm" / RECORD_NAME).unlink(missing_ok=True)
        assert run(bus, ["center", "--device", DEVICE, "--id", "3", "--bridge-config", str(config)]) == 1
        assert record(tmp_path)["group"] == "follower", attribute
        bus.servo.LockEprom.assert_called_with(3)


def test_center_of_a_group_without_marker_writes_nothing(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    bus = FakeBus({33: 1000})
    assert run(bus, ["center", "--device", DEVICE, "--id", "33", "--bridge-config", str(bridge_config(tmp_path))]) == 0
    assert not (tmp_path / "arm").exists()
    assert "swerve_drive has no neutral_stale_marker" in capsys.readouterr().err


def test_center_of_a_servo_not_in_the_config_writes_nothing(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    bus = FakeBus({40: 1000})
    assert run(bus, ["center", "--device", DEVICE, "--id", "40", "--bridge-config", str(bridge_config(tmp_path))]) == 0
    assert not (tmp_path / "arm").exists()
    assert "servo 40 is not a joint" in capsys.readouterr().err


def test_center_without_bridge_config_is_refused_before_the_bus(capsys: pytest.CaptureFixture[str]) -> None:
    bus = FakeBus({3: 2600})
    assert run(bus, ["center", "--device", DEVICE, "--id", "3"]) == 1
    assert bus.middles == [] and bus.writes == []
    assert "--bridge-config" in capsys.readouterr().err


def test_center_with_an_invalid_bridge_config_is_refused(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    bus = FakeBus({3: 2600})
    broken = tmp_path / "broken.yaml"
    broken.write_text("namespace: follower\n")
    assert run(bus, ["center", "--device", DEVICE, "--id", "3", "--bridge-config", str(broken)]) == 1
    assert bus.middles == []
    assert "cannot load bridge config" in capsys.readouterr().err


def test_center_on_another_device_is_refused(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    bus = FakeBus({3: 2600})
    config = bridge_config(tmp_path, device="/dev/null")
    assert run(bus, ["center", "--device", DEVICE, "--id", "3", "--bridge-config", str(config)]) == 1
    assert bus.middles == []
    assert "/dev/null" in capsys.readouterr().err


def test_center_as_another_user_than_the_record_dir_owner_is_refused(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    bus = FakeBus({3: 2600})
    config = bridge_config(tmp_path)
    monkeypatch.setattr(m.os, "geteuid", lambda: os.stat(tmp_path).st_uid + 1)
    assert run(bus, ["center", "--device", DEVICE, "--id", "3", "--bridge-config", str(config)]) == 1
    assert bus.middles == [] and bus.writes == []
    err = capsys.readouterr().err
    assert "sudo -u" in err and "nothing written to the servos" in err


def test_center_whose_record_write_fails_exits_one(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    bus = FakeBus({3: 2600})
    config = bridge_config(tmp_path)
    monkeypatch.setattr(m, "write_zero_change", lambda *a: ["zero change record not written: OSError: disk full"])
    assert run(bus, ["center", "--device", DEVICE, "--id", "3", "--bridge-config", str(config)]) == 1
    err = capsys.readouterr().err
    assert "servo 3 (follower/elbow_flex)" in err and "zero change record not written" in err


# --- calibrate -----------------------------------------------------------------------------------------------------


def calibrate_args(tmp_path: Path, joints: int) -> list[str]:
    """calibrate arguments over the default bridge config."""
    out = tmp_path / "cal.json"
    config = bridge_config(tmp_path)
    return [
        "calibrate",
        "--device",
        DEVICE,
        "--output",
        str(out),
        "--expected-joints",
        str(joints),
        "--bridge-config",
        str(config),
    ]


def test_calibrate_records_every_arm_joint(tmp_path: Path) -> None:
    bus = FakeBus({1: 1500, 3: 2600})
    assert run(bus, calibrate_args(tmp_path, 2)) == 0
    assert bus.middles == [1, 3]
    assert marker(tmp_path)["joints"] == {
        "shoulder_pan": {"before_steps": 1500, "after_steps": 2048},
        "elbow_flex": {"before_steps": 2600, "after_steps": 2048},
    }
    assert record(tmp_path)["group"] == "follower"


def test_calibrate_records_a_failed_define_middle_before_exiting(tmp_path: Path) -> None:
    bus = FakeBus({1: 1500, 3: 2600})
    bus.middle_result = False
    assert run(bus, calibrate_args(tmp_path, 2)) == 1
    assert bus.middles == [1]
    assert record(tmp_path)["group"] == "follower"
    assert not (tmp_path / "arm" / MARKER_NAME).exists()


def test_calibrate_prints_the_define_middle_error_when_the_record_write_also_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    bus = FakeBus({1: 1500, 3: 2600})
    bus.middle_raises = True
    monkeypatch.setattr(m, "write_zero_change", lambda *a: ["zero change record not written: OSError: disk full"])
    assert run(bus, calibrate_args(tmp_path, 2)) == 1
    assert bus.middles == [1]
    err = capsys.readouterr().err
    assert "zero change record not written" in err
    assert "failed to set center for servo 1" in err and "OSError: serial timeout" in err


def test_define_middle_that_raises_logs_the_traceback(caplog: pytest.LogCaptureFixture) -> None:
    servo = MagicMock()
    servo.DefineMiddle.side_effect = OSError("serial timeout")
    with caplog.at_level(logging.ERROR):
        error = m.define_middle(servo, 3)
    assert error == "DefineMiddle (torque_enable=128) raised OSError: serial timeout"
    assert caplog.records and caplog.records[0].exc_info is not None
    assert "Traceback" in caplog.text and "servo 3" in caplog.text


def test_calibrate_without_bridge_config_is_refused_before_define_middle(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    bus = FakeBus({1: 1500, 3: 2600})
    out = tmp_path / "unused.json"
    assert run(bus, ["calibrate", "--device", DEVICE, "--output", str(out), "--expected-joints", "2"]) == 1
    assert bus.middles == []
    assert "--bridge-config" in capsys.readouterr().err


# --- set / write -----------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("command", ["set", "write"])
def test_offset_write_records_the_change(tmp_path: Path, command: str) -> None:
    bus = FakeBus({1: 2100})
    config = bridge_config(tmp_path)
    argv = [
        command,
        "--device",
        DEVICE,
        "--id",
        "1",
        "--register",
        "offset",
        "--value",
        "52",
        "--bridge-config",
        str(config),
    ]
    assert run(bus, argv) == 0
    assert bus.writes == [(1, "offset", 52)]
    assert marker(tmp_path)["joints"] == {"shoulder_pan": {"before_steps": 2100, "after_steps": 2048}}
    assert record(tmp_path)["group"] == "follower"


@pytest.mark.parametrize("command", ["set", "write"])
def test_define_middle_through_torque_enable_records_the_change(tmp_path: Path, command: str) -> None:
    bus = FakeBus({3: 2600})
    config = bridge_config(tmp_path)
    argv = [
        command,
        "--device",
        DEVICE,
        "--id",
        "3",
        "--register",
        "torque_enable",
        "--value",
        "128",
        "--bridge-config",
        str(config),
    ]
    assert run(bus, argv) == 0
    assert marker(tmp_path)["joints"] == {"elbow_flex": {"before_steps": 2600, "after_steps": 2048}}


@pytest.mark.parametrize("command", ["set", "write"])
def test_offset_write_without_bridge_config_is_refused(command: str, capsys: pytest.CaptureFixture[str]) -> None:
    bus = FakeBus({1: 2100})
    assert run(bus, [command, "--device", DEVICE, "--id", "1", "--register", "offset", "--value", "52"]) == 1
    assert bus.writes == []
    assert "--bridge-config" in capsys.readouterr().err


def test_failed_offset_write_still_records_without_marker(tmp_path: Path) -> None:
    bus = FakeBus({1: 2100})
    bus.write_register = lambda *a: False  # type: ignore[method-assign]  # no reply: the write may have landed
    config = bridge_config(tmp_path)
    argv = [
        "write",
        "--device",
        DEVICE,
        "--id",
        "1",
        "--register",
        "offset",
        "--value",
        "52",
        "--bridge-config",
        str(config),
    ]
    assert run(bus, argv) == 1
    assert record(tmp_path)["group"] == "follower"
    assert not (tmp_path / "arm" / MARKER_NAME).exists()


def test_other_register_writes_need_no_bridge_config(tmp_path: Path) -> None:
    bus = FakeBus({1: 2100})
    assert run(bus, ["write", "--device", DEVICE, "--id", "1", "--register", "torque_enable", "--value", "0"]) == 0
    assert bus.writes == [(1, "torque_enable", 0)]


# --- load-config ---------------------------------------------------------------------------------------------------


def test_load_config_with_offsets_records_each_arm_joint(tmp_path: Path) -> None:
    bus = FakeBus({1: 2100, 3: 2600, 33: 1000})
    regs = tmp_path / "regs.json"
    regs.write_text(json.dumps({"1": {"offset": 52, "p_coefficient": 32}, "3": {"offset": 552}, "33": {"offset": 7}}))
    config = bridge_config(tmp_path)
    assert run(bus, ["load-config", "--device", DEVICE, "--file", str(regs), "--bridge-config", str(config)]) == 0
    assert marker(tmp_path)["joints"] == {
        "shoulder_pan": {"before_steps": 2100, "after_steps": 2048},
        "elbow_flex": {"before_steps": 2600, "after_steps": 2048},
    }
    assert (33, "offset", 7) in bus.writes


def test_load_config_with_offsets_without_bridge_config_is_refused(tmp_path: Path) -> None:
    bus = FakeBus({1: 2100})
    regs = tmp_path / "regs.json"
    regs.write_text(json.dumps({"1": {"p_coefficient": 32, "offset": 52}}))
    assert run(bus, ["load-config", "--device", DEVICE, "--file", str(regs)]) == 1
    assert bus.writes == []


def test_load_config_without_zero_registers_needs_no_bridge_config(tmp_path: Path) -> None:
    bus = FakeBus({5: 2048})
    regs = tmp_path / "regs.json"
    regs.write_text(json.dumps({"5": {"p_coefficient": 16}}))
    assert run(bus, ["load-config", "--device", DEVICE, "--file", str(regs)]) == 0
    assert bus.writes == [(5, "p_coefficient", 16)]


# --- helpers ---------------------------------------------------------------------------------------------------------


def test_changes_zero() -> None:
    assert m.changes_zero("offset", 0) and m.changes_zero("torque_enable", 128)
    assert not m.changes_zero("torque_enable", 1) and not m.changes_zero("goal_position", 128)


def test_zero_change_ids_lists_servos_with_zero_registers() -> None:
    config = {"1": {"offset": 3}, "2": {"min": 0}, "3": {"torque_enable": 128}, "x": {"offset": 1}, "4": "bad"}
    assert m.zero_change_ids(config) == [1, 3]
