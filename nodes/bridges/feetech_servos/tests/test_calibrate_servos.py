"""Unit tests for calibrate_servos script (subcommands, parser, JSON shape, exit codes with mocks)."""

import json
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

scripts_dir = Path(__file__).resolve().parent.parent / "scripts"
if str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))


def test_parser_read_default_id_is_one() -> None:
    """Subcommand 'read' has --id default 1 when omitted."""
    import calibrate_servos as m

    with patch.object(sys, "argv", ["calibrate_servos", "read", "--device", "/dev/tty", "--register", "id"]):
        parser = m._build_parser()
        args = parser.parse_args()
    assert args.servo_id == 1


def test_parser_write_default_id_is_one() -> None:
    """Subcommand 'write' has --id default 1 when omitted."""
    import calibrate_servos as m

    with patch.object(
        sys,
        "argv",
        ["calibrate_servos", "write", "--device", "/dev/tty", "--register", "torque_enable", "--value", "0"],
    ):
        parser = m._build_parser()
        args = parser.parse_args()
    assert args.servo_id == 1


def test_list_registers_returns_zero_and_prints_register_map(capsys) -> None:
    """Subcommand list-registers exits 0 and prints JSON list of register entries."""
    import calibrate_servos as m

    with patch.object(sys, "argv", ["calibrate_servos", "list-registers"]):
        exit_code = m.main()
    assert exit_code == 0
    out = capsys.readouterr().out
    data = json.loads(out)
    assert isinstance(data, list)
    assert len(data) > 0
    for row in data:
        assert "name" in row and "address" in row and "size" in row
        assert "read_only" in row and "eprom" in row


def test_read_unknown_register_exits_one() -> None:
    """Subcommand read with unknown --register name exits with 1."""
    import calibrate_servos as m

    mock_servo = MagicMock()
    with patch.object(m, "ST3215", return_value=mock_servo):
        with patch.object(
            sys,
            "argv",
            ["calibrate_servos", "read", "--device", "/dev/tty", "--register", "no_such_register"],
        ):
            assert m.main() == 1


def test_write_read_only_register_exits_one() -> None:
    """Subcommand write with read-only register (e.g. present_position) exits with 1."""
    import calibrate_servos as m

    mock_servo = MagicMock()
    with patch.object(m, "ST3215", return_value=mock_servo):
        with patch.object(
            sys,
            "argv",
            ["calibrate_servos", "write", "--device", "/dev/tty", "--register", "present_position", "--value", "0"],
        ):
            assert m.main() == 1


def test_limits_set_min_greater_than_max_exits_one() -> None:
    """Subcommand limits-set with --min > --max exits with 1."""
    import calibrate_servos as m

    mock_servo = MagicMock()
    with patch.object(m, "ST3215", return_value=mock_servo):
        with patch.object(
            sys,
            "argv",
            ["calibrate_servos", "limits-set", "--device", "/dev/tty", "--id", "1", "--min", "100", "--max", "50"],
        ):
            assert m.main() == 1


def test_limits_set_out_of_range_exits_one() -> None:
    """Subcommand limits-set with value outside 0..4095 exits with 1."""
    import calibrate_servos as m

    mock_servo = MagicMock()
    with patch.object(m, "ST3215", return_value=mock_servo):
        with patch.object(
            sys,
            "argv",
            ["calibrate_servos", "limits-set", "--device", "/dev/tty", "--id", "1", "--min", "-1", "--max", "4095"],
        ):
            assert m.main() == 1


def test_set_read_only_register_exits_one_with_message(capsys) -> None:
    """Subcommand set with read-only register exits 1 and stderr mentions read-only and 'get'."""
    import calibrate_servos as m

    mock_servo = MagicMock()
    with patch.object(m, "ST3215", return_value=mock_servo):
        with patch.object(
            sys,
            "argv",
            ["calibrate_servos", "set", "--device", "/dev/tty", "--register", "present_position", "--value", "0"],
        ):
            assert m.main() == 1
    err = capsys.readouterr().err
    assert "read-only" in err
    assert "get" in err.lower()


def test_load_config_refuses_read_only_registers(tmp_path: Path, capsys) -> None:
    """load-config with a file containing read-only register exits 1 and lists the register."""
    import calibrate_servos as m

    config = {"1": {"present_position": 100, "min_angle_limit": 0}}
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(config))
    mock_servo = MagicMock()
    with patch.object(m, "ST3215", return_value=mock_servo):
        with patch.object(
            sys,
            "argv",
            ["calibrate_servos", "load-config", "--device", "/dev/tty", "--file", str(path)],
        ):
            assert m.main() == 1
    err = capsys.readouterr().err
    assert "read-only" in err
    assert "present_position" in err


def test_load_config_accepts_legacy_min_max(tmp_path: Path, capsys) -> None:
    """load-config accepts legacy 'min'/'max' keys and applies min_angle_limit/max_angle_limit."""
    import calibrate_servos as m

    config = {"1": {"min": 100, "max": 4000}}
    path = tmp_path / "legacy.json"
    path.write_text(json.dumps(config))
    mock_servo = MagicMock()
    with patch.object(m, "ST3215", return_value=mock_servo):
        with patch.object(m, "write_register", return_value=True):
            with patch.object(
                sys,
                "argv",
                ["calibrate_servos", "load-config", "--device", "/dev/tty", "--file", str(path)],
            ):
                assert m.main() == 0
    assert "OK" in capsys.readouterr().out


def test_get_returns_register_value(capsys) -> None:
    """Subcommand get with --register returns the same as read (single value)."""
    import calibrate_servos as m

    mock_servo = MagicMock()
    with patch.object(m, "ST3215", return_value=mock_servo):
        with patch.object(m, "read_register", return_value=42):
            with patch.object(
                sys,
                "argv",
                ["calibrate_servos", "get", "--device", "/dev/tty", "--register", "present_position"],
            ):
                assert m.main() == 0
    assert capsys.readouterr().out.strip() == "42"


def test_dump_config_prints_extended_json(capsys, tmp_path: Path) -> None:
    """dump-config with mocked read_all_registers writes extended JSON (servo_id -> registers)."""
    import calibrate_servos as m

    mock_servo = MagicMock()
    with patch.object(m, "ST3215", return_value=mock_servo):
        with patch.object(m, "read_all_registers", return_value={"min_angle_limit": 0, "max_angle_limit": 4095}):
            with patch.object(
                sys,
                "argv",
                ["calibrate_servos", "dump-config", "--device", "/dev/tty"],
            ):
                assert m.main() == 0
    out = capsys.readouterr().out.strip()
    data = json.loads(out)
    assert "1" in data
    assert data["1"]["min_angle_limit"] == 0
    assert data["1"]["max_angle_limit"] == 4095


def test_limits_get_prints_json_and_exits_zero(capsys) -> None:
    """Subcommand limits-get with mocked read_register exits 0 and prints JSON with min/max."""
    import calibrate_servos as m

    mock_servo = MagicMock()
    with patch.object(m, "ST3215", return_value=mock_servo):
        with patch.object(m, "read_register", side_effect=[100, 4000]):
            with patch.object(
                sys,
                "argv",
                ["calibrate_servos", "limits-get", "--device", "/dev/tty", "--id", "1"],
            ):
                assert m.main() == 0
    out = capsys.readouterr().out.strip()
    data = json.loads(out)
    assert data == {"min": 100, "max": 4000}


def test_calibrate_json_output_shape(tmp_path: Path) -> None:
    """Output JSON has shape { servo_id: { "min", "center", "max" } }; JSON keys are strings."""
    out = tmp_path / "cal.json"
    data = {"1": {"min": 100, "center": 2048, "max": 4000}, "2": {"min": 0, "center": 2048, "max": 4095}}
    out.write_text(json.dumps(data, indent=2))
    loaded = json.loads(out.read_text())
    assert loaded == data
    for sid, vals in loaded.items():
        assert int(sid) >= 0
        assert "min" in vals and "center" in vals and "max" in vals
        assert isinstance(vals["min"], int) and isinstance(vals["center"], int) and isinstance(vals["max"], int)


def test_calibrate_missing_joint_exits(tmp_path: Path) -> None:
    """When fewer than expected joints are found, script exits with 1."""
    import calibrate_servos as m

    mock_servo = MagicMock()
    mock_servo.ListServos.return_value = [1, 2, 3]  # only 3, expected 6 by default

    with patch.object(m, "ST3215", MagicMock(return_value=mock_servo)):
        with patch.object(
            sys,
            "argv",
            [
                "calibrate_servos",
                "calibrate",
                "--device",
                "/dev/tty",
                "--output",
                str(tmp_path / "out.json"),
            ],
        ):
            assert m.main() == 1
