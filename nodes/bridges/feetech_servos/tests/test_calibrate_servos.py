"""Unit tests for calibrate_servos script (JSON shape, missing joint exit with mocks)."""

import json
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

scripts_dir = Path(__file__).resolve().parent.parent / "scripts"
if str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))


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
                "--device",
                "/dev/tty",
                "--output",
                str(tmp_path / "out.json"),
            ],
        ):
            assert m.main() == 1
