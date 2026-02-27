"""Unit tests for set_servo_id script (argparse and exactly-one logic with mocks)."""

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

# Script is in scripts/; add it so we can import set_servo_id
scripts_dir = Path(__file__).resolve().parent.parent / "scripts"
if str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))


def test_set_servo_id_invalid_new_id() -> None:
    """Returns 1 when new-id is out of range."""
    import set_servo_id as m

    with patch.object(sys, "argv", ["set_servo_id", "--device", "/dev/tty", "--new-id", "254"]):
        assert m.main() == 1
    with patch.object(sys, "argv", ["set_servo_id", "--device", "/dev/tty", "--new-id", "-1"]):
        assert m.main() == 1


def test_set_servo_id_exactly_one_servo_success() -> None:
    """Returns 0 when exactly one servo is found and ID is changed."""
    import set_servo_id as m

    mock_servo = MagicMock()
    mock_servo.ListServos.return_value = [1]
    mock_servo.ChangeId.return_value = None  # success

    with patch("st3215.ST3215", MagicMock(return_value=mock_servo)):
        with patch.object(sys, "argv", ["set_servo_id", "--device", "/dev/tty", "--new-id", "2"]):
            assert m.main() == 0
    mock_servo.ChangeId.assert_called_once_with(1, 2)


def test_set_servo_id_zero_servos_fails() -> None:
    """Returns 1 when no servos are found."""
    import set_servo_id as m

    mock_servo = MagicMock()
    mock_servo.ListServos.return_value = []

    with patch("st3215.ST3215", MagicMock(return_value=mock_servo)):
        with patch.object(sys, "argv", ["set_servo_id", "--device", "/dev/tty", "--new-id", "1"]):
            assert m.main() == 1


def test_set_servo_id_two_servos_fails() -> None:
    """Returns 1 when more than one servo is found."""
    import set_servo_id as m

    mock_servo = MagicMock()
    mock_servo.ListServos.return_value = [1, 2]

    with patch("st3215.ST3215", MagicMock(return_value=mock_servo)):
        with patch.object(sys, "argv", ["set_servo_id", "--device", "/dev/tty", "--new-id", "3"]):
            assert m.main() == 1
