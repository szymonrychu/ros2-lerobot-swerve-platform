"""Unit tests for feetech_servos register map and read/write helpers."""

from feetech_servos.registers import (
    REGISTER_MAP,
    WRITABLE_REGISTER_NAMES,
    get_register_entry_by_name,
    read_all_registers,
    read_register,
)


def test_register_map_has_expected_entries() -> None:
    """REGISTER_MAP contains present_position, goal_position, torque_enable, lock."""
    names = {r.name for r in REGISTER_MAP}
    assert "present_position" in names
    assert "goal_position" in names
    assert "torque_enable" in names
    assert "lock" in names
    assert "model" in names
    assert "id" in names


def test_writable_excludes_read_only_and_lock() -> None:
    """WRITABLE_REGISTER_NAMES does not contain present_*, model, or lock."""
    assert "lock" not in WRITABLE_REGISTER_NAMES
    assert "present_position" not in WRITABLE_REGISTER_NAMES
    assert "model" not in WRITABLE_REGISTER_NAMES
    assert "torque_enable" in WRITABLE_REGISTER_NAMES
    assert "goal_position" in WRITABLE_REGISTER_NAMES


def test_get_register_entry_by_name() -> None:
    """get_register_entry_by_name returns correct entry or None."""
    e = get_register_entry_by_name("present_position")
    assert e is not None
    assert e.name == "present_position"
    assert e.size == 2
    assert e.read_only is True
    e2 = get_register_entry_by_name("nonexistent")
    assert e2 is None


def test_read_all_registers_mock_servo() -> None:
    """read_all_registers returns dict; mock returns empty on read error."""

    class MockServo:
        def read1ByteTxRx(self, sts_id: int, address: int):  # noqa: A002
            return 0, 0, 0  # value, comm, err

        def read2ByteTxRx(self, sts_id: int, address: int):
            return 0, 0, 0

    servo = MockServo()
    out = read_all_registers(servo, 1)
    assert isinstance(out, dict)
    # All registers should be read (mock returns 0, success)
    assert len(out) == len(REGISTER_MAP)


def test_read_register_one_byte() -> None:
    """read_register with size 1 returns single byte value."""

    class MockServo:
        def read1ByteTxRx(self, sts_id: int, address: int):  # noqa: A002
            return 42, 0, 0

        def read2ByteTxRx(self, sts_id: int, address: int):
            return 0, 0, 0

    entry = get_register_entry_by_name("torque_enable")
    assert entry is not None
    assert read_register(MockServo(), 1, entry) == 42


def test_read_register_comm_error_returns_none() -> None:
    """read_register returns None when comm or error non-zero."""

    class MockServo:
        def read1ByteTxRx(self, sts_id: int, address: int):  # noqa: A002
            return 0, -1, 0  # comm fail

        def read2ByteTxRx(self, sts_id: int, address: int):
            return 0, 0, 1  # error

    entry1 = get_register_entry_by_name("torque_enable")
    entry2 = get_register_entry_by_name("present_position")
    assert entry1 is not None and entry2 is not None
    assert read_register(MockServo(), 1, entry1) is None
    assert read_register(MockServo(), 1, entry2) is None
