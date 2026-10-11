"""Unit tests for feetech_servos register map and read/write helpers."""

from feetech_servos.registers import (
    REGISTER_BLOCKS,
    REGISTER_MAP,
    STS_LOCK,
    STS_MODEL_L,
    WRITABLE_REGISTER_NAMES,
    RegisterEntry,
    get_register_entry_by_name,
    read_all_registers,
    read_register,
    register_blocks,
    runtime_writable_entry,
    write_register,
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


def test_eprom_registers_marked_for_runtime_rejection() -> None:
    """EPROM registers (e.g. PID, current) must not be written from ROS set_register; bridge rejects them."""
    p_coef = get_register_entry_by_name("p_coefficient")
    protection_curr = get_register_entry_by_name("protection_current")
    assert p_coef is not None and p_coef.eprom is True
    assert protection_curr is not None and protection_curr.eprom is True


def test_ram_registers_accepted_at_runtime() -> None:
    """RAM registers (torque_enable, goal_position) are accepted from ROS set_register."""
    torque = get_register_entry_by_name("torque_enable")
    goal = get_register_entry_by_name("goal_position")
    assert torque is not None and torque.eprom is False
    assert goal is not None and goal.eprom is False


def test_read_all_registers_mock_servo() -> None:
    """read_all_registers returns every register from block reads; a failed block leaves its registers out."""

    class MockServo:
        def __init__(self, fail_from: int | None = None) -> None:
            self.fail_from = fail_from
            self.reads: list[tuple[int, int]] = []

        def readTxRx(self, sts_id: int, address: int, length: int):  # noqa: N802 - st3215 API
            self.reads.append((address, length))
            if self.fail_from is not None and address >= self.fail_from:
                return [], -6, 0  # data, comm (COMM_RX_TIMEOUT), err
            return [0] * length, 0, 0

    servo = MockServo()
    out = read_all_registers(servo, 1)
    assert len(out) == len(REGISTER_MAP)  # mock returns 0, success
    assert servo.reads == REGISTER_BLOCKS
    partial = read_all_registers(MockServo(fail_from=STS_LOCK), 1)
    assert "present_position" not in partial and "lock" not in partial
    assert partial["goal_position"] == 0


def test_register_blocks_cover_every_entry_in_two_reads() -> None:
    """Every map entry lies inside one block; the 50-54 gap is not read; the reply fits one status packet."""
    assert REGISTER_BLOCKS == [(STS_MODEL_L, 47), (STS_LOCK, 16)]
    for entry in REGISTER_MAP:
        assert any(start <= entry.address and entry.address + entry.size <= start + n for start, n in REGISTER_BLOCKS)


def test_register_blocks_split_at_gaps_wider_than_max_gap() -> None:
    entries = [RegisterEntry(10, 2, "a", True, False), RegisterEntry(14, 1, "b", True, False)]
    assert register_blocks(entries, max_gap=2) == [(10, 5)]
    assert register_blocks(entries, max_gap=1) == [(10, 2), (14, 1)]
    overlapping = [RegisterEntry(35, 2, "c", True, False), RegisterEntry(36, 1, "d", True, False)]
    assert register_blocks(overlapping) == [(35, 2)]


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


def test_torque_limit_is_two_byte_ram_register_at_48() -> None:
    """torque_limit (STS3215 Torque_Limit, addr 48, 0..1000) is a writable 2-byte RAM register."""
    entry = get_register_entry_by_name("torque_limit")
    assert entry is not None
    assert entry.address == 48
    assert entry.size == 2
    assert entry.read_only is False
    assert entry.eprom is False
    assert "torque_limit" in WRITABLE_REGISTER_NAMES


def test_runtime_writable_entry_accepts_torque_limit_and_rejects_eprom() -> None:
    """runtime_writable_entry returns RAM registers (torque_limit) and None for EPROM / read-only / lock."""
    entry = runtime_writable_entry("torque_limit")
    assert entry is not None and entry.name == "torque_limit"
    assert runtime_writable_entry("max_torque_limit") is None
    assert runtime_writable_entry("present_load") is None
    assert runtime_writable_entry("lock") is None
    assert runtime_writable_entry("nonexistent") is None


def test_torque_limit_runtime_write_is_two_bytes_without_eprom_unlock() -> None:
    """Writing torque_limit writes 2 bytes at address 48 and never unlocks the EPROM."""
    calls: list[tuple] = []

    class MockServo:
        def write2ByteTxRx(self, sts_id: int, address: int, value: int):
            calls.append(("w2", sts_id, address, value))
            return 0, 0

        def UnLockEprom(self, sts_id: int) -> int:  # noqa: N802
            calls.append(("unlock", sts_id))
            return 0

        def LockEprom(self, sts_id: int) -> int:  # noqa: N802
            calls.append(("lock", sts_id))
            return 0

    entry = runtime_writable_entry("torque_limit")
    assert entry is not None
    cache: dict[str, int] = {}
    assert write_register(MockServo(), 6, entry, 250, cache) is True
    assert calls == [("w2", 6, 48, 250)]
    assert cache["torque_limit"] == 250
