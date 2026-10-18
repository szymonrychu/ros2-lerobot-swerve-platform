"""Write one register on many servos in one broadcast bus packet (STS sync write, no status replies)."""

from typing import Any

from st3215.group_sync_write import GroupSyncWrite

from .registers import RegisterEntry

COMM_SUCCESS = 0
BYTE_MASK = 0xFF


def register_bytes(value: int, size: int) -> list[int]:
    """Encode a raw register value as the bytes write1ByteTxRx / write2ByteTxRx send (STS: little-endian).

    Args:
        value (int): Raw register value.
        size (int): Register size in bytes (1 or 2).

    Returns:
        list[int]: Data bytes, low byte first.
    """
    return [(value >> (8 * i)) & BYTE_MASK for i in range(size)]


def sync_write_register(servo: Any, entry: RegisterEntry, values: dict[int, int]) -> bool:
    """Write entry on every servo in values with one st3215 GroupSyncWrite packet.

    The packet is a broadcast: the servos send no status reply, so the bus is free right after it is sent and a
    servo that is missing or busy cannot be detected here (the next state read reports it).

    Args:
        servo (Any): ST3215-compatible bus (syncWriteTxOnly).
        entry (RegisterEntry): RAM register to write (same address and size on every servo).
        values (dict[int, int]): servo_id -> raw value.

    Returns:
        bool: True if the packet was sent (also True for an empty values dict: nothing to send).
    """
    if not values:
        return True
    group = GroupSyncWrite(servo, entry.address, entry.size)
    for sts_id, value in values.items():
        group.addParam(sts_id, register_bytes(value, entry.size))
    return group.txPacket() == COMM_SUCCESS


def sync_write_block(servo: Any, start_address: int, blocks: dict[int, list[int]]) -> bool:
    """Write a multi-register byte block on every servo in blocks with one GroupSyncWrite packet.

    Same semantics as sync_write_register (broadcast, no status reply, empty input sends nothing), for several
    adjacent registers at once.

    Args:
        servo (Any): ST3215-compatible bus (syncWriteTxOnly).
        start_address (int): Address of the first register of the block.
        blocks (dict[int, list[int]]): servo_id -> data bytes (same length for every servo), low byte first.

    Returns:
        bool: True if the packet was sent (also True for an empty dict: nothing to send).
    """
    if not blocks:
        return True
    length = len(next(iter(blocks.values())))
    group = GroupSyncWrite(servo, start_address, length)
    for sts_id, data in blocks.items():
        group.addParam(sts_id, data)
    return group.txPacket() == COMM_SUCCESS
