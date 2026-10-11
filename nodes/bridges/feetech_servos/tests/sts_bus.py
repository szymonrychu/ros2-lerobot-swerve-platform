"""Emulated STS servo bus for tests: the real st3215 packet handler talks to in-memory servo register tables.

FakeStsPort stands in for st3215.PortHandler. Every packet the library writes is parsed (header, checksum),
counted as one bus transaction and applied to the emulated servos: READ and WRITE get a status reply,
SYNC_READ gets one status packet per present servo, SYNC_WRITE (broadcast) gets no reply. Tests read the
transaction and byte counters and the decoded packets to check the bus traffic shape and the exact values.
"""

from dataclasses import dataclass, field

from st3215.protocol_packet_handler import protocol_packet_handler
from st3215.values import BROADCAST_ID, INST_READ, INST_SYNC_READ, INST_SYNC_WRITE, INST_WRITE

MEMORY_SIZE = 256
HEADER = (0xFF, 0xFF)
BITS_PER_BYTE = 10  # 8N1 framing: start bit + 8 data bits + stop bit
BAUDRATE = 1_000_000  # STS3215 default
# Per-transaction overhead of a USB serial adapter round trip (latency timer, scheduling), seconds.
TRANSACTION_OVERHEAD_S = 0.001
# st3215 packet timeout spent waiting for a reply that never completes (LATENCY_TIMER = 50 ms), seconds.
PACKET_TIMEOUT_S = 0.05


@dataclass
class StsPacket:
    """One instruction packet written to the bus.

    Attributes:
        servo_id: Target ID (BROADCAST_ID for sync instructions).
        instruction: STS instruction code.
        params: Parameter bytes.
    """

    servo_id: int
    instruction: int
    params: list[int]


@dataclass
class SyncWriteEntry:
    """One servo's slice of a decoded SYNC_WRITE packet.

    Attributes:
        servo_id: Servo the data is for.
        address: Start address.
        data: Data bytes written from address.
    """

    servo_id: int
    address: int
    data: list[int]


def checksum(body: list[int]) -> int:
    """Return the STS checksum of the bytes between the header and the checksum.

    Args:
        body (list[int]): ID, length, instruction/error and parameter bytes.

    Returns:
        int: Inverted low byte of the sum.
    """
    return ~sum(body) & 0xFF


def status_packet(servo_id: int, params: list[int], error: int = 0) -> bytes:
    """Build a status (reply) packet.

    Args:
        servo_id (int): Replying servo.
        params (list[int]): Parameter bytes.
        error (int): Servo error byte.

    Returns:
        bytes: Framed packet.
    """
    body = [servo_id, len(params) + 2, error, *params]
    return bytes([*HEADER, *body, checksum(body)])


@dataclass
class FakeStsPort:
    """In-memory STS bus with the PortHandler interface used by protocol_packet_handler.

    Attributes:
        servos: servo_id -> register table (bytearray of MEMORY_SIZE); missing IDs never answer.
        error_byte: servo_id -> error byte returned in that servo's status packets.
        is_using: PortHandler busy flag (managed by the library).
        transactions: Instruction packets written.
        tx_bytes: Bytes written by the host.
        rx_bytes: Bytes read back by the host.
        packets: Every decoded instruction packet, in order.
        sync_writes: Every decoded SYNC_WRITE slice, in order.
        timeouts: Replies the host gave up on (each costs PACKET_TIMEOUT_S on a real bus).
    """

    servos: dict[int, bytearray]
    error_byte: dict[int, int] = field(default_factory=dict)
    is_using: bool = False
    transactions: int = 0
    tx_bytes: int = 0
    rx_bytes: int = 0
    packets: list[StsPacket] = field(default_factory=list)
    sync_writes: list[SyncWriteEntry] = field(default_factory=list)
    timeouts: int = 0
    rx: bytearray = field(default_factory=bytearray)

    def clearPort(self) -> None:  # noqa: N802 - PortHandler API
        """Drop unread reply bytes."""
        self.rx.clear()

    def setPacketTimeout(self, _packet_length: int) -> None:  # noqa: N802 - PortHandler API
        """No-op: the emulated bus answers instantly or never."""

    def setPacketTimeoutMillis(self, _msec: float) -> None:  # noqa: N802 - PortHandler API
        """No-op: the emulated bus answers instantly or never."""

    def isPacketTimeout(self) -> bool:  # noqa: N802 - PortHandler API
        """Return True and count one timeout: asked only when the reply is incomplete, and no more bytes will come."""
        self.timeouts += 1
        return True

    def readPort(self, length: int) -> bytes:  # noqa: N802 - PortHandler API
        """Return up to length pending reply bytes."""
        data = bytes(self.rx[:length])
        del self.rx[:length]
        self.rx_bytes += len(data)
        return data

    def writePort(self, packet: list[int]) -> int:  # noqa: N802 - PortHandler API
        """Parse one instruction packet, apply it to the servos and queue the replies.

        Args:
            packet (list[int]): Framed instruction packet from the library.

        Returns:
            int: Bytes written.
        """
        raw = list(packet)
        assert tuple(raw[:2]) == HEADER, f"bad header {raw[:2]}"
        length = raw[3]
        assert len(raw) == length + 4, f"length byte {length} does not match packet size {len(raw)}"
        assert raw[-1] == checksum(raw[2:-1]), "bad checksum"
        self.transactions += 1
        self.tx_bytes += len(raw)
        sts_packet = StsPacket(raw[2], raw[4], raw[5:-1])
        self.packets.append(sts_packet)
        self.apply(sts_packet)
        return len(raw)

    def reply(self, servo_id: int, params: list[int]) -> None:
        """Queue a status packet from servo_id when it is present."""
        if servo_id in self.servos:
            self.rx.extend(status_packet(servo_id, params, self.error_byte.get(servo_id, 0)))

    def apply(self, packet: StsPacket) -> None:
        """Execute one instruction on the emulated servos."""
        params = packet.params
        if packet.instruction == INST_READ:
            address, size = params[0], params[1]
            if packet.servo_id in self.servos:
                self.reply(packet.servo_id, list(self.servos[packet.servo_id][address : address + size]))
        elif packet.instruction == INST_WRITE:
            address, data = params[0], params[1:]
            if packet.servo_id in self.servos:
                self.servos[packet.servo_id][address : address + len(data)] = bytes(data)
            if packet.servo_id != BROADCAST_ID:
                self.reply(packet.servo_id, [])
        elif packet.instruction == INST_SYNC_WRITE:
            address, size = params[0], params[1]
            for offset in range(2, len(params), size + 1):
                servo_id, data = params[offset], params[offset + 1 : offset + 1 + size]
                self.sync_writes.append(SyncWriteEntry(servo_id, address, data))
                if servo_id in self.servos:
                    self.servos[servo_id][address : address + size] = bytes(data)
        elif packet.instruction == INST_SYNC_READ:
            address, size = params[0], params[1]
            for servo_id in params[2:]:
                if servo_id in self.servos:
                    self.reply(servo_id, list(self.servos[servo_id][address : address + size]))
        else:
            raise AssertionError(f"unexpected instruction {packet.instruction}")

    def wire_time_s(self) -> float:
        """Estimated bus time so far: bytes at BAUDRATE, a fixed cost per transaction and PACKET_TIMEOUT_S per timeout."""
        return (
            (self.tx_bytes + self.rx_bytes) * BITS_PER_BYTE / BAUDRATE
            + self.transactions * TRANSACTION_OVERHEAD_S
            + self.timeouts * PACKET_TIMEOUT_S
        )

    def reset_counters(self) -> None:
        """Zero the transaction and byte counters and forget recorded packets."""
        self.transactions = 0
        self.tx_bytes = 0
        self.rx_bytes = 0
        self.timeouts = 0
        self.packets.clear()
        self.sync_writes.clear()


def make_bus(servos: dict[int, bytearray]) -> tuple[protocol_packet_handler, FakeStsPort]:
    """Build the real st3215 packet handler on an emulated bus.

    Args:
        servos (dict[int, bytearray]): servo_id -> register table.

    Returns:
        tuple[protocol_packet_handler, FakeStsPort]: (bus handler used like an ST3215 instance, emulated port).
    """
    port = FakeStsPort(servos)
    return protocol_packet_handler(port), port


def register_table(values: dict[int, int] | None = None, fill: int = 0) -> bytearray:
    """Build a servo register table.

    Args:
        values (dict[int, int] | None): address -> byte value overrides.
        fill (int): Value of every other byte.

    Returns:
        bytearray: MEMORY_SIZE bytes.
    """
    table = bytearray([fill] * MEMORY_SIZE)
    for address, value in (values or {}).items():
        table[address] = value
    return table


def put_word(table: bytearray, address: int, value: int) -> None:
    """Store a little-endian 2-byte register value."""
    table[address] = value & 0xFF
    table[address + 1] = (value >> 8) & 0xFF
