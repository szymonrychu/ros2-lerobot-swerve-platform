"""Emulated STS bus whose servos read present = (encoder - offset) mod 4096, shared by the offset test suites."""

from dataclasses import dataclass, field

from st3215.values import INST_READ, INST_WRITE

from feetech_servos.registers import (
    STS_MAX_ANGLE_LIMIT_L,
    STS_MIN_ANGLE_LIMIT_L,
    STS_OFS_L,
    STS_PRESENT_POSITION_L,
    STS_TORQUE_ENABLE,
)
from feetech_servos.servo_offsets import decode_offset

from .sts_bus import StsPacket, put_word
from .test_neutral import NeutralPort, word

LIMITS = {1: (500, 3500), 2: (0, 4095), 3: (0, 4095), 4: (0, 4095), 5: (0, 4095), 6: (1900, 3377)}


@dataclass
class OffsetPort(NeutralPort):
    """NeutralPort whose servos read present_position = (encoder - offset) mod 4096, like the STS3215.

    Attributes:
        encoder: servo_id -> raw encoder steps of the resting arm (the arm never moves).
        offset_sign: +1 for the STS3215 semantics; -1 models a servo adding the offset (wrong convention).
        stuck_offset: Servos acknowledging offset writes without storing them.
        limit_shift_on_offset: Steps the angle limit registers move by on an offset write.
        jitter: servo_id -> per-read present_position deltas (popped per read; the arm trembling).
        torque_on_after_offset: servo_id -> servo whose torque turns on after that servo's offset write.
        unacked_offset: Servos storing an offset write without sending the status reply.
        drop_offset_reads_after_writes: servo_id -> offset writes after which its offset reads never answer.
        offset_writes_seen: servo_id -> offset writes received so far.
    """

    encoder: dict[int, int] = field(default_factory=dict)
    offset_sign: int = 1
    stuck_offset: set[int] = field(default_factory=set)
    limit_shift_on_offset: int = 0
    jitter: dict[int, list[int]] = field(default_factory=dict)
    torque_on_after_offset: dict[int, int] = field(default_factory=dict)
    unacked_offset: set[int] = field(default_factory=set)
    drop_offset_reads_after_writes: dict[int, int] = field(default_factory=dict)
    offset_writes_seen: dict[int, int] = field(default_factory=dict)

    def refresh_present(self, servo_id: int) -> None:
        """Recompute present_position of one servo from its encoder and offset register."""
        table = self.servos[servo_id]
        offset = decode_offset(word(table, STS_OFS_L))
        put_word(table, STS_PRESENT_POSITION_L, (self.encoder[servo_id] - self.offset_sign * offset) % 4096)

    def apply(self, packet: StsPacket) -> None:
        """Apply the instruction, then model the offset register's effect and the configured faults."""
        sid, params = packet.servo_id, packet.params
        if packet.instruction == INST_READ and params[0] == STS_PRESENT_POSITION_L and self.jitter.get(sid):
            table = self.servos[sid]
            true_value = word(table, STS_PRESENT_POSITION_L)
            put_word(table, STS_PRESENT_POSITION_L, true_value + self.jitter[sid].pop(0))
            super().apply(packet)
            put_word(table, STS_PRESENT_POSITION_L, true_value)
            return
        is_offset_write = packet.instruction == INST_WRITE and params and params[0] == STS_OFS_L
        is_offset_read = packet.instruction == INST_READ and params[0] == STS_OFS_L
        if is_offset_read and self.offset_writes_seen.get(sid, 0) >= self.drop_offset_reads_after_writes.get(
            sid, 10**9
        ):
            return
        if is_offset_write:
            self.offset_writes_seen[sid] = self.offset_writes_seen.get(sid, 0) + 1
        if is_offset_write and sid in self.unacked_offset:
            self.servos[sid][STS_OFS_L : STS_OFS_L + 2] = bytes(params[1:3])
            self.events.append((sid, "offset", decode_offset(word(self.servos[sid], STS_OFS_L))))
            self.refresh_present(sid)
            return
        if is_offset_write and sid in self.stuck_offset:
            self.events.append((sid, "offset", word(bytearray(params[1:3]), 0)))
            self.reply(sid, [])
            return
        super().apply(packet)
        if is_offset_write and sid in self.servos:
            self.events.append((sid, "offset", decode_offset(word(self.servos[sid], STS_OFS_L))))
            self.refresh_present(sid)
            if self.limit_shift_on_offset:
                table = self.servos[sid]
                put_word(table, STS_MIN_ANGLE_LIMIT_L, LIMITS[sid][0] + self.limit_shift_on_offset)
                put_word(table, STS_MAX_ANGLE_LIMIT_L, LIMITS[sid][1] - self.limit_shift_on_offset)
            if sid in self.torque_on_after_offset:
                self.servos[self.torque_on_after_offset[sid]][STS_TORQUE_ENABLE] = 1


@dataclass
class Clock:
    """Injected monotonic clock."""

    now: float = 1000.0

    def __call__(self) -> float:
        return self.now
