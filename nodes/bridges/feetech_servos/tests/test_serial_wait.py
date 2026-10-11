"""Blocking serial reads for the st3215 bus (serial_wait.py).

The st3215 library opens the port with ``timeout=0`` and polls ``readPort`` in a
loop without sleeping, so every bus round trip spins a CPU core. These tests run
the real library against a pty pair (a fake serial device) and prove that after
``use_blocking_reads`` the wait blocks in the kernel, a missing servo still times
out with ``COMM_RX_TIMEOUT`` within the library's packet timeout, and a reply is
still received promptly.
"""

import importlib
import os
import select
import sys
import threading
import time
from collections.abc import Iterator
from types import ModuleType
from typing import Any
from unittest.mock import MagicMock

import pytest

from feetech_servos.serial_wait import SERIAL_READ_TIMEOUT_S, use_blocking_reads

from .ros_stubs import FakeRclpy
from .test_shutdown import make_config

TEST_BAUDRATE = 115200  # macOS ptys reject the 1 Mbaud custom rate; the wait logic is baud-independent
MISSING_SERVO_ID = 5
PRESENT_SERVO_ID = 7
MODEL_NUMBER = 777
MODEL_NUMBER_ADDRESS = 3
INST_READ = 0x02
# A busy-wait performs tens of thousands of reads per packet timeout (~50 ms); a blocking
# read with a ~1 ms timeout performs about one per millisecond.
MAX_READ_CALLS_PER_TIMEOUT = 200
TIMEOUT_SLACK_S = 0.005
REPLY_DELAY_S = 0.003
POLL_S = 0.01


@pytest.fixture
def st3215_lib() -> Iterator[dict[str, ModuleType]]:
    """Import the real st3215 package, bypassing the conftest MagicMock.

    :return: dict with the real ``port_handler``, ``protocol_packet_handler`` and ``values`` modules.
    """
    saved = {name: mod for name, mod in sys.modules.items() if name == "st3215" or name.startswith("st3215.")}
    for name in saved:
        del sys.modules[name]
    try:
        yield {
            "port_handler": importlib.import_module("st3215.port_handler"),
            "protocol_packet_handler": importlib.import_module("st3215.protocol_packet_handler"),
            "values": importlib.import_module("st3215.values"),
        }
    finally:
        for name in [n for n in sys.modules if n == "st3215" or n.startswith("st3215.")]:
            del sys.modules[name]
        sys.modules.update(saved)


class FakeBus:
    """pty pair standing in for the servo bus; the master side optionally answers register reads."""

    def __init__(self, reply_id: int | None) -> None:
        """Open the pty pair and start the master-side reader thread.

        :param reply_id: servo ID that answers read packets, or None for a silent bus.
        """
        self.master_fd, self.slave_fd = os.openpty()
        self.device = os.ttyname(self.slave_fd)
        self.reply_id = reply_id
        self.ports: list[Any] = []
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def serve(self) -> None:
        """Read host packets; answer a read (instruction 0x02) to ``reply_id`` with MODEL_NUMBER bytes."""
        buf = b""
        while not self.stopped.is_set():
            readable, _, _ = select.select([self.master_fd], [], [], POLL_S)
            if not readable:
                continue
            buf += os.read(self.master_fd, 1024)
            while len(buf) >= 6 and buf[:2] == b"\xff\xff" and len(buf) >= buf[3] + 4:
                packet, buf = buf[: buf[3] + 4], buf[buf[3] + 4 :]
                if self.reply_id is not None and packet[2] == self.reply_id and packet[4] == INST_READ:
                    params = list(MODEL_NUMBER.to_bytes(2, "little"))[: packet[6]]
                    time.sleep(REPLY_DELAY_S)
                    os.write(self.master_fd, status_packet(self.reply_id, params))

    def close(self) -> None:
        """Stop the reader thread, close the serial ports opened on the bus, then both pty ends."""
        self.stopped.set()
        self.thread.join()
        for port in self.ports:
            port.closePort()
        os.close(self.slave_fd)
        os.close(self.master_fd)


def status_packet(servo_id: int, params: list[int]) -> bytes:
    """Build an STS status packet.

    :param servo_id: replying servo ID.
    :param params: parameter bytes.
    :return: framed packet bytes with checksum.
    """
    body = [servo_id, len(params) + 2, 0, *params]
    return bytes([0xFF, 0xFF, *body, ~sum(body) & 0xFF])


def open_handler(lib: dict[str, ModuleType], bus: FakeBus) -> tuple[Any, list[int]]:
    """Open the real st3215 packet handler on the fake bus and count ``readPort`` calls.

    :param lib: modules from the ``st3215_lib`` fixture.
    :param bus: fake bus; the opened port is closed by ``bus.close()``.
    :return: (packet handler with ``portHandler`` attribute, one-element read-call counter).
    """
    port = lib["port_handler"].PortHandler(bus.device)
    port.baudrate = TEST_BAUDRATE
    assert port.openPort()
    bus.ports.append(port)
    handler = lib["protocol_packet_handler"].protocol_packet_handler(port)
    handler.portHandler = port
    calls = [0]
    original = port.readPort

    def counting_read(length: int) -> bytes:
        calls[0] += 1
        return original(length)

    port.readPort = counting_read
    return handler, calls


@pytest.fixture
def silent_bus() -> Iterator[FakeBus]:
    """Fake bus where no servo answers."""
    bus = FakeBus(reply_id=None)
    yield bus
    bus.close()


@pytest.fixture
def answering_bus() -> Iterator[FakeBus]:
    """Fake bus where PRESENT_SERVO_ID answers register reads."""
    bus = FakeBus(reply_id=PRESENT_SERVO_ID)
    yield bus
    bus.close()


def test_use_blocking_reads_sets_positive_port_timeout(st3215_lib: dict[str, ModuleType], silent_bus: FakeBus) -> None:
    handler, _ = open_handler(st3215_lib, silent_bus)
    assert handler.portHandler.ser.timeout == 0  # the library default: non-blocking reads
    use_blocking_reads(handler)
    assert handler.portHandler.ser.timeout == SERIAL_READ_TIMEOUT_S
    assert 0 < SERIAL_READ_TIMEOUT_S <= 0.002


def test_missing_servo_waits_without_spinning_and_still_times_out(
    st3215_lib: dict[str, ModuleType], silent_bus: FakeBus
) -> None:
    handler, calls = open_handler(st3215_lib, silent_bus)
    use_blocking_reads(handler)
    start = time.perf_counter()
    cpu_start = time.process_time()
    _, result, _ = handler.read2ByteTxRx(MISSING_SERVO_ID, MODEL_NUMBER_ADDRESS)
    elapsed = time.perf_counter() - start
    cpu = time.process_time() - cpu_start
    assert result == st3215_lib["values"].COMM_RX_TIMEOUT
    assert calls[0] <= MAX_READ_CALLS_PER_TIMEOUT
    # The library's packet timeout for a 2-byte read: tx time of (2 + 6) + 3 bytes + LATENCY_TIMER ms.
    lib_timeout_s = (handler.portHandler.tx_time_per_byte * 11 + st3215_lib["values"].LATENCY_TIMER) / 1000.0
    assert lib_timeout_s <= elapsed <= lib_timeout_s + SERIAL_READ_TIMEOUT_S + TIMEOUT_SLACK_S
    assert cpu < elapsed / 2  # blocked in the kernel, not burning the wait


def test_present_servo_reply_received_promptly(st3215_lib: dict[str, ModuleType], answering_bus: FakeBus) -> None:
    handler, calls = open_handler(st3215_lib, answering_bus)
    use_blocking_reads(handler)
    start = time.perf_counter()
    model, result, error = handler.read2ByteTxRx(PRESENT_SERVO_ID, MODEL_NUMBER_ADDRESS)
    elapsed = time.perf_counter() - start
    assert result == st3215_lib["values"].COMM_SUCCESS
    assert error == 0
    assert model == MODEL_NUMBER
    assert calls[0] <= MAX_READ_CALLS_PER_TIMEOUT
    assert elapsed < REPLY_DELAY_S + 0.010


def test_use_blocking_reads_without_port_is_noop() -> None:
    class NoPort:
        portHandler = None

    use_blocking_reads(NoPort())  # must not raise


def test_run_bridge_applies_blocking_reads_to_the_opened_bus(monkeypatch: pytest.MonkeyPatch) -> None:
    module = importlib.import_module("feetech_servos.bridge")
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "Node", MagicMock())
    monkeypatch.setattr(module, "SingleThreadedExecutor", MagicMock())
    fake = FakeRclpy()
    fake.alive = False  # exit right after the bus is opened (while waiting for servos)
    monkeypatch.setattr(module, "rclpy", fake)
    blocking = MagicMock()
    monkeypatch.setattr(module, "use_blocking_reads", blocking)
    servo_class = MagicMock()
    monkeypatch.setattr(sys.modules["st3215"], "ST3215", servo_class)

    module.run_bridge(make_config(module, "/dev/ttyUSB0"))

    servo_class.assert_called_once_with("/dev/ttyUSB0")
    blocking.assert_called_once_with(servo_class.return_value)
