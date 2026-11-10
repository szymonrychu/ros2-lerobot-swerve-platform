"""Blocking serial reads for the st3215 bus (serial_wait.py).

The st3215 library opens the port with ``timeout=0`` and polls ``readPort`` in a
loop without sleeping, so every bus round trip spins a CPU core. These tests run
the real library against a pty pair (a fake serial device) and prove that after
``use_blocking_reads`` the wait blocks in the kernel, a missing servo still times
out with ``COMM_RX_TIMEOUT`` within the library's packet timeout, and a reply is
still received promptly.

The wall-clock bounds measure the wait logic, but on a loaded machine (CI, parallel suites) a thread that wakes
from a 1 ms blocking read waits for a CPU, often for several ms, and the fake bus thread can answer late. So:
the reading thread's run-queue wait (Linux ``/proc/thread-self/schedstat``; 0 where it does not exist) is taken off
its wall time, because that is the scheduler's delay and not the wait logic's; a read the fake bus answered after the
library's deadline is a void sample; and each timing test takes up to ``TIMING_ATTEMPTS`` samples on fresh pty pairs.
Every sample must hold the load-independent properties (result code, ``readPort`` call count, the reading thread's
own CPU time) and the timing bound must hold for at least one. A wait that is slow by construction (a busy-wait, a
longer serial timeout) misses the bound in every sample and still fails.
"""

import importlib
import os
import select
import sys
import threading
import time
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
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
PROMPT_REPLY_SLACK_S = 0.010
TIMING_ATTEMPTS = 5
THREAD_SCHEDSTAT = Path("/proc/thread-self/schedstat")
NS_PER_S = 1e9


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
        self.reply_times: list[float] = []
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
                    self.reply_times.append(time.perf_counter())

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


def test_use_blocking_reads_sets_positive_port_timeout(st3215_lib: dict[str, ModuleType], silent_bus: FakeBus) -> None:
    handler, _ = open_handler(st3215_lib, silent_bus)
    assert handler.portHandler.ser.timeout == 0  # the library default: non-blocking reads
    use_blocking_reads(handler)
    assert handler.portHandler.ser.timeout == SERIAL_READ_TIMEOUT_S
    assert 0 < SERIAL_READ_TIMEOUT_S <= 0.002


def runqueue_wait_s() -> float:
    """Time the calling thread has spent runnable but waiting for a CPU.

    :return: seconds (the second field of Linux ``/proc/thread-self/schedstat``), 0.0 where the file does not exist
        (macOS), so the timing bounds then apply to the raw wall time.
    """
    if not THREAD_SCHEDSTAT.exists():
        return 0.0
    return int(THREAD_SCHEDSTAT.read_text().split()[1]) / NS_PER_S


@dataclass(frozen=True)
class ReadSample:
    """One blocking ``read2ByteTxRx`` on a fresh fake bus."""

    result: int
    error: int
    model: int
    read_calls: int
    elapsed_s: float
    runqueue_wait_s: float
    thread_cpu_s: float
    lib_timeout_s: float
    reply_after_s: float | None

    @property
    def wait_s(self) -> float:
        """Wall time of the read without the reading thread's run-queue wait.

        :return: seconds.
        """
        return self.elapsed_s - self.runqueue_wait_s


def sample_read(lib: dict[str, ModuleType], reply_id: int | None, servo_id: int) -> ReadSample:
    """Read MODEL_NUMBER_ADDRESS of ``servo_id`` with blocking reads on a fresh pty pair.

    :param lib: modules from the ``st3215_lib`` fixture.
    :param reply_id: servo ID the fake bus answers, or None for a silent bus.
    :param servo_id: servo ID to read.
    :return: the read's outcome, wall time, the reading thread's run-queue wait and CPU time, the library's packet
        timeout and when (after the request started) the fake bus wrote its reply (None when it did not reply).
    """
    bus = FakeBus(reply_id=reply_id)
    try:
        handler, calls = open_handler(lib, bus)
        use_blocking_reads(handler)
        start = time.perf_counter()
        cpu_start = time.thread_time()
        runqueue_start = runqueue_wait_s()
        model, result, error = handler.read2ByteTxRx(servo_id, MODEL_NUMBER_ADDRESS)
        elapsed = time.perf_counter() - start
        runqueue = runqueue_wait_s() - runqueue_start
        cpu = time.thread_time() - cpu_start
        # The library's packet timeout for a 2-byte read: tx time of (2 + 6) + 3 bytes + LATENCY_TIMER ms.
        lib_timeout_s = (handler.portHandler.tx_time_per_byte * 11 + lib["values"].LATENCY_TIMER) / 1000.0
    finally:
        bus.close()
    reply_after = bus.reply_times[0] - start if bus.reply_times else None
    return ReadSample(result, error, model, calls[0], elapsed, runqueue, cpu, lib_timeout_s, reply_after)


def test_missing_servo_waits_without_spinning_and_still_times_out(st3215_lib: dict[str, ModuleType]) -> None:
    timeout_result = st3215_lib["values"].COMM_RX_TIMEOUT
    samples: list[ReadSample] = []
    for _ in range(TIMING_ATTEMPTS):
        sample = sample_read(st3215_lib, None, MISSING_SERVO_ID)
        samples.append(sample)
        assert sample.result == timeout_result
        assert sample.read_calls <= MAX_READ_CALLS_PER_TIMEOUT
        assert sample.elapsed_s >= sample.lib_timeout_s
        assert sample.thread_cpu_s < sample.elapsed_s / 2  # blocked in the kernel, not burning the wait
        if sample.wait_s <= sample.lib_timeout_s + SERIAL_READ_TIMEOUT_S + TIMEOUT_SLACK_S:
            return
    pytest.fail(f"every missing-servo read overshot the library timeout: {samples}")


def test_present_servo_reply_received_promptly(st3215_lib: dict[str, ModuleType]) -> None:
    success = st3215_lib["values"].COMM_SUCCESS
    samples: list[ReadSample] = []
    for _ in range(TIMING_ATTEMPTS):
        sample = sample_read(st3215_lib, PRESENT_SERVO_ID, PRESENT_SERVO_ID)
        samples.append(sample)
        assert sample.read_calls <= MAX_READ_CALLS_PER_TIMEOUT
        if sample.result != success:
            # Only a reply the fake bus wrote after the library's deadline may be missed.
            assert sample.reply_after_s is None or sample.reply_after_s > sample.lib_timeout_s, sample
            continue
        assert sample.error == 0
        assert sample.model == MODEL_NUMBER
        # Returned within the slack of the fake bus writing the reply (nominally REPLY_DELAY_S after the request).
        if sample.reply_after_s is not None and sample.wait_s < sample.reply_after_s + PROMPT_REPLY_SLACK_S:
            return
    pytest.fail(f"no reply was returned promptly: {samples}")


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
