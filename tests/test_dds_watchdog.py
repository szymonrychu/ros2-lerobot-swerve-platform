"""dds_watchdog detection core: wedged Fast DDS discovery threads found from /proc and cgroup v2 (fixture trees)."""

import importlib.util
import json
import logging
import os
import socket
import stat
import subprocess
import sys
import threading
import time
from collections.abc import Callable, Iterator
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import ModuleType

import pytest

SCRIPT = Path(__file__).resolve().parent.parent / "ansible/roles/monitoring_agent/files/dds_watchdog.py"
UNIT = "ros2-test.service"
RX_MIN = 65536
UDP_HEADER = (
    "  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode ref pointer drops"
)


def load_script() -> ModuleType:
    """Import files/dds_watchdog.py as a module.

    Returns:
        ModuleType: The script module.
    """
    spec = importlib.util.spec_from_file_location("dds_watchdog", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def udp_row(slot: int, port: int, rx: int, addr: str = "00000000") -> str:
    """Build one /proc/net/udp data row.

    Args:
        slot (int): Row number.
        port (int): Local port.
        rx (int): rx_queue bytes.
        addr (str): Local address in hex (8 digits for udp, 32 for udp6).

    Returns:
        str: The row.
    """
    zero = "0" * len(addr)
    return (
        f"  {slot}: {addr}:{port:04X} {zero}:0000 07 00000000:{rx:08X} 00:00000000 00000000  1000        0 "
        f"{123456 + slot} 2 0000000000000000 0"
    )


def write_net(proc: Path, name: str, rows: list[str]) -> None:
    """Write /proc/net/<name> with a header and rows.

    Args:
        proc (Path): Fake proc root.
        name (str): "udp" or "udp6".
        rows (list[str]): Data rows.
    """
    (proc / "net").mkdir(parents=True, exist_ok=True)
    (proc / "net" / name).write_text("\n".join([UDP_HEADER, *rows]) + "\n")


def add_thread(proc: Path, pid: int, tid: int, comm: str, wchan: str) -> None:
    """Create /proc/<pid>/task/<tid>/{comm,wchan} (comm has a newline, wchan none, as in the kernel).

    Args:
        proc (Path): Fake proc root.
        pid (int): Process id.
        tid (int): Thread id.
        comm (str): Thread name.
        wchan (str): Wait channel.
    """
    task = proc / str(pid) / "task" / str(tid)
    task.mkdir(parents=True)
    (task / "comm").write_text(comm + "\n")
    (task / "wchan").write_text(wchan)


def add_procs(cgroup: Path, unit: str, pids: list[int], child: str | None = None) -> None:
    """Write cgroup.procs of the unit, or of one child cgroup of it.

    Args:
        cgroup (Path): Fake /sys/fs/cgroup.
        unit (str): Unit name.
        pids (list[int]): Pids to list.
        child (str | None): Child cgroup directory name, or None for the unit itself.
    """
    path = cgroup / "system.slice" / unit
    if child:
        path = path / child
    path.mkdir(parents=True, exist_ok=True)
    (path / "cgroup.procs").write_text("".join(f"{pid}\n" for pid in pids))


@pytest.fixture
def wd() -> ModuleType:
    """The imported script module.

    Returns:
        ModuleType: dds_watchdog.
    """
    return load_script()


@pytest.fixture
def fake_root(tmp_path: Path) -> Callable[..., tuple[Path, Path]]:
    """Factory building a fake proc + cgroup tree for one unit with pid 100.

    Args:
        tmp_path (Path): pytest temp dir.

    Returns:
        Callable[..., tuple[Path, Path]]: build(threads, rows) -> (proc_root, cgroup_root); threads is a list of
            (comm, wchan), rows the udp rows.
    """

    def build(threads: list[tuple[str, str]], rows: list[str]) -> tuple[Path, Path]:
        proc = tmp_path / "proc"
        cgroup = tmp_path / "sys/fs/cgroup"
        for i, (comm, wchan) in enumerate(threads):
            add_thread(proc, 100, 100 + i, comm, wchan)
        write_net(proc, "udp", rows)
        add_procs(cgroup, UNIT, [100])
        return proc, cgroup

    return build


HEALTHY = ("dds.udp.7418", "__skb_wait_for_more_packets")
WEDGED = ("dds.udp.7458", "futex_wait_queue")
UNICAST = ("dds.udp.7459", "futex_wait_queue")


def test_metatraffic_port_accepts_even_offsets(wd: ModuleType) -> None:
    """Even offsets in 7410..7608 give the port; odd offsets, out of range and other threads give None."""
    assert wd.metatraffic_port("dds.udp.7410") == 7410
    assert wd.metatraffic_port("dds.udp.7458") == 7458
    assert wd.metatraffic_port("dds.udp.7608") == 7608
    for comm in ["dds.udp.7459", "dds.udp.7610", "dds.udp.7408", "dds.udp.7024", "dds.ev.0", "dds.shm.7461", ""]:
        assert wd.metatraffic_port(comm) is None


def test_rx_queues_parses_hex_rx_field(wd: ModuleType, tmp_path: Path) -> None:
    """The tx:rx field is hex: 000F4880 is 1001600."""
    write_net(tmp_path, "udp", [udp_row(1, 7458, 0x000F4880), udp_row(2, 7418, 0)])
    assert wd.rx_queues(tmp_path) == {7458: 1001600, 7418: 0}


def test_rx_queues_reads_udp6_and_takes_max(wd: ModuleType, tmp_path: Path) -> None:
    """udp6 rows (32-digit addresses) are read too and the larger queue wins per port."""
    write_net(tmp_path, "udp", [udp_row(1, 7458, 100)])
    write_net(tmp_path, "udp6", [udp_row(1, 7458, 5000, addr="0" * 32), udp_row(2, 7418, 7, addr="0" * 32)])
    assert wd.rx_queues(tmp_path) == {7458: 5000, 7418: 7}


def test_rx_queues_missing_files_are_skipped(wd: ModuleType, tmp_path: Path) -> None:
    """No /proc/net/udp at all gives an empty dict."""
    assert wd.rx_queues(tmp_path) == {}


def test_unit_pids_includes_child_cgroups(wd: ModuleType, tmp_path: Path) -> None:
    """Pids of the unit and of nested child cgroups are all returned."""
    add_procs(tmp_path, UNIT, [100, 101])
    add_procs(tmp_path, UNIT, [200], child="sub")
    nested = tmp_path / "system.slice" / UNIT / "sub" / "deeper"
    nested.mkdir()
    (nested / "cgroup.procs").write_text("300\n")
    assert sorted(wd.unit_pids(tmp_path, UNIT)) == [100, 101, 200, 300]


def test_unit_pids_missing_unit_is_empty(wd: ModuleType, tmp_path: Path) -> None:
    """A unit without a cgroup directory has no pids."""
    assert wd.unit_pids(tmp_path, UNIT) == []


def test_scan_unit_healthy_is_none(wd: ModuleType, fake_root: Callable[..., tuple[Path, Path]]) -> None:
    """A thread waiting for packets with an empty queue is not a suspect."""
    proc, cgroup = fake_root([HEALTHY], [udp_row(1, 7418, 0)])
    assert wd.scan_unit(proc, cgroup, UNIT, wd.rx_queues(proc), RX_MIN) is None


def test_scan_unit_futex_with_empty_queue_is_none(wd: ModuleType, fake_root: Callable[..., tuple[Path, Path]]) -> None:
    """futex_wait_queue alone (queue below rx_min) is not a suspect."""
    proc, cgroup = fake_root([WEDGED], [udp_row(1, 7458, 0)])
    assert wd.scan_unit(proc, cgroup, UNIT, wd.rx_queues(proc), RX_MIN) is None


def test_scan_unit_full_queue_in_skb_wait_is_none(wd: ModuleType, fake_root: Callable[..., tuple[Path, Path]]) -> None:
    """A full queue while the thread still waits in the socket receive path is just load, not a wedge."""
    proc, cgroup = fake_root([("dds.udp.7458", "__skb_wait_for_more_packets")], [udp_row(1, 7458, 1001600)])
    assert wd.scan_unit(proc, cgroup, UNIT, wd.rx_queues(proc), RX_MIN) is None


def test_scan_unit_futex_and_full_queue_is_suspect(wd: ModuleType, fake_root: Callable[..., tuple[Path, Path]]) -> None:
    """futex_wait_queue plus rx_queue >= rx_min yields the evidence."""
    proc, cgroup = fake_root([HEALTHY, WEDGED], [udp_row(1, 7418, 0), udp_row(2, 7458, 0x000F4880)])
    evidence = wd.scan_unit(proc, cgroup, UNIT, wd.rx_queues(proc), RX_MIN)
    assert evidence == wd.WedgeEvidence(
        unit=UNIT, pid=100, tid=101, port=7458, rx_queue=1001600, wchan="futex_wait_queue"
    )


def test_scan_unit_ignores_user_unicast_port(wd: ModuleType, fake_root: Callable[..., tuple[Path, Path]]) -> None:
    """A user-unicast thread (odd offset) is ignored even with futex wait and a full queue."""
    proc, cgroup = fake_root([UNICAST], [udp_row(1, 7459, 1001600)])
    assert wd.scan_unit(proc, cgroup, UNIT, wd.rx_queues(proc), RX_MIN) is None


def test_scan_unit_tolerates_vanished_pid(wd: ModuleType, fake_root: Callable[..., tuple[Path, Path]]) -> None:
    """A pid in cgroup.procs without a /proc entry (exited meanwhile) is skipped, the rest is still scanned."""
    proc, cgroup = fake_root([WEDGED], [udp_row(1, 7458, 1001600)])
    add_procs(cgroup, UNIT, [999, 100])
    evidence = wd.scan_unit(proc, cgroup, UNIT, wd.rx_queues(proc), RX_MIN)
    assert evidence is not None and evidence.port == 7458


def test_scan_unit_tolerates_vanished_thread_files(wd: ModuleType, fake_root: Callable[..., tuple[Path, Path]]) -> None:
    """A thread dir without comm/wchan files (exited mid-scan) is skipped."""
    proc, cgroup = fake_root([WEDGED], [udp_row(1, 7458, 1001600)])
    (proc / "100" / "task" / "50").mkdir()
    assert wd.scan_unit(proc, cgroup, UNIT, wd.rx_queues(proc), RX_MIN) is not None


# ---- calibration guard: GET /api/calibration/active served by a stub http.server ----

CALIB_IDLE: dict[str, object] = {"active": False, "flows": [], "unknown": []}
# Copy of the consumer fixture nodes/web_ui/tests/fixtures/api/calibration_active.json.
CALIB_FIXTURE: dict[str, object] = {
    "active": True,
    "flows": [
        {"name": "rgbd:head-arm", "reason": "sequence.phase=collect"},
        {"name": "general:home_arm", "reason": "1 request(s) in flight"},
    ],
    "unknown": [{"name": "servo:arm", "reason": "torque_state stale"}],
}
CALIB_UNKNOWN_ONLY: dict[str, object] = {
    "active": False,
    "flows": [],
    "unknown": [{"name": "servo:arm", "reason": "torque_state stale"}],
}
CALIB_TIMEOUT_S = 0.3
CALIB_PATH = "/api/calibration/active"


class CalibrationStub:
    """A threaded HTTP server on 127.0.0.1 (free port) that answers every GET with a configurable reply.

    Attributes:
        body (bytes): Response body.
        status (int): Response status.
        delay_s (float): Seconds to sleep before answering.
        drop_first (int): The first N requests are answered by closing the connection without a response.
        location (str | None): Location header to send (for redirects).
        later_body (bytes | None): When set, the body of every request after the first (body answers the first).
        requests (int): Requests received so far.
        url (str): Full URL of the calibration endpoint on the stub.
    """

    def __init__(self) -> None:
        """Start the server in a daemon thread, answering idle."""
        self.body = json.dumps(CALIB_IDLE).encode()
        self.status = 200
        self.delay_s = 0.0
        self.drop_first = 0
        self.location: str | None = None
        self.later_body: bytes | None = None
        self.requests = 0
        self.lock = threading.Lock()
        stub = self

        class Handler(BaseHTTPRequestHandler):
            """Serve the stub's configured reply."""

            def do_GET(self) -> None:  # noqa: N802 - http.server naming
                """Answer one GET."""
                with stub.lock:
                    stub.requests += 1
                    count = stub.requests
                if count <= stub.drop_first:
                    self.close_connection = True
                    return
                if stub.delay_s:
                    time.sleep(stub.delay_s)
                body = stub.later_body if count > 1 and stub.later_body is not None else stub.body
                self.send_response(stub.status)
                if stub.location:
                    self.send_header("Location", stub.location)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                try:
                    self.wfile.write(body)
                except OSError:
                    pass

            def log_message(self, format: str, *args: object) -> None:  # noqa: A002 - base class signature
                """Keep the test output quiet."""

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.server.daemon_threads = True
        self.server.block_on_close = False
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}{CALIB_PATH}"
        self.thread = threading.Thread(target=self.server.serve_forever, args=(0.05,), daemon=True)
        self.thread.start()

    def set_json(self, payload: object) -> None:
        """Answer with this JSON value from now on.

        Args:
            payload (object): JSON-serializable value.
        """
        self.body = json.dumps(payload).encode()

    def close(self) -> None:
        """Stop the server."""
        self.server.shutdown()
        self.server.server_close()


@pytest.fixture
def stub() -> Iterator[CalibrationStub]:
    """A running calibration stub answering idle.

    Yields:
        CalibrationStub: The stub; stopped after the test.
    """
    server = CalibrationStub()
    yield server
    server.close()


def free_port() -> int:
    """Find a local TCP port nobody listens on.

    Returns:
        int: The port (bound once, then released).
    """
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def test_calibration_idle_allows_restart(wd: ModuleType, stub: CalibrationStub) -> None:
    """An idle answer is not blocked, reason "idle", one request."""
    assert wd.calibration_status(stub.url, CALIB_TIMEOUT_S) == (False, "idle")
    assert stub.requests == 1


def test_calibration_active_blocks(wd: ModuleType, stub: CalibrationStub) -> None:
    """The consumer fixture (two flows, one unknown) blocks with the joined flow names."""
    stub.set_json(CALIB_FIXTURE)
    assert wd.calibration_status(stub.url, CALIB_TIMEOUT_S) == (True, "active: rgbd:head-arm, general:home_arm")


def test_calibration_unknown_only_blocks(wd: ModuleType, stub: CalibrationStub) -> None:
    """active false with one unknown source still blocks: an unreadable source is never idle."""
    stub.set_json(CALIB_UNKNOWN_ONLY)
    assert wd.calibration_status(stub.url, CALIB_TIMEOUT_S) == (True, "unknown: servo:arm (torque_state stale)")


def test_calibration_http_500_blocks(wd: ModuleType, stub: CalibrationStub) -> None:
    """A non-200 status blocks (even with an idle body) and is not retried."""
    stub.status = 500
    blocked, reason = wd.calibration_status(stub.url, CALIB_TIMEOUT_S)
    assert blocked
    assert reason.startswith("unreachable: HTTP 500"), reason
    assert stub.requests == 1


def test_calibration_non_200_success_blocks(wd: ModuleType, stub: CalibrationStub) -> None:
    """A 2xx other than 200 (here 203 with an idle body) is not the documented answer and blocks."""
    stub.status = 203
    blocked, reason = wd.calibration_status(stub.url, CALIB_TIMEOUT_S)
    assert blocked
    assert reason.startswith("unreachable: HTTP 203"), reason


def test_calibration_redirect_blocks(wd: ModuleType, stub: CalibrationStub) -> None:
    """A 3xx is never followed (it could point at another host): blocked as malformed, one request."""
    stub.status = 302
    stub.location = "http://example.invalid/api/calibration/active"
    blocked, reason = wd.calibration_status(stub.url, CALIB_TIMEOUT_S)
    assert blocked
    assert reason.startswith("malformed: redirect 302"), reason
    assert stub.requests == 1


def test_calibration_timeout_blocks(wd: ModuleType, stub: CalibrationStub) -> None:
    """A handler sleeping past the timeout blocks as unreachable; the run waits at most about two timeouts."""
    stub.delay_s = 2.0
    start = time.monotonic()
    blocked, reason = wd.calibration_status(stub.url, CALIB_TIMEOUT_S)
    assert blocked
    assert reason.startswith("unreachable: "), reason
    assert time.monotonic() - start < 1.5


def test_calibration_connection_refused_blocks(wd: ModuleType) -> None:
    """Nobody listening on the port blocks as unreachable."""
    blocked, reason = wd.calibration_status(f"http://127.0.0.1:{free_port()}{CALIB_PATH}", CALIB_TIMEOUT_S)
    assert blocked
    assert reason.startswith("unreachable: "), reason


def test_calibration_invalid_json_blocks(wd: ModuleType, stub: CalibrationStub) -> None:
    """A 200 with a body that is not JSON blocks as malformed and is not retried."""
    stub.body = b"<html>not json</html>"
    blocked, reason = wd.calibration_status(stub.url, CALIB_TIMEOUT_S)
    assert blocked
    assert reason.startswith("malformed: "), reason
    assert stub.requests == 1


def test_calibration_deeply_nested_json_blocks(wd: ModuleType, stub: CalibrationStub) -> None:
    """JSON nested deeper than the parser's recursion limit blocks as malformed instead of raising."""
    stub.body = b"[" * 200000 + b"]" * 200000
    blocked, reason = wd.calibration_status(stub.url, CALIB_TIMEOUT_S)
    assert blocked
    assert reason.startswith("malformed: "), reason


def test_calibration_oversized_body_blocks(wd: ModuleType, stub: CalibrationStub) -> None:
    """A body over the 1 MiB cap blocks as malformed."""
    stub.body = b" " * (1048576 + 10) + json.dumps(CALIB_IDLE).encode()
    blocked, reason = wd.calibration_status(stub.url, CALIB_TIMEOUT_S)
    assert blocked
    assert reason.startswith("malformed: body over"), reason


def test_blocked_seconds_metric_keeps_precision(wd: ModuleType) -> None:
    """A long block is written with millisecond precision, not in exponent notation."""
    state = {UNIT: wd.UnitState(streak=3, blocked_since=1000.0)}
    text = wd.metrics_text(1234567.8916, [UNIT], state, {UNIT})
    assert f'dds_watchdog_restart_blocked_seconds{{unit="{UNIT}"}} 1233567.892\n' in text


def test_calibration_bad_url_blocks(wd: ModuleType) -> None:
    """An empty URL or a non-http scheme blocks as malformed without any I/O."""
    for url in ["", "file:///etc/hostname", "ftp://127.0.0.1/x"]:
        blocked, reason = wd.calibration_status(url, CALIB_TIMEOUT_S)
        assert blocked, url
        assert reason.startswith("malformed: "), reason


@pytest.mark.parametrize("url", ["http://[::1/x", "http://[::1]:notaport/x", "gopher://127.0.0.1/x"])
def test_calibration_malformed_url_never_raises(wd: ModuleType, url: str) -> None:
    """An unterminated IPv6 bracket, a non-numeric port or an unsupported scheme blocks as malformed, never raises."""
    blocked, reason = wd.calibration_status(url, CALIB_TIMEOUT_S)
    assert blocked, url
    assert reason.startswith("malformed: "), reason


@pytest.mark.parametrize("timeout_s", [float("inf"), float("nan"), 0.0, -1.0])
def test_calibration_bad_timeout_never_raises(wd: ModuleType, stub: CalibrationStub, timeout_s: float) -> None:
    """A timeout urllib or the socket rejects (inf overflows time_t, nan, 0, negative) blocks, never raises."""
    blocked, reason = wd.calibration_status(stub.url, timeout_s)
    if blocked:
        assert reason.startswith(("malformed: ", "unreachable: ")), reason
    else:
        assert reason == "idle"


WRONG_CALIBRATION_BODIES: list[object] = [
    {"active": "false", "flows": [], "unknown": []},
    {"active": False, "flows": "none", "unknown": []},
    {"active": False, "flows": [], "unknown": [{"reason": "torque_state stale"}]},
    {"active": True, "flows": [{"name": "rgbd:floor"}], "unknown": []},
    {"active": True, "flows": [{"name": "rgbd:floor", "reason": 1}], "unknown": []},
    {"active": False, "flows": [], "unknown": None},
    {"active": False, "flows": ["rgbd:floor"], "unknown": []},
    {"active": False, "flows": []},
    {"flows": [], "unknown": []},
    {"active": 0, "flows": [], "unknown": []},
    {"active": False, "flows": [{"name": "rgbd:floor", "reason": "sweep.running"}], "unknown": []},
    [],
    None,
    "idle",
]


@pytest.mark.parametrize("body", WRONG_CALIBRATION_BODIES, ids=[json.dumps(body) for body in WRONG_CALIBRATION_BODIES])
def test_calibration_wrong_types_blocks(wd: ModuleType, stub: CalibrationStub, body: object) -> None:
    """Valid JSON of the wrong shape (string active, flows not a list, entry without name, active disagreeing with
    flows, ...) blocks as malformed."""
    stub.set_json(body)
    blocked, reason = wd.calibration_status(stub.url, CALIB_TIMEOUT_S)
    assert blocked
    assert reason.startswith("malformed: "), reason


def test_calibration_retry_once_then_ok(wd: ModuleType, stub: CalibrationStub) -> None:
    """A connection closed without a response is retried once; an idle second answer is not blocked."""
    stub.drop_first = 1
    assert wd.calibration_status(stub.url, CALIB_TIMEOUT_S) == (False, "idle")
    assert stub.requests == 2


def test_calibration_retries_only_once(wd: ModuleType, stub: CalibrationStub) -> None:
    """Two dropped connections in a row block as unreachable after exactly two requests."""
    stub.drop_first = 5
    blocked, reason = wd.calibration_status(stub.url, CALIB_TIMEOUT_S)
    assert blocked
    assert reason.startswith("unreachable: "), reason
    assert stub.requests == 2


# ---- run cycle: subprocess runs against fake roots and a fake systemctl ----

WEDGED_ROWS = [udp_row(0, 7458, 1001600)]


class Harness:
    """A fake host for subprocess runs of the script.

    Attributes:
        root (Path): Scratch directory.
        env (dict[str, str]): Environment of the runs.
        restarts_log (Path): One line per `systemctl restart` call.
        fail_flag (Path): When it exists, the fake `systemctl restart` exits 1.
        list_fail_flag (Path): When it exists, the fake `systemctl list-units` exits 1.
        list_empty_flag (Path): When it exists, the fake `systemctl list-units` prints no unit.
        units_file (Path): When it exists, the fake `systemctl list-units` prints it instead of the one test unit.
        calib (CalibrationStub): The calibration endpoint the runs ask (answers idle unless a test changes it).
    """

    def __init__(self, root: Path, calib: CalibrationStub) -> None:
        """Create the fake systemctl and the default environment.

        Args:
            root (Path): Scratch directory.
            calib (CalibrationStub): Calibration endpoint stub.
        """
        self.root = root
        self.calib = calib
        bindir = root / "bin"
        bindir.mkdir()
        self.restarts_log = root / "restarts.log"
        self.fail_flag = root / "fail-restart"
        self.list_fail_flag = root / "fail-list"
        self.list_empty_flag = root / "empty-list"
        self.units_file = root / "units"
        fake = bindir / "systemctl"
        fake.write_text(
            "#!/bin/sh\n"
            f'case "$1" in\n'
            f"  list-units) [ -e {self.list_fail_flag} ] && exit 1; [ -e {self.list_empty_flag} ] && exit 0; "
            f"if [ -e {self.units_file} ]; then cat {self.units_file}; exit 0; fi; "
            f'echo "{UNIT} loaded active running ROS2 test unit" ;;\n'
            f'  restart) echo "$2" >> {self.restarts_log}; [ -e {self.fail_flag} ] && exit 1; exit 0 ;;\n'
            "esac\n"
        )
        fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
        self.textfile = root / "textfile" / "dds_watchdog.prom"
        self.textfile.parent.mkdir()
        self.state_file = root / "state" / "state.json"
        self.env = {
            **os.environ,
            "PATH": f"{bindir}:{os.environ['PATH']}",
            "SYSTEMCTL": str(fake),
            "DDS_WATCHDOG_PROC_ROOT": str(root / "proc"),
            "DDS_WATCHDOG_CGROUP_ROOT": str(root / "sys/fs/cgroup"),
            "DDS_WATCHDOG_STATE_FILE": str(self.state_file),
            "DDS_WATCHDOG_TEXTFILE": str(self.textfile),
            "DDS_WATCHDOG_DEPLOY_LOCK": str(root / "deploy.lock"),
            "DDS_WATCHDOG_RX_MIN_BYTES": str(RX_MIN),
            "DDS_WATCHDOG_CALIBRATION_URL": calib.url,
            "DDS_WATCHDOG_CALIBRATION_TIMEOUT_S": str(CALIB_TIMEOUT_S),
        }

    def set_threads(self, threads: list[tuple[str, str]], rows: list[str]) -> None:
        """Replace the fake proc and cgroup tree of the unit (pid 100).

        Args:
            threads (list[tuple[str, str]]): (comm, wchan) per thread.
            rows (list[str]): udp rows.
        """
        proc = self.root / "proc"
        if proc.exists():
            subprocess.run(["rm", "-rf", str(proc)], check=True)
        for i, (comm, wchan) in enumerate(threads):
            add_thread(proc, 100, 100 + i, comm, wchan)
        write_net(proc, "udp", rows)
        add_procs(self.root / "sys/fs/cgroup", UNIT, [100])

    def run(self, **env: str) -> subprocess.CompletedProcess[str]:
        """Run the script once.

        Args:
            **env (str): Extra environment variables.

        Returns:
            subprocess.CompletedProcess[str]: The finished run.
        """
        return subprocess.run(
            [sys.executable, str(SCRIPT)], env={**self.env, **env}, capture_output=True, text=True, check=False
        )

    def restarts(self) -> list[str]:
        """List the units passed to `systemctl restart`.

        Returns:
            list[str]: Unit names, one per call.
        """
        return self.restarts_log.read_text().split() if self.restarts_log.exists() else []

    def metrics(self) -> str:
        """Read the textfile.

        Returns:
            str: Its content.
        """
        return self.textfile.read_text()


@pytest.fixture
def host(tmp_path: Path, stub: CalibrationStub) -> Harness:
    """A fake host with a wedged unit and an idle calibration endpoint.

    Args:
        tmp_path (Path): pytest temp dir.
        stub (CalibrationStub): Calibration endpoint, answering idle.

    Returns:
        Harness: The host.
    """
    harness = Harness(tmp_path, stub)
    harness.set_threads([WEDGED], WEDGED_ROWS)
    return harness


def test_three_suspect_runs_restart_once(host: Harness) -> None:
    """Runs 1 and 2 only count; run 3 restarts the unit once and the metrics show one restart."""
    for _ in range(2):
        assert host.run().returncode == 0
    assert host.restarts() == []
    assert f'dds_watchdog_streak{{unit="{UNIT}"}} 2' in host.metrics()
    proc = host.run()
    assert proc.returncode == 0, proc.stderr
    assert host.restarts() == [UNIT]
    assert (
        f"restarting {UNIT}: thread dds.udp.7458 (pid 100) in futex_wait_queue, rx_queue 1001600 B for 3 samples"
        in proc.stdout
    )
    text = host.metrics()
    assert f'dds_watchdog_restarts_total{{unit="{UNIT}"}} 1' in text
    assert f'dds_watchdog_streak{{unit="{UNIT}"}} 0' in text


def test_restarts_total_is_exported_at_zero_for_a_healthy_unit(host: Harness) -> None:
    """A listed unit without restarts exports restarts_total 0, so its first restart is a 0 -> 1 step for increase()."""
    host.set_threads([HEALTHY], [udp_row(0, 7418, 0)])
    assert host.run().returncode == 0
    assert f'dds_watchdog_restarts_total{{unit="{UNIT}"}} 0' in host.metrics()
    assert json.loads(host.state_file.read_text())[UNIT]["restarts_total"] == 0


def test_first_restart_moves_the_series_from_0_to_1(host: Harness) -> None:
    """The series exists at 0 before the first restart and reads 1 after it: increase() over both files is 1."""
    host.run()
    before = host.metrics()
    for _ in range(2):
        host.run()
    after = host.metrics()
    assert f'dds_watchdog_restarts_total{{unit="{UNIT}"}} 0' in before
    assert f'dds_watchdog_restarts_total{{unit="{UNIT}"}} 1' in after


def test_gap_resets_streak(host: Harness) -> None:
    """A healthy sample between suspect samples resets the streak, so no restart happens."""
    host.run()
    host.run()
    host.set_threads([HEALTHY], [udp_row(0, 7418, 0)])
    host.run()
    assert f'dds_watchdog_streak{{unit="{UNIT}"}} 0' in host.metrics()
    host.set_threads([WEDGED], WEDGED_ROWS)
    host.run()
    host.run()
    assert host.restarts() == []


def test_fresh_deploy_lock_blocks_restart(host: Harness) -> None:
    """A deploy lock younger than the max age holds the restart back; the streak stays at the limit."""
    (host.root / "deploy.lock").write_text("")
    for _ in range(4):
        host.run()
    assert host.restarts() == []
    assert f'dds_watchdog_streak{{unit="{UNIT}"}} 4' in host.metrics()


def test_old_deploy_lock_is_ignored(host: Harness) -> None:
    """A deploy lock older than the max age does not block the restart."""
    lock = host.root / "deploy.lock"
    lock.write_text("")
    old = time.time() - 7200
    os.utime(lock, (old, old))
    for _ in range(3):
        host.run(DDS_WATCHDOG_DEPLOY_LOCK_MAX_AGE_S="3600")
    assert host.restarts() == [UNIT]


def test_rate_limit_blocks_second_restart(host: Harness) -> None:
    """After a restart, a still-wedged unit is not restarted again within the minimum interval."""
    for _ in range(9):
        host.run()
    assert host.restarts() == [UNIT]
    state = json.loads(host.state_file.read_text())
    state[UNIT]["last_restart"] -= 301
    host.state_file.write_text(json.dumps(state))
    for _ in range(3):
        host.run()
    assert host.restarts() == [UNIT, UNIT]
    assert f'dds_watchdog_restarts_total{{unit="{UNIT}"}} 2' in host.metrics()


def test_dry_run_logs_without_restart(host: Harness) -> None:
    """Dry run (value parsed case-insensitively) logs "would restart" and changes no counter."""
    for _ in range(3):
        proc = host.run(DDS_WATCHDOG_DRY_RUN="Yes")
    assert "would restart" in proc.stdout
    assert "restarting" not in proc.stdout
    assert "dds_watchdog: dds_watchdog:" not in proc.stdout
    assert host.restarts() == []
    text = host.metrics()
    assert f'dds_watchdog_restarts_total{{unit="{UNIT}"}} 0' in text
    assert f'dds_watchdog_streak{{unit="{UNIT}"}} 3' in text


def test_failed_restart_not_counted(host: Harness) -> None:
    """A failing systemctl restart logs an error, counts nothing, keeps the streak and still writes metrics."""
    host.fail_flag.write_text("")
    for _ in range(3):
        proc = host.run()
    assert proc.returncode == 0
    assert host.restarts() == [UNIT]
    assert f"systemctl restart {UNIT} failed" in proc.stdout
    text = host.metrics()
    assert f'dds_watchdog_restarts_total{{unit="{UNIT}"}} 0' in text
    assert f'dds_watchdog_streak{{unit="{UNIT}"}} 3' in text
    assert "dds_watchdog_up 1" in text
    # The failed attempt armed the rate limit: the next run does not call systemctl, the streak keeps growing.
    host.run()
    assert host.restarts() == [UNIT]
    assert f'dds_watchdog_streak{{unit="{UNIT}"}} 4' in host.metrics()
    # After the interval the restart is retried.
    state = json.loads(host.state_file.read_text())
    state[UNIT]["last_attempt"] -= 301
    host.state_file.write_text(json.dumps(state))
    host.run()
    assert host.restarts() == [UNIT, UNIT]
    assert f'dds_watchdog_restarts_total{{unit="{UNIT}"}} 0' in host.metrics()


def test_corrupt_state_file_resets(host: Harness) -> None:
    """A corrupt state file starts empty with a warning and exits 0; a missing one too."""
    host.state_file.parent.mkdir()
    host.state_file.write_text("{not json")
    proc = host.run()
    assert proc.returncode == 0
    assert "starting empty" in proc.stdout
    assert f'dds_watchdog_streak{{unit="{UNIT}"}} 1' in host.metrics()
    host.state_file.write_text('["wrong", "shape"]')
    assert host.run().returncode == 0
    host.state_file.unlink()
    proc = host.run()
    assert proc.returncode == 0 and "starting empty" in proc.stdout


WRONG_TYPE_STATES = [
    {UNIT: {"streak": None}},
    {UNIT: {"streak": "2"}},
    {UNIT: {"streak": True}},
    {UNIT: {"streak": 1.5}},
    {UNIT: {"restarts_total": "x"}},
    {UNIT: {"last_restart": "yesterday"}},
    {UNIT: {"last_attempt": None}},
    {UNIT: {"blocked_since": None}},
    {UNIT: {"blocked_since": "now"}},
    {UNIT: {"blocked_since": True}},
    {UNIT: "not a dict"},
    {UNIT: ["streak", 1]},
    {UNIT: {"unknown_field": 1}},
]


@pytest.mark.parametrize("bad", WRONG_TYPE_STATES, ids=[json.dumps(bad) for bad in WRONG_TYPE_STATES])
def test_wrong_type_state_resets(host: Harness, bad: dict[str, object]) -> None:
    """Well-formed JSON with wrong field types starts empty with a warning, exits 0 and leaks nothing to the metrics."""
    host.state_file.parent.mkdir()
    host.state_file.write_text(json.dumps(bad))
    proc = host.run()
    assert proc.returncode == 0, proc.stderr
    assert "starting empty" in proc.stdout
    text = host.metrics()
    assert f'dds_watchdog_streak{{unit="{UNIT}"}} 1' in text
    assert f'dds_watchdog_restarts_total{{unit="{UNIT}"}} 0' in text


def test_valid_state_is_kept(host: Harness) -> None:
    """A state file with correct types (ints and floats) is used, not reset."""
    host.state_file.parent.mkdir()
    host.state_file.write_text(json.dumps({UNIT: {"streak": 1, "last_restart": 5, "restarts_total": 4}}))
    proc = host.run()
    assert "starting empty" not in proc.stdout
    text = host.metrics()
    assert f'dds_watchdog_streak{{unit="{UNIT}"}} 2' in text
    assert f'dds_watchdog_restarts_total{{unit="{UNIT}"}} 4' in text


def test_state_with_blocked_since_is_kept(host: Harness) -> None:
    """A state file carrying blocked_since (int or float) loads without a reset and keeps the value."""
    host.state_file.parent.mkdir()
    host.calib.set_json(CALIB_FIXTURE)
    since = time.time() - 50
    host.state_file.write_text(json.dumps({UNIT: {"streak": 5, "blocked_since": since}}))
    proc = host.run()
    assert "starting empty" not in proc.stdout
    assert json.loads(host.state_file.read_text())[UNIT]["blocked_since"] == since
    host.state_file.write_text(json.dumps({UNIT: {"streak": 1, "blocked_since": 0}}))
    proc = host.run()
    assert "starting empty" not in proc.stdout


def test_list_units_failure_writes_up_zero(host: Harness) -> None:
    """When listing the units fails: exit 0, textfile written with up 0, state kept, no scan and no restart."""
    for _ in range(2):
        host.run()
    before = host.state_file.read_text()
    host.list_fail_flag.write_text("")
    proc = host.run()
    assert proc.returncode == 0
    assert "listing ros2 units failed" in proc.stdout
    text = host.metrics()
    assert "dds_watchdog_up 0" in text
    assert "\ndds_watchdog_up 1\n" not in text
    assert "dds_watchdog_streak{" not in text
    assert host.state_file.read_text() == before
    assert host.restarts() == []


def test_stopped_unit_keeps_restart_count(host: Harness) -> None:
    """A unit that is no longer running drops its streak but keeps restarts_total and last_restart in the metrics."""
    for _ in range(3):
        host.run()
    assert host.restarts() == [UNIT]
    before = json.loads(host.state_file.read_text())[UNIT]["last_restart"]
    host.list_empty_flag.write_text("")
    assert host.run().returncode == 0
    assert json.loads(host.state_file.read_text()) == {
        UNIT: {"streak": 0, "last_restart": before, "last_attempt": 0.0, "restarts_total": 1, "blocked_since": 0.0}
    }
    text = host.metrics()
    assert f'dds_watchdog_restarts_total{{unit="{UNIT}"}} 1' in text
    assert "dds_watchdog_streak{" not in text
    assert "dds_watchdog_up 1" in text


def test_config_defaults(wd: ModuleType) -> None:
    """Config.from_env({}) gives the documented defaults."""
    assert wd.Config.from_env({}) == wd.Config(
        proc_root=Path("/proc"),
        cgroup_root=Path("/sys/fs/cgroup"),
        state_file=Path("/var/lib/dds-watchdog/state.json"),
        textfile=Path("/var/lib/alloy/textfile/dds_watchdog.prom"),
        samples=3,
        rx_min_bytes=262144,
        min_restart_interval_s=300,
        deploy_lock=Path("/var/lib/ros2-deploy/deploy.lock"),
        deploy_lock_max_age_s=14400,
        dry_run=False,
        systemctl="systemctl",
        calibration_url="http://127.0.0.1/api/calibration/active",
        calibration_timeout_s=3.0,
        max_restarts_per_run=3,
    )


def test_config_calibration_from_env(wd: ModuleType) -> None:
    """The calibration URL and a fractional timeout are read from the environment."""
    cfg = wd.Config.from_env(
        {
            "DDS_WATCHDOG_CALIBRATION_URL": "http://client.ros2.lan/api/calibration/active",
            "DDS_WATCHDOG_CALIBRATION_TIMEOUT_S": "2.5",
        }
    )
    assert cfg.calibration_url == "http://client.ros2.lan/api/calibration/active"
    assert cfg.calibration_timeout_s == 2.5


@pytest.mark.parametrize("value", ["inf", "-inf", "nan", "0", "-1", "abc", ""])
def test_config_bad_calibration_timeout_uses_default(
    wd: ModuleType, caplog: pytest.LogCaptureFixture, value: str
) -> None:
    """A non-finite, non-positive or non-numeric calibration timeout logs a WARNING and falls back to 3 s."""
    with caplog.at_level(logging.WARNING, logger="dds_watchdog"):
        cfg = wd.Config.from_env({"DDS_WATCHDOG_CALIBRATION_TIMEOUT_S": value})
    assert cfg.calibration_timeout_s == 3.0
    assert [r.levelname for r in caplog.records if "DDS_WATCHDOG_CALIBRATION_TIMEOUT_S" in r.getMessage()] == [
        "WARNING"
    ]


def test_label_values_are_escaped(wd: ModuleType) -> None:
    """Backslash, double quote and newline in a unit name are escaped per the Prometheus text format."""
    unit = 'ros2-we\\ird"\nname.service'
    state = {unit: wd.UnitState(streak=2, restarts_total=1, blocked_since=0.5)}
    text = wd.metrics_text(1.0, [unit], state, {unit})
    escaped = 'ros2-we\\\\ird\\"\\nname.service'
    assert f'dds_watchdog_suspect{{unit="{escaped}"}} 1' in text
    assert f'dds_watchdog_restart_blocked{{unit="{escaped}",reason="calibration"}} 1' in text
    assert f'dds_watchdog_restart_blocked_seconds{{unit="{escaped}"}} 0.5' in text
    assert f'dds_watchdog_streak{{unit="{escaped}"}} 2' in text
    assert f'dds_watchdog_restarts_total{{unit="{escaped}"}} 1' in text
    assert all(line.startswith(("#", "dds_watchdog_")) for line in text.splitlines())


def test_metrics_file_format(host: Harness) -> None:
    """The textfile has HELP and TYPE lines, unit labels, and ends with a newline; no temp file is left behind."""
    host.run()
    text = host.metrics()
    assert text.endswith("\n")
    for name, kind in [
        ("dds_watchdog_up", "gauge"),
        ("dds_watchdog_last_run_timestamp_seconds", "gauge"),
        ("dds_watchdog_suspect", "gauge"),
        ("dds_watchdog_streak", "gauge"),
        ("dds_watchdog_restarts_total", "counter"),
        ("dds_watchdog_restart_blocked", "gauge"),
        ("dds_watchdog_restart_blocked_seconds", "gauge"),
    ]:
        assert f"# HELP {name} " in text
        assert f"# TYPE {name} {kind}" in text
    assert f'dds_watchdog_suspect{{unit="{UNIT}"}} 1' in text
    assert f'dds_watchdog_restart_blocked{{unit="{UNIT}",reason="calibration"}} 0' in text
    assert f'dds_watchdog_restart_blocked_seconds{{unit="{UNIT}"}} 0' in text
    assert sorted(p.name for p in host.textfile.parent.iterdir()) == ["dds_watchdog.prom"]


def test_healthy_host_writes_zero_suspects(host: Harness) -> None:
    """A healthy unit gives suspect 0 and streak 0, and is never restarted."""
    host.set_threads([HEALTHY], [udp_row(0, 7418, 0)])
    for _ in range(4):
        assert host.run().returncode == 0
    text = host.metrics()
    assert f'dds_watchdog_suspect{{unit="{UNIT}"}} 0' in text
    assert f'dds_watchdog_streak{{unit="{UNIT}"}} 0' in text
    assert host.restarts() == []


def test_unwritable_textfile_exits_nonzero(host: Harness) -> None:
    """The only non-zero exit: the textfile directory is missing."""
    proc = host.run(DDS_WATCHDOG_TEXTFILE=str(host.root / "missing" / "x.prom"))
    assert proc.returncode == 1


# ---- calibration guard in the run cycle ----

BLOCKED_ACTIVE_LOG = f"not restarting {UNIT}: calibration active: rgbd:head-arm, general:home_arm"
UNIT2 = "ros2-other.service"


def metric_value(text: str, series: str) -> float:
    """Read the value of one series from the metrics text.

    Args:
        text (str): Metrics text.
        series (str): Metric name with labels, e.g. 'dds_watchdog_streak{unit="x"}'.

    Returns:
        float: The value.
    """
    for line in text.splitlines():
        if line.startswith(series + " "):
            return float(line.split()[-1])
    raise AssertionError(f"{series} not in metrics:\n{text}")


def blocked_series(unit: str = UNIT) -> str:
    """Name the restart_blocked series of a unit.

    Args:
        unit (str): Unit name.

    Returns:
        str: The series with labels.
    """
    return f'dds_watchdog_restart_blocked{{unit="{unit}",reason="calibration"}}'


def blocked_seconds_series(unit: str = UNIT) -> str:
    """Name the restart_blocked_seconds series of a unit.

    Args:
        unit (str): Unit name.

    Returns:
        str: The series with labels.
    """
    return f'dds_watchdog_restart_blocked_seconds{{unit="{unit}"}}'


def test_wedged_unit_not_restarted_during_calibration(
    host: Harness, wd: ModuleType, caplog: pytest.LogCaptureFixture
) -> None:
    """An active calibration holds the restart: no systemctl restart, a WARNING naming the flows, blocked metric 1."""
    host.calib.set_json(CALIB_FIXTURE)
    for _ in range(3):
        proc = host.run()
        assert proc.returncode == 0, proc.stderr
    assert host.restarts() == []
    assert BLOCKED_ACTIVE_LOG in proc.stdout
    assert "restarting ros2" not in proc.stdout.replace("not restarting", "")
    text = host.metrics()
    assert metric_value(text, blocked_series()) == 1
    assert f'dds_watchdog_streak{{unit="{UNIT}"}} 3' in text
    # The same run in-process shows the log level.
    with caplog.at_level(logging.INFO, logger="dds_watchdog"):
        assert wd.run_once(wd.Config.from_env(host.env), time.time()) == 0
    assert [r.levelname for r in caplog.records if r.getMessage() == BLOCKED_ACTIVE_LOG] == ["WARNING"]
    assert host.restarts() == []


def test_unreachable_calibration_blocks_restart(host: Harness) -> None:
    """Fail closed end to end: an endpoint nobody listens on holds the restart and logs it as unreachable."""
    url = f"http://127.0.0.1:{free_port()}{CALIB_PATH}"
    for _ in range(4):
        proc = host.run(DDS_WATCHDOG_CALIBRATION_URL=url)
    assert host.restarts() == []
    assert f"not restarting {UNIT}: calibration unreachable: " in proc.stdout
    assert metric_value(host.metrics(), blocked_series()) == 1


def test_unknown_calibration_source_blocks_restart(host: Harness) -> None:
    """An unknown source with active false holds the restart."""
    host.calib.set_json(CALIB_UNKNOWN_ONLY)
    for _ in range(3):
        proc = host.run()
    assert host.restarts() == []
    assert f"not restarting {UNIT}: calibration unknown: servo:arm (torque_state stale)" in proc.stdout


def test_deferred_restart_after_calibration_ends(host: Harness) -> None:
    """Blocked on runs 3 and 4, then idle: run 5 restarts exactly once and the blocked metrics drop to 0."""
    host.calib.set_json(CALIB_FIXTURE)
    for _ in range(4):
        host.run()
    assert host.restarts() == []
    assert f'dds_watchdog_streak{{unit="{UNIT}"}} 4' in host.metrics()
    assert json.loads(host.state_file.read_text())[UNIT]["blocked_since"] > 0
    host.calib.set_json(CALIB_IDLE)
    proc = host.run()
    assert host.restarts() == [UNIT]
    assert f"restarting {UNIT}: thread dds.udp.7458" in proc.stdout
    text = host.metrics()
    assert metric_value(text, blocked_series()) == 0
    assert metric_value(text, blocked_seconds_series()) == 0
    assert f'dds_watchdog_restarts_total{{unit="{UNIT}"}} 1' in text
    assert json.loads(host.state_file.read_text())[UNIT]["blocked_since"] == 0.0
    for _ in range(3):
        host.run()
    assert host.restarts() == [UNIT]


def test_blocked_seconds_metric_grows(host: Harness) -> None:
    """blocked_since is set on the first blocked run and kept; the seconds metric is now - blocked_since."""
    host.calib.set_json(CALIB_FIXTURE)
    for _ in range(3):
        host.run()
    first = json.loads(host.state_file.read_text())[UNIT]["blocked_since"]
    assert first > 0
    assert 0 <= metric_value(host.metrics(), blocked_seconds_series()) < 5
    state = json.loads(host.state_file.read_text())
    state[UNIT]["blocked_since"] = first - 700
    host.state_file.write_text(json.dumps(state))
    host.run()
    assert json.loads(host.state_file.read_text())[UNIT]["blocked_since"] == first - 700
    assert 700 <= metric_value(host.metrics(), blocked_seconds_series()) < 710
    assert host.restarts() == []


def test_recovered_unit_clears_blocked(host: Harness) -> None:
    """A blocked unit that stops being a suspect clears blocked_since and its metrics, with no restart."""
    host.calib.set_json(CALIB_FIXTURE)
    for _ in range(3):
        host.run()
    host.set_threads([HEALTHY], [udp_row(0, 7418, 0)])
    host.run()
    assert host.restarts() == []
    assert json.loads(host.state_file.read_text())[UNIT]["blocked_since"] == 0.0
    text = host.metrics()
    assert metric_value(text, blocked_series()) == 0
    assert metric_value(text, blocked_seconds_series()) == 0


def test_healthy_host_makes_no_http_calls(host: Harness) -> None:
    """Neither a healthy unit nor a suspect below the sample limit asks the calibration endpoint."""
    host.set_threads([HEALTHY], [udp_row(0, 7418, 0)])
    for _ in range(4):
        host.run()
    assert host.calib.requests == 0
    host.set_threads([WEDGED], WEDGED_ROWS)
    for _ in range(2):
        host.run()
    assert host.calib.requests == 0
    host.run()
    assert host.calib.requests == 1


def test_deploy_lock_and_rate_limit_make_no_http_calls(host: Harness) -> None:
    """The deploy lock and the rate limit are checked before calibration: they hold the restart without a request."""
    (host.root / "deploy.lock").write_text("")
    for _ in range(4):
        host.run()
    assert host.calib.requests == 0
    (host.root / "deploy.lock").unlink()
    host.run()
    assert host.restarts() == [UNIT]
    assert host.calib.requests == 1
    for _ in range(4):
        host.run()
    assert host.restarts() == [UNIT]
    assert host.calib.requests == 1


def test_one_calibration_call_per_run(host: Harness) -> None:
    """Two units due in the same run share one calibration request; both are held back."""
    add_thread(host.root / "proc", 200, 200, "dds.udp.7460", "futex_wait_queue")
    write_net(host.root / "proc", "udp", [*WEDGED_ROWS, udp_row(1, 7460, 1001600)])
    add_procs(host.root / "sys/fs/cgroup", UNIT2, [200])
    host.units_file.write_text(f"{UNIT} loaded active running a\n{UNIT2} loaded active running b\n")
    host.calib.set_json(CALIB_FIXTURE)
    for _ in range(3):
        proc = host.run()
    assert host.calib.requests == 1
    assert host.restarts() == []
    assert f"not restarting {UNIT}: calibration" in proc.stdout
    assert f"not restarting {UNIT2}: calibration" in proc.stdout
    text = host.metrics()
    assert metric_value(text, blocked_series()) == 1
    assert metric_value(text, blocked_series(UNIT2)) == 1


def test_dry_run_still_checks_calibration(host: Harness) -> None:
    """Dry run asks the endpoint too: blocked logs "not restarting ... calibration", idle logs "would restart"."""
    host.calib.set_json(CALIB_FIXTURE)
    for _ in range(3):
        proc = host.run(DDS_WATCHDOG_DRY_RUN="1")
    assert BLOCKED_ACTIVE_LOG in proc.stdout
    assert "would restart" not in proc.stdout
    assert host.calib.requests == 1
    assert metric_value(host.metrics(), blocked_series()) == 1
    host.calib.set_json(CALIB_IDLE)
    proc = host.run(DDS_WATCHDOG_DRY_RUN="1")
    assert "would restart" in proc.stdout
    assert host.restarts() == []
    assert metric_value(host.metrics(), blocked_series()) == 0


def test_calibration_rechecked_after_each_restart(host: Harness) -> None:
    """Two units due in one run: the idle answer is not reused after a restart. The stub answers idle to the first
    request and active to every later one, so the first unit is restarted and the second is held back."""
    add_thread(host.root / "proc", 200, 200, "dds.udp.7460", "futex_wait_queue")
    write_net(host.root / "proc", "udp", [*WEDGED_ROWS, udp_row(1, 7460, 1001600)])
    add_procs(host.root / "sys/fs/cgroup", UNIT2, [200])
    host.units_file.write_text(f"{UNIT} loaded active running a\n{UNIT2} loaded active running b\n")
    host.calib.later_body = json.dumps(CALIB_FIXTURE).encode()
    for _ in range(3):
        proc = host.run()
        assert proc.returncode == 0, proc.stderr
    assert host.restarts() == [UNIT]
    assert host.calib.requests == 2
    assert f"not restarting {UNIT2}: calibration active: rgbd:head-arm, general:home_arm" in proc.stdout
    text = host.metrics()
    assert metric_value(text, blocked_series(UNIT2)) == 1
    assert metric_value(text, blocked_series()) == 0


def test_malformed_url_run_saves_state_and_metrics(host: Harness) -> None:
    """A URL urllib cannot parse never crashes the run: no restart, the streak is saved, the blocked metric is 1."""
    for _ in range(4):
        proc = host.run(DDS_WATCHDOG_CALIBRATION_URL="http://[::1/api/calibration/active")
        assert proc.returncode == 0, proc.stderr
    assert host.restarts() == []
    assert f"not restarting {UNIT}: calibration malformed: " in proc.stdout
    assert json.loads(host.state_file.read_text())[UNIT]["streak"] == 4
    assert metric_value(host.metrics(), blocked_series()) == 1


def test_bad_timeout_run_uses_default(host: Harness) -> None:
    """An infinite calibration timeout never crashes the run: it warns, uses 3 s and the idle answer allows the
    restart."""
    for _ in range(3):
        proc = host.run(DDS_WATCHDOG_CALIBRATION_TIMEOUT_S="inf")
        assert proc.returncode == 0, proc.stderr
    assert "DDS_WATCHDOG_CALIBRATION_TIMEOUT_S" in proc.stdout
    assert host.restarts() == [UNIT]
    assert json.loads(host.state_file.read_text())[UNIT]["restarts_total"] == 1


# ---- restart cap per run ----


def add_wedged_units(host: Harness, count: int) -> list[str]:
    """Make `count` running units wedged (the fixture unit plus extra ones), each with its own pid and port.

    Args:
        host (Harness): The fake host.
        count (int): Number of wedged units in total (at least 1).

    Returns:
        list[str]: Unit names in `systemctl list-units` order.
    """
    units = [UNIT]
    rows = list(WEDGED_ROWS)
    for i in range(1, count):
        name = f"ros2-extra{i}.service"
        port = 7458 + 2 * i
        add_thread(host.root / "proc", 100 + 100 * i, 100 + 100 * i, f"dds.udp.{port}", "futex_wait_queue")
        add_procs(host.root / "sys/fs/cgroup", name, [100 + 100 * i])
        rows.append(udp_row(i, port, 1001600))
        units.append(name)
    write_net(host.root / "proc", "udp", rows)
    host.units_file.write_text("".join(f"{u} loaded active running x\n" for u in units))
    return units


def streaks(host: Harness) -> dict[str, int]:
    """Read the streak of every unit from the state file.

    Args:
        host (Harness): The fake host.

    Returns:
        dict[str, int]: Unit name to streak.
    """
    return {u: v["streak"] for u, v in json.loads(host.state_file.read_text()).items()}


def test_restart_cap_limits_restarts_per_run(host: Harness) -> None:
    """5 due units with the default cap 3: 3 restarts, the other 2 keep their streak, get no calibration GET and are
    not counted as calibration-blocked; the next run restarts them."""
    units = add_wedged_units(host, 5)
    for _ in range(3):
        proc = host.run()
        assert proc.returncode == 0, proc.stderr
    assert host.restarts() == units[:3]
    assert host.calib.requests == 3
    for skipped in units[3:]:
        assert f"not restarting {skipped}: restart cap 3 per run reached" in proc.stdout
        assert streaks(host)[skipped] == 3
        assert metric_value(host.metrics(), blocked_series(skipped)) == 0
        assert metric_value(host.metrics(), blocked_seconds_series(skipped)) == 0
    assert "restart cap" not in "".join(line for line in proc.stdout.splitlines() if units[0] in line)
    proc = host.run()
    assert host.restarts() == units
    assert host.calib.requests == 5


def test_restart_cap_counts_failed_restarts(host: Harness) -> None:
    """A failed restart uses up the cap as well."""
    units = add_wedged_units(host, 5)
    host.fail_flag.touch()
    for _ in range(3):
        host.run()
    assert host.restarts() == units[:3]


def test_restart_cap_counts_dry_run(host: Harness) -> None:
    """Dry run logs "would restart" for at most the cap per run."""
    units = add_wedged_units(host, 5)
    for _ in range(3):
        proc = host.run(DDS_WATCHDOG_DRY_RUN="1")
    assert proc.stdout.count("would restart") == 3
    assert proc.stdout.count("restart cap 3 per run reached") == 2
    assert host.restarts() == []
    assert streaks(host)[units[4]] == 3


def test_restart_cap_from_env(host: Harness) -> None:
    """DDS_WATCHDOG_MAX_RESTARTS_PER_RUN changes the cap."""
    units = add_wedged_units(host, 3)
    for _ in range(3):
        host.run(DDS_WATCHDOG_MAX_RESTARTS_PER_RUN="1")
    assert host.restarts() == units[:1]


def test_restart_cap_does_not_count_blocked_units(host: Harness) -> None:
    """Units held back by calibration (not restarted) do not use up the cap."""
    units = add_wedged_units(host, 4)
    host.calib.set_json(CALIB_FIXTURE)
    for _ in range(3):
        proc = host.run()
    assert host.restarts() == []
    assert "restart cap" not in proc.stdout
    assert metric_value(host.metrics(), blocked_series(units[3])) == 1


def test_config_max_restarts_from_env(wd: ModuleType) -> None:
    """The cap is read from the environment as an integer and a non-integer fails like the other integer settings."""
    assert wd.Config.from_env({"DDS_WATCHDOG_MAX_RESTARTS_PER_RUN": "5"}).max_restarts_per_run == 5
    with pytest.raises(ValueError):
        wd.Config.from_env({"DDS_WATCHDOG_MAX_RESTARTS_PER_RUN": "many"})
