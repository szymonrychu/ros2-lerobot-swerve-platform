"""dds_watchdog detection core: wedged Fast DDS discovery threads found from /proc and cgroup v2 (fixture trees)."""

import importlib.util
import json
import os
import stat
import subprocess
import sys
import time
from collections.abc import Callable
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
    """

    def __init__(self, root: Path) -> None:
        """Create the fake systemctl and the default environment.

        Args:
            root (Path): Scratch directory.
        """
        self.root = root
        bindir = root / "bin"
        bindir.mkdir()
        self.restarts_log = root / "restarts.log"
        self.fail_flag = root / "fail-restart"
        self.list_fail_flag = root / "fail-list"
        self.list_empty_flag = root / "empty-list"
        fake = bindir / "systemctl"
        fake.write_text(
            "#!/bin/sh\n"
            f'case "$1" in\n'
            f"  list-units) [ -e {self.list_fail_flag} ] && exit 1; [ -e {self.list_empty_flag} ] && exit 0; "
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
def host(tmp_path: Path) -> Harness:
    """A fake host with a wedged unit.

    Args:
        tmp_path (Path): pytest temp dir.

    Returns:
        Harness: The host.
    """
    harness = Harness(tmp_path)
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
    assert "dds_watchdog_restarts_total{" not in text
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
    assert "dds_watchdog_restarts_total{" not in text
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
    assert "dds_watchdog_restarts_total{" not in host.metrics()


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
    assert "dds_watchdog_restarts_total{" not in text


def test_valid_state_is_kept(host: Harness) -> None:
    """A state file with correct types (ints and floats) is used, not reset."""
    host.state_file.parent.mkdir()
    host.state_file.write_text(json.dumps({UNIT: {"streak": 1, "last_restart": 5, "restarts_total": 4}}))
    proc = host.run()
    assert "starting empty" not in proc.stdout
    text = host.metrics()
    assert f'dds_watchdog_streak{{unit="{UNIT}"}} 2' in text
    assert f'dds_watchdog_restarts_total{{unit="{UNIT}"}} 4' in text


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
        UNIT: {"streak": 0, "last_restart": before, "last_attempt": 0.0, "restarts_total": 1}
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
    )


def test_label_values_are_escaped(wd: ModuleType) -> None:
    """Backslash, double quote and newline in a unit name are escaped per the Prometheus text format."""
    unit = 'ros2-we\\ird"\nname.service'
    state = {unit: wd.UnitState(streak=2, restarts_total=1)}
    text = wd.metrics_text(1.0, [unit], state, {unit})
    escaped = 'ros2-we\\\\ird\\"\\nname.service'
    assert f'dds_watchdog_suspect{{unit="{escaped}"}} 1' in text
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
    ]:
        assert f"# HELP {name} " in text
        assert f"# TYPE {name} {kind}" in text
    assert f'dds_watchdog_suspect{{unit="{UNIT}"}} 1' in text
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
