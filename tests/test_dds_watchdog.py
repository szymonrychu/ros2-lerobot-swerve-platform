"""dds_watchdog detection core: wedged Fast DDS discovery threads found from /proc and cgroup v2 (fixture trees)."""

import importlib.util
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
