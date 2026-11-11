#!/usr/bin/env python3
"""Detect ros2-* systemd units whose Fast DDS discovery thread is deadlocked (eProsima/Fast-DDS#6502).

A wedged participant keeps its process alive but its metatraffic receive thread (comm `dds.udp.<port>`, port in
7410..7608 at an even offset) sits in futex_wait_queue instead of the socket receive path, while the kernel queue of
that UDP port fills up. A unit is a suspect when such a thread exists and the port's rx_queue is at least a threshold.

This module holds the pure detection functions: they read a /proc tree and a cgroup v2 tree, both passed in, so tests
can use fixture trees.

Environment: none (the detection functions take their inputs as arguments).
"""

import logging
import re
from dataclasses import dataclass
from pathlib import Path

METATRAFFIC_PORT_MIN = 7410
METATRAFFIC_PORT_MAX = 7608
METATRAFFIC_PORT_STEP = 2
WEDGED_WCHAN = "futex_wait_queue"
COMM_RE = re.compile(r"dds\.udp\.(\d+)")
UDP_TABLES = ("udp", "udp6")
SLICE = "system.slice"

LOG = logging.getLogger("dds_watchdog")


@dataclass(frozen=True)
class WedgeEvidence:
    """What was seen on a suspect thread.

    Attributes:
        unit (str): systemd unit name.
        pid (int): Process id.
        tid (int): Thread id.
        port (int): Metatraffic UDP port of the thread.
        rx_queue (int): Bytes queued on that port.
        wchan (str): Kernel wait channel of the thread.
    """

    unit: str
    pid: int
    tid: int
    port: int
    rx_queue: int
    wchan: str


def metatraffic_port(comm: str) -> int | None:
    """Extract the metatraffic port from a thread name.

    Args:
        comm (str): Thread name, e.g. "dds.udp.7458".

    Returns:
        int | None: The port when comm is `dds.udp.<P>` with P in 7410..7608 at an even offset, else None.
    """
    match = COMM_RE.fullmatch(comm)
    if match is None:
        return None
    port = int(match.group(1))
    if not METATRAFFIC_PORT_MIN <= port <= METATRAFFIC_PORT_MAX:
        return None
    if (port - METATRAFFIC_PORT_MIN) % METATRAFFIC_PORT_STEP:
        return None
    return port


def rx_queues(proc_root: Path) -> dict[int, int]:
    """Read the receive queue of every local UDP port.

    Args:
        proc_root (Path): The /proc directory (reads net/udp and net/udp6; a missing file is skipped).

    Returns:
        dict[int, int]: Local port -> largest rx_queue in bytes over all sockets of that port.
    """
    queues: dict[int, int] = {}
    for table in UDP_TABLES:
        try:
            lines = (proc_root / "net" / table).read_text().splitlines()
        except OSError:
            continue
        for line in lines[1:]:
            fields = line.split()
            try:
                port = int(fields[1].rsplit(":", 1)[1], 16)
                rx = int(fields[4].split(":", 1)[1], 16)
            except (IndexError, ValueError):
                continue
            queues[port] = max(queues.get(port, 0), rx)
    return queues


def unit_pids(cgroup_root: Path, unit: str) -> list[int]:
    """List the processes of a unit, including those in child cgroups.

    Args:
        cgroup_root (Path): The cgroup v2 mount, e.g. /sys/fs/cgroup.
        unit (str): Unit name, e.g. "ros2-web_ui.service".

    Returns:
        list[int]: Pids from cgroup.procs of the unit and all nested cgroups; [] when the unit has no cgroup.
    """
    base = cgroup_root / SLICE / unit
    pids: list[int] = []
    for procs in sorted([base / "cgroup.procs", *base.rglob("cgroup.procs")]):
        try:
            pids += [int(line) for line in procs.read_text().split()]
        except (OSError, ValueError):
            continue
    return sorted(set(pids))


def scan_unit(
    proc_root: Path, cgroup_root: Path, unit: str, queues: dict[int, int], rx_min: int
) -> WedgeEvidence | None:
    """Look for a wedged discovery thread in a unit.

    A thread is a suspect when its name is `dds.udp.<P>` (P metatraffic), its wchan is exactly futex_wait_queue and
    queues[P] >= rx_min. Threads or processes that vanish during the scan are skipped.

    Args:
        proc_root (Path): The /proc directory.
        cgroup_root (Path): The cgroup v2 mount.
        unit (str): Unit name.
        queues (dict[int, int]): Port -> rx_queue bytes, from rx_queues().
        rx_min (int): Smallest rx_queue in bytes that counts as full.

    Returns:
        WedgeEvidence | None: The first suspect thread, else None.
    """
    for pid in unit_pids(cgroup_root, unit):
        try:
            tids = sorted(int(path.name) for path in (proc_root / str(pid) / "task").iterdir())
        except (OSError, ValueError):
            continue
        for tid in tids:
            task = proc_root / str(pid) / "task" / str(tid)
            try:
                comm = (task / "comm").read_text().strip()
                port = metatraffic_port(comm)
                if port is None:
                    continue
                wchan = (task / "wchan").read_text().strip()
            except OSError:
                continue
            rx = queues.get(port, 0)
            if wchan == WEDGED_WCHAN and rx >= rx_min:
                LOG.debug("suspect %s pid %d tid %d port %d rx %d", unit, pid, tid, port, rx)
                return WedgeEvidence(unit=unit, pid=pid, tid=tid, port=port, rx_queue=rx, wchan=wchan)
    return None
