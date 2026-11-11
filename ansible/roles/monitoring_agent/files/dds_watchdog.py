#!/usr/bin/env python3
"""Detect ros2-* systemd units whose Fast DDS discovery thread is deadlocked (eProsima/Fast-DDS#6502).

A wedged participant keeps its process alive but its metatraffic receive thread (comm `dds.udp.<port>`, port in
7410..7608 at an even offset) sits in futex_wait_queue instead of the socket receive path, while the kernel queue of
that UDP port fills up. A unit is a suspect when such a thread exists and the port's rx_queue is at least a threshold.

The detection functions read a /proc tree and a cgroup v2 tree, both passed in, so tests can use fixture trees. A run
(run once by dds-watchdog.timer) samples every running ros2-*.service, counts consecutive suspect samples per unit in a
state file, restarts a unit once its streak reaches the configured number of samples (unless a deploy is running or the
unit was restarted recently) and writes metrics to the Alloy textfile collector. The state file and the textfile are
replaced atomically (temp file + rename). The run exits non-zero only when the textfile cannot be written.

Environment:
    DDS_WATCHDOG_PROC_ROOT: /proc directory (default /proc).
    DDS_WATCHDOG_CGROUP_ROOT: cgroup v2 mount (default /sys/fs/cgroup).
    DDS_WATCHDOG_STATE_FILE: JSON state file (default /var/lib/dds-watchdog/state.json).
    DDS_WATCHDOG_TEXTFILE: metrics output file (default /var/lib/alloy/textfile/dds_watchdog.prom).
    DDS_WATCHDOG_SAMPLES: consecutive suspect samples before a restart (default 3).
    DDS_WATCHDOG_RX_MIN_BYTES: smallest rx_queue in bytes that counts as full (default 262144).
    DDS_WATCHDOG_MIN_RESTART_INTERVAL_S: minimum seconds between watchdog restarts of one unit (default 300).
    DDS_WATCHDOG_DEPLOY_LOCK: deploy lock file (default /var/lib/ros2-deploy/deploy.lock).
    DDS_WATCHDOG_DEPLOY_LOCK_MAX_AGE_S: age in seconds after which the lock is stale (default 14400, the 240 minutes of
        ros2_deploy_lock_max_age_min that ros2-deploy-recover uses).
    DDS_WATCHDOG_DRY_RUN: "1", "true", "yes" or "on" (any case) only logs the restarts (default false).
    SYSTEMCTL: systemctl binary (default "systemctl", looked up on PATH).
"""

import json
import logging
import os
import re
import subprocess
import sys
import tempfile
import time
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from pathlib import Path

METATRAFFIC_PORT_MIN = 7410
METATRAFFIC_PORT_MAX = 7608
METATRAFFIC_PORT_STEP = 2
WEDGED_WCHAN = "futex_wait_queue"
COMM_RE = re.compile(r"dds\.udp\.(\d+)")
UDP_TABLES = ("udp", "udp6")
SLICE = "system.slice"
DEFAULT_PROC_ROOT = "/proc"
DEFAULT_CGROUP_ROOT = "/sys/fs/cgroup"
DEFAULT_STATE_FILE = "/var/lib/dds-watchdog/state.json"
DEFAULT_TEXTFILE = "/var/lib/alloy/textfile/dds_watchdog.prom"
DEFAULT_SAMPLES = 3
DEFAULT_RX_MIN_BYTES = 262144
DEFAULT_MIN_RESTART_INTERVAL_S = 300
DEFAULT_DEPLOY_LOCK = "/var/lib/ros2-deploy/deploy.lock"
DEFAULT_DEPLOY_LOCK_MAX_AGE_S = 14400
DEFAULT_SYSTEMCTL = "systemctl"
TRUE_VALUES = frozenset({"1", "true", "yes", "on"})
SYSTEMCTL_TIMEOUT_S = 60.0
UNIT_GLOB = "ros2-*.service"

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


@dataclass(frozen=True)
class Config:
    """Settings of one run.

    Attributes:
        proc_root (Path): The /proc directory.
        cgroup_root (Path): The cgroup v2 mount.
        state_file (Path): JSON state file.
        textfile (Path): Metrics output file.
        samples (int): Consecutive suspect samples before a restart.
        rx_min_bytes (int): Smallest rx_queue in bytes that counts as full.
        min_restart_interval_s (int): Minimum seconds between watchdog restarts of one unit.
        deploy_lock (Path): Deploy lock file.
        deploy_lock_max_age_s (int): Age in seconds after which the deploy lock is stale.
        dry_run (bool): Log restarts instead of doing them.
        systemctl (str): systemctl binary.
    """

    proc_root: Path
    cgroup_root: Path
    state_file: Path
    textfile: Path
    samples: int
    rx_min_bytes: int
    min_restart_interval_s: int
    deploy_lock: Path
    deploy_lock_max_age_s: int
    dry_run: bool
    systemctl: str

    @classmethod
    def from_env(cls, environ: Mapping[str, str]) -> "Config":
        """Build the settings from environment variables, with the defaults of the module docstring.

        Args:
            environ (Mapping[str, str]): Environment, e.g. os.environ.

        Returns:
            Config: The settings.

        Raises:
            ValueError: A numeric variable is not an integer.
        """
        return cls(
            proc_root=Path(environ.get("DDS_WATCHDOG_PROC_ROOT", DEFAULT_PROC_ROOT)),
            cgroup_root=Path(environ.get("DDS_WATCHDOG_CGROUP_ROOT", DEFAULT_CGROUP_ROOT)),
            state_file=Path(environ.get("DDS_WATCHDOG_STATE_FILE", DEFAULT_STATE_FILE)),
            textfile=Path(environ.get("DDS_WATCHDOG_TEXTFILE", DEFAULT_TEXTFILE)),
            samples=int(environ.get("DDS_WATCHDOG_SAMPLES", DEFAULT_SAMPLES)),
            rx_min_bytes=int(environ.get("DDS_WATCHDOG_RX_MIN_BYTES", DEFAULT_RX_MIN_BYTES)),
            min_restart_interval_s=int(
                environ.get("DDS_WATCHDOG_MIN_RESTART_INTERVAL_S", DEFAULT_MIN_RESTART_INTERVAL_S)
            ),
            deploy_lock=Path(environ.get("DDS_WATCHDOG_DEPLOY_LOCK", DEFAULT_DEPLOY_LOCK)),
            deploy_lock_max_age_s=int(environ.get("DDS_WATCHDOG_DEPLOY_LOCK_MAX_AGE_S", DEFAULT_DEPLOY_LOCK_MAX_AGE_S)),
            dry_run=environ.get("DDS_WATCHDOG_DRY_RUN", "false").strip().lower() in TRUE_VALUES,
            systemctl=environ.get("SYSTEMCTL", DEFAULT_SYSTEMCTL),
        )


@dataclass
class UnitState:
    """Persistent per-unit state.

    Attributes:
        streak (int): Consecutive suspect samples.
        last_restart (float): Epoch seconds of the last successful watchdog restart (0 for never).
        last_attempt (float): Epoch seconds of the last failed restart attempt (0 for never); a failed attempt starts
            the rate-limit window like a success does, but is not counted in restarts_total.
        restarts_total (int): Successful watchdog restarts.
    """

    streak: int = 0
    last_restart: float = 0.0
    last_attempt: float = 0.0
    restarts_total: int = 0


STATE_FIELD_TYPES: dict[str, tuple[type, ...]] = {
    "streak": (int,),
    "last_restart": (int, float),
    "last_attempt": (int, float),
    "restarts_total": (int,),
}


def parse_unit_state(fields: object) -> UnitState:
    """Validate and convert one unit entry of the state file.

    Args:
        fields (object): Decoded JSON value of the entry; must be a dict of known fields with number values.

    Returns:
        UnitState: The state (missing fields take their defaults; ints are converted for the float fields).

    Raises:
        ValueError: The entry is not a dict, has an unknown field, or a field has the wrong type (bool counts as wrong).
    """
    if not isinstance(fields, dict):
        raise ValueError(f"unit entry is {type(fields).__name__}, not an object")
    values: dict[str, float] = {}
    for name, value in fields.items():
        types = STATE_FIELD_TYPES.get(name)
        if types is None:
            raise ValueError(f"unknown field {name!r}")
        if isinstance(value, bool) or not isinstance(value, types):
            raise ValueError(f"field {name!r} has wrong type {type(value).__name__}")
        values[name] = value
    return UnitState(**values)


def write_atomic(path: Path, text: str) -> None:
    """Replace a file atomically: write a temp file in the same directory, then rename it over the target.

    Args:
        path (Path): Target file.
        text (str): New content.
    """
    fd, tmp = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as handle:
            handle.write(text)
        os.chmod(tmp, 0o644)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


def load_state(path: Path) -> dict[str, UnitState]:
    """Read the state file.

    Args:
        path (Path): JSON state file.

    Returns:
        dict[str, UnitState]: Unit name -> state; empty (with a warning) when the file is missing, corrupt, or has
            fields of the wrong type.
    """
    try:
        raw = json.loads(path.read_text())
        if not isinstance(raw, dict):
            raise ValueError(f"top level is {type(raw).__name__}, not an object")
        return {unit: parse_unit_state(fields) for unit, fields in raw.items()}
    except FileNotFoundError:
        LOG.warning("no state file %s, starting empty", path)
    except (OSError, ValueError) as exc:
        LOG.warning("unreadable state file %s, starting empty: %s", path, exc)
    return {}


def save_state(path: Path, state: dict[str, UnitState]) -> None:
    """Write the state file atomically, creating its directory.

    Args:
        path (Path): JSON state file.
        state (dict[str, UnitState]): Unit name -> state.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    write_atomic(path, json.dumps({unit: asdict(unit_state) for unit, unit_state in state.items()}, indent=1))


def running_ros2_units(systemctl: str) -> list[str]:
    """List the running ros2-*.service units.

    Args:
        systemctl (str): systemctl binary.

    Returns:
        list[str]: Unit names (first column of `systemctl list-units`).

    Raises:
        OSError: The binary could not be run.
        subprocess.SubprocessError: It failed or timed out.
    """
    proc = subprocess.run(
        [systemctl, "list-units", UNIT_GLOB, "--state=running", "--no-legend", "--plain"],
        capture_output=True,
        text=True,
        check=True,
        timeout=SYSTEMCTL_TIMEOUT_S,
    )
    return [line.split()[0] for line in proc.stdout.splitlines() if line.strip()]


def deploy_lock_active(cfg: Config, now: float) -> bool:
    """Tell whether a deploy is running.

    Args:
        cfg (Config): Settings.
        now (float): Current epoch seconds.

    Returns:
        bool: True when the deploy lock exists and is younger than the max age.
    """
    try:
        return now - cfg.deploy_lock.stat().st_mtime < cfg.deploy_lock_max_age_s
    except OSError:
        return False


def restart_allowed(cfg: Config, unit_state: UnitState, now: float) -> bool:
    """Decide whether a unit whose streak reached the limit may be restarted now.

    Args:
        cfg (Config): Settings.
        unit_state (UnitState): State of the unit.
        now (float): Current epoch seconds.

    Returns:
        bool: True when no fresh deploy lock exists and the last restart attempt (successful or failed) is old enough.
    """
    if deploy_lock_active(cfg, now):
        LOG.info("deploy lock %s is fresh, not restarting", cfg.deploy_lock)
        return False
    last = max(unit_state.last_restart, unit_state.last_attempt)
    if now - last < cfg.min_restart_interval_s:
        LOG.info("restart attempted %.0f s ago, rate limit holds the restart", now - last)
        return False
    return True


def restart_unit(cfg: Config, unit: str) -> bool:
    """Restart a unit with systemctl.

    Args:
        cfg (Config): Settings.
        unit (str): Unit name.

    Returns:
        bool: True when systemctl succeeded.
    """
    try:
        subprocess.run(
            [cfg.systemctl, "restart", unit], capture_output=True, text=True, check=True, timeout=SYSTEMCTL_TIMEOUT_S
        )
    except (OSError, subprocess.SubprocessError) as exc:
        LOG.error("systemctl restart %s failed: %s", unit, exc)
        return False
    return True


def escape_label(value: str) -> str:
    """Escape a label value for the Prometheus text format.

    Args:
        value (str): Raw value.

    Returns:
        str: Value with backslash, double quote and newline escaped.
    """
    return value.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")


def metrics_text(now: float, units: list[str], state: dict[str, UnitState], suspects: set[str], up: bool = True) -> str:
    """Render the metrics in the Prometheus text exposition format.

    Args:
        now (float): Current epoch seconds.
        units (list[str]): Running ros2 units of this run.
        state (dict[str, UnitState]): State after the run.
        suspects (set[str]): Units that were a suspect in this sample.
        up (bool): False when the units could not be listed, so nothing was sampled.

    Returns:
        str: Metrics text, newline terminated.
    """
    lines = [
        "# HELP dds_watchdog_up 1 when the last watchdog run sampled the ros2 units, 0 when listing them failed.",
        "# TYPE dds_watchdog_up gauge",
        f"dds_watchdog_up {int(up)}",
        "# HELP dds_watchdog_last_run_timestamp_seconds Unix time of the last watchdog run.",
        "# TYPE dds_watchdog_last_run_timestamp_seconds gauge",
        f"dds_watchdog_last_run_timestamp_seconds {now}",
        "# HELP dds_watchdog_suspect 1 when the unit has a wedged DDS discovery thread in this sample.",
        "# TYPE dds_watchdog_suspect gauge",
    ]
    lines += [f'dds_watchdog_suspect{{unit="{escape_label(unit)}"}} {int(unit in suspects)}' for unit in units]
    lines += [
        "# HELP dds_watchdog_streak Consecutive suspect samples of the unit.",
        "# TYPE dds_watchdog_streak gauge",
    ]
    lines += [f'dds_watchdog_streak{{unit="{escape_label(unit)}"}} {state[unit].streak}' for unit in units]
    lines += [
        "# HELP dds_watchdog_restarts_total Restarts of the unit by the watchdog.",
        "# TYPE dds_watchdog_restarts_total counter",
    ]
    lines += [
        f'dds_watchdog_restarts_total{{unit="{escape_label(unit)}"}} {st.restarts_total}'
        for unit, st in sorted(state.items())
        if st.restarts_total
    ]
    return "\n".join(lines) + "\n"


def run_once(cfg: Config, now: float) -> int:
    """Run one cycle: sample the units, update streaks, restart wedged units, save the state, write the metrics.

    Args:
        cfg (Config): Settings.
        now (float): Current epoch seconds.

    Returns:
        int: Exit code (0, or 1 when the textfile could not be written).
    """
    state = load_state(cfg.state_file)
    try:
        units = running_ros2_units(cfg.systemctl)
    except (OSError, subprocess.SubprocessError) as exc:
        LOG.error("listing ros2 units failed: %s", exc)
        try:
            write_atomic(cfg.textfile, metrics_text(now, [], state, set(), up=False))
        except OSError as write_exc:
            LOG.error("cannot write textfile %s: %s", cfg.textfile, write_exc)
            return 1
        return 0
    state_keep: dict[str, UnitState] = {}
    queues = rx_queues(cfg.proc_root)
    suspects: set[str] = set()
    for unit in units:
        unit_state = state.get(unit, UnitState())
        evidence = scan_unit(cfg.proc_root, cfg.cgroup_root, unit, queues, cfg.rx_min_bytes)
        if evidence is None:
            unit_state.streak = 0
        else:
            suspects.add(unit)
            unit_state.streak += 1
            if unit_state.streak >= cfg.samples and restart_allowed(cfg, unit_state, now):
                why = (
                    f"{unit}: thread dds.udp.{evidence.port} (pid {evidence.pid}) in {evidence.wchan}, "
                    f"rx_queue {evidence.rx_queue} B for {unit_state.streak} samples"
                )
                if cfg.dry_run:
                    LOG.warning("would restart %s (dry run)", why)
                else:
                    LOG.warning("restarting %s", why)
                    if restart_unit(cfg, unit):
                        unit_state.restarts_total += 1
                        unit_state.last_restart = now
                        unit_state.streak = 0
                    else:
                        unit_state.last_attempt = now
        state_keep[unit] = unit_state
    for unit, unit_state in state.items():
        if unit not in state_keep and unit_state.restarts_total:
            state_keep[unit] = UnitState(last_restart=unit_state.last_restart, restarts_total=unit_state.restarts_total)
    try:
        save_state(cfg.state_file, state_keep)
    except OSError as exc:
        LOG.error("cannot write state file %s: %s", cfg.state_file, exc)
    try:
        write_atomic(cfg.textfile, metrics_text(now, units, state_keep, suspects))
    except OSError as exc:
        LOG.error("cannot write textfile %s: %s", cfg.textfile, exc)
        return 1
    return 0


def main() -> int:
    """Run one watchdog cycle with the settings from the environment.

    Returns:
        int: Exit code of run_once.
    """
    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s", stream=sys.stdout)
    return run_once(Config.from_env(os.environ), time.time())


if __name__ == "__main__":
    sys.exit(main())
