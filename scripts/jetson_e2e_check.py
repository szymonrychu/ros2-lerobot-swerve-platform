"""End-to-end check of the two-board depth system: one command, PASS or FAIL.

Over ssh (BatchMode) it reads the bridge's 127.0.0.1:19114/metrics on the client twice, window_s apart, the Jetson's
/health (curl from the client to 10.77.0.2:8090) and `chronyc -n tracking` on the Jetson, then checks:
Jetson state ok or uncalibrated; chrony reference 10.77.0.1 with |offset| < 1 ms; rgb rate >= 10 Hz and p95 latency
<= 150 ms (from histogram bucket deltas); when calibrated, depth rate >= 10 Hz, depth p95 <= 150 ms and fused >= 2 Hz
(when mono is ok); when uncalibrated, zero depth and fused publishes (no placeholder; the summary says
`depth pending USER STEP calibration`); bridge process CPU mean <= 0.3 core; the retired Pi units absent.

Config: scripts/jetson_e2e_check.yaml (pydantic). Exit 0 PASS, 1 FAIL, 2 could not collect. Read-only.

Usage (from the repo root): uv run python -m scripts.jetson_e2e_check [config.yaml]
"""

import json
import logging
import math
import re
import subprocess
import sys
import time
from collections.abc import Callable
from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field

DEFAULT_CONFIG = Path(__file__).resolve().with_suffix(".yaml")
PENDING_MESSAGE = "depth pending USER STEP calibration"
EXIT_PASS, EXIT_FAIL, EXIT_ERROR = 0, 1, 2
SSH_TIMEOUT_S = 30
LATENCY_METRIC = "jetson_bridge_latency_seconds"
FRAMES_METRIC = "jetson_bridge_frames_total"
CPU_METRIC = "process_cpu_seconds_total"
OK_STATES = ("ok", "uncalibrated")
P95 = 0.95

logger = logging.getLogger("jetson_e2e_check")

Runner = Callable[[str, str], str]


class Settings(BaseModel):
    """Validated contents of jetson_e2e_check.yaml."""

    model_config = ConfigDict(extra="forbid")

    client_host: str = "client"
    jetson_host: str = "jetson"
    jetson_ip: str = "10.77.0.2"
    jetson_http_port: int = 8090
    chrony_reference: str = "10.77.0.1"
    metrics_port: int = 19114
    window_s: float = Field(default=30, gt=0)
    max_offset_ms: float = Field(default=1.0, gt=0)
    min_rgb_hz: float = 10
    max_rgb_p95_ms: float = 150
    min_depth_hz: float = 10
    max_depth_p95_ms: float = 150
    min_fused_hz: float = 2
    max_bridge_cpu_cores: float = Field(default=0.3, gt=0)
    retired_units: list[str] = ["ros2-overview_camera", "ros2-stereo_depth", "ros2-stereo_depth_cpp"]


class Sample(BaseModel):
    """One exposition line."""

    name: str
    labels: dict[str, str]
    value: float


class Snapshot(BaseModel):
    """What one /metrics read contributes: frame counters, cumulative latency buckets per kind, process CPU."""

    frames: dict[str, float]
    buckets: dict[str, dict[float, float]]
    cpu_seconds: float


class Chrony(BaseModel):
    """Parsed `chronyc -n tracking`; None fields mean the line was absent."""

    reference: str | None
    offset_s: float | None


class Observation(BaseModel):
    """Everything collected for one verdict."""

    health: dict
    chrony: Chrony
    first: Snapshot
    second: Snapshot
    window_s: float
    unit_states: dict[str, str]


class Check(BaseModel):
    """One verdict line."""

    name: str
    ok: bool
    detail: str


class Report(BaseModel):
    """All checks plus the one-line summary."""

    checks: list[Check]
    summary: str

    @property
    def passed(self) -> bool:
        """True when every check passed."""
        return all(c.ok for c in self.checks)


_LINE = re.compile(r"^([A-Za-z_:][A-Za-z0-9_:]*)(?:\{(.*)\})?\s+(\S+)")
_LABEL = re.compile(r'([A-Za-z_][A-Za-z0-9_]*)="((?:[^"\\]|\\.)*)"')


def parse_metrics(text: str) -> list[Sample]:
    """Parse Prometheus exposition text; comments and unparsable lines are skipped.

    Args:
        text (str): Body of a /metrics response.

    Returns:
        list[Sample]: Samples in order.
    """
    samples = []
    for line in text.splitlines():
        match = _LINE.match(line.strip())
        if not match or line.startswith("#"):
            continue
        try:
            value = float(match.group(3))
        except ValueError:
            continue
        labels = dict(_LABEL.findall(match.group(2) or ""))
        samples.append(Sample(name=match.group(1), labels=labels, value=value))
    return samples


def snapshot_from_text(text: str) -> Snapshot:
    """Extract the series the check needs.

    Args:
        text (str): Body of a /metrics response.

    Returns:
        Snapshot: Counters, buckets and CPU.

    Raises:
        ValueError: The process CPU series is missing.
    """
    frames: dict[str, float] = {}
    buckets: dict[str, dict[float, float]] = {}
    cpu = None
    for s in parse_metrics(text):
        if s.name == FRAMES_METRIC and "kind" in s.labels:
            frames[s.labels["kind"]] = s.value
        elif s.name == f"{LATENCY_METRIC}_bucket" and "kind" in s.labels and "le" in s.labels:
            buckets.setdefault(s.labels["kind"], {})[float(s.labels["le"])] = s.value
        elif s.name == CPU_METRIC:
            cpu = s.value
    if cpu is None:
        raise ValueError(f"{CPU_METRIC} missing from the bridge metrics")
    return Snapshot(frames=frames, buckets=buckets, cpu_seconds=cpu)


def rate(first: float, second: float, window_s: float) -> float:
    """Per-second rate of a counter between two reads.

    Args:
        first (float): Counter at the first read.
        second (float): Counter at the second read.
        window_s (float): Seconds between the reads.

    Returns:
        float: Events per second.

    Raises:
        ValueError: The window is not positive or the counter went backwards (restart).
    """
    if window_s <= 0:
        raise ValueError("window must be positive")
    if second < first:
        raise ValueError("counter went backwards (service restarted during the window)")
    return (second - first) / window_s


def p95_from_buckets(buckets: dict[float, float]) -> float | None:
    """95th percentile from cumulative histogram buckets, interpolated linearly like Prometheus.

    Args:
        buckets (dict[float, float]): Upper bound (inf for +Inf) to cumulative count.

    Returns:
        float | None: Seconds; inf when it falls in the +Inf bucket; None when there are no observations.
    """
    bounds = sorted(buckets)
    total = buckets[bounds[-1]] if bounds else 0.0
    if total <= 0:
        return None
    target = P95 * total
    lower, below = 0.0, 0.0
    for bound in bounds:
        count = buckets[bound]
        if count >= target:
            if math.isinf(bound):
                return math.inf
            if count == below:
                return bound
            return lower + (bound - lower) * (target - below) / (count - below)
        lower, below = bound, count
    return math.inf


def window_p95(first: Snapshot, second: Snapshot, kind: str) -> float | None:
    """p95 latency of only the observations made between two reads.

    Args:
        first (Snapshot): Earlier read.
        second (Snapshot): Later read.
        kind (str): depth, fused or rgb.

    Returns:
        float | None: Seconds, or None without observations in the window.
    """
    later = second.buckets.get(kind, {})
    earlier = first.buckets.get(kind, {})
    return p95_from_buckets({le: count - earlier.get(le, 0.0) for le, count in later.items()})


_REF = re.compile(r"Reference ID\s*:\s*\S+\s*\(([^)]*)\)")
_SYS = re.compile(r"System time\s*:\s*([0-9.eE+-]+)\s+seconds\s+(fast|slow)")


def parse_chrony(text: str) -> Chrony:
    """Parse `chronyc -n tracking` for the reference address and the absolute system-time offset.

    Args:
        text (str): Command output.

    Returns:
        Chrony: Fields are None when their line is absent.
    """
    ref = _REF.search(text)
    sys_time = _SYS.search(text)
    return Chrony(
        reference=ref.group(1).strip() if ref else None,
        offset_s=abs(float(sys_time.group(1))) if sys_time else None,
    )


def parse_unit_states(text: str) -> dict[str, str]:
    """Parse lines `<unit>.service <LoadState>`.

    Args:
        text (str): Output of the unit probe.

    Returns:
        dict[str, str]: Unit name without .service to its LoadState.
    """
    states = {}
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 2:  # noqa: PLR2004
            states[parts[0].removesuffix(".service")] = parts[1]
    return states


def _fmt_ms(seconds: float | None) -> str:
    if seconds is None:
        return "n/a"
    return "inf" if math.isinf(seconds) else f"{seconds * 1000:.1f} ms"


def _rate_check(name: str, hz: float, minimum: float) -> Check:
    return Check(name=name, ok=hz >= minimum, detail=f"{hz:.2f} Hz (need >= {minimum:g})")


def _latency_check(name: str, p95: float | None, max_ms: float) -> Check:
    ok = p95 is not None and p95 * 1000 <= max_ms
    return Check(name=name, ok=ok, detail=f"p95 {_fmt_ms(p95)} (need <= {max_ms:g} ms)")


def _stream_checks(obs: Observation, cfg: Settings, kind: str, min_hz: float, max_ms: float) -> list[Check]:
    hz = rate(obs.first.frames.get(kind, 0.0), obs.second.frames.get(kind, 0.0), obs.window_s)
    return [
        _rate_check(f"{kind}_rate", hz, min_hz),
        _latency_check(f"{kind}_latency", window_p95(obs.first, obs.second, kind), max_ms),
    ]


def evaluate(obs: Observation, cfg: Settings) -> Report:
    """Turn an observation into the verdict.

    Args:
        obs (Observation): Collected data.
        cfg (Settings): Thresholds.

    Returns:
        Report: Checks and summary.
    """
    checks = []
    state = str(obs.health.get("state"))
    checks.append(Check(name="jetson_health", ok=state in OK_STATES, detail=f"state {state}"))

    c = obs.chrony
    offset_ok = c.offset_s is not None and c.offset_s * 1000 < cfg.max_offset_ms
    offset_txt = "n/a" if c.offset_s is None else f"{c.offset_s * 1000:.3f} ms"
    checks.append(
        Check(
            name="chrony",
            ok=c.reference == cfg.chrony_reference and offset_ok,
            detail=(
                f"reference {c.reference}, offset {offset_txt} "
                f"(need {cfg.chrony_reference}, < {cfg.max_offset_ms:g} ms)"
            ),
        )
    )

    checks += _stream_checks(obs, cfg, "rgb", cfg.min_rgb_hz, cfg.max_rgb_p95_ms)

    summary_extra = ""
    depth_hz = rate(obs.first.frames.get("depth", 0.0), obs.second.frames.get("depth", 0.0), obs.window_s)
    fused_hz = rate(obs.first.frames.get("fused", 0.0), obs.second.frames.get("fused", 0.0), obs.window_s)
    if state == "uncalibrated":
        none = depth_hz == 0.0 and fused_hz == 0.0
        checks.append(
            Check(name="no_placeholder", ok=none, detail=f"depth {depth_hz:.2f} Hz, fused {fused_hz:.2f} Hz (need 0)")
        )
        summary_extra = f"; {PENDING_MESSAGE}"
    elif state == "ok":
        checks += _stream_checks(obs, cfg, "depth", cfg.min_depth_hz, cfg.max_depth_p95_ms)
        if obs.health.get("mono_state") == "ok":
            checks.append(_rate_check("fused_rate", fused_hz, cfg.min_fused_hz))

    cpu = (obs.second.cpu_seconds - obs.first.cpu_seconds) / obs.window_s
    checks.append(
        Check(
            name="bridge_cpu",
            ok=cpu <= cfg.max_bridge_cpu_cores,
            detail=f"{cpu:.3f} core (need <= {cfg.max_bridge_cpu_cores:g})",
        )
    )

    present = sorted(u for u in cfg.retired_units if obs.unit_states.get(u) != "not-found")
    checks.append(
        Check(
            name="retired_units", ok=not present, detail=("present: " + ", ".join(present)) if present else "all absent"
        )
    )

    report = Report(checks=checks, summary="")
    verdict = "PASS" if report.passed else "FAIL: " + ", ".join(ch.name for ch in checks if not ch.ok)
    report.summary = verdict + summary_extra
    return report


def load_settings(path: Path) -> Settings:
    """Load and validate the config file.

    Args:
        path (Path): YAML file.

    Returns:
        Settings: Validated settings.
    """
    return Settings.model_validate(yaml.safe_load(path.read_text()) or {})


def ssh_run(host: str, command: str) -> str:
    """Run a command over non-interactive ssh.

    Args:
        host (str): ssh alias.
        command (str): Remote shell command.

    Returns:
        str: stdout.

    Raises:
        RuntimeError: Non-zero exit or timeout.
    """
    argv = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", host, command]
    try:
        proc = subprocess.run(argv, capture_output=True, text=True, timeout=SSH_TIMEOUT_S, check=False)  # noqa: S603
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f"ssh {host} timed out: {command}") from exc
    if proc.returncode != 0:
        raise RuntimeError(f"ssh {host} failed ({proc.returncode}): {command}: {proc.stderr.strip()}")
    return proc.stdout


def collect(
    cfg: Settings,
    run: Runner = ssh_run,
    sleep: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
) -> Observation:
    """Gather all inputs: metrics twice (window apart), health, chrony, unit states.

    Args:
        cfg (Settings): Hosts and window.
        run (Runner): (host, command) -> stdout; injectable for tests.
        sleep (Callable[[float], None]): Sleep function; injectable for tests.
        clock (Callable[[], float]): Monotonic clock; injectable for tests.

    Returns:
        Observation: The data for evaluate(); window_s is the measured time between the two metrics reads.
    """
    metrics_cmd = f"curl -sf --max-time 5 http://127.0.0.1:{cfg.metrics_port}/metrics"
    health_cmd = f"curl -sf --max-time 5 http://{cfg.jetson_ip}:{cfg.jetson_http_port}/health"
    units = " ".join(f"{u}.service" for u in cfg.retired_units)
    units_cmd = f'for u in {units}; do echo "$u $(systemctl show -p LoadState --value $u)"; done'
    first = snapshot_from_text(run(cfg.client_host, metrics_cmd))
    t_first = clock()
    health = json.loads(run(cfg.client_host, health_cmd))
    chrony = parse_chrony(run(cfg.jetson_host, "chronyc -n tracking"))
    unit_states = parse_unit_states(run(cfg.client_host, units_cmd))
    sleep(max(0.0, cfg.window_s - (clock() - t_first)))
    second = snapshot_from_text(run(cfg.client_host, metrics_cmd))
    window = clock() - t_first
    return Observation(
        health=health, chrony=chrony, first=first, second=second, window_s=window, unit_states=unit_states
    )


def main(argv: list[str] | None = None) -> int:
    """Run the check and print the verdict.

    Args:
        argv (list[str] | None): Optional config path as the only argument.

    Returns:
        int: 0 PASS, 1 FAIL, 2 collection error.
    """
    args = sys.argv[1:] if argv is None else argv
    cfg = load_settings(Path(args[0]) if args else DEFAULT_CONFIG)
    try:
        report = evaluate(collect(cfg, ssh_run, time.sleep, time.monotonic), cfg)
    except (RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}")
        return EXIT_ERROR
    for check in report.checks:
        print(f"{'ok  ' if check.ok else 'FAIL'} {check.name}: {check.detail}")
    print(report.summary)
    return EXIT_PASS if report.passed else EXIT_FAIL


if __name__ == "__main__":
    sys.exit(main())
