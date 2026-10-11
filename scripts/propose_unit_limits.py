"""Propose MemoryLow / MemoryMax / CPUWeight / IOWeight for the ros2-* systemd units from Prometheus history.

The robot's cgroup model (ansible/README.md, "Node resource model") guarantees robot headroom instead of capping the
nodes: no CPUQuota and no MemoryHigh, priority from CPUWeight/IOWeight tiers, protection from MemoryLow at about the
typical working set, and MemoryMax only as a generous safety net (twice the peak).

Reads scripts/propose_unit_limits.yaml (pydantic-validated), queries the Prometheus HTTP API (the server's Mimir at
/prometheus, which stores the series of both hosts under a `host` label) for one host's per-unit cgroup
CPU and memory metrics (container_cpu_usage_seconds_total, container_memory_working_set_bytes) and compares them
with the values configured in ansible/group_vars/client.yml. It prints a table (or YAML) and never edits any file.

Usage: python scripts/propose_unit_limits.py [config.yaml]
"""

import math
import re
import sys
from pathlib import Path
from typing import Literal

import httpx
import yaml
from pydantic import BaseModel, Field, field_validator

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = Path(__file__).resolve().with_suffix(".yaml")
DEFAULT_GROUP_VARS = "ansible/group_vars/client.yml"
CGROUP_ID_REGEX = "/system.slice/ros2-.*.service"
CPU_RATE_WINDOW = "1m"
MEMORY_LOW_STEP_MIB = 8
MEMORY_MAX_STEP_MIB = 16
THROTTLE_FLAG_RATIO = 0.9
TEMPLATE_DEFAULT_MEMORY_MAX = "256M"
UNIT_PREFIX = "ros2-"
UNIT_SUFFIX = ".service"
NO_DATA = "no data"
BYTES_PER_MIB = 1024 * 1024
SIZE_UNITS_MIB = {"K": 1 / 1024, "M": 1, "G": 1024, "T": 1024 * 1024}
# systemd accepts CPUWeight / IOWeight 1..10000.
MIN_WEIGHT = 1
MAX_WEIGHT = 10000
TABLE_COLUMNS = (
    "unit",
    "node_type",
    "CPUWeight",
    "p99 CPU %",
    "MemoryLow",
    "typical mem",
    "max mem",
    "MemoryMax",
    "proposed MemoryLow",
    "proposed MemoryMax",
    "proposed CPUWeight",
    "proposed IOWeight",
    "flag",
)


class Settings(BaseModel):
    """Validated contents of propose_unit_limits.yaml."""

    prometheus_url: str = "http://server.ros2.lan:9009/prometheus"
    # Value of the `host` label (Alloy external label) whose units are proposed: client (robot) or server.
    host: str = Field(default="client", pattern=r"^[A-Za-z0-9_.-]+$")
    window: str = Field(default="24h", pattern=r"^\d+[smhdwy]$")
    # Percentile of the 1m CPU rate (reported, and checked against a CPUQuota where one is still configured).
    percentile: float = Field(default=0.99, gt=0, le=1)
    # Percentile of the working set taken as the typical working set that MemoryLow protects.
    typical_percentile: float = Field(default=0.5, gt=0, le=1)
    # MemoryMax is at least this many times the peak working set over the window.
    memory_max_factor: float = Field(default=2.0, ge=1)
    min_memory_low_mib: int = Field(default=32, ge=0)
    min_memory_max_mib: int = Field(default=64, ge=0)
    # Weights proposed for a unit whose node type sets none (the normal robot tier).
    default_cpu_weight: int = Field(default=500, ge=MIN_WEIGHT, le=MAX_WEIGHT)
    default_io_weight: int = Field(default=500, ge=MIN_WEIGHT, le=MAX_WEIGHT)
    unit_regex: str = r"^ros2-.*\.service$"
    format: Literal["table", "yaml"] = "table"
    group_vars_path: str = DEFAULT_GROUP_VARS
    timeout_s: float = Field(default=30, gt=0)

    @field_validator("unit_regex")
    @classmethod
    def regex_compiles(cls, value: str) -> str:
        """Reject an invalid unit regex early.

        Args:
            value (str): Regular expression.

        Returns:
            str: The same expression.
        """
        re.compile(value)
        return value


class NodeLimits(BaseModel):
    """Currently configured cgroup values of one unit and the node type they come from."""

    node_type: str
    cpu_quota_pct: float | None = None
    memory_max_mib: float
    memory_low_mib: float | None = None
    cpu_weight: int | None = None
    io_weight: int | None = None


class Queries(BaseModel):
    """The three PromQL expressions the script runs."""

    cpu_p: str
    mem_typical: str
    mem_max: str


class Row(BaseModel):
    """One output line: observations, current values and proposals for a unit."""

    unit: str
    node_type: str
    current_cpu_weight: int | None = None
    current_cpu_quota_pct: float | None = None
    current_memory_low_mib: float | None = None
    current_memory_max_mib: float | None = None
    p99_cpu_pct: float | None = None
    typical_mem_mib: float | None = None
    max_mem_mib: float | None = None
    proposed_memory_low_mib: int | None = None
    proposed_memory_max_mib: int | None = None
    proposed_cpu_weight: int | None = None
    proposed_io_weight: int | None = None
    flag: str = ""


def load_settings(path: Path) -> Settings:
    """Load and validate the config file.

    Args:
        path (Path): YAML config file.

    Returns:
        Settings: Validated settings.
    """
    return Settings(**(yaml.safe_load(path.read_text()) or {}))


def round_up(value: float, step: float) -> float:
    """Round a value up to a multiple of step (tolerant to float noise).

    Args:
        value (float): Value to round.
        step (float): Step size, > 0.

    Returns:
        float: Smallest multiple of step >= value.
    """
    return math.ceil(round(value / step, 6)) * step


def propose_memory_low_mib(typical_bytes: float, settings: Settings) -> int:
    """Propose a MemoryLow: the typical working set rounded up to 8 MiB, at least the minimum.

    Args:
        typical_bytes (float): Typical (typical_percentile) working set in bytes.
        settings (Settings): Minimum.

    Returns:
        int: Proposed MemoryLow in MiB.
    """
    return int(max(round_up(typical_bytes / BYTES_PER_MIB, MEMORY_LOW_STEP_MIB), settings.min_memory_low_mib))


def propose_memory_max_mib(max_bytes: float, settings: Settings, current_mib: float | None = None) -> int:
    """Propose a MemoryMax safety net: the peak times memory_max_factor rounded up to 16 MiB, never below the minimum
    or a larger current cap.

    Args:
        max_bytes (float): Peak working set in bytes.
        settings (Settings): Factor and minimum.
        current_mib (float | None): Currently configured MemoryMax in MiB, kept when it is larger.

    Returns:
        int: Proposed MemoryMax in MiB.
    """
    wanted = round_up(max_bytes * settings.memory_max_factor / BYTES_PER_MIB, MEMORY_MAX_STEP_MIB)
    return int(max(wanted, settings.min_memory_max_mib, current_mib or 0))


def propose_weights(current: NodeLimits | None, settings: Settings) -> tuple[int, int]:
    """Propose CPUWeight / IOWeight: the node type's configured tier, else the default (normal robot tier).

    Args:
        current (NodeLimits | None): Configured values of the unit, None for an unknown unit.
        settings (Settings): Default weights.

    Returns:
        tuple[int, int]: (CPUWeight, IOWeight).
    """
    cpu = current.cpu_weight if current and current.cpu_weight is not None else settings.default_cpu_weight
    io = current.io_weight if current and current.io_weight is not None else settings.default_io_weight
    return cpu, io


def parse_quota_pct(text: str) -> float:
    """Parse a systemd CPUQuota such as "50%".

    Args:
        text (str): CPUQuota string.

    Returns:
        float: Percent of one core.
    """
    return float(str(text).strip().rstrip("%"))


def parse_memory_mib(text: str) -> float:
    """Parse a systemd memory size such as "128M" or "1G".

    Args:
        text (str): Size with an optional K/M/G/T suffix (bytes when absent).

    Returns:
        float: Size in MiB.
    """
    match = re.fullmatch(r"\s*([\d.]+)\s*([KMGT]?)B?\s*", str(text).upper())
    if match is None:
        raise ValueError(f"unparseable memory size: {text!r}")
    number, suffix = match.groups()
    return float(number) * (SIZE_UNITS_MIB[suffix] if suffix else 1 / BYTES_PER_MIB)


def parse_vector(response: dict, unit_regex: str) -> dict[str, float]:
    """Turn a Prometheus instant-vector response into unit -> value.

    Args:
        response (dict): JSON of /api/v1/query.
        unit_regex (str): Only units whose name matches are kept.

    Returns:
        dict[str, float]: Unit name (e.g. ros2-web_ui.service) -> sample value; empty on an error response.
    """
    if response.get("status") != "success":
        return {}
    pattern = re.compile(unit_regex)
    values: dict[str, float] = {}
    for item in response["data"]["result"]:
        unit = item["metric"].get("id", "").rsplit("/", 1)[-1]
        if pattern.search(unit):
            values[unit] = float(item["value"][1])
    return values


def load_node_limits(path: Path) -> dict[str, NodeLimits]:
    """Map each present node to its unit and configured cgroup values (node name -> node_type -> defaults).

    Args:
        path (Path): group_vars YAML file with ros2_nodes and ros2_node_type_defaults.

    Returns:
        dict[str, NodeLimits]: Unit name -> values; unset keys are None (the template renders no directive) except
        MemoryMax, which falls back to the template default 256M.
    """
    doc = yaml.safe_load(path.read_text())
    defaults = doc.get("ros2_node_type_defaults", {})
    limits: dict[str, NodeLimits] = {}
    for node in doc.get("ros2_nodes", []):
        if not node.get("present", True):
            continue
        type_defaults = defaults.get(node["node_type"]) or {}
        quota = type_defaults.get("cpu_quota")
        low = type_defaults.get("memory_low")
        limits[f"{UNIT_PREFIX}{node['name']}{UNIT_SUFFIX}"] = NodeLimits(
            node_type=node["node_type"],
            cpu_quota_pct=parse_quota_pct(quota) if quota else None,
            memory_max_mib=parse_memory_mib(type_defaults.get("memory_max", TEMPLATE_DEFAULT_MEMORY_MAX)),
            memory_low_mib=parse_memory_mib(low) if low else None,
            cpu_weight=type_defaults.get("cpu_weight"),
            io_weight=type_defaults.get("io_weight"),
        )
    return limits


def build_queries(settings: Settings) -> Queries:
    """Build the PromQL for p99 CPU (cores), the typical and the peak working set of one host's units.

    Args:
        settings (Settings): Window, percentiles and host label value.

    Returns:
        Queries: The three expressions.
    """
    selector = f'host="{settings.host}", id=~"{CGROUP_ID_REGEX}"'
    cpu_rate = f"sum by (id) (rate(container_cpu_usage_seconds_total{{{selector}}}[{CPU_RATE_WINDOW}]))"
    memory = f"container_memory_working_set_bytes{{{selector}}}"
    return Queries(
        cpu_p=f"quantile_over_time({settings.percentile}, {cpu_rate}[{settings.window}:{CPU_RATE_WINDOW}])",
        mem_typical=f"quantile_over_time({settings.typical_percentile}, {memory}[{settings.window}])",
        mem_max=f"max_over_time({memory}[{settings.window}])",
    )


def build_rows(
    settings: Settings,
    limits: dict[str, NodeLimits],
    cpu_p: dict[str, float],
    mem_typical: dict[str, float],
    mem_max: dict[str, float],
) -> list[Row]:
    """Combine observations with configured values into output rows.

    Args:
        settings (Settings): Factors, minimums and default weights.
        limits (dict[str, NodeLimits]): Configured values per unit.
        cpu_p (dict[str, float]): p99 CPU in cores per unit.
        mem_typical (dict[str, float]): Typical working set in bytes per unit.
        mem_max (dict[str, float]): Peak working set in bytes per unit.

    Returns:
        list[Row]: One row per known unit (configured or observed), sorted by unit. Units lacking any of the three
        series get the flag "no data" and no proposal. Flags: "oom risk" (peak above MemoryMax), "tight max"
        (MemoryMax below memory_max_factor x peak), "unprotected" (no MemoryLow or one below the typical working set),
        "throttle risk" (a CPUQuota is still set and p99 CPU reaches 90 % of it).
    """
    rows: list[Row] = []
    for unit in sorted(set(limits) | set(cpu_p) | set(mem_typical) | set(mem_max)):
        current = limits.get(unit)
        row = Row(
            unit=unit,
            node_type=current.node_type if current else "unknown",
            current_cpu_weight=current.cpu_weight if current else None,
            current_cpu_quota_pct=current.cpu_quota_pct if current else None,
            current_memory_low_mib=current.memory_low_mib if current else None,
            current_memory_max_mib=current.memory_max_mib if current else None,
        )
        if unit not in cpu_p or unit not in mem_typical or unit not in mem_max:
            row.flag = NO_DATA
            rows.append(row)
            continue
        row.p99_cpu_pct = cpu_p[unit] * 100
        row.typical_mem_mib = mem_typical[unit] / BYTES_PER_MIB
        row.max_mem_mib = mem_max[unit] / BYTES_PER_MIB
        row.proposed_memory_low_mib = propose_memory_low_mib(mem_typical[unit], settings)
        row.proposed_memory_max_mib = propose_memory_max_mib(
            mem_max[unit], settings, current.memory_max_mib if current else None
        )
        row.proposed_cpu_weight, row.proposed_io_weight = propose_weights(current, settings)
        flags: list[str] = []
        if current and row.max_mem_mib > current.memory_max_mib:
            flags.append("oom risk")
        if current and current.memory_max_mib < row.max_mem_mib * settings.memory_max_factor:
            flags.append("tight max")
        if current and (current.memory_low_mib is None or current.memory_low_mib < row.typical_mem_mib):
            flags.append("unprotected")
        if current and current.cpu_quota_pct and row.p99_cpu_pct >= current.cpu_quota_pct * THROTTLE_FLAG_RATIO:
            flags.append("throttle risk")
        row.flag = ", ".join(flags)
        rows.append(row)
    return rows


def memory_low_sum_mib(rows: list[Row]) -> int:
    """Sum the proposed MemoryLow over the rows that have a proposal (what system.slice must allocate).

    Args:
        rows (list[Row]): Output rows.

    Returns:
        int: Sum in MiB.
    """
    return sum(r.proposed_memory_low_mib for r in rows if r.proposed_memory_low_mib is not None)


def fmt(value: float | None, suffix: str = "", digits: int = 0) -> str:
    """Format an optional number for the table.

    Args:
        value (float | None): Number or None.
        suffix (str): Unit suffix.
        digits (int): Decimal places.

    Returns:
        str: Text, "-" for None.
    """
    return "-" if value is None else f"{value:.{digits}f}{suffix}"


def render_table(rows: list[Row]) -> str:
    """Render rows as an aligned text table, followed by the sum of the proposed MemoryLow.

    Args:
        rows (list[Row]): Output rows.

    Returns:
        str: Table text with a header line and a closing sum line.
    """
    lines = [list(TABLE_COLUMNS)]
    for r in rows:
        lines.append(
            [
                r.unit,
                r.node_type,
                fmt(r.current_cpu_weight),
                fmt(r.p99_cpu_pct, "%", 1),
                fmt(r.current_memory_low_mib, "M"),
                fmt(r.typical_mem_mib, "M", 1),
                fmt(r.max_mem_mib, "M", 1),
                fmt(r.current_memory_max_mib, "M"),
                fmt(r.proposed_memory_low_mib, "M"),
                fmt(r.proposed_memory_max_mib, "M"),
                fmt(r.proposed_cpu_weight),
                fmt(r.proposed_io_weight),
                r.flag,
            ]
        )
    widths = [max(len(line[i]) for line in lines) for i in range(len(TABLE_COLUMNS))]
    table = ["  ".join(cell.ljust(w) for cell, w in zip(line, widths, strict=True)).rstrip() for line in lines]
    table.append(f"sum of proposed MemoryLow: {memory_low_sum_mib(rows)}M")
    return "\n".join(table)


def render_yaml(rows: list[Row]) -> str:
    """Render the proposals as YAML, unit -> memory_low / memory_max / cpu_weight / io_weight (units with data only).

    Args:
        rows (list[Row]): Output rows.

    Returns:
        str: YAML document.
    """
    doc = {
        r.unit: {
            "node_type": r.node_type,
            "memory_low": f"{r.proposed_memory_low_mib}M",
            "memory_max": f"{r.proposed_memory_max_mib}M",
            "cpu_weight": r.proposed_cpu_weight,
            "io_weight": r.proposed_io_weight,
            "flag": r.flag,
        }
        for r in rows
        if r.proposed_memory_low_mib is not None
    }
    return yaml.safe_dump(doc, sort_keys=False)


def query_vector(client: httpx.Client, base_url: str, promql: str, unit_regex: str) -> dict[str, float]:
    """Run one instant query against Prometheus.

    Args:
        client (httpx.Client): HTTP client (carries the timeout).
        base_url (str): Prometheus base URL.
        promql (str): Expression.
        unit_regex (str): Unit filter.

    Returns:
        dict[str, float]: Unit -> value.
    """
    response = client.get(f"{base_url.rstrip('/')}/api/v1/query", params={"query": promql})
    response.raise_for_status()
    return parse_vector(response.json(), unit_regex)


def main(argv: list[str]) -> int:
    """Query Prometheus and print the proposals in the configured format.

    Args:
        argv (list[str]): Optional single argument: config file path (default propose_unit_limits.yaml).

    Returns:
        int: Process exit code.
    """
    settings = load_settings(Path(argv[0]) if argv else DEFAULT_CONFIG)
    queries = build_queries(settings)
    limits = load_node_limits(REPO_ROOT / settings.group_vars_path)
    with httpx.Client(timeout=settings.timeout_s) as client:
        cpu_p, mem_typical, mem_max = (
            query_vector(client, settings.prometheus_url, q, settings.unit_regex)
            for q in (queries.cpu_p, queries.mem_typical, queries.mem_max)
        )
    rows = build_rows(settings, limits, cpu_p, mem_typical, mem_max)
    print(render_yaml(rows) if settings.format == "yaml" else render_table(rows))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
