"""Propose CPUQuota / MemoryMax / MemoryHigh for the ros2-* systemd units from Prometheus history.

Reads scripts/propose_unit_limits.yaml (pydantic-validated), queries the Prometheus HTTP API for the per-unit cgroup
CPU and memory metrics (container_cpu_usage_seconds_total, container_memory_working_set_bytes) and compares them
with the limits configured in ansible/group_vars/client.yml. It prints a table (or YAML) and never edits any file.

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
CPU_QUOTA_STEP_PCT = 5
MEMORY_STEP_MIB = 16
MEMORY_MAX_HEADROOM = 1.2
MEMORY_HIGH_RATIO = 0.85
THROTTLE_FLAG_RATIO = 0.9
TEMPLATE_DEFAULT_CPU_QUOTA = "50%"
TEMPLATE_DEFAULT_MEMORY_MAX = "256M"
UNIT_PREFIX = "ros2-"
UNIT_SUFFIX = ".service"
NO_DATA = "no data"
BYTES_PER_MIB = 1024 * 1024
SIZE_UNITS_MIB = {"K": 1 / 1024, "M": 1, "G": 1024, "T": 1024 * 1024}
TABLE_COLUMNS = (
    "unit",
    "node_type",
    "CPUQuota",
    "p99 CPU %",
    "proposed CPUQuota",
    "MemoryMax",
    "p99 mem",
    "max mem",
    "proposed MemoryMax",
    "proposed MemoryHigh",
    "flag",
)


class Settings(BaseModel):
    """Validated contents of propose_unit_limits.yaml."""

    prometheus_url: str = "http://client.ros2.lan:9090"
    window: str = Field(default="48h", pattern=r"^\d+[smhdwy]$")
    percentile: float = Field(default=0.99, gt=0, le=1)
    cpu_margin: float = Field(default=1.5, gt=0)
    mem_margin: float = Field(default=1.5, gt=0)
    min_cpu_quota_pct: int = Field(default=10, ge=0)
    min_memory_mib: int = Field(default=64, ge=0)
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
    """Currently configured limits of one unit and the node type they come from."""

    node_type: str
    cpu_quota_pct: float
    memory_max_mib: float


class Queries(BaseModel):
    """The three PromQL expressions the script runs."""

    cpu_p: str
    mem_p: str
    mem_max: str


class Row(BaseModel):
    """One output line: observations, current limits and proposals for a unit."""

    unit: str
    node_type: str
    current_cpu_quota_pct: float | None = None
    p99_cpu_pct: float | None = None
    proposed_cpu_quota_pct: int | None = None
    current_memory_max_mib: float | None = None
    p99_mem_mib: float | None = None
    max_mem_mib: float | None = None
    proposed_memory_max_mib: int | None = None
    proposed_memory_high_mib: int | None = None
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


def propose_cpu_quota_pct(p99_cores: float, settings: Settings) -> int:
    """Propose a CPUQuota: p99 times margin, rounded up to 5 %, at least the minimum.

    Args:
        p99_cores (float): p99 CPU usage in cores.
        settings (Settings): Margins and minimum.

    Returns:
        int: Proposed CPUQuota in percent of one core.
    """
    proposed = round_up(p99_cores * 100 * settings.cpu_margin, CPU_QUOTA_STEP_PCT)
    return int(max(proposed, settings.min_cpu_quota_pct))


def propose_memory_max_mib(p99_bytes: float, max_bytes: float, settings: Settings) -> int:
    """Propose a MemoryMax: max(p99 * margin, max * 1.2) rounded up to 16 MiB, at least the minimum.

    Args:
        p99_bytes (float): p99 working set in bytes.
        max_bytes (float): Maximum working set in bytes.
        settings (Settings): Margin and minimum.

    Returns:
        int: Proposed MemoryMax in MiB.
    """
    wanted = max(p99_bytes * settings.mem_margin, max_bytes * MEMORY_MAX_HEADROOM) / BYTES_PER_MIB
    return int(max(round_up(wanted, MEMORY_STEP_MIB), settings.min_memory_mib))


def propose_memory_high_mib(memory_max_mib: float) -> int:
    """Propose MemoryHigh as about 85 % of the proposed MemoryMax.

    Args:
        memory_max_mib (float): Proposed MemoryMax in MiB.

    Returns:
        int: MemoryHigh in MiB (rounded down).
    """
    return int(memory_max_mib * MEMORY_HIGH_RATIO)


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
    """Map each present node to its unit and configured limits (node name -> node_type -> defaults).

    Args:
        path (Path): group_vars YAML file with ros2_nodes and ros2_node_type_defaults.

    Returns:
        dict[str, NodeLimits]: Unit name -> limits; the template defaults apply when a type sets none.
    """
    doc = yaml.safe_load(path.read_text())
    defaults = doc.get("ros2_node_type_defaults", {})
    limits: dict[str, NodeLimits] = {}
    for node in doc.get("ros2_nodes", []):
        if not node.get("present", True):
            continue
        type_defaults = defaults.get(node["node_type"], {})
        limits[f"{UNIT_PREFIX}{node['name']}{UNIT_SUFFIX}"] = NodeLimits(
            node_type=node["node_type"],
            cpu_quota_pct=parse_quota_pct(type_defaults.get("cpu_quota", TEMPLATE_DEFAULT_CPU_QUOTA)),
            memory_max_mib=parse_memory_mib(type_defaults.get("memory_max", TEMPLATE_DEFAULT_MEMORY_MAX)),
        )
    return limits


def build_queries(settings: Settings) -> Queries:
    """Build the PromQL for p99 CPU (cores), p99 and max memory working set.

    Args:
        settings (Settings): Window and percentile.

    Returns:
        Queries: The three expressions.
    """
    selector = f'id=~"{CGROUP_ID_REGEX}"'
    cpu_rate = f"sum by (id) (rate(container_cpu_usage_seconds_total{{{selector}}}[{CPU_RATE_WINDOW}]))"
    memory = f"container_memory_working_set_bytes{{{selector}}}"
    return Queries(
        cpu_p=f"quantile_over_time({settings.percentile}, {cpu_rate}[{settings.window}:{CPU_RATE_WINDOW}])",
        mem_p=f"quantile_over_time({settings.percentile}, {memory}[{settings.window}])",
        mem_max=f"max_over_time({memory}[{settings.window}])",
    )


def build_rows(
    settings: Settings,
    limits: dict[str, NodeLimits],
    cpu_p: dict[str, float],
    mem_p: dict[str, float],
    mem_max: dict[str, float],
) -> list[Row]:
    """Combine observations with configured limits into output rows.

    Args:
        settings (Settings): Margins and minimums.
        limits (dict[str, NodeLimits]): Configured limits per unit.
        cpu_p (dict[str, float]): p99 CPU in cores per unit.
        mem_p (dict[str, float]): p99 working set in bytes per unit.
        mem_max (dict[str, float]): Max working set in bytes per unit.

    Returns:
        list[Row]: One row per known unit (configured or observed), sorted by unit. Units lacking any of the three
        series get the flag "no data" and no proposal.
    """
    rows: list[Row] = []
    for unit in sorted(set(limits) | set(cpu_p) | set(mem_p) | set(mem_max)):
        current = limits.get(unit)
        row = Row(
            unit=unit,
            node_type=current.node_type if current else "unknown",
            current_cpu_quota_pct=current.cpu_quota_pct if current else None,
            current_memory_max_mib=current.memory_max_mib if current else None,
        )
        if unit not in cpu_p or unit not in mem_p or unit not in mem_max:
            row.flag = NO_DATA
            rows.append(row)
            continue
        row.p99_cpu_pct = cpu_p[unit] * 100
        row.p99_mem_mib = mem_p[unit] / BYTES_PER_MIB
        row.max_mem_mib = mem_max[unit] / BYTES_PER_MIB
        row.proposed_cpu_quota_pct = propose_cpu_quota_pct(cpu_p[unit], settings)
        row.proposed_memory_max_mib = propose_memory_max_mib(mem_p[unit], mem_max[unit], settings)
        row.proposed_memory_high_mib = propose_memory_high_mib(row.proposed_memory_max_mib)
        flags: list[str] = []
        if current and row.p99_cpu_pct >= current.cpu_quota_pct * THROTTLE_FLAG_RATIO:
            flags.append("throttle risk")
        if current and row.max_mem_mib > current.memory_max_mib:
            flags.append("oom risk")
        row.flag = ", ".join(flags)
        rows.append(row)
    return rows


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
    """Render rows as an aligned text table.

    Args:
        rows (list[Row]): Output rows.

    Returns:
        str: Table text with a header line.
    """
    lines = [list(TABLE_COLUMNS)]
    for r in rows:
        lines.append(
            [
                r.unit,
                r.node_type,
                fmt(r.current_cpu_quota_pct, "%"),
                fmt(r.p99_cpu_pct, "%", 1),
                fmt(r.proposed_cpu_quota_pct, "%"),
                fmt(r.current_memory_max_mib, "M"),
                fmt(r.p99_mem_mib, "M", 1),
                fmt(r.max_mem_mib, "M", 1),
                fmt(r.proposed_memory_max_mib, "M"),
                fmt(r.proposed_memory_high_mib, "M"),
                r.flag,
            ]
        )
    widths = [max(len(line[i]) for line in lines) for i in range(len(TABLE_COLUMNS))]
    return "\n".join("  ".join(cell.ljust(w) for cell, w in zip(line, widths, strict=True)).rstrip() for line in lines)


def render_yaml(rows: list[Row]) -> str:
    """Render the proposals as YAML, unit -> cpu_quota / memory_max / memory_high (units with data only).

    Args:
        rows (list[Row]): Output rows.

    Returns:
        str: YAML document.
    """
    doc = {
        r.unit: {
            "node_type": r.node_type,
            "cpu_quota": f"{r.proposed_cpu_quota_pct}%",
            "memory_max": f"{r.proposed_memory_max_mib}M",
            "memory_high": f"{r.proposed_memory_high_mib}M",
            "flag": r.flag,
        }
        for r in rows
        if r.proposed_cpu_quota_pct is not None
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
        cpu_p, mem_p, mem_max = (
            query_vector(client, settings.prometheus_url, q, settings.unit_regex)
            for q in (queries.cpu_p, queries.mem_p, queries.mem_max)
        )
    rows = build_rows(settings, limits, cpu_p, mem_p, mem_max)
    print(render_yaml(rows) if settings.format == "yaml" else render_table(rows))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
