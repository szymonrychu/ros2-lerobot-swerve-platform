"""Per-node CPU before/after table from Prometheus/Mimir, for the C++ cut-over and the cheap Python fixes.

Reads scripts/node_cpu_compare.yaml (pydantic-validated). For each entry {name, before_unit, after_unit, switch_time}
it queries rate(container_cpu_usage_seconds_total[1m]) of /system.slice/<before_unit> over [switch - window, switch]
and of /system.slice/<after_unit> over [switch + settle, switch + settle + window], and prints the mean and p95 in
cores plus a total row (the sum of the rows' statistics). The query, window and statistics functions are those of
scripts/mppi_cpu_compare.py. An empty series is an error (exit 2), never a result. Read-only.

Usage (from the repo root): uv run python -m scripts.node_cpu_compare [config.yaml]
"""

import logging
import sys
from pathlib import Path

import httpx
import yaml
from pydantic import BaseModel, Field, field_validator

from scripts.mppi_cpu_compare import (
    EXIT_ERROR,
    HTTP_TIMEOUT_S,
    NoDataError,
    Stats,
    query_window,
    summarize,
    window_bounds,
)
from scripts.mppi_cpu_compare import Settings as WindowSettings

DEFAULT_CONFIG = Path(__file__).resolve().with_suffix(".yaml")
TOTAL_NAME = "total"

logger = logging.getLogger("node_cpu_compare")

__all__ = ["EXIT_ERROR", "NoDataError", "query_window", "summarize", "window_bounds"]


class NodeEntry(BaseModel):
    """One node: the unit measured before the switch, the unit measured after it, and the switch epoch."""

    name: str
    before_unit: str
    after_unit: str
    switch_time: float

    @field_validator("switch_time")
    @classmethod
    def switch_time_is_positive(cls, value: float) -> float:
        """Reject a non-positive epoch.

        Args:
            value (float): Epoch seconds.

        Returns:
            float: The same value.
        """
        if value <= 0:
            raise ValueError("switch_time must be a positive epoch in seconds")
        return value


class Settings(BaseModel):
    """Validated contents of node_cpu_compare.yaml."""

    prometheus_url: str = "http://server.ros2.lan:9009/prometheus"
    host: str = "client"
    window_h: float = Field(default=6, gt=0)
    settle_min: float = Field(default=10, ge=0)
    entries: list[NodeEntry] = Field(min_length=1)


class Row(BaseModel):
    """Mean and p95 cores of one node before and after."""

    name: str
    before: Stats
    after: Stats


def load_settings(path: Path) -> Settings:
    """Load and validate the config file.

    Args:
        path (Path): YAML config file.

    Returns:
        Settings: Validated settings.
    """
    return Settings(**(yaml.safe_load(path.read_text()) or {}))


def window_settings(settings: Settings, unit: str, switch: float) -> WindowSettings:
    """Settings of mppi_cpu_compare for one unit, so its query and window functions apply unchanged.

    Args:
        settings (Settings): Host, Prometheus URL and window lengths.
        unit (str): systemd unit to query.
        switch (float): Switch time, epoch seconds.

    Returns:
        WindowSettings: Settings of the unit with a numeric switch time.
    """
    return WindowSettings(
        prometheus_url=settings.prometheus_url,
        host=settings.host,
        unit=unit,
        window_h=settings.window_h,
        settle_min=settings.settle_min,
        switch_time=switch,
    )


def compare_entry(client: httpx.Client, settings: Settings, entry: NodeEntry) -> Row:
    """Query one node's before window (before_unit) and after window (after_unit).

    Args:
        client (httpx.Client): HTTP client.
        settings (Settings): Config.
        entry (NodeEntry): The node.

    Returns:
        Row: Statistics before and after.

    Raises:
        NoDataError: A window has no samples.
    """
    before_settings = window_settings(settings, entry.before_unit, entry.switch_time)
    after_settings = window_settings(settings, entry.after_unit, entry.switch_time)
    (b_start, b_end), _ = window_bounds(before_settings, entry.switch_time)
    _, (a_start, a_end) = window_bounds(after_settings, entry.switch_time)
    before = summarize(query_window(client, before_settings, b_start, b_end))
    after = summarize(query_window(client, after_settings, a_start, a_end))
    return Row(name=entry.name, before=before, after=after)


def compare_all(client: httpx.Client, settings: Settings) -> list[Row]:
    """Compare every entry and append the total row.

    Args:
        client (httpx.Client): HTTP client.
        settings (Settings): Config.

    Returns:
        list[Row]: One row per entry, then the total (sum of the rows' mean and p95).

    Raises:
        NoDataError: Any window has no samples.
    """
    rows = [compare_entry(client, settings, entry) for entry in settings.entries]
    total = Row(
        name=TOTAL_NAME,
        before=Stats(mean=sum(r.before.mean for r in rows), p95=sum(r.before.p95 for r in rows)),
        after=Stats(mean=sum(r.after.mean for r in rows), p95=sum(r.after.p95 for r in rows)),
    )
    return [*rows, total]


def render_table(rows: list[Row]) -> str:
    """Format the rows as a table of cores.

    Args:
        rows (list[Row]): Rows from compare_all.

    Returns:
        str: Multi-line text.
    """
    width = max(len(r.name) for r in rows)
    lines = [f"{'node':<{width}}{'mean_before':>13}{'mean_after':>12}{'p95_before':>12}{'p95_after':>11}"]
    for row in rows:
        lines.append(
            f"{row.name:<{width}}{row.before.mean:>13.4f}{row.after.mean:>12.4f}"
            f"{row.before.p95:>12.4f}{row.after.p95:>11.4f}"
        )
    return "\n".join(lines)


def main(argv: list[str]) -> int:
    """Run the comparison and print the table.

    Args:
        argv (list[str]): Optional single argument: config file path (default node_cpu_compare.yaml).

    Returns:
        int: 0 on success, 2 on an error (no data, HTTP failure).
    """
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    settings = load_settings(Path(argv[0]) if argv else DEFAULT_CONFIG)
    try:
        with httpx.Client(timeout=HTTP_TIMEOUT_S) as client:
            rows = compare_all(client, settings)
    except (NoDataError, httpx.HTTPError) as error:
        logger.error("%s", error)
        return EXIT_ERROR
    print(render_table(rows))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
