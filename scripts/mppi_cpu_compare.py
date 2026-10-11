"""Compare the CPU of a systemd unit before and after a switch (the MPPI overlay deploy) from Prometheus/Mimir.

Reads scripts/mppi_cpu_compare.yaml (pydantic-validated), queries rate(container_cpu_usage_seconds_total[1m]) of
/system.slice/<unit> over [switch - window, switch] and [switch + settle, switch + settle + window], and prints the
mean and p95 of both windows. PASS when after <= max(before * (1 + rel_limit), before + abs_limit_cores) for both
statistics. Exits 1 on FAIL and 2 on errors (an empty series is an error, never a PASS). Read-only.

Usage: python scripts/mppi_cpu_compare.py [config.yaml]
"""

import logging
import sys
from pathlib import Path
from typing import Literal

import httpx
import numpy as np
import yaml
from pydantic import BaseModel, Field, field_validator

DEFAULT_CONFIG = Path(__file__).resolve().with_suffix(".yaml")
CPU_RATE_WINDOW = "1m"
STEP_S = 60
P95 = 95
SECONDS_PER_HOUR = 3600
SECONDS_PER_MINUTE = 60
HTTP_TIMEOUT_S = 30.0
EXIT_FAIL = 1
EXIT_ERROR = 2

logger = logging.getLogger("mppi_cpu_compare")


class NoDataError(RuntimeError):
    """A required Prometheus series is empty, so no verdict can be given."""


class Settings(BaseModel):
    """Validated contents of mppi_cpu_compare.yaml."""

    prometheus_url: str = "http://server.ros2.lan:9009/prometheus"
    host: str = "client"
    unit: str = "ros2-nav2_bringup.service"
    window_h: float = Field(default=6, gt=0)
    settle_min: float = Field(default=10, ge=0)
    rel_limit: float = Field(default=0.05, ge=0)
    abs_limit_cores: float = Field(default=0.01, ge=0)
    switch_time: float | Literal["auto"] = "auto"

    @field_validator("switch_time")
    @classmethod
    def switch_time_is_positive(cls, value: float | str) -> float | str:
        """Reject a non-positive epoch.

        Args:
            value (float | str): "auto" or epoch seconds.

        Returns:
            float | str: The same value.
        """
        if value != "auto" and value <= 0:
            raise ValueError("switch_time must be 'auto' or a positive epoch in seconds")
        return value

    @property
    def selector(self) -> str:
        """PromQL label selector of the unit's cgroup."""
        return f'host="{self.host}", id="/system.slice/{self.unit}"'


class Stats(BaseModel):
    """Mean and p95 of one window, in cores."""

    mean: float
    p95: float


class Result(BaseModel):
    """Outcome of one before/after comparison."""

    switch_time: float
    before: Stats
    after: Stats
    mean_limit: float
    p95_limit: float

    @property
    def mean_passed(self) -> bool:
        """True when the mean after the switch is within its limit."""
        return self.after.mean <= self.mean_limit

    @property
    def p95_passed(self) -> bool:
        """True when the p95 after the switch is within its limit."""
        return self.after.p95 <= self.p95_limit

    @property
    def passed(self) -> bool:
        """True when both statistics are within their limits."""
        return self.mean_passed and self.p95_passed


def load_settings(path: Path) -> Settings:
    """Load and validate the config file.

    Args:
        path (Path): YAML config file.

    Returns:
        Settings: Validated settings.
    """
    return Settings(**(yaml.safe_load(path.read_text()) or {}))


def window_bounds(settings: Settings, switch: float) -> tuple[tuple[float, float], tuple[float, float]]:
    """Compute the before and after windows (the settle period after the switch is excluded).

    Args:
        settings (Settings): Window and settle lengths.
        switch (float): Switch time, epoch seconds.

    Returns:
        tuple[tuple[float, float], tuple[float, float]]: (before_start, before_end), (after_start, after_end).
    """
    window = settings.window_h * SECONDS_PER_HOUR
    after_start = switch + settings.settle_min * SECONDS_PER_MINUTE
    return (switch - window, switch), (after_start, after_start + window)


def resolve_switch_time(client: httpx.Client, settings: Settings) -> float:
    """Return the switch time, looking up the unit's latest start time when it is "auto".

    Args:
        client (httpx.Client): HTTP client.
        settings (Settings): Settings with switch_time.

    Returns:
        float: Epoch seconds.

    Raises:
        NoDataError: "auto" and no container_start_time_seconds series for the unit.
    """
    if settings.switch_time != "auto":
        return float(settings.switch_time)
    promql = f"max(container_start_time_seconds{{{settings.selector}}})"
    response = client.get(f"{settings.prometheus_url.rstrip('/')}/api/v1/query", params={"query": promql})
    response.raise_for_status()
    result = response.json()["data"]["result"]
    if not result:
        raise NoDataError(f"no container_start_time_seconds for {settings.unit} on host {settings.host}")
    return float(result[0]["value"][1])


def query_window(client: httpx.Client, settings: Settings, start: float, end: float) -> list[float]:
    """Query the unit's 1m CPU rate (cores) over one window.

    Args:
        client (httpx.Client): HTTP client.
        settings (Settings): Host and unit.
        start (float): Window start, epoch seconds.
        end (float): Window end, epoch seconds.

    Returns:
        list[float]: Samples in cores.

    Raises:
        NoDataError: The window has no samples.
    """
    promql = f"sum(rate(container_cpu_usage_seconds_total{{{settings.selector}}}[{CPU_RATE_WINDOW}]))"
    response = client.get(
        f"{settings.prometheus_url.rstrip('/')}/api/v1/query_range",
        params={"query": promql, "start": start, "end": end, "step": STEP_S},
    )
    response.raise_for_status()
    values = [float(v[1]) for series in response.json()["data"]["result"] for v in series["values"]]
    values = [v for v in values if np.isfinite(v)]
    if not values:
        raise NoDataError(f"no CPU samples for {settings.unit} on host {settings.host} in [{start:.0f}, {end:.0f}]")
    return values


def summarize(values: list[float]) -> Stats:
    """Mean and p95 of samples.

    Args:
        values (list[float]): Non-empty samples.

    Returns:
        Stats: Mean and p95.
    """
    return Stats(mean=float(np.mean(values)), p95=float(np.percentile(values, P95)))


def limit(before: float, settings: Settings) -> float:
    """Highest acceptable "after" value for one statistic.

    Args:
        before (float): The statistic before the switch.
        settings (Settings): rel_limit and abs_limit_cores.

    Returns:
        float: max(before * (1 + rel), before + abs).
    """
    return max(before * (1 + settings.rel_limit), before + settings.abs_limit_cores)


def compare(client: httpx.Client, settings: Settings) -> Result:
    """Query both windows and judge them.

    Args:
        client (httpx.Client): HTTP client.
        settings (Settings): Config.

    Returns:
        Result: Statistics, limits and verdict.

    Raises:
        NoDataError: A window or the start time has no data.
    """
    switch = resolve_switch_time(client, settings)
    (b_start, b_end), (a_start, a_end) = window_bounds(settings, switch)
    before = summarize(query_window(client, settings, b_start, b_end))
    after = summarize(query_window(client, settings, a_start, a_end))
    return Result(
        switch_time=switch,
        before=before,
        after=after,
        mean_limit=limit(before.mean, settings),
        p95_limit=limit(before.p95, settings),
    )


def render_table(result: Result) -> str:
    """Format the result as a short table.

    Args:
        result (Result): Comparison result.

    Returns:
        str: Multi-line text ending with the verdict.
    """
    lines = [
        f"switch time: {result.switch_time:.0f}",
        f"{'stat':<6}{'before':>10}{'after':>10}{'limit':>10}  verdict",
    ]
    for name, before, after, lim, ok in (
        ("mean", result.before.mean, result.after.mean, result.mean_limit, result.mean_passed),
        ("p95", result.before.p95, result.after.p95, result.p95_limit, result.p95_passed),
    ):
        lines.append(f"{name:<6}{before:>10.4f}{after:>10.4f}{lim:>10.4f}  {'ok' if ok else 'OVER'}")
    lines.append("PASS" if result.passed else "FAIL")
    return "\n".join(lines)


def main(argv: list[str]) -> int:
    """Run the comparison and print the table.

    Args:
        argv (list[str]): Optional single argument: config file path (default mppi_cpu_compare.yaml).

    Returns:
        int: 0 on PASS, 1 on FAIL, 2 on an error (no data, HTTP failure).
    """
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    settings = load_settings(Path(argv[0]) if argv else DEFAULT_CONFIG)
    try:
        with httpx.Client(timeout=HTTP_TIMEOUT_S) as client:
            result = compare(client, settings)
    except (NoDataError, httpx.HTTPError) as error:
        logger.error("%s", error)
        return EXIT_ERROR
    print(render_table(result))
    return 0 if result.passed else EXIT_FAIL


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
