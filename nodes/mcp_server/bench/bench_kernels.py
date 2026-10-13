"""Profile of the mcp_server compute kernels: which are slow enough to be worth porting to C++.

Times the compute path of every mcp_server tool that does geometry or planning (no ROS), per kernel function the
median and p95 wall and CPU time per call, reads how often each tool is really called from Mimir
(mcp_tool_calls_total, mcp_tool_duration_seconds), and writes bench/REPORT.md (one ranked table) and
bench/report.json (the same rows). Verdict: "port" when the median wall time is at least threshold_ms.

    cd nodes/mcp_server && uv run python -m bench.bench_kernels [--config bench/bench_kernels.yaml] [--out bench]
    cd nodes/mcp_server && uv run python -m bench.bench_kernels --threshold-ms 15   # on the Pi 5
"""

import argparse
import json
import platform
import statistics
import time
from collections import defaultdict
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Literal

import httpx
import numpy as np
import yaml
from pydantic import BaseModel, ConfigDict, Field, field_validator

from bench import inputs

BENCH_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = BENCH_DIR / "bench_kernels.yaml"
FAMILIES = ("camera", "grasp", "ik_reach", "collision", "frontiers")
Family = Literal["camera", "grasp", "ik_reach", "collision", "frontiers"]
Verdict = Literal["port", "below threshold"]
SECONDS_PER_DAY = 86400.0
MS = 1000.0
P95 = 95
QUERY_TIMEOUT_S = 10.0
RANGE_STEPS = 288
NO_VALUE = "n/a"


class BenchConfig(BaseModel):
    """bench_kernels.yaml.

    Attributes:
        repetitions: Timed calls per kernel.
        threshold_ms: Median wall time (ms) from which a kernel gets the verdict "port".
        families: Kernel families to run.
        prometheus_url: Mimir Prometheus API base URL.
        lookback_hours: How far back the call counts are read.
    """

    model_config = ConfigDict(extra="forbid")

    repetitions: int = Field(ge=1)
    threshold_ms: float = Field(gt=0.0)
    families: list[Family] = Field(min_length=1)
    prometheus_url: str = Field(default="http://server.ros2.lan:9009/prometheus", min_length=1)
    lookback_hours: float = Field(default=24.0, gt=0.0)

    @field_validator("families")
    @classmethod
    def unique_families(cls, value: list[Family]) -> list[Family]:
        """Require every family at most once.

        Args:
            value (list[Family]): Configured families.

        Returns:
            list[Family]: The same list.

        Raises:
            ValueError: On a duplicate.
        """
        if len(set(value)) != len(value):
            raise ValueError(f"families must be unique: {value}")
        return value


class Timing(BaseModel):
    """Per call times of one kernel in milliseconds."""

    median_wall_ms: float
    p95_wall_ms: float
    median_cpu_ms: float
    p95_cpu_ms: float


class KernelRow(BaseModel):
    """One row of the report."""

    kernel: str
    family: Family
    median_ms: float
    p95_ms: float
    cpu_median_ms: float
    cpu_p95_ms: float
    calls_per_day: float | None
    verdict: Verdict
    tools: tuple[str, ...]


class MimirUsage(BaseModel):
    """What Mimir says about the real tool usage.

    Attributes:
        reachable: Mimir answered with data (an empty window counts as reachable).
        error: Why it did not.
        window_hours: Span between the first and the last sample of the call counter.
        calls_per_day: Tool -> calls per day over the window.
        mean_duration_ms: Tool -> mean call duration over the window.
    """

    reachable: bool
    error: str | None = None
    window_hours: float = 0.0
    calls_per_day: dict[str, float] = Field(default_factory=dict)
    mean_duration_ms: dict[str, float] = Field(default_factory=dict)


@dataclass(frozen=True)
class KernelSpec:
    """A kernel: its name, family, the MCP tools that run it and a factory of its zero-argument call.

    Attributes:
        name: module.function of the kernel.
        family: Kernel family.
        tools: MCP tool names whose calls count as calls of this kernel.
        make: Builds the inputs and returns the call that is timed.
        max_repetitions: Upper bound on the timed calls (kernels that take seconds), None for the configured count.
    """

    name: str
    family: Family
    tools: tuple[str, ...]
    make: Callable[[], Callable[[], Any]]
    max_repetitions: int | None = None


CAMERA_TOOLS = ("get_camera_image", "get_annotated_camera_image", "pixel_to_ground", "mark_candidate_points")
GRASP_TOOLS = ("plan_grasp", "grasp_object", "confirm_grasp")
ARM_TOOLS = ("move_arm_cartesian", "move_arm_joints", "get_arm_reach", "plan_grasp", "grasp_object")
BODY_TOOLS = (
    "move_arm_cartesian",
    "move_arm_joints",
    "enqueue_motions",
    "get_body_state",
    "plan_grasp",
    "grasp_object",
)
FRONTIER_TOOLS = ("list_frontiers", "plan_frontier_visit", "go_to_frontier", "look_toward_frontier")
MAP_TOOLS = ("get_map_summary", "get_topdown_view", "get_robot_state")

KERNELS: tuple[KernelSpec, ...] = (
    KernelSpec("camera_scene.build_scene", "camera", CAMERA_TOOLS, inputs.build_scene_call),
    KernelSpec("camera_scene.build_scene_gripper", "camera", CAMERA_TOOLS, inputs.gripper_scene_call),
    KernelSpec("camera_candidates.candidate_store", "camera", ("mark_candidate_points",), inputs.candidate_store_call),
    KernelSpec(
        "geometry.pixel_to_ground", "camera", ("pixel_to_ground", "mark_candidate_points"), inputs.pixel_to_ground_call
    ),
    KernelSpec(
        "camera_overlay.grid_polylines", "camera", ("get_annotated_camera_image",), lambda: inputs.grid_call(False)
    ),
    KernelSpec("camera_overlay.draw_grid", "camera", ("get_annotated_camera_image",), lambda: inputs.grid_call(True)),
    KernelSpec("camera_overlay.draw_lidar", "camera", ("get_annotated_camera_image",), inputs.lidar_overlay_call),
    KernelSpec("camera_overlay.project_points", "camera", ("get_annotated_camera_image",), inputs.project_points_call),
    KernelSpec("camera_overlay.circle_runs", "camera", ("get_annotated_camera_image",), inputs.circle_runs_call),
    KernelSpec("perception.scan_points", "camera", MAP_TOOLS, lambda: inputs.scan_call(False)),
    KernelSpec("perception.summarize_scan", "camera", ("get_robot_state",), lambda: inputs.scan_call(True)),
    KernelSpec("perception.map_stats", "camera", ("get_map_summary",), lambda: inputs.map_call(False)),
    KernelSpec(
        "perception.render_map_crop", "camera", ("get_topdown_view", "get_map_summary"), lambda: inputs.map_call(True)
    ),
    KernelSpec("perception.encode_jpeg", "camera", ("get_camera_image",), lambda: inputs.image_call("encode")),
    KernelSpec("perception.image_to_bgr", "camera", ("get_camera_image",), lambda: inputs.image_call("decode")),
    KernelSpec("perception.recompress_jpeg", "camera", ("get_camera_image",), lambda: inputs.image_call("recompress")),
    KernelSpec("grasp.plan_auto", "grasp", GRASP_TOOLS, lambda: inputs.grasp_plan_call(None), max_repetitions=3),
    KernelSpec("grasp.plan_scoop", "grasp", GRASP_TOOLS, lambda: inputs.grasp_plan_call("scoop"), max_repetitions=3),
    KernelSpec("grasp_check.band_verdict", "grasp", ("confirm_grasp", "grasp_object"), inputs.band_verdict_call),
    KernelSpec("jaw_crop.jaw_crop", "grasp", ("confirm_grasp", "grasp_object"), inputs.jaw_crop_call),
    KernelSpec("ik.inverse", "ik_reach", ARM_TOOLS, lambda: inputs.ik_call("inverse")),
    KernelSpec("ik.inverse_flange", "ik_reach", ARM_TOOLS, lambda: inputs.ik_call("flange")),
    KernelSpec("ik.forward", "ik_reach", ARM_TOOLS, lambda: inputs.ik_call("forward")),
    KernelSpec("ik.link_frames", "ik_reach", ARM_TOOLS, lambda: inputs.ik_call("frames")),
    KernelSpec("arm_reach.point_reach", "ik_reach", ("get_arm_reach", "move_arm_cartesian"), inputs.point_reach_call),
    KernelSpec("reach_map.build", "ik_reach", ("get_arm_reach",), inputs.reach_map_call, max_repetitions=2),
    KernelSpec("arm_geometry.box_surface_points", "ik_reach", BODY_TOOLS, inputs.box_points_call),
    KernelSpec(
        "sag.deflection", "ik_reach", ("move_arm_joints", "move_arm_cartesian"), lambda: inputs.sag_call("deflection")
    ),
    KernelSpec(
        "sag.compensate", "ik_reach", ("move_arm_joints", "move_arm_cartesian"), lambda: inputs.sag_call("compensate")
    ),
    KernelSpec("self_collision.path", "collision", BODY_TOOLS, lambda: inputs.collision_call("path")),
    KernelSpec(
        "self_collision.pose", "collision", ("get_body_state", "get_arm_state"), lambda: inputs.collision_call("pose")
    ),
    KernelSpec("self_collision.geometry", "collision", BODY_TOOLS, lambda: inputs.collision_call("geometry")),
    KernelSpec(
        "footprint_geometry.body_outline", "collision", ("get_body_state",), lambda: inputs.body_outline_call("outline")
    ),
    KernelSpec(
        "footprint_geometry.shapes", "collision", ("get_body_state",), lambda: inputs.body_outline_call("shapes")
    ),
    KernelSpec("body_model.signed_distance", "collision", BODY_TOOLS, inputs.signed_distance_call),
    KernelSpec("frontiers.extract_frontiers", "frontiers", FRONTIER_TOOLS, lambda: inputs.frontier_call("extract")),
    KernelSpec("frontiers.traversability", "frontiers", FRONTIER_TOOLS, lambda: inputs.frontier_call("traversability")),
    KernelSpec("frontiers.geodesic_from", "frontiers", FRONTIER_TOOLS, lambda: inputs.frontier_call("geodesic")),
    KernelSpec("frontiers.frontier_field", "frontiers", FRONTIER_TOOLS, lambda: inputs.frontier_call("field")),
    KernelSpec("frontiers.observation_pose", "frontiers", FRONTIER_TOOLS, lambda: inputs.frontier_call("observation")),
    KernelSpec("frontiers.tracker_update", "frontiers", FRONTIER_TOOLS, lambda: inputs.frontier_call("update")),
)


def load_config(path: Path) -> BenchConfig:
    """Read and validate bench_kernels.yaml.

    Args:
        path (Path): The yaml file.

    Returns:
        BenchConfig: The validated config.

    Raises:
        pydantic.ValidationError: On a missing, unknown or invalid key.
    """
    return BenchConfig.model_validate(yaml.safe_load(path.read_text()))


def kernels_for(families: Sequence[str]) -> list[KernelSpec]:
    """Kernels of the given families, in registry order.

    Args:
        families (Sequence[str]): Family names.

    Returns:
        list[KernelSpec]: The kernels.
    """
    return [k for k in KERNELS if k.family in families]


def verdict_for(median_ms: float, threshold_ms: float) -> Verdict:
    """'port' when the median wall time reaches the threshold.

    Args:
        median_ms (float): Median wall time per call (ms).
        threshold_ms (float): Threshold (ms), inclusive.

    Returns:
        Verdict: 'port' or 'below threshold'.
    """
    return "port" if median_ms >= threshold_ms else "below threshold"


def time_kernel(fn: Callable[[], Any], repetitions: int) -> Timing:
    """Time a call: one warm-up, then repetitions timed calls (wall and thread CPU time).

    Args:
        fn (Callable[[], Any]): The kernel call.
        repetitions (int): Timed calls.

    Returns:
        Timing: Median and p95 per call in ms.
    """
    fn()
    wall: list[float] = []
    cpu: list[float] = []
    for _ in range(repetitions):
        cpu_start, wall_start = time.thread_time(), time.perf_counter()
        fn()
        wall.append((time.perf_counter() - wall_start) * MS)
        cpu.append((time.thread_time() - cpu_start) * MS)
    return Timing(
        median_wall_ms=statistics.median(wall),
        p95_wall_ms=float(np.percentile(wall, P95)),
        median_cpu_ms=statistics.median(cpu),
        p95_cpu_ms=float(np.percentile(cpu, P95)),
    )


def rank_rows(rows: Sequence[KernelRow]) -> list[KernelRow]:
    """Rows slowest first by median wall time (ties keep their order).

    Args:
        rows (Sequence[KernelRow]): Rows.

    Returns:
        list[KernelRow]: Ranked rows.
    """
    return sorted(rows, key=lambda r: -r.median_ms)


def group_by_family(rows: Sequence[KernelRow]) -> dict[str, list[KernelRow]]:
    """Rows per family in FAMILIES order, each list keeping the given order.

    Args:
        rows (Sequence[KernelRow]): Rows.

    Returns:
        dict[str, list[KernelRow]]: Family -> rows, only families that have rows.
    """
    grouped: dict[str, list[KernelRow]] = defaultdict(list)
    for r in rows:
        grouped[r.family].append(r)
    return {f: grouped[f] for f in FAMILIES if f in grouped}


def run_kernels(specs: Sequence[KernelSpec], cfg: BenchConfig) -> list[KernelRow]:
    """Time every kernel and rank the rows.

    Args:
        specs (Sequence[KernelSpec]): Kernels.
        cfg (BenchConfig): Repetitions and threshold.

    Returns:
        list[KernelRow]: Ranked rows without call counts.
    """
    rows = []
    for k in specs:
        reps = cfg.repetitions if k.max_repetitions is None else min(cfg.repetitions, k.max_repetitions)
        t = time_kernel(k.make(), reps)
        rows.append(
            KernelRow(
                kernel=k.name,
                family=k.family,
                median_ms=t.median_wall_ms,
                p95_ms=t.p95_wall_ms,
                cpu_median_ms=t.median_cpu_ms,
                cpu_p95_ms=t.p95_cpu_ms,
                calls_per_day=None,
                verdict=verdict_for(t.median_wall_ms, cfg.threshold_ms),
                tools=k.tools,
            )
        )
    return rank_rows(rows)


def query_range(
    client: httpx.Client, base_url: str, query: str, start: float, end: float
) -> dict[str, list[tuple[float, float]]]:
    """Run a Prometheus range query grouped by the tool label.

    Args:
        client (httpx.Client): HTTP client.
        base_url (str): Prometheus API base URL.
        query (str): PromQL.
        start (float): Start (unix s).
        end (float): End (unix s).

    Returns:
        dict[str, list[tuple[float, float]]]: Tool -> (timestamp, value) samples.

    Raises:
        RuntimeError: When Mimir reports an error status.
        httpx.HTTPError: On a transport or HTTP error.
    """
    resp = client.get(
        f"{base_url.rstrip('/')}/api/v1/query_range",
        params={"query": query, "start": start, "end": end, "step": max(60.0, (end - start) / RANGE_STEPS)},
        timeout=QUERY_TIMEOUT_S,
    )
    resp.raise_for_status()
    body = resp.json()
    if body.get("status") != "success":
        raise RuntimeError(f"Mimir query failed: {body.get('error', body)}")
    return {
        item["metric"].get("tool", ""): [(float(t), float(v)) for t, v in item["values"]]
        for item in body["data"]["result"]
    }


def counter_increase(samples: list[tuple[float, float]]) -> float:
    """Increase of a counter over its samples; a drop (restart) counts the new value as the increase.

    Args:
        samples (list[tuple[float, float]]): (timestamp, value) in time order.

    Returns:
        float: Total increase.
    """
    total = 0.0
    for (_, before), (_, now) in zip(samples, samples[1:], strict=False):
        total += now - before if now >= before else now
    return total


def fetch_mimir(prometheus_url: str, lookback_hours: float, client: httpx.Client | None = None) -> MimirUsage:
    """Calls per day and mean duration per tool from mcp_tool_calls_total and mcp_tool_duration_seconds.

    Mimir being unreachable or answering with an error is returned as reachable=False with the reason; nothing is
    invented.

    Args:
        prometheus_url (str): Mimir Prometheus API base URL.
        lookback_hours (float): Look back this far (the data actually present is the window).
        client (httpx.Client | None): HTTP client (tests pass a mock transport).

    Returns:
        MimirUsage: The usage.
    """
    end = time.time()
    start = end - lookback_hours * 3600.0
    http = client or httpx.Client()
    try:
        calls = query_range(http, prometheus_url, "sum by (tool) (mcp_tool_calls_total)", start, end)
        dur_sum = query_range(http, prometheus_url, "sum by (tool) (mcp_tool_duration_seconds_sum)", start, end)
        dur_count = query_range(http, prometheus_url, "sum by (tool) (mcp_tool_duration_seconds_count)", start, end)
    except (httpx.HTTPError, RuntimeError, ValueError, KeyError) as exc:
        return MimirUsage(reachable=False, error=f"{type(exc).__name__}: {exc}")
    finally:
        if client is None:
            http.close()
    stamps = [t for series in calls.values() for t, _ in series]
    window_s = max(stamps) - min(stamps) if stamps else 0.0
    if window_s <= 0.0:
        return MimirUsage(reachable=True)
    per_day = {tool: counter_increase(s) / window_s * SECONDS_PER_DAY for tool, s in calls.items()}
    mean_ms = {}
    for tool, n_series in dur_count.items():
        n = counter_increase(n_series)
        if n > 0 and tool in dur_sum:
            mean_ms[tool] = counter_increase(dur_sum[tool]) / n * MS
    return MimirUsage(reachable=True, window_hours=window_s / 3600.0, calls_per_day=per_day, mean_duration_ms=mean_ms)


def attach_usage(rows: Sequence[KernelRow], usage: MimirUsage) -> list[KernelRow]:
    """Rows with calls/day set to the sum over the kernel's tools (None when Mimir was unreachable).

    Args:
        rows (Sequence[KernelRow]): Rows.
        usage (MimirUsage): Mimir result.

    Returns:
        list[KernelRow]: Rows with calls_per_day.
    """
    if not usage.reachable:
        return [r.model_copy(update={"calls_per_day": None}) for r in rows]
    return [r.model_copy(update={"calls_per_day": sum(usage.calls_per_day.get(t, 0.0) for t in r.tools)}) for r in rows]


def fmt_ms(value: float) -> str:
    """Milliseconds with a precision that keeps microsecond kernels readable.

    Args:
        value (float): Milliseconds.

    Returns:
        str: Text.
    """
    return f"{value:.3f}" if value < 10 else f"{value:.1f}"


def render_markdown(rows: Sequence[KernelRow], usage: MimirUsage, cfg: BenchConfig) -> str:
    """REPORT.md: the ranked table and how to read it.

    Args:
        rows (Sequence[KernelRow]): Ranked rows with call counts.
        usage (MimirUsage): Mimir result.
        cfg (BenchConfig): Config used.

    Returns:
        str: Markdown.
    """
    if not usage.reachable:
        source = f"Mimir was not reachable ({usage.error}); calls/day is {NO_VALUE}, not estimated."
    elif usage.window_hours <= 0.0:
        source = "Mimir answered but holds no mcp_tool_calls_total samples in the window; calls/day is 0."
    else:
        source = f"calls/day from Mimir over a window of {usage.window_hours:.1f} h, scaled to 24 h."
    ports = [r.kernel for r in rows if r.verdict == "port"]
    lines = [
        "# mcp_server kernel profile",
        "",
        f"Run {datetime.now(UTC):%Y-%m-%d %H:%M} UTC on {platform.machine()} ({platform.system()}), Python "
        f"{platform.python_version()}, {cfg.repetitions} timed calls per kernel (some capped lower), after one warm-up call.",
        f"Verdict: **port** when the median wall time is >= {cfg.threshold_ms:g} ms (5 ms on the dev Mac, about 15 ms on the "
        "Pi 5), else below threshold. " + source,
        "",
        "Kernels to port: " + (", ".join(f"`{k}`" for k in ports) if ports else "none"),
        "",
        "| kernel | family | median ms | p95 ms | calls/day | verdict |",
        "|---|---|---:|---:|---:|---|",
    ]
    for r in rows:
        calls = NO_VALUE if r.calls_per_day is None else f"{r.calls_per_day:.0f}"
        lines.append(
            f"| {r.kernel} | {r.family} | {fmt_ms(r.median_ms)} | {fmt_ms(r.p95_ms)} | {calls} | {r.verdict} |"
        )
    lines += [
        "",
        "Median and p95 are wall time per call; CPU times per call are in report.json (cpu_median_ms, cpu_p95_ms).",
        "calls/day is the sum of the calls of the MCP tools that run the kernel (report.json: tools).",
        "",
    ]
    return "\n".join(lines)


def write_reports(rows: Sequence[KernelRow], usage: MimirUsage, cfg: BenchConfig, out_dir: Path) -> None:
    """Write REPORT.md and report.json.

    Args:
        rows (Sequence[KernelRow]): Ranked rows with call counts.
        usage (MimirUsage): Mimir result.
        cfg (BenchConfig): Config used.
        out_dir (Path): Directory for both files.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "REPORT.md").write_text(render_markdown(rows, usage, cfg))
    doc: dict[str, Any] = {
        "generated_at": datetime.now(UTC).isoformat(timespec="seconds"),
        "machine": platform.machine(),
        "system": platform.system(),
        "repetitions": cfg.repetitions,
        "threshold_ms": cfg.threshold_ms,
        "families": cfg.families,
        "mimir": usage.model_dump(),
        "rows": [r.model_dump(mode="json") for r in rows],
    }
    (out_dir / "report.json").write_text(json.dumps(doc, indent=2) + "\n")


def main(argv: list[str] | None = None) -> int:
    """Run the bench and write the reports.

    Args:
        argv (list[str] | None): Arguments (default sys.argv).

    Returns:
        int: Exit code.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--out", type=Path, default=BENCH_DIR)
    parser.add_argument("--threshold-ms", type=float, default=None, help="override threshold_ms of the yaml")
    args = parser.parse_args(argv)
    cfg = load_config(args.config)
    if args.threshold_ms is not None:
        cfg = cfg.model_copy(update={"threshold_ms": args.threshold_ms})
    rows = run_kernels(kernels_for(cfg.families), cfg)
    usage = fetch_mimir(cfg.prometheus_url, cfg.lookback_hours)
    write_reports(attach_usage(rows, usage), usage, cfg, args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
