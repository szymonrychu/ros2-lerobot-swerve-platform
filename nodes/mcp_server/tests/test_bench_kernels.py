"""Tests for bench/bench_kernels.py: ranking, verdicts, family grouping, yaml validation and the Mimir error path."""

import json
import time
from pathlib import Path
from typing import Any

import httpx
import pytest
import yaml
from pydantic import ValidationError

from bench.bench_kernels import (
    FAMILIES,
    KERNELS,
    BenchConfig,
    KernelRow,
    KernelSpec,
    MimirUsage,
    attach_usage,
    fetch_mimir,
    group_by_family,
    kernels_for,
    load_config,
    rank_rows,
    render_markdown,
    run_kernels,
    time_kernel,
    verdict_for,
    write_reports,
)

BENCH_DIR = Path(__file__).resolve().parents[1] / "bench"
FAMILY_KERNELS = {
    "camera": {
        "camera_scene.build_scene",
        "camera_candidates.candidate_store",
        "geometry.pixel_to_ground",
        "camera_overlay.grid_polylines",
        "perception.scan_points",
    },
    "grasp": {"grasp.plan_auto", "grasp_check.band_verdict", "jaw_crop.jaw_crop"},
    "ik_reach": {
        "ik.inverse",
        "arm_reach.point_reach",
        "reach_map.build",
        "arm_geometry.box_surface_points",
        "sag.deflection",
    },
    "collision": {"self_collision.path", "footprint_geometry.body_outline", "body_model.signed_distance"},
    "frontiers": {"frontiers.extract_frontiers"},
}


def spec(name: str, family: str = "camera", sleep_s: float = 0.0, tools: tuple[str, ...] = ()) -> KernelSpec:
    def make() -> Any:
        return lambda: time.sleep(sleep_s)

    return KernelSpec(name=name, family=family, tools=tools, make=make)


def row(name: str, median_ms: float, family: str = "camera", threshold_ms: float = 5.0) -> KernelRow:
    return KernelRow(
        kernel=name,
        family=family,
        median_ms=median_ms,
        p95_ms=median_ms * 1.5,
        cpu_median_ms=median_ms,
        cpu_p95_ms=median_ms * 1.5,
        calls_per_day=None,
        verdict=verdict_for(median_ms, threshold_ms),
        tools=(),
    )


def test_config_yaml_in_repo_is_valid() -> None:
    cfg = load_config(BENCH_DIR / "bench_kernels.yaml")
    assert cfg.repetitions >= 1
    assert cfg.threshold_ms == 5.0
    assert set(cfg.families) == set(FAMILIES)
    assert cfg.prometheus_url.startswith("http")


@pytest.mark.parametrize(
    "bad",
    [
        {"repetitions": 0},
        {"threshold_ms": -1.0},
        {"families": []},
        {"families": ["camera", "nope"]},
        {"families": ["camera", "camera"]},
        {"unknown_key": 1},
        {"lookback_hours": 0},
    ],
)
def test_config_rejects_bad_values(bad: dict[str, Any], tmp_path: Path) -> None:
    good = {
        "repetitions": 3,
        "threshold_ms": 5.0,
        "families": ["camera"],
        "prometheus_url": "http://mimir.invalid/prometheus",
        "lookback_hours": 24,
    }
    path = tmp_path / "c.yaml"
    path.write_text(yaml.safe_dump(good | bad))
    with pytest.raises(ValidationError):
        load_config(path)


def test_verdict_threshold_is_inclusive() -> None:
    assert verdict_for(5.0, 5.0) == "port"
    assert verdict_for(12.0, 5.0) == "port"
    assert verdict_for(4.999, 5.0) == "below threshold"
    assert verdict_for(14.9, 15.0) == "below threshold"


def test_rank_orders_by_median_descending_and_is_stable() -> None:
    ranked = rank_rows([row("a", 1.0), row("b", 30.0), row("c", 7.0), row("d", 7.0)])
    assert [r.kernel for r in ranked] == ["b", "c", "d", "a"]


def test_group_by_family_keeps_ranking_inside_each_family() -> None:
    rows = rank_rows([row("a", 1.0, "grasp"), row("b", 3.0, "camera"), row("c", 2.0, "grasp")])
    grouped = group_by_family(rows)
    assert list(grouped) == ["camera", "grasp"]
    assert [r.kernel for r in grouped["grasp"]] == ["c", "a"]


def test_time_kernel_reports_median_p95_wall_and_cpu() -> None:
    state = {"n": 0}

    def fn() -> None:
        state["n"] += 1
        time.sleep(0.002)

    timing = time_kernel(fn, repetitions=7)
    assert state["n"] >= 7
    assert timing.median_wall_ms >= 2.0
    assert timing.p95_wall_ms >= timing.median_wall_ms
    assert timing.median_cpu_ms < timing.median_wall_ms  # sleeping burns no CPU
    assert timing.p95_cpu_ms >= timing.median_cpu_ms


def test_run_kernels_times_every_spec_and_applies_threshold() -> None:
    cfg = BenchConfig(repetitions=3, threshold_ms=5.0, families=["camera", "grasp"])
    specs = [spec("slow", "camera", sleep_s=0.02, tools=("t1",)), spec("fast", "grasp")]
    rows = run_kernels(specs, cfg)
    by_name = {r.kernel: r for r in rows}
    assert by_name["slow"].verdict == "port"
    assert by_name["fast"].verdict == "below threshold"
    assert by_name["slow"].family == "camera"
    assert by_name["slow"].tools == ("t1",)
    assert rows[0].kernel == "slow"  # ranked


def test_kernels_for_filters_by_family() -> None:
    only = kernels_for(["frontiers"])
    assert only and {k.family for k in only} == {"frontiers"}


def test_every_family_has_a_kernel_and_the_planned_kernels_exist() -> None:
    for family in FAMILIES:
        assert [k for k in KERNELS if k.family == family], family
    for family, names in FAMILY_KERNELS.items():
        have = {k.name for k in KERNELS if k.family == family}
        assert names <= have, (family, names - have)


def test_kernel_names_are_unique_and_tools_are_strings() -> None:
    names = [k.name for k in KERNELS]
    assert len(names) == len(set(names))
    assert all(isinstance(t, str) for k in KERNELS for t in k.tools)


@pytest.mark.parametrize("kernel", KERNELS, ids=lambda k: k.name)
def test_every_real_kernel_runs_once(kernel: KernelSpec) -> None:
    fn = kernel.make()
    fn()  # the call itself must not raise
    timing = time_kernel(fn, repetitions=1)
    assert timing.median_wall_ms > 0.0


def prom_matrix(series: dict[str, list[tuple[float, float]]], label: str = "tool") -> dict[str, Any]:
    return {
        "status": "success",
        "data": {
            "resultType": "matrix",
            "result": [
                {"metric": {label: name}, "values": [[t, str(v)] for t, v in vals]} for name, vals in series.items()
            ],
        },
    }


def mimir_client(handler: Any) -> httpx.Client:
    return httpx.Client(transport=httpx.MockTransport(handler))


def test_fetch_mimir_computes_calls_per_day_from_the_window() -> None:
    hour = 3600.0
    calls = {"get_camera_image": [(0.0, 10), (hour, 30), (2 * hour, 50)], "drive": [(0.0, 1), (2 * hour, 3)]}
    sums = {"get_camera_image": [(0.0, 1.0), (2 * hour, 5.0)]}
    counts = {"get_camera_image": [(0.0, 10), (2 * hour, 50)]}

    def handler(request: httpx.Request) -> httpx.Response:
        q = request.url.params["query"]
        assert request.url.path.endswith("/api/v1/query_range")
        if "mcp_tool_calls_total" in q:
            return httpx.Response(200, json=prom_matrix(calls))
        if "_sum" in q:
            return httpx.Response(200, json=prom_matrix(sums))
        return httpx.Response(200, json=prom_matrix(counts))

    usage = fetch_mimir("http://m/prometheus", 24.0, client=mimir_client(handler))
    assert usage.reachable and usage.error is None
    assert usage.window_hours == pytest.approx(2.0)
    assert usage.calls_per_day["get_camera_image"] == pytest.approx(40 / 2 * 24)
    assert usage.calls_per_day["drive"] == pytest.approx(2 / 2 * 24)
    assert usage.mean_duration_ms["get_camera_image"] == pytest.approx((4.0 / 40) * 1000)


def test_counter_reset_does_not_count_negative_calls() -> None:
    calls = {"drive": [(0.0, 5), (60.0, 8), (120.0, 1), (180.0, 3)]}

    def handler(request: httpx.Request) -> httpx.Response:
        if "mcp_tool_calls_total" in request.url.params["query"]:
            return httpx.Response(200, json=prom_matrix(calls))
        return httpx.Response(200, json=prom_matrix({}))

    usage = fetch_mimir("http://m/prometheus", 1.0, client=mimir_client(handler))
    assert usage.calls_per_day["drive"] == pytest.approx((3 + 1 + 2) / 180.0 * 86400)


def test_mimir_connection_error_is_reported_not_faked() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        raise httpx.ConnectError("connection refused", request=request)

    usage = fetch_mimir("http://m/prometheus", 24.0, client=mimir_client(handler))
    assert not usage.reachable
    assert usage.error and "connection refused" in usage.error
    assert usage.calls_per_day == {}


def test_mimir_http_error_and_bad_status_are_reported() -> None:
    usage = fetch_mimir("http://m/prometheus", 24.0, client=mimir_client(lambda r: httpx.Response(503, text="down")))
    assert not usage.reachable and "503" in (usage.error or "")
    bad = fetch_mimir(
        "http://m/prometheus",
        24.0,
        client=mimir_client(lambda r: httpx.Response(200, json={"status": "error", "error": "boom"})),
    )
    assert not bad.reachable and "boom" in (bad.error or "")


def test_mimir_empty_window_is_reachable_with_no_calls() -> None:
    usage = fetch_mimir(
        "http://m/prometheus", 24.0, client=mimir_client(lambda r: httpx.Response(200, json=prom_matrix({})))
    )
    assert usage.reachable and usage.window_hours == 0.0 and usage.calls_per_day == {}


def test_attach_usage_sums_the_tools_of_a_kernel_and_leaves_none_when_unreachable() -> None:
    rows = [row("a", 1.0), row("b", 2.0)]
    rows[0] = rows[0].model_copy(update={"tools": ("t1", "t2")})
    usage = MimirUsage(reachable=True, error=None, window_hours=2.0, calls_per_day={"t1": 10.0, "t2": 5.0, "t3": 99.0})
    out = attach_usage(rows, usage)
    assert out[0].calls_per_day == 15.0
    assert out[1].calls_per_day == 0.0  # no tool of b was called
    down = attach_usage(rows, MimirUsage(reachable=False, error="x"))
    assert all(r.calls_per_day is None for r in down)


def test_report_files_have_the_same_rows(tmp_path: Path) -> None:
    cfg = BenchConfig(repetitions=3, threshold_ms=5.0, families=["camera", "grasp"])
    rows = rank_rows([row("fast", 1.0, "grasp"), row("slow", 20.0, "camera")])
    rows = attach_usage(rows, MimirUsage(reachable=False, error="connection refused"))
    write_reports(rows, MimirUsage(reachable=False, error="connection refused"), cfg, tmp_path)
    md = (tmp_path / "REPORT.md").read_text()
    data = json.loads((tmp_path / "report.json").read_text())
    assert [r["kernel"] for r in data["rows"]] == ["slow", "fast"]
    assert data["threshold_ms"] == 5.0 and data["mimir"]["reachable"] is False
    assert "| kernel | family | median ms | p95 ms | calls/day | verdict |" in md
    assert md.index("slow") < md.index("fast")
    assert "port" in md and "below threshold" in md
    assert "connection refused" in md  # unreachable Mimir is stated in the report
    assert "n/a" in md  # and calls/day is not invented


def test_markdown_lists_only_ranked_rows_in_one_table() -> None:
    cfg = BenchConfig(repetitions=3, threshold_ms=5.0, families=["camera"])
    md = render_markdown([row("a", 9.0), row("b", 1.0)], MimirUsage(reachable=True, error=None, window_hours=3.0), cfg)
    assert md.count("| kernel |") == 1
    assert "3.0 h" in md
