"""Unit tests for scripts/node_cpu_compare.py with a mocked httpx transport (no network)."""

import httpx
import pydantic
import pytest

from scripts import mppi_cpu_compare
from scripts import node_cpu_compare as ncc
from scripts.node_cpu_compare import (
    DEFAULT_CONFIG,
    EXIT_ERROR,
    NodeEntry,
    Settings,
    compare_all,
    load_settings,
    main,
)

REAL_CLIENT = httpx.Client
SWITCH = 1_000_000.0
STEP = 60
SAMPLES = 60
SETTLE_MIN = 10.0


def make_settings(entries: list[dict] | None = None, **overrides: object) -> Settings:
    """Build Settings with a 1 h window and the given entries."""
    values: dict[str, object] = {
        "window_h": 1.0,
        "settle_min": SETTLE_MIN,
        "entries": entries
        or [
            {
                "name": "map_guard",
                "before_unit": "ros2-map_guard.service",
                "after_unit": "ros2-map_guard_cpp.service",
                "switch_time": SWITCH,
            },
            {
                "name": "web_ui",
                "before_unit": "ros2-web_ui.service",
                "after_unit": "ros2-web_ui.service",
                "switch_time": SWITCH,
            },
        ],
    }
    values.update(overrides)
    return Settings(**values)


def make_client(per_unit: dict[str, tuple[list[float], list[float]]], requests: list | None = None) -> httpx.Client:
    """httpx client answering range queries per unit: before samples in the early window, after in the late one."""
    after_start = SWITCH + SETTLE_MIN * 60

    def handler(request: httpx.Request) -> httpx.Response:
        if requests is not None:
            requests.append(request)
        params = request.url.params
        query = params["query"]
        assert "container_cpu_usage_seconds_total" in query and 'host="client"' in query
        unit = next(u for u in per_unit if f'id="/system.slice/{u}"' in query)
        start = float(params["start"])
        values = per_unit[unit][1] if start == after_start else per_unit[unit][0]
        series = (
            [{"metric": {}, "values": [[start + i * STEP, str(v)] for i, v in enumerate(values)]}] if values else []
        )
        return httpx.Response(200, json={"status": "success", "data": {"resultType": "matrix", "result": series}})

    return REAL_CLIENT(transport=httpx.MockTransport(handler))


UNITS = {
    "ros2-map_guard.service": ([0.50] * SAMPLES, [0.50] * SAMPLES),
    "ros2-map_guard_cpp.service": ([0.0] * SAMPLES, [0.20] * SAMPLES),
    "ros2-web_ui.service": ([0.30] * SAMPLES, [0.25] * SAMPLES),
}


def test_reports_each_entry_with_before_unit_before_and_after_unit_after() -> None:
    rows = compare_all(make_client(UNITS), make_settings())
    by_name = {r.name: r for r in rows}
    assert by_name["map_guard"].before.mean == pytest.approx(0.50)
    assert by_name["map_guard"].after.mean == pytest.approx(0.20)
    assert by_name["web_ui"].before.p95 == pytest.approx(0.30)
    assert by_name["web_ui"].after.p95 == pytest.approx(0.25)


def test_total_row_sums_before_and_after() -> None:
    rows = compare_all(make_client(UNITS), make_settings())
    total = rows[-1]
    assert total.name == "total"
    assert total.before.mean == pytest.approx(0.80)
    assert total.after.mean == pytest.approx(0.45)


def test_render_table_lists_every_row_and_the_total() -> None:
    text = ncc.render_table(compare_all(make_client(UNITS), make_settings()))
    for name in ("map_guard", "web_ui", "total"):
        assert name in text


def test_reuses_the_query_and_statistics_functions_of_mppi_cpu_compare() -> None:
    assert ncc.query_window is mppi_cpu_compare.query_window
    assert ncc.summarize is mppi_cpu_compare.summarize
    assert ncc.window_bounds is mppi_cpu_compare.window_bounds


def test_empty_series_raises_no_data() -> None:
    units = {**UNITS, "ros2-web_ui.service": ([], [])}
    with pytest.raises(mppi_cpu_compare.NoDataError):
        compare_all(make_client(units), make_settings())


def test_main_exits_2_on_an_empty_series(tmp_path: pytest.TempPathFactory, monkeypatch: pytest.MonkeyPatch) -> None:
    config = tmp_path / "c.yaml"
    config.write_text(
        "window_h: 1\nsettle_min: 10\nentries:\n"
        f"  - {{name: web_ui, before_unit: a.service, after_unit: a.service, switch_time: {SWITCH}}}\n"
    )
    monkeypatch.setattr(ncc.httpx, "Client", lambda **_: make_client({"a.service": ([], [])}))
    assert main([str(config)]) == EXIT_ERROR


def test_main_prints_and_exits_0(tmp_path: pytest.TempPathFactory, monkeypatch: pytest.MonkeyPatch, capsys) -> None:
    config = tmp_path / "c.yaml"
    config.write_text(
        "window_h: 1\nsettle_min: 10\nentries:\n"
        f"  - {{name: web_ui, before_unit: a.service, after_unit: a.service, switch_time: {SWITCH}}}\n"
    )
    monkeypatch.setattr(ncc.httpx, "Client", lambda **_: make_client({"a.service": ([0.3] * 60, [0.2] * 60)}))
    assert main([str(config)]) == 0
    assert "total" in capsys.readouterr().out


def test_queries_use_the_configured_host_and_both_windows() -> None:
    requests: list[httpx.Request] = []
    compare_all(make_client(UNITS, requests), make_settings())
    assert len(requests) == 4  # 2 entries x (before + after)
    starts = {float(r.url.params["start"]) for r in requests}
    assert starts == {SWITCH - 3600, SWITCH + SETTLE_MIN * 60}


def test_entry_requires_all_fields_and_a_positive_switch_time() -> None:
    with pytest.raises(pydantic.ValidationError):
        NodeEntry(name="x", before_unit="a", after_unit="b")
    with pytest.raises(pydantic.ValidationError):
        NodeEntry(name="x", before_unit="a", after_unit="b", switch_time=0)


def test_shipped_config_lists_the_four_nodes() -> None:
    settings = load_settings(DEFAULT_CONFIG)
    by_name = {e.name: e for e in settings.entries}
    assert list(by_name) == ["map_guard", "web_ui", "gripper_uvc_camera", "mcp_server"]
    assert by_name["map_guard"].after_unit == "ros2-map_guard_cpp.service"
    for name in ("web_ui", "gripper_uvc_camera", "mcp_server"):
        assert by_name[name].before_unit == by_name[name].after_unit
