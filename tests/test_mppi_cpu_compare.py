"""Unit tests for scripts/mppi_cpu_compare.py with a mocked httpx transport (no network)."""

import httpx
import pydantic
import pytest

from scripts.mppi_cpu_compare import (
    DEFAULT_CONFIG,
    NoDataError,
    Settings,
    compare,
    load_settings,
    resolve_switch_time,
    window_bounds,
)

SWITCH = 1_000_000.0
STEP = 60


def make_settings(**overrides: object) -> Settings:
    """Build Settings with a fixed numeric switch time (window 1 h keeps the series short)."""
    values: dict[str, object] = {"switch_time": SWITCH, "window_h": 1.0, "settle_min": 10.0}
    values.update(overrides)
    return Settings(**values)


def matrix(values: list[float], start: float) -> dict:
    """Prometheus query_range response with one series sampled every STEP seconds (none when values is empty)."""
    series = [{"metric": {}, "values": [[start + i * STEP, str(v)] for i, v in enumerate(values)]}] if values else []
    return {"status": "success", "data": {"resultType": "matrix", "result": series}}


def make_client(
    before: list[float],
    after: list[float],
    start_time: float | None = 123.0,
    requests: list | None = None,
) -> httpx.Client:
    """httpx client whose transport answers range queries per window and the start-time instant query."""
    (before_start, _), (after_start, _) = window_bounds(make_settings(), SWITCH)

    def handler(request: httpx.Request) -> httpx.Response:
        if requests is not None:
            requests.append(request)
        params = request.url.params
        if request.url.path.endswith("/api/v1/query"):
            assert "container_start_time_seconds" in params["query"]
            result = [] if start_time is None else [{"metric": {}, "value": [1, str(start_time)]}]
            return httpx.Response(200, json={"status": "success", "data": {"resultType": "vector", "result": result}})
        assert "container_cpu_usage_seconds_total" in params["query"]
        assert 'id="/system.slice/ros2-nav2_bringup.service"' in params["query"]
        assert 'host="client"' in params["query"]
        start = float(params["start"])
        assert start in (before_start, after_start)
        return httpx.Response(200, json=matrix(before if start == before_start else after, start))

    return httpx.Client(transport=httpx.MockTransport(handler))


FLAT = [0.10] * 60


def test_pass_when_after_equals_before() -> None:
    result = compare(make_client(FLAT, FLAT), make_settings())
    assert result.passed
    assert result.before.mean == pytest.approx(0.10)
    assert result.after.p95 == pytest.approx(0.10)


def test_fail_on_relative_limit() -> None:
    # 0.10 -> 0.12: +0.02 core is above abs 0.01 and +20 % is above rel 5 %.
    result = compare(make_client(FLAT, [0.12] * 60), make_settings())
    assert not result.passed


def test_pass_on_abs_slack_when_rel_exceeded() -> None:
    # 0.010 -> 0.019: +90 % relative but only +0.009 core, inside abs 0.01.
    assert compare(make_client([0.010] * 60, [0.019] * 60), make_settings()).passed


def test_pass_on_rel_slack_when_abs_exceeded() -> None:
    # 1.00 -> 1.04: +0.04 core above abs 0.01, but +4 % inside rel 5 %.
    assert compare(make_client([1.0] * 60, [1.04] * 60), make_settings()).passed


def test_fail_on_abs_limit() -> None:
    # 0.010 -> 0.021: above abs (0.020) and rel (0.0105).
    assert not compare(make_client([0.010] * 60, [0.021] * 60), make_settings()).passed


def test_fail_when_only_p95_regresses() -> None:
    # 5 of 60 samples at 0.2: mean 0.1083 is inside the 0.11 limit, p95 0.2 is not.
    result = compare(make_client(FLAT, [0.10] * 55 + [0.20] * 5), make_settings())
    assert result.mean_passed
    assert not result.p95_passed
    assert not result.passed


def test_auto_switch_time_uses_container_start_time() -> None:
    settings = make_settings(switch_time="auto")
    requests: list[httpx.Request] = []
    client = make_client(FLAT, FLAT, start_time=SWITCH, requests=requests)
    assert resolve_switch_time(client, settings) == SWITCH
    result = compare(client, settings)
    assert result.passed
    assert result.switch_time == SWITCH


def test_auto_switch_time_without_series_is_an_error() -> None:
    with pytest.raises(NoDataError):
        resolve_switch_time(make_client(FLAT, FLAT, start_time=None), make_settings(switch_time="auto"))


def test_numeric_switch_time_skips_the_start_time_query() -> None:
    requests: list[httpx.Request] = []
    assert resolve_switch_time(make_client(FLAT, FLAT, requests=requests), make_settings()) == SWITCH
    assert requests == []


def test_window_bounds_exclude_the_settle_period() -> None:
    (b_start, b_end), (a_start, a_end) = window_bounds(make_settings(), SWITCH)
    assert (b_start, b_end) == (SWITCH - 3600, SWITCH)
    assert (a_start, a_end) == (SWITCH + 600, SWITCH + 600 + 3600)


def test_settle_period_samples_are_never_requested() -> None:
    requests: list[httpx.Request] = []
    compare(make_client(FLAT, FLAT, requests=requests), make_settings())
    starts = sorted(float(r.url.params["start"]) for r in requests)
    assert all(not (SWITCH < s < SWITCH + 600) for s in starts)
    assert all(float(r.url.params["end"]) <= SWITCH or float(r.url.params["start"]) >= SWITCH + 600 for r in requests)


@pytest.mark.parametrize("empty", ["before", "after"])
def test_empty_series_is_an_error_not_a_pass(empty: str) -> None:
    before, after = ([], FLAT) if empty == "before" else (FLAT, [])
    with pytest.raises(NoDataError):
        compare(make_client(before, after), make_settings())


def test_default_config_loads_with_the_documented_values() -> None:
    settings = load_settings(DEFAULT_CONFIG)
    assert settings.prometheus_url == "http://server.ros2.lan:9009/prometheus"
    assert (settings.host, settings.unit) == ("client", "ros2-nav2_bringup.service")
    assert (settings.window_h, settings.settle_min) == (6, 10)
    assert (settings.rel_limit, settings.abs_limit_cores) == (0.05, 0.01)
    assert settings.switch_time == "auto"


@pytest.mark.parametrize(
    "bad",
    [
        {"rel_limit": -0.1},
        {"abs_limit_cores": -0.01},
        {"window_h": 0},
        {"settle_min": -1},
        {"switch_time": "yesterday"},
        {"switch_time": -5},
    ],
)
def test_config_validation_rejects_bad_values(bad: dict) -> None:
    with pytest.raises(pydantic.ValidationError):
        Settings(**bad)
