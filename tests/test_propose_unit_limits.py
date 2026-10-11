"""Unit tests for scripts/propose_unit_limits.py with fixture Prometheus responses (no network)."""

import tempfile
from pathlib import Path

import pydantic
import pytest
import yaml

from scripts.propose_unit_limits import (
    DEFAULT_CONFIG,
    Settings,
    build_queries,
    build_rows,
    load_node_limits,
    load_settings,
    memory_low_sum_mib,
    parse_memory_mib,
    parse_quota_pct,
    parse_vector,
    propose_memory_low_mib,
    propose_memory_max_mib,
    propose_weights,
    render_table,
    render_yaml,
    round_up,
)

MIB = 1024 * 1024

GROUP_VARS = """
ros2_node_type_defaults:
  web_ui:
    memory_low: "112M"
    memory_max: "256M"
    cpu_weight: 300
    io_weight: 300
  feetech_servos:
    memory_low: "56M"
    memory_max: "1G"
    cpu_weight: 1000
    io_weight: 1000
  quota_type:
    cpu_quota: "25%"
    memory_max: "192M"
  bare_type: {}
ros2_nodes:
  - name: web_ui
    node_type: web_ui
  - name: lerobot_follower
    node_type: feetech_servos
  - name: ros2-master
    node_type: bare_type
  - name: scraper
    node_type: quota_type
  - name: gone_node
    node_type: web_ui
    present: false
"""


def vector(values: dict[str, float]) -> dict:
    """Build a Prometheus instant-vector response keyed by cgroup id.

    Args:
        values (dict[str, float]): cgroup id -> sample value.

    Returns:
        dict: Prometheus /api/v1/query JSON.
    """
    result = [{"metric": {"id": k}, "value": [1700000000, str(v)]} for k, v in values.items()]
    return {"status": "success", "data": {"resultType": "vector", "result": result}}


def settings(**overrides: object) -> Settings:
    """Return default settings with overrides.

    Args:
        **overrides (object): Fields to override.

    Returns:
        Settings: Validated settings.
    """
    return Settings(**overrides)


def test_settings_defaults() -> None:
    cfg = settings()
    # Mimir on the server stores the robot's metrics; its Prometheus API lives under /prometheus.
    assert cfg.prometheus_url == "http://server.ros2.lan:9009/prometheus"
    assert cfg.host == "client"
    assert cfg.window == "24h"
    assert (cfg.percentile, cfg.typical_percentile) == (0.99, 0.5)
    assert cfg.memory_max_factor == 2.0
    assert (cfg.min_memory_low_mib, cfg.min_memory_max_mib) == (32, 64)
    assert (cfg.default_cpu_weight, cfg.default_io_weight) == (500, 500)
    assert cfg.unit_regex == r"^ros2-.*\.service$"
    assert cfg.format == "table"


def test_load_settings_from_yaml(tmp_path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text(yaml.safe_dump({"window": "24h", "format": "yaml"}))
    cfg = load_settings(path)
    assert cfg.window == "24h" and cfg.format == "yaml"


def test_shipped_yaml_queries_the_robot_series_in_mimir() -> None:
    """scripts/propose_unit_limits.yaml points at the server's Mimir and selects the robot's series (host label)."""
    cfg = load_settings(DEFAULT_CONFIG)
    assert cfg.prometheus_url == "http://server.ros2.lan:9009/prometheus"
    assert cfg.host == "client"
    assert cfg.memory_max_factor == 2.0 and cfg.typical_percentile == 0.5


def test_settings_reject_bad_values() -> None:
    for bad in (
        {"format": "json"},
        {"percentile": 1.5},
        {"typical_percentile": 0},
        {"window": "two days"},
        {"memory_max_factor": 0.5},
        {"default_cpu_weight": 0},
        {"default_io_weight": 10001},
    ):
        with pytest.raises(pydantic.ValidationError):
            settings(**bad)


def test_round_up() -> None:
    assert round_up(51, 5) == 55
    assert round_up(55, 5) == 55
    assert round_up(0.1, 16) == 16


def test_memory_low_is_the_typical_working_set_rounded_up_to_8() -> None:
    # 49.7 MiB -> 56, 48.0 -> 48 (already a multiple of 8)
    assert propose_memory_low_mib(49.7 * MIB, settings()) == 56
    assert propose_memory_low_mib(48 * MIB, settings()) == 48


def test_memory_low_minimum() -> None:
    assert propose_memory_low_mib(1 * MIB, settings()) == 32
    assert propose_memory_low_mib(1 * MIB, settings(min_memory_low_mib=48)) == 48


def test_memory_max_is_twice_the_peak_rounded_up_to_16() -> None:
    # peak 66.4 MiB * 2 = 132.8 -> 144
    assert propose_memory_max_mib(66.4 * MIB, settings()) == 144
    # peak 253.1 * 2 = 506.2 -> 512
    assert propose_memory_max_mib(253.1 * MIB, settings()) == 512
    assert propose_memory_max_mib(100 * MIB, settings(memory_max_factor=3)) == 304


def test_memory_max_keeps_a_larger_current_cap_and_the_minimum() -> None:
    # The safety net never shrinks: a current 1G stays when twice the peak is less.
    assert propose_memory_max_mib(383.4 * MIB, settings(), current_mib=1024) == 1024
    assert propose_memory_max_mib(1 * MIB, settings()) == 64
    assert propose_memory_max_mib(1 * MIB, settings(min_memory_max_mib=128)) == 128


def test_weights_keep_the_configured_tier_or_use_the_default() -> None:
    limits = load_node_limits_from_text(GROUP_VARS)
    assert propose_weights(limits["ros2-lerobot_follower.service"], settings()) == (1000, 1000)
    assert propose_weights(limits["ros2-ros2-master.service"], settings()) == (500, 500)
    assert propose_weights(None, settings(default_cpu_weight=300, default_io_weight=200)) == (300, 200)


def test_parse_quota_and_memory() -> None:
    assert parse_quota_pct("50%") == 50
    assert parse_quota_pct("200%") == 200
    assert parse_memory_mib("128M") == 128
    assert parse_memory_mib("1G") == 1024
    assert parse_memory_mib("512K") == 0.5


def test_parse_vector_keys_by_unit_and_filters_regex() -> None:
    resp = vector(
        {
            "/system.slice/ros2-web_ui.service": 0.25,
            "/system.slice/ssh.service": 9.0,
            "/system.slice": 3.0,
        }
    )
    assert parse_vector(resp, settings().unit_regex) == {"ros2-web_ui.service": 0.25}


def test_parse_vector_ignores_error_response() -> None:
    assert parse_vector({"status": "error", "error": "x"}, r".*") == {}


def load_node_limits_from_text(text: str) -> dict:
    """Write group_vars text to a temporary file and load its node limits.

    Args:
        text (str): group_vars YAML.

    Returns:
        dict: Unit name -> NodeLimits.
    """
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "client.yml"
        path.write_text(text)
        return load_node_limits(path)


def test_load_node_limits_maps_node_to_type() -> None:
    limits = load_node_limits_from_text(GROUP_VARS)
    web = limits["ros2-web_ui.service"]
    assert web.node_type == "web_ui"
    assert (web.memory_low_mib, web.memory_max_mib, web.cpu_weight, web.io_weight) == (112, 256, 300, 300)
    assert web.cpu_quota_pct is None
    assert limits["ros2-lerobot_follower.service"].memory_max_mib == 1024
    # template defaults when the type sets nothing: no quota, no MemoryLow, no weights, MemoryMax 256M
    bare = limits["ros2-ros2-master.service"]
    assert (bare.cpu_quota_pct, bare.memory_low_mib, bare.cpu_weight, bare.io_weight) == (None, None, None, None)
    assert bare.memory_max_mib == 256
    assert limits["ros2-scraper.service"].cpu_quota_pct == 25
    # present: false nodes have no unit
    assert "ros2-gone_node.service" not in limits


def test_build_queries_use_window_and_percentiles() -> None:
    queries = build_queries(settings(window="12h", percentile=0.95, typical_percentile=0.6))
    assert "quantile_over_time(0.95" in queries.cpu_p
    assert "rate(container_cpu_usage_seconds_total" in queries.cpu_p
    assert "[1m]" in queries.cpu_p and "[12h:1m]" in queries.cpu_p
    assert "quantile_over_time(0.6, container_memory_working_set_bytes" in queries.mem_typical
    assert "max_over_time" in queries.mem_max and "[12h]" in queries.mem_max
    assert "/system.slice/ros2-.*.service" in queries.cpu_p


def test_build_queries_select_the_configured_host() -> None:
    """Mimir holds both hosts' series: every selector carries host="<host>" (default client, configurable)."""
    for host in ("client", "server"):
        queries = build_queries(settings(host=host))
        for expr in (queries.cpu_p, queries.mem_typical, queries.mem_max):
            assert f'host="{host}"' in expr, expr


def test_build_rows_proposals_and_flags() -> None:
    limits = load_node_limits_from_text(GROUP_VARS)
    regex = settings().unit_regex
    ids = {
        "web": "/system.slice/ros2-web_ui.service",
        "follower": "/system.slice/ros2-lerobot_follower.service",
        "scraper": "/system.slice/ros2-scraper.service",
    }
    cpu = parse_vector(vector({ids["web"]: 0.34, ids["follower"]: 0.2, ids["scraper"]: 0.24}), regex)
    typical = parse_vector(vector({ids["web"]: 120 * MIB, ids["follower"]: 50 * MIB, ids["scraper"]: 100 * MIB}), regex)
    peak = parse_vector(vector({ids["web"]: 300 * MIB, ids["follower"]: 60 * MIB, ids["scraper"]: 128 * MIB}), regex)
    rows = {r.unit: r for r in build_rows(settings(), limits, cpu, typical, peak)}

    web = rows["ros2-web_ui.service"]
    assert web.node_type == "web_ui"
    assert web.proposed_memory_low_mib == 120
    assert web.proposed_memory_max_mib == 608  # 300 * 2 = 600, rounded up to 16 MiB
    assert (web.proposed_cpu_weight, web.proposed_io_weight) == (300, 300)
    assert "oom risk" in web.flag  # peak 300 MiB > MemoryMax 256 MiB
    assert "tight max" in web.flag  # MemoryMax 256 < 2 x peak
    assert "unprotected" in web.flag  # MemoryLow 112 < typical 120
    assert "throttle risk" not in web.flag  # no quota

    follower = rows["ros2-lerobot_follower.service"]
    assert follower.flag == ""
    assert follower.proposed_memory_max_mib == 1024  # current 1G kept
    assert follower.proposed_memory_low_mib == 56

    scraper = rows["ros2-scraper.service"]
    assert "throttle risk" in scraper.flag  # p99 24 % >= 90 % of 25 %
    assert "unprotected" in scraper.flag  # no MemoryLow at all
    assert (scraper.proposed_cpu_weight, scraper.proposed_io_weight) == (500, 500)


def test_build_rows_reports_no_data_and_unknown_units() -> None:
    limits = load_node_limits_from_text(GROUP_VARS)
    cpu = parse_vector(vector({"/system.slice/ros2-extra.service": 0.1}), settings().unit_regex)
    rows = {r.unit: r for r in build_rows(settings(), limits, cpu, {}, {})}
    master = rows["ros2-ros2-master.service"]
    assert master.flag == "no data"
    assert master.proposed_memory_low_mib is None
    assert master.proposed_memory_max_mib is None
    assert master.proposed_cpu_weight is None
    # CPU present but memory missing is still no data
    assert rows["ros2-extra.service"].flag == "no data"
    assert rows["ros2-extra.service"].node_type == "unknown"


def test_memory_low_sum_counts_rows_with_a_proposal() -> None:
    cfg = settings()
    ids = ("/system.slice/ros2-a.service", "/system.slice/ros2-b.service")
    cpu = parse_vector(vector({ids[0]: 0.1, ids[1]: 0.1}), cfg.unit_regex)
    typical = parse_vector(vector({ids[0]: 40 * MIB, ids[1]: 100 * MIB}), cfg.unit_regex)
    peak = parse_vector(vector({ids[0]: 50 * MIB, ids[1]: 110 * MIB}), cfg.unit_regex)
    rows = build_rows(cfg, {}, cpu, typical, peak) + build_rows(cfg, {}, cpu, {}, {})
    assert memory_low_sum_mib(rows) == 40 + 104


def test_render_table_shows_columns_and_the_memory_low_sum() -> None:
    cfg = settings()
    cpu = parse_vector(vector({"/system.slice/ros2-a.service": 0.2}), cfg.unit_regex)
    mem = parse_vector(vector({"/system.slice/ros2-a.service": 80 * MIB}), cfg.unit_regex)
    mem_max = parse_vector(vector({"/system.slice/ros2-a.service": 90 * MIB}), cfg.unit_regex)
    text = render_table(build_rows(cfg, {}, cpu, mem, mem_max))
    header = text.splitlines()[0]
    for col in (
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
    ):
        assert col in header
    assert "CPUQuota" not in header and "MemoryHigh" not in header
    assert "ros2-a.service" in text
    assert "no data" not in text
    assert text.splitlines()[-1] == "sum of proposed MemoryLow: 80M"


def test_render_yaml_roundtrips() -> None:
    cfg = settings()
    cpu = parse_vector(vector({"/system.slice/ros2-a.service": 0.2}), cfg.unit_regex)
    mem = parse_vector(vector({"/system.slice/ros2-a.service": 80 * MIB}), cfg.unit_regex)
    mem_max = parse_vector(vector({"/system.slice/ros2-a.service": 90 * MIB}), cfg.unit_regex)
    doc = yaml.safe_load(render_yaml(build_rows(cfg, {}, cpu, mem, mem_max)))
    entry = doc["ros2-a.service"]
    assert entry["memory_low"] == "80M"
    assert entry["memory_max"] == "192M"  # 90 * 2 = 180 -> 192
    assert (entry["cpu_weight"], entry["io_weight"]) == (500, 500)
    assert "cpu_quota" not in entry and "memory_high" not in entry
