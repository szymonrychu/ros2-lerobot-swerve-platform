"""Unit tests for scripts/propose_unit_limits.py with fixture Prometheus responses (no network)."""

import pydantic
import pytest
import yaml

from scripts.propose_unit_limits import (
    Settings,
    build_queries,
    build_rows,
    load_node_limits,
    load_settings,
    parse_memory_mib,
    parse_quota_pct,
    parse_vector,
    propose_cpu_quota_pct,
    propose_memory_high_mib,
    propose_memory_max_mib,
    render_table,
    render_yaml,
    round_up,
)

MIB = 1024 * 1024

GROUP_VARS = """
ros2_node_type_defaults:
  web_ui:
    cpu_quota: "30%"
    memory_max: "256M"
  feetech_servos:
    cpu_quota: "50%"
    memory_max: "1G"
  bare_type: {}
ros2_nodes:
  - name: web_ui
    node_type: web_ui
  - name: lerobot_follower
    node_type: feetech_servos
  - name: ros2-master
    node_type: bare_type
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
    assert cfg.prometheus_url == "http://client.ros2.lan:9090"
    assert cfg.window == "48h"
    assert cfg.percentile == 0.99
    assert (cfg.cpu_margin, cfg.mem_margin) == (1.5, 1.5)
    assert (cfg.min_cpu_quota_pct, cfg.min_memory_mib) == (10, 64)
    assert cfg.unit_regex == r"^ros2-.*\.service$"
    assert cfg.format == "table"


def test_load_settings_from_yaml(tmp_path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text(yaml.safe_dump({"window": "24h", "format": "yaml"}))
    cfg = load_settings(path)
    assert cfg.window == "24h" and cfg.format == "yaml"


def test_settings_reject_bad_values() -> None:
    for bad in ({"format": "json"}, {"percentile": 1.5}, {"window": "two days"}, {"cpu_margin": 0}):
        with pytest.raises(pydantic.ValidationError):
            settings(**bad)


def test_round_up() -> None:
    assert round_up(51, 5) == 55
    assert round_up(55, 5) == 55
    assert round_up(0.1, 16) == 16


def test_cpu_quota_rounds_up_to_five_with_margin() -> None:
    # 0.33 cores = 33 % * 1.5 = 49.5 -> 50
    assert propose_cpu_quota_pct(0.33, settings()) == 50
    # 0.34 cores * 1.5 = 51 -> 55
    assert propose_cpu_quota_pct(0.34, settings()) == 55


def test_cpu_quota_minimum() -> None:
    assert propose_cpu_quota_pct(0.001, settings()) == 10
    assert propose_cpu_quota_pct(0.0, settings(min_cpu_quota_pct=20)) == 20


def test_memory_max_uses_larger_of_margin_and_headroom() -> None:
    # p99 100 MiB * 1.5 = 150; max 200 MiB * 1.2 = 240 -> 240 -> 240 (multiple of 16)
    assert propose_memory_max_mib(100 * MIB, 200 * MIB, settings()) == 240
    # p99 200 * 1.5 = 300; max 210 * 1.2 = 252 -> 300 -> 304
    assert propose_memory_max_mib(200 * MIB, 210 * MIB, settings()) == 304


def test_memory_max_minimum() -> None:
    assert propose_memory_max_mib(1 * MIB, 2 * MIB, settings()) == 64
    assert propose_memory_max_mib(1 * MIB, 2 * MIB, settings(min_memory_mib=128)) == 128


def test_memory_high_is_85_percent() -> None:
    assert propose_memory_high_mib(304) == 258
    assert propose_memory_high_mib(64) == 54


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


def test_load_node_limits_maps_node_to_type(tmp_path) -> None:
    path = tmp_path / "client.yml"
    path.write_text(GROUP_VARS)
    limits = load_node_limits(path)
    assert limits["ros2-web_ui.service"].node_type == "web_ui"
    assert limits["ros2-web_ui.service"].cpu_quota_pct == 30
    assert limits["ros2-web_ui.service"].memory_max_mib == 256
    assert limits["ros2-lerobot_follower.service"].memory_max_mib == 1024
    # template defaults when the type sets nothing
    assert limits["ros2-ros2-master.service"].cpu_quota_pct == 50
    assert limits["ros2-ros2-master.service"].memory_max_mib == 256
    # present: false nodes have no unit
    assert "ros2-gone_node.service" not in limits


def test_build_queries_use_window_and_percentile() -> None:
    queries = build_queries(settings(window="12h", percentile=0.95))
    assert "quantile_over_time(0.95" in queries.cpu_p
    assert "rate(container_cpu_usage_seconds_total" in queries.cpu_p
    assert "[1m]" in queries.cpu_p and "[12h:1m]" in queries.cpu_p
    assert "container_memory_working_set_bytes" in queries.mem_p
    assert "max_over_time" in queries.mem_max and "[12h]" in queries.mem_max
    assert "/system.slice/ros2-.*.service" in queries.cpu_p


def test_build_rows_proposals_and_flags(tmp_path) -> None:
    path = tmp_path / "client.yml"
    path.write_text(GROUP_VARS)
    limits = load_node_limits(path)
    cpu = parse_vector(
        vector({"/system.slice/ros2-web_ui.service": 0.34, "/system.slice/ros2-lerobot_follower.service": 0.2}),
        settings().unit_regex,
    )
    mem_p = parse_vector(
        vector(
            {
                "/system.slice/ros2-web_ui.service": 100 * MIB,
                "/system.slice/ros2-lerobot_follower.service": 50 * MIB,
            }
        ),
        settings().unit_regex,
    )
    mem_max = parse_vector(
        vector(
            {
                "/system.slice/ros2-web_ui.service": 300 * MIB,
                "/system.slice/ros2-lerobot_follower.service": 60 * MIB,
            }
        ),
        settings().unit_regex,
    )
    rows = {r.unit: r for r in build_rows(settings(), limits, cpu, mem_p, mem_max)}

    web = rows["ros2-web_ui.service"]
    assert web.node_type == "web_ui"
    assert web.proposed_cpu_quota_pct == 55
    assert web.proposed_memory_max_mib == 368  # 300 * 1.2 = 360, rounded up to 16 MiB
    assert web.proposed_memory_high_mib == 312
    assert "oom risk" in web.flag  # max 300 MiB > MemoryMax 256 MiB
    assert "throttle risk" in web.flag  # p99 34 % >= 90 % of 30 %

    follower = rows["ros2-lerobot_follower.service"]
    assert follower.flag == ""
    assert follower.proposed_cpu_quota_pct == 30


def test_build_rows_reports_no_data_and_unknown_units(tmp_path) -> None:
    path = tmp_path / "client.yml"
    path.write_text(GROUP_VARS)
    limits = load_node_limits(path)
    cpu = parse_vector(vector({"/system.slice/ros2-extra.service": 0.1}), settings().unit_regex)
    rows = {r.unit: r for r in build_rows(settings(), limits, cpu, {}, {})}
    master = rows["ros2-ros2-master.service"]
    assert master.flag == "no data"
    assert master.proposed_cpu_quota_pct is None
    assert master.proposed_memory_max_mib is None
    assert master.proposed_memory_high_mib is None
    # CPU present but memory missing is still no data
    assert rows["ros2-extra.service"].flag == "no data"
    assert rows["ros2-extra.service"].node_type == "unknown"


def test_render_table_shows_columns_and_no_data() -> None:
    cfg = settings()
    limits = {}
    cpu = parse_vector(vector({"/system.slice/ros2-a.service": 0.2}), cfg.unit_regex)
    mem = parse_vector(vector({"/system.slice/ros2-a.service": 80 * MIB}), cfg.unit_regex)
    mem_max = parse_vector(vector({"/system.slice/ros2-a.service": 90 * MIB}), cfg.unit_regex)
    text = render_table(build_rows(cfg, limits, cpu, mem, mem_max))
    header = text.splitlines()[0]
    for col in ("unit", "node_type", "CPUQuota", "p99 CPU %", "MemoryMax", "p99 mem", "max mem", "MemoryHigh", "flag"):
        assert col in header
    assert "ros2-a.service" in text
    assert "no data" not in text


def test_render_yaml_roundtrips() -> None:
    cfg = settings()
    cpu = parse_vector(vector({"/system.slice/ros2-a.service": 0.2}), cfg.unit_regex)
    mem = parse_vector(vector({"/system.slice/ros2-a.service": 80 * MIB}), cfg.unit_regex)
    mem_max = parse_vector(vector({"/system.slice/ros2-a.service": 90 * MIB}), cfg.unit_regex)
    doc = yaml.safe_load(render_yaml(build_rows(cfg, {}, cpu, mem, mem_max)))
    entry = doc["ros2-a.service"]
    assert entry["cpu_quota"] == "30%"
    assert entry["memory_max"] == "128M"
    assert entry["memory_high"] == "108M"
