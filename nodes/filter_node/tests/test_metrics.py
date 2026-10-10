"""Tests for the filter node metrics (filter_node.metrics) and the metrics_port config field."""

from pathlib import Path

from prometheus_client import CollectorRegistry

from filter_node.config import load_config
from filter_node.metrics import FilterMetrics


def make_metrics() -> tuple[FilterMetrics, CollectorRegistry]:
    registry = CollectorRegistry()
    return FilterMetrics(registry), registry


def test_metrics_port_default_and_parsed(tmp_path: Path) -> None:
    p = tmp_path / "c.yaml"
    p.write_text("algorithm: kalman\n")
    cfg = load_config(p)
    assert cfg is not None and cfg.metrics_port is None
    p.write_text("algorithm: kalman\nmetrics_port: 19102\n")
    cfg = load_config(p)
    assert cfg is not None and cfg.metrics_port == 19102


def test_input_age_tracks_time_since_last_message() -> None:
    m, reg = make_metrics()
    assert reg.get_sample_value("filter_input_age_seconds", {"source": "leader"}) is None
    m.record_input("leader", 10.0)
    m.refresh("leader", 10.5)
    assert reg.get_sample_value("filter_input_age_seconds", {"source": "leader"}) == 0.5
    # a source that never sent anything has no sample (unknown, not zero)
    assert reg.get_sample_value("filter_input_age_seconds", {"source": "web_ui"}) is None
    m.record_input("web_ui", 11.0)
    m.refresh("leader", 12.0)
    assert reg.get_sample_value("filter_input_age_seconds", {"source": "web_ui"}) == 1.0
    assert reg.get_sample_value("filter_input_age_seconds", {"source": "leader"}) == 2.0


def test_active_source_one_hot() -> None:
    m, reg = make_metrics()
    m.refresh("web_ui", 1.0)
    values = {
        s: reg.get_sample_value("filter_active_source", {"source": s}) for s in ("leader", "web_ui", "autonomy", "none")
    }
    assert values == {"leader": 0.0, "web_ui": 1.0, "autonomy": 0.0, "none": 0.0}


def test_source_switches_counted_on_change_only() -> None:
    m, reg = make_metrics()
    m.refresh("leader", 1.0)  # first observation is not a switch
    m.refresh("leader", 2.0)
    assert reg.get_sample_value("filter_source_switches_total") == 0.0
    m.refresh("autonomy", 3.0)
    m.refresh("autonomy", 4.0)
    m.refresh("none", 5.0)
    assert reg.get_sample_value("filter_source_switches_total") == 2.0


def test_loop_overrun_counted_when_iteration_late() -> None:
    m, reg = make_metrics()
    period = 0.01
    m.observe_loop(100.0, period)  # first iteration: no interval yet
    m.observe_loop(100.01, period)  # on time
    assert reg.get_sample_value("filter_loop_overruns_total") == 0.0
    m.observe_loop(100.05, period)  # 40 ms late for a 10 ms period
    assert reg.get_sample_value("filter_loop_overruns_total") == 1.0
