"""Unit tests for the relay metrics and the metrics_port config field."""

from pathlib import Path

from prometheus_client import REGISTRY

from rf2o_odom_relay.config import load_config
from rf2o_odom_relay.twist import PoseSample, relay_twist


def sample(name: str) -> float:
    return REGISTRY.get_sample_value(name) or 0.0


def test_metrics_port_defaults_to_none(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("{}\n")
    cfg = load_config(path)
    assert cfg is not None
    assert cfg.metrics_port is None


def test_metrics_port_parsed(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("metrics_port: 19108\n")
    cfg = load_config(path)
    assert cfg is not None
    assert cfg.metrics_port == 19108


def test_relay_twist_counts_published_messages() -> None:
    before = sample("relay_messages_total")
    twist = relay_twist(PoseSample(0.0, 0.0, 0.0, 1.0), PoseSample(0.1, 0.0, 0.0, 1.1), 1.0)
    assert twist is not None
    assert sample("relay_messages_total") == before + 1


def test_relay_twist_counts_dt_rejections() -> None:
    before_rejected = sample("relay_dt_rejected_total")
    before_messages = sample("relay_messages_total")
    assert relay_twist(PoseSample(0.0, 0.0, 0.0, 1.0), PoseSample(0.1, 0.0, 0.0, 5.0), 1.0) is None
    assert relay_twist(PoseSample(0.0, 0.0, 0.0, 1.0), PoseSample(0.1, 0.0, 0.0, 1.0), 1.0) is None
    assert sample("relay_dt_rejected_total") == before_rejected + 2
    assert sample("relay_messages_total") == before_messages
