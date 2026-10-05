"""Unit tests for the rf2o relay config loading."""

from pathlib import Path

from rf2o_odom_relay.config import load_config


def test_missing_file_is_none(tmp_path: Path) -> None:
    assert load_config(tmp_path / "nope.yaml") is None


def test_defaults(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("{}\n")
    cfg = load_config(path)
    assert cfg is not None
    assert cfg.input_topic == "/odom_rf2o"
    assert cfg.output_topic == "/odom_rf2o_twist"
    assert cfg.var_vx_vy > 0 and cfg.var_vyaw > 0 and cfg.max_dt_s > 0


def test_overrides(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("input_topic: /a\noutput_topic: /b\nvar_vx_vy: 0.5\nvar_vyaw: 0.6\nmax_dt_s: 2.0\n")
    cfg = load_config(path)
    assert cfg is not None
    assert (cfg.input_topic, cfg.output_topic, cfg.var_vx_vy, cfg.var_vyaw, cfg.max_dt_s) == ("/a", "/b", 0.5, 0.6, 2.0)
