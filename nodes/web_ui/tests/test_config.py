"""Tests for web_ui config loading."""

from __future__ import annotations

from pathlib import Path

import pytest

from web_ui.config import AppConfig, load_config


def test_load_config_minimal(config_yaml: Path) -> None:
    cfg = load_config(config_yaml)
    assert isinstance(cfg, AppConfig)
    assert cfg.http_port == 8080
    assert cfg.tabs == []
    assert cfg.overlays == []


def test_load_config_http_port_override(tmp_path: Path) -> None:
    p = tmp_path / "cfg.yaml"
    p.write_text("http_port: 9999\ntabs: []\noverlays: []\n")
    cfg = load_config(p)
    assert cfg.http_port == 9999


def test_load_config_missing_raises(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        load_config(tmp_path / "nonexistent.yaml")


def test_all_subscribed_topics(tmp_path: Path) -> None:
    p = tmp_path / "cfg.yaml"
    p.write_text(
        """
tabs:
  - id: cam
    type: camera
    label: Cam
    topic: /controller/camera_0/image_raw
  - id: nav
    type: nav_local
    label: Nav
    scan_topic: /controller/scan
    costmap_topic: /controller/local_costmap
    odom_topic: /controller/odom
    goal_topic: /controller/goal_pose
overlays:
  - topic: /controller/gps/fix
    field: latitude
    label: Lat
"""
    )
    cfg = load_config(p)
    topics = cfg.all_subscribed_topics()
    assert "/controller/camera_0/image_raw" in topics
    assert "/controller/scan" in topics
    assert "/controller/gps/fix" in topics
    assert "/controller/goal_pose" not in topics  # publish-only


def test_load_config_from_env_var(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    p = tmp_path / "env_config.yaml"
    p.write_text("http_port: 7777\ntabs: []\noverlays: []\n")
    monkeypatch.setenv("WEB_UI_CONFIG", str(p))
    cfg = load_config()
    assert cfg.http_port == 7777


def test_publish_topics(tmp_path: Path) -> None:
    p = tmp_path / "cfg.yaml"
    p.write_text(
        """
tabs:
  - id: nav
    type: nav_local
    label: Nav
    scan_topic: /controller/scan
    goal_topic: /controller/goal_pose
overlays: []
"""
    )
    cfg = load_config(p)
    assert "/controller/goal_pose" in cfg.publish_topics()
