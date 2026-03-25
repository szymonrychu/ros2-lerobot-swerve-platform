"""Tests for bridge config loading and validation."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from bridge.config import AppConfig, BridgeConfig, TabConfig, load_config


def _write_config(tmp_path: Path, data: dict) -> Path:
    p = tmp_path / "config.yaml"
    p.write_text(yaml.dump(data))
    return p


class TestBridgeConfig:
    def test_defaults(self) -> None:
        cfg = BridgeConfig()
        assert cfg.host == "localhost"
        assert cfg.port == 9090
        assert cfg.ros_static_peers == "client.ros2.lan"

    def test_custom_port(self) -> None:
        cfg = BridgeConfig(host="0.0.0.0", port=9091)
        assert cfg.port == 9091

    def test_ros_domain_id_int_coerced_to_str(self) -> None:
        cfg = BridgeConfig(ros_domain_id=0)  # type: ignore[arg-type]
        assert cfg.ros_domain_id == "0"


class TestTabConfig:
    def test_valid_camera_tab(self) -> None:
        tab = TabConfig(id="cam", type="camera", label="Cam", topic="/foo")
        assert tab.type == "camera"

    def test_invalid_type_raises(self) -> None:
        with pytest.raises(Exception):
            TabConfig(id="x", type="unknown", label="X")


class TestAppConfig:
    def test_empty_config_is_valid(self) -> None:
        cfg = AppConfig()
        assert cfg.tabs == []
        assert cfg.overlays == []

    def test_all_subscribed_topics_camera(self) -> None:
        cfg = AppConfig.model_validate(
            {
                "tabs": [{"id": "c", "type": "camera", "label": "C", "topic": "/cam/image"}],
                "overlays": [{"topic": "/gps/fix", "field": "latitude", "label": "Lat"}],
            }
        )
        topics = cfg.all_subscribed_topics()
        assert "/cam/image" in topics
        assert "/gps/fix" in topics

    def test_all_subscribed_topics_graph(self) -> None:
        cfg = AppConfig.model_validate(
            {
                "tabs": [
                    {
                        "id": "g",
                        "type": "sensor_graph",
                        "label": "G",
                        "topics": [{"topic": "/imu/data", "fields": [{"path": "linear_acceleration.x", "label": "X"}]}],
                    }
                ],
                "overlays": [],
            }
        )
        assert "/imu/data" in cfg.all_subscribed_topics()

    def test_publish_topics_includes_goal(self) -> None:
        cfg = AppConfig.model_validate(
            {
                "tabs": [
                    {
                        "id": "n",
                        "type": "nav_local",
                        "label": "Nav",
                        "goal_topic": "/controller/goal_pose",
                    }
                ],
                "overlays": [],
            }
        )
        assert "/controller/goal_pose" in cfg.publish_topics()

    def test_no_duplicate_topics(self) -> None:
        cfg = AppConfig.model_validate(
            {
                "tabs": [
                    {"id": "a", "type": "camera", "label": "A", "topic": "/cam"},
                    {"id": "b", "type": "camera", "label": "B", "topic": "/cam"},
                ],
                "overlays": [],
            }
        )
        topics = cfg.all_subscribed_topics()
        assert topics.count("/cam") == 1


class TestLoadConfig:
    def test_load_valid_file(self, tmp_path: Path) -> None:
        p = _write_config(
            tmp_path,
            {
                "bridge": {"host": "localhost", "port": 9090},
                "tabs": [],
                "overlays": [],
            },
        )
        cfg = load_config(p)
        assert cfg.bridge.port == 9090

    def test_missing_file_raises(self) -> None:
        with pytest.raises(FileNotFoundError):
            load_config(Path("/nonexistent/config.yaml"))

    def test_empty_yaml_returns_defaults(self, tmp_path: Path) -> None:
        p = tmp_path / "config.yaml"
        p.write_text("")
        cfg = load_config(p)
        assert cfg.tabs == []
