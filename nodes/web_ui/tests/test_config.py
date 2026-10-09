"""Tests for web_ui config loading."""

from __future__ import annotations

from pathlib import Path

import pytest
from pydantic import ValidationError

from web_ui.config import AppConfig, TabConfig, load_config


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
    type: camera
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
    type: camera
    label: Nav
    scan_topic: /controller/scan
    goal_topic: /controller/goal_pose
overlays: []
"""
    )
    cfg = load_config(p)
    assert "/controller/goal_pose" in cfg.publish_topics()


def test_rgbd_camera_tab_type_is_valid(tmp_path: Path) -> None:
    """rgbd_camera is a valid tab type."""
    p = tmp_path / "cfg.yaml"
    p.write_text(
        """
tabs:
  - id: rgbd
    type: rgbd_camera
    label: RGBD Cam
    color_topic: /camera/camera/color/image_raw
    depth_topic: /camera/camera/depth/image_rect_raw
    camera_info_topic: /camera/camera/depth/camera_info
overlays: []
"""
    )
    cfg = load_config(p)
    assert cfg.tabs[0].type == "rgbd_camera"
    assert cfg.tabs[0].color_topic == "/camera/camera/color/image_raw"
    assert cfg.tabs[0].depth_topic == "/camera/camera/depth/image_rect_raw"
    assert cfg.tabs[0].camera_info_topic == "/camera/camera/depth/camera_info"


def test_rgbd_topics_in_all_subscribed_topics(tmp_path: Path) -> None:
    """color_topic, depth_topic, camera_info_topic are included in subscribed topics."""
    p = tmp_path / "cfg.yaml"
    p.write_text(
        """
tabs:
  - id: rgbd
    type: rgbd_camera
    label: RGBD Cam
    color_topic: /camera/camera/color/image_raw
    depth_topic: /camera/camera/depth/image_rect_raw
    camera_info_topic: /camera/camera/depth/camera_info
overlays: []
"""
    )
    cfg = load_config(p)
    topics = cfg.all_subscribed_topics()
    assert "/camera/camera/color/image_raw" in topics
    assert "/camera/camera/depth/image_rect_raw" in topics
    assert "/camera/camera/depth/camera_info" in topics


def test_default_yaml_overview_tab_is_a_plain_camera_tab() -> None:
    """The shipped default config has an Overview camera tab on the compressed overhead camera and no RGBD tab."""
    cfg = load_config(Path(__file__).resolve().parents[1] / "config" / "default.yaml")
    assert not [t for t in cfg.tabs if t.type == "rgbd_camera"]
    tab = next(t for t in cfg.tabs if t.id == "overview_camera")
    assert tab.type == "camera"
    assert tab.topic == "/overview_camera/image_raw/compressed"
    assert tab.label == "Overview"
    assert "/overview_camera/image_raw/compressed" in cfg.all_subscribed_topics()


def test_gps_status_absent_means_off() -> None:
    cfg = AppConfig()
    assert cfg.gps_status is None
    assert cfg.topic_roles() == {}


def test_gps_status_defaults_and_roles_and_api_dump() -> None:
    cfg = AppConfig.model_validate(
        {"gps_status": {"rover_topic": "/client/gps/status", "base_url": "http://s:18100/x"}}
    )
    gps = cfg.gps_status
    assert gps is not None
    assert (gps.base_poll_hz, gps.stale_after_s, gps.base_timeout_s) == (1.0, 5.0, 2.0)
    assert cfg.topic_roles() == {"/client/gps/status": "gps_status"}
    assert "/client/gps/status" in cfg.all_subscribed_topics()
    assert cfg.model_dump()["gps_status"]["base_url"] == "http://s:18100/x"


def test_gps_status_all_optional() -> None:
    gps = AppConfig.model_validate({"gps_status": {}}).gps_status
    assert gps is not None
    assert gps.rover_topic is None and gps.base_url is None


@pytest.mark.parametrize("field", ["base_poll_hz", "stale_after_s", "base_timeout_s"])
def test_gps_status_positive_numbers(field: str) -> None:
    with pytest.raises(ValidationError):
        AppConfig.model_validate({"gps_status": {field: 0}})


def test_map_nav_compass_anchor_defaults() -> None:
    tab = TabConfig(id="m", type="map_nav", label="Map")
    assert tab.gps_anchor_compass is True
    assert tab.gps_anchor_imu_topic is None
    assert tab.gps_anchor_imu_calibration_topic is None
    assert tab.magnetic_declination_deg == 0.0
    assert tab.imu_yaw_offset_deg == 0.0
    assert AppConfig(tabs=[tab]).gps_compass_settings() is None


def test_gps_compass_settings_from_config() -> None:
    tab = TabConfig(
        id="m",
        type="map_nav",
        label="Map",
        gps_anchor_imu_topic="/imu/data",
        gps_anchor_imu_calibration_topic="/imu/calibration",
        magnetic_declination_deg=6.6,
        imu_yaw_offset_deg=-90.0,
    )
    settings = AppConfig(tabs=[tab]).gps_compass_settings()
    assert settings is not None
    assert (settings.imu_topic, settings.calibration_topic) == ("/imu/data", "/imu/calibration")
    assert (settings.declination_deg, settings.imu_yaw_offset_deg) == (6.6, -90.0)


def test_gps_compass_settings_none_when_disabled_or_without_map_nav_tab() -> None:
    off = TabConfig(id="m", type="map_nav", label="Map", gps_anchor_imu_topic="/imu/data", gps_anchor_compass=False)
    assert AppConfig(tabs=[off]).gps_compass_settings() is None
    assert AppConfig(tabs=[]).gps_compass_settings() is None
