"""Tests for web_ui config loading."""

from __future__ import annotations

from pathlib import Path

import pytest
from pydantic import ValidationError

from web_ui.config import VALID_TAB_TYPES, AppConfig, CalibrationConfig, GpsStatusConfig, TabConfig, load_config


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


PREVIEW_TOPICS = ["/front_camera/image_raw/compressed", "/back_camera/image_raw/compressed"]


def test_camera_tab_topics_carry_labels_and_are_subscribed(tmp_path: Path) -> None:
    """A camera tab with `topics` (topic + label) subscribes every topic and keeps the labels in order."""
    p = tmp_path / "cfg.yaml"
    p.write_text(
        """
tabs:
  - id: pi_cameras
    type: camera
    label: Front/Back
    topics:
      - {topic: /front_camera/image_raw/compressed, label: Front}
      - {topic: /back_camera/image_raw/compressed, label: Back}
"""
    )
    cfg = load_config(p)
    assert [(t.topic, t.label) for t in cfg.tabs[0].topics] == list(zip(PREVIEW_TOPICS, ["Front", "Back"]))
    assert set(PREVIEW_TOPICS) <= set(cfg.all_subscribed_topics())


def test_topic_spec_label_is_optional() -> None:
    assert TabConfig(id="g", type="sensor_graph", label="G", topics=[{"topic": "/x"}]).topics[0].label is None


def test_default_yaml_has_the_front_back_preview_tab() -> None:
    cfg = load_config(Path(__file__).resolve().parents[1] / "config" / "default.yaml")
    tab = next(t for t in cfg.tabs if t.id == "pi_cameras")
    assert tab.type == "camera"
    assert [(t.topic, t.label) for t in tab.topics] == list(zip(PREVIEW_TOPICS, ["Front", "Back"]))
    assert tab.topic is None


def test_gps_status_absent_means_off() -> None:
    cfg = AppConfig()
    assert cfg.gps_status is None
    assert cfg.topic_roles() == {}


def test_gps_status_defaults_and_roles_and_api_dump() -> None:
    cfg = AppConfig.model_validate(
        {"gps_status": {"rover_topic": "/client/gps/status", "base_url": "http://s:5016/status"}}
    )
    gps = cfg.gps_status
    assert gps is not None
    assert (gps.base_poll_hz, gps.stale_after_s, gps.base_timeout_s) == (1.0, 5.0, 2.0)
    assert cfg.topic_roles() == {"/client/gps/status": "gps_status"}
    assert "/client/gps/status" in cfg.all_subscribed_topics()
    assert cfg.model_dump()["gps_status"]["base_url"] == "http://s:5016/status"


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


def test_default_yaml_gps_status_points_at_base_caster_status_route() -> None:
    """default.yaml enables both GPS chips; the base status comes from the base node's NTRIP caster GET /status."""
    cfg = load_config(Path(__file__).resolve().parents[1] / "config" / "default.yaml")
    gps = cfg.gps_status
    assert gps is not None
    assert gps.rover_topic == "/client/gps/status"
    assert gps.base_url == "http://server.ros2.lan:5016/status"
    defaults = GpsStatusConfig()
    assert (gps.base_poll_hz, gps.stale_after_s, gps.base_timeout_s) == (
        defaults.base_poll_hz,
        defaults.stale_after_s,
        defaults.base_timeout_s,
    )


def test_arm_is_a_valid_tab_type() -> None:
    assert "arm" in VALID_TAB_TYPES


def test_arm_tab_defaults() -> None:
    tab = TabConfig(id="arm", type="arm", label="Arm")
    assert tab.gripper_camera_topic == "/camera_0/image_raw/compressed"
    assert tab.overview_camera_topic == "/overview_camera/image_raw/compressed"
    assert tab.arm_joint_states_topic == "/follower/joint_states"
    assert tab.arm_urdf == "so101_arm.urdf"
    assert tab.arm_offset == (0.0592, -0.05, 0.100)
    assert tab.gripper_camera_mount is not None
    assert tab.gripper_camera_mount.xyz == (-0.0046, 0.0364, -0.0364)
    assert tab.gripper_camera_mount.rpy == (-1.5622, 1.1950, -1.4466)
    assert tab.active_source_topic == "/filter/active_source"
    assert tab.arm_home_service == "/arm/home"
    assert tab.arm_set_home_service == "/arm/set_home"
    assert tab.arm_take_control_service == "/arm/take_control"
    assert tab.arm_release_control_service == "/arm/release_control"


def test_arm_tab_overrides_and_empty_rejected() -> None:
    tab = TabConfig(
        id="arm",
        type="arm",
        label="Arm",
        gripper_camera_topic="/g",
        overview_camera_topic="/o",
        gripper_camera_mount={"xyz": [1, 2, 3], "rpy": [0.1, 0.2, 0.3]},
        arm_offset=(1.0, 2.0, 3.0),
    )
    assert (tab.gripper_camera_topic, tab.overview_camera_topic) == ("/g", "/o")
    assert tab.gripper_camera_mount is not None and tab.gripper_camera_mount.xyz == (1, 2, 3)
    assert tab.arm_offset == (1.0, 2.0, 3.0)
    with pytest.raises(ValidationError):
        TabConfig(id="arm", type="arm", label="Arm", gripper_camera_topic=" ")


def test_other_tabs_get_no_arm_defaults() -> None:
    tab = TabConfig(id="c", type="camera", label="C", topic="/x")
    assert tab.gripper_camera_topic is None and tab.gripper_camera_mount is None and tab.active_source_topic is None


def test_arm_tab_topics_subscribed_and_services_listed() -> None:
    cfg = AppConfig(tabs=[TabConfig(id="arm", type="arm", label="Arm")])
    topics = cfg.all_subscribed_topics()
    for topic in ("/camera_0/image_raw/compressed", "/overview_camera/image_raw/compressed", "/follower/joint_states"):
        assert topic in topics
    assert cfg.trigger_services() == ["/arm/home", "/arm/release_control", "/arm/set_home", "/arm/take_control"]


def test_default_yaml_arm_tab_replaces_gripper_cam_tab() -> None:
    cfg = load_config(Path(__file__).resolve().parents[1] / "config" / "default.yaml")
    ids = [t.id for t in cfg.tabs]
    assert "camera" not in ids
    assert ids.index("arm") == 1
    assert cfg.tabs[1].type == "arm" and cfg.tabs[1].label == "Arm"
    assert ids.index("head") == 2
    assert ids.index("overview_camera") == 3
    assert cfg.tabs[1].arm_urdf == "so101_arm.urdf"


HEAD_TAB_YAML = """
tabs:
  - id: head
    type: head
    label: Head
    rgb_camera_topic: /head_camera/image_raw/compressed
    tof_depth_topic: /tof/depth/image_raw
    tof_amplitude_topic: /tof/amplitude/image_raw
    tof_range_m: 3.0
overlays: []
"""


def test_head_tab_type_is_valid() -> None:
    """head is a valid tab type."""
    assert "head" in VALID_TAB_TYPES


def test_head_tab_fields_load_and_are_subscribed(tmp_path: Path) -> None:
    """The three head topics load and reach all_subscribed_topics()."""
    p = tmp_path / "cfg.yaml"
    p.write_text(HEAD_TAB_YAML)
    cfg = load_config(p)
    tab = cfg.tabs[0]
    assert tab.rgb_camera_topic == "/head_camera/image_raw/compressed"
    assert tab.tof_depth_topic == "/tof/depth/image_raw"
    assert tab.tof_amplitude_topic == "/tof/amplitude/image_raw"
    assert tab.tof_range_m == 3.0
    topics = cfg.all_subscribed_topics()
    for topic in (tab.rgb_camera_topic, tab.tof_depth_topic, tab.tof_amplitude_topic):
        assert topic in topics


def test_head_tab_topics_default_to_none_and_range_to_4m() -> None:
    """Unset head topics are None and tof_range_m defaults to 4.0."""
    tab = TabConfig(id="h", type="head", label="Head")
    assert tab.rgb_camera_topic is None
    assert tab.tof_depth_topic is None
    assert tab.tof_amplitude_topic is None
    assert tab.tof_range_m == 4.0


@pytest.mark.parametrize("bad", [0, -1.5])
def test_head_tab_rejects_non_positive_range(bad: float) -> None:
    """tof_range_m must be positive."""
    with pytest.raises(ValidationError):
        TabConfig(id="h", type="head", label="Head", tof_range_m=bad)


def test_head_topic_roles_and_ranges(tmp_path: Path) -> None:
    """Head topics get head roles; the depth topic carries its range."""
    p = tmp_path / "cfg.yaml"
    p.write_text(HEAD_TAB_YAML)
    cfg = load_config(p)
    assert cfg.topic_roles() == {
        "/head_camera/image_raw/compressed": "head_rgb",
        "/tof/depth/image_raw": "head_depth",
        "/tof/amplitude/image_raw": "head_amplitude",
    }
    assert cfg.tof_ranges() == {"/tof/depth/image_raw": 3.0}


def test_head_tab_has_no_fused_depth_topic(tmp_path: Path) -> None:
    """fused_depth_topic is not a field; a YAML key of that name is ignored (not subscribed, no role)."""
    assert "fused_depth_topic" not in TabConfig.model_fields
    p = tmp_path / "cfg.yaml"
    p.write_text(
        HEAD_TAB_YAML.replace("    tof_range_m", "    fused_depth_topic: /stereo/depth/fused\n    tof_range_m")
    )
    cfg = load_config(p)
    assert "/stereo/depth/fused" not in cfg.all_subscribed_topics()
    assert "/stereo/depth/fused" not in cfg.topic_roles()
    assert "/stereo/depth/fused" not in cfg.tof_ranges()


# ---------------------------------------------------------------- calibration


def test_calibration_absent_by_default() -> None:
    assert AppConfig().calibration is None


def test_calibration_config_needs_no_depthd_url() -> None:
    cfg = AppConfig.model_validate({"calibration": {}})
    cal = cfg.calibration
    assert cal is not None
    assert CalibrationConfig() == cal
    assert "depthd_url" not in CalibrationConfig.model_fields
    assert cal.imu_status_topic == "/imu/calibration"
    assert cal.imu_save_service == "/imu/calibration/save"
    assert cal.imu_reset_service == "/imu/calibration/reset"
    assert cal.imu_stale_s == 3.0
    assert cal.imu_service_timeout_s == 5.0


@pytest.mark.parametrize("bad", [{"imu_stale_s": 0}, {"imu_service_timeout_s": 0}, {"imu_service_timeout_s": -1}])
def test_calibration_rejections(bad: dict) -> None:
    with pytest.raises(ValidationError):
        AppConfig.model_validate({"calibration": bad})
