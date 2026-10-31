"""Tests for web_ui config loading."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from web_ui.config import (
    MAP_NAV_DEFAULTS,
    VALID_TAB_TYPES,
    AppConfig,
    CalibrationConfig,
    GpsStatusConfig,
    GroundRecorderConfig,
    TabConfig,
    load_config,
)


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


def test_publish_topic_types(tmp_path: Path) -> None:
    """Goal topics are PoseStamped; arm and head command topics are JointState; map3d commands are not listed."""
    p = tmp_path / "cfg.yaml"
    p.write_text(
        """
tabs:
  - id: nav
    type: map_nav
    label: Nav
    goal_topic: /goal_pose
    arm_command_topic: /arm/cmd
  - id: head
    type: head
    label: Head
    head_command_topic: /head/cmd
overlays: []
"""
    )
    cfg = load_config(p)
    types = cfg.publish_topic_types()
    assert types["/goal_pose"] == "geometry_msgs/PoseStamped"
    assert types["/arm/cmd"] == "sensor_msgs/JointState"
    assert types["/head/cmd"] == "sensor_msgs/JointState"
    assert set(types) <= set(cfg.publish_topics())


def test_publish_topic_types_empty_without_command_fields(tmp_path: Path) -> None:
    p = tmp_path / "cfg.yaml"
    p.write_text("tabs: []\noverlays: []\n")
    assert load_config(p).publish_topic_types() == {}


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


def test_default_config_has_no_overview_tab() -> None:
    """The shipped default config has no overview_camera tab, subscribes no /overview_camera/* topic and no RGBD tab."""
    cfg = load_config(Path(__file__).resolve().parents[1] / "config" / "default.yaml")
    assert not [t for t in cfg.tabs if t.type == "rgbd_camera"]
    assert not [t for t in cfg.tabs if t.id == "overview_camera"]
    assert not [t for t in cfg.all_subscribed_topics() if t.startswith("/overview_camera/")]
    assert "pi_cameras" not in [t.id for t in cfg.tabs]


PREVIEW_TOPICS = ["/cam_a/image_raw/compressed", "/cam_b/image_raw/compressed"]


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
      - {topic: /cam_a/image_raw/compressed, label: Front}
      - {topic: /cam_b/image_raw/compressed, label: Back}
"""
    )
    cfg = load_config(p)
    assert [(t.topic, t.label) for t in cfg.tabs[0].topics] == list(zip(PREVIEW_TOPICS, ["Front", "Back"]))
    assert set(PREVIEW_TOPICS) <= set(cfg.all_subscribed_topics())


def test_topic_spec_label_is_optional() -> None:
    assert TabConfig(id="g", type="sensor_graph", label="G", topics=[{"topic": "/x"}]).topics[0].label is None


def test_default_yaml_has_no_pi_cameras_tab() -> None:
    """The retired Front/Back preview tab is gone from the shipped default config."""
    cfg = load_config(Path(__file__).resolve().parents[1] / "config" / "default.yaml")
    assert "pi_cameras" not in [t.id for t in cfg.tabs]
    assert not [t for t in cfg.tabs if t.type == "camera"]


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
    assert tab.head_camera_topic == "/head/d435/color/image_raw/compressed"
    assert tab.arm_joint_states_topic == "/follower/joint_states"
    assert tab.arm_urdf == "so101_arm.urdf"
    assert tab.arm_offset == (0.0592, -0.05, 0.100)
    assert tab.active_source_topic == "/filter/active_source"
    assert tab.arm_home_service == "/arm/home"
    assert tab.arm_set_home_service == "/arm/set_home"
    assert tab.arm_take_control_service == "/arm/take_control"
    assert tab.arm_release_control_service == "/arm/release_control"
    assert tab.arm_command_topic == "/filter/web_ui_joint_commands"


def test_arm_tab_has_no_gripper_camera() -> None:
    """The gripper camera is retired: its arm tab keys are rejected and the head camera is the D435 colour stream."""
    for key, value in (
        ("gripper_camera_topic", "/camera_0/image_raw/compressed"),
        ("gripper_camera_mount", {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}),
    ):
        with pytest.raises(ValidationError, match=key):
            TabConfig.model_validate({"id": "arm", "type": "arm", "label": "Arm", key: value})
    assert TabConfig(id="arm", type="arm", label="Arm").head_camera_topic == "/head/d435/color/image_raw/compressed"
    raw = yaml.safe_load((Path(__file__).resolve().parents[1] / "config" / "default.yaml").read_text())
    arm_raw = next(t for t in raw["tabs"] if t["type"] == "arm")
    assert not [k for k in arm_raw if k.startswith("gripper_camera")]
    assert arm_raw["head_camera_topic"] == "/head/d435/color/image_raw/compressed"


def test_map_nav_tab_has_no_arm_command_topic_but_publish_topics_list_the_arm_one() -> None:
    """The MAP tab no longer drives the arm; the allowlist gets the topic from the arm tab."""
    cfg = AppConfig(
        tabs=[TabConfig(id="map", type="map_nav", label="Map"), TabConfig(id="arm", type="arm", label="Arm")]
    )
    assert cfg.tabs[0].arm_command_topic is None
    assert "/filter/web_ui_joint_commands" in cfg.publish_topics()


def test_default_yaml_arm_command_topic_lives_on_the_arm_tab_only() -> None:
    cfg = load_config(Path(__file__).resolve().parents[1] / "config" / "default.yaml")
    holders = [t.type for t in cfg.tabs if t.arm_command_topic]
    assert holders == ["arm"]
    raw = yaml.safe_load((Path(__file__).resolve().parents[1] / "config" / "default.yaml").read_text())
    map_raw = next(t for t in raw["tabs"] if t["type"] == "map_nav")
    arm_raw = next(t for t in raw["tabs"] if t["type"] == "arm")
    assert "arm_command_topic" not in map_raw
    assert arm_raw["arm_command_topic"] == "/filter/web_ui_joint_commands"


def test_default_arm_tab_uses_the_head_camera() -> None:
    """The shipped default config's arm tab streams the head D435 colour through head_camera_topic."""
    cfg = load_config(Path(__file__).resolve().parents[1] / "config" / "default.yaml")
    arm = next(t for t in cfg.tabs if t.type == "arm")
    assert arm.head_camera_topic == "/head/d435/color/image_raw/compressed"
    assert "front_camera" not in "".join(cfg.all_subscribed_topics())


def test_arm_tab_overrides_and_empty_rejected() -> None:
    tab = TabConfig(
        id="arm",
        type="arm",
        label="Arm",
        head_camera_topic="/o",
        arm_offset=(1.0, 2.0, 3.0),
    )
    assert tab.head_camera_topic == "/o"
    assert tab.arm_offset == (1.0, 2.0, 3.0)
    with pytest.raises(ValidationError):
        TabConfig(id="arm", type="arm", label="Arm", head_camera_topic=" ")


def test_other_tabs_get_no_arm_defaults() -> None:
    tab = TabConfig(id="c", type="camera", label="C", topic="/x")
    assert tab.head_camera_topic is None and tab.active_source_topic is None


def test_arm_tab_topics_subscribed_and_services_listed() -> None:
    cfg = AppConfig(tabs=[TabConfig(id="arm", type="arm", label="Arm")])
    topics = cfg.all_subscribed_topics()
    for topic in ("/head/d435/color/image_raw/compressed", "/follower/joint_states"):
        assert topic in topics
    assert not [t for t in topics if t.startswith("/camera_0/")]
    assert not [t for t in topics if t.startswith("/overview_camera/") or t.startswith("/front" + "_camera/")]
    assert cfg.trigger_services() == ["/arm/home", "/arm/release_control", "/arm/set_home", "/arm/take_control"]


def test_default_yaml_arm_tab_replaces_gripper_cam_tab() -> None:
    cfg = load_config(Path(__file__).resolve().parents[1] / "config" / "default.yaml")
    ids = [t.id for t in cfg.tabs]
    assert "camera" not in ids
    assert ids.index("arm") == 1
    assert cfg.tabs[1].type == "arm" and cfg.tabs[1].label == "Arm"
    assert ids.index("head") == 2
    assert "pi_cameras" not in ids
    assert cfg.tabs[1].arm_urdf == "so101_arm.urdf"


HEAD_TAB_YAML = """
tabs:
  - id: head
    type: head
    label: Head
    rgb_camera_topic: /head/d435/color/image_raw/compressed
    depth_topic: /head/d435/depth/image_rect_raw
    depth_range_m: 3.0
overlays: []
"""
D435_RGB = "/head/d435/color/image_raw/compressed"
D435_DEPTH = "/head/d435/depth/image_rect_raw"
RGBD_RGB = "/head_rgbd/rgb/image_raw/compressed"
RGBD_DEPTH = "/head_rgbd/depth/image_raw"
CLIENT_YML = Path(__file__).resolve().parents[3] / "ansible" / "group_vars" / "client.yml"
D435_TOPICS_YAML = (
    Path(__file__).resolve().parents[3] / "nodes" / "bridges" / "realsense_d435" / "config" / "topics.yaml"
)
RETIRED_TOF_KEYS = ("tof_depth_topic", "tof_amplitude_topic", "tof_range_m")


def client_web_ui_config() -> AppConfig:
    """Validate the deployed web_ui config block of ansible/group_vars/client.yml.

    Returns:
        AppConfig: The deployed web_ui config.
    """
    nodes = yaml.safe_load(CLIENT_YML.read_text())["ros2_nodes"]
    node = next(n for n in nodes if n["name"] == "web_ui")
    return AppConfig.model_validate(yaml.safe_load(node["config"]))


def test_client_cameras_tab_lists_front_then_back() -> None:
    """The deployed web_ui config validates; its cameras tab shows the front then the back CSI camera."""
    cfg = client_web_ui_config()
    tab = next(t for t in cfg.tabs if t.id == "cameras")
    assert tab.type == "camera"
    assert [(t.topic, t.label) for t in tab.topics] == [
        ("/front_camera/image_raw/compressed", "Front"),
        ("/back_camera/image_raw/compressed", "Back"),
    ]
    assert {"/front_camera/image_raw/compressed", "/back_camera/image_raw/compressed"} <= set(
        cfg.all_subscribed_topics()
    )


def test_head_tab_type_is_valid() -> None:
    """head is a valid tab type."""
    assert "head" in VALID_TAB_TYPES


def test_head_tab_fields_load_and_are_subscribed(tmp_path: Path) -> None:
    """The head RGB and depth topics load and reach all_subscribed_topics()."""
    p = tmp_path / "cfg.yaml"
    p.write_text(HEAD_TAB_YAML)
    cfg = load_config(p)
    tab = cfg.tabs[0]
    assert tab.rgb_camera_topic == D435_RGB
    assert tab.depth_topic == D435_DEPTH
    assert tab.depth_range_m == 3.0
    topics = cfg.all_subscribed_topics()
    for topic in (tab.rgb_camera_topic, tab.depth_topic):
        assert topic in topics


def test_head_tab_topics_default_to_none_and_range_to_4m() -> None:
    """Unset head topics are None and depth_range_m defaults to 4.0."""
    tab = TabConfig(id="h", type="head", label="Head")
    assert tab.rgb_camera_topic is None
    assert tab.depth_topic is None
    assert tab.depth_range_m == 4.0


@pytest.mark.parametrize("bad", [0, -1.5])
def test_head_tab_rejects_non_positive_range(bad: float) -> None:
    """depth_range_m must be positive."""
    with pytest.raises(ValidationError):
        TabConfig(id="h", type="head", label="Head", depth_range_m=bad)


def test_head_tab_depth_keys_replace_tof_keys(tmp_path: Path) -> None:
    """depth_topic and depth_range_m are the head tab keys; the ToF keys are not fields and are rejected as unknown."""
    assert {"depth_topic", "depth_range_m"} <= set(TabConfig.model_fields)
    assert not set(RETIRED_TOF_KEYS) & set(TabConfig.model_fields)
    for key, value in (
        ("tof_depth_topic", "/tof/depth/image_raw"),
        ("tof_amplitude_topic", "/tof/amplitude/image_raw"),
        ("tof_range_m", "4.0"),
    ):
        p = tmp_path / f"{key}.yaml"
        p.write_text(HEAD_TAB_YAML.replace("    depth_range_m", f"    {key}: {value}\n    depth_range_m"))
        with pytest.raises(ValidationError, match=key):
            load_config(p)


def test_head_topic_roles_and_ranges(tmp_path: Path) -> None:
    """Head topics get head roles; the depth topic carries its depth_range_m."""
    p = tmp_path / "cfg.yaml"
    p.write_text(HEAD_TAB_YAML)
    cfg = load_config(p)
    assert cfg.topic_roles() == {D435_RGB: "head_rgb", D435_DEPTH: "head_depth"}
    assert cfg.depth_ranges() == {D435_DEPTH: 3.0}


def test_head_tab_rejects_fused_depth_topic(tmp_path: Path) -> None:
    """fused_depth_topic is not a field; a YAML key of that name is rejected as unknown, never subscribed."""
    assert "fused_depth_topic" not in TabConfig.model_fields
    p = tmp_path / "cfg.yaml"
    p.write_text(
        HEAD_TAB_YAML.replace("    depth_range_m", "    fused_depth_topic: /stereo/depth/fused\n    depth_range_m")
    )
    with pytest.raises(ValidationError, match="fused_depth_topic"):
        load_config(p)


def test_client_head_tab_reads_the_d435() -> None:
    """The deployed head tab reads the D435 colour and depth (live pinned topics) and the /head_rgbd pair."""
    cfg = client_web_ui_config()
    head = next(t for t in cfg.tabs if t.type == "head")
    assert head.rgb_camera_topic == D435_RGB
    assert head.depth_topic == D435_DEPTH
    assert (head.rgbd_rgb_topic, head.rgbd_depth_topic) == (RGBD_RGB, RGBD_DEPTH)
    assert head.depth_range_m == 4.0
    live = yaml.safe_load(D435_TOPICS_YAML.read_text())["topics"]
    assert live[D435_RGB]["type"] == "sensor_msgs/msg/CompressedImage"
    assert live[D435_DEPTH]["type"] == "sensor_msgs/msg/Image"
    assert live[D435_RGB].get("published", True) and live[D435_DEPTH].get("published", True)
    roles = cfg.topic_roles()
    assert {t: roles[t] for t in (D435_RGB, D435_DEPTH, RGBD_RGB, RGBD_DEPTH)} == {
        D435_RGB: "head_rgb",
        D435_DEPTH: "head_depth",
        RGBD_RGB: "head_rgbd_rgb",
        RGBD_DEPTH: "head_rgbd_depth",
    }
    assert cfg.depth_ranges() == {D435_DEPTH: 4.0, RGBD_DEPTH: 4.0}
    assert not [v for v in head.model_dump().values() if isinstance(v, str) and v.startswith("/tof/")]


D435_OBSTACLE_CLOUD = "/head/d435/points_filtered"
LASER_FILTER_YAML = Path(__file__).resolve().parents[3] / "nodes" / "laser_filter" / "config" / "footprint_filter.yaml"


def test_obstacle_cloud_topic_is_the_d435_cloud() -> None:
    """The map tab obstacle cloud is the laser_filter D435 cloud: the default, default.yaml and the deployed value."""
    assert MAP_NAV_DEFAULTS["obstacle_cloud_topic"] == D435_OBSTACLE_CLOUD
    assert TabConfig(id="m", type="map_nav", label="Map").obstacle_cloud_topic == D435_OBSTACLE_CLOUD
    assert yaml.safe_load(LASER_FILTER_YAML.read_text())["cloud_filtered_topic"] == D435_OBSTACLE_CLOUD
    shipped = load_config(Path(__file__).resolve().parents[1] / "config" / "default.yaml")
    for cfg in (shipped, client_web_ui_config()):
        tab = next(t for t in cfg.tabs if t.type == "map_nav")
        assert tab.obstacle_cloud_topic == D435_OBSTACLE_CLOUD
        assert cfg.topic_roles()[D435_OBSTACLE_CLOUD] == "cloud"
        assert not [t for t in cfg.topic_roles() if t.startswith("/tof/")]


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


def test_default_head_tab_subscribes_the_d435_topics() -> None:
    """The shipped default head tab subscribes the D435 colour (head_rgb), D435 depth (head_depth) and the RGB-D pair."""
    cfg = load_config(Path(__file__).resolve().parents[1] / "config" / "default.yaml")
    head = next(t for t in cfg.tabs if t.type == "head")
    roles = cfg.topic_roles()
    head_roles = {topic: role for topic, role in roles.items() if role.startswith("head_")}
    assert head_roles == {
        D435_RGB: "head_rgb",
        D435_DEPTH: "head_depth",
        RGBD_RGB: "head_rgbd_rgb",
        RGBD_DEPTH: "head_rgbd_depth",
    }
    assert (head.rgb_camera_topic, head.depth_topic) == (D435_RGB, D435_DEPTH)
    assert (head.rgbd_rgb_topic, head.rgbd_depth_topic) == (RGBD_RGB, RGBD_DEPTH)
    assert cfg.depth_ranges() == {D435_DEPTH: 4.0, RGBD_DEPTH: 4.0}


AGENT_CHAT_TAB = {"id": "agent", "type": "agent_chat", "label": "Agent", "agent_url": "http://127.0.0.1:18300"}
RETROS_TAB = {"id": "retros", "type": "agent_retros", "label": "Retros"}


def test_agent_retros_is_a_valid_tab_type() -> None:
    assert "agent_retros" in VALID_TAB_TYPES


def test_agent_retros_with_agent_chat_tab_is_valid_and_uses_its_url() -> None:
    cfg = AppConfig.model_validate({"tabs": [AGENT_CHAT_TAB, RETROS_TAB]})
    assert [t.type for t in cfg.tabs] == ["agent_chat", "agent_retros"]
    assert cfg.agent_chat_tab().agent_url == "http://127.0.0.1:18300"


def test_agent_retros_without_agent_chat_tab_is_rejected_with_reason() -> None:
    with pytest.raises(ValidationError, match="agent_retros.*requires an agent_chat tab"):
        AppConfig.model_validate({"tabs": [RETROS_TAB]})


def test_ground_recorder_defaults_and_ranges() -> None:
    cfg = AppConfig().ground_recorder
    assert (cfg.enabled, cfg.dir, cfg.max_duration_s, cfg.max_files, cfg.tf_timeout_s) == (
        False,
        "/tmp/web_ui_ground_recordings",
        30,
        4,
        0.5,
    )
    assert cfg.cloud_topic == "/head/d435/points_filtered"
    assert cfg.depth_info_topic == "/head/d435/depth/camera_info"
    assert cfg.pair_depth_topic == "/head_rgbd/depth/image_raw"
    assert cfg.pair_info_topic == "/head_rgbd/depth/camera_info"
    assert cfg.pair_rgb_topic == "/head_rgbd/rgb/image_raw/compressed"
    assert (cfg.imu_topic, cfg.head_js_topic, cfg.state_topic) == ("/imu/data", "/head/joint_states", "/ground/state")
    assert (cfg.base_frame, cfg.odom_frame) == ("base_link", "odom")
    for bad in (
        {"max_duration_s": 0.5},
        {"max_duration_s": 61},
        {"max_files": 0},
        {"max_files": 21},
        {"tf_timeout_s": -0.1},
        {"tf_timeout_s": 2.5},
        {"unknown_key": 1},
    ):
        with pytest.raises(ValidationError):
            GroundRecorderConfig(**bad)
    assert GroundRecorderConfig(max_duration_s=60, max_files=20, tf_timeout_s=0).max_files == 20
    with pytest.raises(ValidationError):
        AppConfig.model_validate({"ground_recorder": {"bogus": 1}})
