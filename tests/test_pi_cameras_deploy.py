"""Static invariants of the Pi camera previews: two Camera Module 3 (IMX708) on the Pi 5 CSI ports.

The pi_cameras node type builds libcamera and camera_ros from pinned sources (the same pins and patches as the retired
overview_camera, so the existing build stamps match and nothing rebuilds), runs in the background tier and publishes
/front_camera/* (and /back_camera/* when the back role is launched). The retired /overview_camera/* names must not be
used. CSI cam1 carries the Arducam ToF camera, so only the front role is deployed and the boot
config (tasks/pi_camera_boot_config.yml) loads the imx708 overlay on cam0 explicitly.
"""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE / "group_vars" / "client.yml"
NODE_DIR = REPO_ROOT / "nodes" / "pi_cameras"
LAUNCH_FILE = NODE_DIR / "launch" / "pi_cameras.launch.py"
README = NODE_DIR / "README.md"
TESTS_README = REPO_ROOT / "tests" / "README.md"
ANSIBLE_README = ANSIBLE / "README.md"
LIBCAMERA_TAG = "v0.7.2+rpt20260817"
CAMERA_ROS_COMMIT = "8f792e27a6dbc81e4943a75765fc1b7b7d37b301"
FORBIDDEN_APT_PACKAGES = ("ros-jazzy-camera-ros", "ros-jazzy-libcamera", "libcamera-dev", "libcamera0")
PREVIEW_TOPICS = ("/front_camera/image_raw/compressed",)
BOOT_TASKS = ANSIBLE / "playbooks" / "tasks" / "pi_camera_boot_config.yml"


def client_vars() -> dict:
    """Load group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def node_type() -> dict:
    """Return the pi_cameras node type defaults.

    Returns:
        dict: ros2_node_type_defaults["pi_cameras"].
    """
    return client_vars()["ros2_node_type_defaults"]["pi_cameras"]


def node_entry() -> dict:
    """Return the pi_cameras ros2_nodes entry.

    Returns:
        dict: The entry.
    """
    return next(n for n in client_vars()["ros2_nodes"] if n["name"] == "pi_cameras")


def test_node_type_builds_libcamera_then_camera_ros_from_the_pinned_sources() -> None:
    sources = node_type()["colcon_source"]
    assert [s["package"] for s in sources] == ["libcamera", "camera_ros"]
    assert sources[0]["commit"] == LIBCAMERA_TAG
    assert sources[1]["commit"] == CAMERA_ROS_COMMIT
    for source in sources:
        assert source["workspace"] == "/opt/ros2-ws"
        for patch in source.get("patches", []):
            relative = patch.replace("{{ ros2_repo_dest }}/", "")
            assert (REPO_ROOT / relative).is_file(), patch
            assert "nodes/pi_cameras/patches/" in patch


def test_node_type_never_installs_the_apt_camera_stack() -> None:
    apt = node_type()["apt_packages"]
    for package in FORBIDDEN_APT_PACKAGES:
        assert package not in apt
    for package in ("ros-jazzy-image-transport-plugins", "ros-jazzy-compressed-image-transport", "meson"):
        assert package in apt


def test_node_type_launches_the_pi_cameras_launch_file() -> None:
    node = node_type()
    assert node["deploy_mode"] == "native"
    assert node["node_launch_command"].endswith("nodes/pi_cameras/launch/pi_cameras.launch.py")
    assert "PI_CAMERAS_CONFIG=/etc/ros2/pi_cameras/config.yaml" in node["env"]
    assert node["config_path"] == "/etc/ros2/pi_cameras"
    assert node["src_paths"] == ["nodes/pi_cameras"]


def test_node_type_is_a_background_preview_that_loses_to_the_robot() -> None:
    node = node_type()
    assert (node["cpu_weight"], node["io_weight"], node["oom_score_adjust"]) == (100, 100, 0)
    assert node["nice"] >= 5


def test_node_entry_is_present_and_enabled_with_the_front_port_documented() -> None:
    entry = node_entry()
    assert entry["node_type"] == "pi_cameras"
    assert entry["present"] is True and entry["enabled"] is True
    config = yaml.safe_load(entry["config"])
    assert config["front_port"] == "cam0"


def test_node_entry_launches_only_the_front_camera_while_cam1_carries_the_tof() -> None:
    assert yaml.safe_load(node_entry()["config"])["roles"] == ["front"]


def test_entry_comment_documents_the_port_to_role_mapping() -> None:
    lines = CLIENT_VARS.read_text().splitlines()
    index = lines.index("  - name: pi_cameras")
    comment = []
    while lines[index - 1].lstrip().startswith("#"):
        index -= 1
        comment.append(lines[index])
    text = " ".join(comment)
    assert "front_port" in text and "cam0" in text


def test_web_ui_shows_the_previews_not_the_overview_camera_names() -> None:
    web_ui = next(n for n in client_vars()["ros2_nodes"] if n["name"] == "web_ui")
    config = yaml.safe_load(web_ui["config"])
    tab = next(t for t in config["tabs"] if t["id"] == "pi_cameras")
    assert tab["type"] == "camera"
    assert [spec["topic"] for spec in tab["topics"]] == list(PREVIEW_TOPICS)
    assert [spec["label"] for spec in tab["topics"]] == ["Front"]


def test_boot_config_loads_the_imx708_overlay_on_cam0() -> None:
    tasks = yaml.safe_load(BOOT_TASKS.read_text())
    lines = [t["ansible.builtin.lineinfile"] for t in tasks if "ansible.builtin.lineinfile" in t]
    assert any(m.get("line") == "dtoverlay=imx708,cam0" for m in lines)


def test_launch_file_uses_the_pi_cameras_namespaces_only() -> None:
    text = LAUNCH_FILE.read_text()
    assert "overview_camera" not in text
    assert "camera_ros" in text and "component_container_mt" in text


def test_docs_describe_the_previews_and_the_front_port() -> None:
    readme = README.read_text()
    for needle in (
        "/front_camera/image_raw/compressed",
        "/back_camera/image_raw/compressed",
        "front_port",
        "roles",
        "cam0",
    ):
        assert needle in readme, needle
    assert "pi_cameras" in ANSIBLE_README.read_text()
    assert "test_pi_cameras_deploy.py" in TESTS_README.read_text()
    assert "test_pi_cameras_config.py" in TESTS_README.read_text()
