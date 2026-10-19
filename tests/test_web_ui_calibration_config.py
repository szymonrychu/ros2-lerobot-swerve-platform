"""Wiring of the web_ui Calibrate page in Ansible group_vars: the IMU section talks to the live bno055_imu names."""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
# bno055_imu defaults (nodes/bridges/bno055_imu/bno055_imu/config.py); client.yml does not override them
IMU_STATUS_TOPIC = "/imu/calibration"
IMU_SAVE_SERVICE = "/imu/calibration/save"
IMU_RESET_SERVICE = "/imu/calibration/reset"


def node_config(name: str) -> dict:
    """Parse the `config: |` block of one client ros2_nodes entry.

    Args:
        name: Node name.

    Returns:
        dict: Parsed config YAML of the node.
    """
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    return yaml.safe_load(next(n for n in nodes if n["name"] == name)["config"]) or {}


def test_web_ui_enables_the_imu_calibration_section() -> None:
    """Without a calibration block GET /api/calibration/config answers "calibration page not configured"."""
    calibration = node_config("web_ui").get("calibration")
    assert calibration is not None
    assert calibration["imu_status_topic"] == IMU_STATUS_TOPIC
    assert calibration["imu_save_service"] == IMU_SAVE_SERVICE
    assert calibration["imu_reset_service"] == IMU_RESET_SERVICE


def test_web_ui_rgbd_url_targets_the_calibration_node_port() -> None:
    """The RGB-D section proxies to head_rgbd_calibration on loopback at its http_port."""
    calibration = node_config("web_ui")["calibration"]
    port = node_config("head_rgbd_calibration")["http_port"]
    assert calibration["rgbd_url"] == f"http://127.0.0.1:{port}"


def test_bno055_keeps_the_default_calibration_names() -> None:
    """The bno055_imu entry does not rename the topic or services the web_ui section uses."""
    imu = node_config("bno055_imu")
    for key in ("calibration_topic", "save_calibration_service", "reset_calibration_service"):
        assert key not in imu
