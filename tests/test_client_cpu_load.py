"""Client CPU load trims (2026-10-08: load ~13 on 4 cores, the EKF missed its rate and the map pose went stale)."""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"


def client_node(name: str, text: str | None = None) -> dict:
    """Return one ros2_nodes entry of group_vars/client.yml.

    Args:
        name (str): Node name.
        text (str | None): YAML text to parse instead of the file.

    Returns:
        dict: The node entry.
    """
    nodes = yaml.safe_load(text if text is not None else CLIENT_VARS.read_text())["ros2_nodes"]
    return next(n for n in nodes if n["name"] == name)


def test_master2master_is_disabled() -> None:
    """Its /controller/* relays (the gripper camera included) served only the legacy Steam Deck UI."""
    node = client_node("master2master")
    assert node["present"] is True and node["enabled"] is False


def test_gripper_camera_publish_rate_is_capped() -> None:
    """The UVC camera delivered about 18 fps; 10 fps is enough for the agent and the web UI."""
    env = client_node("gripper_uvc_camera")["env"]
    assert "UVC_MAX_FPS=10" in env
