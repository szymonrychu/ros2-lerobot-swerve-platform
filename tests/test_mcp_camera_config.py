"""Static invariants of the mcp_server camera tools wiring: calibration directory task, config keys, README contract."""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
SETUP_TASKS = REPO_ROOT / "ansible" / "playbooks" / "tasks" / "mcp_server_setup.yml"
MCP_README = REPO_ROOT / "nodes" / "mcp_server" / "README.md"
CALIBRATION_DIR = "/var/lib/ros2/camera_calibration"
CAMERA_TOOLS = (
    "pixel_to_ground",
    "get_annotated_camera_image",
    "mark_candidate_points",
    "resolve_candidate",
    "capture_calibration_sample",
    "solve_camera_calibration",
    "clear_calibration_samples",
)


def mcp_config() -> dict:
    """Parse the mcp_server `config: |` block of group_vars/client.yml.

    Returns:
        dict: The node config.
    """
    entry = next(
        n
        for n in yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
        if n["name"] == "mcp_server"
    )
    return yaml.safe_load(entry["config"])


def test_setup_tasks_create_the_calibration_directory_owned_by_the_node_user() -> None:
    tasks = yaml.safe_load(SETUP_TASKS.read_text())
    dirs = {
        t["ansible.builtin.file"]["path"]: t["ansible.builtin.file"]
        for t in tasks
        if "ansible.builtin.file" in t
    }
    calib = dirs[CALIBRATION_DIR]
    assert calib["state"] == "directory"
    assert (
        calib["owner"] == "{{ ansible_user }}"
        and calib["group"] == "{{ ansible_user }}"
    )


def test_cameras_config_defaults_to_not_calibrated() -> None:
    cameras = mcp_config()["cameras"]
    assert cameras["calibration_dir"] == CALIBRATION_DIR
    assert set(cameras) == {"calibration_dir", "gripper", "front"}
    for name, parent in (("gripper", "gripper_link"), ("front", "base_link")):
        assert cameras[name] == {
            "parent_frame": parent,
            "intrinsics": None,
            "mount": None,
        }, name


def test_gripper_parent_frame_is_a_link_of_the_arm_urdf() -> None:
    cfg = mcp_config()
    urdf = (REPO_ROOT / cfg["arm"]["urdf_path"]).read_text()
    assert f'<link name="{cfg["cameras"]["gripper"]["parent_frame"]}"' in urdf


def test_arm_reach_keys_are_ordered() -> None:
    arm = mcp_config()["arm"]
    assert 0 < arm["reach_inner_m"] < arm["reach_outer_m"]


def test_readme_documents_the_camera_tools_frames_and_calibration() -> None:
    text = MCP_README.read_text()
    for tool in CAMERA_TOOLS:
        assert f"`{tool}" in text, tool
    for needle in (
        "## Camera tools and calibration",
        "not calibrated",
        "base_in_base_link",
        CALIBRATION_DIR,
        "gripper_link",
        "solve_camera_calibration",
    ):
        assert needle in text, needle
