"""Tests for the realsense_d435 node: the launch line, the parameter file, the client instance and the playbook step.

The D435 (device serial 935422070045) runs as the realsense2_camera node from rs_launch.py. Only the camera identity
is a launch argument (rs_launch.py builds the node name from it); every other parameter is in the node's config file.
"""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
CLIENT_PLAYBOOK = ANSIBLE_DIR / "playbooks" / "deploy_nodes_client.yml"
NODE_DIR = "nodes/bridges/realsense_d435"
CONFIG_FILE = REPO_ROOT / NODE_DIR / "config" / "realsense_d435.yaml"
NODE_TYPE = "realsense_d435"
OLD_NAME = "realsense_d435i"
# realsense2_camera matches serial_no against the device serial number (rs-enumerate-devices "Serial Number"), not
# the ASIC serial / firmware update id 935723024187 that lsusb and rs-fw-update -s show.
SERIAL = "935422070045"
EXPECTED_LAUNCH_COMMAND = (
    "ros2 launch realsense2_camera rs_launch.py camera_namespace:=head camera_name:=d435 "
    f"serial_no:=_{SERIAL} config_file:={{{{ ros2_repo_dest }}}}/{NODE_DIR}/config/realsense_d435.yaml"
)
# NORMAL tier of tests/test_unit_cgroups.py: CPUWeight, IOWeight, OOMScoreAdjust.
NORMAL_WEIGHTS = (500, 500, -500)


def load(path: Path) -> object:
    """Load a YAML file.

    Args:
        path (Path): YAML file.

    Returns:
        object: Parsed document.
    """
    return yaml.safe_load(path.read_text())


def node_type_defaults() -> dict:
    """Return the client ros2_node_type_defaults.

    Returns:
        dict: node_type -> defaults.
    """
    return load(CLIENT_VARS)["ros2_node_type_defaults"]


def client_nodes() -> list[dict]:
    """Return the client ros2_nodes instances.

    Returns:
        list[dict]: Node instances.
    """
    return load(CLIENT_VARS)["ros2_nodes"]


def playbook_tasks(tasks: list[dict]) -> list[dict]:
    """Flatten a task list, descending into block, rescue and always sections.

    Args:
        tasks (list[dict]): Tasks of a play section.

    Returns:
        list[dict]: Every task, nested ones included.
    """
    flat = []
    for task in tasks:
        flat.append(task)
        for section in ("block", "rescue", "always"):
            flat.extend(playbook_tasks(task.get(section, [])))
    return flat


def test_launch_command_names_the_camera_and_config_file() -> None:
    """The launch line names namespace head, camera d435, the serial and the config file; no IMU arguments."""
    entry = node_type_defaults()[NODE_TYPE]
    command = entry["node_launch_command"]
    assert command == EXPECTED_LAUNCH_COMMAND
    assert "enable_accel" not in command and "enable_gyro" not in command
    assert entry["deploy_mode"] == "native"
    assert entry["src_paths"] == [NODE_DIR]
    assert entry["apt_packages"] == ["ros-jazzy-realsense2-camera"]


def test_profile_is_hardware_synced_and_minimal() -> None:
    """Depth and colour share one fps (RGB-depth hardware sync), align on, pointcloud and infra off, no TF."""
    params = load(CONFIG_FILE)
    depth = params["depth_module.depth_profile"]
    colour = params["rgb_camera.color_profile"]
    assert depth == "640x360x15"
    assert colour == "640x360x15"
    assert depth.split("x")[2] == colour.split("x")[2]
    assert params["align_depth.enable"] is True
    assert params["pointcloud.enable"] is False
    assert params["enable_infra1"] is False
    assert params["enable_infra2"] is False
    assert params["enable_color"] is True
    assert params["enable_depth"] is True
    assert params["publish_tf"] is False
    assert not any(key in params for key in ("enable_accel", "enable_gyro", "unite_imu_method"))


def test_profile_comments_every_key() -> None:
    """Every parameter line of the config file carries a comment on the line above it."""
    lines = CONFIG_FILE.read_text().splitlines()
    for number, line in enumerate(lines):
        if line and not line.startswith("#"):
            assert number > 0 and lines[number - 1].startswith("#"), line


def test_instance_enabled_and_old_name_gone() -> None:
    """realsense_d435 is present and enabled in the normal tier; no realsense_d435i type, instance or step remains."""
    defaults = node_type_defaults()
    instances = [node for node in client_nodes() if node["name"] == NODE_TYPE]
    assert len(instances) == 1
    instance = instances[0]
    assert instance["node_type"] == NODE_TYPE
    assert instance["present"] is True and instance["enabled"] is True
    entry = defaults[NODE_TYPE]
    assert (entry["cpu_weight"], entry["io_weight"], entry["oom_score_adjust"]) == NORMAL_WEIGHTS
    assert (entry["memory_low"], entry["memory_max"]) == ("128M", "512M")
    assert OLD_NAME not in defaults
    assert all(OLD_NAME not in (node["name"], node["node_type"]) for node in client_nodes())
    assert OLD_NAME not in CLIENT_VARS.read_text()
    assert OLD_NAME not in CLIENT_PLAYBOOK.read_text()


def test_playbook_step_deploys_realsense_d435() -> None:
    """The client playbook step is named, tagged and parameterised realsense_d435."""
    play = load(CLIENT_PLAYBOOK)[0]
    tasks = playbook_tasks(play["tasks"])
    steps = [task for task in tasks if task.get("name") == f"Deploy {NODE_TYPE}"]
    assert len(steps) == 1
    step = steps[0]
    assert step["ansible.builtin.include_tasks"]["file"] == "tasks/resolve_and_deploy.yml"
    assert step["ansible.builtin.include_tasks"]["apply"]["tags"] == [NODE_TYPE]
    assert step["vars"]["_deploy_node_name"] == NODE_TYPE
    assert step["tags"] == [NODE_TYPE, "apt", "python", "build", "config"]
