"""Deploy wiring of the rtabmap_mapping node type on the server (D1, D8): node type fields, apt list, caps, database
directory task and the playbook steps."""

from pathlib import Path

import yaml

ANSIBLE_DIR = Path(__file__).resolve().parent.parent / "ansible"
SERVER_VARS = ANSIBLE_DIR / "group_vars" / "server.yml"
PLAYBOOK = ANSIBLE_DIR / "playbooks" / "deploy_nodes_server.yml"
DB_DIR_TASK = ANSIBLE_DIR / "playbooks" / "tasks" / "rtabmap_db_dir.yml"
NODE = "rtabmap_mapping"
REQUIRED_APT = {
    "ros-jazzy-rtabmap-slam",
    "ros-jazzy-rtabmap-sync",
    "ros-jazzy-rtabmap-util",
    "ros-jazzy-rtabmap-msgs",
    "ros-jazzy-rtabmap-conversions",
    "ros-jazzy-image-transport-plugins",
    "ros-jazzy-rclcpp-components",
    "python3-pydantic",
    "python3-yaml",
}
FORBIDDEN_APT = {"ros-jazzy-rtabmap-ros", "ros-jazzy-rtabmap-viz"}
TIER_KEYS = {"cpu_weight", "io_weight", "oom_score_adjust", "memory_low", "memory_high"}


def server_vars() -> dict:
    return yaml.safe_load(SERVER_VARS.read_text())


def node_type() -> dict:
    return server_vars()["ros2_node_type_defaults"][NODE]


def playbook_tasks() -> list[dict]:
    plays = yaml.safe_load(PLAYBOOK.read_text())
    tasks = [t for play in plays for t in play.get("tasks", [])]
    return [s for t in tasks for s in t.get("block", [t])]


def test_node_type_fields() -> None:
    nt = node_type()
    assert nt["deploy_mode"] == "native"
    launch = "{{ ros2_repo_dest }}/nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py"
    assert nt["node_launch_command"] == f"ros2 launch {launch}"
    assert nt["node_src_dir"] == ""
    assert nt["src_paths"] == ["nodes/rtabmap_mapping"]
    assert nt["config_path"] == "/etc/ros2/rtabmap_mapping"
    assert nt["env"] == ["RTABMAP_MAPPING_CONFIG=/etc/ros2/rtabmap_mapping/config.yaml"]


def test_apt_list_and_forbidden_packages() -> None:
    apt = set(node_type()["apt_packages"])
    assert apt == REQUIRED_APT
    assert not apt & FORBIDDEN_APT


def test_quota_and_cap_without_tier_keys() -> None:
    nt = node_type()
    assert nt["cpu_quota"] == "200%"
    assert nt["memory_max"] == "900M"
    assert not TIER_KEYS & set(nt)


def test_no_static_peer() -> None:
    entry = next(n for n in server_vars()["ros2_nodes"] if n["name"] == NODE)
    envs = node_type().get("env", []) + entry.get("env", [])
    assert not any("ROS_STATIC_PEERS" in e for e in envs)


def test_ros2_nodes_entry() -> None:
    entries = [n for n in server_vars()["ros2_nodes"] if n["name"] == NODE]
    assert len(entries) == 1
    entry = entries[0]
    assert entry["node_type"] == NODE and entry["present"] is True and entry["enabled"] is True
    assert not entry.get("config")
    block = SERVER_VARS.read_text().split(f"  - name: {NODE}\n", 1)[1]
    assert block.lstrip().startswith("node_type") and "#" in block.split("\n  - name:")[0]


def test_db_dir_task() -> None:
    (task,) = yaml.safe_load(DB_DIR_TASK.read_text())
    args = task["ansible.builtin.file"]
    assert args["path"] == "/var/lib/ros2/rtabmap"
    assert args["state"] == "directory"
    assert args["owner"] == "{{ ansible_user }}" and args["group"] == "{{ ansible_user }}"
    assert args["mode"] == "0755"


def test_playbook_steps_and_order() -> None:
    flat = playbook_tasks()
    names = [t["name"] for t in flat]
    db = next(t for t in flat if "rtabmap_db_dir.yml" in str(t.get("ansible.builtin.include_tasks", "")))
    assert set(db["tags"]) == {"setup", NODE}
    deploy = next(t for t in flat if t.get("vars", {}).get("_deploy_node_name") == NODE)
    assert deploy["name"] == f"Deploy {NODE}"
    assert set(deploy["tags"]) == {NODE, "apt", "python", "build", "config"}
    assert "resolve_and_deploy.yml" in str(deploy["ansible.builtin.include_tasks"]["file"])
    assert names.index("Deploy gps_rtk_base") < names.index(db["name"]) < names.index(deploy["name"])
