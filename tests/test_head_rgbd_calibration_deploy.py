"""Static invariants of the head_rgbd_calibration deploy: node type, entry, state dir, seeding, apply units."""

import re
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
SETUP_TASKS = REPO_ROOT / "ansible" / "playbooks" / "tasks" / "head_rgbd_calibration_setup.yml"
TEMPLATES = REPO_ROOT / "ansible" / "playbooks" / "templates"
DEFAULTS = REPO_ROOT / "nodes" / "head_rgbd_calibration" / "config" / "head_rgbd_calibration.yaml"
PORT_LINE = re.compile(r"^\s*(?:http_port|port|metrics_port):\s*(\d+)\s*(?:#.*)?$", re.MULTILINE)
STATE_DIR = "/var/lib/ros2/head_rgbd_calibration"
HTTP_PORT = 18110
TIER = {
    "deploy_mode": "native",
    "node_launch_command": "python3 -m head_rgbd_calibration",
    "node_src_dir": "nodes/head_rgbd_calibration",
    "src_shared": True,
    "memory_low": "64M",
    "memory_max": "768M",
    "cpu_weight": 200,
    "io_weight": 200,
    "oom_score_adjust": 0,
    "config_path": "/etc/ros2/head_rgbd_calibration",
}
SEED_FILES = (
    "nodes/head_camera_cpp/config/head_camera_calibration.yaml",
    "nodes/head_rgbd_cpp/config/tof_calibration.yaml",
    "nodes/head_rgbd_cpp/config/head_rgbd_extrinsics.yaml",
)


def client_vars() -> dict[str, Any]:
    return yaml.safe_load(CLIENT_VARS.read_text())


def setup_tasks() -> list[dict[str, Any]]:
    return yaml.safe_load(SETUP_TASKS.read_text())


def test_node_type_and_tier() -> None:
    node_type = client_vars()["ros2_node_type_defaults"]["head_rgbd_calibration"]
    for key, value in TIER.items():
        assert node_type[key] == value, key
    assert node_type["env"] == ["HEAD_RGBD_CALIBRATION_CONFIG=/etc/ros2/head_rgbd_calibration/config.yaml"]
    # Below the robot tiers: no OOM protection, weight under no critical node.
    assert node_type["oom_score_adjust"] >= 0


def test_ros2_nodes_entry_matches_defaults() -> None:
    nodes = client_vars()["ros2_nodes"]
    names = [n["name"] for n in nodes]
    entry = nodes[names.index("head_rgbd_calibration")]
    assert entry["node_type"] == "head_rgbd_calibration"
    assert entry["present"] is True
    assert entry["enabled"] is True
    assert names.index("head_rgbd_calibration") > names.index("head_rgbd_cpp")
    assert yaml.safe_load(entry["config"]) == yaml.safe_load(DEFAULTS.read_text())


def test_state_dirs_owned_by_node_user_mode_0755() -> None:
    task = next(t for t in setup_tasks() if "ansible.builtin.file" in t)
    spec = task["ansible.builtin.file"]
    assert spec["state"] == "directory"
    assert spec["owner"] == "{{ ansible_user }}"
    assert spec["group"] == "{{ ansible_user }}"
    assert spec["mode"] == "0755"
    assert task["loop"] == [STATE_DIR, f"{STATE_DIR}/sessions", f"{STATE_DIR}/applied"]


def test_seed_copies_never_overwrite_and_need_existing_source() -> None:
    tasks = setup_tasks()
    stat = next(t for t in tasks if "ansible.builtin.stat" in t)
    assert tuple(stat["loop"]) == SEED_FILES
    copies = [t for t in tasks if "ansible.builtin.copy" in t]
    assert len(copies) == 1
    spec = copies[0]["ansible.builtin.copy"]
    assert spec["force"] is False
    assert spec["remote_src"] is True
    assert spec["dest"].startswith(f"{STATE_DIR}/applied/")
    assert "exists" in str(copies[0]["when"])


def test_no_task_clears_state_dir() -> None:
    for task in setup_tasks():
        for module, spec in task.items():
            if not module.startswith("ansible.builtin.") or not isinstance(spec, dict):
                continue
            assert spec.get("state") != "absent", task["name"]
            if module == "ansible.builtin.copy":
                assert spec.get("force") is not True, task["name"]
            if module != "ansible.builtin.stat":
                assert "content" not in spec or not str(spec.get("dest", "")).startswith(STATE_DIR), task["name"]
    assert "state: absent" not in SETUP_TASKS.read_text()
    assert "force: true" not in SETUP_TASKS.read_text()


def test_apply_units_content() -> None:
    path_unit = (TEMPLATES / "ros2-head-rgbd-calibration-apply.path.j2").read_text()
    service_unit = (TEMPLATES / "ros2-head-rgbd-calibration-apply.service.j2").read_text()
    assert f"PathChanged={STATE_DIR}/applied/applied.json" in path_unit
    assert "Unit=ros2-head-rgbd-calibration-apply.service" in path_unit
    assert "WantedBy=multi-user.target" in path_unit
    assert "Type=oneshot" in service_unit
    assert (
        "ExecStart=/usr/bin/systemctl try-restart ros2-head_camera_cpp.service ros2-head_rgbd_cpp.service "
        "ros2-head_state_publisher.service"
    ) in service_unit
    tasks = {t["name"]: t for t in setup_tasks()}
    reload_task = tasks["Reload systemd after head_rgbd_calibration apply unit changes"]
    assert reload_task["ansible.builtin.systemd"]["daemon_reload"] is True
    enable = tasks["Enable and start head_rgbd_calibration apply path unit"]["ansible.builtin.systemd"]
    assert enable["name"] == "ros2-head-rgbd-calibration-apply.path"
    assert enable["enabled"] is True
    assert enable["state"] == "started"


def test_playbook_setup_before_deploy() -> None:
    plays = yaml.safe_load(CLIENT_PLAYBOOK.read_text())
    tasks = [t for play in plays for block in play.get("tasks", []) for t in block.get("block", [block])]
    names = [t.get("name") for t in tasks]
    setup = tasks[names.index("Head RGB-D calibration state directory and apply units")]
    deploy = tasks[names.index("Deploy head_rgbd_calibration")]
    assert names.index(setup["name"]) < names.index(deploy["name"])
    assert setup["ansible.builtin.include_tasks"]["file"] == "tasks/head_rgbd_calibration_setup.yml"
    assert {"head_rgbd_calibration", "setup"} <= set(setup["tags"])
    assert deploy["ansible.builtin.include_tasks"]["file"] == "tasks/resolve_and_deploy.yml"
    assert deploy["vars"]["_deploy_node_name"] == "head_rgbd_calibration"
    assert "head_rgbd_calibration" in deploy["tags"]


def test_http_port_unique_in_client_vars() -> None:
    """Config blocks may hold Jinja, so the values are matched per line, comments stripped."""
    found = [int(m.group(1)) for m in PORT_LINE.finditer(CLIENT_VARS.read_text())]
    assert found.count(HTTP_PORT) == 1
