"""Static invariants of the head_rgbd_calibration deploy: node type, entry, state dir, apply units and script."""

import os
import re
import subprocess
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
    "cpu_weight": 100,
    "io_weight": 100,
    "oom_score_adjust": 0,
    "config_path": "/etc/ros2/head_rgbd_calibration",
}
APPLY_SCRIPT = REPO_ROOT / "ansible" / "playbooks" / "files" / "ros2-head-rgbd-calibration-apply.sh"
APPLY_SCRIPT_DEST = "/usr/local/sbin/ros2-head-rgbd-calibration-apply"
# The D435 consumers of the applied head chain (design D10); head_camera_cpp and arducam_tof are retired.
RESTART_UNITS = ["ros2-head_rgbd_cpp.service", "ros2-head_state_publisher.service"]
FRONT_UNIT = "ros2-front_camera.service"
BACK_UNIT = "ros2-back_camera.service"
# FILE:UNIT[,UNIT] arguments of the apply restart script: each applied file and the units that read it.
RESTART_PAIRS = [
    f"head_chain.json:{','.join(RESTART_UNITS)}",
    f"front_camera_calibration.yaml:{FRONT_UNIT}",
    f"back_camera_calibration.yaml:{BACK_UNIT}",
]
# Applied files of the retired ToF and head camera (CM3) calibration (design D10).
RETIRED_APPLIED_FILES = (
    "head_camera_calibration.yaml",
    "tof_calibration.yaml",
    "head_rgbd_extrinsics.yaml",
    "tof_joint.json",
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


def test_no_tof_or_head_camera_applied_files_are_seeded() -> None:
    """applied/ is written by the node's Apply only: no setup task stats or copies a file into it."""
    text = SETUP_TASKS.read_text()
    for name in RETIRED_APPLIED_FILES:
        assert name not in text, name
    for task in setup_tasks():
        assert "ansible.builtin.stat" not in task, task["name"]
        spec = task.get("ansible.builtin.copy", {})
        assert not str(spec.get("dest", "")).startswith(STATE_DIR), task["name"]


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
    assert "StateDirectory=ros2-head-rgbd-calibration-apply" in service_unit
    exec_start = [line for line in service_unit.splitlines() if line.startswith("ExecStart=")]
    assert exec_start == [f"ExecStart={APPLY_SCRIPT_DEST} {' '.join(RESTART_PAIRS)}"]
    for retired in ("head_camera_cpp", "arducam_tof"):
        assert retired not in service_unit, retired
    # The node reports the state of the units the apply restarts.
    assert yaml.safe_load(DEFAULTS.read_text())["restart_units"] == RESTART_UNITS
    assert calib_config()["restart_units"] == RESTART_UNITS
    tasks = {t["name"]: t for t in setup_tasks()}
    script = tasks["Install the head_rgbd_calibration apply restart script"]["ansible.builtin.copy"]
    assert script["src"] == "{{ playbook_dir }}/files/ros2-head-rgbd-calibration-apply.sh"
    assert script["dest"] == APPLY_SCRIPT_DEST
    assert (script["owner"], script["group"], script["mode"]) == ("root", "root", "0755")
    reload_task = tasks["Reload systemd after head_rgbd_calibration apply unit changes"]
    assert reload_task["ansible.builtin.systemd"]["daemon_reload"] is True
    enable = tasks["Enable and start head_rgbd_calibration apply path unit"]["ansible.builtin.systemd"]
    assert enable["name"] == "ros2-head-rgbd-calibration-apply.path"
    assert enable["enabled"] is True
    assert enable["state"] == "started"


def run_apply(tmp_path: Path, fail_restart: bool = False) -> tuple[int, list[str]]:
    """Run the apply restart script against tmp_path/applied, with a fake systemctl.

    Args:
        tmp_path: Work directory: applied/ holds the applied files, seen/ is the script's state directory.
        fail_restart: The fake systemctl exits 1.

    Returns:
        tuple[int, list[str]]: The script's exit code and the systemctl calls it made.
    """
    (tmp_path / "applied").mkdir(exist_ok=True)
    (tmp_path / "seen").mkdir(exist_ok=True)
    fake = tmp_path / "bin" / "systemctl"
    fake.parent.mkdir(exist_ok=True)
    fake.write_text(f'#!/bin/bash\necho "$@" >> "$LOG"\nexit {1 if fail_restart else 0}\n')
    fake.chmod(0o755)
    log = tmp_path / "log"
    log.unlink(missing_ok=True)
    env = {
        **os.environ,
        "PATH": f"{fake.parent}:{os.environ['PATH']}",
        "LOG": str(log),
        "HEAD_RGBD_APPLIED_DIR": str(tmp_path / "applied"),
        "STATE_DIRECTORY": str(tmp_path / "seen"),
    }
    result = subprocess.run(
        ["bash", str(APPLY_SCRIPT), *RESTART_PAIRS], capture_output=True, text=True, env=env, check=False
    )
    return result.returncode, log.read_text().splitlines() if log.exists() else []


def test_apply_restarts_only_when_the_applied_head_chain_changes(tmp_path: Path) -> None:
    restart = [f"try-restart {' '.join(RESTART_UNITS)}"]
    chain = tmp_path / "applied" / "head_chain.json"
    assert run_apply(tmp_path) == (0, []), "nothing applied: nothing to restart"
    chain.write_text('{"head_tilt": 1}')
    assert run_apply(tmp_path) == (0, restart), "first head chain"
    assert (tmp_path / "seen" / "head_chain.json").read_text() == '{"head_tilt": 1}'
    (tmp_path / "applied" / "applied.json").write_text('{"kind": "intrinsics"}')
    assert run_apply(tmp_path) == (0, []), "an intrinsics accept rewrites applied.json only"
    chain.write_text('{"head_tilt": 1}')
    assert run_apply(tmp_path) == (0, []), "the same head chain written again"
    chain.write_text('{"head_tilt": 2}')
    assert run_apply(tmp_path) == (0, restart), "a new head chain"
    chain.unlink()
    assert run_apply(tmp_path) == (0, restart), "the head chain removed"
    assert not (tmp_path / "seen" / "head_chain.json").exists()
    assert run_apply(tmp_path) == (0, []), "still none"


def test_apply_restarts_only_the_units_of_a_changed_file(tmp_path: Path) -> None:
    head = f"try-restart {' '.join(RESTART_UNITS)}"
    front, back = f"try-restart {FRONT_UNIT}", f"try-restart {BACK_UNIT}"
    applied = tmp_path / "applied"
    assert run_apply(tmp_path) == (0, [])
    (applied / "front_camera_calibration.yaml").write_text("image_width: 1152\n")
    assert run_apply(tmp_path) == (0, [front]), "a front calibration restarts the front camera only"
    (applied / "head_chain.json").write_text('{"head_tilt": 1}')
    assert run_apply(tmp_path) == (0, [head]), "a head chain restarts the head units only"
    (applied / "front_camera_calibration.yaml").write_text("image_width: 1153\n")
    (applied / "back_camera_calibration.yaml").write_text("image_width: 1152\n")
    assert run_apply(tmp_path) == (0, [front, back]), "both cameras, in argument order"
    (applied / "applied.json").write_text('{"csi_intrinsics": {}}')
    assert run_apply(tmp_path) == (0, []), "an applied.json-only rewrite"


def test_apply_pairs_match_the_node_csi_cameras() -> None:
    """The script's FILE:UNIT pairs of the CSI cameras are the node's csi_cameras (applied file and unit)."""
    cameras = calib_config()["csi_cameras"]
    pairs = [f"{name}_camera_calibration.yaml:{camera['unit']}" for name, camera in cameras.items()]
    assert pairs == RESTART_PAIRS[1:]


def test_apply_rejects_an_argument_without_units(tmp_path: Path) -> None:
    result = subprocess.run(["bash", str(APPLY_SCRIPT), "head_chain.json"], capture_output=True, text=True, check=False)
    assert result.returncode != 0
    assert "FILE:UNIT" in result.stderr


def test_apply_retries_a_failed_restart(tmp_path: Path) -> None:
    restart = [f"try-restart {' '.join(RESTART_UNITS)}"]
    (tmp_path / "applied").mkdir()
    (tmp_path / "applied" / "head_chain.json").write_text('{"head_tilt": 1}')
    code, calls = run_apply(tmp_path, fail_restart=True)
    assert (code != 0, calls) == (True, restart)
    assert not (tmp_path / "seen" / "head_chain.json").exists(), "a failed restart records nothing"
    assert run_apply(tmp_path) == (0, restart), "the next trigger restarts again"


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


def node_entry(name: str) -> dict[str, Any]:
    return next(n for n in client_vars()["ros2_nodes"] if n["name"] == name)


def calib_config() -> dict[str, Any]:
    return yaml.safe_load(node_entry("head_rgbd_calibration")["config"])


def mcp_config() -> dict[str, Any]:
    return yaml.safe_load(node_entry("mcp_server")["config"])


def test_fit_joint_offsets_equal_mcp_server_arm() -> None:
    assert calib_config()["fit"]["joint_offsets_rad"] == mcp_config()["arm"]["joint_offsets_rad"]


def test_fit_base_in_base_link_equals_mcp_server_arm() -> None:
    assert calib_config()["fit"]["base_in_base_link"] == mcp_config()["arm"]["base_in_base_link"]


def test_pose_check_body_equals_mcp_server_body() -> None:
    body = calib_config()["arm_sequence"]["pose_check"]["body"]
    mcp_body = mcp_config()["body"]
    assert body["boxes"] == mcp_body["boxes"]
    assert body["cylinders"] == mcp_body["cylinders"]


def test_arm_urdf_file_is_the_mcp_server_arm_urdf() -> None:
    mcp_urdf = mcp_config()["arm"]["urdf_path"]
    assert calib_config()["arm_urdf_file"].endswith("/" + mcp_urdf)


def test_fit_board_equals_board() -> None:
    config = calib_config()
    assert config["fit"]["board"] == config["board"]


def test_service_joins_mcp_token_group_created_by_mcp_server_setup() -> None:
    # The role reads supplementary_groups from the node type defaults (as claude_agent).
    assert "mcp-token" in client_vars()["ros2_node_type_defaults"]["head_rgbd_calibration"]["supplementary_groups"]
    mcp_setup = yaml.safe_load((REPO_ROOT / "ansible" / "playbooks" / "tasks" / "mcp_server_setup.yml").read_text())
    groups = [t["ansible.builtin.group"] for t in mcp_setup if "ansible.builtin.group" in t]
    assert any(g["name"] == "mcp-token" and g.get("state", "present") == "present" for g in groups)


def test_setup_creates_mcp_token_group_before_the_service_starts() -> None:
    """The client playbook runs this setup before mcp_server_setup, so it must create the group itself."""
    group = next(t for t in setup_tasks() if "ansible.builtin.group" in t)["ansible.builtin.group"]
    assert group["name"] == "mcp-token"
    assert group.get("state", "present") == "present"
    assert group["system"] is True
