"""Static invariants of the old head sensor retirements of the D435 switchover.

The head ToF (arducam_tof) is retired once every consumer reads the D435 (design D11): its ros2_nodes entry is
present: false with no config (the next deploy uninstalls the unit, venv and config), Alloy no longer scrapes its
metrics port, and no enabled node's config, env or launch command and no nav2 parameter names a /tof/ topic.

The gripper USB camera (gripper_uvc_camera) is retired the same way: the head D435 sees the jaws, so its entry is
present: false, Alloy does not scrape it, the master2master relay of its stream is gone and no tracked config names
/camera_0.

The head Camera Module 3 (head_camera_cpp) is retired as well: present: false without config, unscraped, and no enabled
node's config, env or launch command names a /head_camera/ topic. Its node type and nodes/head_camera_cpp stay for
front-and-back-camera-module (plan Q8).

Once retired, the arducam_tof bridge source is deleted: nodes/bridges/arducam_tof is untracked, no lint or CI list
names it, and its node type is the uninstall stub (deploy_mode and launch command only).
"""

import subprocess
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
TOF_NODE = "arducam_tof"
TOF_METRICS_PORT = 19112
TOF_TOPIC_PREFIX = "/tof/"
GRIPPER_NODE = "gripper_uvc_camera"
GRIPPER_NODE_TYPE = "uvc_camera"
GRIPPER_METRICS_PORT = 19105
GRIPPER_TOPIC_PREFIX = "/camera_0"
HEAD_CAMERA_NODE = "head_camera_cpp"
HEAD_CAMERA_METRICS_PORT = 19116
HEAD_CAMERA_TOPIC_PREFIX = "/head_camera/"
UNINSTALL_STUB_KEYS = {"deploy_mode", "node_launch_command"}
LINT_LISTS = ("pyproject.toml", "scripts/lint-all-nodes.sh", ".github/workflows/ci.yml")
RETIREMENT_COMMENT = "Retired (D435 switchover): present: false uninstalls the unit, venv and config on the next deploy"


def load_client() -> dict:
    """Load ansible/group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def is_running(entry: dict) -> bool:
    """Return whether a ros2_nodes entry is installed and enabled.

    Args:
        entry (dict): A ros2_nodes entry.

    Returns:
        bool: True when present and enabled.
    """
    return bool(entry.get("present", True) and entry.get("enabled", True))


def running_node_texts(variables: dict) -> dict[str, str]:
    """Return what each running client node is configured with: its config block, env and launch command.

    Args:
        variables (dict): Parsed client group_vars.

    Returns:
        dict[str, str]: Node name to the joined config, node and type env and type launch command.
    """
    types = variables["ros2_node_type_defaults"]
    out: dict[str, str] = {}
    for entry in variables["ros2_nodes"]:
        if not is_running(entry):
            continue
        node_type = types.get(entry["node_type"], {})
        parts = [
            entry.get("config", ""),
            *entry.get("env", []),
            *node_type.get("env", []),
            node_type.get("node_launch_command", ""),
        ]
        out[entry["name"]] = "\n".join(parts)
    return out


def retirement_comment(name: str) -> str:
    """Return the comment lines right above a ros2_nodes entry of client.yml, joined.

    Args:
        name (str): Node name.

    Returns:
        str: The joined comment text of the three lines above ``- name: <name>``.
    """
    lines = CLIENT_VARS.read_text().splitlines()
    index = lines.index(f"  - name: {name}")
    return " ".join(
        line.strip().lstrip("#").strip() for line in lines[index - 3 : index] if line.strip().startswith("#")
    )


def tracked_config_files() -> list[Path]:
    """Return client.yml, every tracked file under a node's config/ directory and every role template.

    Returns:
        list[Path]: Absolute paths.
    """
    tracked = subprocess.run(
        ["git", "ls-files", "nodes", "ansible/roles"], cwd=REPO_ROOT, capture_output=True, text=True, check=True
    ).stdout.split()
    picked = [
        path
        for path in tracked
        if (path.startswith("nodes/") and "/config/" in path)
        or (path.startswith("ansible/roles/") and "/templates/" in path)
    ]
    return [CLIENT_VARS, *(REPO_ROOT / path for path in picked)]


def assert_bridge_source_is_gone(node_type: str) -> None:
    """Assert nodes/bridges/<node_type> is untracked, unlisted for lint and CI, and its node type is the stub.

    Args:
        node_type (str): Node type name, equal to its bridge directory name.
    """
    source_dir = f"nodes/bridges/{node_type}"
    tracked = subprocess.run(["git", "ls-files", source_dir], cwd=REPO_ROOT, capture_output=True, text=True, check=True)
    assert tracked.stdout == ""
    for path in LINT_LISTS:
        assert f"bridges/{node_type}" not in (REPO_ROOT / path).read_text(), path
    assert set(load_client()["ros2_node_type_defaults"][node_type]) == UNINSTALL_STUB_KEYS


def test_arducam_tof_is_retired() -> None:
    """arducam_tof is present: false without config, unscraped, and no enabled consumer reads a /tof/ topic."""
    variables = load_client()
    entry = next(n for n in variables["ros2_nodes"] if n["name"] == TOF_NODE)
    assert entry == {"name": TOF_NODE, "node_type": TOF_NODE, "present": False}

    assert RETIREMENT_COMMENT in retirement_comment(TOF_NODE)

    targets = variables["monitoring_agent_node_targets"]
    assert TOF_NODE not in {t["node"] for t in targets}
    assert not any(t["address"].endswith(f":{TOF_METRICS_PORT}") for t in targets)

    readers = {name for name, text in running_node_texts(variables).items() if TOF_TOPIC_PREFIX in text}
    assert not readers
    assert TOF_TOPIC_PREFIX not in NAV2_PARAMS.read_text()


def test_gripper_camera_is_retired() -> None:
    """gripper_uvc_camera is present: false without env, unscraped, and no tracked config names /camera_0."""
    variables = load_client()
    entry = next(n for n in variables["ros2_nodes"] if n["name"] == GRIPPER_NODE)
    assert entry == {"name": GRIPPER_NODE, "node_type": GRIPPER_NODE_TYPE, "present": False}
    assert RETIREMENT_COMMENT in retirement_comment(GRIPPER_NODE)

    targets = variables["monitoring_agent_node_targets"]
    assert GRIPPER_NODE not in {t["node"] for t in targets}
    assert not any(t["address"].endswith(f":{GRIPPER_METRICS_PORT}") for t in targets)

    hits = [
        str(path.relative_to(REPO_ROOT))
        for path in tracked_config_files()
        if path.is_file() and GRIPPER_TOPIC_PREFIX in path.read_text(errors="ignore")
    ]
    assert hits == []


def test_head_camera_is_retired() -> None:
    """head_camera_cpp is present: false without config, unscraped, and no enabled consumer reads /head_camera/."""
    variables = load_client()
    entry = next(n for n in variables["ros2_nodes"] if n["name"] == HEAD_CAMERA_NODE)
    assert entry == {"name": HEAD_CAMERA_NODE, "node_type": HEAD_CAMERA_NODE, "present": False}
    comment = retirement_comment(HEAD_CAMERA_NODE)
    assert RETIREMENT_COMMENT in comment and "front-and-back-camera-module" in comment
    assert HEAD_CAMERA_NODE in variables["ros2_node_type_defaults"]

    targets = variables["monitoring_agent_node_targets"]
    assert HEAD_CAMERA_NODE not in {t["node"] for t in targets}
    assert not any(t["address"].endswith(f":{HEAD_CAMERA_METRICS_PORT}") for t in targets)

    readers = {name for name, text in running_node_texts(variables).items() if HEAD_CAMERA_TOPIC_PREFIX in text}
    assert not readers


def test_arducam_tof_source_is_gone() -> None:
    """The retired arducam_tof bridge is deleted; its node type stays only for the uninstall deploy."""
    assert_bridge_source_is_gone(TOF_NODE)


def test_uvc_camera_source_is_gone() -> None:
    """The uvc_camera bridge (only the retired gripper camera ran it) is deleted; its node type stays as the stub."""
    assert_bridge_source_is_gone(GRIPPER_NODE_TYPE)
