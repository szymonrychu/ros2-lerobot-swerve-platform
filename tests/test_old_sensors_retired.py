"""Static invariants of the old head sensor retirements of the D435 switchover.

The head ToF (arducam_tof) is retired once every consumer reads the D435 (design D11): its ros2_nodes entry is
present: false with no config (the next deploy uninstalls the unit, venv and config), Alloy no longer scrapes its
metrics port, and no enabled node's config, env or launch command and no nav2 parameter names a /tof/ topic.
"""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
TOF_NODE = "arducam_tof"
TOF_METRICS_PORT = 19112
TOF_TOPIC_PREFIX = "/tof/"
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


def test_arducam_tof_is_retired() -> None:
    """arducam_tof is present: false without config, unscraped, and no enabled consumer reads a /tof/ topic."""
    variables = load_client()
    entry = next(n for n in variables["ros2_nodes"] if n["name"] == TOF_NODE)
    assert entry == {"name": TOF_NODE, "node_type": TOF_NODE, "present": False}

    lines = CLIENT_VARS.read_text().splitlines()
    index = lines.index(f"  - name: {TOF_NODE}")
    comment = " ".join(
        line.strip().lstrip("#").strip() for line in lines[index - 3 : index] if line.strip().startswith("#")
    )
    assert RETIREMENT_COMMENT in comment

    targets = variables["monitoring_agent_node_targets"]
    assert TOF_NODE not in {t["node"] for t in targets}
    assert not any(t["address"].endswith(f":{TOF_METRICS_PORT}") for t in targets)

    readers = {name for name, text in running_node_texts(variables).items() if TOF_TOPIC_PREFIX in text}
    assert not readers
    assert TOF_TOPIC_PREFIX not in NAV2_PARAMS.read_text()
