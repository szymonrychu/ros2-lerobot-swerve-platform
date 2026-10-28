"""Static invariants of the cut-over to the C++ nodes (D2, D3): _cpp live, Python present and disabled as rollback."""

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
ANSIBLE_README = REPO_ROOT / "ansible" / "README.md"
CPP_SUFFIX = "_cpp"
PAIRS = ["map_guard"]
BACKGROUND_TIER = (100, 100, 0)
METRICS_PORTS = {"map_guard": 19110}


def client() -> dict:
    """Load ansible/group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def nodes() -> dict[str, dict]:
    """Return the client ros2_nodes entries by name.

    Returns:
        dict[str, dict]: Entry per node name.
    """
    return {n["name"]: n for n in client()["ros2_nodes"]}


def is_running(entry: dict) -> bool:
    """True when an entry is both present and enabled.

    Args:
        entry (dict): A ros2_nodes entry.

    Returns:
        bool: Whether the node is installed and running.
    """
    return bool(entry.get("present", True) and entry.get("enabled", True))


@pytest.mark.parametrize("python", PAIRS)
def test_cpp_node_is_live(python: str) -> None:
    entry = nodes()[python + CPP_SUFFIX]
    assert entry["present"] is True and entry["enabled"] is True


@pytest.mark.parametrize("python", PAIRS)
def test_python_node_stays_present_and_disabled_for_rollback(python: str) -> None:
    entry = nodes()[python]
    assert entry["present"] is True and entry["enabled"] is False


@pytest.mark.parametrize("python", PAIRS)
def test_cpp_config_block_equals_the_python_block(python: str) -> None:
    all_nodes = nodes()
    cpp_config = yaml.safe_load(all_nodes[python + CPP_SUFFIX]["config"])
    assert cpp_config == yaml.safe_load(all_nodes[python]["config"])
    assert cpp_config["metrics_port"] == METRICS_PORTS[python]


def test_map_guard_cpp_keeps_dry_run() -> None:
    assert yaml.safe_load(nodes()["map_guard_cpp"]["config"])["dry_run"] is True


def test_background_tier_holds_only_throttles_and_previews() -> None:
    types = client()["ros2_node_type_defaults"]
    running = [
        name
        for name, entry in nodes().items()
        if is_running(entry)
        and (
            types[entry["node_type"]].get("cpu_weight"),
            types[entry["node_type"]].get("io_weight"),
            types[entry["node_type"]].get("oom_score_adjust"),
        )
        == BACKGROUND_TIER
    ]
    # The retired Pi camera nodes (D7, D9) are gone: the throttle relays, head_camera_cpp and the head_rgbd_cpp map
    # input (enabled on the D435, user 2026-10-28) are left.
    assert sorted(running) == [
        "head_camera_cpp",
        "head_rgbd_calibration",
        "head_rgbd_cpp",
        "joint_state_throttle_follower",
        "joint_state_throttle_swerve",
        "map3d_uplink",
    ]


def test_only_the_running_node_of_each_pair_is_scraped() -> None:
    targets = {t["node"] for t in client()["monitoring_agent_node_targets"]}
    for python in PAIRS:
        assert python + CPP_SUFFIX in targets
        assert python not in targets


def test_readme_documents_cutover_and_rollback() -> None:
    text = ANSIBLE_README.read_text()
    assert "Cut-over and rollback" in text
    section = text[text.index("Cut-over and rollback") :]
    for phrase in ("map_guard_cpp", "enabled: false", "scripts/deploy-nodes.sh"):
        assert phrase in section, phrase
