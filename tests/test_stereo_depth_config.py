"""Static invariants of the stereo_depth deployment (Ansible group_vars, background cgroup tier, playbook wiring).

stereo_depth runs on the client, matches the synced IMX219 pair (overview_camera) and publishes 16UC1 mm depth on
/stereo/depth/image_raw. Its C++ port stereo_depth_cpp is the enabled node since the cut-over; this entry is kept
disabled for rollback, in the same background tier (below every robot and agent tier). The enabled node's metrics
port is scraped by Alloy.
"""

import importlib.util
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
GROUP_VARS = REPO_ROOT / "ansible" / "group_vars"
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
NODE_CONFIG_MODULE = REPO_ROOT / "nodes" / "stereo_depth" / "stereo_depth" / "config.py"
ANSIBLE_README = REPO_ROOT / "ansible" / "README.md"
DEPLOY_SKILL = REPO_ROOT / ".claude" / "skills" / "ansible-deploy" / "SKILL.md"
TESTS_README = REPO_ROOT / "tests" / "README.md"
NODE_TESTS_DIR = REPO_ROOT / "nodes" / "stereo_depth" / "tests"
NODE = "stereo_depth"
METRICS_PORT = 19113
BACKGROUND_TIER = (100, 100, 0)
APT_PACKAGES = ("ros-jazzy-message-filters", "ros-jazzy-stereo-msgs")


def load_group(name: str) -> dict:
    """Load one group_vars file.

    Args:
        name (str): Group name (client, server).

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load((GROUP_VARS / f"{name}.yml").read_text())


def client_node(name: str) -> dict:
    """Return one client ros2_nodes entry.

    Args:
        name (str): Node entry name.

    Returns:
        dict: The entry.
    """
    return next(n for n in load_group("client")["ros2_nodes"] if n["name"] == name)


def node_config_module() -> object:
    """Import nodes/stereo_depth/stereo_depth/config.py by path (pydantic and yaml only, no ROS).

    Returns:
        object: The module.
    """
    spec = importlib.util.spec_from_file_location("stereo_depth_node_config", NODE_CONFIG_MODULE)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_node_type_is_declared_on_the_client() -> None:
    defaults = load_group("client")["ros2_node_type_defaults"][NODE]
    assert defaults["deploy_mode"] == "native"
    assert defaults["node_launch_command"] == "python3 -m stereo_depth"
    assert defaults["node_src_dir"] == "nodes/stereo_depth"
    assert not defaults.get("pythonpath_parent", False)
    assert defaults["src_shared"] is True
    assert defaults["config_path"] == "/etc/ros2/stereo_depth"
    assert "STEREO_DEPTH_CONFIG=/etc/ros2/stereo_depth/config.yaml" in defaults["env"]
    for package in APT_PACKAGES:
        assert package in defaults["apt_packages"], package


def test_node_type_is_the_only_member_of_the_background_tier() -> None:
    types = load_group("client")["ros2_node_type_defaults"]
    defaults = types[NODE]
    assert (defaults["cpu_weight"], defaults["io_weight"], defaults["oom_score_adjust"]) == BACKGROUND_TIER
    members = [
        name
        for name, entry in types.items()
        if (entry.get("cpu_weight"), entry.get("io_weight"), entry.get("oom_score_adjust")) == BACKGROUND_TIER
    ]
    # Its C++ port (the enabled node since the cut-over) and the joint-state throttle relays share the tier.
    assert members == [NODE, f"{NODE}_cpp", "joint_state_throttle_follower", "joint_state_throttle_swerve"]
    assert "cpu_quota" not in defaults and "memory_high" not in defaults


def test_node_type_memory_estimate() -> None:
    defaults = load_group("client")["ros2_node_type_defaults"][NODE]
    assert defaults["memory_low"] == "96M"
    assert defaults["memory_max"] == "384M"
    text = (GROUP_VARS / "client.yml").read_text()
    block = text[text.index(f"\n  {NODE}:\n") :]
    block = block[: block.index("\n  ", block.index("memory_max"))]
    assert "stimate" in block, "the memory figures are marked as an estimate"


def test_ros2_nodes_entry_is_present_and_disabled_after_the_cutover() -> None:
    """stereo_depth_cpp took over (tests/test_cpp_cutover.py); this entry stays installed as the rollback."""
    node = client_node(NODE)
    assert node["node_type"] == NODE
    assert node["present"] is True
    assert node["enabled"] is False
    assert isinstance(node["config"], str)


def test_deployed_config_makes_every_default_explicit() -> None:
    """Every DepthConfig field is in the deployed config, at its default except metrics_port (pinned 19113)."""
    module = node_config_module()
    config = yaml.safe_load(client_node(NODE)["config"])
    fields = module.DepthConfig.model_fields
    assert set(config) == set(fields)
    defaults = module.DepthConfig()
    for key, value in config.items():
        expected = getattr(defaults, key)
        assert (str(expected) if isinstance(expected, Path) else expected) == value, key
    assert config["metrics_port"] == METRICS_PORT


def test_deployed_config_loads_in_the_node_load_config(tmp_path: Path) -> None:
    module = node_config_module()
    path = tmp_path / "config.yaml"
    path.write_text(client_node(NODE)["config"])
    loaded = module.load_config(path)
    assert loaded.depth_topic == "/stereo/depth/image_raw"
    assert loaded.left_image_topic == "/overview_camera/image_raw"
    assert loaded.right_image_topic == "/stereo/right/image_raw"
    assert loaded.metrics_port == METRICS_PORT


def test_metrics_port_is_scraped_from_the_enabled_cpp_node() -> None:
    targets = {t["node"]: t for t in load_group("client")["monitoring_agent_node_targets"]}
    assert NODE not in targets
    assert targets[f"{NODE}_cpp"]["address"] == f"127.0.0.1:{METRICS_PORT}"
    assert targets[f"{NODE}_cpp"]["path"] == "/metrics"


def test_client_playbook_deploys_the_node_after_overview_camera() -> None:
    text = CLIENT_PLAYBOOK.read_text()
    assert "name: Deploy stereo_depth" in text
    assert "_deploy_node_name: stereo_depth" in text
    assert "tags: [stereo_depth, apt, python, build, config]" in text
    assert text.index("name: Deploy overview_camera") < text.index("name: Deploy stereo_depth")
    block = text[text.index("name: Deploy stereo_depth") :]
    block = block[: block.index("\n\n")]
    assert "tags: [stereo_depth]" in block, "the include applies the node tag to its tasks"


def test_boot_include_display_name_names_the_imx219_pair() -> None:
    text = CLIENT_PLAYBOOK.read_text()
    line = next(row for row in text.splitlines() if "Camera boot overlay" in row)
    assert "IMX219 pair" in line and "IMX708" not in line


def test_docs_list_the_node_tier_memory_and_port() -> None:
    readme = ANSIBLE_README.read_text()
    assert "stereo_depth" in readme and "19113" in readme
    assert "background" in readme.lower()
    skill = DEPLOY_SKILL.read_text()
    assert any(row.startswith("|") and "stereo_depth" in row for row in skill.splitlines())


def test_tests_readme_lists_every_stereo_depth_node_test_file() -> None:
    readme = TESTS_README.read_text()
    assert "Per-node tests (stereo_depth)" in readme
    section = readme[readme.index("Per-node tests (stereo_depth)") :]
    for path in sorted(NODE_TESTS_DIR.glob("test_*.py")):
        assert path.name in section, path.name
    assert "### test_stereo_depth_config.py" in readme or "### `test_stereo_depth_config.py`" in readme
    for name in [n for n in globals() if n.startswith("test_")]:
        assert f"`{name}`" in readme, name
