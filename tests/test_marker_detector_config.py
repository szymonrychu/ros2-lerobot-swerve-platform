"""Static invariants of the marker_detector wiring (Ansible type, entry, playbook, metrics, docs)."""

import re
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
DEPLOY_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
NODE_DIR = REPO_ROOT / "nodes" / "marker_detector"
DOCS = [
    REPO_ROOT / "ansible" / "README.md",
    REPO_ROOT / ".claude" / "skills" / "ansible-deploy" / "SKILL.md",
    REPO_ROOT / "nodes" / "README.md",
]
METRICS_PORT = 19118
METRICS_TARGET = {"node": "marker_detector", "address": f"127.0.0.1:{METRICS_PORT}", "path": "/metrics"}

sys.path.insert(0, str(NODE_DIR))
from marker_detector.config import MarkerDetectorConfig  # noqa: E402


def client_vars() -> dict:
    """Load group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def test_marker_detector_node_type_defaults() -> None:
    node_type = client_vars()["ros2_node_type_defaults"]["marker_detector"]
    assert node_type["deploy_mode"] == "native"
    assert node_type["node_launch_command"] == "python3 -m marker_detector"
    assert node_type["node_src_dir"] == "nodes/marker_detector"
    assert node_type["src_shared"] is True
    assert (node_type["cpu_weight"], node_type["io_weight"], node_type["oom_score_adjust"]) == (500, 500, -500)
    assert node_type["memory_low"] == "128M" and node_type["memory_max"] == "512M"
    assert node_type["config_path"] == "/etc/ros2/marker_detector"
    assert "MARKER_DETECTOR_CONFIG=/etc/ros2/marker_detector/config.yaml" in node_type["env"]


def test_marker_detector_entry_after_poi_store_with_config() -> None:
    nodes = client_vars()["ros2_nodes"]
    names = [n["name"] for n in nodes]
    assert names.index("marker_detector") > names.index("poi_store")
    entry = nodes[names.index("marker_detector")]
    assert entry["node_type"] == "marker_detector" and entry["present"] and entry["enabled"]
    config = yaml.safe_load(entry["config"])
    assert config["metrics_port"] == METRICS_PORT
    assert set(config) == set(MarkerDetectorConfig.model_fields)
    MarkerDetectorConfig(**config)


def test_marker_detector_cameras_are_head_front_back() -> None:
    entry = next(n for n in client_vars()["ros2_nodes"] if n["name"] == "marker_detector")
    cameras = yaml.safe_load(entry["config"])["cameras"]
    assert cameras == [
        {
            "name": "head",
            "image": "/head/d435/color/image_raw/compressed",
            "camera_info": "/head/d435/color/camera_info",
            "depth_plane": True,
        },
        {
            "name": "front",
            "image": "/front_camera/image_raw/compressed",
            "camera_info": "/front_camera/camera_info",
            "depth_plane": False,
        },
        {
            "name": "back",
            "image": "/back_camera/image_raw/compressed",
            "camera_info": "/back_camera/camera_info",
            "depth_plane": False,
        },
    ]


def test_marker_detector_entry_has_no_zoom() -> None:
    nodes = client_vars()["ros2_nodes"]
    entry = next(n for n in nodes if n["name"] == "marker_detector")
    assert "zoom" not in entry["config"]


def test_metrics_target_for_marker_detector() -> None:
    assert METRICS_TARGET in client_vars()["monitoring_agent_node_targets"]


def test_playbook_deploys_marker_detector_after_poi_store() -> None:
    plays = yaml.safe_load(DEPLOY_PLAYBOOK.read_text())
    tasks = [t for play in plays for block in play.get("tasks", []) for t in block.get("block", [block])]
    names = [t.get("name") for t in tasks]
    assert names.index("Deploy marker_detector") > names.index("Deploy poi_store")
    task = tasks[names.index("Deploy marker_detector")]
    assert task["vars"]["_deploy_node_name"] == "marker_detector"
    assert task["tags"] == ["marker_detector", "apt", "python", "build", "config"]


def test_node_files_exist() -> None:
    for name in ("pyproject.toml", "uv.lock", "README.md"):
        assert (NODE_DIR / name).is_file()


def test_docs_mention_marker_detector_and_port() -> None:
    for doc in DOCS:
        assert re.search(r"marker_detector", doc.read_text()), doc
    assert re.search(rf"`marker_detector` \| {METRICS_PORT}", DOCS[0].read_text())
