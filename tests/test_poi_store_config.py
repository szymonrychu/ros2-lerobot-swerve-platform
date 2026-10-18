"""Static invariants of the poi_store wiring (Ansible node type, ros2_nodes entry, playbooks, lint script, docs)."""

import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
DEPLOY_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
DIR_TASKS = REPO_ROOT / "ansible" / "playbooks" / "tasks" / "poi_store_dir.yml"
LINT_SCRIPT = REPO_ROOT / "scripts" / "lint-all-nodes.sh"
NODE_DIR = REPO_ROOT / "nodes" / "poi_store"
DOCS = [
    REPO_ROOT / "nodes" / "README.md",
    REPO_ROOT / "ansible" / "README.md",
    REPO_ROOT / ".claude" / "skills" / "ansible-deploy" / "SKILL.md",
]
STORE_DIR = "/var/lib/ros2/poi"
STORE_PATH = "/var/lib/ros2/poi/poi.json"
PHOTO_DIR = "/var/lib/ros2/poi/lookups"
PHOTOS = {
    "store_dir": PHOTO_DIR,
    "max_records": 3000,
    "max_bytes": 300000000,
    "max_age_days": None,
    "max_image_bytes": 5000000,
    "prune_interval_s": 3600,
    "summary_topic": "/poi/photos",
}
TOPICS = {"list_topic": "/poi/list", "command_topic": "/poi/command", "result_topic": "/poi/result"}


def client_vars() -> dict:
    """Load group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def test_poi_store_node_type_defaults() -> None:
    node_type = client_vars()["ros2_node_type_defaults"]["poi_store"]
    assert node_type["deploy_mode"] == "native"
    assert node_type["node_launch_command"] == "python3 -m poi_store"
    assert node_type["node_src_dir"] == "nodes/poi_store"
    assert node_type["config_path"] == "/etc/ros2/poi_store"
    assert "POI_STORE_CONFIG=/etc/ros2/poi_store/config.yaml" in node_type["env"]


def test_poi_store_entry_after_mcp_server_with_config() -> None:
    nodes = client_vars()["ros2_nodes"]
    names = [n["name"] for n in nodes]
    assert names.index("poi_store") == names.index("mcp_server") + 1
    entry = nodes[names.index("poi_store")]
    assert entry["node_type"] == "poi_store" and entry["present"] and entry["enabled"]
    config = yaml.safe_load(entry["config"])
    assert config["store_path"] == STORE_PATH
    for key, topic in TOPICS.items():
        assert config[key] == topic


def test_poi_directory_task_owned_by_node_user() -> None:
    tasks = yaml.safe_load(DIR_TASKS.read_text())
    file_task = tasks[0]["ansible.builtin.file"]
    assert file_task["path"] == STORE_DIR and file_task["state"] == "directory"
    assert file_task["owner"] == "{{ ansible_user }}"


def test_poi_playbook_deploys_node_and_creates_directory() -> None:
    full = DEPLOY_PLAYBOOK.read_text()
    assert "_deploy_node_name: poi_store" in full and "poi_store_dir.yml" in full
    assert full.index("_deploy_node_name: mcp_server") < full.index("_deploy_node_name: poi_store")


def test_lint_script_and_node_files() -> None:
    assert "nodes/poi_store" in LINT_SCRIPT.read_text()
    for name in ("pyproject.toml", "uv.lock", "README.md"):
        assert (NODE_DIR / name).is_file()
    assert (NODE_DIR / "tests").is_dir()


def test_docs_mention_poi_store() -> None:
    for doc in DOCS:
        assert re.search(r"poi_store", doc.read_text()), doc


def poi_store_entry_config_text() -> str:
    """Return the raw poi_store config text of the client.yml ros2_nodes entry.

    Returns:
        str: The YAML config block as written (with comments).
    """
    return next(n for n in client_vars()["ros2_nodes"] if n["name"] == "poi_store")["config"]


def test_poi_store_config_photos_block() -> None:
    config = yaml.safe_load(poi_store_entry_config_text())
    assert config["photos"] == PHOTOS


def test_poi_store_config_photos_block_has_a_comment_per_key() -> None:
    lines = poi_store_entry_config_text().splitlines()
    start = lines.index("photos:")
    for index, line in enumerate(lines[start + 1 :], start + 1):
        if line.startswith("  ") and not line.lstrip().startswith("#"):
            assert lines[index - 1].lstrip().startswith("#"), line


def test_poi_directory_task_ensures_photo_directory_after_parent() -> None:
    tasks = yaml.safe_load(DIR_TASKS.read_text())
    paths = [t["ansible.builtin.file"]["path"] for t in tasks]
    assert paths.index(PHOTO_DIR) > paths.index(STORE_DIR)
    photo = tasks[paths.index(PHOTO_DIR)]["ansible.builtin.file"]
    assert photo["state"] == "directory" and photo["owner"] == "{{ ansible_user }}" and photo["mode"] == "0755"
