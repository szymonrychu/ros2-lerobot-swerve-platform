"""Wiring of the web_ui Agent tab in Ansible group_vars: tab entry, its position and the claude_agent port."""

import re
from pathlib import Path
from urllib.parse import urlparse

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
TESTS_README = REPO_ROOT / "tests" / "README.md"

AGENT_TAB_ID = "agent"
AGENT_TAB_TYPE = "agent_chat"
AGENT_TAB_LABEL = "Agent"
MAP_TAB_ID = "map"


def node_config(name: str) -> dict:
    """Parse the `config: |` block of one client ros2_nodes entry.

    Args:
        name: Node name.

    Returns:
        dict: Parsed config YAML of the node.
    """
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    return yaml.safe_load(next(n for n in nodes if n["name"] == name)["config"])


def agent_tab() -> dict:
    """Return the web_ui agent tab entry.

    Returns:
        dict: The tab config.
    """
    return next(t for t in node_config("web_ui")["tabs"] if t["id"] == AGENT_TAB_ID)


def test_agent_tab_exists_with_type_and_label() -> None:
    """web_ui has an `agent` tab of type agent_chat labelled "Agent"."""
    tab = agent_tab()
    assert tab["type"] == AGENT_TAB_TYPE
    assert tab["label"] == AGENT_TAB_LABEL


def test_agent_tab_directly_after_map_tab() -> None:
    """The Agent tab is listed right after the map tab."""
    ids = [t["id"] for t in node_config("web_ui")["tabs"]]
    assert ids.index(AGENT_TAB_ID) == ids.index(MAP_TAB_ID) + 1


def test_agent_tab_url_matches_claude_agent_port() -> None:
    """agent_url points at 127.0.0.1 and the port claude_agent listens on."""
    url = urlparse(agent_tab()["agent_url"])
    cfg = node_config("claude_agent")
    assert url.hostname == cfg["http_host"] == "127.0.0.1"
    assert url.port == cfg["http_port"]


def test_every_test_is_documented_in_tests_readme() -> None:
    """Every test_* function of this file is listed in tests/README.md."""
    names = re.findall(r"^def (test_\w+)", Path(__file__).read_text(), flags=re.M)
    readme = TESTS_README.read_text()
    assert [n for n in names if f"`{n}`" not in readme] == []
