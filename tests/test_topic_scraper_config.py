"""Client topic_scraper_api config: which message types the debug scraper subscribes to."""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
SERVER_VARS = REPO_ROOT / "ansible" / "group_vars" / "server.yml"


def scraper_config() -> dict:
    """Parse the client topic_scraper_api node config.

    Returns:
        dict: The node's config block.
    """
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    entry = next(n for n in nodes if n["name"] == "topic_scraper_api")
    return yaml.safe_load(entry["config"])


def test_scraper_does_not_subscribe_raw_images() -> None:
    """Raw 640x480 frames (~20 MB/s) cost the scraper ~0.4 cores to deserialize; previews use the compressed topics."""
    allowed = scraper_config()["allowed_types"]
    assert "sensor_msgs/msg/Image" not in allowed
    assert "sensor_msgs/msg/CompressedImage" in allowed


def test_scraper_is_disabled_on_both_hosts_and_not_scraped() -> None:
    """topic_scraper_api is installed but stopped on robot and server (2026-10-11); Alloy does not scrape it."""
    for path in (CLIENT_VARS, SERVER_VARS):
        data = yaml.safe_load(path.read_text())
        entry = next(n for n in data["ros2_nodes"] if n["name"] == "topic_scraper_api")
        assert entry["present"] is True, path.name
        assert entry["enabled"] is False, path.name
        targets = data.get("monitoring_agent_node_targets") or []
        assert all(t["node"] != "topic_scraper_api" for t in targets), path.name
