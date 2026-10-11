"""Client topic_scraper_api config: which message types the debug scraper subscribes to."""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"


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
