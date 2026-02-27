"""Configuration parsing for master2master topic proxy."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass
class TopicRule:
    """Single topic proxy rule: subscribe to source, publish to dest (optional rename)."""

    source: str
    dest: str
    direction: str  # "in" (server->client) or "out" (client->server)
    msg_type: str = "string"  # "string" | "JointState" for relay message type


def load_config(path: Path | None = None) -> list[TopicRule]:
    """Load proxy rules from YAML file. Returns list of TopicRule."""
    if path is None:
        path = Path("/etc/ros2/master2master/config.yaml")
    if not path.exists():
        return []
    data = yaml.safe_load(path.read_text())
    if not data:
        return []
    topics = data.get("topics") or data.get("topic_proxy") or []
    rules: list[TopicRule] = []
    for t in topics:
        if isinstance(t, dict):
            source = t.get("source") or t.get("from")
            dest = t.get("dest") or t.get("to") or source
            direction = (t.get("direction") or "in").lower()
            msg_type = (t.get("type") or "string").lower()
            if source:
                rules.append(TopicRule(source=source, dest=dest, direction=direction, msg_type=msg_type))
        elif isinstance(t, str):
            rules.append(TopicRule(source=t, dest=t, direction="in"))
    return rules


def load_config_from_dict(data: dict[str, Any]) -> list[TopicRule]:
    """Build TopicRule list from a dict (for tests)."""
    topics = data.get("topics") or data.get("topic_proxy") or []
    rules: list[TopicRule] = []
    for t in topics:
        if isinstance(t, dict):
            source = t.get("source") or t.get("from")
            dest = t.get("dest") or t.get("to") or source
            direction = (t.get("direction") or "in").lower()
            msg_type = (t.get("type") or "string").lower()
            if source:
                rules.append(TopicRule(source=source, dest=dest, direction=direction, msg_type=msg_type))
        elif isinstance(t, str):
            rules.append(TopicRule(source=t, dest=t, direction="in"))
    return rules
