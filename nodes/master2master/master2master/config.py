"""Configuration parsing and validation for master2master topic proxy."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

SUPPORTED_DIRECTIONS = ("in", "out")
SUPPORTED_MSG_TYPES = ("string", "jointstate")


class ConfigError(Exception):
    """Raised when config is invalid or missing required fields."""

    pass


@dataclass
class TopicRule:
    """Single topic proxy rule: subscribe to source, publish to dest (optional rename)."""

    source: str
    dest: str
    direction: str  # "in" (server->client) or "out" (client->server)
    msg_type: str = "string"  # "string" | "jointstate" for relay message type


def _normalize_topic(topic: str) -> str:
    """Ensure topic has leading slash and no trailing slash for consistency."""
    s = (topic or "").strip()
    if not s:
        return s
    if not s.startswith("/"):
        s = "/" + s
    return s.rstrip("/") or "/"


def _parse_rule_entry(t: Any, index: int) -> TopicRule | None:
    """Parse a single topic entry (dict or str) into TopicRule. Returns None if skipped (e.g. no source)."""
    if isinstance(t, str):
        source = _normalize_topic(t)
        if not source:
            return None
        return TopicRule(source=source, dest=source, direction="in", msg_type="string")
    if not isinstance(t, dict):
        raise ConfigError(f"Topic entry at index {index}: expected dict or str, got {type(t).__name__}")
    source = t.get("source") or t.get("from")
    if source is None or (isinstance(source, str) and not source.strip()):
        return None
    source = _normalize_topic(str(source).strip())
    dest = t.get("dest") or t.get("to") or source
    dest = _normalize_topic(str(dest).strip())
    direction = t.get("direction") or "in"
    if isinstance(direction, str):
        direction = direction.lower().strip()
    if direction not in SUPPORTED_DIRECTIONS:
        raise ConfigError(
            f"Topic entry at index {index} (source={source!r}): direction must be one of "
            f"{SUPPORTED_DIRECTIONS}, got {direction!r}"
        )
    msg_type = t.get("type") or "string"
    if isinstance(msg_type, str):
        msg_type = msg_type.lower().strip()
    if msg_type not in SUPPORTED_MSG_TYPES:
        raise ConfigError(
            f"Topic entry at index {index} (source={source!r}): type must be one of "
            f"{SUPPORTED_MSG_TYPES}, got {msg_type!r}"
        )
    return TopicRule(source=source, dest=dest, direction=direction, msg_type=msg_type)


def load_config(path: Path | None = None) -> list[TopicRule]:
    """Load and validate proxy rules from YAML file. Returns list of TopicRule.

    Raises ConfigError if file content is invalid. Returns [] if path is missing or empty.
    """
    if path is None:
        path = Path("/etc/ros2/master2master/config.yaml")
    if not path.exists():
        return []
    data = yaml.safe_load(path.read_text())
    if not data:
        return []
    topics = data.get("topics") or data.get("topic_proxy") or []
    if not isinstance(topics, list):
        raise ConfigError("Config must have 'topics' or 'topic_proxy' as a list")
    rules: list[TopicRule] = []
    for i, t in enumerate(topics):
        try:
            rule = _parse_rule_entry(t, i)
        except ConfigError:
            raise
        except Exception as e:
            raise ConfigError(f"Topic entry at index {i}: {e}") from e
        if rule is not None:
            rules.append(rule)
    return rules


def load_config_from_dict(data: dict[str, Any]) -> list[TopicRule]:
    """Build and validate TopicRule list from a dict (for tests and programmatic use).

    Raises ConfigError if any entry is invalid.
    """
    if not data:
        return []
    topics = data.get("topics") or data.get("topic_proxy") or []
    if not isinstance(topics, list):
        raise ConfigError("Config must have 'topics' or 'topic_proxy' as a list")
    rules: list[TopicRule] = []
    for i, t in enumerate(topics):
        try:
            rule = _parse_rule_entry(t, i)
        except ConfigError:
            raise
        except Exception as e:
            raise ConfigError(f"Topic entry at index {i}: {e}") from e
        if rule is not None:
            rules.append(rule)
    return rules


__all__ = [
    "ConfigError",
    "SUPPORTED_DIRECTIONS",
    "SUPPORTED_MSG_TYPES",
    "TopicRule",
    "load_config",
    "load_config_from_dict",
]
