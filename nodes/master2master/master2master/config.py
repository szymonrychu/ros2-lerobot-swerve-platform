"""Configuration parsing and validation for master2master topic proxy."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel
from pydantic import ValidationError as PydanticValidationError
from pydantic import field_validator

SUPPORTED_DIRECTIONS = ("in", "out")
SUPPORTED_MSG_TYPES = ("string", "jointstate")
DEFAULT_CONFIG_PATH = Path("/etc/ros2/master2master/config.yaml")


class ConfigError(Exception):
    """Raised when config is invalid or missing required fields."""

    pass


def normalize_topic(topic: str) -> str:
    """Ensure topic has leading slash and no trailing slash for consistency.

    Args:
        topic: Raw topic name (str).

    Returns:
        str: Normalized topic, e.g. "/foo" or "/".
    """
    s = (topic or "").strip()
    if not s:
        return s
    if not s.startswith("/"):
        s = "/" + s
    return s.rstrip("/") or "/"


class TopicRule(BaseModel):
    """Single topic proxy rule: subscribe to source, publish to dest (optional rename).

    Attributes:
        source: Topic to subscribe to (str).
        dest: Topic to publish to (str).
        direction: "in" (server->client) or "out" (client->server).
        msg_type: "string" or "jointstate" for relay message type.
    """

    source: str
    dest: str
    direction: str = "in"
    msg_type: str = "string"

    @field_validator("source", "dest", mode="before")
    @classmethod
    def normalize_topic_field(_cls: type["TopicRule"], v: Any) -> str:
        if not isinstance(v, str):
            raise ValueError("Topic must be a string")
        return normalize_topic(v)

    @field_validator("direction", mode="before")
    @classmethod
    def check_direction(_cls: type["TopicRule"], v: Any) -> str:
        if not isinstance(v, str):
            raise ValueError("Direction must be a string")
        d = v.lower().strip()
        if d not in SUPPORTED_DIRECTIONS:
            raise ValueError(f"direction must be one of {SUPPORTED_DIRECTIONS}, got {v!r}")
        return d

    @field_validator("msg_type", mode="before")
    @classmethod
    def check_msg_type(_cls: type["TopicRule"], v: Any) -> str:
        if not isinstance(v, str):
            raise ValueError("msg_type must be a string")
        t = v.lower().strip()
        if t not in SUPPORTED_MSG_TYPES:
            raise ValueError(f"type must be one of {SUPPORTED_MSG_TYPES}, got {v!r}")
        return t


def parse_rule_entry(entry: Any, index: int) -> TopicRule | None:
    """Parse a single topic entry (dict or str) into TopicRule.

    Args:
        entry: One topic entry, either a string (topic name) or dict with
            source, dest, direction, type.
        index: Index of the entry (for error messages).

    Returns:
        TopicRule | None: Parsed rule, or None if entry has no source.

    Raises:
        ConfigError: If entry type is wrong or validation fails.
    """
    if isinstance(entry, str):
        source = normalize_topic(entry)
        if not source:
            return None
        return TopicRule(source=source, dest=source, direction="in", msg_type="string")
    if not isinstance(entry, dict):
        raise ConfigError(f"Topic entry at index {index}: expected dict or str, got {type(entry).__name__}")
    source = entry.get("source") or entry.get("from")
    if source is None or (isinstance(source, str) and not source.strip()):
        return None
    source = normalize_topic(str(source).strip())
    dest = entry.get("dest") or entry.get("to") or source
    dest = normalize_topic(str(dest).strip())
    direction = entry.get("direction") or "in"
    msg_type = entry.get("type") or "string"
    try:
        return TopicRule(
            source=source,
            dest=dest,
            direction=direction,
            msg_type=msg_type,
        )
    except PydanticValidationError as e:
        raise ConfigError(f"Topic entry at index {index}: {e}") from e


def load_config(path: Path | None = None) -> list[TopicRule]:
    """Load and validate proxy rules from YAML file.

    Args:
        path: Path to YAML config file. If None, uses DEFAULT_CONFIG_PATH.

    Returns:
        list[TopicRule]: List of validated topic rules. Empty if path is missing or empty.

    Raises:
        ConfigError: If file content is invalid.
    """
    if path is None:
        path = DEFAULT_CONFIG_PATH
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
            rule = parse_rule_entry(t, i)
        except ConfigError:
            raise
        except Exception as e:
            raise ConfigError(f"Topic entry at index {i}: {e}") from e
        if rule is not None:
            rules.append(rule)
    return rules


def load_config_from_dict(data: dict[str, Any]) -> list[TopicRule]:
    """Build and validate TopicRule list from a dict (for tests and programmatic use).

    Args:
        data: Dict with "topics" or "topic_proxy" list.

    Returns:
        list[TopicRule]: List of validated topic rules.

    Raises:
        ConfigError: If any entry is invalid.
    """
    if not data:
        return []
    topics = data.get("topics") or data.get("topic_proxy") or []
    if not isinstance(topics, list):
        raise ConfigError("Config must have 'topics' or 'topic_proxy' as a list")
    rules: list[TopicRule] = []
    for i, t in enumerate(topics):
        try:
            rule = parse_rule_entry(t, i)
        except ConfigError:
            raise
        except Exception as e:
            raise ConfigError(f"Topic entry at index {i}: {e}") from e
        if rule is not None:
            rules.append(rule)
    return rules


__all__ = [
    "ConfigError",
    "DEFAULT_CONFIG_PATH",
    "SUPPORTED_DIRECTIONS",
    "SUPPORTED_MSG_TYPES",
    "TopicRule",
    "load_config",
    "load_config_from_dict",
    "normalize_topic",
    "parse_rule_entry",
]
