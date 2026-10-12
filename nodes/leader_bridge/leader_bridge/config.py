"""Configuration for leader_bridge: topics, QoS and drop rules, validated by pydantic from a YAML file."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator, model_validator

DEFAULT_CONFIG_PATH = Path("/etc/ros2/leader_bridge/config.yaml")
CONFIG_ENV_VAR = "LEADER_BRIDGE_CONFIG"
DEFAULT_SOURCE_TOPIC = "/leader/joint_states"
DEFAULT_DEST_TOPIC = "/filter/input_joint_updates"
# Topics the bridge must never publish to: each would bypass or impersonate filter_node arbitration (follower output,
# the web UI and autonomy command inputs and lease release, the arbiter's report and the follower feedback it compares
# the leader against for the takeover_threshold_rad proximity hand-back).
ARBITRATION_BYPASS_TOPICS = frozenset(
    {
        "/follower/joint_commands",
        "/filter/web_ui_joint_commands",
        "/filter/autonomy_joint_commands",
        "/filter/autonomy_release",
        "/filter/active_source",
        "/follower/joint_states",
    }
)


class ConfigError(Exception):
    """Raised when the config file is missing, unreadable or invalid."""


def normalize_topic(topic: str) -> str:
    """Return topic with exactly one leading slash and no trailing slash.

    Args:
        topic (str): Raw topic name.

    Returns:
        str: Normalized topic, e.g. "/leader/joint_states".
    """
    return "/" + topic.strip().strip("/")


class QosSettings(BaseModel):
    """QoS of one endpoint (KEEP_LAST history).

    Attributes:
        reliability (str): "reliable" or "best_effort".
        depth (int): KEEP_LAST queue depth, at least 1.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    reliability: Literal["reliable", "best_effort"] = "reliable"
    depth: int = Field(default=10, ge=1)


class BridgeConfig(BaseModel):
    """leader_bridge settings.

    Attributes:
        source_topic (str): Leader joint states to read (published by lerobot_leader on the server).
        dest_topic (str): filter_node leader input to write (the arbitrated, proximity-gated input).
        subscribe_qos (QosSettings): Subscription QoS; BEST_EFFORT matches any publisher.
        publish_qos (QosSettings): Publisher QoS; RELIABLE matches filter_node's RELIABLE subscription.
        max_age_s (float): Drop a message whose header stamp is older than this; 0 disables the check.
        max_future_s (float): Drop a message stamped further than this in the future (cross-host clock skew bound).
        drop_log_period_s (float): Minimum seconds between two drop warnings of the same reason.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    source_topic: str = DEFAULT_SOURCE_TOPIC
    dest_topic: str = DEFAULT_DEST_TOPIC
    subscribe_qos: QosSettings = QosSettings(reliability="best_effort", depth=5)
    publish_qos: QosSettings = QosSettings(reliability="reliable", depth=10)
    max_age_s: float = Field(default=0.5, ge=0.0)
    max_future_s: float = Field(default=0.2, ge=0.0)
    drop_log_period_s: float = Field(default=5.0, ge=0.0)

    @field_validator("source_topic", "dest_topic")
    @classmethod
    def check_topic(_cls, value: str) -> str:
        """Normalize a topic name and reject an empty one.

        Args:
            value (str): Raw topic.

        Returns:
            str: Normalized topic.
        """
        topic = normalize_topic(value)
        if topic == "/":
            raise ValueError("topic must not be empty")
        return topic

    @model_validator(mode="after")
    def check_route(self) -> BridgeConfig:
        """Reject a relay loop and any destination that bypasses filter_node arbitration.

        Returns:
            BridgeConfig: self, unchanged.
        """
        if self.source_topic == self.dest_topic:
            raise ValueError(f"relay loop: dest_topic equals source_topic {self.source_topic!r}")
        if self.dest_topic in ARBITRATION_BYPASS_TOPICS:
            raise ValueError(
                f"dest_topic {self.dest_topic!r} bypasses filter_node arbitration; "
                f"publish only to the leader input ({DEFAULT_DEST_TOPIC})"
            )
        return self


def load_config(path: Path) -> BridgeConfig:
    """Load and validate the bridge config from a YAML file; an empty file gives the defaults.

    Args:
        path (Path): YAML config file.

    Returns:
        BridgeConfig: Validated settings.

    Raises:
        ConfigError: If the file is missing, is not YAML, is not a mapping or fails validation.
    """
    if not path.is_file():
        raise ConfigError(f"config file not found: {path}")
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as e:
        raise ConfigError(f"{path}: invalid YAML: {e}") from e
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: expected a mapping at the top level, got {type(data).__name__}")
    try:
        return BridgeConfig.model_validate(data)
    except ValidationError as e:
        raise ConfigError(f"{path}: {e}") from e
