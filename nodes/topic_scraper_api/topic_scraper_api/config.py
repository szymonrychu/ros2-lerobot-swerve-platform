"""Configuration loader for topic_scraper_api."""

import os
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_CONFIG_PATH = Path("/etc/ros2/topic_scraper_api/config.yaml")
ENV_CONFIG_PATH_KEY = "TOPIC_SCRAPER_API_CONFIG"

RULE_TYPE_COMPARE = "compare"
RULE_TYPE_OSCILLATION = "oscillation"
RULE_TYPES = (RULE_TYPE_COMPARE, RULE_TYPE_OSCILLATION)


@dataclass
class ObservationRule:
    """Single observation rule: compare topics and/or detect oscillation.

    Attributes:
        name: Unique rule id for API.
        topics: List of topic paths (e.g. leader, follower, filter input).
        rule_type: "compare" or "oscillation".
        joint_names: Optional joint names for JointState (empty = all).
        window_s: Time window for oscillation (seconds).
        variance_threshold: Oscillation: position-delta variance above this flags oscillation.
        sign_change_min_hz: Oscillation: min sign-change rate (Hz) to flag oscillation.
    """

    name: str
    topics: list[str]
    rule_type: str
    joint_names: list[str]
    window_s: float
    variance_threshold: float
    sign_change_min_hz: float


@dataclass
class ApiConfig:
    """HTTP server and topic refresh configuration.

    Attributes:
        host: Bind host, for example "0.0.0.0".
        port: HTTP port.
        topic_refresh_interval_s: How often to refresh ROS graph subscriptions.
        allowed_types: Optional message type allow-list (e.g. sensor_msgs/msg/JointState).
        observation_rules: Optional list of rules for compare/oscillation.
    """

    host: str
    port: int
    topic_refresh_interval_s: float
    allowed_types: list[str]
    observation_rules: list[ObservationRule]


def parse_observation_rules(raw: object) -> list[ObservationRule]:
    """Parse observation_rules from YAML.

    Each entry: name, topics (list), type ("compare"|"oscillation"),
    optional joint_names, window_s, variance_threshold, sign_change_min_hz.
    """
    if not isinstance(raw, list):
        return []
    out: list[ObservationRule] = []
    for i, item in enumerate(raw):
        if not isinstance(item, dict):
            continue
        name = str(item.get("name") or f"rule_{i}").strip()
        if not name:
            continue
        raw_topics = item.get("topics") or item.get("topic_list") or []
        topics = [str(t).strip() for t in raw_topics if str(t).strip()] if isinstance(raw_topics, list) else []
        if not topics:
            continue
        rule_type = str(item.get("type") or RULE_TYPE_COMPARE).strip().lower()
        if rule_type not in RULE_TYPES:
            rule_type = RULE_TYPE_COMPARE
        raw_joints = item.get("joint_names") or []
        joint_names = [str(j).strip() for j in raw_joints if str(j).strip()] if isinstance(raw_joints, list) else []
        try:
            window_s = max(0.1, float(item.get("window_s", 1.0)))
        except (TypeError, ValueError):
            window_s = 1.0
        try:
            variance_threshold = max(0.0, float(item.get("variance_threshold", 1e-6)))
        except (TypeError, ValueError):
            variance_threshold = 1e-6
        try:
            sign_change_min_hz = max(0.0, float(item.get("sign_change_min_hz", 2.0)))
        except (TypeError, ValueError):
            sign_change_min_hz = 2.0
        out.append(
            ObservationRule(
                name=name,
                topics=topics,
                rule_type=rule_type,
                joint_names=joint_names,
                window_s=window_s,
                variance_threshold=variance_threshold,
                sign_change_min_hz=sign_change_min_hz,
            )
        )
    return out


def parse_allowed_types(raw: object) -> list[str]:
    """Normalize allowed topic types.

    Args:
        raw: Any YAML value under the `allowed_types` key.

    Returns:
        list[str]: Normalized list of non-empty type strings.
    """

    if not isinstance(raw, list):
        return []
    out: list[str] = []
    for item in raw:
        value = str(item).strip()
        if value:
            out.append(value)
    return out


def load_config(path: Path | None = None) -> ApiConfig | None:
    """Load config from YAML file.

    Args:
        path: Path to YAML file. Uses DEFAULT_CONFIG_PATH when None.

    Returns:
        ApiConfig | None: Parsed configuration or None when missing/invalid.
    """

    if path is None:
        path = DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict):
        return None

    host = str(data.get("host") or "0.0.0.0").strip()
    try:
        port = int(data.get("port", 18100))
    except (TypeError, ValueError):
        port = 18100
    try:
        refresh_interval = float(data.get("topic_refresh_interval_s", 0.25))
    except (TypeError, ValueError):
        refresh_interval = 0.25
    if refresh_interval <= 0.0:
        refresh_interval = 0.25

    observation_rules = parse_observation_rules(data.get("observation_rules"))

    return ApiConfig(
        host=host,
        port=port,
        topic_refresh_interval_s=refresh_interval,
        allowed_types=parse_allowed_types(data.get("allowed_types")),
        observation_rules=observation_rules,
    )


def load_config_from_env() -> ApiConfig | None:
    """Load config path from TOPIC_SCRAPER_API_CONFIG env or default path."""

    path_str = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    path = Path(path_str) if path_str else DEFAULT_CONFIG_PATH
    return load_config(path)
