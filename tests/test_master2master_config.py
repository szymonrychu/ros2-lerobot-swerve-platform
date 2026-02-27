"""Unit tests for master2master config parsing."""

import sys
from pathlib import Path

import pytest

# Add nodes/master2master so we can import master2master package
_nodes_master2master = Path(__file__).resolve().parent.parent / "nodes" / "master2master"
if _nodes_master2master.exists():
    sys.path.insert(0, str(_nodes_master2master))

from master2master.config import (  # noqa: E402
    ConfigError,
    load_config,
    load_config_from_dict,
    normalize_topic,
    parse_rule_entry,
)


def test_load_config_from_dict_empty() -> None:
    """Empty or missing topics returns empty list."""
    assert load_config_from_dict({}) == []
    assert load_config_from_dict({"topics": []}) == []
    assert load_config_from_dict({"topic_proxy": []}) == []


def test_load_config_from_dict_string_entries() -> None:
    """String entries become TopicRule with source=dest, direction=in."""
    rules = load_config_from_dict({"topics": ["/foo", "/bar"]})
    assert len(rules) == 2
    assert rules[0].source == "/foo" and rules[0].dest == "/foo" and rules[0].direction == "in"
    assert rules[1].source == "/bar" and rules[1].dest == "/bar" and rules[1].direction == "in"


def test_load_config_from_dict_dict_entries() -> None:
    """Dict entries support source, dest, direction."""
    rules = load_config_from_dict(
        {
            "topics": [
                {"source": "/a", "dest": "/b", "direction": "out"},
                {"from": "/c", "to": "/d"},
            ],
        }
    )
    assert len(rules) == 2
    assert rules[0].source == "/a" and rules[0].dest == "/b" and rules[0].direction == "out"
    assert rules[1].source == "/c" and rules[1].dest == "/d" and rules[1].direction == "in"


def test_load_config_from_dict_skips_missing_source() -> None:
    """Entries without source are skipped."""
    rules = load_config_from_dict({"topics": [{"dest": "/only"}]})
    assert rules == []


def test_load_config_from_dict_msg_type() -> None:
    """Dict entries support type (string | JointState); default is string."""
    rules = load_config_from_dict(
        {
            "topics": [
                {"source": "/a", "type": "JointState"},
                {"source": "/b"},
            ],
        }
    )
    assert len(rules) == 2
    assert rules[0].msg_type == "jointstate"
    assert rules[1].msg_type == "string"


def test_load_config_from_dict_invalid_direction_raises() -> None:
    """Invalid direction raises ConfigError."""
    with pytest.raises(ConfigError, match="direction must be one of"):
        load_config_from_dict({"topics": [{"source": "/a", "direction": "invalid"}]})


def test_load_config_from_dict_invalid_type_raises() -> None:
    """Invalid type raises ConfigError."""
    with pytest.raises(ConfigError, match="type must be one of"):
        load_config_from_dict({"topics": [{"source": "/a", "type": "Image"}]})


def test_load_config_from_dict_topics_not_list_raises() -> None:
    """topics must be a list; otherwise ConfigError."""
    with pytest.raises(ConfigError, match="as a list"):
        load_config_from_dict({"topics": "not a list"})


def test_load_config_from_dict_normalizes_topic_slash() -> None:
    """Topic names get leading slash if missing."""
    rules = load_config_from_dict({"topics": [{"source": "foo", "dest": "bar"}]})
    assert len(rules) == 1
    assert rules[0].source == "/foo"
    assert rules[0].dest == "/bar"


def test_normalize_topic_empty_or_whitespace() -> None:
    """normalize_topic returns empty string for empty or whitespace-only input."""
    assert normalize_topic("") == ""
    assert normalize_topic("   ") == ""


def test_normalize_topic_adds_leading_slash() -> None:
    """normalize_topic adds leading slash when missing."""
    assert normalize_topic("foo") == "/foo"
    assert normalize_topic("  bar  ") == "/bar"


def test_normalize_topic_strips_trailing_slash() -> None:
    """normalize_topic strips trailing slash; single slash becomes '/'."""
    assert normalize_topic("/foo/") == "/foo"
    assert normalize_topic("/") == "/"


def test_load_config_from_file(tmp_path: Path) -> None:
    """load_config reads and parses YAML from file path."""
    config_file = tmp_path / "config.yaml"
    config_file.write_text("topics:\n  - /topic_a\n  - source: /b\n    dest: /c\n")
    rules = load_config(config_file)
    assert len(rules) == 2
    assert rules[0].source == "/topic_a" and rules[0].dest == "/topic_a"
    assert rules[1].source == "/b" and rules[1].dest == "/c"


def test_load_config_nonexistent_returns_empty() -> None:
    """load_config returns empty list for nonexistent path."""
    assert load_config(Path("/nonexistent/config.yaml")) == []


def test_parse_rule_entry_invalid_type_raises() -> None:
    """parse_rule_entry raises ConfigError for non-dict non-str entry."""
    with pytest.raises(ConfigError, match="expected dict or str"):
        parse_rule_entry(123, 0)
    with pytest.raises(ConfigError, match="expected dict or str"):
        parse_rule_entry([], 1)
