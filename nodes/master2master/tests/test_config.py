"""Unit tests for master2master config loading and validation."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest
from master2master.config import (
    ConfigError,
    TopicRule,
    load_config,
    load_config_from_dict,
    normalize_topic,
    parse_rule_entry,
    validate_relay_rules,
)
from pydantic import ValidationError

# ---------------------------------------------------------------------------
# normalize_topic
# ---------------------------------------------------------------------------


def test_normalize_topic_adds_leading_slash() -> None:
    """normalize_topic prepends slash to bare topic name."""
    assert normalize_topic("foo") == "/foo"


def test_normalize_topic_strips_trailing_slash() -> None:
    """normalize_topic removes trailing slashes."""
    assert normalize_topic("/foo/") == "/foo"


def test_normalize_topic_already_normalised() -> None:
    """normalize_topic is idempotent for already-normalised names."""
    assert normalize_topic("/foo/bar") == "/foo/bar"


def test_normalize_topic_empty_string() -> None:
    """normalize_topic returns empty string for blank input."""
    assert normalize_topic("") == ""


def test_normalize_topic_root() -> None:
    """normalize_topic returns '/' for a lone slash."""
    assert normalize_topic("/") == "/"


# ---------------------------------------------------------------------------
# TopicRule construction
# ---------------------------------------------------------------------------


def test_topic_rule_valid_defaults() -> None:
    """TopicRule can be constructed with just source and dest; defaults apply."""
    rule = TopicRule(source="/foo", dest="/bar")
    assert rule.source == "/foo"
    assert rule.dest == "/bar"
    assert rule.direction == "in"
    assert rule.msg_type == "string"


def test_topic_rule_normalises_topics() -> None:
    """TopicRule normalises topics that lack a leading slash."""
    rule = TopicRule(source="foo", dest="bar")
    assert rule.source == "/foo"
    assert rule.dest == "/bar"


def test_topic_rule_valid_direction_out() -> None:
    """TopicRule accepts 'out' as direction."""
    rule = TopicRule(source="/a", dest="/b", direction="out")
    assert rule.direction == "out"


def test_topic_rule_valid_msg_type_jointstate() -> None:
    """TopicRule accepts 'jointstate' msg_type."""
    rule = TopicRule(source="/a", dest="/b", msg_type="jointstate")
    assert rule.msg_type == "jointstate"


def test_topic_rule_direction_case_insensitive() -> None:
    """TopicRule normalises direction to lowercase."""
    rule = TopicRule(source="/a", dest="/b", direction="IN")
    assert rule.direction == "in"


def test_topic_rule_msg_type_case_insensitive() -> None:
    """TopicRule normalises msg_type to lowercase."""
    rule = TopicRule(source="/a", dest="/b", msg_type="JointState")
    assert rule.msg_type == "jointstate"


def test_topic_rule_invalid_direction_raises() -> None:
    """TopicRule raises ValidationError for unknown direction."""
    with pytest.raises(ValidationError):
        TopicRule(source="/a", dest="/b", direction="sideways")


def test_topic_rule_invalid_msg_type_raises() -> None:
    """TopicRule raises ValidationError for unknown msg_type."""
    with pytest.raises(ValidationError):
        TopicRule(source="/a", dest="/b", msg_type="image")


def test_topic_rule_non_string_source_raises() -> None:
    """TopicRule raises ValidationError when source is not a string."""
    with pytest.raises(ValidationError):
        TopicRule(source=123, dest="/b")  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# parse_rule_entry
# ---------------------------------------------------------------------------


def test_parse_rule_entry_string() -> None:
    """parse_rule_entry creates identical source/dest from a plain string."""
    rule = parse_rule_entry("/topic", 0)
    assert rule is not None
    assert rule.source == "/topic"
    assert rule.dest == "/topic"


def test_parse_rule_entry_dict_full() -> None:
    """parse_rule_entry parses a fully-specified dict entry."""
    entry: dict[str, Any] = {
        "source": "/src",
        "dest": "/dst",
        "direction": "out",
        "type": "jointstate",
    }
    rule = parse_rule_entry(entry, 0)
    assert rule is not None
    assert rule.source == "/src"
    assert rule.dest == "/dst"
    assert rule.direction == "out"
    assert rule.msg_type == "jointstate"


def test_parse_rule_entry_dict_uses_from_to_aliases() -> None:
    """parse_rule_entry accepts 'from'/'to' as aliases for source/dest."""
    entry: dict[str, Any] = {"from": "/a", "to": "/b"}
    rule = parse_rule_entry(entry, 0)
    assert rule is not None
    assert rule.source == "/a"
    assert rule.dest == "/b"


def test_parse_rule_entry_dict_dest_defaults_to_source() -> None:
    """parse_rule_entry uses source as dest when dest is omitted."""
    entry: dict[str, Any] = {"source": "/only"}
    rule = parse_rule_entry(entry, 0)
    assert rule is not None
    assert rule.dest == "/only"


def test_parse_rule_entry_no_source_returns_none() -> None:
    """parse_rule_entry returns None when source is missing."""
    assert parse_rule_entry({}, 0) is None


def test_parse_rule_entry_empty_string_returns_none() -> None:
    """parse_rule_entry returns None for empty string input."""
    assert parse_rule_entry("", 0) is None


def test_parse_rule_entry_invalid_type_raises() -> None:
    """parse_rule_entry raises ConfigError for non-dict, non-str input."""
    with pytest.raises(ConfigError):
        parse_rule_entry(42, 0)


def test_parse_rule_entry_invalid_direction_raises() -> None:
    """parse_rule_entry raises ConfigError when direction is unknown."""
    entry: dict[str, Any] = {"source": "/a", "dest": "/b", "direction": "bad"}
    with pytest.raises(ConfigError):
        parse_rule_entry(entry, 0)


# ---------------------------------------------------------------------------
# load_config_from_dict
# ---------------------------------------------------------------------------


def test_load_config_from_dict_empty_returns_empty() -> None:
    """load_config_from_dict returns [] for empty dict."""
    assert load_config_from_dict({}) == []


def test_load_config_from_dict_valid() -> None:
    """load_config_from_dict parses a valid topics list correctly."""
    data: dict[str, Any] = {
        "topics": [
            {"source": "/server/cmd_vel", "dest": "/client/cmd_vel", "direction": "in"},
            {"source": "/client/odom", "dest": "/server/odom", "direction": "out"},
        ]
    }
    rules = load_config_from_dict(data)
    assert len(rules) == 2
    assert rules[0].source == "/server/cmd_vel"
    assert rules[0].dest == "/client/cmd_vel"
    assert rules[1].direction == "out"


def test_load_config_from_dict_topic_proxy_key() -> None:
    """load_config_from_dict also accepts 'topic_proxy' as the list key."""
    data: dict[str, Any] = {
        "topic_proxy": [
            {"source": "/a", "dest": "/b"},
        ]
    }
    rules = load_config_from_dict(data)
    assert len(rules) == 1
    assert rules[0].source == "/a"


def test_load_config_from_dict_empty_topics_list() -> None:
    """load_config_from_dict returns [] when topics list is empty."""
    rules = load_config_from_dict({"topics": []})
    assert rules == []


def test_load_config_from_dict_skips_none_entries() -> None:
    """load_config_from_dict skips entries that parse to None (missing source)."""
    data: dict[str, Any] = {
        "topics": [
            {"source": "/a", "dest": "/b"},
            {},  # no source -> None -> skipped
        ]
    }
    rules = load_config_from_dict(data)
    assert len(rules) == 1


def test_load_config_from_dict_invalid_topics_type_raises() -> None:
    """load_config_from_dict raises ConfigError when 'topics' is not a list."""
    with pytest.raises(ConfigError):
        load_config_from_dict({"topics": "not-a-list"})


def test_load_config_from_dict_realistic_complete() -> None:
    """load_config_from_dict handles a realistic multi-rule config correctly."""
    data: dict[str, Any] = {
        "topics": [
            {"source": "/server/cmd_vel", "dest": "/client/cmd_vel", "direction": "in"},
            {"source": "/client/odom", "dest": "/server/odom", "direction": "out"},
            {
                "source": "/leader/joint_states",
                "dest": "/leader/joint_states",
                "direction": "in",
                "type": "jointstate",
            },
        ]
    }
    rules = load_config_from_dict(data)
    assert len(rules) == 3
    assert rules[2].msg_type == "jointstate"
    assert rules[2].source == "/leader/joint_states"


def test_load_config_from_dict_duplicate_topic_names_allowed() -> None:
    """Duplicate source names are not rejected at load time (relay loop guard is separate)."""
    data: dict[str, Any] = {
        "topics": [
            {"source": "/a", "dest": "/b"},
            {"source": "/a", "dest": "/c"},
        ]
    }
    rules = load_config_from_dict(data)
    assert len(rules) == 2


# ---------------------------------------------------------------------------
# load_config (file-based)
# ---------------------------------------------------------------------------


def test_load_config_missing_file_returns_empty() -> None:
    """load_config returns [] when the config file does not exist."""
    rules = load_config(Path("/nonexistent/config.yaml"))
    assert rules == []


def test_load_config_empty_file_returns_empty(tmp_path: Path) -> None:
    """load_config returns [] for an empty YAML file."""
    p = tmp_path / "config.yaml"
    p.write_text("")
    rules = load_config(p)
    assert rules == []


def test_load_config_valid_file(tmp_path: Path) -> None:
    """load_config parses a well-formed YAML file into TopicRules."""
    p = tmp_path / "config.yaml"
    p.write_text("topics:\n" "  - source: /server/cmd_vel\n" "    dest: /client/cmd_vel\n" "    direction: in\n")
    rules = load_config(p)
    assert len(rules) == 1
    assert rules[0].source == "/server/cmd_vel"


# ---------------------------------------------------------------------------
# validate_relay_rules
# ---------------------------------------------------------------------------


def test_validate_relay_rules_no_loops_passes() -> None:
    """validate_relay_rules does not raise when there are no feedback loops."""
    rules = [
        TopicRule(source="/a", dest="/b"),
        TopicRule(source="/c", dest="/d"),
    ]
    # Should not raise
    validate_relay_rules(rules)


def test_validate_relay_rules_detects_loop() -> None:
    """validate_relay_rules raises ValueError when dest of one rule is source of another."""
    rules = [
        TopicRule(source="/a", dest="/b"),
        TopicRule(source="/b", dest="/c"),
    ]
    with pytest.raises(ValueError, match="Relay loop guard"):
        validate_relay_rules(rules)


def test_validate_relay_rules_empty_list_passes() -> None:
    """validate_relay_rules does not raise for an empty list."""
    validate_relay_rules([])
