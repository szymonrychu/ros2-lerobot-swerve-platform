"""Unit tests for master2master config parsing."""

import sys
from pathlib import Path

# Add nodes/master2master so we can import master2master package
_nodes_master2master = Path(__file__).resolve().parent.parent / "nodes" / "master2master"
if _nodes_master2master.exists():
    sys.path.insert(0, str(_nodes_master2master))

from master2master.config import load_config_from_dict  # noqa: E402


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
