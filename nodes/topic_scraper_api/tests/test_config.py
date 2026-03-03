from pathlib import Path

from topic_scraper_api.config import (
    RULE_TYPE_COMPARE,
    RULE_TYPE_OSCILLATION,
    load_config,
    parse_allowed_types,
    parse_observation_rules,
)


def test_parse_allowed_types_non_list() -> None:
    assert parse_allowed_types("sensor_msgs/msg/JointState") == []


def test_parse_allowed_types_filters_empty_values() -> None:
    out = parse_allowed_types(["sensor_msgs/msg/JointState", " ", "", "std_msgs/msg/String"])
    assert out == ["sensor_msgs/msg/JointState", "std_msgs/msg/String"]


def test_load_config_defaults(tmp_path: Path) -> None:
    config_path = tmp_path / "config.yaml"
    config_path.write_text("{}", encoding="utf-8")
    cfg = load_config(config_path)
    assert cfg is not None
    assert cfg.host == "0.0.0.0"
    assert cfg.port == 18100
    assert cfg.topic_refresh_interval_s == 0.25
    assert cfg.allowed_types == []
    assert cfg.observation_rules == []


def test_load_config_values(tmp_path: Path) -> None:
    config_path = tmp_path / "config.yaml"
    config_path.write_text(
        "\n".join(
            [
                'host: "127.0.0.1"',
                "port: 19100",
                "topic_refresh_interval_s: 0.1",
                "allowed_types:",
                "  - sensor_msgs/msg/JointState",
            ]
        ),
        encoding="utf-8",
    )
    cfg = load_config(config_path)
    assert cfg is not None
    assert cfg.host == "127.0.0.1"
    assert cfg.port == 19100
    assert cfg.topic_refresh_interval_s == 0.1
    assert cfg.allowed_types == ["sensor_msgs/msg/JointState"]


def test_parse_observation_rules_empty() -> None:
    assert parse_observation_rules(None) == []
    assert parse_observation_rules([]) == []


def test_parse_observation_rules_compare() -> None:
    raw = [
        {
            "name": "leader_follower",
            "topics": ["/filter/input_joint_updates", "/follower/joint_states"],
            "type": "compare",
            "joint_names": ["joint_5", "joint_6"],
        }
    ]
    rules = parse_observation_rules(raw)
    assert len(rules) == 1
    assert rules[0].name == "leader_follower"
    assert rules[0].topics == ["/filter/input_joint_updates", "/follower/joint_states"]
    assert rules[0].rule_type == RULE_TYPE_COMPARE
    assert rules[0].joint_names == ["joint_5", "joint_6"]


def test_load_config_observation_rules(tmp_path: Path) -> None:
    config_path = tmp_path / "config.yaml"
    config_path.write_text(
        "host: 0.0.0.0\nport: 18100\n"
        "observation_rules:\n"
        "  - name: osc\n"
        "    topics: [/a, /b]\n"
        "    type: oscillation\n"
        "    window_s: 0.5\n"
        "    variance_threshold: 1e-4\n",
        encoding="utf-8",
    )
    cfg = load_config(config_path)
    assert cfg is not None
    assert len(cfg.observation_rules) == 1
    assert cfg.observation_rules[0].name == "osc"
    assert cfg.observation_rules[0].rule_type == RULE_TYPE_OSCILLATION
    assert cfg.observation_rules[0].window_s == 0.5
    assert cfg.observation_rules[0].variance_threshold == 1e-4
