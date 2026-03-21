"""Unit tests for RulesObserver (compare and oscillation from scraper payloads)."""

import pytest
from topic_scraper_api.config import RULE_TYPE_COMPARE, RULE_TYPE_OSCILLATION, ObservationRule
from topic_scraper_api.observer import RulesObserver


def test_observer_empty_rules() -> None:
    obs = RulesObserver([])
    assert obs.get_rules_summary() == []
    assert obs.get_rule_result("any") is None


def test_observer_compare_rule_produces_delta() -> None:
    rule = ObservationRule(
        name="ab",
        topics=["/a", "/b"],
        rule_type=RULE_TYPE_COMPARE,
        joint_names=["joint_5", "joint_6"],
        window_s=1.0,
        variance_threshold=1e-5,
        sign_change_min_hz=2.0,
    )
    obs = RulesObserver([rule])

    def get_payload(topic: str):
        if topic == "/a":
            return {"message": {"name": ["joint_5", "joint_6"], "position": [1.0, 0.5]}}
        if topic == "/b":
            return {"message": {"name": ["joint_5", "joint_6"], "position": [0.9, 0.4]}}
        return None

    obs.tick(get_payload)
    result = obs.get_rule_result("ab")
    assert result is not None
    assert result.comparison is not None
    assert result.comparison["joint_5"] == pytest.approx(0.1)
    assert result.comparison["joint_6"] == pytest.approx(0.1)


def test_observer_compare_missing_payload_yields_none_comparison() -> None:
    rule = ObservationRule(
        name="ab",
        topics=["/a", "/b"],
        rule_type=RULE_TYPE_COMPARE,
        joint_names=[],
        window_s=1.0,
        variance_threshold=1e-5,
        sign_change_min_hz=2.0,
    )
    obs = RulesObserver([rule])

    def get_payload(topic: str):
        return None

    obs.tick(get_payload)
    result = obs.get_rule_result("ab")
    assert result is not None
    assert result.comparison is None


def test_observer_rules_summary_has_entries() -> None:
    rule = ObservationRule(
        name="r1",
        topics=["/x", "/y"],
        rule_type=RULE_TYPE_OSCILLATION,
        joint_names=["joint_5"],
        window_s=0.5,
        variance_threshold=1e-4,
        sign_change_min_hz=1.0,
    )
    obs = RulesObserver([rule])
    summary = obs.get_rules_summary()
    assert len(summary) == 1
    assert summary[0]["name"] == "r1"
    assert "has_result" in summary[0]
    assert "oscillation_detected" in summary[0]
