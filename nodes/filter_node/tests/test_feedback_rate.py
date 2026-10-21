"""FeedbackRateSwitch: throttled follower feedback while idle, full-rate topic while a source commands."""

import logging

import pytest

from filter_node.feedback_rate import FeedbackRateSwitch

FULL = "/follower/joint_states"
IDLE = "/follower/joint_states/throttled"
IDLE_TIMEOUT_S = 0.5
WEB_UI_TIMEOUT_S = 0.5
FALLBACK_S = 2.0
LONG_AGO = -1000.0


def make_switch(idle_topic: str = IDLE) -> FeedbackRateSwitch:
    return FeedbackRateSwitch(FULL, idle_topic, IDLE_TIMEOUT_S, WEB_UI_TIMEOUT_S, FALLBACK_S)


def select(
    switch: FeedbackRateSwitch,
    now: float,
    leader: float = LONG_AGO,
    web_ui: float = LONG_AGO,
    autonomy: bool = False,
    idle_msg: float | None = None,
) -> str:
    return switch.select(now, leader, web_ui, autonomy, idle_msg)


def test_idle_selects_the_throttled_topic() -> None:
    assert select(make_switch(), 10.0) == IDLE


def test_any_active_source_selects_full_rate() -> None:
    switch = make_switch()
    assert select(switch, 10.0, leader=9.8) == FULL
    assert select(switch, 10.0, web_ui=9.8) == FULL
    assert select(switch, 10.0, autonomy=True) == FULL


def test_return_to_idle_selects_throttled_again() -> None:
    switch = make_switch()
    assert select(switch, 10.0, leader=9.9) == FULL
    assert select(switch, 11.0, leader=9.9) == IDLE


def test_silent_idle_topic_falls_back_to_full_rate(caplog: pytest.LogCaptureFixture) -> None:
    switch = make_switch()
    with caplog.at_level(logging.WARNING):
        assert select(switch, 10.0) == IDLE
        assert select(switch, 11.9) == IDLE
        assert select(switch, 12.1) == FULL
        assert select(switch, 12.5) == FULL
    assert len([r for r in caplog.records if "throttled" in r.getMessage().lower() or IDLE in r.getMessage()]) == 1


def test_idle_topic_delivering_keeps_it_selected() -> None:
    switch = make_switch()
    assert select(switch, 10.0) == IDLE
    assert select(switch, 12.5, idle_msg=12.4) == IDLE


def test_fallback_retries_the_idle_topic_and_logs_once_per_outage(caplog: pytest.LogCaptureFixture) -> None:
    switch = make_switch()
    with caplog.at_level(logging.WARNING):
        select(switch, 10.0)
        assert select(switch, 12.1) == FULL
        assert select(switch, 12.1 + switch.retry_s + 0.1) == IDLE
        assert select(switch, 12.1 + switch.retry_s + 2.3) == FULL
    assert len(caplog.records) == 1


def test_empty_idle_topic_keeps_full_rate() -> None:
    switch = make_switch(idle_topic="")
    assert select(switch, 10.0) == FULL
    assert select(switch, 100.0, leader=9.0) == FULL
