"""Tests for log throttling helpers, the ws implementation choice and the stale-topic warning rate."""

from __future__ import annotations

from types import SimpleNamespace

import uvicorn.config

from web_ui.bridge import TOPIC_STALE_S, BridgeNode
from web_ui.server import UVICORN_WS_IMPL, build_uvicorn_kwargs
from web_ui.throttle import (
    SLOW_CYCLE_THRESHOLD_MS,
    WARN_INTERVAL_S,
    SlowCycleTracker,
    WarnThrottle,
)


def test_ws_impl_is_not_legacy_and_exists() -> None:
    assert UVICORN_WS_IMPL != "websockets"
    assert UVICORN_WS_IMPL in uvicorn.config.WS_PROTOCOLS
    assert build_uvicorn_kwargs(8080)["ws"] == UVICORN_WS_IMPL


def test_fast_work_with_long_sleep_is_not_slow() -> None:
    tracker = SlowCycleTracker()
    # the sleep is not part of the measured work: only work_ms is passed in
    assert tracker.record(work_ms=5.0, now=0.0) is None
    assert tracker.record(work_ms=SLOW_CYCLE_THRESHOLD_MS, now=1.0) is None


def test_many_slow_cycles_give_one_warning_with_count_and_max() -> None:
    tracker = SlowCycleTracker()
    reports = [tracker.record(work_ms=100.0 + i, now=float(i)) for i in range(50)]
    first = reports[0]
    assert first == {"slow_cycles": 1, "max_duration_ms": 100}
    assert all(r is None for r in reports[1:])
    report = tracker.record(work_ms=200.0, now=WARN_INTERVAL_S + 1)
    assert report == {"slow_cycles": 50, "max_duration_ms": 200}


def test_fast_cycle_flushes_pending_report_after_window() -> None:
    tracker = SlowCycleTracker()
    assert tracker.record(work_ms=90.0, now=0.0) is not None
    assert tracker.record(work_ms=80.0, now=1.0) is None
    assert tracker.record(work_ms=1.0, now=WARN_INTERVAL_S + 1) == {"slow_cycles": 1, "max_duration_ms": 80}
    assert tracker.record(work_ms=1.0, now=WARN_INTERVAL_S + 2) is None


def test_warn_throttle_once_per_interval_per_key() -> None:
    throttle = WarnThrottle()
    assert throttle.allow("a", now=0.0)
    assert not throttle.allow("a", now=10.0)
    assert throttle.allow("b", now=10.0)
    assert throttle.allow("a", now=WARN_INTERVAL_S)


def test_topic_stale_warning_throttled_per_topic(monkeypatch) -> None:
    import web_ui.bridge as bridge

    events: list[str] = []
    monkeypatch.setattr(bridge.log, "warning", lambda _event, **kw: events.append(kw["topic"]))
    clock = {"t": 1000.0}
    monkeypatch.setattr(bridge.time, "monotonic", lambda: clock["t"])
    fake = SimpleNamespace(
        _topic_last_rx={"/a": 0.0, "/b": 0.0},
        _stale_warn=WarnThrottle(),
    )
    for _ in range(6):  # six checks, TOPIC_STALE_S apart: 50 s
        BridgeNode._check_topic_health(fake)
        clock["t"] += TOPIC_STALE_S
    assert sorted(events) == ["/a", "/b"]
    clock["t"] += WARN_INTERVAL_S
    BridgeNode._check_topic_health(fake)
    assert sorted(events) == ["/a", "/a", "/b", "/b"]
