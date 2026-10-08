"""Executor spin loop hardening and the frame cache behind persistent camera subscriptions (no ROS)."""

import threading
import time

import pytest

from mcp_server.spin import FrameCache, run_or_exit, spin_forever


class StaleHandle(Exception):
    """Stands in for rclpy InvalidHandle (an entity destroyed while the executor built its wait set)."""


def test_spin_forever_survives_a_stale_handle_and_keeps_spinning() -> None:
    calls: list[int] = []

    def spin_once() -> None:
        calls.append(1)
        if len(calls) == 2:
            raise StaleHandle("cannot use Destroyable because destruction was requested")

    spin_forever(spin_once, ok=lambda: len(calls) < 5, tolerated=(StaleHandle,))
    assert len(calls) == 5


def test_spin_forever_propagates_other_errors() -> None:
    def spin_once() -> None:
        raise RuntimeError("boom")

    with pytest.raises(RuntimeError):
        spin_forever(spin_once, ok=lambda: True, tolerated=(StaleHandle,))


def test_run_or_exit_exits_when_the_spinner_dies() -> None:
    """A dead executor thread must not leave a process that serves stale data: exit so systemd restarts it."""
    exits: list[int] = []

    def target() -> None:
        raise RuntimeError("executor died")

    run_or_exit(target, ok=lambda: True, exit_fn=exits.append)
    assert exits == [1]


def test_run_or_exit_returns_quietly_on_shutdown() -> None:
    exits: list[int] = []
    run_or_exit(lambda: None, ok=lambda: False, exit_fn=exits.append)
    assert exits == []


def test_run_or_exit_exits_when_the_spinner_returns_while_running() -> None:
    exits: list[int] = []
    run_or_exit(lambda: None, ok=lambda: True, exit_fn=exits.append)
    assert exits == [1]


def test_frame_cache_waits_for_a_frame_newer_than_the_request() -> None:
    now = [10.0]
    cache = FrameCache(clock=lambda: now[0])
    cache.update("old")  # arrived at 10.0
    now[0] = 11.0
    since = now[0]

    def later() -> None:
        time.sleep(0.05)
        now[0] = 11.5
        cache.update("new")

    threading.Thread(target=later).start()
    assert cache.wait_newer(since, timeout=2.0) == "new"


def test_frame_cache_times_out_without_a_new_frame() -> None:
    now = [10.0]
    cache = FrameCache(clock=lambda: now[0])
    cache.update("old")
    assert cache.wait_newer(10.5, timeout=0.05) is None
