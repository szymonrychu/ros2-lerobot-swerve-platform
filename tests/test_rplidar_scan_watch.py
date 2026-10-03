"""Unit tests for the RPLidar scan watchdog decision logic (nodes/bridges/rplidar_a1/scan_watch.py).

The RPLidar A1 driver sometimes wedges in its device handshake after a restart (no /scan publisher, 22% CPU spin,
2026-10-03); the supervisor restarts it when scans never start or stop arriving.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "nodes" / "bridges" / "rplidar_a1"))

from scan_watch import ScanWatch  # noqa: E402


def test_no_restart_during_startup_grace() -> None:
    watch = ScanWatch(started_at=0.0, startup_grace_s=30.0, silence_timeout_s=5.0)
    assert watch.should_restart(now=29.9) is False


def test_restart_when_no_scan_after_grace() -> None:
    watch = ScanWatch(started_at=0.0, startup_grace_s=30.0, silence_timeout_s=5.0)
    assert watch.should_restart(now=30.1) is True


def test_no_restart_while_scans_arrive() -> None:
    watch = ScanWatch(started_at=0.0, startup_grace_s=30.0, silence_timeout_s=5.0)
    for t in (2.0, 2.15, 2.3, 40.0):
        watch.on_scan(t)
    assert watch.should_restart(now=44.9) is False


def test_restart_when_scans_stop() -> None:
    watch = ScanWatch(started_at=0.0, startup_grace_s=30.0, silence_timeout_s=5.0)
    watch.on_scan(10.0)
    assert watch.should_restart(now=15.1) is True


def test_first_scan_before_grace_then_silence_uses_silence_timeout() -> None:
    watch = ScanWatch(started_at=0.0, startup_grace_s=30.0, silence_timeout_s=5.0)
    watch.on_scan(3.0)
    assert watch.should_restart(now=7.9) is False
    assert watch.should_restart(now=8.1) is True
