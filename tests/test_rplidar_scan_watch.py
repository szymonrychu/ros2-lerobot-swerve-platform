"""Unit tests for the RPLidar scan watchdog decision logic (nodes/bridges/rplidar_a1/scan_watch.py).

The RPLidar A1 driver sometimes wedges in its device handshake after a restart (no /scan publisher, 22% CPU spin,
2026-10-03); the supervisor restarts it when scans never start or stop arriving.
"""

import sys
from pathlib import Path

import pytest

RPLIDAR_DIR = Path(__file__).resolve().parent.parent / "nodes" / "bridges" / "rplidar_a1"
SHARED_METRICS_DIR = Path(__file__).resolve().parent.parent / "shared" / "ros2_metrics"
sys.path.insert(0, str(RPLIDAR_DIR))
sys.path.insert(0, str(SHARED_METRICS_DIR))

from prometheus_client import REGISTRY  # noqa: E402
from scan_watch import ScanWatch  # noqa: E402


def sample(name: str, labels: dict[str, str] | None = None) -> float | None:
    """Return the current sample value, None when the series does not exist."""
    return REGISTRY.get_sample_value(name, labels or {})


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


def test_restart_reason_distinguishes_startup_from_gap() -> None:
    watch = ScanWatch(started_at=0.0, startup_grace_s=30.0, silence_timeout_s=5.0)
    assert watch.restart_reason(now=10.0) is None
    assert watch.restart_reason(now=31.0) == "startup"
    watch.on_scan(40.0)
    assert watch.restart_reason(now=42.0) is None
    assert watch.restart_reason(now=45.5) == "gap"


def test_rate_over_recent_scans() -> None:
    watch = ScanWatch(started_at=0.0, startup_grace_s=30.0, silence_timeout_s=5.0)
    for i in range(21):
        watch.on_scan(1.0 + i * 0.1)  # 10 Hz for 2 s
    assert watch.rate_hz(now=3.0) == pytest.approx(10.0)


def test_rate_unknown_until_two_scans_and_zero_after_silence() -> None:
    watch = ScanWatch(started_at=0.0, startup_grace_s=30.0, silence_timeout_s=5.0)
    assert watch.rate_hz(now=1.0) is None
    watch.on_scan(1.0)
    assert watch.rate_hz(now=1.0) is None
    watch.on_scan(1.1)
    assert watch.rate_hz(now=1.1) == pytest.approx(10.0)
    assert watch.rate_hz(now=100.0) == 0.0


def test_max_gap_over_last_60_seconds_and_ages_out() -> None:
    watch = ScanWatch(started_at=0.0, startup_grace_s=30.0, silence_timeout_s=5.0)
    assert watch.max_gap_s(now=1.0) is None
    for t in (1.0, 1.1, 3.1, 3.2):
        watch.on_scan(t)
    assert watch.max_gap_s(now=3.3) == pytest.approx(2.0)
    assert watch.max_gap_s(now=62.0) == pytest.approx(58.8)  # open gap since the last scan dominates
    watch.on_scan(62.1)
    assert watch.max_gap_s(now=62.2) == pytest.approx(58.9)
    watch.on_scan(62.2)
    assert watch.max_gap_s(now=124.0) == pytest.approx(61.8)


def test_open_gap_counts_before_the_next_scan_arrives() -> None:
    watch = ScanWatch(started_at=0.0, startup_grace_s=30.0, silence_timeout_s=5.0)
    watch.on_scan(1.0)
    watch.on_scan(1.1)
    assert watch.max_gap_s(now=4.1) == pytest.approx(3.0)


def test_export_metrics_sets_counter_rate_and_gap() -> None:
    watch = ScanWatch(started_at=0.0, startup_grace_s=30.0, silence_timeout_s=5.0)
    before = sample("lidar_scans_total") or 0.0
    for t in (1.0, 1.1, 1.2, 1.3):
        watch.on_scan(t)
    watch.export_metrics(now=1.3)
    assert sample("lidar_scans_total") == before + 4
    assert sample("lidar_scan_rate_hz") == pytest.approx(10.0)
    assert sample("lidar_scan_gap_seconds_max") == pytest.approx(0.1)


def test_record_restart_counts_by_reason_and_series_exist_from_import() -> None:
    assert sample("lidar_driver_restarts_total", {"reason": "startup"}) is not None
    assert sample("lidar_driver_restarts_total", {"reason": "gap"}) is not None
    before = sample("lidar_driver_restarts_total", {"reason": "gap"})
    ScanWatch.record_restart("gap")
    assert sample("lidar_driver_restarts_total", {"reason": "gap"}) == before + 1


def test_supervisor_wires_metrics_server_from_env_port() -> None:
    source = (RPLIDAR_DIR / "scan_supervisor.py").read_text()
    head = source.split("LAUNCH_FILE", 1)[0]
    assert "from ros2_metrics import resolve_metrics_port, start_metrics_server" in head
    assert "start_metrics_server(resolve_metrics_port(None)," in source
    assert "ScanWatch.record_restart(" in source
