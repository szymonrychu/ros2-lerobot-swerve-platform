"""Tests for the map guard state machine: transitions, pause toggle tracking, reset/backup, dry run, cooldown."""

from collections.abc import Sequence
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import pytest
from prometheus_client import REGISTRY

from map_guard.agent import EVENT_DISTURBED, EVENT_RESET, EVENT_RESUMED
from map_guard.config import MapGuardConfig
from map_guard.detectors import REASON_CARRIED, REASON_LIFTED, REASON_MISMATCH, DisturbanceReport
from map_guard.guard import MapGuard, State

TICK = 0.1
HEALTHY = (0.9, 5000)
BAD = (0.1, 5000)
FIXED_UTC = datetime(2026, 10, 10, 12, 0, 0, tzinfo=UTC)


class FakeSlam:
    """slam_toolbox stand-in: the pause service toggles, the parameter reports the real state."""

    def __init__(self, paused: bool = False, toggle_ok: bool = True, toggle_works: bool = True) -> None:
        self.paused = paused
        self.toggle_ok = toggle_ok
        self.toggle_works = toggle_works
        self.serialize_ok = True
        self.reset_ok = True
        self.calls: list[Any] = []

    def query_paused(self) -> bool | None:
        self.calls.append("query")
        return self.paused

    def toggle_pause(self) -> bool:
        self.calls.append("toggle")
        if self.toggle_ok and self.toggle_works:
            self.paused = not self.paused
        return self.toggle_ok

    def serialize_map(self, filename: str) -> bool:
        self.calls.append(("serialize", filename))
        if self.serialize_ok:
            Path(filename + ".posegraph").write_text("g")
            Path(filename + ".data").write_text("d")
        return self.serialize_ok

    def reset(self) -> bool:
        self.calls.append("reset")
        return self.reset_ok


class FakeNotifier:
    def __init__(self) -> None:
        self.events: list[tuple[str, str, float | None, dict]] = []

    def notify(self, now: float, kind: str, reason: str, ratio: float | None, data: dict | None = None) -> None:
        self.events.append((kind, reason, ratio, data or {}))


class Harness:
    def __init__(self, tmp_path: Path, slam: FakeSlam | None = None, **overrides: Any) -> None:
        config = MapGuardConfig(**{"dry_run": False, "backup_dir": str(tmp_path), **overrides})
        self.slam = slam or FakeSlam()
        self.notifier = FakeNotifier()
        self.resets_published = 0
        self.logs: list[str] = []
        self.guard = MapGuard(
            config,
            self.slam,
            self.notifier,
            self.publish_reset,
            self.logs.append,
            utc_now=lambda: FIXED_UTC,
        )
        self.now = 0.0

    def publish_reset(self) -> None:
        self.resets_published += 1

    def run(self, seconds: float, reasons: Sequence[str] = (), sample: tuple[float, int] | None = None) -> None:
        """Tick at 10 Hz; an agreement sample every 0.5 s when given."""
        for _ in range(int(round(seconds / TICK))):
            report = DisturbanceReport(frozenset(reasons), None, None, None)
            samples = [sample] if sample is not None and int(round(self.now / TICK)) % 5 == 0 else []
            self.guard.tick(self.now, report, samples)
            self.now = round(self.now + TICK, 6)


def counter(name: str, labels: dict[str, str] | None = None) -> float:
    return REGISTRY.get_sample_value(name, labels or {}) or 0.0


def state_gauge(state: State) -> float:
    return counter("map_guard_state", {"state": state.value})


def test_starts_ok(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    h.run(1.0, sample=HEALTHY)
    assert h.guard.state is State.OK
    assert h.slam.calls == []
    assert state_gauge(State.OK) == 1.0


def test_lifted_pauses_settles_and_resumes(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    pauses, resumes = counter("map_guard_pauses_total"), counter("map_guard_resumes_total")
    triggers = counter("map_guard_triggers_total", {"reason": "lifted"})
    h.run(1.0, sample=HEALTHY)
    h.run(0.1, reasons=[REASON_LIFTED], sample=HEALTHY)
    assert h.guard.state is State.DISTURBED
    assert h.slam.paused is True and h.guard.paused is True
    assert h.slam.calls == ["query", "toggle", "query"]
    assert counter("map_guard_pauses_total") == pauses + 1
    assert counter("map_guard_triggers_total", {"reason": "lifted"}) == triggers + 1
    assert h.notifier.events[0][:2] == (EVENT_DISTURBED, REASON_LIFTED)
    assert state_gauge(State.DISTURBED) == 1.0 and state_gauge(State.OK) == 0.0

    h.run(2.0, reasons=[REASON_LIFTED], sample=HEALTHY)  # still lifted: stays disturbed, no extra trigger
    assert h.guard.state is State.DISTURBED
    assert counter("map_guard_triggers_total", {"reason": "lifted"}) == triggers + 1

    h.run(0.1, sample=HEALTHY)
    assert h.guard.state is State.SETTLING
    h.run(2.8, sample=HEALTHY)
    assert h.guard.state is State.SETTLING
    h.run(0.4, sample=HEALTHY)
    assert h.guard.state is State.EVALUATING
    h.run(4.1, sample=HEALTHY)
    assert h.guard.state in (State.RESUMED, State.OK)
    h.run(0.2, sample=HEALTHY)
    assert h.guard.state is State.OK
    assert h.slam.paused is False
    assert counter("map_guard_resumes_total") == resumes + 1
    assert h.notifier.events[-1][:2] == (EVENT_RESUMED, REASON_LIFTED)
    assert "reset" not in h.slam.calls
    assert h.resets_published == 0
    assert any("disturbed" in line and "lifted" in line for line in h.logs)


def test_new_trigger_while_settling_goes_back_to_disturbed(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    h.run(0.1, reasons=[REASON_LIFTED])
    h.run(1.0)
    assert h.guard.state is State.SETTLING
    h.run(0.1, reasons=[REASON_CARRIED])
    assert h.guard.state is State.DISTURBED
    assert h.slam.calls.count("toggle") == 1  # already paused: no second toggle


def test_trigger_while_evaluating_goes_back_to_disturbed(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    h.run(0.1, reasons=[REASON_LIFTED])
    h.run(3.2, sample=HEALTHY)
    assert h.guard.state is State.EVALUATING
    h.run(0.1, reasons=[REASON_CARRIED], sample=HEALTHY)
    assert h.guard.state is State.DISTURBED


def test_low_agreement_after_settling_backs_up_and_resets(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    resets = counter("map_guard_resets_total", {"dry_run": "false"})
    h.run(0.1, reasons=[REASON_CARRIED])
    h.run(3.2, sample=BAD)
    assert h.guard.state is State.EVALUATING
    h.run(4.2, sample=BAD)
    h.run(0.2, sample=BAD)
    assert h.guard.state is State.OK
    expected = str(tmp_path / "map_guard_backup_20261010T120000Z")
    assert ("serialize", expected) in h.slam.calls
    assert h.slam.calls.index(("serialize", expected)) < h.slam.calls.index("reset")
    assert h.slam.paused is False
    assert h.resets_published == 1
    assert counter("map_guard_resets_total", {"dry_run": "false"}) == resets + 1
    kind, reason, ratio, data = h.notifier.events[-1]
    assert (kind, reason) == (EVENT_RESET, REASON_CARRIED)
    assert ratio == pytest.approx(0.1)
    assert data["backup"] == expected


def test_reset_prunes_old_backups(tmp_path: Path) -> None:
    for i in range(6):
        stem = f"map_guard_backup_2026010{i}T000000Z"
        (tmp_path / f"{stem}.posegraph").write_text("g")
        (tmp_path / f"{stem}.data").write_text("d")
    h = Harness(tmp_path, keep_backups=3)
    h.run(0.1, reasons=[REASON_CARRIED])
    h.run(8.0, sample=BAD)
    stems = sorted({p.stem for p in tmp_path.iterdir()})
    assert stems == [
        "map_guard_backup_20260104T000000Z",
        "map_guard_backup_20260105T000000Z",
        "map_guard_backup_20261010T120000Z",
    ]


def test_failed_backup_skips_the_reset(tmp_path: Path) -> None:
    slam = FakeSlam()
    slam.serialize_ok = False
    h = Harness(tmp_path, slam=slam)
    h.run(0.1, reasons=[REASON_CARRIED])
    h.run(8.0, sample=BAD)
    assert h.guard.state is State.OK
    assert "reset" not in slam.calls
    assert slam.paused is False
    assert h.resets_published == 0
    assert any("serialize" in line for line in h.logs)


def test_failed_reset_does_not_publish(tmp_path: Path) -> None:
    slam = FakeSlam()
    slam.reset_ok = False
    h = Harness(tmp_path, slam=slam)
    h.run(0.1, reasons=[REASON_CARRIED])
    h.run(8.0, sample=BAD)
    assert "reset" in slam.calls
    assert h.resets_published == 0
    assert slam.paused is False


def test_no_agreement_samples_during_evaluation_resumes(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    h.run(0.1, reasons=[REASON_LIFTED])
    h.run(8.0)
    assert h.guard.state is State.OK
    assert "reset" not in h.slam.calls
    assert h.notifier.events[-1][0] == EVENT_RESUMED


def test_mismatch_alone_pauses_and_evaluates_without_settling(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    triggers = counter("map_guard_triggers_total", {"reason": "mismatch"})
    h.run(5.0, sample=HEALTHY)
    h.run(7.5, sample=BAD)
    assert h.guard.state is State.EVALUATING
    assert h.slam.paused is True
    assert counter("map_guard_triggers_total", {"reason": "mismatch"}) == triggers + 1
    assert h.notifier.events[0][:2] == (EVENT_DISTURBED, REASON_MISMATCH)
    h.run(4.5, sample=BAD)
    assert "reset" in h.slam.calls
    assert h.notifier.events[-1][:2] == (EVENT_RESET, REASON_MISMATCH)


def test_cooldown_blocks_mismatch_retrigger(tmp_path: Path) -> None:
    h = Harness(tmp_path, cooldown_s=30.0)
    h.run(5.0, sample=HEALTHY)
    h.run(12.5, sample=BAD)  # mismatch -> evaluate -> reset
    assert h.slam.calls.count("reset") == 1
    h.run(4.0, sample=HEALTHY)  # the new map is healthy (re-arms the monitor)
    h.run(10.0, sample=BAD)  # still inside the 30 s cooldown
    assert h.guard.state is State.OK
    assert h.slam.calls.count("reset") == 1
    h.run(20.0, sample=BAD)  # cooldown (until ~45 s) over: the mismatch fires, is evaluated and resets again
    assert h.slam.calls.count("reset") == 2


def test_cooldown_does_not_block_physical_triggers(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    h.run(0.1, reasons=[REASON_CARRIED])
    h.run(8.0, sample=BAD)
    assert h.slam.calls.count("reset") == 1
    h.run(0.1, reasons=[REASON_LIFTED])
    assert h.guard.state is State.DISTURBED


def test_disabled_reasons_are_ignored(tmp_path: Path) -> None:
    h = Harness(tmp_path, reasons=["mismatch"])
    h.run(3.0, reasons=[REASON_LIFTED, REASON_CARRIED])
    assert h.guard.state is State.OK
    assert h.slam.calls == []


def test_disabled_mismatch_is_ignored(tmp_path: Path) -> None:
    h = Harness(tmp_path, reasons=["lifted"])
    h.run(5.0, sample=HEALTHY)
    h.run(15.0, sample=BAD)
    assert h.guard.state is State.OK


# ---------------------------------------------------------------------------
# Pause toggle tracking
# ---------------------------------------------------------------------------


def test_already_paused_is_not_toggled(tmp_path: Path) -> None:
    h = Harness(tmp_path, slam=FakeSlam(paused=True))
    h.run(0.1, reasons=[REASON_LIFTED])
    assert "toggle" not in h.slam.calls
    assert h.guard.paused is True


def test_failed_toggle_keeps_the_tracked_state(tmp_path: Path) -> None:
    h = Harness(tmp_path, slam=FakeSlam(toggle_ok=False))
    h.run(0.1, reasons=[REASON_LIFTED])
    assert h.guard.state is State.DISTURBED
    assert h.guard.paused is False
    assert any("pause" in line for line in h.logs)


def test_toggle_that_did_not_take_effect_is_detected(tmp_path: Path) -> None:
    h = Harness(tmp_path, slam=FakeSlam(toggle_works=False))
    h.run(0.1, reasons=[REASON_LIFTED])
    assert h.guard.paused is False
    assert any("still" in line for line in h.logs)


def test_unknown_parameter_state_falls_back_to_tracking(tmp_path: Path) -> None:
    class NoParamSlam(FakeSlam):
        def query_paused(self) -> bool | None:
            self.calls.append("query")
            return None

    slam = NoParamSlam()
    h = Harness(tmp_path, slam=slam)
    h.run(0.1, reasons=[REASON_LIFTED])
    assert slam.calls.count("toggle") == 1
    assert h.guard.paused is True
    h.run(0.1, reasons=[REASON_CARRIED])
    h.run(1.0)
    h.run(0.1, reasons=[REASON_LIFTED])
    assert slam.calls.count("toggle") == 1  # tracked as paused: no second toggle
    h.run(8.0, sample=HEALTHY)
    assert slam.calls.count("toggle") == 2
    assert h.guard.paused is False


# ---------------------------------------------------------------------------
# Dry run
# ---------------------------------------------------------------------------


def test_dry_run_never_calls_services_or_publishes(tmp_path: Path) -> None:
    slam = FakeSlam()
    h = Harness(tmp_path, slam=slam, dry_run=True)
    resets = counter("map_guard_resets_total", {"dry_run": "true"})
    pauses = counter("map_guard_pauses_total")
    h.run(0.1, reasons=[REASON_CARRIED])
    assert h.guard.state is State.DISTURBED
    h.run(8.0, sample=BAD)
    assert h.guard.state is State.OK
    assert slam.calls == []
    assert h.resets_published == 0
    assert list(tmp_path.iterdir()) == []
    assert counter("map_guard_resets_total", {"dry_run": "true"}) == resets + 1
    assert counter("map_guard_pauses_total") == pauses + 1
    assert [e[0] for e in h.notifier.events] == [EVENT_DISTURBED, EVENT_RESET]
    assert any("dry run" in line and "carried" in line for line in h.logs)
    assert any("ratio" in line for line in h.logs)


def test_dry_run_gauge(tmp_path: Path) -> None:
    Harness(tmp_path, dry_run=True)
    assert counter("map_guard_dry_run") == 1.0
    Harness(tmp_path, dry_run=False)
    assert counter("map_guard_dry_run") == 0.0


def test_report_numbers_reach_the_gauges(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    h.guard.tick(0.0, DisturbanceReport(frozenset(), 3.5, 0.04, 0.0), [(0.77, 5000)])
    assert counter("map_guard_tilt_deg") == pytest.approx(3.5)
    assert counter("map_guard_unexplained_speed_mps") == pytest.approx(0.04)
