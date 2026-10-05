"""Tests for shared/ros2_common/battery.py: BatteryConfig validation and BatteryGuard hysteresis (pure logic)."""

import math
import threading

import pytest
from pydantic import ValidationError

from shared.ros2_common.battery import BatteryConfig, BatteryGuard

CELLS = 3
CUTOFF_CELL_V = 2.8
RESUME_CELL_V = 2.9
STALE_S = 5.0
LOW_V = 8.21  # 2.74 V/cell
THREAD_WRITES = 2000
THREADS = 4


class FakeClock:
    """Manually advanced monotonic clock."""

    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


def make_guard(clock: FakeClock | None = None) -> BatteryGuard:
    """Build a guard with the default thresholds.

    Args:
        clock: Optional fake clock.

    Returns:
        BatteryGuard: Guard with 3 cells, 2.8/2.9 V per cell, 5 s staleness.
    """
    return BatteryGuard(CELLS, CUTOFF_CELL_V, RESUME_CELL_V, STALE_S, clock=clock or FakeClock())


def test_battery_config_defaults() -> None:
    """Defaults match the Ansible battery block."""
    cfg = BatteryConfig()
    assert (cfg.topic, cfg.cells, cfg.cutoff_cell_v, cfg.resume_cell_v, cfg.stale_s) == (
        "/battery_state",
        3,
        2.8,
        2.9,
        5.0,
    )


def test_battery_config_rejects_resume_below_cutoff() -> None:
    """resume_cell_v must be >= cutoff_cell_v."""
    with pytest.raises(ValidationError):
        BatteryConfig(cutoff_cell_v=3.0, resume_cell_v=2.9)


def test_battery_config_allows_equal_thresholds() -> None:
    """Equal thresholds are valid."""
    assert BatteryConfig(cutoff_cell_v=2.8, resume_cell_v=2.8).resume_cell_v == 2.8


@pytest.mark.parametrize("cells", [0, -1])
def test_battery_config_rejects_bad_cells(cells: int) -> None:
    """cells must be >= 1."""
    with pytest.raises(ValidationError):
        BatteryConfig(cells=cells)


def test_guard_unknown_without_reading() -> None:
    """No reading: unknown, not blocked."""
    guard = make_guard()
    assert guard.is_cutoff() is False
    assert guard.state()["voltage"] is None


def test_guard_enters_cutoff_below_threshold() -> None:
    """Cut-off starts strictly below cells * cutoff_cell_v (8.4 V)."""
    guard = make_guard()
    guard.update(8.41)
    assert guard.is_cutoff() is False
    guard.update(8.39)
    assert guard.is_cutoff() is True


def test_guard_hysteresis_leaves_only_above_resume() -> None:
    """Cut-off is released only strictly above cells * resume_cell_v (8.7 V)."""
    guard = make_guard()
    guard.update(8.0)
    guard.update(8.6)
    assert guard.is_cutoff() is True
    guard.update(8.7)
    assert guard.is_cutoff() is True
    guard.update(8.71)
    assert guard.is_cutoff() is False
    guard.update(8.5)
    assert guard.is_cutoff() is False


def test_guard_stale_reading_is_not_blocked() -> None:
    """A reading older than stale_s counts as unknown."""
    clock = FakeClock()
    guard = make_guard(clock)
    guard.update(8.0)
    assert guard.is_cutoff() is True
    clock.now += STALE_S + 0.1
    assert guard.is_cutoff() is False
    assert guard.state()["stale"] is True


def test_guard_ignores_invalid_voltage() -> None:
    """NaN, inf, zero and negative readings are ignored."""
    guard = make_guard()
    for bad in (math.nan, math.inf, 0.0, -1.0):
        assert guard.update(bad) is False
    assert guard.state()["voltage"] is None
    guard.update(8.0)
    guard.update(math.nan)
    assert guard.state()["voltage"] == 8.0


def test_guard_state_and_rejection_message() -> None:
    """state() snapshot and the motion rejection text."""
    guard = make_guard()
    guard.update(LOW_V)
    state = guard.state()
    assert state["cutoff"] is True
    assert state["cell_voltage"] == pytest.approx(LOW_V / CELLS)
    assert state["cutoff_v"] == pytest.approx(8.4) and state["resume_v"] == pytest.approx(8.7)
    assert guard.rejection_message() == "battery below cut-off: 8.21 V (2.74 V/cell < 2.80 V/cell); motion refused"


def test_guard_is_thread_safe() -> None:
    """Concurrent updates and reads leave a consistent state."""
    guard = make_guard()

    def writer() -> None:
        for i in range(THREAD_WRITES):
            guard.update(8.0 if i % 2 else 11.0)

    threads = [threading.Thread(target=writer) for _ in range(THREADS)]
    for t in threads:
        t.start()
    for _ in range(THREAD_WRITES):
        guard.is_cutoff()
        guard.state()
    for t in threads:
        t.join()
    assert guard.state()["voltage"] in (8.0, 11.0)


def test_guard_from_config() -> None:
    """from_config applies the config thresholds (4 x 2.5 = 10.0 V)."""
    guard = BatteryGuard.from_config(BatteryConfig(cells=4, cutoff_cell_v=2.5, resume_cell_v=2.6))
    guard.update(9.9)
    assert guard.is_cutoff() is True
