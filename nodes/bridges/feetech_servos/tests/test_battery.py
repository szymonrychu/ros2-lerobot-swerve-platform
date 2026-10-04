"""Unit tests for battery voltage monitoring (round-robin reads, freshness, median, message fields)."""

import math

import pytest

from feetech_servos.battery import (
    BATTERY_HEALTH_UNKNOWN,
    BATTERY_STATUS_UNKNOWN,
    BATTERY_TECHNOLOGY_UNKNOWN,
    BatteryMonitor,
    battery_fields,
    raw_to_volts,
)


def test_raw_to_volts_converts_tenths() -> None:
    """Raw register unit is 0.1 V."""
    assert raw_to_volts(114) == pytest.approx(11.4)


@pytest.mark.parametrize("raw", [None, 0, -1])
def test_raw_to_volts_rejects_implausible(raw: int | None) -> None:
    """None, 0 and negative raw values are not voltages."""
    assert raw_to_volts(raw) is None


def test_monitor_reads_one_servo_per_interval_round_robin() -> None:
    """Each due step reads exactly one servo, cycling through all ids."""
    mon = BatteryMonitor([1, 2, 3], interval_s=1.0, stale_s=30.0)
    read: list[int] = []

    def read_raw(sid: int) -> int | None:
        read.append(sid)
        return 110

    mon.step(0.0, read_raw)
    mon.step(0.5, read_raw)  # not due yet
    mon.step(1.0, read_raw)
    mon.step(2.0, read_raw)
    mon.step(3.0, read_raw)
    assert read == [1, 2, 3, 1]


def test_monitor_disabled_with_zero_interval() -> None:
    """interval 0 never reads and never returns a voltage."""
    mon = BatteryMonitor([1], interval_s=0.0, stale_s=30.0)
    assert mon.step(10.0, lambda sid: pytest.fail("must not read")) is None


def test_monitor_returns_median_of_fresh_readings() -> None:
    """Voltage is the median of the latest readings of all servos."""
    mon = BatteryMonitor([1, 2, 3], interval_s=1.0, stale_s=30.0)
    values = {1: 110, 2: 114, 3: 200}
    results = [mon.step(float(t), lambda sid: values[sid]) for t in range(3)]
    assert results[0] == pytest.approx(11.0)
    assert results[1] == pytest.approx(11.2)
    assert results[2] == pytest.approx(11.4)


def test_monitor_ignores_failed_and_zero_reads() -> None:
    """Failed/zero reads are dropped; with no valid reading nothing is returned."""
    mon = BatteryMonitor([1, 2], interval_s=1.0, stale_s=30.0)
    assert mon.step(0.0, lambda sid: None) is None
    assert mon.step(1.0, lambda sid: 0) is None
    assert mon.step(2.0, lambda sid: 111) == pytest.approx(11.1)


def test_monitor_failed_read_keeps_previous_reading() -> None:
    """A failed read does not erase the servo's earlier fresh reading."""
    mon = BatteryMonitor([1], interval_s=1.0, stale_s=30.0)
    assert mon.step(0.0, lambda sid: 111) == pytest.approx(11.1)
    assert mon.step(1.0, lambda sid: None) == pytest.approx(11.1)


def test_monitor_drops_stale_readings() -> None:
    """Readings older than stale_s are dropped; when all are stale nothing is returned."""
    mon = BatteryMonitor([1], interval_s=1.0, stale_s=5.0)
    assert mon.step(0.0, lambda sid: 111) is not None
    assert mon.step(6.0, lambda sid: None) is None


def test_monitor_stale_servo_excluded_from_median() -> None:
    """A stale servo no longer influences the median."""
    mon = BatteryMonitor([1, 2], interval_s=1.0, stale_s=1.5)
    mon.step(0.0, lambda sid: 100)  # servo 1 at t=0
    mon.step(1.0, lambda sid: 120)  # servo 2 at t=1
    assert mon.step(2.0, lambda sid: None) == pytest.approx(12.0)  # servo 1 (t=0) stale, servo 2 fresh


def test_battery_fields_match_spec() -> None:
    """Field values follow sensor_msgs/BatteryState with unknowns as NaN / UNKNOWN."""
    f = battery_fields(11.4, cells=3)
    assert f["voltage"] == pytest.approx(11.4)
    assert f["present"] is True
    assert len(f["cell_voltage"]) == 3 and all(math.isnan(v) for v in f["cell_voltage"])
    for key in ("temperature", "current", "charge", "capacity", "design_capacity", "percentage"):
        assert math.isnan(f[key])
    assert f["power_supply_status"] == BATTERY_STATUS_UNKNOWN == 0
    assert f["power_supply_health"] == BATTERY_HEALTH_UNKNOWN == 0
    assert f["power_supply_technology"] == BATTERY_TECHNOLOGY_UNKNOWN == 0
