"""Battery pack voltage from Feetech servos: round-robin reads, freshness, median, BatteryState fields."""

import math
import statistics
from collections.abc import Callable
from typing import Any

# Register present_voltage unit: 0.1 V per LSB.
VOLTS_PER_RAW_UNIT = 0.1

# sensor_msgs/BatteryState constants (value 0 = UNKNOWN for status, health and technology).
BATTERY_STATUS_UNKNOWN = 0
BATTERY_HEALTH_UNKNOWN = 0
BATTERY_TECHNOLOGY_UNKNOWN = 0


def raw_to_volts(raw: int | None) -> float | None:
    """Convert a raw present_voltage register value to volts.

    Args:
        raw: Raw register value (int, 0.1 V units) or None when the read failed.

    Returns:
        float | None: Voltage in volts, or None when raw is None or not positive (implausible).
    """
    if raw is None or raw <= 0:
        return None
    return raw * VOLTS_PER_RAW_UNIT


class BatteryMonitor:
    """Read the pack voltage from one servo per interval (round-robin) and aggregate fresh readings.

    Attributes:
        servo_ids: Servo IDs to read in turn.
        interval_s: Seconds between reads (0 disables).
        stale_s: Readings older than this many seconds are dropped.
    """

    def __init__(self, servo_ids: list[int], interval_s: float, stale_s: float) -> None:
        """Create the monitor.

        Args:
            servo_ids: Servo IDs to read in turn.
            interval_s: Seconds between reads; 0 disables.
            stale_s: Maximum age of a per-servo reading in seconds.
        """
        self.servo_ids = servo_ids
        self.interval_s = interval_s
        self.stale_s = stale_s
        self._index = 0
        self._next_read: float | None = None
        self._readings: dict[int, tuple[float, float]] = {}  # servo_id -> (volts, timestamp)

    def voltage(self, now: float) -> float | None:
        """Median voltage of readings not older than stale_s.

        Args:
            now: Current monotonic time in seconds (float).

        Returns:
            float | None: Median voltage in volts, or None when no reading is fresh.
        """
        fresh = [v for v, t in self._readings.values() if now - t <= self.stale_s]
        return statistics.median(fresh) if fresh else None

    def step(self, now: float, read_raw: Callable[[int], int | None]) -> float | None:
        """Read at most one servo when an interval has elapsed.

        Args:
            now: Current monotonic time in seconds (float).
            read_raw: Reads raw present_voltage of one servo ID; None on failure.

        Returns:
            float | None: Median pack voltage in volts when a read was due and a fresh reading exists,
                otherwise None (nothing to publish).
        """
        if self.interval_s <= 0 or not self.servo_ids:
            return None
        if self._next_read is not None and now < self._next_read:
            return None
        self._next_read = now + self.interval_s
        sid = self.servo_ids[self._index % len(self.servo_ids)]
        self._index += 1
        volts = raw_to_volts(read_raw(sid))
        if volts is not None:
            self._readings[sid] = (volts, now)
        return self.voltage(now)


def battery_fields(voltage: float, cells: int) -> dict[str, Any]:
    """Field values of a sensor_msgs/BatteryState for a pack voltage; unknown quantities are NaN.

    Args:
        voltage: Pack voltage in volts (float).
        cells: Number of cells in series (int); individual cell voltages are unknown (NaN).

    Returns:
        dict[str, Any]: BatteryState field name -> value (excluding header).
    """
    nan = math.nan
    return {
        "voltage": voltage,
        "temperature": nan,
        "current": nan,
        "charge": nan,
        "capacity": nan,
        "design_capacity": nan,
        "percentage": nan,
        "power_supply_status": BATTERY_STATUS_UNKNOWN,
        "power_supply_health": BATTERY_HEALTH_UNKNOWN,
        "power_supply_technology": BATTERY_TECHNOLOGY_UNKNOWN,
        "present": True,
        "cell_voltage": [nan] * cells,
    }
