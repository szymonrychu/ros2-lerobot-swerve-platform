"""Battery cut-off: pydantic config and a thread-safe hysteresis guard over the pack voltage."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from typing import Any

from pydantic import BaseModel, Field, model_validator

DEFAULT_BATTERY_TOPIC = "/battery_state"
DEFAULT_BATTERY_CELLS = 3
DEFAULT_BATTERY_CUTOFF_CELL_V = 2.8
DEFAULT_BATTERY_RESUME_CELL_V = 2.9
DEFAULT_BATTERY_STALE_S = 5.0


class BatteryConfig(BaseModel):
    """Battery pack monitoring and command cut-off (sensor_msgs/BatteryState topic)."""

    topic: str = DEFAULT_BATTERY_TOPIC
    cells: int = Field(default=DEFAULT_BATTERY_CELLS, ge=1)
    cutoff_cell_v: float = Field(default=DEFAULT_BATTERY_CUTOFF_CELL_V, gt=0)
    resume_cell_v: float = Field(default=DEFAULT_BATTERY_RESUME_CELL_V, gt=0)
    stale_s: float = Field(default=DEFAULT_BATTERY_STALE_S, gt=0)

    @model_validator(mode="after")
    def check_hysteresis(self) -> BatteryConfig:
        """Require resume_cell_v >= cutoff_cell_v.

        Returns:
            BatteryConfig: This config when valid.
        """
        if self.resume_cell_v < self.cutoff_cell_v:
            raise ValueError("battery resume_cell_v must be >= cutoff_cell_v")
        return self


class BatteryGuard:
    """Tracks the pack voltage and whether commands must be rejected (cut-off).

    Enters cut-off when the voltage drops below cells * cutoff_cell_v and leaves it only when the voltage is
    above cells * resume_cell_v (hysteresis). With no reading, or a reading older than stale_s, the battery state
    is unknown and nothing is blocked. Updated from the ROS callback thread, read from other threads.
    """

    def __init__(
        self,
        cells: int,
        cutoff_cell_v: float,
        resume_cell_v: float,
        stale_s: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Create the guard.

        Args:
            cells (int): Number of series cells.
            cutoff_cell_v (float): Per-cell cut-off voltage in volts.
            resume_cell_v (float): Per-cell voltage in volts above which cut-off is released.
            stale_s (float): Readings older than this many seconds count as unknown.
            clock (Callable[[], float]): Monotonic clock in seconds (injectable for tests).
        """
        self.cells = cells
        self.cutoff_cell_v = cutoff_cell_v
        self.resume_cell_v = resume_cell_v
        self.stale_s = stale_s
        self._clock = clock
        self._lock = threading.Lock()
        self._voltage: float | None = None
        self._received_at = 0.0
        self._cutoff = False

    @classmethod
    def from_config(cls, config: BatteryConfig) -> BatteryGuard:
        """Build a guard from the battery config section.

        Args:
            config (BatteryConfig): Validated battery config.

        Returns:
            BatteryGuard: Guard with the config's thresholds.
        """
        return cls(config.cells, config.cutoff_cell_v, config.resume_cell_v, config.stale_s)

    def update(self, voltage: float) -> bool:
        """Record a pack voltage reading and update the cut-off state.

        Args:
            voltage (float): Pack voltage in volts; non-finite or non-positive values are ignored.

        Returns:
            bool: True if the reading was accepted.
        """
        if not math.isfinite(voltage) or voltage <= 0:
            return False
        with self._lock:
            self._voltage = voltage
            self._received_at = self._clock()
            if voltage < self.cells * self.cutoff_cell_v:
                self._cutoff = True
            elif voltage > self.cells * self.resume_cell_v:
                self._cutoff = False
        return True

    def is_stale_locked(self) -> bool:
        """Whether there is no reading or it is older than stale_s (caller holds the lock).

        Returns:
            bool: True when the battery state is unknown.
        """
        return self._voltage is None or self._clock() - self._received_at > self.stale_s

    def is_cutoff(self) -> bool:
        """Whether commands must currently be rejected.

        Returns:
            bool: True only with a fresh reading in the cut-off state.
        """
        with self._lock:
            return self._cutoff and not self.is_stale_locked()

    def state(self) -> dict[str, Any]:
        """Snapshot of the guard for clients.

        Returns:
            dict[str, Any]: voltage (V or None), cells, cell_voltage (V/cell or None), cutoff, stale,
                cutoff_cell_v, resume_cell_v, cutoff_v, resume_v.
        """
        with self._lock:
            stale = self.is_stale_locked()
            voltage = self._voltage
            return {
                "voltage": voltage,
                "cells": self.cells,
                "cell_voltage": voltage / self.cells if voltage is not None else None,
                "cutoff": self._cutoff and not stale,
                "stale": stale,
                "cutoff_cell_v": self.cutoff_cell_v,
                "resume_cell_v": self.resume_cell_v,
                "cutoff_v": self.cells * self.cutoff_cell_v,
                "resume_v": self.cells * self.resume_cell_v,
            }

    def rejection_message(self) -> str:
        """Error text for a refused motion command.

        Returns:
            str: e.g. "battery below cut-off: 8.21 V (2.74 V/cell < 2.80 V/cell); motion refused".
        """
        with self._lock:
            voltage = self._voltage or 0.0
        return (
            f"battery below cut-off: {voltage:.2f} V ({voltage / self.cells:.2f} V/cell "
            f"< {self.cutoff_cell_v:.2f} V/cell); motion refused"
        )
