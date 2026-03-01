"""Base interface for filter algorithms (per-joint state, update, predict)."""

from abc import ABC, abstractmethod
from typing import Any


class FilterAlgorithm(ABC):
    """Interface for a joint filter: create state per joint, update from measurement, output filtered position."""

    @abstractmethod
    def create_state(self, joint_name: str, initial_position_rad: float, now_s: float) -> Any:
        """Create initial state for a joint.

        Args:
            joint_name: Joint identifier (may be used for algorithm-specific init).
            initial_position_rad: Initial position in radians.
            now_s: Current time (monotonic) in seconds.

        Returns:
            Any: Opaque state object for this joint.
        """
        ...

    @abstractmethod
    def update(
        self,
        state: Any,
        joint_name: str,
        measurement_rad: float,
        now_s: float,
    ) -> None:
        """Update state with a new measurement (predict + update step).

        Args:
            state: Per-joint state from create_state or previous update.
            joint_name: Joint identifier (may be used for algorithm-specific logic).
            measurement_rad: Measured position in radians.
            now_s: Current time (monotonic) in seconds.
        """
        ...

    @abstractmethod
    def predict(
        self,
        state: Any,
        joint_name: str,
        now_s: float,
        last_measurement_time_s: float | None = None,
    ) -> float:
        """Run predict step and return filtered position in radians.

        Args:
            state: Per-joint state.
            joint_name: Joint identifier (may be used for algorithm-specific logic).
            now_s: Current time (monotonic) in seconds.
            last_measurement_time_s: Time of last measurement; if too old, implementation may hold position.

        Returns:
            float: Filtered position in radians.
        """
        ...
