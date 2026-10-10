"""Data age bookkeeping: every cached ROS sample carries the monotonic time it was received."""

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any


def age_s(stamp: float | None, now: float) -> float | None:
    """Return how old a sample is.

    Args:
        stamp (float | None): Receive time in seconds (monotonic), or None when nothing arrived yet.
        now (float): Current time in the same clock.

    Returns:
        float | None: Age in seconds, or None without a sample.
    """
    return None if stamp is None else now - stamp


def is_fresh(stamp: float | None, now: float, max_age_s: float) -> bool:
    """Whether a sample exists and is not older than max_age_s.

    Args:
        stamp (float | None): Receive time in seconds, or None.
        now (float): Current time in the same clock.
        max_age_s (float): Maximum accepted age.

    Returns:
        bool: True when fresh.
    """
    age = age_s(stamp, now)
    return age is not None and age <= max_age_s


@dataclass(frozen=True)
class Stamped[T]:
    """A value with the time it was received."""

    value: T
    stamp: float

    def age(self, now: float) -> float:
        """Return the age of this value.

        Args:
            now (float): Current time.

        Returns:
            float: Age in seconds.
        """
        return now - self.stamp

    def fresh(self, now: float, max_age_s: float) -> bool:
        """Whether this value is not older than max_age_s.

        Args:
            now (float): Current time.
            max_age_s (float): Maximum accepted age.

        Returns:
            bool: True when fresh.
        """
        return is_fresh(self.stamp, now, max_age_s)


def sample_ages(latest: Mapping[str, "Stamped[Any]"], now: float) -> dict[str, float]:
    """Return the age of every cached feed (the mcp_sample_age_seconds metric).

    Args:
        latest (Mapping[str, Stamped[Any]]): Latest sample per feed key.
        now (float): Current time in the same clock as the stamps.

    Returns:
        dict[str, float]: Age in seconds per feed key.
    """
    return {feed: item.age(now) for feed, item in latest.items()}
