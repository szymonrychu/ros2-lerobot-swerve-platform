"""Data age bookkeeping: every cached ROS sample carries the monotonic time it was received."""

import threading
from collections.abc import Callable, Mapping
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


class LazyStamped:
    """A serialized message with its receive time, deserialized at most once and only when `value` is read."""

    def __init__(self, raw: bytes, stamp: float, decode: Callable[[bytes], Any]) -> None:
        """Keep the raw bytes.

        Args:
            raw (bytes): Serialized message.
            stamp (float): Receive time in seconds (monotonic).
            decode (Callable[[bytes], Any]): Deserializer, called at most once.
        """
        self.stamp = stamp
        self.raw: bytes | None = raw
        self.decode = decode
        self.decoded: Any = None
        self.lock = threading.Lock()

    @property
    def value(self) -> Any:
        """The deserialized message (decoded on the first read, then reused)."""
        with self.lock:
            if self.raw is not None:
                self.decoded = self.decode(self.raw)
                self.raw = None
            return self.decoded

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


class MonitorFeed:
    """Hands the latest not yet fed sample of one topic to a monitor hook, once, when flushed from a timer."""

    def __init__(self, hook: Callable[[Any], None]) -> None:
        """Create an empty feed.

        Args:
            hook (Callable[[Any], None]): Called with the decoded latest message.
        """
        self.hook = hook
        self.pending: LazyStamped | None = None
        self.lock = threading.Lock()

    def offer(self, sample: LazyStamped) -> None:
        """Replace the pending sample (older unflushed ones are dropped undecoded).

        Args:
            sample (LazyStamped): Newest received sample.
        """
        with self.lock:
            self.pending = sample

    def flush(self) -> bool:
        """Decode and feed the pending sample, if a new one arrived since the last flush.

        Returns:
            bool: True when the hook was called.
        """
        with self.lock:
            sample, self.pending = self.pending, None
        if sample is None:
            return False
        self.hook(sample.value)
        return True


def sample_ages(latest: Mapping[str, "Stamped[Any] | LazyStamped"], now: float) -> dict[str, float]:
    """Return the age of every cached feed (the mcp_sample_age_seconds metric).

    Args:
        latest (Mapping[str, Stamped[Any] | LazyStamped]): Latest sample per feed key.
        now (float): Current time in the same clock as the stamps.

    Returns:
        dict[str, float]: Age in seconds per feed key.
    """
    return {feed: item.age(now) for feed, item in latest.items()}
