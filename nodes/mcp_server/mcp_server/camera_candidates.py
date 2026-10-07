"""In-memory candidate point sets (mark_candidate_points / resolve_candidate)."""

import math
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Any

MAX_SETS = 10
MOVE_BASE_M = 0.02
MOVE_BASE_RAD = 0.03
MOVE_ARM_RAD = 0.02


@dataclass(frozen=True)
class RobotSnapshot:
    """Where the robot and arm were (map pose, arm joints) when a set was created."""

    pose: tuple[float, float, float] | None
    joints: dict[str, float] | None


@dataclass(frozen=True)
class CandidateSet:
    """A numbered set of candidate pixels with their floor points."""

    set_id: str
    camera: str
    created: float
    snapshot: RobotSnapshot
    points: list[dict[str, Any]] = field(default_factory=list)


def angle_diff(a: float, b: float) -> float:
    """Smallest absolute difference between two angles.

    Args:
        a (float): Angle in rad.
        b (float): Angle in rad.

    Returns:
        float: Difference in [0, pi].
    """
    d = (a - b + math.pi) % (2 * math.pi) - math.pi
    return abs(d)


def base_moved(before: tuple[float, float, float] | None, now: tuple[float, float, float] | None) -> bool | None:
    """Whether the base moved between two map poses.

    Args:
        before (tuple[float, float, float] | None): (x, y, yaw) at creation.
        now (tuple[float, float, float] | None): (x, y, yaw) now.

    Returns:
        bool | None: None when either pose is unknown.
    """
    if before is None or now is None:
        return None
    translated = math.hypot(now[0] - before[0], now[1] - before[1]) > MOVE_BASE_M
    return translated or angle_diff(now[2], before[2]) > MOVE_BASE_RAD


def arm_moved(before: dict[str, float] | None, now: dict[str, float] | None) -> bool | None:
    """Whether any arm joint moved between two snapshots.

    Args:
        before (dict[str, float] | None): Joints at creation.
        now (dict[str, float] | None): Joints now.

    Returns:
        bool | None: None when either snapshot is unknown.
    """
    if before is None or now is None:
        return None
    return any(abs(now.get(j, v) - v) > MOVE_ARM_RAD for j, v in before.items())


class CandidateStore:
    """Thread-safe store of the last MAX_SETS candidate sets."""

    def __init__(self) -> None:
        """Create an empty store."""
        self.sets: OrderedDict[str, CandidateSet] = OrderedDict()
        self.counter = 0
        self.lock = threading.Lock()

    def add(self, camera: str, snapshot: RobotSnapshot, points: list[dict[str, Any]]) -> CandidateSet:
        """Store a new set, evicting the oldest beyond MAX_SETS.

        Args:
            camera (str): Camera name.
            snapshot (RobotSnapshot): Robot state at creation.
            points (list[dict[str, Any]]): Numbered points.

        Returns:
            CandidateSet: The stored set.
        """
        with self.lock:
            self.counter += 1
            item = CandidateSet(f"c{self.counter}", camera, time.monotonic(), snapshot, points)
            self.sets[item.set_id] = item
            while len(self.sets) > MAX_SETS:
                self.sets.popitem(last=False)
            return item

    def get(self, set_id: str) -> CandidateSet:
        """Look up a set.

        Args:
            set_id (str): Set id from mark_candidate_points.

        Returns:
            CandidateSet: The set.

        Raises:
            KeyError: When unknown or evicted.
        """
        with self.lock:
            return self.sets[set_id]
