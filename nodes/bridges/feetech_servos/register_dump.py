"""Incremental full-register dump: one servo per control cycle so the loop never blocks on 40+ reads x N servos."""

from collections.abc import Callable


class RegisterDumpScheduler:
    """Spread a full register dump of all joints over consecutive control cycles.

    Attributes:
        joints: (joint_name, servo_id) pairs to dump, in order.
        interval_s: Seconds between dump starts; 0 disables dumping.
    """

    def __init__(self, joints: list[tuple[str, int]], interval_s: float) -> None:
        """Create the scheduler.

        Args:
            joints: (joint_name, servo_id) pairs to dump.
            interval_s: Seconds between dump starts; 0 disables dumping.
        """
        self.joints = joints
        self.interval_s = interval_s
        self._next_start: float | None = None
        self._index: int | None = None
        self._payload: dict[str, dict[str, int]] = {}

    def step(self, now: float, read_all: Callable[[int], dict[str, int]]) -> dict[str, dict[str, int]] | None:
        """Advance the dump by at most one servo.

        Args:
            now: Current monotonic time in seconds.
            read_all: Reads all registers of one servo ID.

        Returns:
            dict[str, dict[str, int]] | None: Complete payload (joint_name -> registers) when the last servo
                of a dump was read this step, otherwise None.
        """
        if self.interval_s <= 0 or not self.joints:
            return None
        if self._index is None:
            if self._next_start is not None and now < self._next_start:
                return None
            self._index = 0
            self._payload = {}
            self._next_start = now + self.interval_s
        name, sid = self.joints[self._index]
        self._payload[name] = read_all(sid)
        self._index += 1
        if self._index < len(self.joints):
            return None
        self._index = None
        return self._payload
