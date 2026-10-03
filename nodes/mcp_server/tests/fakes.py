"""Test doubles: a simulated follower arm backend with a fake clock."""

from collections.abc import Callable

from mcp_server.arm import JointSample
from mcp_server.staleness import Stamped

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


class FakeArmBackend:
    """Simulated follower: measured positions jump to each command when `follow` is set; time advances on sleep."""

    def __init__(self, positions: dict[str, float] | None = None) -> None:
        self.t = 100.0
        self.positions: dict[str, float] = dict(positions or {j: 0.0 for j in JOINTS})
        self.efforts: dict[str, float] = {j: 0.0 for j in JOINTS}
        self.commands: list[dict[str, float]] = []
        self.releases = 0
        self.source: str | None = "autonomy"
        self.follow = True
        self.stale_after: float | None = None
        self.no_samples = False
        self.on_sleep: Callable[[FakeArmBackend], None] | None = None

    def joint_sample(self) -> JointSample | None:
        if self.no_samples:
            return None
        stamp = self.t if self.stale_after is None else min(self.t, self.stale_after)
        return JointSample(positions=dict(self.positions), efforts=dict(self.efforts), stamp=stamp)

    def publish_command(self, positions: dict[str, float]) -> None:
        self.commands.append(dict(positions))
        if self.follow and (self.stale_after is None or self.t < self.stale_after):
            self.positions.update(positions)

    def publish_release(self) -> None:
        self.releases += 1

    def active_source(self) -> Stamped[str] | None:
        return None if self.source is None else Stamped(value=self.source, stamp=self.t)

    def now(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        self.t += seconds
        if self.on_sleep is not None:
            self.on_sleep(self)
