"""Test doubles: a simulated follower arm backend with a fake clock."""

from collections.abc import Callable

import numpy as np

from mcp_server.arm import JointSample
from mcp_server.models import BasePose, RobotError
from mcp_server.poi_client import PoiStoreDown
from mcp_server.staleness import Stamped
from mcp_server.topdown import TopdownInputs

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
        # Steady-state error per joint while following (e.g. gravity sag: measured = command + sag).
        self.sag: dict[str, float] = {}
        self.stale_after: float | None = None
        self.no_samples = False
        self.on_sleep: Callable[[FakeArmBackend], None] | None = None
        self.on_active_source: Callable[[FakeArmBackend], None] | None = None
        self.on_publish_command: Callable[[FakeArmBackend], None] | None = None
        # Ordered log of everything published on the autonomy topics: ("command", positions) / ("release", None).
        self.events: list[tuple[str, dict[str, float] | None]] = []

    def joint_sample(self) -> JointSample | None:
        if self.no_samples:
            return None
        stamp = self.t if self.stale_after is None else min(self.t, self.stale_after)
        return JointSample(positions=dict(self.positions), efforts=dict(self.efforts), stamp=stamp)

    def publish_command(self, positions: dict[str, float]) -> None:
        if self.on_publish_command is not None:
            hook, self.on_publish_command = self.on_publish_command, None
            hook(self)
        self.commands.append(dict(positions))
        self.events.append(("command", dict(positions)))
        if self.follow and (self.stale_after is None or self.t < self.stale_after):
            self.positions.update({j: v + self.sag.get(j, 0.0) for j, v in positions.items()})

    def publish_release(self) -> None:
        self.releases += 1
        self.events.append(("release", None))

    def active_source(self) -> Stamped[str] | None:
        if self.on_active_source is not None:
            hook, self.on_active_source = self.on_active_source, None
            hook(self)
        return None if self.source is None else Stamped(value=self.source, stamp=self.t)

    def now(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        self.t += seconds
        if self.on_sleep is not None:
            self.on_sleep(self)


class PerceptionFakeMixin:
    """RobotApi perception/memory/POI methods for the tool tests (state is set up by init_perception)."""

    def init_perception(self) -> None:
        self.pose = BasePose(frame="map", x=1.0, y=2.0, yaw=0.5, age_s=0.05)
        self.stops = 0
        self.poi_store_up = True
        self.pois: list[dict] = []
        self.poi_commands: list[tuple[str, dict]] = []
        self.poi_clears: list[str] = []
        self.topdown = TopdownInputs(
            pose=(1.0, 2.0, 0.5),
            scan_points=np.array([[1.0, 0.0], [0.0, 1.0]]),
            plan=np.array([[1.0, 2.0], [2.0, 2.0]]),
            ages={"scan": 0.1, "plan": 2.0},
            missing={"costmap": "no /local_costmap/costmap message within 1.5 s"},
        )

    def robot_pose(self) -> BasePose | None:
        return self.pose

    def stop_count(self) -> int:
        return self.stops

    def event_seq(self) -> int:
        return 0

    def interrupt_since(self, seq: int) -> str | None:
        return None

    def topdown_inputs(self) -> TopdownInputs:
        return self.topdown

    def poi_list(self) -> tuple[list[dict], int]:
        if not self.poi_store_up:
            raise RobotError("poi_store is not running (no /poi/list received)")
        return list(self.pois), len(self.poi_commands)

    def poi_clear(self, created_by: str) -> dict:
        if not self.poi_store_up:
            raise PoiStoreDown("poi_store is not running (no subscriber on /poi/command)")
        self.poi_clears.append(created_by)
        doomed = [p for p in self.pois if p.get("created_by") == created_by]
        self.pois = [p for p in self.pois if p.get("created_by") != created_by]
        return {"ok": True, "message": f"cleared {len(doomed)}", "poi": {"removed": len(doomed)}}

    def poi_request(self, op: str, poi: dict) -> dict:
        if not self.poi_store_up:
            raise RobotError("poi_store is not running (no subscriber on /poi/command)")
        self.poi_commands.append((op, dict(poi)))
        if op == "add":
            stored = {"id": f"id{len(self.pois)}", "status": "open", "polygon": [], "radius_m": 0.2, **poi}
            self.pois.append(stored)
            return {"ok": True, "message": "added", "poi": stored}
        match = next((p for p in self.pois if p["id"] == poi.get("id")), None)
        if match is None:
            raise RobotError(f"unknown poi id {poi.get('id')}")
        if op == "update":
            match.update(poi)
        else:
            self.pois.remove(match)
        return {"ok": True, "message": op, "poi": match}
