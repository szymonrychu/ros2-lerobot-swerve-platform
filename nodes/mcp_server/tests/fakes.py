"""Test doubles: a simulated follower arm backend with a fake clock."""

from collections.abc import Callable

import numpy as np

from mcp_server.arm import JointSample
from mcp_server.frontiers import MapGrid
from mcp_server.models import BasePose, PathPlan, RobotError
from mcp_server.poi_client import PoiStoreDown
from mcp_server.staleness import Stamped
from mcp_server.topdown import TopdownInputs

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
STEER_JOINTS = ("fl_steer", "fr_steer", "rl_steer", "rr_steer")


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
        # Servo register writes (joint, register, value) on the follower set_register topic; with echo_registers the
        # bridge's register dump reports each write at once (read-back), else registers holds what was put there.
        self.register_writes: list[tuple[str, str, int]] = []
        self.registers: dict[tuple[str, str], Stamped[int]] = {}
        self.echo_registers = False
        self.last_sleep = 0.0
        # Autonomy commands and register writes in publish order (events keeps the autonomy topics only).
        self.timeline: list[tuple[str, dict[str, float]]] = []
        # Backend time of each entry of commands (DelayedFollower replays them after a per-joint delay).
        self.command_times: list[float] = []

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
        self.command_times.append(self.t)
        self.events.append(("command", dict(positions)))
        self.timeline.append(("command", dict(positions)))
        if self.follow and (self.stale_after is None or self.t < self.stale_after):
            self.positions.update({j: v + self.sag.get(j, 0.0) for j, v in positions.items()})

    def publish_release(self) -> None:
        self.releases += 1
        self.events.append(("release", None))

    def set_servo_register(self, joint: str, register: str, value: int) -> None:
        self.register_writes.append((joint, register, value))
        self.timeline.append(("register", {register: float(value)}))
        if self.echo_registers:
            self.registers[(joint, register)] = Stamped(value=value, stamp=self.t)

    def servo_register(self, joint: str, register: str) -> Stamped[int] | None:
        return self.registers.get((joint, register))

    def active_source(self) -> Stamped[str] | None:
        if self.on_active_source is not None:
            hook, self.on_active_source = self.on_active_source, None
            hook(self)
        return None if self.source is None else Stamped(value=self.source, stamp=self.t)

    def now(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        self.t += seconds
        self.last_sleep = seconds
        if self.on_sleep is not None:
            self.on_sleep(self)


class DelayedFollower:
    """Follower whose joints reach each published command only after a per-joint delay (an on_sleep hook).

    Lag model of the follower chain: every joint follows the streamed setpoints as a pure delay. Fitted 2026-10-11
    against the actual setpoint publish times with P=16: 0.20-0.24 s on every joint (tests use 0.22 s). The earlier fit
    against index-timed setpoints (0.53-0.65 s) included the 25% tick stretch of the fixed-sleep stream. Joints
    without their own delay use default_delay_s.
    """

    def __init__(self, backend: FakeArmBackend, delays_s: dict[str, float], default_delay_s: float) -> None:
        """Attach to the backend: it stops jumping to commands and replays them delayed on every sleep.

        Args:
            backend (FakeArmBackend): Simulated follower.
            delays_s (dict[str, float]): Joint -> command-to-motion delay (s).
            default_delay_s (float): Delay of the other joints (s).
        """
        self.delays_s = delays_s
        self.default_delay_s = default_delay_s
        backend.follow = False
        backend.on_sleep = self

    def __call__(self, backend: FakeArmBackend) -> None:
        """Set every joint to the newest command published at least its delay ago.

        Args:
            backend (FakeArmBackend): Simulated follower.
        """
        for joint in backend.positions:
            due = backend.t - self.delays_s.get(joint, self.default_delay_s)
            for stamp, command in zip(reversed(backend.command_times), reversed(backend.commands), strict=True):
                if stamp <= due + 1e-9 and joint in command:
                    backend.positions[joint] = command[joint]
                    break


class PerceptionFakeMixin:
    """RobotApi perception/memory/POI and base reset methods for the tool tests (state is set up by init_perception)."""

    def init_perception(self) -> None:
        self.pose = BasePose(frame="map", x=1.0, y=2.0, yaw=0.5, age_s=0.05)
        self.stops = 0
        self.poi_store_up = True
        self.pois: list[dict] = []
        self.poi_commands: list[tuple[str, dict]] = []
        self.poi_clears: list[str] = []
        self.base_resets = 0
        # Footprint sink (footprint.FootprintSink): published polygons, parameter sets and periodic callbacks.
        self.footprint_polygons: list[tuple[str, list[tuple[float, float]]]] = []
        self.stamped_polygons: list[tuple[str, str, list[tuple[float, float]]]] = []
        self.node_params: list[tuple[str, str, str | bool | float]] = []
        self.footprint_timers: list[tuple[float, Callable[[], None]]] = []
        # Outcome each consumer node reports for a parameter set ("ok" when absent).
        self.param_outcomes: dict[str, str] = {}
        # Last polygon seen on each costmap `published_footprint` topic (what the costmap uses).
        self.observed_footprints: dict[str, list[tuple[float, float]]] = {}
        self.consumer_ready_callbacks: list[Callable[[str], None]] = []
        self.steer: dict[str, float] | None = {j: 0.0 for j in STEER_JOINTS}  # None = no fresh swerve joint states
        # Frontier support: settable map (None = no fresh /map), compute_path script (a PathPlan, an exception to raise
        # or a callable (x, y, yaw, timeout_s) -> PathPlan), the calls made and the nav_active flag.
        self.map: MapGrid | None = None
        self.path_plan: PathPlan | Exception | Callable[[float, float, float, float], PathPlan] | None = None
        self.path_requests: list[tuple[float, float, float, float]] = []
        self.nav_busy = False
        self.topdown = TopdownInputs(
            pose=(1.0, 2.0, 0.5),
            scan_points=np.array([[1.0, 0.0], [0.0, 1.0]]),
            plan=np.array([[1.0, 2.0], [2.0, 2.0]]),
            ages={"scan": 0.1, "plan": 2.0},
            missing={"costmap": "no /local_costmap/costmap message within 1.5 s"},
        )

    def robot_pose(self) -> BasePose | None:
        return self.pose

    def map_grid(self) -> MapGrid | None:
        return self.map

    def compute_path(self, x: float, y: float, yaw: float, timeout_s: float) -> PathPlan:
        self.path_requests.append((x, y, yaw, timeout_s))
        plan = self.path_plan
        if isinstance(plan, Exception):
            raise plan
        if callable(plan):
            return plan(x, y, yaw, timeout_s)
        if plan is None:
            raise RobotError("Nav2 planner action server /compute_path_to_pose is not ready")
        return plan.model_copy(update={"goal": BasePose(frame="map", x=x, y=y, yaw=yaw)})

    def nav_active(self) -> bool:
        return self.nav_busy

    def reset_base(self) -> bool:
        self.base_resets += 1
        return True

    def steer_positions(self) -> dict[str, float] | None:
        return None if self.steer is None else dict(self.steer)

    def publish_polygon(self, topic: str, points: list[tuple[float, float]]) -> None:
        self.footprint_polygons.append((topic, list(points)))

    def publish_stamped_polygon(self, topic: str, frame: str, points: list[tuple[float, float]]) -> None:
        self.stamped_polygons.append((topic, frame, list(points)))

    def set_node_parameter(
        self, node: str, name: str, value: str | bool | float, report: Callable[[str], None] | None = None
    ) -> None:
        self.node_params.append((node, name, value))
        if report is not None:
            report(self.param_outcomes.get(node, "ok"))

    def observed_polygon(self, topic: str) -> list[tuple[float, float]] | None:
        return self.observed_footprints.get(topic)

    def every(self, period_s: float, callback: Callable[[], None]) -> None:
        self.footprint_timers.append((period_s, callback))

    def on_consumer_ready(self, callback: Callable[[str], None]) -> None:
        self.consumer_ready_callbacks.append(callback)

    def arm_lease_held(self) -> bool:
        return self.arm.control_held

    def arm_parts(self) -> dict[str, np.ndarray] | None:
        sample = self.arm.fresh_sample()
        return None if sample is None else self.arm.body_check.part_points(self.arm.measured(sample))

    def arm_mount_xy(self) -> tuple[float, float]:
        mount = self.arm.cfg.arm.base_in_base_link
        return (0.0, 0.0) if mount is None else (mount.x, mount.y)

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
