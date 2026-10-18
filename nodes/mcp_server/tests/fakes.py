"""Test doubles: a simulated follower arm backend with a fake clock, and poi_store's photo store."""

import json
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np
from pydantic import ValidationError
from ros2_common.lookups import INDEX_FILE, LookupRecord
from ros2_common.maps import MapResult, MapStatus
from ros2_common.photos import PHOTO_KIND, PhotoAdd, PhotoSelector, photo_as_poi

from mcp_server import config as mcp_config
from mcp_server.approach import CostGrid, PathInfo
from mcp_server.arm import JointSample
from mcp_server.base_dock import DockGoal, DockResult
from mcp_server.frontiers import MapGrid
from mcp_server.map_client import MapRequests
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
        # Gripper Present_Current (A) as /follower/joint_currents would carry it; empty = no fresh currents message.
        self.currents: dict[str, float] = {}
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
        return JointSample(
            positions=dict(self.positions), efforts=dict(self.efforts), stamp=stamp, currents=dict(self.currents)
        )

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

    def begin_motion(self, subsystems: tuple[str, ...]) -> None:
        """Full-rate joint states for a motion: the simulated follower is always at full rate."""

    def end_motion(self, subsystems: tuple[str, ...]) -> None:
        """End of the full-rate motion window: nothing to do for the simulated follower."""


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


PHOTO_EDITABLE = {"id", "name", "note", "status", "poi_ids"}


class FakePhotoStore:
    """poi_store's photo store as the tool tests see it (E11): it owns index.jsonl next to the JPEGs mcp_server writes.

    add / update / delete / delete_photos follow the poi_store contract and answer in photo_as_poi form; a rejection is
    a RobotError carrying poi_store's message.
    """

    def __init__(self, root: Path) -> None:
        self.root = root

    @property
    def index(self) -> Path:
        return self.root / INDEX_FILE

    def records(self) -> list[LookupRecord]:
        if not self.index.exists():
            return []
        recs = []
        for line in self.index.read_text().splitlines():
            try:
                recs.append(LookupRecord.model_validate_json(line))
            except ValidationError:
                continue
        return recs

    def holds(self, photo_id: object) -> bool:
        return any(r.id == photo_id for r in self.records())

    def write(self, recs: list[LookupRecord]) -> None:
        self.index.write_text("".join(r.model_dump_json() + "\n" for r in recs))

    def add(self, poi: dict[str, Any]) -> dict[str, Any]:
        try:
            payload = PhotoAdd.model_validate({k: v for k, v in poi.items() if k != "kind"})
        except ValidationError as exc:
            raise RobotError(f"poi_store rejected the add: {exc}") from exc
        image = self.root / payload.file
        if not image.is_file():
            raise RobotError(f"poi_store rejected the add: photo file {payload.file} does not exist")
        if self.holds(payload.id):
            raise RobotError(f"poi_store rejected the add: photo id {payload.id} exists")
        rec = LookupRecord.model_validate(payload.model_dump() | {"bytes": image.stat().st_size})
        with self.index.open("a") as fh:
            fh.write(rec.model_dump_json() + "\n")
        return photo_as_poi(rec)

    def update(self, poi: dict[str, Any], pois: list[dict[str, Any]]) -> dict[str, Any]:
        if set(poi) - PHOTO_EDITABLE:
            raise RobotError("poi_store rejected the update: photo capture data is immutable")
        known = {str(p.get("id")) for p in pois}
        dangling = [p for p in poi.get("poi_ids", []) if p not in known]
        if dangling:
            raise RobotError(f"poi_store rejected the update: unknown poi_ids {dangling}")
        recs = self.records()
        index = next(i for i, r in enumerate(recs) if r.id == poi["id"])
        changes = {k: v for k, v in poi.items() if k != "id"}
        recs[index] = recs[index].model_copy(update=changes | {"updated_at": time.time()})
        self.write(recs)
        return photo_as_poi(recs[index])

    def delete(self, photo_id: str) -> dict[str, Any]:
        recs = self.records()
        rec = next(r for r in recs if r.id == photo_id)
        self.write([r for r in recs if r.id != photo_id])
        (self.root / rec.file).unlink(missing_ok=True)
        return photo_as_poi(rec)

    def delete_many(self, raw: dict[str, Any]) -> dict[str, Any]:
        selector = PhotoSelector.model_validate(raw)
        recs = [
            r
            for r in self.records()
            if (selector.created_by is None or r.created_by == selector.created_by)
            and (selector.camera is None or r.camera == selector.camera)
        ]
        missing: list[str] = []
        if selector.ids is not None:
            chosen = [r for r in recs if r.id in selector.ids]
            missing = [i for i in selector.ids if i not in {r.id for r in chosen}]
        elif selector.older_than_s is not None:
            chosen = [r for r in recs if time.time() - r.ts > selector.older_than_s]
        else:
            newest = sorted(recs, key=lambda r: r.ts, reverse=True)
            chosen = newest[selector.keep_newest :]
        size = sum(r.bytes for r in chosen)
        if selector.dry_run:
            return {"would_remove": len(chosen), "bytes": size}
        gone = {r.id for r in chosen}
        self.write([r for r in self.records() if r.id not in gone])
        for rec in chosen:
            (self.root / rec.file).unlink(missing_ok=True)
        return {"removed": len(chosen), "bytes_freed": size, "missing": missing}


class PerceptionFakeMixin:
    """RobotApi perception/memory/POI and base reset methods for the tool tests (state is set up by init_perception)."""

    def init_perception(self) -> None:
        self.pose = BasePose(frame="map", x=1.0, y=2.0, yaw=0.5, age_s=0.05)
        self.stops = 0
        self.poi_store_up = True
        self.pois: list[dict] = []
        self.poi_commands: list[tuple[str, dict]] = []
        self.poi_clears: list[str | None] = []
        self.poi_clear_args: list[dict] = []
        # map_manager: commands seen, the scripted answer (a result dict without request_id, None = no answer),
        # whether it listens, the latched status (None = none yet) and a hook run on stop.
        self.map_commands: list[dict] = []
        self.map_script: Callable[[dict], dict | None] = lambda cmd: {"ok": True, "message": "ok"}
        self.map_up = True
        self.map_status_msg: MapStatus | None = None
        self.stop_hook: Callable[[], None] | None = None
        self.map_client = MapRequests(self.map_publish, lambda: self.map_up, lambda op: 0.05)
        self.poi_clear_statuses: list[list[str] | None] = []
        # poi_store's photo store (E11) in the default lookups dir (conftest points it at the test's tmp dir), every
        # photo command (op, payload) and an error to raise for photo commands (PoiTimeout, PoiStoreDown, ...).
        self.photo_store = FakePhotoStore(mcp_config.DEFAULT_LOOKUPS_DIR)
        self.photo_commands: list[tuple[str, dict]] = []
        self.photo_error: Exception | None = None
        self.base_resets = 0
        # Grasp docking (no motion): goals counted, detections recorded, costmap and planner results set by the test.
        self.dock_goals: list[tuple[DockGoal, float]] = []
        self.dock_result: DockResult | None = None  # None = succeeded
        self.planned_poses: list[BasePose] = []
        self.path_info = PathInfo(status="ok", length_m=1.0)
        self.costmap: CostGrid | None = None
        self.detections: list[tuple[BasePose, int]] = []
        self.dock_server_up = True  # whether the docking action server answers
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

    def dock(self, goal: DockGoal, timeout_s: float) -> DockResult:
        self.dock_goals.append((goal, timeout_s))
        return self.dock_result or DockResult(status="succeeded", goal=goal, final_pose=self.pose)

    def dock_ready(self) -> bool:
        return self.dock_server_up

    def plan_path(self, goal: BasePose) -> PathInfo:
        self.planned_poses.append(goal)
        return self.path_info

    def global_costmap(self) -> CostGrid | None:
        return self.costmap

    def publish_detection(self, pose: BasePose, stamp_ns: int) -> None:
        self.detections.append((pose, stamp_ns))

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

    def map_publish(self, text: str) -> None:
        cmd = json.loads(text)
        self.map_commands.append(cmd)
        reply = self.map_script(cmd)
        if reply is not None:
            self.map_client.on_result(json.dumps({"request_id": cmd["request_id"], **reply}))

    def map_status(self) -> MapStatus:
        if self.map_status_msg is None:
            raise RobotError("no /map_manager/status received yet")
        return self.map_status_msg

    def map_request(self, command: dict) -> MapResult:
        return self.map_client.send(command)

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

    def poi_clear(
        self,
        created_by: str | None,
        statuses: list[str] | None = None,
        kinds: list[str] | None = None,
        keep_zones: bool = False,
    ) -> dict:
        if not self.poi_store_up:
            raise PoiStoreDown("poi_store is not running (no subscriber on /poi/command)")
        self.poi_clears.append(created_by)
        self.poi_clear_statuses.append(statuses)
        self.poi_clear_args.append({"created_by": created_by, "kinds": kinds, "keep_zones": keep_zones})

        def doomed_poi(p: dict) -> bool:
            zone = p.get("kind") == "area" and p.get("zone") in ("keepout", "slow")
            return (
                (created_by is None or p.get("created_by") == created_by)
                and (statuses is None or p.get("status") in statuses)
                and (kinds is None or p.get("kind") in kinds)
                and not (keep_zones and zone)
            )

        doomed = [p for p in self.pois if doomed_poi(p)]
        self.pois = [p for p in self.pois if not doomed_poi(p)]
        return {"ok": True, "message": f"cleared {len(doomed)}", "poi": {"removed": len(doomed)}}

    def poi_request(self, op: str, poi: dict) -> dict:
        if not self.poi_store_up:
            raise PoiStoreDown("poi_store is not running (no subscriber on /poi/command)")
        is_photo = (
            op == "delete_photos"
            or (op == "add" and poi.get("kind") == PHOTO_KIND)
            or (op in ("update", "delete") and self.photo_store.holds(poi.get("id")))
        )
        if is_photo:
            return self.photo_request(op, poi)
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

    def photo_request(self, op: str, poi: dict) -> dict:
        """A photo command answered by the fake photo store (photo_error is raised instead when set)."""
        self.photo_commands.append((op, dict(poi)))
        if self.photo_error is not None:
            raise self.photo_error
        store = self.photo_store
        if op == "add":
            return {"ok": True, "message": "added photo", "poi": store.add(poi)}
        if op == "update":
            return {"ok": True, "message": "updated photo", "poi": store.update(poi, self.pois)}
        if op == "delete":
            return {"ok": True, "message": "deleted photo", "poi": store.delete(poi["id"])}
        return {"ok": True, "message": "deleted photos", "poi": store.delete_many(poi)}
