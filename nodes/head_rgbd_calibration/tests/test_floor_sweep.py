"""Tests for the guided head floor sweep: fake node, fake clock and sleep, synthetic clouds."""

import importlib.util
import json
import math
import re
from collections import Counter
from collections.abc import Callable
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.chain import UrdfChain

from head_rgbd_calibration import floor_sweep
from head_rgbd_calibration.config import FloorSweepConfig, NodeConfig
from head_rgbd_calibration.floor_sweep import Busy, FloorSweep, SweepError

REPO_ROOT = Path(__file__).resolve().parents[3]
URDF_TEXT = (REPO_ROOT / "nodes" / "web_ui" / "urdf" / "head.urdf").read_text()
SYNTHETIC = importlib.util.spec_from_file_location(
    "head_floor_synthetic", REPO_ROOT / "tests" / "head_floor_synthetic.py"
)
assert SYNTHETIC is not None and SYNTHETIC.loader is not None
synthetic = importlib.util.module_from_spec(SYNTHETIC)
SYNTHETIC.loader.exec_module(synthetic)

TICK_S = 0.05
CLOUD_EVERY_TICKS = 2
SPEED_DEG_S = 60.0
START_DEG = (10.0, 5.0)
RAYS = (30, 20)
SID_PATTERN = re.compile(r"^floor-\d{8}-\d{6}$")
CHAIN = UrdfChain.from_text(URDF_TEXT)


class FakeLogger:
    def info(self, *_a: Any, **_k: Any) -> None: ...
    def warning(self, *_a: Any, **_k: Any) -> None: ...
    def error(self, *_a: Any, **_k: Any) -> None: ...


class FakePublisher:
    def __init__(self, world: "World") -> None:
        self.world = world

    def publish(self, msg: Any) -> None:
        self.world.command(msg)


class FakeNode:
    def __init__(self, world: "World") -> None:
        self.world = world
        self.created: list[Any] = []
        self.destroyed: list[Any] = []

    def get_logger(self) -> FakeLogger:
        return FakeLogger()

    def create_subscription(self, _type: Any, topic: str, callback: Callable[[Any], None], _qos: Any) -> Any:
        handle = SimpleNamespace(kind="sub", topic=topic)
        self.world.callbacks[topic] = callback
        self.created.append(handle)
        return handle

    def create_publisher(self, _type: Any, topic: str, _qos: Any) -> Any:
        handle = FakePublisher(self.world)
        handle.topic = topic  # type: ignore[attr-defined]
        self.created.append(handle)
        return handle

    def destroy_subscription(self, handle: Any) -> None:
        self.destroyed.append(handle)

    def destroy_publisher(self, handle: Any) -> None:
        self.destroyed.append(handle)


class World:
    """Head model plus fake clock: advancing time moves the head and delivers joint states and clouds."""

    def __init__(self) -> None:
        self.t = 100.0
        self.pan, self.tilt = START_DEG
        self.target = START_DEG
        self.commands: list[Any] = []
        self.command_times: list[float] = []
        self.callbacks: dict[str, Callable[[Any], None]] = {}
        self.blocked: set[tuple[float, float]] = set()
        self.joints_until: float | None = None
        self.publish_joints = True
        self.cloud_offsets: Callable[[int], float] = lambda _n: 0.0
        self.cloud_empty: Callable[[int, float, float], bool] = lambda _n, _pan, _tilt: False
        self.on_tick: Callable[[float], None] | None = None
        self.cloud_count = 0
        self.ticks = 0

    def clock(self) -> float:
        return self.t

    def command(self, msg: Any) -> None:
        self.commands.append(msg)
        self.command_times.append(self.t)
        pan, tilt = (math.degrees(v) for v in msg.position)
        if (round(pan, 3), round(tilt, 3)) not in self.blocked:
            self.target = (pan, tilt)

    def move(self, current: float, target: float) -> float:
        step = SPEED_DEG_S * TICK_S
        return target if abs(target - current) <= step else current + math.copysign(step, target - current)

    def stamp(self, t: float) -> Any:
        ns = int(round(t * 1e9))
        return SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

    def sleep(self, seconds: float) -> None:
        for _ in range(max(1, round(seconds / TICK_S))):
            self.tick()

    def tick(self) -> None:
        self.t += TICK_S
        self.ticks += 1
        self.pan = self.move(self.pan, self.target[0])
        self.tilt = self.move(self.tilt, self.target[1])
        joints_on = self.publish_joints and (self.joints_until is None or self.t <= self.joints_until)
        joint_cb = self.callbacks.get("/head/joint_states")
        if joint_cb is not None and joints_on:
            joint_cb(
                SimpleNamespace(
                    header=SimpleNamespace(stamp=self.stamp(self.t), frame_id=""),
                    name=["head_pan", "head_tilt"],
                    position=[math.radians(self.pan), math.radians(self.tilt)],
                )
            )
        cloud_cb = self.callbacks.get("/tof/points")
        if cloud_cb is not None and self.ticks % CLOUD_EVERY_TICKS == 0:
            stamp = self.t + self.cloud_offsets(self.cloud_count)
            empty = self.cloud_empty(self.cloud_count, self.pan, self.tilt)
            self.cloud_count += 1
            cloud_cb(
                SimpleNamespace(
                    header=SimpleNamespace(stamp=self.stamp(stamp), frame_id="tof_optical_frame"),
                    pan=self.pan,
                    tilt=self.tilt,
                    empty=empty,
                )
            )
        if self.on_tick is not None:
            self.on_tick(self.t)


def read_points(msg: Any, **_kwargs: Any) -> np.ndarray:
    """Floor points of the flat floor seen from the head angles the cloud was taken at; none for an empty cloud."""
    if msg.empty:
        return np.zeros((0, 3))
    pose = synthetic.sensor_pose(CHAIN, math.radians(msg.pan), math.radians(msg.tilt), {})
    return synthetic.floor_points(pose, synthetic.ray_grid(RAYS))


@pytest.fixture(autouse=True)
def patched_points(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(floor_sweep.point_cloud2, "read_points_numpy", read_points)


def make_cfg(tmp_path: Path, **floor: Any) -> NodeConfig:
    defaults: dict[str, Any] = {
        "pan_deg": [-30, 30],
        "tilt_deg": [20, 28],
        "clouds_per_pose": 2,
        "fit": {"plane": {"min_inliers": 50, "ransac_iterations": 40}},
    }
    return NodeConfig(state_dir=str(tmp_path), floor=FloorSweepConfig(**{**defaults, **floor}))


def make_sweep(
    tmp_path: Path, world: World, blocked: Callable[[], str | None] | None = None, **floor: Any
) -> tuple[FloorSweep, FakeNode]:
    node = FakeNode(world)
    sweep = FloorSweep(
        node,
        make_cfg(tmp_path, **floor),
        lambda: URDF_TEXT,
        world.clock,
        world.sleep,
        now=lambda: datetime(2026, 10, 21, 12, 30, 45),
        blocked=blocked,
    )
    return sweep, node


def run(sweep: FloorSweep) -> str:
    sid = sweep.start()
    sweep.wait(60)
    return sid


def sweep_dir(tmp_path: Path, sid: str) -> Path:
    return tmp_path / "floor_fit" / sid


def degrees(world: World) -> list[tuple[float, float]]:
    return [(round(math.degrees(c.position[0]), 3), round(math.degrees(c.position[1]), 3)) for c in world.commands]


def test_commands_serpentine_grid_with_frame_id(tmp_path: Path) -> None:
    world = World()
    sweep, _node = make_sweep(tmp_path, world)
    run(sweep)
    assert degrees(world) == [(-30, 20), (30, 20), (30, 28), (-30, 28), START_DEG]
    for msg in world.commands:
        assert msg.name == ["head_pan", "head_tilt"]
        assert msg.header.frame_id == "head_rgbd_calibration"
    assert sweep.state()["status"] == "done"


def test_session_id_and_directory(tmp_path: Path) -> None:
    world = World()
    sweep, _node = make_sweep(tmp_path, world)
    sid = run(sweep)
    assert SID_PATTERN.match(sid)
    assert sid == "floor-20261021-123045"
    assert (sweep_dir(tmp_path, sid) / "observations.json").is_file()
    assert sweep.state()["session_id"] == sid


def test_writes_observations_json(tmp_path: Path) -> None:
    world = World()
    sweep, _node = make_sweep(tmp_path, world)
    sid = run(sweep)
    data = json.loads((sweep_dir(tmp_path, sid) / "observations.json").read_text())
    assert data["session"] == sid
    assert "created" in data
    assert [(g["pan_deg"], g["tilt_deg"]) for g in data["grid"]] == [(-30, 20), (30, 20), (30, 28), (-30, 28)]
    assert len(data["observations"]) == 8
    first = data["observations"][0]
    assert set(first) == {
        "pose",
        "pan_rad",
        "tilt_rad",
        "normal",
        "height_m",
        "inliers",
        "candidates",
        "rms_m",
        "stamp_ns",
    }
    assert first["pose"] == 0
    assert first["pan_rad"] == pytest.approx(math.radians(-30), abs=1e-6)
    assert first["tilt_rad"] == pytest.approx(math.radians(20), abs=1e-6)
    assert len(first["normal"]) == 3
    assert first["height_m"] > 0
    assert data["skipped"] == []
    assert not list(sweep_dir(tmp_path, sid).glob("*.tmp*"))


def test_clouds_only_after_still_and_settle(tmp_path: Path) -> None:
    world = World()
    sweep, _node = make_sweep(tmp_path, world, settle_s=0.5)
    sid = run(sweep)
    data = json.loads((sweep_dir(tmp_path, sid) / "observations.json").read_text())
    for obs in data["observations"]:
        command_t = world.command_times[obs["pose"]]
        assert obs["stamp_ns"] / 1e9 > command_t + 0.5


def test_skips_unreachable_pose_and_unmatched_cloud(tmp_path: Path) -> None:
    test_pose_with_only_unmatched_clouds_skipped_with_reason(tmp_path / "unmatched")
    world = World()
    world.blocked = {(30.0, 28.0)}
    sweep, _node = make_sweep(tmp_path, world, move_timeout_s=2.0)
    sid = run(sweep)
    data = json.loads((sweep_dir(tmp_path, sid) / "observations.json").read_text())
    assert [s["pose"] for s in data["skipped"]] == [2]
    assert data["skipped"][0]["pan_deg"] == 30
    assert data["skipped"][0]["tilt_deg"] == 28
    assert "did not reach pan/tilt" in data["skipped"][0]["reason"]
    assert {o["pose"] for o in data["observations"]} == {0, 1, 3}
    assert sweep.state()["skipped"] == 1


def test_clouds_without_joint_state_dropped_and_counted(tmp_path: Path) -> None:
    world = World()
    world.cloud_offsets = lambda n: 5.0 if n % 2 else 0.0
    sweep, _node = make_sweep(tmp_path, world, clouds_per_pose=4, min_clouds_per_pose=1)
    sid = run(sweep)
    data = json.loads((sweep_dir(tmp_path, sid) / "observations.json").read_text())
    per_pose = {p: sum(1 for o in data["observations"] if o["pose"] == p) for p in range(4)}
    assert all(0 < n < 4 for n in per_pose.values())
    assert data["skipped"] == []
    assert set(data["dropped"]) == {"no joint state within max_stamp_dt_s"}
    assert data["dropped"]["no joint state within max_stamp_dt_s"] == 4 * 4 - len(data["observations"])


def test_pose_with_only_unmatched_clouds_skipped_with_reason(tmp_path: Path) -> None:
    world = World()
    world.cloud_offsets = lambda _n: 5.0
    sweep, _node = make_sweep(tmp_path, world)
    sid = run(sweep)
    data = json.loads((sweep_dir(tmp_path, sid) / "observations.json").read_text())
    assert data["observations"] == []
    assert len(data["skipped"]) == 4
    assert "no joint state within max_stamp_dt_s" in data["skipped"][0]["reason"]


def test_restores_start_pose_on_every_exit(tmp_path: Path) -> None:
    test_restore_and_handles_on_stop(tmp_path / "stop")
    test_restore_and_handles_on_error(tmp_path / "error")
    world = World()
    sweep, node = make_sweep(tmp_path, world)
    run(sweep)
    assert degrees(world)[-1] == START_DEG
    assert (round(world.pan, 3), round(world.tilt, 3)) == START_DEG
    assert len(node.created) == 3
    assert sorted(map(id, node.destroyed)) == sorted(map(id, node.created))


def test_restore_and_handles_on_stop(tmp_path: Path) -> None:
    world = World()
    sweep, node = make_sweep(tmp_path, world)

    def stop_when_second_pose_commanded(_t: float) -> None:
        if len(world.commands) == 2:
            world.on_tick = None
            sweep.stop()

    world.on_tick = stop_when_second_pose_commanded
    sid = run(sweep)
    assert degrees(world)[-1] == START_DEG
    assert sweep.state()["status"] == "stopped"
    assert sweep.state()["running"] is False
    data = json.loads((sweep_dir(tmp_path, sid) / "observations.json").read_text())
    assert {o["pose"] for o in data["observations"]} == {0}
    assert sorted(map(id, node.destroyed)) == sorted(map(id, node.created))


def test_restore_and_handles_on_error(tmp_path: Path) -> None:
    world = World()
    world.joints_until = world.t + 3.0
    sweep, node = make_sweep(tmp_path, world, move_timeout_s=2.0)
    sid = run(sweep)
    state = sweep.state()
    assert state["status"] == "error"
    assert "joint states" in state["error"]
    assert len(world.commands) >= 2
    assert degrees(world)[-1] == START_DEG
    assert (sweep_dir(tmp_path, sid) / "observations.json").is_file()
    assert sorted(map(id, node.destroyed)) == sorted(map(id, node.created))


def test_restore_and_handles_on_exception(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    world = World()

    def boom(*_a: Any, **_k: Any) -> None:
        raise RuntimeError("boom")

    monkeypatch.setattr(floor_sweep, "fit_plane", boom)
    sweep, node = make_sweep(tmp_path, world)
    sid = run(sweep)
    state = sweep.state()
    assert state["status"] == "error"
    assert "boom" in state["error"]
    assert degrees(world)[-1] == START_DEG
    assert (sweep_dir(tmp_path, sid) / "observations.json").is_file()
    assert sorted(map(id, node.destroyed)) == sorted(map(id, node.created))


def test_busy_while_running(tmp_path: Path) -> None:
    world = World()
    sweep, _node = make_sweep(tmp_path, world)
    errors: list[Exception] = []

    def try_second_start(_t: float) -> None:
        world.on_tick = None
        try:
            sweep.start()
        except Busy as exc:
            errors.append(exc)

    world.on_tick = try_second_start
    first_sid = run(sweep)
    assert len(errors) == 1
    assert sweep.state()["status"] == "done"
    sweep2_sid = run(sweep)
    assert sweep2_sid.startswith("floor-20261021-123045")
    assert sweep2_sid != first_sid


def test_refuses_without_joint_state(tmp_path: Path) -> None:
    world = World()
    world.publish_joints = False
    sweep, node = make_sweep(tmp_path, world, move_timeout_s=1.0)
    with pytest.raises(SweepError, match="no head joint state"):
        sweep.start()
    assert world.commands == []
    assert sorted(map(id, node.destroyed)) == sorted(map(id, node.created))
    assert not (tmp_path / "floor_fit").exists()
    assert sweep.state()["running"] is False


def test_blocked_by_capture_refused_nothing_created(tmp_path: Path) -> None:
    world = World()
    sweep, node = make_sweep(tmp_path, world, blocked=lambda: "an RGB-D capture is running")
    with pytest.raises(Busy, match="RGB-D capture is running"):
        sweep.start()
    assert node.created == []
    assert world.commands == []


def test_state_progress_fields(tmp_path: Path) -> None:
    world = World()
    sweep, _node = make_sweep(tmp_path, world)
    idle = sweep.state()
    assert idle == {
        "running": False,
        "session_id": None,
        "pose": 0,
        "total": 0,
        "observations": 0,
        "skipped": 0,
        "status": "idle",
        "error": None,
    }
    seen: list[dict[str, Any]] = []
    world.on_tick = lambda _t: seen.append(sweep.state())
    run(sweep)
    final = sweep.state()
    assert final["total"] == 4
    assert final["pose"] == 4
    assert final["observations"] == 8
    assert final["running"] is False
    assert any(s["running"] and s["total"] == 4 for s in seen)


def test_grid_stays_inside_configured_range(tmp_path: Path) -> None:
    world = World()
    sweep, _node = make_sweep(tmp_path, world, pan_deg=[-60, 0, 60], tilt_deg=[20])
    run(sweep)
    pans = [p for p, _t in degrees(world)[:-1]]
    assert pans == [-60, 0, 60]


def test_partly_blocked_pose_is_skipped(tmp_path: Path) -> None:
    world = World()

    def empty(n: int, pan: float, tilt: float) -> bool:
        """At (30, 20) 3 of every 4 clouds see no floor, at (-30, 28) every other one."""
        at = (round(pan, 3), round(tilt, 3))
        return (at == (30.0, 20.0) and n % 4 != 0) or (at == (-30.0, 28.0) and n % 2 == 1)

    world.cloud_empty = empty
    sweep, _node = make_sweep(tmp_path, world, clouds_per_pose=4)
    assert sweep.floor.min_clouds_per_pose == 2
    sid = run(sweep)
    data = json.loads((sweep_dir(tmp_path, sid) / "observations.json").read_text())
    assert data["skipped"] == [{"pose": 1, "pan_deg": 30, "tilt_deg": 20, "reason": "partly blocked: 1 of 4 clouds"}]
    per_pose = Counter(o["pose"] for o in data["observations"])
    assert per_pose == {0: 4, 2: 4, 3: 2}
    assert data["dropped"] == {"no floor plane in the cloud": 3 + 2}
    assert sweep.state()["skipped"] == 1
    assert sweep.state()["observations"] == 10

    # A higher minimum also skips the half-blocked pose; nothing of a skipped pose is kept.
    world = World()
    world.cloud_empty = empty
    sweep, _node = make_sweep(tmp_path / "strict", world, clouds_per_pose=4, min_clouds_per_pose=3)
    sid = run(sweep)
    data = json.loads((sweep_dir(tmp_path / "strict", sid) / "observations.json").read_text())
    assert [(s["pose"], s["reason"]) for s in data["skipped"]] == [
        (1, "partly blocked: 1 of 4 clouds"),
        (3, "partly blocked: 2 of 4 clouds"),
    ]
    assert {o["pose"] for o in data["observations"]} == {0, 2}
