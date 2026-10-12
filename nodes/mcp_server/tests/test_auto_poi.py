"""AutoPoiMarker: collision_stop and Nav2 aborts become stuck point POIs, wired through the monitor sink fan-out and the
navigate_to_pose result hook in call_tool_with_digest (auto_poi_id on the digest event)."""

import logging
import threading
import time
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError

from mcp_server.auto_poi import AUTO_POI_ID_KEY, AutoPoiMarker, attach_auto_poi
from mcp_server.config import AutoPoiSettings, McpServerConfig, MonitorSettings
from mcp_server.models import BasePose, NavigationResult, RobotError, RobotEvent
from mcp_server.monitor import RobotMonitor
from mcp_server.poi_client import PoiStoreDown
from mcp_server.tools import build_mcp_server

from .test_tools import TOKEN, FakeRobot, call, take

DIGEST_KEY = "robot_events_since_last_call"
EVENT_TS = 1_760_270_400.0  # 2025-10-12 12:00:00 UTC


class FakePoiStore:
    """The PoiBackend the marker needs: an in-memory store that records every command."""

    def __init__(self) -> None:
        self.pois: list[dict[str, Any]] = []
        self.commands: list[tuple[str, dict[str, Any]]] = []
        self.list_error: Exception | None = None
        self.request_error: Exception | None = None

    def poi_list(self) -> tuple[list[dict[str, Any]], int]:
        if self.list_error is not None:
            raise self.list_error
        return [dict(p) for p in self.pois], len(self.commands)

    def poi_request(self, op: str, poi: dict[str, Any]) -> dict[str, Any]:
        if self.request_error is not None:
            raise self.request_error
        self.commands.append((op, dict(poi)))
        if op == "add":
            stored = {"id": f"{len(self.pois):032x}", "status": "open", "polygon": [], "radius_m": 0.2, **poi}
            self.pois.append(stored)
            return {"ok": True, "poi": stored}
        match = next(p for p in self.pois if p["id"] == poi["id"])
        match.update(poi)
        return {"ok": True, "poi": dict(match)}


def event(type_: str = "collision_stop", seq: int = 1, ts: float = EVENT_TS, **data: Any) -> RobotEvent:
    return RobotEvent(
        seq=seq, ts=ts, type=type_, severity="critical", source="FootprintStop", message="stopped", data=data
    )


def pose(x: float = 1.0, y: float = 2.0, age_s: float | None = 0.1) -> BasePose:
    return BasePose(frame="map", x=x, y=y, yaw=0.0, age_s=age_s)


@pytest.fixture
def store() -> FakePoiStore:
    return FakePoiStore()


@pytest.fixture
def marker(store: FakePoiStore) -> AutoPoiMarker:
    return AutoPoiMarker(store, AutoPoiSettings())


# --- AutoPoiMarker.on_event -----------------------------------------------------------------------------------------


def test_collision_stop_with_a_fresh_pose_adds_an_agent_point_poi(marker: AutoPoiMarker, store: FakePoiStore) -> None:
    poi_id = marker.on_event(event(polygon="FootprintStop"), pose(1.5, -0.5))
    assert poi_id == store.pois[0]["id"]
    [(op, fields)] = store.commands
    assert op == "add"
    assert fields["kind"] == "point" and fields["frame"] == "map"
    assert (fields["x"], fields["y"]) == (1.5, -0.5)
    assert fields["created_by"] == "agent"
    assert fields["name"] == "stuck: collision stop"
    assert "stuck 1 time" in fields["note"]
    assert time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(EVENT_TS)) in fields["note"]
    assert "FootprintStop" in fields["note"]


def test_nav_abort_adds_a_point_named_after_the_abort(marker: AutoPoiMarker, store: FakePoiStore) -> None:
    poi_id = marker.on_event(event("nav_abort", tool="navigate_to_pose", message="robot in inflation"), pose())
    assert poi_id is not None
    assert store.commands[0][1]["name"] == "stuck: Nav2 abort"
    assert "robot in inflation" in store.commands[0][1]["note"]


def test_a_repeat_within_the_merge_radius_updates_the_same_poi(marker: AutoPoiMarker, store: FakePoiStore) -> None:
    first = marker.on_event(event(seq=1), pose(1.0, 2.0))
    second = marker.on_event(event("nav_abort", seq=2, ts=EVENT_TS + 60), pose(1.2, 2.1))
    assert second == first
    assert len(store.pois) == 1
    op, fields = store.commands[-1]
    assert op == "update"
    assert fields["id"] == first
    assert "stuck 2 times" in fields["note"]
    assert time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(EVENT_TS + 60)) in fields["note"]
    assert set(fields) == {"id", "note"}  # an update never touches kind, position, zone or polygon
    third = marker.on_event(event(seq=3, ts=EVENT_TS + 120), pose(0.9, 2.0))
    assert third == first and "stuck 3 times" in store.pois[0]["note"]


def test_a_stuck_spot_beyond_the_merge_radius_is_a_new_poi(marker: AutoPoiMarker, store: FakePoiStore) -> None:
    first = marker.on_event(event(seq=1), pose(0.0, 0.0))
    second = marker.on_event(event(seq=2), pose(5.0, 0.0))
    assert first != second and len(store.pois) == 2


def test_only_open_agent_stuck_pois_are_merged(marker: AutoPoiMarker, store: FakePoiStore) -> None:
    store.pois = [
        {"id": "a" * 32, "kind": "point", "name": "stuck: collision stop", "x": 1.0, "y": 2.0, "status": "done",
         "created_by": "agent", "note": "stuck 1 time"},
        {"id": "b" * 32, "kind": "point", "name": "sofa", "x": 1.0, "y": 2.0, "status": "open",
         "created_by": "agent", "note": ""},
        {"id": "c" * 32, "kind": "point", "name": "stuck: mine", "x": 1.0, "y": 2.0, "status": "open",
         "created_by": "user", "note": ""},
    ]  # fmt: skip
    poi_id = marker.on_event(event(), pose(1.0, 2.0))
    assert poi_id not in {"a" * 32, "b" * 32, "c" * 32}
    assert store.commands[0][0] == "add"


def test_a_stuck_area_the_agent_drew_around_the_spot_is_updated(marker: AutoPoiMarker, store: FakePoiStore) -> None:
    square = [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]
    store.pois = [
        {"id": "d" * 32, "kind": "area", "zone": "keepout", "polygon": square, "name": "stuck: bean bag", "x": 1.0,
         "y": 1.0, "status": "open", "created_by": "agent", "note": "stuck 4 times, last x"},
    ]  # fmt: skip
    assert marker.on_event(event(), pose(1.9, 1.9)) == "d" * 32
    op, fields = store.commands[0]
    assert op == "update" and set(fields) == {"id", "note"} and "stuck 5 times" in fields["note"]


def test_no_pose_or_a_stale_pose_means_no_poi(marker: AutoPoiMarker, store: FakePoiStore) -> None:
    assert marker.on_event(event(), None) is None
    assert marker.on_event(event(), pose(age_s=2.5)) is None  # older than auto_poi.max_pose_age_s (2 s)
    assert marker.on_event(event(), pose(age_s=None)) is None  # unknown age cannot be vouched for
    assert marker.on_event(event(), BasePose(frame="odom", x=0.0, y=0.0, yaw=0.0, age_s=0.0)) is None
    assert store.commands == []


def test_other_event_types_and_a_disabled_marker_add_nothing(store: FakePoiStore) -> None:
    assert AutoPoiMarker(store, AutoPoiSettings()).on_event(event("bump"), pose()) is None
    assert AutoPoiMarker(store, AutoPoiSettings(enabled=False)).on_event(event(), pose()) is None
    only_aborts = AutoPoiMarker(store, AutoPoiSettings(event_types=["nav_abort"]))
    assert only_aborts.on_event(event(), pose()) is None
    assert store.commands == []
    assert only_aborts.on_event(event("nav_abort"), pose()) is not None


def test_the_marker_never_creates_an_area_or_a_keepout(marker: AutoPoiMarker, store: FakePoiStore) -> None:
    for i, (x, y) in enumerate([(0.0, 0.0), (3.0, 0.0), (0.1, 0.0)]):
        marker.on_event(event(seq=i + 1), pose(x, y))
    for op, fields in store.commands:
        assert fields.get("kind", "point") == "point"
        assert "zone" not in fields and "polygon" not in fields, op


@pytest.mark.parametrize("failure", ["list", "request"])
def test_a_poi_store_failure_is_logged_and_returns_none(
    marker: AutoPoiMarker, store: FakePoiStore, failure: str, caplog: pytest.LogCaptureFixture
) -> None:
    if failure == "list":
        store.list_error = RobotError("poi_store has not published /poi/list yet")
    else:
        store.request_error = PoiStoreDown("poi_store is not running")
    with caplog.at_level(logging.WARNING, logger="mcp_server.auto_poi"):
        assert marker.on_event(event(), pose()) is None
    assert "auto POI" in caplog.text


# --- config ---------------------------------------------------------------------------------------------------------


def test_auto_poi_settings_defaults_and_strictness() -> None:
    cfg = McpServerConfig()
    assert cfg.auto_poi.enabled is True
    assert cfg.auto_poi.merge_radius_m == 0.5
    assert cfg.auto_poi.max_pose_age_s == 2.0
    assert cfg.auto_poi.event_types == ["collision_stop", "nav_abort"]
    with pytest.raises(ValidationError):
        AutoPoiSettings(unknown=1)
    with pytest.raises(ValidationError):
        AutoPoiSettings(merge_radius_m=0.0)


# --- monitor fan-out and digest -------------------------------------------------------------------------------------


def make_monitor() -> RobotMonitor:
    return RobotMonitor(MonitorSettings(), None, cpu_temp_reader=lambda: 48.0, throttled_reader=lambda: 0)


def test_monitor_fans_events_out_to_every_sink_even_when_one_fails() -> None:
    monitor = make_monitor()
    seen: list[str] = []

    def broken(_event: RobotEvent) -> None:
        raise RuntimeError("publisher gone")

    monitor.set_sink(lambda e: seen.append(f"pub:{e.type}"))
    monitor.add_sink(broken)
    monitor.add_sink(lambda e: seen.append(f"marker:{e.type}"))
    monitor.report_nav_abort("navigate_to_pose", "goal aborted")
    assert seen == ["pub:nav_abort", "marker:nav_abort"]
    monitor.set_sink(None)  # set_sink replaces every sink
    monitor.report_nav_abort("navigate_to_pose", "again")
    assert len(seen) == 2


def test_nav_abort_events_are_never_debounced() -> None:
    monitor = make_monitor()
    monitor.report_nav_abort("navigate_to_pose", "first")
    monitor.report_nav_abort("move_relative", "second")
    events, _ = monitor.digest()
    assert [(e["type"], e["severity"], e["data"]["tool"]) for e in events] == [
        ("nav_abort", "warning", "navigate_to_pose"),
        ("nav_abort", "warning", "move_relative"),
    ]


def test_collision_stop_gets_auto_poi_id_in_the_digest(store: FakePoiStore) -> None:
    monitor = make_monitor()
    attach_auto_poi(monitor, AutoPoiMarker(store, AutoPoiSettings()), lambda: pose(0.5, 0.5))
    with monitor.base_motion():
        monitor.on_collision(1, "FootprintStop")
    events, _ = monitor.digest()
    [stop] = events
    assert stop["type"] == "collision_stop"
    assert stop[AUTO_POI_ID_KEY] == store.pois[0]["id"]
    assert store.pois[0]["name"] == "stuck: collision stop"


def test_events_without_a_poi_carry_no_auto_poi_id(store: FakePoiStore) -> None:
    monitor = make_monitor()
    attach_auto_poi(monitor, AutoPoiMarker(store, AutoPoiSettings()), lambda: None)  # no pose
    with monitor.base_motion():
        monitor.on_collision(1, "FootprintStop")
    events, _ = monitor.digest()
    assert AUTO_POI_ID_KEY not in events[0]
    assert store.commands == []


def test_digest_waits_for_an_in_flight_marker_delivery(store: FakePoiStore) -> None:
    """A collision delivered on a ROS thread finishes its POI write before a tool thread reads the digest."""
    monitor = make_monitor()
    gate = threading.Event()
    marker = AutoPoiMarker(store, AutoPoiSettings())

    def slow_pose() -> BasePose:
        gate.wait(2.0)
        return pose()

    attach_auto_poi(monitor, marker, slow_pose)
    with monitor.base_motion():
        ros_thread = threading.Thread(target=monitor.on_collision, args=(1, "FootprintStop"))
        ros_thread.start()
        while monitor.last_seq() == 0:
            time.sleep(0.001)
    threading.Timer(0.1, gate.set).start()
    events, _ = monitor.digest()
    ros_thread.join()
    assert events[0][AUTO_POI_ID_KEY] == store.pois[0]["id"]


# --- navigate_to_pose result hook in call_tool_with_digest ----------------------------------------------------------


class AbortingRobot(FakeRobot):
    """Nav2 aborts every goal."""

    def navigate(
        self, x: float, y: float, yaw: float, frame: str, timeout_s: float, precise: bool = False
    ) -> NavigationResult:
        self.calls.append(("navigate", (x, y, yaw, frame, timeout_s, precise)))
        return NavigationResult(status="aborted", message="robot is inside the inflation layer")


@pytest.fixture
def aborting(tmp_path: Path) -> AbortingRobot:
    return AbortingRobot(tmp_path)


def build(robot: FakeRobot, monitor: RobotMonitor) -> Any:
    cfg = McpServerConfig()
    cfg.cameras.calibration_dir = robot.arm.cfg.arm.home_file.parent / "calibration"
    return build_mcp_server(robot, cfg, TOKEN, None, monitor)


def test_navigate_to_pose_abort_becomes_a_stuck_poi_in_the_digest(aborting: AbortingRobot) -> None:
    monitor = make_monitor()
    attach_auto_poi(monitor, AutoPoiMarker(aborting, AutoPoiSettings()), aborting.robot_pose)
    server = build(aborting, monitor)
    take(server, "base")
    call(server, "get_robot_state", {})  # digest cursor
    res = call(server, "navigate_to_pose", {"x": 3.0, "y": 0.0})
    assert res.structured_content["status"] == "aborted"
    [abort] = res.structured_content[DIGEST_KEY]
    assert abort["type"] == "nav_abort"
    assert abort["data"]["tool"] == "navigate_to_pose"
    [stuck] = [p for p in aborting.pois if p["name"].startswith("stuck:")]
    assert abort[AUTO_POI_ID_KEY] == stuck["id"]
    assert (stuck["x"], stuck["y"], stuck["kind"]) == (1.0, 2.0, "point")  # the robot pose, not the goal


def test_a_succeeded_navigation_marks_nothing(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    monitor = make_monitor()
    attach_auto_poi(monitor, AutoPoiMarker(robot, AutoPoiSettings()), robot.robot_pose)
    server = build(robot, monitor)
    take(server, "base")
    call(server, "get_robot_state", {})
    res = call(server, "navigate_to_pose", {"x": 3.0, "y": 0.0})
    assert res.structured_content[DIGEST_KEY] == []
    assert robot.pois == []


def test_a_poi_store_failure_never_breaks_the_tool_result(aborting: AbortingRobot) -> None:
    aborting.poi_store_up = False
    monitor = make_monitor()
    attach_auto_poi(monitor, AutoPoiMarker(aborting, AutoPoiSettings()), aborting.robot_pose)
    server = build(aborting, monitor)
    take(server, "base")
    call(server, "get_robot_state", {})
    res = call(server, "navigate_to_pose", {"x": 3.0, "y": 0.0})
    assert res.structured_content["status"] == "aborted"
    [abort] = res.structured_content[DIGEST_KEY]
    assert abort["type"] == "nav_abort" and AUTO_POI_ID_KEY not in abort
