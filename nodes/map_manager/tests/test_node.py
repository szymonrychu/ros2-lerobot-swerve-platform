"""Tests for the rclpy layer of map_manager with the ROS packages stubbed (no ROS in the dev environment)."""

from __future__ import annotations

import json
import math
import threading
import time
import tomllib
from pathlib import Path

import pytest
from prometheus_client import REGISTRY
from ros2_common.maps import COMMAND_TOPIC, LOCALISED_TOPIC, RESULT_TOPIC, STATUS_TOPIC, MapResult, MapStatus, Pose2D

from map_manager.config import MapManagerConfig
from map_manager.node import MapManagerNode, grid_from_msg, yaw_from_quaternion
from map_manager.relocalise import SearchSettings
from tests.fakes import room_grid
from tests.ros_stubs import (
    LASER_FRAME,
    FakeBuffer,
    FakeNode,
    FakeTransformListener,
    Msg,
    quat_yaw,
    transform_stamped,
    vec,
)

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
NOW_NS = 1_000_000_000_000
HEAVY_TOPICS = {"/map", "/scan_filtered", "/pose"}
IDLE_TOPICS = {COMMAND_TOPIC, "/poi/list", "/poi/result"}
POI_LIST = {"pois": [{"id": "p1", "name": "sofa"}], "revision": 4}
SERVICES = {
    "/slam_toolbox/serialize_map",
    "/slam_toolbox/deserialize_map",
    "/slam_toolbox/pause_new_measurements",
    "/slam_toolbox/get_parameters",
}


def string_msg(payload: object) -> Msg:
    """std_msgs/String carrying JSON (or raw text)."""
    return Msg(data=payload if isinstance(payload, str) else json.dumps(payload))


def grid_msg() -> Msg:
    """A /map message of a small room."""
    grid = room_grid(2.0, 2.0)
    height, width = grid.cells.shape
    info = Msg(
        width=width,
        height=height,
        resolution=grid.resolution,
        origin=Msg(position=vec(grid.origin_x, grid.origin_y), orientation=quat_yaw(0.0)),
    )
    return Msg(info=info, data=grid.cells.ravel().tolist())


def sample(name: str, labels: dict[str, str]) -> float:
    """Value of a default-registry sample (0 when absent)."""
    return REGISTRY.get_sample_value(name, labels) or 0.0


def serialize_handler(request: Msg) -> Msg:
    """slam_toolbox serialize: write the pose graph files and succeed."""
    for suffix in (".posegraph", ".data"):
        Path(request.filename + suffix).write_bytes(b"graph")
    return Msg(result=0)


@pytest.fixture(autouse=True)
def clean_stubs() -> None:
    """Reset the shared stub state around every test."""
    FakeBuffer.transforms = {}
    FakeTransformListener.live = []
    FakeNode.latched = {"/poi/list": string_msg(POI_LIST)}


@pytest.fixture
def config(tmp_path: Path) -> MapManagerConfig:
    """Config with fast waits and a temp maps directory."""
    return MapManagerConfig(maps_root=str(tmp_path / "maps"), sensor_wait_s=0.05, poi_timeout_s=0.2)


@pytest.fixture
def node(config: MapManagerConfig) -> MapManagerNode:
    """The node under test on the stubbed rclpy."""
    return MapManagerNode(config)


def join_workers(node: MapManagerNode) -> None:
    """Wait for every command worker thread."""
    for thread in list(node.workers):
        thread.join(timeout=5.0)


def results(node: MapManagerNode) -> list[MapResult]:
    """Published results."""
    return [MapResult.model_validate_json(msg.data) for msg in node.publisher_on(RESULT_TOPIC).published]


def prepare_save_world(node: MapManagerNode) -> None:
    """Sensors and slam_toolbox for a save: TF map <- base_link, the map, a scan frame, working serialize."""
    FakeBuffer.transforms[("map", "base_link")] = transform_stamped(1.0, 0.8, 0.3, NOW_NS)
    FakeNode.latched["/map"] = grid_msg()
    FakeNode.latched["/scan_filtered"] = Msg(
        header=Msg(frame_id=LASER_FRAME), ranges=[1.0], angle_min=0.0, angle_increment=0.1, range_min=0.1, range_max=8.0
    )
    node.client_of("/slam_toolbox/serialize_map").handler = serialize_handler


def test_idle_has_only_the_light_subscriptions(node: MapManagerNode) -> None:
    topics = {sub.topic for sub in node.subscriptions}
    assert topics == IDLE_TOPICS
    assert not (topics & HEAVY_TOPICS)
    assert FakeTransformListener.live == []
    assert {client.srv_name for client in node.clients} == SERVICES
    assert {pub.topic for pub in node.publishers} >= {RESULT_TOPIC, STATUS_TOPIC, LOCALISED_TOPIC, "/poi/command"}
    command_qos = node.subscription_on(COMMAND_TOPIC).qos
    assert command_qos.reliability.name == "RELIABLE"
    assert node.publisher_on(STATUS_TOPIC).qos.durability.name == "TRANSIENT_LOCAL"
    assert node.publisher_on(LOCALISED_TOPIC).qos.durability.name == "TRANSIENT_LOCAL"
    assert node.publisher_on(RESULT_TOPIC).qos.durability.name == "VOLATILE"


def test_operation_creates_and_destroys_the_heavy_subscriptions(node: MapManagerNode) -> None:
    node.sensors.start_operation()
    topics = {sub.topic for sub in node.subscriptions}
    assert HEAVY_TOPICS <= topics
    assert len(FakeTransformListener.live) == 1
    assert node.subscription_on("/map").qos.durability.name == "TRANSIENT_LOCAL"
    assert node.subscription_on("/scan_filtered").qos.reliability.name == "BEST_EFFORT"
    node.sensors.end_operation()
    assert {sub.topic for sub in node.subscriptions} == IDLE_TOPICS
    assert FakeTransformListener.live == []


def test_a_command_produces_a_result_and_a_status_update(node: MapManagerNode) -> None:
    prepare_save_world(node)
    node.on_command(string_msg({"op": "save", "request_id": "r1", "name": "kitchen"}))
    join_workers(node)
    [result] = results(node)
    assert (result.request_id, result.ok) == ("r1", True), result.message
    assert {sub.topic for sub in node.subscriptions} == IDLE_TOPICS  # heavy subscriptions gone again
    statuses = [MapStatus.model_validate_json(msg.data) for msg in node.publisher_on(STATUS_TOPIC).published]
    assert [m.name for m in statuses[-1].maps] == ["kitchen"]
    assert node.publisher_on(LOCALISED_TOPIC).published == []  # a save is not a load


def test_status_is_published_at_start(config: MapManagerConfig, tmp_path: Path) -> None:
    bundle_dir = tmp_path / "maps"
    first = MapManagerNode(config)
    prepare_save_world(first)
    first.on_command(string_msg({"op": "save", "request_id": "r1", "name": "kept"}))
    join_workers(first)
    assert bundle_dir.is_dir()
    second = MapManagerNode(config)
    [msg] = second.publisher_on(STATUS_TOPIC).published
    status = MapStatus.model_validate_json(msg.data)
    assert [m.name for m in status.maps] == ["kept"]
    assert status.load_seq == 0
    assert status.active_map is None
    assert status.localisation.state == "unknown"


def test_localised_is_not_published_at_start(node: MapManagerNode) -> None:
    assert node.publisher_on(LOCALISED_TOPIC).published == []
    assert len(node.publisher_on(STATUS_TOPIC).published) == 1


@pytest.mark.parametrize(
    "payload",
    ["not json", '{"op": "explode"}', '{"op": "save", "request_id": "r9", "name": "bad name"}', "[1, 2]"],
)
def test_invalid_commands_answer_ok_false(node: MapManagerNode, payload: str) -> None:
    before = sample("map_manager_operations_total", {"op": "invalid", "ok": "false"})
    node.on_command(string_msg(payload))
    join_workers(node)
    [result] = results(node)
    assert result.ok is False
    assert result.message
    assert sample("map_manager_operations_total", {"op": "invalid", "ok": "false"}) == before + 1
    if "r9" in payload:
        assert result.request_id == "r9"


def test_numpy_is_declared_for_the_sensor_msgs_import() -> None:
    declared = tomllib.loads((PACKAGE_ROOT / "pyproject.toml").read_text())["project"]["dependencies"]
    assert any(dep.split(">")[0].split("=")[0].strip() == "numpy" for dep in declared)
    source = (PACKAGE_ROOT / "map_manager" / "node.py").read_text()
    assert "from sensor_msgs.msg import LaserScan" in source


def test_pose_timer_records_one_tf_lookup_and_nothing_when_tf_is_missing(node: MapManagerNode) -> None:
    [timer] = node.timers
    assert timer.period == 1.0
    timer.callback()  # idle: no listener, no TF
    assert not node.manager.pose_ring
    node.sensors.start_operation()
    timer.callback()  # listener alive but the transform is missing
    assert not node.manager.pose_ring
    FakeBuffer.transforms[("map", "base_link")] = transform_stamped(2.0, 1.0, 0.0, NOW_NS)
    timer.callback()
    [(_, pose)] = list(node.manager.pose_ring)
    assert (pose.x, pose.y) == (2.0, 1.0)
    node.sensors.end_operation()


def test_pose_older_than_max_age_is_not_recorded(node: MapManagerNode) -> None:
    node.sensors.start_operation()
    stale = NOW_NS - int((node.config.max_pose_age_s + 1.0) * 1e9)
    FakeBuffer.transforms[("map", "base_link")] = transform_stamped(2.0, 1.0, 0.0, stale)
    node.timers[0].callback()
    assert not node.manager.pose_ring
    node.sensors.end_operation()


def test_relocalise_is_routed_to_the_manager_and_answered_with_the_request_id(node: MapManagerNode) -> None:
    node.on_command(string_msg({"op": "relocalise", "request_id": "map_guard-7", "discard_on_fail": True}))
    join_workers(node)
    [result] = results(node)
    assert result.request_id == "map_guard-7"
    assert result.ok is False
    assert "no last known pose" in result.message


def test_a_relocalise_failure_is_counted(node: MapManagerNode) -> None:
    before = sample("map_manager_operations_total", {"op": "relocalise", "ok": "false"})
    node.on_command(string_msg({"op": "relocalise", "request_id": "x"}))
    join_workers(node)
    assert sample("map_manager_operations_total", {"op": "relocalise", "ok": "false"}) == before + 1


def test_save_is_counted_as_ok(node: MapManagerNode) -> None:
    before = sample("map_manager_operations_total", {"op": "save", "ok": "true"})
    prepare_save_world(node)
    node.on_command(string_msg({"op": "save", "request_id": "m", "name": "counted"}))
    join_workers(node)
    assert sample("map_manager_operations_total", {"op": "save", "ok": "true"}) == before + 1


def test_cleanup_temp_runs_at_start(config: MapManagerConfig) -> None:
    root = Path(config.maps_root)
    (root / ".tmp-half-1").mkdir(parents=True)
    (root / "real").mkdir()
    MapManagerNode(config)
    assert not (root / ".tmp-half-1").exists()
    assert (root / "real").exists()


def test_poi_replace_sends_the_op_and_matches_the_result_by_request_id(node: MapManagerNode) -> None:
    command_pub = node.publisher_on("/poi/command")

    def answer(msg: Msg) -> None:
        sent = json.loads(msg.data)
        node.subscription_on("/poi/result").callback(
            string_msg({"request_id": "someone-else", "ok": False, "message": "noise", "poi": None})
        )
        node.subscription_on("/poi/result").callback(
            string_msg({"request_id": sent["request_id"], "ok": True, "message": "replaced 1", "poi": None})
        )

    command_pub.on_publish = answer
    ok, message = node.poi.replace([{"id": "a", "name": "x"}])
    assert (ok, message) == (True, "replaced 1")
    sent = json.loads(command_pub.published[0].data)
    assert sent["op"] == "replace"
    assert sent["pois"] == [{"id": "a", "name": "x"}]
    assert sent["request_id"]


def test_poi_replace_times_out_without_an_answer(node: MapManagerNode) -> None:
    ok, message = node.poi.replace([])
    assert ok is False
    assert "no answer" in message


def test_poi_list_is_parsed_from_the_latched_topic(node: MapManagerNode) -> None:
    assert node.poi.latest_list() == (POI_LIST["pois"], 4)


def test_serialize_and_deserialize_requests(node: MapManagerNode) -> None:
    serialize = node.client_of("/slam_toolbox/serialize_map")
    deserialize = node.client_of("/slam_toolbox/deserialize_map")
    serialize.handler = lambda request: Msg(result=0)
    deserialize.handler = lambda request: Msg()
    assert node.slam.serialize("/x/map") is True
    assert node.slam.deserialize("/x/map", Pose2D(x=1.0, y=2.0, yaw=0.5)) is True
    request = deserialize.requests[0]
    assert (request.filename, request.match_type) == ("/x/map", 2)
    assert (request.initial_pose.x, request.initial_pose.y, request.initial_pose.theta) == (1.0, 2.0, 0.5)
    serialize.handler = lambda request: Msg(result=255)
    assert node.slam.serialize("/x/map") is False
    deserialize.available = False
    assert node.slam.deserialize("/x/map", Pose2D(x=0.0, y=0.0, yaw=0.0)) is False


def test_pause_toggle_and_parameter_read(node: MapManagerNode) -> None:
    node.client_of("/slam_toolbox/pause_new_measurements").handler = lambda request: Msg(status=True)
    node.client_of("/slam_toolbox/get_parameters").handler = lambda request: Msg(values=[Msg(type=1, bool_value=True)])
    assert node.slam.toggle_pause() is True
    assert node.slam.query_paused() is True
    node.client_of("/slam_toolbox/get_parameters").handler = lambda request: None
    assert node.slam.query_paused() is None


def test_wait_pose_after_confirms_only_a_pose_received_after_the_stamp(node: MapManagerNode) -> None:
    node.sensors.start_operation()
    covariance = [0.1, 0, 0, 0, 0, 0, 0, 0.2] + [0.0] * 27 + [0.3]
    pose_msg = Msg(pose=Msg(pose=Msg(position=vec(1.0, 2.0), orientation=quat_yaw(0.4)), covariance=covariance))
    assert node.slam.pose_covariance() is None
    assert node.slam.wait_pose_after(time.time(), 0.05) is None
    node.subscription_on("/pose").callback(pose_msg)
    assert node.slam.wait_pose_after(time.time() + 1000.0, 0.05) is None
    pose = node.slam.wait_pose_after(0.0, 0.05)
    assert pose is not None
    assert (pose.x, pose.y) == (1.0, 2.0)
    assert pose.yaw == pytest.approx(0.4)
    assert node.slam.pose_covariance() == [0.1, 0.2, 0.3]
    node.sensors.end_operation()


def feed_scans(node: MapManagerNode, scan: Msg, count: int) -> threading.Thread:
    """Deliver the scan on /scan_filtered from another thread, like the executor would."""

    def run() -> None:
        for _ in range(count):
            time.sleep(0.05)
            node.subscription_on("/scan_filtered").callback(scan)

    thread = threading.Thread(target=run)
    thread.start()
    return thread


def room_scan(range_m: float) -> Msg:
    """Five beams (-pi .. pi) all of one range, from the laser at the room centre."""
    return Msg(
        header=Msg(frame_id=LASER_FRAME),
        ranges=[range_m] * 5,
        angle_min=-math.pi,
        angle_increment=math.pi / 2,
        range_min=0.1,
        range_max=8.0,
    )


def test_agreement_samples_come_from_scans_through_tf_against_the_map(config: MapManagerConfig) -> None:
    node = MapManagerNode(config.model_copy(update={"agreement_hz": 100.0, "search": SearchSettings(min_points=4)}))
    FakeBuffer.transforms[("map", LASER_FRAME)] = transform_stamped(1.0, 1.0, 0.0, NOW_NS)
    FakeNode.latched["/map"] = grid_msg()
    node.sensors.start_operation()
    feeder = feed_scans(node, room_scan(1.02), 4)  # the endpoints land on the room walls
    samples = node.sensors.agreement_samples(0.5)
    feeder.join()
    assert len(samples) >= 2
    assert all(value == pytest.approx(1.0) for value in samples)
    feeder = feed_scans(node, room_scan(0.5), 4)  # endpoints in free space: a mismatch
    mismatched = node.sensors.agreement_samples(0.5)
    feeder.join()
    assert mismatched
    assert all(value == pytest.approx(0.0) for value in mismatched)
    node.sensors.end_operation()


def test_scans_are_ignored_outside_a_sample_window(node: MapManagerNode) -> None:
    FakeBuffer.transforms[("map", LASER_FRAME)] = transform_stamped(1.0, 1.0, 0.0, NOW_NS)
    FakeNode.latched["/map"] = grid_msg()
    node.sensors.start_operation()
    node.subscription_on("/scan_filtered").callback(room_scan(1.02))
    assert node.sensors.samples is None
    node.sensors.end_operation()


def test_scan_points_base_and_laser_frame(node: MapManagerNode) -> None:
    FakeBuffer.transforms[("base_link", LASER_FRAME)] = transform_stamped(0.1, 0.0, 0.0, NOW_NS)
    FakeNode.latched["/scan_filtered"] = room_scan(1.0)
    node.sensors.start_operation()
    assert node.sensors.laser_frame() == LASER_FRAME
    points = node.sensors.scan_points_base()
    assert points is not None
    assert points.shape == (5, 2)
    assert points[2] == pytest.approx([1.1, 0.0])  # beam at angle 0, shifted by the laser mount
    node.sensors.end_operation()
    assert node.sensors.laser_frame() is None


def test_grid_and_yaw_conversions() -> None:
    grid = grid_from_msg(grid_msg())
    assert grid.cells.shape == (grid.cells.shape[0], grid.cells.shape[1])
    assert grid.origin_yaw == 0.0
    assert yaw_from_quaternion(quat_yaw(0.7)) == pytest.approx(0.7)


def test_main_exits_78_on_a_config_error(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    from map_manager.__main__ import main

    bad = tmp_path / "config.yaml"
    bad.write_text("maps_root: ''\nunknown_key: 1\n")
    monkeypatch.setenv("MAP_MANAGER_CONFIG", str(bad))
    assert main() == 78
    monkeypatch.setenv("MAP_MANAGER_CONFIG", str(tmp_path / "missing.yaml"))
    assert main() == 78
