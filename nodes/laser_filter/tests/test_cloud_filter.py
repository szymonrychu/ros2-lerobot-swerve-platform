"""ToF cloud self filter (laser_filter.self_filter, no ROS): the per-point keep mask against the deployed self polygon
(H body + parked gripper, notches kept, height limit, non-finite removed), the runtime polygon replacement with a
released arm shadow, the cloud frame -> polygon_frame transform helper, the whole cloud filter in the cloud frame, the
newest-wins pending slot for clouds waiting for TF, the cloud config keys, and laser_filter.node.LaserFilterNode's cloud
path against ROS doubles (published with the input header only after TF at the stamp, the expiry timer never
publishes, newest wins, empty cloud passed through, scans not delayed by a cloud waiting for TF)."""

import importlib
import math
import sys
from collections.abc import Callable, Iterator
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
import shapely
from pydantic import ValidationError
from shapely.geometry import Polygon, box

from laser_filter.config import LaserFilterConfig, load_config
from laser_filter.self_filter import (
    PendingSlot,
    SelfFilter,
    apply_polygon_param,
    cloud_self_mask,
    parse_polygon,
    quaternion_matrix,
    transform_points,
)

DEPLOYED = Path(__file__).resolve().parent.parent / "config" / "footprint_filter.yaml"
GENERATED_BEGIN = "# >>> gen_nav_footprint: laser_polygon"
GENERATED_END = "# <<< gen_nav_footprint: laser_polygon"
MAX_HEIGHT_M = 1.0
SIDE_FRAME = (0.0, 0.15, 0.10)  # on the side rail of the H
PARKED_GRIPPER = (0.30, -0.05, 0.20)  # the parked arm's gripper in front of the body
FRONT_NOTCH = (0.21, 0.04, 0.10)  # front notch beside the parked gripper: a real obstacle
REAR_NOTCH = (-0.19, 0.0, 0.10)  # rear notch between the rear wheels, behind the crossbar: a real obstacle
ABOVE_BODY = (0.0, 0.15, 1.20)  # over the robot, above cloud_self_max_height_m: a real overhang
CLIFF = (1.0, 0.0, -0.30)  # below the floor in front of the robot: a cliff
ARM_POINT = (0.50, 0.0, 0.30)  # the released arm reaching forward
QUARTER = math.sin(math.pi / 8), math.cos(math.pi / 8)


def deployed_polygon() -> list[tuple[float, float]]:
    return parse_polygon(load_config(DEPLOYED).polygon)


def prepared(points: list[tuple[float, float]]) -> Polygon:
    shape = Polygon(points)
    shapely.prepare(shape)
    return shape


def polygon_string(shape: Polygon) -> str:
    return str([[round(x, 4), round(y, 4)] for x, y in list(shape.exterior.coords)[:-1]])


def head_transform() -> tuple[np.ndarray, np.ndarray]:
    """tof_optical_frame -> base_link: head 0.10 m ahead and 0.60 m up, panned 30 deg and tilted 20 deg down, optical
    axes (z forward, x right, y down)."""
    pan = quaternion_matrix(0.0, 0.0, math.sin(math.radians(15)), math.cos(math.radians(15)))
    tilt = quaternion_matrix(0.0, math.sin(math.radians(10)), 0.0, math.cos(math.radians(10)))
    optical = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    return pan @ tilt @ optical, np.array([0.10, 0.0, 0.60])


def test_points_on_the_side_frame_and_parked_gripper_are_removed() -> None:
    keep = cloud_self_mask(np.array([SIDE_FRAME, PARKED_GRIPPER]), prepared(deployed_polygon()), MAX_HEIGHT_M)
    assert keep.dtype == bool and keep.tolist() == [False, False]


def test_points_in_the_front_and_rear_notch_are_kept() -> None:
    keep = cloud_self_mask(np.array([FRONT_NOTCH, REAR_NOTCH]), prepared(deployed_polygon()), MAX_HEIGHT_M)
    assert keep.tolist() == [True, True]


def test_points_above_the_height_limit_inside_the_polygon_are_kept() -> None:
    at_limit = (SIDE_FRAME[0], SIDE_FRAME[1], MAX_HEIGHT_M)
    keep = cloud_self_mask(np.array([ABOVE_BODY, at_limit]), prepared(deployed_polygon()), MAX_HEIGHT_M)
    assert keep.tolist() == [True, False]  # z <= max_height_m is the robot, above it a real overhang


def test_below_floor_points_outside_the_polygon_are_kept() -> None:
    under_body = (SIDE_FRAME[0], SIDE_FRAME[1], -0.20)
    keep = cloud_self_mask(np.array([CLIFF, under_body]), prepared(deployed_polygon()), MAX_HEIGHT_M)
    assert keep.tolist() == [True, False]


def test_non_finite_points_are_removed() -> None:
    points = np.array([[math.nan, 1.0, 0.0], [1.0, math.inf, 0.0], [1.0, 0.0, -math.inf], [1.0, 0.0, 0.0]])
    assert cloud_self_mask(points, prepared(deployed_polygon()), MAX_HEIGHT_M).tolist() == [False, False, False, True]


def test_mask_accepts_a_vertex_list_and_an_empty_cloud() -> None:
    assert cloud_self_mask(np.array([SIDE_FRAME, CLIFF]), deployed_polygon(), MAX_HEIGHT_M).tolist() == [False, True]
    empty = cloud_self_mask(np.zeros((0, 3)), deployed_polygon(), MAX_HEIGHT_M)
    assert empty.shape == (0,) and empty.dtype == bool


def test_replaced_runtime_polygon_with_a_released_arm_shadow_removes_arm_points() -> None:
    """mcp_server sets the dynamic footprint (static outline + released arm shadow) through the `polygon` parameter."""
    flt = SelfFilter(deployed_polygon())
    points = np.array([ARM_POINT, FRONT_NOTCH, CLIFF])
    assert cloud_self_mask(points, flt.base_polygon, MAX_HEIGHT_M).tolist() == [True, True, True]
    dynamic = Polygon(deployed_polygon()).union(box(0.15, -0.06, 0.62, 0.06)).simplify(0.0)
    ok, outcome = apply_polygon_param(flt, polygon_string(dynamic))
    assert ok and outcome == "replaced"
    assert cloud_self_mask(points, flt.base_polygon, MAX_HEIGHT_M).tolist() == [False, False, True]
    ok, _ = apply_polygon_param(flt, polygon_string(Polygon(deployed_polygon())))
    assert ok  # the arm parked again: the static outline is restored and the notch is an obstacle again
    assert cloud_self_mask(points, flt.base_polygon, MAX_HEIGHT_M).tolist() == [True, True, True]


def test_transform_helper_round_trips_a_yawed_and_tilted_transform() -> None:
    rotation = quaternion_matrix(0.0, 0.0, *QUARTER) @ quaternion_matrix(QUARTER[0], 0.0, 0.0, QUARTER[1])
    translation = np.array([0.1, -0.2, 0.6])
    points = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.3, -0.4, 2.0]])
    moved = transform_points(points, rotation, translation)
    # yaw 45 deg after roll 45 deg: x stays a pure yaw, y is rolled up first
    np.testing.assert_allclose(moved[0], [math.sqrt(0.5) + 0.1, math.sqrt(0.5) - 0.2, 0.6], atol=1e-12)
    np.testing.assert_allclose(moved[1], [-0.5 + 0.1, 0.5 - 0.2, math.sqrt(0.5) + 0.6], atol=1e-12)
    back = transform_points(moved, rotation.T, -rotation.T @ translation)
    np.testing.assert_allclose(back, points, atol=1e-12)
    assert transform_points(np.zeros((0, 3)), rotation, translation).shape == (0, 3)


def test_filter_cloud_keeps_obstacles_in_the_cloud_frame() -> None:
    """The node's per-cloud path: cloud frame points -> polygon_frame -> mask -> the kept points as given (float32)."""
    rotation, translation = head_transform()
    base = np.array([SIDE_FRAME, FRONT_NOTCH, PARKED_GRIPPER, CLIFF, ABOVE_BODY, REAR_NOTCH])
    cloud = transform_points(base, rotation.T, -rotation.T @ translation).astype(np.float32)
    cloud = np.vstack([cloud, [[math.nan, math.nan, math.nan]]]).astype(np.float32)
    kept = SelfFilter(deployed_polygon()).filter_cloud(cloud, rotation, translation, MAX_HEIGHT_M)
    assert kept.dtype == np.float32
    np.testing.assert_array_equal(kept, cloud[[1, 3, 4, 5]])  # same frame: points are not moved


def test_filter_cloud_passes_an_empty_cloud_through_as_empty() -> None:
    rotation, translation = head_transform()
    kept = SelfFilter(deployed_polygon()).filter_cloud(np.zeros((0, 3), np.float32), rotation, translation, 1.0)
    assert kept.shape == (0, 3) and kept.dtype == np.float32


def test_pending_slot_is_newest_wins_and_counts_drops() -> None:
    slot: PendingSlot[str] = PendingSlot(timeout_s=0.2)
    assert slot.item is None and slot.dropped == 0
    assert slot.put("a", now_s=10.0) is None
    assert slot.put("b", now_s=10.05) == "a"  # superseded by the newer cloud: dropped
    assert slot.dropped == 1 and slot.item == "b"
    assert slot.expire(now_s=10.2) is None  # 0.15 s old: still waiting for TF
    assert slot.take() == "b" and slot.item is None and slot.take() is None
    assert slot.dropped == 1  # taken (published), not dropped


def test_pending_slot_drops_a_cloud_after_the_timeout() -> None:
    slot: PendingSlot[str] = PendingSlot(timeout_s=0.2)
    slot.put("a", now_s=10.0)
    assert slot.expire(now_s=10.21) == "a"
    assert slot.item is None and slot.dropped == 1
    assert slot.expire(now_s=11.0) is None and slot.dropped == 1


def test_cloud_config_defaults() -> None:
    cfg = LaserFilterConfig(polygon="[[0, 0], [1, 0], [1, 1]]")
    assert cfg.cloud_topic == "/tof/points"
    assert cfg.cloud_filtered_topic == "/tof/points_filtered"
    assert cfg.cloud_self_max_height_m == 1.0
    assert cfg.cloud_transform_timeout_s == 0.2
    assert LaserFilterConfig(polygon="[[0, 0], [1, 0], [1, 1]]", cloud_topic=None).cloud_topic is None


@pytest.mark.parametrize(
    "extra",
    [
        {"cloud_self_max_height_m": 0.0},
        {"cloud_self_max_height_m": -1.0},
        {"cloud_transform_timeout_s": 0.0},
        {"cloud_frame": "base_link"},
        {"cloud_polygon": "[[0, 0], [1, 0], [1, 1]]"},
    ],
)
def test_cloud_config_rejects_bad_values_and_unknown_keys(extra: dict[str, object]) -> None:
    with pytest.raises(ValidationError):
        LaserFilterConfig(polygon="[[0, 0], [1, 0], [1, 1]]", **extra)


def test_deployed_config_sets_the_cloud_keys_outside_the_generated_region() -> None:
    text = DEPLOYED.read_text()
    outside = text.split(GENERATED_BEGIN)[0] + text.split(GENERATED_END)[1]
    for line in (
        "cloud_topic: /tof/points",
        "cloud_filtered_topic: /tof/points_filtered",
        "cloud_self_max_height_m: 1.0",
        "cloud_transform_timeout_s: 0.2",
    ):
        assert f"\n{line}\n" in outside, line
    cfg = load_config(DEPLOYED)
    assert (cfg.cloud_topic, cfg.cloud_filtered_topic) == ("/tof/points", "/tof/points_filtered")


def test_pending_slot_drop_counts_the_waiting_cloud() -> None:
    """A newer cloud got its TF first (or the TF lookup failed): the waiting one is dropped and counted."""
    slot: PendingSlot[str] = PendingSlot(timeout_s=0.2)
    assert slot.drop() is None and slot.dropped == 0
    slot.put("a", now_s=10.0)
    assert slot.drop() == "a"
    assert slot.item is None and slot.dropped == 1


# --- laser_filter.node.LaserFilterNode cloud path against ROS doubles (rclpy is not installed on dev machines) ---

ROS_MODULES = (
    "rclpy",
    "rclpy.node",
    "rclpy.duration",
    "rclpy.parameter",
    "rclpy.qos",
    "rclpy.task",
    "rclpy.time",
    "geometry_msgs",
    "geometry_msgs.msg",
    "rcl_interfaces",
    "rcl_interfaces.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "sensor_msgs_py",
    "sensor_msgs_py.point_cloud2",
    "tf2_ros",
)
CLOUD_FRAME = "tof_optical_frame"
LASER_FRAME = "laser"
CLOUD_STAMP = (100, 250_000_000)


class FakeTime:
    """rclpy.time.Time double: nanoseconds since the epoch, built from a builtin_interfaces/Time-like stamp."""

    def __init__(self, nanoseconds: int = 0) -> None:
        self.nanoseconds = nanoseconds

    @classmethod
    def from_msg(cls, stamp: SimpleNamespace) -> "FakeTime":
        return cls(stamp.sec * 1_000_000_000 + stamp.nanosec)


class FakeFuture:
    """rclpy.task.Future double: result, exception, cancel and done callbacks."""

    def __init__(self) -> None:
        self.value: object = None
        self.finished = False
        self.was_cancelled = False
        self.callbacks: list[Callable[[FakeFuture], None]] = []

    def done(self) -> bool:
        return self.finished

    def cancelled(self) -> bool:
        return self.was_cancelled

    def cancel(self) -> None:
        self.was_cancelled = True

    def result(self) -> object:
        return self.value

    def exception(self) -> BaseException | None:
        return None

    def set_result(self, value: object) -> None:
        self.value, self.finished = value, True
        for callback in self.callbacks:
            callback(self)

    def add_done_callback(self, callback: Callable[["FakeFuture"], None]) -> None:
        self.callbacks.append(callback)


class FakeTransformException(Exception):
    """tf2_ros.TransformException double."""


class FakeBuffer:
    """tf2_ros.Buffer double: transforms known per (target, source, stamp ns); a future waits until arrive()."""

    def __init__(self) -> None:
        self.known: dict[tuple[str, str, int], SimpleNamespace] = {}
        self.waiting: list[tuple[tuple[str, str, int], FakeFuture]] = []
        self.static: dict[tuple[str, str], SimpleNamespace] = {}

    def wait_for_transform_async(self, target: str, source: str, time_: FakeTime) -> FakeFuture:
        future = FakeFuture()
        key = (target, source, time_.nanoseconds)
        if key in self.known:
            future.set_result(self.known[key])
        else:
            self.waiting.append((key, future))
        return future

    def arrive(self, target: str, source: str, stamp: tuple[int, int], tf: SimpleNamespace) -> None:
        """The TF listener received this transform: complete the futures waiting for it."""
        key = (target, source, stamp[0] * 1_000_000_000 + stamp[1])
        self.known[key] = tf
        for waiting_key, future in list(self.waiting):
            if waiting_key == key:
                self.waiting.remove((waiting_key, future))
                future.set_result(tf)

    def lookup_transform(self, target: str, source: str, *_args: object, **_kwargs: object) -> SimpleNamespace:
        if (target, source) not in self.static:
            raise FakeTransformException(f"{source} -> {target} unknown")
        return self.static[(target, source)]


class FakeLogger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def info(self, *_args: object, **_kwargs: object) -> None:
        pass

    def warning(self, text: str, **_kwargs: object) -> None:
        self.warnings.append(text)


class FakeNode:
    """rclpy.node.Node double recording publishers, subscriptions and timers by topic."""

    def __init__(self, _name: str) -> None:
        self.published: dict[str, list[object]] = {}
        self.callbacks: dict[str, Callable[[object], None]] = {}
        self.timers: list[Callable[[], None]] = []
        self.logger = FakeLogger()

    def declare_parameter(self, *_args: object, **_kwargs: object) -> None:
        pass

    def add_on_set_parameters_callback(self, *_args: object, **_kwargs: object) -> None:
        pass

    def create_publisher(self, _msg_type: object, topic: str, _qos: object) -> SimpleNamespace:
        self.published[topic] = []
        return SimpleNamespace(publish=self.published[topic].append)

    def create_subscription(self, _msg_type: object, topic: str, callback: Callable[[object], None], _qos: object):
        self.callbacks[topic] = callback

    def create_timer(self, _period_s: float, callback: Callable[[], None]) -> None:
        self.timers.append(callback)

    def get_logger(self) -> FakeLogger:
        return self.logger


def fake_read_points_numpy(msg: SimpleNamespace, field_names: tuple[str, ...], skip_nans: bool) -> np.ndarray:
    assert field_names == ("x", "y", "z") and not skip_nans
    return msg.points


def fake_create_cloud_xyz32(header: SimpleNamespace, points: np.ndarray) -> SimpleNamespace:
    return SimpleNamespace(header=header, points=np.asarray(points, np.float32), is_dense=False)


def stamp_msg(stamp: tuple[int, int]) -> SimpleNamespace:
    return SimpleNamespace(sec=stamp[0], nanosec=stamp[1])


def cloud_msg(points: np.ndarray, stamp: tuple[int, int] = CLOUD_STAMP) -> SimpleNamespace:
    return SimpleNamespace(header=SimpleNamespace(frame_id=CLOUD_FRAME, stamp=stamp_msg(stamp)), points=points)


def transform_msg(rotation: np.ndarray, translation: np.ndarray) -> SimpleNamespace:
    """TransformStamped-like message (rotation matrix -> unit quaternion) for the TF doubles."""
    w = math.sqrt(max(0.0, 1.0 + rotation[0, 0] + rotation[1, 1] + rotation[2, 2])) / 2.0
    x = math.copysign(
        math.sqrt(max(0.0, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) / 2.0,
        rotation[2, 1] - rotation[1, 2],
    )
    y = math.copysign(
        math.sqrt(max(0.0, 1.0 - rotation[0, 0] + rotation[1, 1] - rotation[2, 2])) / 2.0,
        rotation[0, 2] - rotation[2, 0],
    )
    z = math.copysign(
        math.sqrt(max(0.0, 1.0 - rotation[0, 0] - rotation[1, 1] + rotation[2, 2])) / 2.0,
        rotation[1, 0] - rotation[0, 1],
    )
    t = SimpleNamespace(x=float(translation[0]), y=float(translation[1]), z=float(translation[2]))
    return SimpleNamespace(transform=SimpleNamespace(rotation=SimpleNamespace(x=x, y=y, z=z, w=w), translation=t))


@pytest.fixture
def node_module(monkeypatch: pytest.MonkeyPatch) -> Iterator[ModuleType]:
    """laser_filter.node imported against the ROS doubles (removed again afterwards)."""
    for name in ROS_MODULES:
        monkeypatch.setitem(sys.modules, name, MagicMock(name=name))
    sys.modules["rclpy.node"].Node = FakeNode
    sys.modules["rclpy.time"].Time = FakeTime
    sys.modules["tf2_ros"].Buffer = FakeBuffer
    sys.modules["tf2_ros"].TransformException = FakeTransformException
    point_cloud2 = sys.modules["sensor_msgs_py"].point_cloud2
    point_cloud2.read_points_numpy = fake_read_points_numpy
    point_cloud2.create_cloud_xyz32 = fake_create_cloud_xyz32
    sys.modules.pop("laser_filter.node", None)
    yield importlib.import_module("laser_filter.node")
    sys.modules.pop("laser_filter.node", None)


def make_node(node_module: ModuleType) -> tuple[ModuleType, FakeBuffer]:
    node = node_module.LaserFilterNode(load_config(DEPLOYED))
    return node, node.tf_buffer


def head_cloud() -> tuple[np.ndarray, SimpleNamespace]:
    """Side frame, front notch, parked gripper, cliff in the cloud frame and the head TF that maps them to base_link."""
    rotation, translation = head_transform()
    base = np.array([SIDE_FRAME, FRONT_NOTCH, PARKED_GRIPPER, CLIFF])
    cloud = transform_points(base, rotation.T, -rotation.T @ translation).astype(np.float32)
    return cloud, transform_msg(rotation, translation)


def test_laser_filter_node_publishes_kept_points_with_the_input_header_only_after_tf_at_the_stamp(
    node_module: ModuleType,
) -> None:
    node, tf_buffer = make_node(node_module)
    points, tf = head_cloud()
    msg = cloud_msg(points)
    node.callbacks["/tof/points"](msg)
    assert node.published["/tof/points_filtered"] == []  # no TF at the stamp yet: nothing published
    tf_buffer.arrive("base_link", CLOUD_FRAME, (CLOUD_STAMP[0], CLOUD_STAMP[1] - 1), tf)  # TF at another stamp
    for timer in node.timers:
        timer()
    assert node.published["/tof/points_filtered"] == []
    tf_buffer.arrive("base_link", CLOUD_FRAME, CLOUD_STAMP, tf)
    (out,) = node.published["/tof/points_filtered"]
    assert out.header is msg.header  # same frame, same stamp
    assert out.points.dtype == np.float32 and out.is_dense
    np.testing.assert_allclose(out.points, points[[1, 3]], atol=1e-6)  # side frame and parked gripper removed


def test_laser_filter_node_timer_never_publishes_and_drops_a_cloud_without_tf(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    node, tf_buffer = make_node(node_module)
    points, tf = head_cloud()
    clock = [1000.0]
    monkeypatch.setattr(node_module.time, "monotonic", lambda: clock[0])
    node.callbacks["/tof/points"](cloud_msg(points))
    clock[0] += 0.1
    for timer in node.timers:
        timer()
    assert node.cloud_slot.item is not None and node.cloud_slot.dropped == 0  # still waiting
    clock[0] += 0.15  # past cloud_transform_timeout_s (0.2 s)
    for timer in node.timers:
        timer()
    assert node.cloud_slot.item is None and node.cloud_slot.dropped == 1
    assert any("no TF within 0.2 s" in text and "1 clouds dropped" in text for text in node.logger.warnings)
    tf_buffer.arrive("base_link", CLOUD_FRAME, CLOUD_STAMP, tf)  # too late: the dropped cloud stays dropped
    assert node.published["/tof/points_filtered"] == []


def test_laser_filter_node_pending_cloud_is_newest_wins(node_module: ModuleType) -> None:
    node, tf_buffer = make_node(node_module)
    points, tf = head_cloud()
    later = (CLOUD_STAMP[0], CLOUD_STAMP[1] + 100_000_000)
    node.callbacks["/tof/points"](cloud_msg(points))
    newer = cloud_msg(points, later)
    node.callbacks["/tof/points"](newer)
    assert node.cloud_slot.dropped == 1
    tf_buffer.arrive("base_link", CLOUD_FRAME, CLOUD_STAMP, tf)  # the superseded cloud's TF: not published
    assert node.published["/tof/points_filtered"] == []
    tf_buffer.arrive("base_link", CLOUD_FRAME, later, tf)
    (out,) = node.published["/tof/points_filtered"]
    assert out.header is newer.header


def test_laser_filter_node_passes_an_empty_cloud_through_with_its_header(node_module: ModuleType) -> None:
    node, tf_buffer = make_node(node_module)
    _, tf = head_cloud()
    tf_buffer.arrive("base_link", CLOUD_FRAME, CLOUD_STAMP, tf)
    msg = cloud_msg(np.zeros((0, 3), np.float32))
    node.callbacks["/tof/points"](msg)
    (out,) = node.published["/tof/points_filtered"]
    assert out.header is msg.header and out.points.shape == (0, 3)


def test_laser_filter_node_scan_is_not_delayed_by_a_cloud_waiting_for_tf(node_module: ModuleType) -> None:
    node, tf_buffer = make_node(node_module)
    points, _ = head_cloud()
    tf_buffer.static[(LASER_FRAME, "base_link")] = transform_msg(np.eye(3), np.zeros(3))
    node.callbacks["/tof/points"](cloud_msg(points))  # waits for its TF in the pending slot
    scan = SimpleNamespace(
        header=SimpleNamespace(frame_id=LASER_FRAME, stamp=stamp_msg(CLOUD_STAMP)),
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=math.pi / 2,
        time_increment=0.0,
        scan_time=0.1,
        range_min=0.05,
        range_max=12.0,
        ranges=[0.05, 3.0, 3.0, 3.0],
        intensities=[],
    )
    node.callbacks["/scan"](scan)
    (out,) = node.published["/scan_filtered"]
    assert out.header is scan.header
    assert node.cloud_slot.item is not None and node.published["/tof/points_filtered"] == []
