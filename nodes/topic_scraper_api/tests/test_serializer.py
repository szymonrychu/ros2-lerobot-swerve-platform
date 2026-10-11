import json
import tracemalloc
from array import array
from dataclasses import dataclass

from topic_scraper_api.serializer import message_to_payload, ros_message_to_builtin, ros_time_to_ns


@dataclass
class Stamp:
    sec: int
    nanosec: int


class FakeHeader:
    def __init__(self, stamp: Stamp) -> None:
        self.stamp = stamp
        self.frame_id = "x"

    def get_fields_and_field_types(self) -> dict[str, str]:
        return {"stamp": "builtin_interfaces/Time", "frame_id": "string"}


class FakeJointState:
    def __init__(self) -> None:
        self.header = FakeHeader(Stamp(sec=1, nanosec=2))
        self.name = ["joint_5", "joint_6"]
        self.position = [0.1, 0.2]

    def get_fields_and_field_types(self) -> dict[str, str]:
        return {"header": "std_msgs/Header", "name": "string[]", "position": "double[]"}


def test_ros_time_to_ns() -> None:
    assert ros_time_to_ns(Stamp(sec=5, nanosec=7)) == 5_000_000_007


def test_ros_message_to_builtin_nested() -> None:
    msg = FakeJointState()
    data = ros_message_to_builtin(msg)
    assert data["header"]["frame_id"] == "x"
    assert data["name"] == ["joint_5", "joint_6"]
    assert data["position"] == [0.1, 0.2]


def test_ros_message_to_builtin_array_values() -> None:
    data = ros_message_to_builtin(array("d", [0.1, 0.2, 0.3]))
    assert data == [0.1, 0.2, 0.3]


class FakeRawImage:
    def __init__(self, height: int = 480, width: int = 640) -> None:
        self.header = FakeHeader(Stamp(sec=1, nanosec=2))
        self.height = height
        self.width = width
        self.encoding = "bgr8"
        self.is_bigendian = 0
        self.step = width * 3
        self.data = array("B", bytes(height * width * 3))

    def get_fields_and_field_types(self) -> dict[str, str]:
        return {
            "header": "std_msgs/Header",
            "height": "uint32",
            "width": "uint32",
            "encoding": "string",
            "is_bigendian": "uint8",
            "step": "uint32",
            "data": "uint8[]",
        }


def test_image_payload_omits_data_but_keeps_other_fields() -> None:
    out = message_to_payload(FakeRawImage(), "sensor_msgs/msg/Image")
    assert not isinstance(out["data"], list)
    assert out["data"] == {"omitted": True, "byte_length": 640 * 480 * 3}
    assert out["encoding"] == "bgr8"
    assert out["width"] == 640
    assert out["height"] == 480
    assert out["header"]["frame_id"] == "x"
    assert json.dumps(out)


def test_compressed_and_pointcloud_payload_omit_data() -> None:
    for type_name in ("sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/PointCloud2"):
        msg = FakeRawImage(2, 2)
        out = message_to_payload(msg, type_name)
        assert out["data"] == {"omitted": True, "byte_length": 12}


def test_other_types_unchanged_by_message_to_payload() -> None:
    out = message_to_payload(FakeJointState(), "sensor_msgs/msg/JointState")
    assert out == ros_message_to_builtin(FakeJointState())


def test_image_payload_peak_memory_is_bounded() -> None:
    frame_bytes = 640 * 480 * 3
    msgs = [FakeRawImage() for _ in range(4)]
    tracemalloc.start()
    base = tracemalloc.get_traced_memory()[0]
    tracemalloc.reset_peak()
    for msg in msgs:
        message_to_payload(msg, "sensor_msgs/msg/Image")
    peak = tracemalloc.get_traced_memory()[1] - base
    tracemalloc.stop()
    assert peak < frame_bytes // 4


def test_non_finite_floats_become_none_and_output_is_strict_json() -> None:
    msg = {"effort": [float("nan"), float("inf"), -float("inf"), 1.5], "x": float("nan")}
    data = ros_message_to_builtin(msg)
    assert data == {"effort": [None, None, None, 1.5], "x": None}
    json.dumps(data, allow_nan=False)


def test_non_finite_floats_in_array_become_none() -> None:
    assert ros_message_to_builtin(array("d", [float("nan"), 2.0])) == [None, 2.0]


def test_message_to_payload_sanitizes_binary_type_fields() -> None:
    class Img:
        height = float("nan")
        data = b"abc"

        def get_fields_and_field_types(self) -> dict[str, str]:
            return {"height": "uint32", "data": "uint8[]"}

    out = message_to_payload(Img(), "sensor_msgs/msg/Image")
    assert out == {"height": None, "data": {"omitted": True, "byte_length": 3}}
