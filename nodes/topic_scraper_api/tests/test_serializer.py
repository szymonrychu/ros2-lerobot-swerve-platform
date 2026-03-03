from array import array
from dataclasses import dataclass

from topic_scraper_api.serializer import ros_message_to_builtin, ros_time_to_ns


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
