"""Non-finite floats (feetech publishes NaN effort for unmeasured joints) must serialize as JSON null."""

import array
import json
import math

from web_ui.msg_serializer import msg_to_dict


class FakeJointState:
    """JointState-like message: rclpy-style underscore slots with field properties."""

    __slots__ = ("_name", "_position", "_velocity", "_effort")

    name = property(lambda self: self._name)
    position = property(lambda self: self._position)
    velocity = property(lambda self: self._velocity)
    effort = property(lambda self: self._effort)


def joint_state(effort: list[float]) -> FakeJointState:
    """Build a JointState-like message.

    Args:
        effort (list[float]): Effort values, may contain NaN/inf.

    Returns:
        FakeJointState: Message with name, position, velocity and effort.
    """
    msg = FakeJointState()
    msg._name = ["a", "b", "c"]
    msg._position = array.array("d", [0.1, 0.2, 0.3])
    msg._velocity = array.array("d", [])
    msg._effort = array.array("d", effort)
    return msg


def test_nan_and_inf_effort_serialize_as_null_and_frame_is_valid_json() -> None:
    data = msg_to_dict(joint_state([math.nan, math.inf, 32.0]))
    frame = json.dumps(data, allow_nan=False)
    assert json.loads(frame)["effort"] == [None, None, 32.0]
    assert json.loads(frame)["position"] == [0.1, 0.2, 0.3]
