"""Output command message filling for the filter node (rclpy-free, duck-typed on sensor_msgs/JointState).

Every message on the output topic carries its command source in header.frame_id (``leader``, ``web_ui`` or
``autonomy``). The follower bridge passes ``web_ui`` / ``autonomy`` positions through as follower joint radians
and applies its leader -> follower range mapping (gripper) only to leader commands.
"""

from collections.abc import Sequence
from typing import Any


def fill_command(msg: Any, stamp: Any, names: Sequence[str], positions: Sequence[float], source: str) -> Any:
    """Fill a JointState command with positions only and tag it with its command source.

    Args:
        msg (Any): sensor_msgs/JointState to fill (modified in place).
        stamp (Any): header.stamp (builtin_interfaces/Time).
        names (Sequence[str]): Joint names.
        positions (Sequence[float]): Joint positions (rad), same order as names.
        source (str): Command source (filter_node.arbitration SOURCE_*), written to header.frame_id.

    Returns:
        Any: The same msg.
    """
    msg.header.stamp = stamp
    msg.header.frame_id = source
    msg.name = list(names)
    msg.position = list(positions)
    msg.velocity = []
    msg.effort = []
    return msg
