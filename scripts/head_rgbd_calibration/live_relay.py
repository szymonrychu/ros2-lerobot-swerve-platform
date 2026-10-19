"""Robot-side relay for the live calibration pose counter (scripts/head_rgbd_calibration/live.py).

Not imported on the dev machine: live.py pipes this file to the robot's ROS python over ssh, as
`python3 - <frame header> <rgb topic> <amplitude topic>`. Each received message is written to stdout as the
struct-packed (topic index, length) header followed by its serialized CDR bytes: index 0 is the RGB topic, 1 the ToF
amplitude topic. The relay ends on SIGINT (the remote timeout) or when the dev machine closes the pipe.
"""

import struct
import sys
from collections.abc import Callable
from typing import BinaryIO

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

NODE_NAME = "head_rgbd_calibration_live_relay"


def forwarder(header: str, index: int, out: BinaryIO) -> Callable[[bytes], None]:
    """Callback writing one serialized message with its (index, length) header to out and flushing it."""

    def forward(data: bytes) -> None:
        out.write(struct.pack(header, index, len(data)) + data)
        out.flush()

    return forward


def main() -> None:
    """Relay the RGB and ToF amplitude topics named on the command line to stdout until interrupted."""
    header, rgb_topic, amplitude_topic = sys.argv[1:4]
    out = sys.stdout.buffer
    rclpy.init()
    node = rclpy.create_node(NODE_NAME)
    node.create_subscription(CompressedImage, rgb_topic, forwarder(header, 0, out), qos_profile_sensor_data, raw=True)
    node.create_subscription(Image, amplitude_topic, forwarder(header, 1, out), qos_profile_sensor_data, raw=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, BrokenPipeError):
        pass


if __name__ == "__main__":
    main()
