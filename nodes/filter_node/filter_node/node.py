"""ROS2 filter node: subscribe input JointState, run algorithm, publish filtered JointState."""

import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from .algorithms import get_algorithm
from .config import FilterConfig

FILTER_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def run_filter_node(config: FilterConfig) -> None:
    """Run the filter node: subscribe input_topic, publish filtered output_topic at control_loop_hz."""
    rclpy.init()
    node = Node("filter_node")
    algorithm_cls = get_algorithm(config.algorithm)
    algorithm = algorithm_cls(config.algorithm_params)
    pub = node.create_publisher(JointState, config.output_topic, FILTER_QOS)
    state_by_joint: dict[str, Any] = {}
    last_measurement_time: dict[str, float] = {}
    joint_order: list[str] = list(config.joint_names) if config.joint_names else []
    clock = node.get_clock()
    control_period_s = 1.0 / max(1.0, config.control_loop_hz)

    def on_input(msg: JointState) -> None:
        now = time.monotonic()
        for i, name in enumerate(msg.name):
            if i >= len(msg.position):
                continue
            pos = float(msg.position[i])
            if name not in state_by_joint:
                state_by_joint[name] = algorithm.create_state(name, pos, now)
                if name not in joint_order:
                    joint_order.append(name)
            # If config had no joint_names, first message defines order
            algorithm.update(state_by_joint[name], name, pos, now)
            last_measurement_time[name] = now

    node.create_subscription(JointState, config.input_topic, on_input, FILTER_QOS)
    node.get_logger().info("Filter node: %s -> %s [%s]" % (config.input_topic, config.output_topic, config.algorithm))

    while rclpy.ok():
        if state_by_joint and joint_order:
            now = time.monotonic()
            names = [n for n in joint_order if n in state_by_joint]
            positions: list[float] = []
            for n in names:
                st = state_by_joint[n]
                last_t = last_measurement_time.get(n)
                pos = algorithm.predict(st, n, now, last_t)
                positions.append(pos)
            if names and len(positions) == len(names):
                out = JointState()
                out.header.stamp = clock.now().to_msg()
                out.header.frame_id = ""
                out.name = names
                out.position = positions
                out.velocity = []
                out.effort = []
                pub.publish(out)
        rclpy.spin_once(node, timeout_sec=0.001)
        time.sleep(control_period_s)

    node.destroy_node()
    rclpy.shutdown()
