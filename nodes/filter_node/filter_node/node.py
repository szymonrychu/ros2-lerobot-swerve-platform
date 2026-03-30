"""ROS2 filter node: subscribe input JointState, run algorithm, publish filtered JointState."""

import time
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
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

INPUT_STALE_WARN_S = 5.0
HEALTH_TIMER_PERIOD_S = 5.0


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
    last_input_time: list[float] = [time.monotonic()]
    active_source: list[str] = ["leader"]
    last_web_ui_time: list[float] = [0.0]
    follower_positions: dict[str, float] = {}

    def on_input(msg: JointState) -> None:
        # Feature 2+3: when web UI is active, check if leader can take over
        if config.web_ui_input_topic and active_source[0] == "web_ui":
            elapsed = time.monotonic() - last_web_ui_time[0]
            if elapsed < config.web_ui_timeout_s:
                # Web UI still active. Allow leader takeover only if joints are close.
                if config.follower_feedback_topic and follower_positions:
                    all_close = all(
                        abs(float(msg.position[i]) - follower_positions.get(name, float("inf")))
                        <= config.takeover_threshold_rad
                        for i, name in enumerate(msg.name)
                        if i < len(msg.position)
                    )
                    if all_close:
                        active_source[0] = "leader"
                        node.get_logger().info("Leader takeover: all joints within threshold, switching to leader")
                    else:
                        return  # leader too far, ignore
                else:
                    return  # no feedback configured, cannot verify proximity
            else:
                active_source[0] = "leader"
                node.get_logger().info("Web UI idle, reverting to leader source")
        last_input_time[0] = time.monotonic()
        now = last_input_time[0]
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

    def health_check() -> None:
        elapsed = time.monotonic() - last_input_time[0]
        if elapsed > INPUT_STALE_WARN_S:
            node.get_logger().warning(f"No input on {config.input_topic} for {elapsed:.1f}s")

    def on_web_ui_input(msg: JointState) -> None:
        active_source[0] = "web_ui"
        last_web_ui_time[0] = time.monotonic()
        # Publish web UI commands directly — web UI sends clean data, no Kalman needed
        out = JointState()
        out.header.stamp = clock.now().to_msg()
        out.header.frame_id = ""
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = []
        out.effort = []
        pub.publish(out)

    def on_follower_feedback(msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                follower_positions[name] = float(msg.position[i])

    node.create_subscription(JointState, config.input_topic, on_input, FILTER_QOS)
    node.create_timer(HEALTH_TIMER_PERIOD_S, health_check)
    if config.web_ui_input_topic:
        node.create_subscription(JointState, config.web_ui_input_topic, on_web_ui_input, FILTER_QOS)
        node.get_logger().info(
            f"Web UI arbitration enabled: {config.web_ui_input_topic} (timeout {config.web_ui_timeout_s}s)"
        )
    if config.follower_feedback_topic:
        node.create_subscription(JointState, config.follower_feedback_topic, on_follower_feedback, FILTER_QOS)
        node.get_logger().info(
            f"Follower feedback for takeover: {config.follower_feedback_topic}"
            f" (threshold {config.takeover_threshold_rad} rad)"
        )
    node.get_logger().info("Filter node: %s -> %s [%s]" % (config.input_topic, config.output_topic, config.algorithm))

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    idle_timeout = config.idle_timeout_s
    was_idle = False

    while rclpy.ok():
        if state_by_joint and joint_order:
            now = time.monotonic()
            input_age = now - last_input_time[0]
            if idle_timeout > 0 and input_age > idle_timeout:
                if not was_idle:
                    node.get_logger().info(
                        f"Input idle for {input_age:.1f}s (threshold {idle_timeout}s), pausing output"
                    )
                    was_idle = True
                executor.spin_once(timeout_sec=control_period_s)
                continue
            # When web UI is active and not timed out, skip Kalman publish
            # (web UI callback already published directly)
            if config.web_ui_input_topic and active_source[0] == "web_ui":
                if time.monotonic() - last_web_ui_time[0] < config.web_ui_timeout_s:
                    executor.spin_once(timeout_sec=control_period_s)
                    continue
            if was_idle:
                node.get_logger().info("Input resumed, publishing output")
                was_idle = False
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
        executor.spin_once(timeout_sec=control_period_s)

    node.destroy_node()
    rclpy.shutdown()
