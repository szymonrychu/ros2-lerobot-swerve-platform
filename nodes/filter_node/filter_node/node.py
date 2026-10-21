"""ROS2 filter node: subscribe input JointState, run algorithm, publish filtered JointState."""

import time
from typing import Any

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import Executor, ExternalShutdownException, SingleThreadedExecutor
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros2_metrics import resolve_metrics_port, start_metrics_server
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from .algorithms import get_algorithm
from .arbitration import (
    SOURCE_AUTONOMY,
    SOURCE_LEADER,
    SOURCE_WEB_UI,
    ActiveSourceReporter,
    SourceArbiter,
    parse_released,
)
from .command import fill_command
from .config import EXECUTOR_KINDS, FilterConfig
from .feedback_rate import FeedbackRateSwitch
from .metrics import FilterMetrics

NODE_NAME = "filter_node"
FILTER_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

INPUT_STALE_WARN_S = 5.0
HEALTH_TIMER_PERIOD_S = 5.0
# Poll faster than the 1 Hz report period so timer jitter never stretches it to 2 s.
ACTIVE_SOURCE_POLL_S = 0.1


def joint_positions(msg: JointState) -> dict[str, float]:
    """Map joint names to positions, skipping names without a position.

    Args:
        msg (JointState): Incoming joint state.

    Returns:
        dict[str, float]: Joint name -> position (rad).
    """
    return {name: float(msg.position[i]) for i, name in enumerate(msg.name) if i < len(msg.position)}


def make_executor(kind: str) -> Executor:
    """Build the executor that spins the node.

    Args:
        kind (str): "events" for rclpy.experimental.EventsExecutor, "single_threaded" for SingleThreadedExecutor.

    Returns:
        Executor: A new executor of the requested kind.

    Raises:
        ValueError: If kind is not one of EXECUTOR_KINDS.
    """
    if kind == "events":
        return EventsExecutor()
    if kind == "single_threaded":
        return SingleThreadedExecutor()
    raise ValueError(f"executor must be one of {EXECUTOR_KINDS}, got {kind!r}")


def run_filter_node(config: FilterConfig) -> None:
    """Run the filter node until systemd (or Ctrl-C) stops it; shutdown is clean (exit 0, no traceback).

    Args:
        config (FilterConfig): Node configuration.
    """
    start_metrics_server(resolve_metrics_port(config.metrics_port), NODE_NAME)
    rclpy.init()
    node = Node(NODE_NAME)
    try:
        serve_filter_node(config, node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():  # a publish or wait set racing the SIGTERM handler is shutdown noise; anything else is a bug
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def serve_filter_node(config: FilterConfig, node: Node) -> None:
    """Subscribe input_topic and publish filtered output_topic at control_loop_hz until the context is shut down.

    Args:
        config (FilterConfig): Node configuration.
        node (Node): The rclpy node to attach subscriptions, publishers and timers to.
    """
    metrics = FilterMetrics()
    algorithm_cls = get_algorithm(config.algorithm)
    algorithm = algorithm_cls(config.algorithm_params)
    pub = node.create_publisher(JointState, config.output_topic, FILTER_QOS)
    state_by_joint: dict[str, Any] = {}
    last_measurement_time: dict[str, float] = {}
    joint_order: list[str] = list(config.joint_names) if config.joint_names else []
    clock = node.get_clock()
    control_period_s = 1.0 / max(1.0, config.control_loop_hz)
    last_input_time: list[float] = [time.monotonic()]
    last_leader_msg_time: list[float] = [time.monotonic()]  # any leader message, accepted or not
    last_idle_feedback_time: list[float | None] = [None]
    arbiter = SourceArbiter(
        web_ui_timeout_s=config.web_ui_timeout_s,
        takeover_threshold_rad=config.takeover_threshold_rad,
        proximity_check_enabled=bool(config.follower_feedback_topic),
    )
    reporter = ActiveSourceReporter()
    source_pub = (
        node.create_publisher(String, config.active_source_topic, FILTER_QOS) if config.active_source_topic else None
    )
    log = node.get_logger()

    def publish_direct(msg: JointState, source: str) -> None:
        """Republish a clean command (web UI / autonomy) unfiltered on the output topic, tagged with its source."""
        pub.publish(fill_command(JointState(), clock.now().to_msg(), msg.name, msg.position, source))

    def report_source() -> None:
        """Update metrics, then publish the active source when it changed or the 1 Hz period elapsed."""
        metrics.refresh(arbiter.active_source, time.monotonic())
        if source_pub is not None and reporter.due(arbiter.active_source, time.monotonic()):
            source_msg = String()
            source_msg.data = arbiter.active_source
            source_pub.publish(source_msg)

    def on_input(msg: JointState) -> None:
        last_leader_msg_time[0] = time.monotonic()
        previous = arbiter.active_source
        metrics.record_input(SOURCE_LEADER, time.monotonic())
        decision = arbiter.on_leader_input(joint_positions(msg), time.monotonic())
        if not decision.accepted:
            return
        if decision.resumed_after_release:
            # Drop stale pre-lease estimates so the first output starts at the (follower-near) leader pose.
            state_by_joint.clear()
            last_measurement_time.clear()
            log.info("Leader takeover after autonomy release: all joints within threshold")
        elif previous != arbiter.active_source:
            log.info(f"Leader takeover from {previous}")
        report_source()
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
        # Web UI sends clean data, no Kalman needed; ignored while autonomy holds the lease
        metrics.record_input(SOURCE_WEB_UI, time.monotonic())
        if arbiter.on_web_ui_command(time.monotonic()):
            publish_direct(msg, SOURCE_WEB_UI)
            report_source()

    def on_autonomy_input(msg: JointState) -> None:
        # The MCP server already streams smooth, rate-limited setpoints: republish directly
        metrics.record_input(SOURCE_AUTONOMY, time.monotonic())
        was_held = arbiter.autonomy_held
        if not arbiter.on_autonomy_command():
            return
        if not was_held:
            log.info("Autonomy lease taken: web UI and leader input ignored until release")
        publish_direct(msg, SOURCE_AUTONOMY)
        report_source()

    def on_autonomy_release(msg: Bool) -> None:
        if arbiter.on_autonomy_release(bool(msg.data)):
            log.info("Autonomy lease released: web UI active immediately, leader resumes on proximity")
            report_source()

    def on_torque_state(msg: String) -> None:
        try:
            released = parse_released(msg.data)
        except ValueError as exc:
            log.warning(f"Ignoring unparsable torque_state payload: {exc}")
            return
        if not arbiter.on_follower_released(released):
            return
        if released:
            log.info("Follower released: all command sources quieted")
        else:
            # Drop stale estimates; the leader resumes only within the proximity threshold.
            state_by_joint.clear()
            last_measurement_time.clear()
            log.info("Follower torque re-enabled: leader resumes on proximity")
        report_source()

    def on_follower_feedback(msg: JointState) -> None:
        arbiter.update_follower_positions(joint_positions(msg))

    def on_idle_feedback(msg: JointState) -> None:
        last_idle_feedback_time[0] = time.monotonic()
        on_follower_feedback(msg)

    node.create_subscription(JointState, config.input_topic, on_input, FILTER_QOS)
    node.create_timer(HEALTH_TIMER_PERIOD_S, health_check)
    if config.web_ui_input_topic:
        node.create_subscription(JointState, config.web_ui_input_topic, on_web_ui_input, FILTER_QOS)
        node.get_logger().info(
            f"Web UI arbitration enabled: {config.web_ui_input_topic} (timeout {config.web_ui_timeout_s}s)"
        )
    feedback_switch = FeedbackRateSwitch(
        config.follower_feedback_topic,
        config.follower_feedback_idle_topic,
        config.idle_timeout_s,
        config.web_ui_timeout_s,
        config.follower_feedback_idle_fallback_s,
        warn=log.warning,
    )
    feedback_sub: list[Any] = [None]  # current follower feedback subscription
    feedback_topic: list[str] = [""]

    def select_feedback_topic(now: float) -> str:
        return feedback_switch.select(
            now, last_leader_msg_time[0], arbiter.last_web_ui_time, arbiter.autonomy_held, last_idle_feedback_time[0]
        )

    def swap_feedback_subscription(now: float) -> None:
        """Move the follower feedback subscription to the selected topic. Main loop only, never in a callback."""
        topic = select_feedback_topic(now)
        if topic == feedback_topic[0]:
            return
        if feedback_sub[0] is not None:
            node.destroy_subscription(feedback_sub[0])
        callback = on_idle_feedback if topic == config.follower_feedback_idle_topic else on_follower_feedback
        feedback_sub[0] = node.create_subscription(JointState, topic, callback, FILTER_QOS)
        feedback_topic[0] = topic
        log.info(f"Follower feedback for takeover: {topic} (threshold {config.takeover_threshold_rad} rad)")

    if config.follower_feedback_topic:
        swap_feedback_subscription(time.monotonic())
    if config.follower_torque_state_topic:
        node.create_subscription(String, config.follower_torque_state_topic, on_torque_state, LATCHED_QOS)
        log.info(f"Follower torque state: {config.follower_torque_state_topic}")
    if config.autonomy_input_topic:
        node.create_subscription(JointState, config.autonomy_input_topic, on_autonomy_input, FILTER_QOS)
        if config.autonomy_release_topic:
            node.create_subscription(Bool, config.autonomy_release_topic, on_autonomy_release, FILTER_QOS)
        else:
            log.warning("No autonomy_release_topic: an autonomy lease can only end by restarting the node")
        log.info(f"Autonomy lease enabled: {config.autonomy_input_topic} (release {config.autonomy_release_topic})")
        if not config.follower_feedback_topic:
            log.warning("No follower_feedback_topic: leader cannot resume after an autonomy release")
    # Always poll: keeps the input-age and active-source gauges fresh even when no active_source topic is set.
    node.create_timer(ACTIVE_SOURCE_POLL_S, report_source)
    node.get_logger().info(f"Filter node: {config.input_topic} -> {config.output_topic} [{config.algorithm}]")

    executor = make_executor(config.executor)
    executor.add_node(node)

    idle_timeout = config.idle_timeout_s
    was_idle = False

    while rclpy.ok():
        metrics.observe_loop(time.monotonic(), control_period_s)
        if config.follower_feedback_topic:
            swap_feedback_subscription(time.monotonic())
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
            # Web UI (within timeout) and autonomy publish directly from their callbacks; after an autonomy
            # release nothing is published until the leader passes the proximity rule.
            if not arbiter.should_publish_filtered(now):
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
                pub.publish(fill_command(JointState(), clock.now().to_msg(), names, positions, SOURCE_LEADER))
        executor.spin_once(timeout_sec=control_period_s)
