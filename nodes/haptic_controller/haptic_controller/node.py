"""ROS2 haptic controller node: resistance and zero-G hold for leader gripper."""

import json
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .config import MODE_OFF, MODE_RESISTANCE, MODE_ZERO_G, HapticConfig
from .resistance import compute_resistance_target, should_apply_resistance_hysteresis

HAPTIC_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _index_of(name: str, names: list[str]) -> int | None:
    """Return index of name in names, or None."""
    try:
        return names.index(name)
    except ValueError:
        return None


def run_haptic_node(config: HapticConfig) -> None:
    """Run the haptic controller node. Does not return until shutdown."""
    rclpy.init()
    node = Node("haptic_controller")
    pub = node.create_publisher(JointState, config.leader_cmd_topic, HAPTIC_QOS)
    pub_set_register = node.create_publisher(String, config.leader_set_register_topic, HAPTIC_QOS)

    leader_state: dict[str, float] = {}  # joint -> position
    leader_velocity: dict[str, float] = {}  # joint -> velocity (from last delta)
    leader_names_order: list[str] = []
    leader_stamp: float = 0.0
    follower_state: dict[str, float] = {}  # joint -> position
    follower_load: dict[str, float] = {}  # joint -> effort (load)
    follower_stamp: float = 0.0
    last_leader_pos: dict[str, float] = {}
    last_leader_time: float = 0.0
    resistance_last_active_s: float = 0.0
    resistance_torque_enabled: bool = False
    resistance_initialized: bool = False
    resistance_was_active_per_joint: dict[str, bool] = {}  # hysteresis state
    resistance_active_cycles: int = 0  # debounce: consecutive cycles active

    def on_leader(msg: JointState) -> None:
        nonlocal leader_names_order, leader_stamp, last_leader_time
        now = time.monotonic()
        for i, name in enumerate(msg.name):
            if i >= len(msg.position):
                continue
            pos = float(msg.position[i])
            vel = msg.velocity[i] if msg.velocity and i < len(msg.velocity) else 0.0
            if name in last_leader_pos and last_leader_time > 0:
                dt = now - last_leader_time
                if dt > 0:
                    vel = (pos - last_leader_pos[name]) / dt
            leader_state[name] = pos
            leader_velocity[name] = vel
            if name not in leader_names_order:
                leader_names_order.append(name)
        leader_names_order = [n for n in leader_names_order if n in leader_state]
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                last_leader_pos[name] = float(msg.position[i])
        last_leader_time = now
        leader_stamp = now

    def on_follower(msg: JointState) -> None:
        nonlocal follower_stamp
        now = time.monotonic()
        for i, name in enumerate(msg.name):
            if i >= len(msg.position):
                continue
            follower_state[name] = float(msg.position[i])
            if msg.effort and i < len(msg.effort):
                follower_load[name] = float(msg.effort[i])
            else:
                follower_load[name] = 0.0
        follower_stamp = now

    node.create_subscription(
        JointState,
        config.leader_state_topic,
        on_leader,
        HAPTIC_QOS,
    )
    node.create_subscription(
        JointState,
        config.follower_state_topic,
        on_follower,
        HAPTIC_QOS,
    )

    node.get_logger().info(
        "Haptic controller: mode=%s gripper=%s leader_cmd=%s leader_set_register=%s"
        % (config.mode, config.gripper_joint_names, config.leader_cmd_topic, config.leader_set_register_topic)
    )

    control_period_s = 1.0 / max(1.0, config.control_loop_hz)
    now = time.monotonic()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def set_gripper_torque(enable: bool) -> None:
        value = 1 if enable else 0
        for joint_name in config.gripper_joint_names:
            payload = {
                "joint_name": joint_name,
                "register": "torque_enable",
                "value": value,
            }
            pub_set_register.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    while rclpy.ok():
        executor.spin_once(timeout_sec=control_period_s)
        now = time.monotonic()

        if config.mode == MODE_OFF:
            continue

        # Watchdog: require fresh leader and follower data
        if now - leader_stamp > config.watchdog_timeout_s or now - follower_stamp > config.watchdog_timeout_s:
            continue

        # Build command only for gripper joints that we have leader state for
        gripper_set = set(config.gripper_joint_names)
        names = [n for n in leader_names_order if n in gripper_set and n in leader_state]
        if not names:
            continue

        if config.mode == MODE_RESISTANCE and not resistance_initialized:
            # Safety: start in manual mode (torque off), only enable during active resistance rendering.
            set_gripper_torque(False)
            resistance_torque_enabled = False
            resistance_initialized = True

        # Delay safety: in resistance mode, drop to passive if leader/follower stamp skew too high
        skip_resistance_due_to_skew = (
            config.mode == MODE_RESISTANCE and abs(leader_stamp - follower_stamp) > config.delay_safety_max_skew_s
        )

        positions: list[float] = []
        out_names: list[str] = []
        resistance_active = False
        for name in names:
            pos = leader_state[name]
            if config.mode == MODE_ZERO_G:
                out_names.append(name)
                positions.append(pos)
            elif config.mode == MODE_RESISTANCE:
                if skip_resistance_due_to_skew:
                    resistance_was_active_per_joint[name] = False
                    continue
                vel = leader_velocity.get(name, 0.0)
                load = follower_load.get(name, 0.0)
                was_active = resistance_was_active_per_joint.get(name, False)
                active = should_apply_resistance_hysteresis(
                    vel,
                    load,
                    config.resistance_load_deadband,
                    config.resistance_activation_velocity_threshold,
                    config.resistance_load_release_ratio,
                    was_active,
                )
                resistance_was_active_per_joint[name] = active
                if active:
                    resistance_active = True
                    out_names.append(name)
                    pos = compute_resistance_target(
                        pos,
                        vel,
                        load,
                        config.resistance_load_deadband,
                        config.resistance_max_stiffness,
                        config.resistance_max_step_per_cycle,
                    )
                    positions.append(pos)
            else:
                out_names.append(name)
                positions.append(pos)

        if config.mode == MODE_RESISTANCE:
            if resistance_active:
                resistance_last_active_s = now
                resistance_active_cycles = min(
                    resistance_active_cycles + 1,
                    config.resistance_activation_debounce_cycles + 1,
                )
                if (
                    not resistance_torque_enabled
                    and resistance_active_cycles >= config.resistance_activation_debounce_cycles
                ):
                    set_gripper_torque(True)
                    resistance_torque_enabled = True
            else:
                resistance_active_cycles = 0
                if resistance_torque_enabled and (now - resistance_last_active_s >= config.resistance_release_delay_s):
                    set_gripper_torque(False)
                    resistance_torque_enabled = False

        if out_names and positions:
            out = JointState()
            out.header.stamp = node.get_clock().now().to_msg()
            out.header.frame_id = ""
            out.name = out_names
            out.position = positions
            out.velocity = []
            out.effort = []
            pub.publish(out)

    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    """Entry point: load config and run node. Exits on config error."""
    from .config import load_config_from_env

    config = load_config_from_env()
    if config is None:
        import sys

        sys.exit(
            "Haptic controller config not found or invalid. Set HAPTIC_CONTROLLER_CONFIG to a YAML path "
            "or deploy to /etc/ros2/haptic_controller/config.yaml"
        )
    run_haptic_node(config)
