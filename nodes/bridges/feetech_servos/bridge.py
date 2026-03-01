"""Feetech servos bridge: publish joint_states, subscribe joint_commands, read/write servo registers.

Configuration is loaded from a YAML file (path via FEETECH_SERVOS_CONFIG or default).
When device is set: connects to hardware, reads all registers at startup (prints one-line JSON per servo),
publishes joint_states and servo_registers (JSON), subscribes to joint_commands and set_register.
EPROM writes use unlock -> write -> lock; writes are skipped when value unchanged.
When device is not set: stub (publish zero joint_states, log joint_commands).
"""

import json
import sys
import time
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .config import BridgeConfig, load_config_from_env
from .joint_updates import get_position_updates
from .registers import WRITABLE_REGISTER_NAMES, get_register_entry_by_name, read_all_registers
from .registers import read_register as read_register_raw
from .registers import write_register
from .trajectory import JointInterpolator, make_joint_interpolator, sample_joint, set_joint_target

DEFAULT_QOS_DEPTH = 10
# Run the control loop faster for smoother follower motion.
SPIN_CYCLES_PER_LOOP = 2
SPIN_TIMEOUT_S = 0.002
REGISTER_PUBLISH_INTERVAL_S = 1.0
SERVO_WAIT_INTERVAL_S = 1.0
# Radians to steps: scale for joint_commands position -> servo goal_position (steps); adjust per hardware.
POSITION_RADIAN_TO_STEPS = 1000.0
MIN_STEPS = 0
MAX_STEPS = 4095


def _wait_for_servos(servo: Any, expected_ids: list[int], node: Node) -> None:
    """Block until all expected servo IDs are present on the bus. Log and print missing IDs every 1s.

    Raises RuntimeError if rclpy is shut down before all servos appear (so we never run with missing servos).
    """
    expected_set = set(expected_ids)
    while rclpy.ok():
        present = servo.ListServos()
        if present is None:
            node.get_logger().warn("Servo scan failed (ListServos returned None), retrying...")
            time.sleep(SERVO_WAIT_INTERVAL_S)
            continue
        present_set = set(present)
        missing = sorted(expected_set - present_set)
        if not missing:
            return
        node.get_logger().info(
            f"Waiting for servos: {missing} (configured: {sorted(expected_ids)}, present: {sorted(present_set)})"
        )
        print(f"Waiting for servos: {missing}", flush=True)
        time.sleep(SERVO_WAIT_INTERVAL_S)
    raise RuntimeError("Shutdown during servo wait")


def _clamp_steps(value: float) -> int:
    return max(MIN_STEPS, min(MAX_STEPS, int(round(value))))


def run_bridge(config: BridgeConfig) -> None:
    """Run the bridge: joint_states, joint_commands, optional full register read/write when device is set."""
    rclpy.init()
    node = Node("feetech_servos_bridge")
    control_loop_sleep_s = 1.0 / max(1.0, config.control_loop_hz)
    namespace = config.namespace
    joint_names = config.joint_names
    state_topic = f"/{namespace}/joint_states"
    cmd_topic = f"/{namespace}/joint_commands"
    registers_topic = f"/{namespace}/servo_registers"
    set_register_topic = f"/{namespace}/set_register"

    pub_state = node.create_publisher(JointState, state_topic, DEFAULT_QOS_DEPTH)
    pub_registers = node.create_publisher(String, registers_topic, DEFAULT_QOS_DEPTH)

    last_positions: dict[str, float] = {}
    last_written: dict[int, dict[str, int]] = {}  # servo_id -> { register_name: value }
    pending_goal_targets: dict[int, int] = {}
    interpolators: dict[int, JointInterpolator] = {}
    servo: Any = None

    if config.device:
        try:
            from st3215 import ST3215

            servo = ST3215(config.device)
        except (ImportError, ValueError, OSError) as e:
            node.get_logger().error(f"Failed to open device {config.device}: {e}. Running as stub.")
            servo = None

    if servo is not None:
        expected_ids = [j.id for j in config.joints]
        _wait_for_servos(servo, expected_ids, node)
        # Read all registers for each joint and print one-line JSON per servo (debug).
        for joint in config.joints:
            sid = joint.id
            regs = read_all_registers(servo, sid)
            payload = {"servo_id": sid, "joint_name": joint.name, "registers": regs}
            print(json.dumps(payload, separators=(",", ":")), flush=True)
            last_written[sid] = {}
        if config.enable_torque_on_start or config.disable_torque_on_start:
            torque_entry = get_register_entry_by_name("torque_enable")
            if torque_entry is not None:
                torque_value = 1 if config.enable_torque_on_start else 0
                for joint in config.joints:
                    write_register(servo, joint.id, torque_entry, torque_value, last_written[joint.id])
                node.get_logger().info(f"Applied torque_enable={torque_value} on startup for all configured servos.")

    goal_entry = get_register_entry_by_name("goal_position")

    def on_command(msg: JointState) -> None:
        if servo is None:
            return
        if goal_entry is None:
            return
        for i, name in enumerate(msg.name):
            if i >= len(msg.position):
                continue
            sid = config.servo_id_for_joint_name(name)
            if sid is None:
                continue
            target_steps = _clamp_steps(float(msg.position[i]) * POSITION_RADIAN_TO_STEPS)
            if sid not in last_written:
                last_written[sid] = {}
            if config.interpolation_enabled:
                pending_goal_targets[sid] = target_steps
            else:
                write_register(servo, sid, goal_entry, target_steps, last_written[sid])

    def on_set_register(msg: String) -> None:
        if servo is None:
            return
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            node.get_logger().warn("set_register: invalid JSON")
            return
        joint_name = data.get("joint_name") or data.get("joint")
        reg_name = data.get("register") or data.get("register_name")
        raw = data.get("value")
        if joint_name is None or reg_name is None or raw is None:
            node.get_logger().warn("set_register: missing joint_name, register, or value")
            return
        if reg_name not in WRITABLE_REGISTER_NAMES:
            node.get_logger().warn(f"set_register: unknown or read-only register '{reg_name}'")
            return
        sid = config.servo_id_for_joint_name(str(joint_name))
        if sid is None:
            node.get_logger().warn(f"set_register: unknown joint '{joint_name}'")
            return
        try:
            value = int(raw)
        except (TypeError, ValueError):
            node.get_logger().warn("set_register: value must be int")
            return
        entry = get_register_entry_by_name(reg_name)
        if entry is None:
            return
        if sid not in last_written:
            last_written[sid] = {}
        if not write_register(servo, sid, entry, value, last_written[sid]):
            node.get_logger().warn(f"set_register: write failed for {joint_name}/{reg_name}={value}")

    node.create_subscription(JointState, cmd_topic, on_command, DEFAULT_QOS_DEPTH)
    node.create_subscription(String, set_register_topic, on_set_register, DEFAULT_QOS_DEPTH)

    last_register_publish = 0.0

    while rclpy.ok():
        if servo is not None:
            # Publish joint_states from present position.
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = ""
            msg.name = list(joint_names)
            positions: list[float] = []
            velocities: list[float] = []
            position_steps_by_id: dict[int, int] = {}
            pos_entry = get_register_entry_by_name("present_position")
            speed_entry = get_register_entry_by_name("present_speed")
            for joint in config.joints:
                pos = read_register_raw(servo, joint.id, pos_entry) if pos_entry else None
                speed = read_register_raw(servo, joint.id, speed_entry) if speed_entry else None
                if pos is not None:
                    position_steps_by_id[joint.id] = int(pos)
                positions.append(float(pos) / POSITION_RADIAN_TO_STEPS if pos is not None else 0.0)
                velocities.append(float(speed) if speed is not None else 0.0)
            msg.position = positions
            msg.velocity = velocities
            msg.effort = []
            pub_state.publish(msg)
            if config.log_joint_updates:
                changing = get_position_updates(msg.name, msg.position, last_positions)
                if changing:
                    line = ",".join(f"{name}:{val}" for name, val in changing)
                    print(line, flush=True)
            if goal_entry is not None and config.interpolation_enabled:
                now = time.monotonic()
                for joint in config.joints:
                    sid = joint.id
                    if sid not in interpolators:
                        initial_steps = position_steps_by_id.get(sid, 0)
                        interpolators[sid] = make_joint_interpolator(float(initial_steps), now)
                    pending = pending_goal_targets.pop(sid, None)
                    if pending is not None:
                        set_joint_target(interpolators[sid], float(pending), now, config.command_smoothing_time_s)
                    smoothed = int(round(sample_joint(interpolators[sid], now)))
                    write_register(servo, sid, goal_entry, smoothed, last_written[sid])
            # Publish full register dump at REGISTER_PUBLISH_INTERVAL_S.
            now = time.monotonic()
            if now - last_register_publish >= REGISTER_PUBLISH_INTERVAL_S:
                last_register_publish = now
                payload: dict[str, dict[str, int]] = {}
                for joint in config.joints:
                    regs: dict[str, int] = read_all_registers(servo, joint.id)
                    payload[joint.name] = regs
                pub_registers.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        else:
            # Stub: zero joint_states.
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = ""
            msg.name = list(joint_names)
            msg.position = [0.0] * len(joint_names)
            msg.velocity = [0.0] * len(joint_names)
            msg.effort = []
            pub_state.publish(msg)

        for _ in range(SPIN_CYCLES_PER_LOOP):
            rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)
        time.sleep(control_loop_sleep_s)

    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    """Entry point: load config from env and run bridge. Exits with message on config error."""
    config = load_config_from_env()
    if config is None:
        sys.exit(
            "Feetech servos config not found or invalid. Set FEETECH_SERVOS_CONFIG to a YAML path "
            "with 'namespace' and 'joint_names' (list of { name, id } per joint), or deploy config to "
            "/etc/ros2/feetech_servos/config.yaml"
        )
    run_bridge(config)
