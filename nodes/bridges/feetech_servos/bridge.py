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
from .startup_torque import set_startup_torque_state
from .trajectory import JointInterpolator, make_joint_interpolator, sample_joint, set_joint_target

DEFAULT_QOS_DEPTH = 10
# Run the control loop faster for smoother follower motion.
SPIN_CYCLES_PER_LOOP = 2
SPIN_TIMEOUT_S = 0.002
REGISTER_PUBLISH_INTERVAL_S = 1.0
SERVO_WAIT_INTERVAL_S = 1.0
TORQUE_WRITE_ATTEMPTS = 8
TORQUE_VERIFY_SLEEP_S = 0.02
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
    latest_goal_targets: dict[int, int] = {}
    filtered_goal_targets: dict[int, float] = {}
    applied_goal_targets: dict[int, int] = {}
    last_retarget_time: dict[int, float] = {}
    command_velocity_steps_s: dict[int, float] = {}
    last_command_steps: dict[int, float] = {}
    last_command_time: dict[int, float] = {}
    last_sent_goal: dict[int, int] = {}
    last_sent_time: dict[int, float] = {}
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
                joint_ids = [joint.id for joint in config.joints]
                failed_ids = set_startup_torque_state(
                    joint_ids=joint_ids,
                    torque_value=torque_value,
                    write_once=lambda sid, value: write_register(servo, sid, torque_entry, value, last_written[sid]),
                    read_once=lambda sid: read_register_raw(servo, sid, torque_entry),
                    attempts=TORQUE_WRITE_ATTEMPTS,
                    verify_sleep_s=TORQUE_VERIFY_SLEEP_S,
                )
                if failed_ids:
                    node.get_logger().warn(f"Failed startup torque_enable={torque_value} for servos: {failed_ids}")
                else:
                    node.get_logger().info(
                        f"Applied torque_enable={torque_value} on startup for all configured servos."
                    )

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
                now = time.monotonic()
                prev_filtered = filtered_goal_targets.get(sid, float(target_steps))
                alpha = config.target_lowpass_alpha
                filtered = prev_filtered + alpha * (float(target_steps) - prev_filtered)
                filtered_goal_targets[sid] = filtered
                latest_goal_targets[sid] = int(round(filtered))
                prev_steps = last_command_steps.get(sid)
                prev_time = last_command_time.get(sid, now)
                if prev_steps is not None and now > prev_time:
                    command_velocity_steps_s[sid] = (filtered - prev_steps) / (now - prev_time)
                else:
                    command_velocity_steps_s[sid] = 0.0
                last_command_steps[sid] = filtered
                last_command_time[sid] = now
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
                        applied_goal_targets[sid] = int(round(initial_steps))
                        last_retarget_time[sid] = now
                        last_sent_goal[sid] = int(round(initial_steps))
                        last_sent_time[sid] = now
                    pending = latest_goal_targets.get(sid)
                    if pending is not None:
                        last_applied = applied_goal_targets.get(sid, pending)
                        moving_threshold = config.source_motion_velocity_threshold_steps_s
                        source_velocity = abs(command_velocity_steps_s.get(sid, 0.0))
                        is_source_moving = source_velocity >= moving_threshold
                        active_target_hz = (
                            config.moving_target_update_hz
                            if is_source_moving
                            else config.interpolation_target_update_hz
                        )
                        active_deadband_steps = (
                            config.moving_command_deadband_steps if is_source_moving else config.command_deadband_steps
                        )
                        retarget_period_s = 1.0 / max(1.0, active_target_hz)
                        delta_steps = abs(int(pending) - int(last_applied))
                        if active_deadband_steps <= 0:
                            should_retarget = delta_steps > 0
                        else:
                            should_retarget = delta_steps >= active_deadband_steps
                        if should_retarget and now - last_retarget_time.get(sid, 0.0) >= retarget_period_s:
                            set_joint_target(
                                interpolators[sid],
                                float(pending),
                                now,
                                config.command_smoothing_time_s,
                            )
                            applied_goal_targets[sid] = int(pending)
                            last_retarget_time[sid] = now
                    smoothed = int(round(sample_joint(interpolators[sid], now)))
                    if config.max_goal_step_rate > 0:
                        previous_goal = last_sent_goal.get(sid, smoothed)
                        previous_time = last_sent_time.get(sid, now)
                        dt = max(0.0, now - previous_time)
                        max_delta = max(1, int(round(config.max_goal_step_rate * dt)))
                        delta = smoothed - previous_goal
                        if delta > max_delta:
                            smoothed = previous_goal + max_delta
                        elif delta < -max_delta:
                            smoothed = previous_goal - max_delta
                    last_sent_goal[sid] = smoothed
                    last_sent_time[sid] = now
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
