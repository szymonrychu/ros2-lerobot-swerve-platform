"""Feetech servos bridge: publish joint_states, subscribe joint_commands, read/write servo registers.

Configuration is loaded from a YAML file (path via FEETECH_SERVOS_CONFIG or default).
When device is set: connects to hardware, reads all registers at startup (prints one-line JSON per servo),
publishes joint_states and servo_registers (JSON), subscribes to joint_commands and set_register.
EPROM writes use unlock -> write -> lock; writes are skipped when value unchanged.
When device is not set or cannot be opened: nothing is published (no placeholder joint_states).
Multiple namespaces (extra_groups) can share one bus; velocity-mode joints run in wheel mode.
Filtering/smoothing of joint_commands is handled by a separate filter node; this bridge applies commands directly.
"""

import json
import sys
import time
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .command_mapping import (
    map_position_to_steps,
    position_to_raw_steps,
    speed_register_to_velocity,
    steps_to_radians,
    velocity_to_speed_register,
)
from .config import BridgeConfig, JointEntry, JointGroup, load_config_from_env
from .joint_updates import get_position_updates
from .register_dump import RegisterDumpScheduler
from .registers import WRITABLE_REGISTER_NAMES, get_register_entry_by_name, read_all_registers
from .registers import read_register as read_register_raw
from .registers import write_register
from .startup_torque import hold_current_positions, set_startup_torque_state
from .sync_read import PRESENT_POSITION_ADDRESS, SYNC_READ_LENGTH, read_positions_and_speeds
from .velocity_watchdog import apply_velocity_command, expired_velocity_joints

DEFAULT_QOS_DEPTH = 10
SERVO_WAIT_INTERVAL_S = 1.0
TORQUE_WRITE_ATTEMPTS = 8
TORQUE_VERIFY_SLEEP_S = 0.02
MIN_STEPS = 0
MAX_STEPS = 4095
WHEEL_MODE = 1  # STS mode register: 0 = position servo, 1 = continuous rotation (wheel)
MISSING_READ_LOG_INTERVAL_S = 5.0


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


def run_bridge(config: BridgeConfig) -> None:
    """Run the bridge: per-group joint_states / joint_commands over one serial bus.

    Position-mode joints are driven by JointState.position (goal_position); velocity-mode joints run in
    wheel mode and are driven by JointState.velocity (goal_speed, rad/s). Velocity joints are stopped when
    commands go stale (velocity_command_timeout_s) and on shutdown.
    """
    rclpy.init()
    node = Node("feetech_servos_bridge")
    control_loop_sleep_s = 1.0 / max(1.0, config.control_loop_hz)
    groups = config.groups
    all_joints = config.all_joints
    joint_by_id = {j.id: j for j in all_joints}
    velocity_joints = [j for j in all_joints if j.mode == "velocity"]

    pub_state = {
        g.namespace: node.create_publisher(JointState, f"/{g.namespace}/joint_states", DEFAULT_QOS_DEPTH)
        for g in groups
    }
    pub_registers = node.create_publisher(String, f"/{config.namespace}/servo_registers", DEFAULT_QOS_DEPTH)

    last_positions: dict[str, float] = {}
    last_published_positions: dict[str, float] = {}  # for publish_only_on_change gate
    last_written: dict[int, dict[str, int]] = {}  # servo_id -> { register_name: value }
    command_limits: dict[int, tuple[int, int]] = {}  # servo_id -> (cmd_min, cmd_max)
    last_velocity_commands: dict[int, tuple[float, float]] = {}  # servo_id -> (time, rad/s) of last good write
    servo: Any = None

    if config.device:
        try:
            from st3215 import ST3215

            servo = ST3215(config.device)
        except (ImportError, ValueError, OSError) as e:
            node.get_logger().error(f"Failed to open device {config.device}: {e}. No joint_states will be published.")
            servo = None

    goal_entry = get_register_entry_by_name("goal_position")
    speed_goal_entry = get_register_entry_by_name("goal_speed")
    mode_entry = get_register_entry_by_name("mode")
    pos_entry = get_register_entry_by_name("present_position")
    speed_entry = get_register_entry_by_name("present_speed")
    load_entry = get_register_entry_by_name("present_load")
    effort_joint_set = set(config.publish_effort_joints) if config.publish_effort_joints else set()

    if servo is not None:
        expected_ids = [j.id for j in all_joints]
        _wait_for_servos(servo, expected_ids, node)
        # Read all registers for each joint and print one-line JSON per servo (debug).
        for joint in all_joints:
            sid = joint.id
            regs = read_all_registers(servo, sid)
            payload = {"servo_id": sid, "joint_name": joint.name, "registers": regs}
            print(json.dumps(payload, separators=(",", ":")), flush=True)
            last_written[sid] = {}
        # Velocity joints: wheel mode (RAM value; set every start) and stopped before torque comes on.
        if velocity_joints and mode_entry is not None and speed_goal_entry is not None:
            for joint in velocity_joints:
                write_register(servo, joint.id, speed_goal_entry, 0, last_written[joint.id])
            failed_mode_ids = set_startup_torque_state(
                joint_ids=[j.id for j in velocity_joints],
                torque_value=WHEEL_MODE,
                write_once=lambda sid, value: write_register(servo, sid, mode_entry, value, last_written[sid]),
                read_once=lambda sid: read_register_raw(servo, sid, mode_entry),
                attempts=TORQUE_WRITE_ATTEMPTS,
                verify_sleep_s=TORQUE_VERIFY_SLEEP_S,
            )
            if failed_mode_ids:
                node.get_logger().error(f"Failed to set wheel mode for servos {failed_mode_ids}; velocity ignored.")
                velocity_joints = [j for j in velocity_joints if j.id not in failed_mode_ids]
        if config.enable_torque_on_start or config.disable_torque_on_start:
            torque_entry = get_register_entry_by_name("torque_enable")
            if torque_entry is not None:
                torque_value = 1 if config.enable_torque_on_start else 0
                joint_ids = [joint.id for joint in all_joints]
                if torque_value == 1 and goal_entry is not None and pos_entry is not None:
                    # goal_position may hold a stale value (e.g. 0): hold present position before torque-on.
                    unheld = hold_current_positions(
                        [j.id for j in all_joints if j.mode == "position"],
                        lambda sid: read_register_raw(servo, sid, pos_entry),
                        lambda sid, value: write_register(servo, sid, goal_entry, value, last_written[sid]),
                    )
                    if unheld:
                        node.get_logger().error(f"Could not hold position of servos {unheld}; torque left off.")
                        joint_ids = [sid for sid in joint_ids if sid not in unheld]
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
        # Build per-joint command range: config override or read min/max_angle_limit from servo.
        min_entry = get_register_entry_by_name("min_angle_limit")
        max_entry = get_register_entry_by_name("max_angle_limit")
        for joint in all_joints:
            if joint.mode == "velocity":
                continue
            if joint.command_min_steps is not None and joint.command_max_steps is not None:
                command_limits[joint.id] = (joint.command_min_steps, joint.command_max_steps)
            elif min_entry and max_entry:
                min_val = read_register_raw(servo, joint.id, min_entry)
                max_val = read_register_raw(servo, joint.id, max_entry)
                if min_val is not None and max_val is not None:
                    command_limits[joint.id] = (min_val, max_val)
                else:
                    node.get_logger().warn(
                        f"Could not read min/max_angle_limit for {joint.name} (id={joint.id}), using 0-4095."
                    )
                    command_limits[joint.id] = (MIN_STEPS, MAX_STEPS)
            else:
                command_limits[joint.id] = (MIN_STEPS, MAX_STEPS)

    velocity_ids = {j.id for j in velocity_joints}

    def write_velocity(joint: JointEntry, velocity: float) -> None:
        raw = velocity_to_speed_register(velocity, joint.max_velocity_rad_s, inverted=joint.inverted)
        cache = last_written.setdefault(joint.id, {})
        apply_velocity_command(
            lambda: write_register(servo, joint.id, speed_goal_entry, raw, cache),
            joint.id,
            velocity,
            time.monotonic(),
            last_velocity_commands,
        )

    def make_on_command(group: JointGroup) -> Any:
        def on_command(msg: JointState) -> None:
            if servo is None or goal_entry is None:
                return
            for i, name in enumerate(msg.name):
                joint_entry = group.joint_entry_by_name(name)
                if joint_entry is None:
                    continue
                if joint_entry.mode == "velocity":
                    if joint_entry.id in velocity_ids and i < len(msg.velocity):
                        write_velocity(joint_entry, float(msg.velocity[i]))
                    continue
                if i >= len(msg.position):
                    continue
                sid = joint_entry.id
                position_val = float(msg.position[i])
                cmd_min, cmd_max = command_limits.get(sid, (MIN_STEPS, MAX_STEPS))
                # Backward-compatible default: if source range is not configured, keep raw pass-through behavior.
                if joint_entry.source_min_steps is None and joint_entry.source_max_steps is None:
                    target_steps = max(cmd_min, min(cmd_max, position_to_raw_steps(position_val, joint_entry.inverted)))
                else:
                    source_min = joint_entry.source_min_steps if joint_entry.source_min_steps is not None else 0
                    source_max = joint_entry.source_max_steps if joint_entry.source_max_steps is not None else 4095
                    target_steps = map_position_to_steps(
                        position_val,
                        source_min,
                        source_max,
                        cmd_min,
                        cmd_max,
                        source_inverted=joint_entry.source_inverted,
                    )
                write_register(servo, sid, goal_entry, target_steps, last_written.setdefault(sid, {}))

        return on_command

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
        # Reject EPROM writes from ROS: PID/current/t limits must be set once via calibrate_servos load-config.
        if entry.eprom:
            node.get_logger().warn(
                "set_register: rejecting EPROM register '%s'; set once via calibrate_servos load-config" % reg_name
            )
            return
        if sid not in last_written:
            last_written[sid] = {}
        if not write_register(servo, sid, entry, value, last_written[sid]):
            node.get_logger().warn(f"set_register: write failed for {joint_name}/{reg_name}={value}")

    for group in groups:
        node.create_subscription(
            JointState, f"/{group.namespace}/joint_commands", make_on_command(group), DEFAULT_QOS_DEPTH
        )
    node.create_subscription(String, f"/{config.namespace}/set_register", on_set_register, DEFAULT_QOS_DEPTH)

    def fallback_read(sid: int) -> tuple[int, int] | None:
        pos = read_register_raw(servo, sid, pos_entry) if pos_entry else None
        speed = read_register_raw(servo, sid, speed_entry) if speed_entry else None
        if pos is None or speed is None:
            return None
        return (pos, speed)

    def make_sync_group() -> Any:
        from st3215.group_sync_read import GroupSyncRead

        return GroupSyncRead(servo, PRESENT_POSITION_ADDRESS, SYNC_READ_LENGTH)

    def stop_velocity_joints(joints: list[JointEntry]) -> None:
        for joint in joints:
            write_velocity(joint, 0.0)

    register_dump = RegisterDumpScheduler([(j.name, j.id) for j in all_joints], config.register_publish_interval_s)
    last_missing_log = 0.0
    if servo is None:
        node.get_logger().error("No servo bus available: not publishing joint_states (no placeholder data).")

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            if servo is not None:
                # Velocity watchdog: stop wheels whose commands went stale.
                expired = expired_velocity_joints(
                    last_velocity_commands, time.monotonic(), config.velocity_command_timeout_s
                )
                if expired:
                    stop_velocity_joints([joint_by_id[sid] for sid in expired])
                readings = read_positions_and_speeds(expected_ids, make_sync_group, fallback_read)
                stamp = node.get_clock().now().to_msg()
                for group in groups:
                    missing = [j.name for j in group.joints if j.id not in readings]
                    if missing:
                        # Never publish placeholder values: skip this group's sample for this cycle.
                        now = time.monotonic()
                        if now - last_missing_log > MISSING_READ_LOG_INTERVAL_S:
                            last_missing_log = now
                            node.get_logger().warn(f"/{group.namespace}: read failed for {missing}; skipping sample")
                        continue
                    msg = JointState()
                    msg.header.stamp = stamp
                    msg.header.frame_id = ""
                    msg.name = list(group.joint_names)
                    msg.position = [steps_to_radians(readings[j.id][0], j.inverted) for j in group.joints]
                    msg.velocity = [speed_register_to_velocity(readings[j.id][1], j.inverted) for j in group.joints]
                    efforts: list[float] = []
                    for joint in group.joints:
                        load_val = None
                        if joint.name in effort_joint_set and load_entry:
                            load_val = read_register_raw(servo, joint.id, load_entry)
                        efforts.append(float(load_val) if load_val is not None else 0.0)
                    msg.effort = efforts if effort_joint_set else []
                    if config.publish_only_on_change:
                        changed = get_position_updates(
                            msg.name, msg.position, last_published_positions, config.publish_change_epsilon
                        )
                        if changed:
                            pub_state[group.namespace].publish(msg)
                    else:
                        pub_state[group.namespace].publish(msg)
                    if config.log_joint_updates:
                        # Velocity-mode (wheel) positions change every cycle while driving: not logged.
                        logged = [
                            (n, p) for n, p, j in zip(msg.name, msg.position, group.joints) if j.mode != "velocity"
                        ]
                        changing = get_position_updates([n for n, _ in logged], [p for _, p in logged], last_positions)
                        if changing:
                            line = ",".join(f"{name}:{val}" for name, val in changing)
                            print(line, flush=True)
                # Full register dump, one servo per cycle (0 interval = disabled) so the loop never stalls.
                dump = register_dump.step(time.monotonic(), lambda sid: read_all_registers(servo, sid))
                if dump is not None:
                    pub_registers.publish(String(data=json.dumps(dump, separators=(",", ":"))))

            executor.spin_once(timeout_sec=control_loop_sleep_s)
    finally:
        if servo is not None:
            stop_velocity_joints(velocity_joints)
        node.destroy_node()
        if rclpy.ok():
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
