"""Feetech servos bridge: publish joint_states, subscribe joint_commands, read/write servo registers.

Configuration is loaded from a YAML file (path via FEETECH_SERVOS_CONFIG or default).
When device is set: connects to hardware, reads all registers at startup (prints one-line JSON per servo),
publishes joint_states and servo_registers (JSON), subscribes to joint_commands and set_register.
EPROM writes use unlock -> write -> lock; writes are skipped when value unchanged.
When device is not set or cannot be opened: nothing is published (no placeholder joint_states).
Multiple namespaces (extra_groups) can share one bus; velocity-mode joints run in wheel mode.
Filtering/smoothing of joint_commands is handled by a separate filter node; this bridge applies commands directly.
Every loop iteration drains all pending ROS callbacks (bounded; command callbacks only record the latest target per
joint), writes the changed goal positions in one sync write (wheel speeds one acknowledged write each), then runs
the watchdog and one state sync read (position, speed and, for publish_effort_joints, present load), then sleeps the
remainder of the control period. Per-cycle logic lives in bridge_cycle.BridgeCycle (no rclpy).
Unchanged goal positions are rewritten every goal_refresh_interval_s (a sync write has no reply). A sync read keeps
every reply that arrived; a servo failing read_failure_limit cycles in a row is left out of it and probed every
read_retry_interval_s. Each joint_states header is stamped with the request time of the group's oldest reading.
"""

import json
import sys
import time
from typing import Any

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from ros2_metrics import resolve_metrics_port, start_metrics_server
from sensor_msgs.msg import BatteryState, JointState
from st3215.group_sync_read import GroupSyncRead
from std_msgs.msg import String

from .battery import BatteryMonitor, battery_fields
from .bridge_cycle import MAX_STEPS, MIN_STEPS, BridgeCycle, drain_callbacks, end_cycle, split_stamp_ns
from .command_mapping import speed_register_to_velocity, steps_to_radians
from .config import BridgeConfig, JointGroup, load_config_from_env
from .joint_updates import get_position_updates
from .metrics import BUS_UP, init_joints, record_read_cycle, record_register_dump
from .register_dump import RegisterDumpScheduler
from .registers import (
    get_register_entry_by_name,
    read_all_registers,
    write_register,
)
from .registers import read_register as read_register_raw
from .serial_wait import use_blocking_reads
from .set_register import apply_set_register
from .startup_torque import hold_current_positions, set_startup_torque_state
from .sync_read import (
    PRESENT_POSITION_ADDRESS,
    ReadBackoff,
    ServoState,
    group_stamp_ns,
    joint_currents,
    joint_efforts,
    read_servo_state,
    read_servo_states,
    sync_read_length,
)

DEFAULT_QOS_DEPTH = 10
SERVO_WAIT_INTERVAL_S = 1.0
TORQUE_WRITE_ATTEMPTS = 8
TORQUE_VERIFY_SLEEP_S = 0.02
WHEEL_MODE = 1  # STS mode register: 0 = position servo, 1 = continuous rotation (wheel)
MISSING_READ_LOG_INTERVAL_S = 5.0
DEFAULT_METRICS_NODE_NAME = (
    "lerobot_follower"  # ros2_nodes name on the robot; the server runs this code as lerobot_leader
)
DEFAULT_ROS_NODE_NAME = "feetech_servos_bridge"


def _wait_for_servos(servo: Any, expected_ids: list[int], node: Node) -> None:
    """Block until all expected servo IDs are present on the bus. Log and print missing IDs every 1s.

    Raises ExternalShutdownException if rclpy is shut down before all servos appear (so we never run with missing
    servos; run_bridge treats it as a normal stop).
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
    raise ExternalShutdownException()


def run_bridge(config: BridgeConfig) -> None:
    """Run the bridge: per-group joint_states / joint_commands over one serial bus.

    Position-mode joints are driven by JointState.position (goal_position); velocity-mode joints run in
    wheel mode and are driven by JointState.velocity (goal_speed, rad/s). Velocity joints are stopped when
    commands go stale (velocity_command_timeout_s) and on shutdown.
    """
    # Port unset (the server's lerobot_leader) disables the exporter; the registered metrics are then never served.
    start_metrics_server(resolve_metrics_port(config.metrics_port), config.node_name or DEFAULT_METRICS_NODE_NAME)
    rclpy.init()
    node = Node(config.node_name or DEFAULT_ROS_NODE_NAME)
    try:
        serve_bridge(config, node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():  # a publish or wait set racing the SIGTERM handler is shutdown noise; anything else is a bug
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def serve_bridge(config: BridgeConfig, node: Node) -> None:
    """Set the servos up, then run the bridge loop until the rclpy context is shut down.

    Velocity joints are stopped on every way out of the loop (the finally block below), before the node is destroyed.

    Args:
        config (BridgeConfig): Bridge configuration.
        node (Node): The rclpy node to attach publishers and subscriptions to.
    """
    control_loop_period_s = 1.0 / max(1.0, config.control_loop_hz)
    groups = config.groups
    all_joints = config.all_joints
    velocity_joints = [j for j in all_joints if j.mode == "velocity"]
    init_joints([j.name for j in all_joints])
    metric_joints = [(j.name, j.id) for j in all_joints]
    joint_inverted = {j.name: j.inverted for j in all_joints}

    pub_state = {
        g.namespace: node.create_publisher(JointState, f"/{g.namespace}/joint_states", DEFAULT_QOS_DEPTH)
        for g in groups
    }
    pub_currents = (
        node.create_publisher(JointState, f"/{config.namespace}/joint_currents", DEFAULT_QOS_DEPTH)
        if config.publish_current_joints
        else None
    )
    pub_registers = node.create_publisher(String, f"/{config.namespace}/servo_registers", DEFAULT_QOS_DEPTH)

    pub_battery = (
        node.create_publisher(BatteryState, config.battery_topic, DEFAULT_QOS_DEPTH)
        if config.battery_interval_s > 0
        else None
    )

    last_positions: dict[str, float] = {}
    last_published_positions: dict[str, float] = {}  # for publish_only_on_change gate
    last_written: dict[int, dict[str, int]] = {}  # servo_id -> { register_name: value }
    command_limits: dict[int, tuple[int, int]] = {}  # servo_id -> (cmd_min, cmd_max)
    servo: Any = None
    torque_on_ids: set[int] = set()  # servos holding a verified torque_enable=1 after the startup sequence

    if config.device:
        try:
            from st3215 import ST3215

            servo = ST3215(config.device)
            use_blocking_reads(servo)  # the library opens the port non-blocking and busy-waits on reads
        except (ImportError, ValueError, OSError) as e:
            node.get_logger().error(f"Failed to open device {config.device}: {e}. No joint_states will be published.")
            servo = None

    goal_entry = get_register_entry_by_name("goal_position")
    speed_goal_entry = get_register_entry_by_name("goal_speed")
    mode_entry = get_register_entry_by_name("mode")
    pos_entry = get_register_entry_by_name("present_position")
    voltage_entry = get_register_entry_by_name("present_voltage")
    effort_joint_set = set(config.publish_effort_joints) if config.publish_effort_joints else set()
    current_joint_set = set(config.publish_current_joints)
    # present_load and present_current follow present_position/present_speed, so they only lengthen the one state sync
    # read (one length for every servo; current is decoded and published only for current_joint_set).
    state_read_length = sync_read_length(with_load=bool(effort_joint_set), with_current=bool(current_joint_set))

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
                if torque_value == 1:
                    torque_on_ids = set(joint_ids) - set(failed_ids)
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

    cycle = BridgeCycle(
        servo=servo,
        goal_entry=goal_entry,
        speed_goal_entry=speed_goal_entry,
        velocity_joints=velocity_joints,
        command_limits=command_limits,
        last_written=last_written,
        velocity_command_timeout_s=config.velocity_command_timeout_s,
        direct_command_sources=config.direct_command_sources,
        goal_refresh_interval_s=config.goal_refresh_interval_s,
    )
    if servo is not None:
        cycle.queue_startup_goals(all_joints, torque_on_ids, node.get_logger().info)
    read_backoff = ReadBackoff(config.read_failure_limit, config.read_retry_interval_s)

    def clock_ns() -> int:
        return node.get_clock().now().nanoseconds

    callbacks_run = [0]  # incremented by every subscription callback; lets the loop tell when the queue is empty

    def make_on_command(group: JointGroup) -> Any:
        def on_command(msg: JointState) -> None:
            callbacks_run[0] += 1
            cycle.handle_command(group, msg.name, msg.position, msg.velocity, source=msg.header.frame_id)

        return on_command

    def on_set_register(msg: String) -> None:
        callbacks_run[0] += 1
        apply_set_register(servo, config, last_written, msg.data, node.get_logger().warn)

    for group in groups:
        node.create_subscription(
            JointState, f"/{group.namespace}/joint_commands", make_on_command(group), DEFAULT_QOS_DEPTH
        )
    node.create_subscription(String, f"/{config.namespace}/set_register", on_set_register, DEFAULT_QOS_DEPTH)

    def fallback_read(sid: int) -> ServoState | None:
        return read_servo_state(servo, sid, state_read_length)

    def make_sync_group() -> GroupSyncRead:
        return GroupSyncRead(servo, PRESENT_POSITION_ADDRESS, state_read_length)

    register_dump = RegisterDumpScheduler([(j.name, j.id) for j in all_joints], config.register_publish_interval_s)
    battery = BatteryMonitor([j.id for j in all_joints], config.battery_interval_s, config.battery_stale_s)
    last_missing_log = 0.0
    BUS_UP.set(0.0)
    if servo is None:
        node.get_logger().error("No servo bus available: not publishing joint_states (no placeholder data).")

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_ready() -> bool:
        before = callbacks_run[0]
        executor.spin_once(timeout_sec=0.0)
        return callbacks_run[0] != before

    try:
        while rclpy.ok():
            cycle_start = time.monotonic()
            # Drain every pending callback so no command is starved behind another (see bridge_cycle).
            drain_callbacks(spin_ready)
            if servo is not None:
                # Newest target per joint: changed goal positions in one sync write; superseded targets never sent.
                cycle.write_pending_commands()
                # Velocity watchdog: stop wheels with no drive command within velocity_command_timeout_s.
                cycle.stop_expired()
                readings = read_servo_states(
                    expected_ids, make_sync_group, fallback_read, state_read_length, read_backoff, clock_ns
                )
                record_read_cycle(metric_joints, readings)
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
                    # Stamp of the oldest reading: a fallback read is later than the sync read it follows.
                    stamp_ns = group_stamp_ns(group.joints, readings)
                    msg.header.stamp.sec, msg.header.stamp.nanosec = split_stamp_ns(
                        stamp_ns if stamp_ns is not None else clock_ns()
                    )
                    msg.header.frame_id = ""
                    msg.name = list(group.joint_names)
                    msg.position = [steps_to_radians(readings[j.id].position, j.inverted) for j in group.joints]
                    msg.velocity = [speed_register_to_velocity(readings[j.id].speed, j.inverted) for j in group.joints]
                    msg.effort = joint_efforts(group.joints, readings, effort_joint_set)
                    currents = (
                        joint_currents(group.joints, readings, current_joint_set)
                        if pub_currents is not None and group.namespace == config.namespace
                        else None
                    )
                    if currents is not None:
                        # Same stamp as joint_states; published only when every listed current was read this cycle.
                        current_msg = JointState()
                        current_msg.header.stamp = msg.header.stamp
                        current_msg.header.frame_id = ""
                        current_msg.name = [j.name for j in group.joints if j.name in current_joint_set]
                        current_msg.effort = currents
                        pub_currents.publish(current_msg)
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
                    record_register_dump(dump, joint_inverted)
                    pub_registers.publish(String(data=json.dumps(dump, separators=(",", ":"))))

                # Pack voltage: one servo per interval; nothing is published without a fresh reading.
                if voltage_entry is not None and pub_battery is not None:
                    pack_voltage = battery.step(
                        time.monotonic(), lambda sid: read_register_raw(servo, sid, voltage_entry)
                    )
                    if pack_voltage is not None:
                        battery_msg = BatteryState()
                        battery_msg.header.stamp = node.get_clock().now().to_msg()
                        battery_msg.header.frame_id = config.battery_frame_id
                        for name, value in battery_fields(pack_voltage, config.battery_cells).items():
                            setattr(battery_msg, name, value)
                        pub_battery.publish(battery_msg)

            time.sleep(end_cycle(control_loop_period_s, time.monotonic() - cycle_start))
    finally:
        if servo is not None:
            cycle.stop(velocity_joints)


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
