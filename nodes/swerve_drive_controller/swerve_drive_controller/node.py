"""ROS2 swerve drive controller: cmd_vel -> joint commands, odometry from FK."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from ros2_metrics import resolve_metrics_port, start_metrics_server
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .config import SwerveControllerConfig
from .control import CmdVelSample, ControlOutput, ControlState, JointSample, gate_cmd_vel, run_cycle
from .kinematics import odometry_twist_variances
from .tilt_guard import TiltGuard, TiltLevel, TiltTransition, drive_torque_payloads

NODE_NAME = "swerve_controller"
WAIT_LOG_INTERVAL_S = 5.0

CONTROL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
# Latched tilt guard state: a late subscriber (web UI, agent) gets the current level at once.
TILT_STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# Bridge torque_state is latched; match its QoS so a restarted controller learns the current state at once.
TORQUE_STATE_QOS = TILT_STATE_QOS
# REP 145: orientation_covariance[0] == -1 means the IMU provides no orientation estimate.
IMU_NO_ORIENTATION_COVARIANCE = -1.0
# After a torque-on restore the payloads are repeated on this many further heartbeats: the publish is volatile
# (a subscriber that was not matched yet misses it) and a lost restore leaves the drive motors limp.
TORQUE_RESTORE_REPEATS = 2


def joint_maps(msg: JointState) -> tuple[dict[str, float], dict[str, float]]:
    """Build name -> position and name -> velocity maps from a JointState (only fields actually present).

    Args:
        msg: JointState from the feetech bridge.

    Returns:
        tuple[dict[str, float], dict[str, float]]: (positions rad, velocities rad/s).
    """
    positions = {name: float(msg.position[i]) for i, name in enumerate(msg.name) if i < len(msg.position)}
    velocities = {name: float(msg.velocity[i]) for i, name in enumerate(msg.name) if i < len(msg.velocity)}
    return positions, velocities


def parse_released(data: str) -> bool | None:
    """Parse the "released" field of a bridge torque_state payload.

    Args:
        data: JSON text of the std_msgs/String.

    Returns:
        bool | None: The released flag, or None when the payload is not a JSON object with a boolean "released".
    """
    try:
        payload = json.loads(data)
    except ValueError:
        return None
    if not isinstance(payload, dict) or not isinstance(payload.get("released"), bool):
        return None
    return payload["released"]


def build_odometry(
    config: SwerveControllerConfig,
    output: ControlOutput,
    pose: tuple[float, float, float],
    stamp: object,
) -> Odometry:
    """Build the Odometry message for one control cycle.

    Args:
        config: Controller configuration (frame ids).
        output: Control step output (measured twist, moving flag).
        pose: (x, y, theta) in the odom frame.
        stamp: builtin_interfaces/Time stamp.

    Returns:
        Odometry: Message ready to publish.
    """
    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = config.odom_frame_id
    odom.child_frame_id = config.base_frame_id
    odom.pose.pose.position.x = pose[0]
    odom.pose.pose.position.y = pose[1]
    odom.pose.pose.position.z = 0.0
    q = _yaw_to_quaternion(pose[2])
    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]
    odom.twist.twist.linear.x = output.odom_twist[0]
    odom.twist.twist.linear.y = output.odom_twist[1]
    odom.twist.twist.angular.z = output.odom_twist[2]
    cov_xy, cov_yaw = odometry_twist_variances(
        output.odom_residual_mps, config.half_length_m, config.half_width_m, output.parked
    )
    odom.pose.covariance[0] = cov_xy
    odom.pose.covariance[7] = cov_xy
    odom.pose.covariance[35] = cov_yaw
    odom.twist.covariance[0] = cov_xy
    odom.twist.covariance[7] = cov_xy
    odom.twist.covariance[35] = cov_yaw
    return odom


def build_transform(
    config: SwerveControllerConfig, pose: tuple[float, float, float], stamp: object
) -> TransformStamped:
    """Build the odom -> base_link transform.

    Args:
        config: Controller configuration (frame ids).
        pose: (x, y, theta) in the odom frame.
        stamp: builtin_interfaces/Time stamp.

    Returns:
        TransformStamped: Transform ready to broadcast.
    """
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = config.odom_frame_id
    t.child_frame_id = config.base_frame_id
    t.transform.translation.x = pose[0]
    t.transform.translation.y = pose[1]
    t.transform.translation.z = 0.0
    q = _yaw_to_quaternion(pose[2])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t


def run_swerve_controller(config: SwerveControllerConfig) -> None:
    """Run the swerve controller node.

    A timer paces the control step at exactly control_loop_hz; subscription callbacks only store the latest
    message. Each cycle (only while joint states are fresh and complete) publishes ONE combined JointState
    (steer positions, drive velocities, NaN elsewhere) and /odom, plus the odom -> base_link TF when
    config.publish_tf is set.

    The tilt guard (config.tilt_guard) evaluates every /imu/data orientation and publishes the latched
    /tilt_guard/state JSON on every level change and at state_publish_hz. From entering ERROR until it is back at
    OK (an error falling to WARNING stays inhibited) every received cmd_vel is ignored and each cycle commands zero
    wheel speed with the steering held (error_action brake); with error_action torque_off the drive motors' torque
    is also switched off through the bridge set_register topic and switched back on when the inhibition clears.

    Args:
        config: Controller configuration.
    """
    start_metrics_server(resolve_metrics_port(config.metrics_port), NODE_NAME)
    rclpy.init()
    node = Node("swerve_drive_controller")
    clock = node.get_clock()
    logger = node.get_logger()

    pub_cmd = node.create_publisher(JointState, config.joint_commands_topic, CONTROL_QOS)
    pub_odom = node.create_publisher(Odometry, config.odom_topic, CONTROL_QOS)
    tf_broadcaster = TransformBroadcaster(node) if config.publish_tf else None
    tilt_cfg = config.tilt_guard
    guard = TiltGuard(tilt_cfg)
    pub_tilt_state = node.create_publisher(String, tilt_cfg.state_topic, TILT_STATE_QOS)
    pub_torque = (
        node.create_publisher(String, tilt_cfg.torque_topic, CONTROL_QOS)
        if tilt_cfg.error_action == "torque_off"
        else None
    )
    drive_joints = [config.joint_names[i] for i in (0, 2, 4, 6)]
    tilt_state_period_s = 1.0 / tilt_cfg.state_publish_hz
    last_tilt_state_time = 0.0
    imu_was_stale = False
    torque_disabled = False
    torque_synced = False  # True once the torque state was written (startup restore or a transition)
    torque_restore_repeats_left = 0
    last_torque_pub_time = 0.0
    start_time = time.monotonic()
    last_no_imu_log = start_time

    latest_cmd = CmdVelSample()
    latest_positions: dict[str, float] = {}
    latest_velocities: dict[str, float] = {}
    last_joint_states_time = 0.0
    state = ControlState()
    last_wait_log = 0.0
    released = False

    def on_cmd_vel(msg: Twist) -> None:
        nonlocal latest_cmd
        if released:
            return
        sample = CmdVelSample((msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic())
        latest_cmd = gate_cmd_vel(sample, guard.base_drive_inhibited)

    def on_torque_state(msg: String) -> None:
        nonlocal released, latest_cmd
        parsed = parse_released(msg.data)
        if parsed is None:
            logger.warn(f"ignoring unparsable torque_state payload on {config.torque_state_topic}: {msg.data[:200]!r}")
            return
        if parsed and not released:
            latest_cmd = CmdVelSample()  # never resume a command received before the release
            logger.warn("steering released (torque off): no steering is commanded until it is re-enabled")
        elif released and not parsed:
            logger.info("steering torque re-enabled: steering targets seeded from the measured positions")
        released = parsed

    def publish_tilt_state(now: float) -> None:
        nonlocal last_tilt_state_time
        state_dict = guard.state_dict(now, clock.now().nanoseconds)
        if state_dict is None:  # no IMU sample yet: nothing real to report
            return
        msg = String()
        msg.data = json.dumps(state_dict)
        pub_tilt_state.publish(msg)
        last_tilt_state_time = now

    def publish_torque(enable: bool, now: float) -> None:
        nonlocal torque_synced, last_torque_pub_time
        assert pub_torque is not None
        for payload in drive_torque_payloads(drive_joints, enable=enable):
            torque_msg = String()
            torque_msg.data = payload
            pub_torque.publish(torque_msg)
        torque_synced = True
        last_torque_pub_time = now

    def sync_torque(now: float) -> None:
        """Startup restore, heartbeat repeat of the torque-off writes and the repeats after a restore."""
        nonlocal torque_synced, torque_restore_repeats_left
        if pub_torque is None:
            return
        if not torque_synced:  # a restart: the previous run may have left the drive motors with torque off
            if not torque_disabled:
                publish_torque(True, now)
            torque_synced = True
            return
        if now - last_torque_pub_time < tilt_state_period_s:
            return
        if torque_disabled:
            publish_torque(False, now)
        elif torque_restore_repeats_left > 0:
            torque_restore_repeats_left -= 1
            publish_torque(True, now)

    def on_tilt_transition(transition: TiltTransition, now: float) -> None:
        nonlocal latest_cmd, torque_disabled, torque_restore_repeats_left
        if transition.current is TiltLevel.ERROR:
            logger.error(transition.message)
            latest_cmd = CmdVelSample()  # never resume a command received before the error
        elif transition.current is TiltLevel.WARNING:
            logger.warn(transition.message)
        else:
            logger.info(f"tilt guard: {transition.previous.name.lower()} cleared, {transition.message}")
        if pub_torque is not None and guard.base_drive_inhibited != torque_disabled:
            torque_disabled = guard.base_drive_inhibited
            torque_restore_repeats_left = 0 if torque_disabled else TORQUE_RESTORE_REPEATS
            publish_torque(not torque_disabled, now)
        publish_tilt_state(now)

    def on_imu(msg: Imu) -> None:
        if msg.orientation_covariance[0] == IMU_NO_ORIENTATION_COVARIANCE:
            return
        now = time.monotonic()
        q = msg.orientation
        transition = guard.update_from_quaternion(q.x, q.y, q.z, q.w, now)
        if transition is not None:
            on_tilt_transition(transition, now)

    def on_joint_states(msg: JointState) -> None:
        nonlocal last_joint_states_time
        positions, velocities = joint_maps(msg)
        latest_positions.update(positions)
        latest_velocities.update(velocities)
        last_joint_states_time = time.monotonic()

    def publish_cycle(output: ControlOutput, new_state: ControlState) -> None:
        stamp = clock.now().to_msg()
        cmd = JointState()
        cmd.header.stamp = stamp
        cmd.name = output.names
        cmd.position = output.positions
        cmd.velocity = output.velocities
        pub_cmd.publish(cmd)
        pub_odom.publish(build_odometry(config, output, new_state.pose, stamp))
        if tf_broadcaster is not None:
            tf_broadcaster.sendTransform(build_transform(config, new_state.pose, stamp))

    def on_timer() -> None:
        nonlocal state, last_wait_log, imu_was_stale, last_no_imu_log
        now = time.monotonic()
        imu_stale = guard.last_sample_s is not None and guard.imu_stale(now)
        if imu_stale != imu_was_stale:
            imu_was_stale = imu_stale
            if imu_stale:
                logger.warn(
                    f"tilt guard: IMU on {tilt_cfg.imu_topic} stale for > {tilt_cfg.imu_timeout_s:.1f} s, "
                    f"holding level {guard.level.name.lower()}"
                )
            else:
                logger.info(f"tilt guard: IMU on {tilt_cfg.imu_topic} fresh again")
        if now - last_tilt_state_time >= tilt_state_period_s:
            publish_tilt_state(now)
        sync_torque(now)
        if (
            guard.startup_pending
            and now - start_time > tilt_cfg.imu_timeout_s
            and now - last_no_imu_log >= WAIT_LOG_INTERVAL_S
        ):
            last_no_imu_log = now
            logger.warn(
                f"tilt guard: no IMU sample yet on {tilt_cfg.imu_topic} ({now - start_time:.0f} s after start), "
                "base drive inhibited until the first orientation is evaluated"
            )
        joints = JointSample(latest_positions, latest_velocities, last_joint_states_time)
        # Nothing is published without fresh, complete wheel state (the bridge watchdog stops the wheels).
        state, published = run_cycle(
            config,
            state,
            latest_cmd,
            joints,
            now,
            publish_cycle,
            inhibited=guard.base_drive_inhibited,
            released=released,
        )
        if not published and now - last_wait_log > WAIT_LOG_INTERVAL_S:
            last_wait_log = now
            logger.warn(f"Waiting for fresh joint states on {config.joint_states_topic}")

    node.create_subscription(Twist, config.cmd_vel_topic, on_cmd_vel, CONTROL_QOS)
    node.create_subscription(JointState, config.joint_states_topic, on_joint_states, CONTROL_QOS)
    if config.torque_state_topic:
        node.create_subscription(String, config.torque_state_topic, on_torque_state, TORQUE_STATE_QOS)
    node.create_subscription(Imu, tilt_cfg.imu_topic, on_imu, qos_profile_sensor_data)
    node.create_timer(1.0 / config.control_loop_hz, on_timer)

    logger.info(
        f"Swerve controller: {config.cmd_vel_topic} -> {config.joint_commands_topic}, odom -> {config.odom_topic} (Lx={config.half_length_m:.4f} Ly={config.half_width_m:.4f} R={config.wheel_radius_m:.3f}, steer limit {config.max_steer_angle_rad:.2f} rad, wheel max {config.max_wheel_angular_velocity_rad_s:.2f} rad/s, "
        f"{config.control_loop_hz:.0f} Hz, tf={config.publish_tf})"
    )
    logger.info(
        f"Tilt guard {'enabled' if tilt_cfg.enabled else 'DISABLED'}: {tilt_cfg.imu_topic} -> {tilt_cfg.state_topic}, "
        f"roll warn/error {tilt_cfg.roll_warn_deg:.1f}/{tilt_cfg.roll_error_deg:.1f} deg, pitch warn/error "
        f"{tilt_cfg.pitch_warn_deg:.1f}/{tilt_cfg.pitch_error_deg:.1f} deg, persistence {tilt_cfg.persistence_s:.1f} s, "
        f"hysteresis {tilt_cfg.hysteresis_deg:.1f} deg, hold {tilt_cfg.recovery_hold_s:.1f} s, "
        f"error_action {tilt_cfg.error_action}"
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():  # a publish racing the SIGTERM handler is shutdown noise; anything else is a bug
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Convert yaw (rad) to quaternion (x, y, z, w)."""
    c = math.cos(yaw / 2.0)
    s = math.sin(yaw / 2.0)
    return (0.0, 0.0, s, c)
