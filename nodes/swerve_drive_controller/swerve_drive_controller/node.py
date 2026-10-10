"""ROS2 swerve drive controller: cmd_vel -> joint commands, odometry from FK."""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros2_metrics import resolve_metrics_port, start_metrics_server
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from .config import SwerveControllerConfig
from .control import CmdVelSample, ControlOutput, ControlState, JointSample, run_cycle
from .kinematics import odometry_twist_variances

NODE_NAME = "swerve_controller"
WAIT_LOG_INTERVAL_S = 5.0

CONTROL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


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

    latest_cmd = CmdVelSample()
    latest_positions: dict[str, float] = {}
    latest_velocities: dict[str, float] = {}
    last_joint_states_time = 0.0
    state = ControlState()
    last_wait_log = 0.0

    def on_cmd_vel(msg: Twist) -> None:
        nonlocal latest_cmd
        latest_cmd = CmdVelSample((msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic())

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
        nonlocal state, last_wait_log
        now = time.monotonic()
        joints = JointSample(latest_positions, latest_velocities, last_joint_states_time)
        # Nothing is published without fresh, complete wheel state (the bridge watchdog stops the wheels).
        state, published = run_cycle(config, state, latest_cmd, joints, now, publish_cycle)
        if not published and now - last_wait_log > WAIT_LOG_INTERVAL_S:
            last_wait_log = now
            logger.warn(f"Waiting for fresh joint states on {config.joint_states_topic}")

    node.create_subscription(Twist, config.cmd_vel_topic, on_cmd_vel, CONTROL_QOS)
    node.create_subscription(JointState, config.joint_states_topic, on_joint_states, CONTROL_QOS)
    node.create_timer(1.0 / config.control_loop_hz, on_timer)

    logger.info(
        f"Swerve controller: {config.cmd_vel_topic} -> {config.joint_commands_topic}, odom -> {config.odom_topic} (Lx={config.half_length_m:.4f} Ly={config.half_width_m:.4f} R={config.wheel_radius_m:.3f}, steer limit {config.max_steer_angle_rad:.2f} rad, wheel max {config.max_wheel_angular_velocity_rad_s:.2f} rad/s, "
        f"{config.control_loop_hz:.0f} Hz, tf={config.publish_tf})"
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Convert yaw (rad) to quaternion (x, y, z, w)."""
    c = math.cos(yaw / 2.0)
    s = math.sin(yaw / 2.0)
    return (0.0, 0.0, s, c)
