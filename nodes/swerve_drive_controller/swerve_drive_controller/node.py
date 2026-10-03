"""ROS2 swerve drive controller: cmd_vel -> joint commands, odometry from FK."""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from .config import SwerveControllerConfig
from .kinematics import compute_wheel_commands, forward_kinematics, integrate_odometry, should_zero_drive, wheel_states

CMD_VEL_DEADBAND = 0.005  # m/s and rad/s
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


def run_swerve_controller(config: SwerveControllerConfig) -> None:
    """Run the swerve controller node.

    Each control cycle (only while joint states are fresh and complete): IK for the latest cmd_vel ->
    steering positions (JointState.position) and drive velocities (JointState.velocity) to the bridge;
    FK of the measured wheel states -> /odom and the odom -> base_link TF.

    Args:
        config: Controller configuration.
    """
    rclpy.init()
    node = Node("swerve_drive_controller")
    clock = node.get_clock()
    logger = node.get_logger()
    lx = config.half_length_m
    ly = config.half_width_m
    R = config.wheel_radius_m
    joint_names = config.joint_names
    # Expect 8 joints: fl_drive, fl_steer, fr_drive, fr_steer, rl_drive, rl_steer, rr_drive, rr_steer.
    steer_joints = [joint_names[1], joint_names[3], joint_names[5], joint_names[7]]
    drive_joints = [joint_names[0], joint_names[2], joint_names[4], joint_names[6]]

    pub_cmd = node.create_publisher(JointState, config.joint_commands_topic, CONTROL_QOS)
    pub_odom = node.create_publisher(Odometry, config.odom_topic, CONTROL_QOS)
    tf_broadcaster = TransformBroadcaster(node)

    latest_cmd: list[float] = [0.0, 0.0, 0.0]  # vx, vy, omega
    latest_cmd_time: float = 0.0
    joint_positions: dict[str, float] = {}
    joint_velocities: dict[str, float] = {}
    last_joint_states_time: float = 0.0
    steer_targets: list[float] | None = None  # last commanded steer; held while stopped

    pose = (0.0, 0.0, 0.0)  # x, y, theta in odom frame
    last_odom_time: float | None = None
    last_wait_log = 0.0

    def on_cmd_vel(msg: Twist) -> None:
        nonlocal latest_cmd, latest_cmd_time
        latest_cmd = [msg.linear.x, msg.linear.y, msg.angular.z]
        latest_cmd_time = time.monotonic()

    def on_joint_states(msg: JointState) -> None:
        nonlocal last_joint_states_time
        positions, velocities = joint_maps(msg)
        joint_positions.update(positions)
        joint_velocities.update(velocities)
        last_joint_states_time = time.monotonic()

    node.create_subscription(Twist, config.cmd_vel_topic, on_cmd_vel, CONTROL_QOS)
    node.create_subscription(JointState, config.joint_states_topic, on_joint_states, CONTROL_QOS)

    control_period_s = 1.0 / max(1.0, config.control_loop_hz)
    logger.info(
        "Swerve controller: %s -> %s, odom -> %s (Lx=%.4f Ly=%.4f R=%.3f, steer limit %.2f rad, wheel max %.2f rad/s)"
        % (
            config.cmd_vel_topic,
            config.joint_commands_topic,
            config.odom_topic,
            lx,
            ly,
            R,
            config.max_steer_angle_rad,
            config.max_wheel_angular_velocity_rad_s,
        )
    )

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    while rclpy.ok():
        executor.spin_once(timeout_sec=control_period_s)
        now = time.monotonic()
        measured = wheel_states(joint_positions, joint_velocities, steer_joints, drive_joints)
        if measured is None or now - last_joint_states_time > config.joint_states_timeout_s:
            # No fresh, complete wheel state: publish nothing (bridge watchdog stops the wheels).
            if now - last_wait_log > WAIT_LOG_INTERVAL_S:
                last_wait_log = now
                logger.warn("Waiting for fresh joint states on %s" % config.joint_states_topic)
            last_odom_time = None
            continue
        steer_angles, drive_velocities = measured
        stamp = clock.now().to_msg()

        vx, vy, omega = latest_cmd if now - latest_cmd_time <= config.cmd_vel_timeout_s else (0.0, 0.0, 0.0)
        # Velocity deadband: suppress tiny commands to avoid servo jitter
        if abs(vx) < CMD_VEL_DEADBAND and abs(vy) < CMD_VEL_DEADBAND and abs(omega) < CMD_VEL_DEADBAND:
            vx, vy, omega = 0.0, 0.0, 0.0

        if steer_targets is None:
            steer_targets = list(steer_angles)
        desired_steer, desired_drive = compute_wheel_commands(
            vx,
            vy,
            omega,
            steer_targets,
            lx,
            ly,
            R,
            config.max_steer_angle_rad,
            config.max_wheel_angular_velocity_rad_s,
        )
        steer_targets = desired_steer
        # No-propulsion safeguard: zero drive for a wheel until its steering has caught up.
        for i in range(4):
            if should_zero_drive(steer_angles[i], desired_steer[i], config.steer_error_threshold_rad):
                desired_drive[i] = 0.0

        steer_cmd = JointState()
        steer_cmd.header.stamp = stamp
        steer_cmd.name = list(steer_joints)
        steer_cmd.position = list(desired_steer)
        pub_cmd.publish(steer_cmd)
        drive_cmd = JointState()
        drive_cmd.header.stamp = stamp
        drive_cmd.name = list(drive_joints)
        drive_cmd.velocity = list(desired_drive)
        pub_cmd.publish(drive_cmd)

        # Forward kinematics and odometry from measured wheel states.
        vx_fk, vy_fk, omega_fk = forward_kinematics(steer_angles, drive_velocities, lx, ly, R)
        if last_odom_time is not None:
            pose = integrate_odometry(pose, (vx_fk, vy_fk, omega_fk), now - last_odom_time)
        last_odom_time = now
        pose_x, pose_y, pose_theta = pose

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = config.odom_frame_id
        odom.child_frame_id = config.base_frame_id
        odom.pose.pose.position.x = pose_x
        odom.pose.pose.position.y = pose_y
        odom.pose.pose.position.z = 0.0
        q = _yaw_to_quaternion(pose_theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = vx_fk
        odom.twist.twist.linear.y = vy_fk
        odom.twist.twist.angular.z = omega_fk
        is_moving = abs(vx) > CMD_VEL_DEADBAND or abs(vy) > CMD_VEL_DEADBAND or abs(omega) > CMD_VEL_DEADBAND
        cov_xy = 0.01 if is_moving else 0.001
        cov_yaw = 0.01 if is_moving else 0.0001
        odom.pose.covariance[0] = cov_xy
        odom.pose.covariance[7] = cov_xy
        odom.pose.covariance[35] = cov_yaw
        odom.twist.covariance[0] = cov_xy
        odom.twist.covariance[7] = cov_xy
        odom.twist.covariance[35] = cov_yaw
        pub_odom.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = config.odom_frame_id
        t.child_frame_id = config.base_frame_id
        t.transform.translation.x = pose_x
        t.transform.translation.y = pose_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        tf_broadcaster.sendTransform(t)

    node.destroy_node()
    rclpy.shutdown()


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Convert yaw (rad) to quaternion (x, y, z, w)."""
    c = math.cos(yaw / 2.0)
    s = math.sin(yaw / 2.0)
    return (0.0, 0.0, s, c)
