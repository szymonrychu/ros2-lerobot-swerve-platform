"""ROS2 swerve drive controller: cmd_vel -> joint commands, odometry from FK."""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from .config import SwerveControllerConfig
from .kinematics import forward_kinematics, inverse_kinematics, should_zero_drive

CONTROL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _joint_map(msg: JointState) -> dict[str, float]:
    """Build name -> position map; name -> velocity if present."""
    out: dict[str, float] = {}
    for i, name in enumerate(msg.name):
        if i < len(msg.position):
            out[name] = float(msg.position[i])
    return out


def _joint_map_velocity(msg: JointState) -> dict[str, float]:
    """Build name -> velocity map (0.0 if not in message)."""
    out: dict[str, float] = {}
    for i, name in enumerate(msg.name):
        if i < len(msg.velocity):
            out[name] = float(msg.velocity[i])
        else:
            out[name] = 0.0
    return out


def run_swerve_controller(config: SwerveControllerConfig) -> None:
    """Run the swerve controller node."""
    rclpy.init()
    node = Node("swerve_drive_controller")
    clock = node.get_clock()
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
    cmd_timeout_s = 0.5

    joint_positions: dict[str, float] = {}
    joint_velocities: dict[str, float] = {}
    last_joint_states_time: float = 0.0

    # Pose integration (odom frame)
    pose_x = 0.0
    pose_y = 0.0
    pose_theta = 0.0
    last_odom_time: float | None = None

    def on_cmd_vel(msg: Twist) -> None:
        nonlocal latest_cmd, latest_cmd_time
        latest_cmd = [msg.linear.x, msg.linear.y, msg.angular.z]
        latest_cmd_time = time.monotonic()

    def on_joint_states(msg: JointState) -> None:
        nonlocal joint_positions, joint_velocities, last_joint_states_time
        joint_positions = _joint_map(msg)
        joint_velocities = _joint_map_velocity(msg)
        last_joint_states_time = time.monotonic()

    node.create_subscription(Twist, config.cmd_vel_topic, on_cmd_vel, CONTROL_QOS)
    node.create_subscription(JointState, config.joint_states_topic, on_joint_states, CONTROL_QOS)

    control_period_s = 1.0 / max(1.0, config.control_loop_hz)
    node.get_logger().info(
        "Swerve controller: %s -> %s, odom -> %s"
        % (config.cmd_vel_topic, config.joint_commands_topic, config.odom_topic)
    )

    while rclpy.ok():
        now = time.monotonic()
        stamp = clock.now()

        # Timeout cmd_vel: stop if no command recently
        if now - latest_cmd_time > cmd_timeout_s:
            latest_cmd = [0.0, 0.0, 0.0]

        vx, vy, omega = latest_cmd[0], latest_cmd[1], latest_cmd[2]

        # Build current steer angles and drive velocities from joint_states (for FK and safeguard)
        steer_angles: list[float] = []
        drive_velocities: list[float] = []
        for i in range(4):
            steer_angles.append(joint_positions.get(steer_joints[i], 0.0))
            drive_velocities.append(joint_velocities.get(drive_joints[i], 0.0))

        # Inverse kinematics: desired steer and drive
        desired_steer, desired_drive_angular = inverse_kinematics(vx, vy, omega, lx, ly, R)

        # No-propulsion safeguard: zero drive for a wheel if steer error too large
        for i in range(4):
            if should_zero_drive(
                steer_angles[i],
                desired_steer[i],
                config.steer_error_threshold_rad,
            ):
                desired_drive_angular[i] = 0.0

        # Publish joint commands: positions for steer (rad), positions for drive (integrated from velocity)
        # Feetech bridge expects position in rad. For drive we send target position; the bridge will
        # move toward it. So we can send current_position + velocity * dt for drive, or just
        # position setpoints. Actually the bridge only accepts position (goal_position in steps).
        # So we need to convert desired_drive_angular to position delta: we integrate.
        # For simplicity: publish steer target (desired_steer) and drive target as current + omega_drive * dt.
        out = JointState()
        out.header.stamp = stamp.to_msg()
        out.header.frame_id = ""
        out.name = list(joint_names)
        out.position = []
        out.velocity = []
        out.effort = []
        dt = control_period_s
        for i in range(4):
            drive_pos = joint_positions.get(drive_joints[i], 0.0) + desired_drive_angular[i] * dt
            out.position.append(drive_pos)
            out.position.append(desired_steer[i])
        pub_cmd.publish(out)

        # Forward kinematics and odometry
        if len(steer_angles) == 4 and len(drive_velocities) == 4:
            vx_fk, vy_fk, omega_fk = forward_kinematics(steer_angles, drive_velocities, lx, ly, R)
            if last_odom_time is not None:
                dt_odom = now - last_odom_time
                pose_theta += omega_fk * dt_odom
                pose_x += (vx_fk * math.cos(pose_theta) - vy_fk * math.sin(pose_theta)) * dt_odom
                pose_y += (vx_fk * math.sin(pose_theta) + vy_fk * math.cos(pose_theta)) * dt_odom
            last_odom_time = now

            odom = Odometry()
            odom.header.stamp = stamp.to_msg()
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
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = omega_fk
            pub_odom.publish(odom)

            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
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

        rclpy.spin_once(node, timeout_sec=0.001)
        time.sleep(control_period_s)

    node.destroy_node()
    rclpy.shutdown()


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Convert yaw (rad) to quaternion (x, y, z, w)."""
    c = math.cos(yaw / 2.0)
    s = math.sin(yaw / 2.0)
    return (0.0, 0.0, s, c)
