"""Register the rclpy stubs (with the shutdown exception classes) before any test module imports map_guard.node."""

from .ros_stubs import install_ros_stubs

install_ros_stubs()
