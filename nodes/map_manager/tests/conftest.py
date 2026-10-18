"""Register the ROS stubs before any test module imports map_manager.node."""

from .ros_stubs import install_ros_stubs

install_ros_stubs()
