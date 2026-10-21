#!/usr/bin/env python3
"""Launch Nav2 navigation (repo navigation_launch.py, no localization) plus a second, namespaced docking server for grasp approaches.

Why a second server: Jazzy's navigation_launch.py does not remap docking_server cmd_vel, so the stock /docking_server
publishes straight to /cmd_vel and bypasses velocity_smoother and collision_monitor. /grasp/docking_server publishes
to /cmd_vel_nav instead (velocity_smoother -> collision_monitor -> /cmd_vel) and uses config/grasp_docking_params.yaml.
The stock docking_server section of nav2_params.yaml is untouched.

The stock nav2_bringup bringup launch file is not included because it includes the stock navigation launch; this wrapper starts the
same component container and includes the repo copy (bond_timeout 0, no route/waypoint/stock docking servers). When the
container dies the whole launch shuts down (non-zero exit) so systemd Restart= restarts nav2.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
NAV2_PARAMS = str(CONFIG_DIR / "nav2_params.yaml")
GRASP_DOCKING_PARAMS = str(CONFIG_DIR / "grasp_docking_params.yaml")
NAVIGATION_LAUNCH = str(Path(__file__).resolve().parent / "navigation_launch.py")


def generate_launch_description() -> LaunchDescription:
    """Build the launch description: container, navigation and the grasp docking group.

    Returns:
        LaunchDescription: Nav2 container, repo navigation servers and /grasp/docking_server with its lifecycle manager.
    """
    container = Node(
        package="rclcpp_components",
        executable="component_container_isolated",
        name="nav2_container",
        output="screen",
        parameters=[NAV2_PARAMS, {"autostart": True}],
        arguments=["--ros-args", "--log-level", "info"],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        on_exit=Shutdown(reason="nav2_container exited"),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(NAVIGATION_LAUNCH),
        launch_arguments={
            "params_file": NAV2_PARAMS,
            "use_composition": "True",
            "container_name": "nav2_container",
            "autostart": "True",
        }.items(),
    )
    grasp_docking = GroupAction(
        [
            PushRosNamespace("grasp"),
            Node(
                package="opennav_docking",
                executable="opennav_docking",
                name="docking_server",
                output="screen",
                parameters=[GRASP_DOCKING_PARAMS],
                remappings=[("cmd_vel", "/cmd_vel_nav"), ("navigate_to_pose", "/navigate_to_pose")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_grasp",
                output="screen",
                parameters=[{"autostart": True, "node_names": ["docking_server"], "bond_timeout": 0.0}],
            ),
        ]
    )
    return LaunchDescription([container, navigation, grasp_docking])
