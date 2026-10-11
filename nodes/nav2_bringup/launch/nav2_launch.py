#!/usr/bin/env python3
"""Launch the stock Nav2 bringup plus a second, namespaced docking server for grasp approaches.

Why a second server: Jazzy's navigation_launch.py does not remap docking_server cmd_vel, so the stock /docking_server
publishes straight to /cmd_vel and bypasses velocity_smoother and collision_monitor. /grasp/docking_server publishes
to /cmd_vel_nav instead (velocity_smoother -> collision_monitor -> /cmd_vel) and uses config/grasp_docking_params.yaml.
The stock docking_server section of nav2_params.yaml is untouched.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
NAV2_PARAMS = str(CONFIG_DIR / "nav2_params.yaml")
GRASP_DOCKING_PARAMS = str(CONFIG_DIR / "grasp_docking_params.yaml")


def generate_launch_description() -> LaunchDescription:
    """Build the launch description: stock bringup and the grasp docking group.

    Returns:
        LaunchDescription: Stock nav2 bringup (no localization) and /grasp/docking_server with its lifecycle manager.
    """
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("nav2_bringup")) / "launch" / "bringup_launch.py")
        ),
        launch_arguments={"use_localization": "False", "params_file": NAV2_PARAMS}.items(),
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
                parameters=[{"autostart": True, "node_names": ["docking_server"]}],
            ),
        ]
    )
    return LaunchDescription([bringup, grasp_docking])
