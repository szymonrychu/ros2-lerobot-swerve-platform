#!/usr/bin/env python3
"""Launch slam_toolbox online async mapping as a self-activating lifecycle node.

Params come from config/slam_params.yaml next to this launch file (override with env SLAM_TOOLBOX_PARAMS).
If a saved posegraph <map base>.posegraph exists (default /var/lib/ros2/maps/slam_map, override with env
SLAM_TOOLBOX_MAP_BASE), it is loaded with map_start_at_dock so mapping continues on the saved map;
otherwise a fresh map is started. Save a map with the /slam_toolbox/serialize_map service (filename = map base).
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

PARAMS_ENV = "SLAM_TOOLBOX_PARAMS"
MAP_BASE_ENV = "SLAM_TOOLBOX_MAP_BASE"
DEFAULT_PARAMS_PATH = str(Path(__file__).resolve().parent.parent / "config" / "slam_params.yaml")
DEFAULT_MAP_BASE = "/var/lib/ros2/maps/slam_map"


def map_resume_parameters(map_base: Path) -> dict:
    """Parameters that make slam_toolbox continue a saved posegraph, if one exists.

    Args:
        map_base: Map path without extension (slam_toolbox writes <base>.posegraph and <base>.data).

    Returns:
        dict: {"map_file_name": str, "map_start_at_dock": True} when <base>.posegraph exists, else {}.
    """
    if not map_base.with_suffix(".posegraph").is_file():
        return {}
    return {"map_file_name": str(map_base), "map_start_at_dock": True}


def generate_launch_description() -> LaunchDescription:
    """Build the slam_toolbox launch description.

    Returns:
        LaunchDescription: The async_slam_toolbox_node plus configure and activate transitions.
    """
    params_path = os.environ.get(PARAMS_ENV, DEFAULT_PARAMS_PATH)
    map_base = Path(os.environ.get(MAP_BASE_ENV, DEFAULT_MAP_BASE))
    resume = map_resume_parameters(map_base)
    status = f"resuming saved posegraph {map_base}" if resume else f"no posegraph at {map_base}, starting a fresh map"

    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[params_path, {"use_lifecycle_manager": False, "use_sim_time": False, **resume}],
    )
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_after_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="slam_toolbox configured, activating"),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )
    return LaunchDescription(
        [
            LogInfo(msg=f"slam_toolbox: {status}"),
            slam_node,
            activate_after_configure,
            configure,
        ]
    )
