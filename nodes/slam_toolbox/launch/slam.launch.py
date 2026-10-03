#!/usr/bin/env python3
"""Launch slam_toolbox online async mapping as a self-activating lifecycle node.

Params: config/slam_params.yaml next to this launch file is loaded first as the defaults (override the path with env
SLAM_TOOLBOX_PARAMS), then the deployed config /etc/ros2/slam_toolbox/config.yaml (env SLAM_TOOLBOX_CONFIG, written
by Ansible from the ros2_nodes `config: |` block) as overrides when it exists and is non-empty. ROS params files are
applied in order, so the deployed values win.

Map base (posegraph path without extension): env SLAM_TOOLBOX_MAP_BASE if set, else `map_file_name` from the params
files (last file wins), else /var/lib/ros2/maps/slam_map. If <map base>.posegraph exists it is loaded with
map_start_at_dock so mapping continues on the saved map; otherwise map_file_name is blanked and a fresh map is
started. Save a map with the /slam_toolbox/serialize_map service (filename = map base).
"""

import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

PARAMS_ENV = "SLAM_TOOLBOX_PARAMS"
CONFIG_ENV = "SLAM_TOOLBOX_CONFIG"
MAP_BASE_ENV = "SLAM_TOOLBOX_MAP_BASE"
DEFAULT_PARAMS_PATH = str(Path(__file__).resolve().parent.parent / "config" / "slam_params.yaml")
DEFAULT_CONFIG_PATH = "/etc/ros2/slam_toolbox/config.yaml"
DEFAULT_MAP_BASE = "/var/lib/ros2/maps/slam_map"
NODE_PARAM_KEYS = ("slam_toolbox", "/**")


def params_files(defaults_path: Path, override_path: Path) -> list[str]:
    """Params files for the node, in load order (later files override earlier ones).

    Args:
        defaults_path: Repo defaults (config/slam_params.yaml).
        override_path: Deployed overrides; skipped when missing or empty (Ansible writes an empty file for an
            empty config block, and an empty params file is not a valid ROS params file).

    Returns:
        list[str]: [defaults] or [defaults, override].
    """
    files = [str(defaults_path)]
    if override_path.is_file() and override_path.stat().st_size > 0:
        files.append(str(override_path))
    return files


def configured_map_base(param_files: list[str], env_map_base: str | None) -> Path:
    """Map base path (posegraph path without extension) from the environment or the params files.

    Args:
        param_files: Params files in load order; `map_file_name` under `slam_toolbox` or `/**` ros__parameters
            counts, the last file that sets it wins.
        env_map_base: Value of SLAM_TOOLBOX_MAP_BASE, or None when unset; takes precedence when non-empty.

    Returns:
        Path: The map base, DEFAULT_MAP_BASE when nothing configures it.
    """
    if env_map_base:
        return Path(env_map_base)
    map_base = DEFAULT_MAP_BASE
    for path in param_files:
        doc = yaml.safe_load(Path(path).read_text()) or {}
        for key in NODE_PARAM_KEYS:
            params = (doc.get(key) or {}).get("ros__parameters") or {}
            if params.get("map_file_name"):
                map_base = str(params["map_file_name"])
    return Path(map_base)


def map_resume_parameters(map_base: Path) -> dict:
    """Parameters that make slam_toolbox continue a saved posegraph if one exists, else start a fresh map.

    Args:
        map_base: Map path without extension (slam_toolbox writes <base>.posegraph and <base>.data).

    Returns:
        dict: {"map_file_name": str, "map_start_at_dock": True} when <base>.posegraph exists, else
            {"map_file_name": ""} so a map_file_name from a params file cannot point slam_toolbox at a missing file.
    """
    if not map_base.with_suffix(".posegraph").is_file():
        return {"map_file_name": ""}
    return {"map_file_name": str(map_base), "map_start_at_dock": True}


def generate_launch_description() -> LaunchDescription:
    """Build the slam_toolbox launch description.

    Returns:
        LaunchDescription: The async_slam_toolbox_node plus configure and activate transitions.
    """
    files = params_files(
        Path(os.environ.get(PARAMS_ENV, DEFAULT_PARAMS_PATH)),
        Path(os.environ.get(CONFIG_ENV, DEFAULT_CONFIG_PATH)),
    )
    map_base = configured_map_base(files, os.environ.get(MAP_BASE_ENV))
    resume = map_resume_parameters(map_base)
    status = (
        f"resuming saved posegraph {map_base}"
        if resume["map_file_name"]
        else f"no posegraph at {map_base}, starting a fresh map"
    )

    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[*files, {"use_lifecycle_manager": False, "use_sim_time": False, **resume}],
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
            LogInfo(msg=f"slam_toolbox: params {files}; {status}"),
            slam_node,
            activate_after_configure,
            configure,
        ]
    )
